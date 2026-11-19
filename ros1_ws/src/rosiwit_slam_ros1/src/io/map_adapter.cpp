#include "map_adapter.h"

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>

#include "algorithms/map_store/voxel_map_store.h"
#include "xzmap_reader.h"
#include "xzmap_writer.h"

namespace rosiwit_slam_ros1 {
namespace map_adapter {
namespace {

std::string mapBase(const std::string& path) {
    if (path.empty()) return path;
    std::string p = path;
    if (p.size() >= 6 && p.compare(p.size() - 6, 6, ".xzmap") == 0) {
        p.resize(p.size() - 6);
    }
    if (p.back() == '/') p += "map";
    return p;
}

/// 地图文件读写互斥锁: 同一个进程内 load/save 串行化。
/// 用 timed_mutex 是因为存图在服务线程上, 不能无限等待 25s 的载图。
// 递归锁: 外层 MapIoGuard 包住整段写盘, 内层 load/save 还会再锁一次
std::recursive_timed_mutex& mapIoMutex() {
    static std::recursive_timed_mutex mu;
    return mu;
}

}  // namespace

/// Load a plain PCD as a single-keyframe map (same semantics as the ROS2
/// /load_map service). This lets ROS1 consume the maps shipped in docs/maps.
bool loadPcdMapData(const std::string& pcd_path, const std::string& base,
                    rosiwit_slam::MapData& out) {
    rosiwit_slam::CloudType::Ptr cloud(new rosiwit_slam::CloudType());
    if (pcl::io::loadPCDFile(pcd_path, *cloud) != 0 || cloud->empty()) {
        std::cerr << "[map_adapter] cannot read PCD " << pcd_path << std::endl;
        return false;
    }

    out = rosiwit_slam::MapData{};
    out.meta.map_id = base;
    out.meta.frame_id = "map";
    out.meta.resolution = 0.05;

    rosiwit_slam::KeyFrame kf;
    kf.pose.time = -1.0;
    kf.pose.trans = Eigen::Vector3d::Zero();
    kf.pose.rot = Eigen::Matrix3d::Identity();
    kf.cloud = cloud;
    out.keyframes.push_back(kf);

    auto store = std::make_shared<rosiwit_slam::VoxelMapStore>(0.3);
    store->buildFromCloud(cloud);
    out.points = store;

    std::cout << "[map_adapter] loaded PCD " << pcd_path << " ("
              << cloud->size() << " pts) as 1 keyframe" << std::endl;
    return true;
}

bool loadMapData(const std::string& path, rosiwit_slam::MapData& out) {
    std::lock_guard<std::recursive_timed_mutex> io_lk(mapIoMutex());
    const std::string base = mapBase(path);

    // Direct PCD (e.g. /load_map with docs/maps/map/map.pcd).
    if (path.size() >= 4 && path.compare(path.size() - 4, 4, ".pcd") == 0) {
        return loadPcdMapData(path, base, out);
    }

    std::vector<rosiwit_slam::io::XzMapKeyframe> kfs;
    if (!rosiwit_slam::io::readXzMap(base + ".xzmap", kfs) || kfs.empty()) {
        // /switch_map passes a basename: accept <base>.pcd when no xzmap exists.
        std::ifstream pcd_probe(base + ".pcd");
        if (pcd_probe.good()) {
            return loadPcdMapData(base + ".pcd", base, out);
        }
        std::cerr << "[map_adapter] cannot read " << base << ".xzmap" << std::endl;
        return false;
    }

    out = rosiwit_slam::MapData{};
    out.meta.map_id = base;
    out.meta.frame_id = "map";
    out.meta.resolution = 0.05;

    rosiwit_slam::CloudType::Ptr global(new rosiwit_slam::CloudType());
    out.keyframes.reserve(kfs.size());
    for (const auto& src : kfs) {
        rosiwit_slam::PoseStamped pose;
        pose.trans = Eigen::Vector3d(src.pose.x, src.pose.y, src.pose.z);
        pose.rot = (Eigen::AngleAxisd(src.pose.yaw, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(src.pose.pitch, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(src.pose.roll, Eigen::Vector3d::UnitX()))
                       .toRotationMatrix();
        pose.time = -1.0;

        rosiwit_slam::KeyFrame kf;
        kf.pose = pose;
        kf.cloud.reset(new rosiwit_slam::CloudType());
        kf.cloud->reserve(src.points.size());
        for (const auto& p : src.points) {
            rosiwit_slam::PointType q;
            q.x = p.x(); q.y = p.y(); q.z = p.z();
            kf.cloud->push_back(q);
        }
        out.keyframes.push_back(kf);

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = pose.rot.cast<float>();
        T.block<3,1>(0,3) = pose.trans.cast<float>();
        rosiwit_slam::CloudType::Ptr world(new rosiwit_slam::CloudType());
        pcl::transformPointCloud(*kf.cloud, *world, T);
        *global += *world;
    }
    auto store = std::make_shared<rosiwit_slam::VoxelMapStore>(0.3);
    store->buildFromCloud(global);
    out.points = store;
    std::cout << "[map_adapter] loaded " << out.keyframes.size() << " kfs, "
              << global->size() << " pts from " << base << ".xzmap" << std::endl;
    return true;
}

bool saveMapData(const rosiwit_slam::MapData& data, const std::string& path, int wait_ms) {
    // 载图正在跑时不要动同一个目录; 等不到就报忙 (上层转 MAP_SAVE_BUSY)
    std::unique_lock<std::recursive_timed_mutex> io_lk(mapIoMutex(), std::defer_lock);
    if (!io_lk.try_lock_for(std::chrono::milliseconds(wait_ms))) {
        std::cerr << "[map_adapter] saveMapData: map io busy (load in progress), giving up: "
                  << path << std::endl;
        return false;
    }
    const std::string base = mapBase(path);

    // ---- 先把所有关键帧点转到世界系，统计地面高度（设备端约定：地面 z=0）----
    std::vector<rosiwit_slam::CloudType::Ptr> worlds;
    worlds.reserve(data.keyframes.size());
    double zmin = 1e9, zmax = -1e9;
    for (const auto& kf : data.keyframes) {
        if (!kf.cloud || kf.cloud->empty()) { worlds.emplace_back(nullptr); continue; }
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3, 3>(0, 0) = kf.pose.rot.cast<float>();
        T.block<3, 1>(0, 3) = kf.pose.trans.cast<float>();
        rosiwit_slam::CloudType::Ptr w(new rosiwit_slam::CloudType());
        pcl::transformPointCloud(*kf.cloud, *w, T);
        for (const auto& q : w->points) {
            if (!std::isfinite(q.z)) continue;
            zmin = std::min(zmin, (double)q.z);
            zmax = std::max(zmax, (double)q.z);
        }
        worlds.push_back(w);
    }
    // 地面 z 锚点: 地图原点(第一个关键帧)附近 R 内点的 2% 分位 -> 平移到 0。
    // 不能用"全局最低强峰": 图有 z 漂移时会被漂移区拉走, 实测把整图抬高 3.9 m,
    // 重定位 fitness 永远到不了 0.89 => 一直"定位丢失"。
    double floor_z = 0.0;
    {
        constexpr double kGroundRadius = 12.0;
        const size_t kMinPts = 200;
        double gx = 0.0, gy = 0.0;
        bool has_origin = false;
        for (const auto& kf : data.keyframes) {
            if (!kf.cloud || kf.cloud->empty()) continue;
            gx = kf.pose.trans.x();
            gy = kf.pose.trans.y();
            has_origin = true;
            break;
        }
        std::vector<float> zs;
        if (has_origin) {
            for (const auto& w : worlds) {
                if (!w) continue;
                for (const auto& q : w->points) {
                    if (!std::isfinite(q.z)) continue;
                    const double dx = (double)q.x - gx, dy = (double)q.y - gy;
                    if (dx * dx + dy * dy <= kGroundRadius * kGroundRadius) zs.push_back(q.z);
                }
            }
        }
        if (zs.size() < kMinPts) {          // 兜底: 全图 2% 分位
            zs.clear();
            for (const auto& w : worlds) {
                if (!w) continue;
                for (const auto& q : w->points)
                    if (std::isfinite(q.z)) zs.push_back(q.z);
            }
        }
        if (!zs.empty()) {
            const size_t i = std::min(zs.size() - 1,
                                      (size_t)(0.02 * (double)(zs.size() - 1)));
            std::nth_element(zs.begin(), zs.begin() + i, zs.end());
            floor_z = (double)zs[i];
        }
    }

    std::vector<rosiwit_slam::io::XzMapKeyframe> kfs;
    kfs.reserve(data.keyframes.size());
    for (size_t idx = 0; idx < data.keyframes.size(); ++idx) {
        const auto& kf = data.keyframes[idx];
        if (!kf.cloud || kf.cloud->empty() || !worlds[idx]) continue;
        const double yaw = std::atan2(kf.pose.rot(1, 0), kf.pose.rot(0, 0));
        const double cy = std::cos(yaw), sy = std::sin(yaw);
        rosiwit_slam::io::XzMapKeyframe xk;
        // 位姿扁平化：z=0, roll=pitch=0，只保留 yaw（mapping_conf: fix_z/fix_rotation）
        xk.pose.x = kf.pose.trans.x();
        xk.pose.y = kf.pose.trans.y();
        xk.pose.z = 0.0;
        xk.pose.roll = 0.0;
        xk.pose.pitch = 0.0;
        xk.pose.yaw = yaw;
        xk.points.reserve(worlds[idx]->size());
        for (const auto& q : worlds[idx]->points) {
            if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) continue;
            const double dx = (double)q.x - xk.pose.x;
            const double dy = (double)q.y - xk.pose.y;
            // 局部坐标（只按 yaw 旋转），z 以地面为 0
            xk.points.emplace_back((float)(cy * dx + sy * dy),
                                   (float)(-sy * dx + cy * dy),
                                   (float)((double)q.z - floor_z));
        }
        kfs.push_back(std::move(xk));
    }
    if (kfs.empty()) {
        std::cerr << "[map_adapter] saveMapData: no keyframes" << std::endl;
        return false;
    }
    const std::string map_id = data.meta.map_id.empty() ? "map" : data.meta.map_id;
    if (!rosiwit_slam::io::writeXzMap(base + ".xzmap", map_id, kfs)) {
        std::cerr << "[map_adapter] write failed: " << base << ".xzmap" << std::endl;
        return false;
    }
    // 同时落一份通用 PCD，方便离线检查/可视化；定位读取仍只使用 xzmap。
    if (data.points) {
        rosiwit_slam::CloudType::Ptr cloud = data.points->exportCloud();
        if (cloud && !cloud->empty()) {
            const std::string pcd_path = base + ".pcd";
            if (pcl::io::savePCDFileBinary(pcd_path, *cloud) == 0) {
                std::cout << "[map_adapter] saved PCD: " << pcd_path
                          << " (" << cloud->size() << " pts)" << std::endl;
            } else {
                std::cerr << "[map_adapter] PCD write failed: " << pcd_path << std::endl;
            }
        }
    }
    std::cout << "[map_adapter] saved " << kfs.size() << " kfs -> "
              << base << ".xzmap" << std::endl;
    return true;
}

MapIoGuard::MapIoGuard(int wait_ms) {
    locked_ = mapIoMutex().try_lock_for(std::chrono::milliseconds(wait_ms));
    if (!locked_) {
        std::cerr << "[map_adapter] map io busy (load/save in progress), gave up after "
                  << wait_ms << "ms" << std::endl;
    }
}

MapIoGuard::~MapIoGuard() {
    if (locked_) mapIoMutex().unlock();
}

}  // namespace map_adapter
}  // namespace rosiwit_slam_ros1
