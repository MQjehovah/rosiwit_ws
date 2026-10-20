#include "map_adapter.h"

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <fstream>
#include <iostream>

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

}  // namespace

bool loadMapData(const std::string& path, rosiwit_slam::MapData& out) {
    const std::string base = mapBase(path);
    std::vector<rosiwit_slam::io::XzMapKeyframe> kfs;
    if (!rosiwit_slam::io::readXzMap(base + ".xzmap", kfs) || kfs.empty()) {
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

bool saveMapData(const rosiwit_slam::MapData& data, const std::string& path) {
    const std::string base = mapBase(path);
    std::vector<rosiwit_slam::io::XzMapKeyframe> kfs;
    kfs.reserve(data.keyframes.size());
    for (const auto& kf : data.keyframes) {
        if (!kf.cloud || kf.cloud->empty()) continue;
        rosiwit_slam::io::XzMapKeyframe xk;
        xk.pose.x = kf.pose.trans.x();
        xk.pose.y = kf.pose.trans.y();
        xk.pose.z = kf.pose.trans.z();
        const Eigen::Vector3d rpy =
            kf.pose.rot.eulerAngles(2, 1, 0);   // yaw, pitch, roll (ZYX)
        xk.pose.yaw = rpy.x();
        xk.pose.pitch = rpy.y();
        xk.pose.roll = rpy.z();
        xk.points.reserve(kf.cloud->size());
        for (const auto& p : kf.cloud->points) {
            xk.points.emplace_back(p.x, p.y, p.z);
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

}  // namespace map_adapter
}  // namespace rosiwit_slam_ros1
