// tools/map_cleanup.cpp
// 离线静态底图清理 (Removert/ERASOR 风): 用已配准的关键帧(xzmap)做可见性体素投票,
// 移除"被多次观测为自由/少数帧占据"的动态点, 产出干净定位底图.
//
// 用法:
//   map_cleanup <in.pcd> <out.pcd> [--xzmap <map.pcd.xzmap>] [--voxel 0.3]
//               [--free-ratio 2.0] [--stride 10]
//
// 原理:
//   1. 对每关键帧: 机体系点云转到世界系, 命中体素 occ++ (并记录代表点)
//   2. 从传感器原点到每个命中点做射线行进, 途经体素 free++ (排除端点附近)
//   3. 体素 free > free_ratio*occ 判为动态 -> 该体素内的输入点从底图移除
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <string>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <pcl/io/pcd_io.h>
#include "slam_core/slam_types.h"
#include "io/xzmap_reader.h"

using namespace rosiwit_slam;

namespace {
inline int64_t vkey(double x, double y, double z, double v) {
    const int64_t ix = static_cast<int64_t>(std::floor(x / v)) & 0x1FFFFF;
    const int64_t iy = static_cast<int64_t>(std::floor(y / v)) & 0x1FFFFF;
    const int64_t iz = static_cast<int64_t>(std::floor(z / v)) & 0x1FFFFF;
    return (ix << 42) | (iy << 21) | iz;
}
} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: map_cleanup <in.pcd> <out.pcd> [--xzmap path] "
                     "[--voxel 0.3] [--free-ratio 2.0] [--stride 10]\n";
        return 1;
    }
    const std::string in_path = argv[1];
    const std::string out_path = argv[2];
    std::string xzmap;
    double voxel = 0.3, free_ratio = 2.0;
    int stride = 10;
    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--xzmap" && i + 1 < argc) xzmap = argv[++i];
        else if (a == "--voxel" && i + 1 < argc) voxel = std::atof(argv[++i]);
        else if (a == "--free-ratio" && i + 1 < argc) free_ratio = std::atof(argv[++i]);
        else if (a == "--stride" && i + 1 < argc) stride = std::atoi(argv[++i]);
    }

    CloudType::Ptr map(new CloudType());
    if (pcl::io::loadPCDFile(in_path, *map) != 0) {
        std::cerr << "[cleanup] failed to load input: " << in_path << "\n";
        return 1;
    }

    if (xzmap.empty()) xzmap = in_path + ".xzmap";
    std::vector<io::XzMapKeyframe> kfs;
    if (!io::readXzMap(xzmap, kfs) || kfs.empty()) {
        std::cerr << "[cleanup] no keyframes (" << xzmap << "); copy input -> output\n";
        pcl::io::savePCDFileBinary(out_path, *map);
        return 0;
    }

    // 可见性体素投票
    std::unordered_map<int64_t, float> occ;   // 占据次数
    std::unordered_map<int64_t, float> fre;   // 自由(射线穿过)次数
    const double step = std::max(0.05, voxel * 0.5);
    for (const auto& kf : kfs) {
        const Eigen::Vector3d o(kf.pose.x, kf.pose.y, kf.pose.z);
        const Eigen::Matrix3d R =
            (Eigen::AngleAxisd(kf.pose.yaw, V3D::UnitZ()) *
             Eigen::AngleAxisd(kf.pose.pitch, V3D::UnitY()) *
             Eigen::AngleAxisd(kf.pose.roll, V3D::UnitX())).toRotationMatrix();
        const int n = static_cast<int>(kf.points.size());
        for (int i = 0; i < n; i += std::max(1, stride)) {
            const Eigen::Vector3d p(kf.points[i].x(), kf.points[i].y(), kf.points[i].z());
            const Eigen::Vector3d w = R * p + o;
            occ[vkey(w.x(), w.y(), w.z(), voxel)] += 1.0f;
            // 射线行进 (排除端点附近)
            const Eigen::Vector3d d = w - o;
            const double len = d.norm();
            if (len < step * 2) continue;
            const Eigen::Vector3d u = d / len;
            for (double t = step; t < len - voxel; t += step) {
                const Eigen::Vector3d q = o + u * t;
                fre[vkey(q.x(), q.y(), q.z(), voxel)] += 1.0f;
            }
        }
    }

    // 标记动态体素
    std::unordered_map<int64_t, char> dyn;
    for (const auto& kv : fre) {
        auto it = occ.find(kv.first);
        const float o = (it == occ.end()) ? 0.0f : it->second;
        if (kv.second > free_ratio * (o + 1e-3f)) dyn[kv.first] = 1;
    }

    // 过滤输入地图
    CloudType::Ptr out(new CloudType());
    out->reserve(map->size());
    size_t removed = 0;
    for (const auto& p : map->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (dyn.count(vkey(p.x, p.y, p.z, voxel))) { ++removed; continue; }
        out->push_back(p);
    }
    pcl::io::savePCDFileBinary(out_path, *out);
    std::cout << "[cleanup] kfs=" << kfs.size() << " voxels(free)=" << fre.size()
              << " dynamic=" << dyn.size() << " removed=" << removed
              << " kept=" << out->size() << " -> " << out_path << "\n";
    return 0;
}
