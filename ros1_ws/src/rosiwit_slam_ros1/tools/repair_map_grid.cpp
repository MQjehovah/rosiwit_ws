// ============================================================================
// repair_map_grid —— 把旧 xzmap 重新生成为带 unknown(-1) 的 PGM。
//
// 旧 height_projection 地图把所有未观测格写成 free，导致全白；本工具从
// <map_dir>/map.xzmap 的关键帧重建逐帧观测，用 ray_casting 生成：
//   0=free(白), 100=occupied(黑), -1=unknown(灰)
// 输出写 <map_dir>/map.repaired.pgm/.yaml，不覆盖原文件。
// ============================================================================
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "algorithms/grid_map/grid_map_io.h"
#include "algorithms/grid_map/grid_map_postprocess.h"
#include "algorithms/grid_map/ray_casting_grid_map.h"
#include "io/xzmap_reader.h"
#include "slam_core/slam_types.h"

using namespace rosiwit_slam;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: repair_map_grid <map_dir> [resolution]\n";
        return 1;
    }
    const std::string dir = argv[1];
    const std::string base = dir + "/map";
    double resolution = (argc >= 3) ? std::atof(argv[2]) : 0.05;
    if (resolution <= 0.0) resolution = 0.05;

    std::vector<io::XzMapKeyframe> kfs;
    if (!io::readXzMap(base + ".xzmap", kfs) || kfs.empty()) {
        std::cerr << "[repair_map_grid] cannot read " << base << ".xzmap\n";
        return 2;
    }

    std::vector<Observation> obs;
    obs.reserve(kfs.size());
    long long total_pts = 0;
    for (const auto& kf : kfs) {
        if (kf.points.empty()) continue;
        const Eigen::AngleAxisd rz(kf.pose.yaw, Eigen::Vector3d::UnitZ());
        const Eigen::AngleAxisd ry(kf.pose.pitch, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd rx(kf.pose.roll, Eigen::Vector3d::UnitX());
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.translate(Eigen::Vector3d(kf.pose.x, kf.pose.y, kf.pose.z));
        T.rotate(rz * ry * rx);

        Observation ob;
        ob.origin = Eigen::Vector3d(kf.pose.x, kf.pose.y, kf.pose.z);
        ob.cloud.reset(new CloudType());
        ob.cloud->reserve(kf.points.size());
        for (const auto& p : kf.points) {
            if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) continue;
            const Eigen::Vector3d w = T * Eigen::Vector3d(p.x(), p.y(), p.z());
            PointType q;
            q.x = static_cast<float>(w.x());
            q.y = static_cast<float>(w.y());
            q.z = static_cast<float>(w.z());
            ob.cloud->push_back(q);
        }
        total_pts += static_cast<long long>(ob.cloud->size());
        obs.push_back(std::move(ob));
    }
    if (obs.empty()) {
        std::cerr << "[repair_map_grid] no points in xzmap\n";
        return 3;
    }

    GridMapConfig cfg;
    cfg.resolution = resolution;
    cfg.min_height = 0.1;
    cfg.max_height = 2.0;
    cfg.ground_method = "ransac";
    cfg.max_range = 30.0;
    cfg.point_spacing = resolution;

    GridInfo info;
    RayCastingGridMap gen;
    if (!gen.buildFromObservations(obs, Eigen::Vector3d::Zero(), cfg, info)) {
        std::cerr << "[repair_map_grid] ray casting failed\n";
        return 4;
    }

    GridPostConfig post;
    post.resolution = info.resolution;
    post.point_spacing = resolution;
    // 历史地图可能只有 1 个关键帧；按默认 denoise 会把整面墙当小连通域删掉。
    post.min_component_cells = 0;
    applyGridPostProcess(info, post);

    const std::string out_pgm = base + ".repaired.pgm";
    const std::string out_yaml = base + ".repaired.yaml";
    if (!saveGridMapPGM(info, out_pgm, out_yaml, "map")) {
        std::cerr << "[repair_map_grid] save failed\n";
        return 5;
    }
    long long unknown = 0, free = 0, occupied = 0;
    for (int8_t v : info.data) {
        if (v < 0) ++unknown;
        else if (v == 0) ++free;
        else ++occupied;
    }
    std::cout << "[repair_map_grid] kfs=" << obs.size() << " pts=" << total_pts
              << " size=" << info.width << "x" << info.height
              << " free=" << free << " unknown=" << unknown
              << " occupied=" << occupied << "\n"
              << "output: " << out_pgm << " / " << out_yaml << std::endl;
    return 0;
}
