// tools/grid_map_tool.cpp
// 离线工具: 读取一个全局 PCD 点云, 用所有已注册的 IGridMapGenerator 策略
// 各生成一张 2D 栅格 PGM/YAML, 便于并排对比各算法效果.
//
// 用法:
//   grid_map_tool <map.pcd> <out_dir> [resolution] [选项]
//   选项:
//     --sensor x y z        射线类策略的传感器原点 (默认 0 0 0)
//     --min-height h        地面以上障碍最低高度 (默认 0.1)
//     --max-height h        地面以上障碍最高高度 (默认 2.0)
//     --dilate r            障碍膨胀半径(格), -1=自动 (默认 -1)
//     --point-spacing s     点间距(m), 用于自动膨胀 (默认 0.1)
//     --max-range r         射线有效距离(m), 0=不限 (默认 0)
//
// 注意: PCD 只有合并后的全局点云, 无逐帧原点. 射线投射/OctoMap 策略
//       将以 --sensor 指定的单一原点处理整张点云(退化用法), 结果仅供对照.
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include "algorithms/grid_map/grid_map_factory.h"
#include "algorithms/grid_map/grid_map_io.h"

using namespace rosiwit_slam;

namespace {
int occupiedCount(const GridInfo& g, int v) {
    int n = 0;
    for (auto x : g.data) if (x == v) ++n;
    return n;
}
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: grid_map_tool <map.pcd> <out_dir> [resolution] [options]\n";
        return 1;
    }
    const std::string pcd_path = argv[1];
    const std::string out_dir = argv[2];
    double resolution = (argc >= 4 && argv[3][0] != '-') ? std::atof(argv[3]) : 0.05;

    double sx = 0, sy = 0, sz = 0;
    GridMapConfig cfg;
    cfg.resolution = resolution;
    cfg.min_height = 0.1;
    cfg.max_height = 2.0;
    cfg.occupied_thresh = 1;
    cfg.dilate_radius = -1;
    cfg.point_spacing = 0.1;
    cfg.max_range = 0.0;

    for (int i = 4; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](double& out) { if (i + 1 < argc) out = std::atof(argv[++i]); };
        if (a == "--sensor" && i + 3 < argc) {
            sx = std::atof(argv[++i]); sy = std::atof(argv[++i]); sz = std::atof(argv[++i]);
        } else if (a == "--min-height") next(cfg.min_height);
        else if (a == "--max-height") next(cfg.max_height);
        else if (a == "--point-spacing") next(cfg.point_spacing);
        else if (a == "--max-range") next(cfg.max_range);
        else if (a == "--dilate" && i + 1 < argc) cfg.dilate_radius = std::atoi(argv[++i]);
    }

    CloudType::Ptr cloud(new CloudType());
    if (pcl::io::loadPCDFile(pcd_path, *cloud) == -1) {
        std::cerr << "[grid_map_tool] load failed: " << pcd_path << std::endl;
        return 2;
    }
    std::cout << "[grid_map_tool] loaded " << cloud->size() << " pts from " << pcd_path
              << ", res=" << cfg.resolution
              << ", sensor=(" << sx << "," << sy << "," << sz << ")" << std::endl;

    const Eigen::Vector3d origin(sx, sy, sz);
    int generated = 0;
    for (const auto& name : GridMapFactory::listNames()) {
        auto gen = GridMapFactory::create(name);
        if (!gen) continue;
        GridInfo info;
        bool ok;
        if (gen->needsObservations()) {
            std::vector<Observation> obs;
            Observation o;
            o.origin = origin;
            o.cloud = cloud;
            obs.push_back(o);
            ok = gen->buildFromObservations(obs, origin, cfg, info);
        } else {
            ok = gen->build(cloud, origin, cfg, info);
        }
        if (!ok) {
            std::cerr << "[grid_map_tool] strategy '" << name << "' failed" << std::endl;
            continue;
        }
        const std::string pgm = out_dir + "/" + name + ".pgm";
        const std::string yml = out_dir + "/" + name + ".yaml";
        if (!saveGridMapPGM(info, pgm, yml, "map")) {
            std::cerr << "[grid_map_tool] save failed: " << pgm << std::endl;
            continue;
        }
        const int total = info.width * info.height;
        std::cout << "[grid_map_tool] " << name << ": " << info.width << "x" << info.height
                  << " occ=" << occupiedCount(info, 100)
                  << " free=" << occupiedCount(info, 0)
                  << " unknown=" << occupiedCount(info, -1)
                  << " (" << total << ")\n";
        ++generated;
    }
    std::cout << "[grid_map_tool] generated " << generated << " maps -> " << out_dir << std::endl;
    return generated > 0 ? 0 : 3;
}
