// tools/offline_map_export.cpp
// 离线重导出: 读取 dumpMapData 产出的全量数据 (逐帧观测 + 全局点云 + traj),
// 用与在线完全相同的逐帧地面/栅格生成器 + xzmap 导出器重新生成 map.pcd/pgm/xzmap.
// 建图只跑一次, 之后改分辨率/后处理都不用重跑 SLAM.
//
// 用法:
//   offline_map_export <dump_dir> <out_dir> [options]
//     --res 0.05  --kf-step 1.0  --generator ray_casting
//     --post denoise,close,dilate  --close 1  --dilate 1  --min-comp 30
//     --min-height 0.1  --max-height 2.0  --ground ransac
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>

#include "algorithms/grid_map/grid_map_factory.h"
#include "algorithms/grid_map/grid_map_io.h"
#include "algorithms/grid_map/grid_map_postprocess.h"
#include "io/robot_map_export.h"
#include "slam_core/i_grid_map_generator.h"
#include "slam_core/slam_types.h"

using namespace rosiwit_slam;

static bool loadCloud(const std::string& path, CloudType::Ptr& out) {
    out.reset(new CloudType());
    return pcl::io::loadPCDFile<PointType>(path, *out) == 0;
}

static std::vector<double> splitCsv(const std::string& s) {
    std::vector<double> v;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) v.push_back(std::atof(tok.c_str()));
    return v;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: offline_map_export <dump_dir> <out_dir> [--res 0.05] [--kf-step 1.0]\n"
                     "  [--generator ray_casting] [--post denoise,close,dilate]\n"
                     "  [--close 1] [--dilate 1] [--min-comp 30]\n"
                     "  [--min-height 0.1] [--max-height 2.0] [--ground ransac]\n";
        return 1;
    }
    const std::string dump = argv[1];
    const std::string out = argv[2];
    double res = 0.05, kf_step = 1.0, min_h = 0.1, max_h = 2.0;
    std::string generator = "ray_casting";
    std::string post_str = "denoise,close,dilate";
    std::string ground = "ransac";
    int close_r = 1, dilate_r = 1, min_comp = 30;

    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](double& d) { if (i + 1 < argc) d = std::atof(argv[++i]); };
        auto nexti = [&](int& d) { if (i + 1 < argc) d = std::atoi(argv[++i]); };
        if (a == "--res") next(res);
        else if (a == "--kf-step") next(kf_step);
        else if (a == "--min-height") next(min_h);
        else if (a == "--max-height") next(max_h);
        else if (a == "--generator" && i + 1 < argc) generator = argv[++i];
        else if (a == "--post" && i + 1 < argc) post_str = argv[++i];
        else if (a == "--close") nexti(close_r);
        else if (a == "--dilate") nexti(dilate_r);
        else if (a == "--min-comp") nexti(min_comp);
        else if (a == "--ground" && i + 1 < argc) ground = argv[++i];
    }

    std::filesystem::create_directories(out);

    CloudType::Ptr global;
    if (!loadCloud(dump + "/global_map.pcd", global)) {
        std::cerr << "[offline] load global_map.pcd failed\n";
        return 2;
    }
    std::cout << "[offline] global map " << global->size() << " pts\n";

    std::vector<Observation> observations;
    {
        std::ifstream idx(dump + "/observations.txt");
        std::string line;
        while (std::getline(idx, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream iss(line);
            size_t i = 0, n = 0;
            double ox = 0, oy = 0, oz = 0;
            std::string file;
            if (!(iss >> i >> ox >> oy >> oz >> n >> file)) continue;
            CloudType::Ptr c(new CloudType());
            if (pcl::io::loadPCDFile<PointType>(dump + "/" + file, *c) != 0) continue;
            Observation ob;
            ob.origin = Eigen::Vector3d(ox, oy, oz);
            ob.cloud = c;
            observations.push_back(ob);
        }
    }
    std::cout << "[offline] observations " << observations.size() << " frames\n";

    std::vector<io::RobotKfSample> track;
    {
        std::ifstream tf(dump + "/traj.csv");
        std::string line;
        std::getline(tf, line);   // header
        while (std::getline(tf, line)) {
            auto v = splitCsv(line);
            if (v.size() < 4) continue;
            io::RobotKfSample s;
            s.x = v[0]; s.y = v[1]; s.z = v[2]; s.yaw = v[3];
            track.push_back(s);
        }
    }
    std::cout << "[offline] track " << track.size() << " samples\n";

    auto gen = GridMapFactory::create(generator);
    if (!gen) {
        std::cerr << "[offline] unknown generator: " << generator << "\n";
        return 3;
    }

    GridMapConfig cfg;
    cfg.resolution = res;
    cfg.min_height = min_h;
    cfg.max_height = max_h;
    cfg.ground_method = ground;
    cfg.ransac_dist_thr = 0.10;

    GridInfo info;
    const bool ok = !observations.empty()
        ? gen->buildFromObservations(observations, Eigen::Vector3d::Zero(), cfg, info)
        : gen->build(global, Eigen::Vector3d::Zero(), cfg, info);
    if (!ok) {
        std::cerr << "[offline] grid build failed\n";
        return 4;
    }

    GridPostConfig post;
    post.pipeline = parsePostPipeline(post_str);
    post.min_component_cells = min_comp;
    post.close_radius = close_r;
    post.dilate_radius = dilate_r;
    post.resolution = info.resolution;
    post.point_spacing = 0.3;
    applyGridPostProcess(info, post);

    if (!saveGridMapPGM(info, out + "/map.pgm", out + "/map.yaml", "map")) {
        std::cerr << "[offline] save pgm failed\n";
        return 5;
    }
    pcl::io::savePCDFileBinary(out + "/map.pcd", *global);

    std::vector<Eigen::Vector3f> world;
    world.reserve(global->size());
    for (const auto& p : global->points)
        world.emplace_back(p.x, p.y, p.z);
    if (!io::exportRobotXzMap(out + "/map.xzmap", "map", track, world, kf_step)) {
        std::cerr << "[offline] xzmap export failed\n";
        return 6;
    }

    std::cout << "[offline] done -> " << out << "  pgm=" << info.width << "x" << info.height
              << " res=" << info.resolution << "  pts=" << global->size()
              << "  kf=" << track.size() << std::endl;
    return 0;
}
