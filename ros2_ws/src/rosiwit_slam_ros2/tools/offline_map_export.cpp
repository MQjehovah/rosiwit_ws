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
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

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

// 地面归一化: 5m 分块取 z 的 2% 分位作为局部地面, 把所有点和轨迹减去它.
// 这样地面在 z=0, 墙/台阶保留相对高度; 不依赖 SLAM 的 z 约束.
static void normalizeGround(CloudType::Ptr& cloud, std::vector<io::RobotKfSample>& track) {
    constexpr double B = 5.0;
    auto key = [](int bx, int by) -> long long {
        return (static_cast<long long>(bx) << 32) ^ static_cast<unsigned int>(by);
    };
    std::unordered_map<long long, std::vector<float>> zs;
    zs.reserve(cloud->size() / 8 + 1);
    for (const auto& p : cloud->points) {
        const int bx = static_cast<int>(std::floor(p.x / B));
        const int by = static_cast<int>(std::floor(p.y / B));
        zs[key(bx, by)].push_back(p.z);
    }
    std::unordered_map<long long, float> ground;
    ground.reserve(zs.size());
    auto zmin0 = cloud->points.empty() ? 0.0f : cloud->points[0].z;
    auto zmax0 = zmin0;
    for (const auto& p : cloud->points) {
        zmin0 = std::min(zmin0, p.z);
        zmax0 = std::max(zmax0, p.z);
    }
    for (auto& kv : zs) {
        auto& v = kv.second;
        std::sort(v.begin(), v.end());
        ground[kv.first] = v[std::min(v.size() - 1, static_cast<size_t>(0.02 * v.size()))];
    }
    auto getg = [&](double x, double y) -> float {
        const int bx = static_cast<int>(std::floor(x / B));
        const int by = static_cast<int>(std::floor(y / B));
        for (int r = 0; r <= 8; ++r) {
            for (int dx = -r; dx <= r; ++dx) {
                for (int dy = -r; dy <= r; ++dy) {
                    if (r > 0 && std::abs(dx) != r && std::abs(dy) != r) continue;
                    auto it = ground.find(key(bx + dx, by + dy));
                    if (it != ground.end()) return it->second;
                }
            }
        }
        return 0.0f;
    };
    for (auto& p : cloud->points) p.z -= getg(p.x, p.y);
    for (auto& s : track) s.z -= getg(s.x, s.y);
    float zmin1 = cloud->points.empty() ? 0.0f : cloud->points[0].z;
    float zmax1 = zmin1;
    for (const auto& p : cloud->points) {
        zmin1 = std::min(zmin1, p.z);
        zmax1 = std::max(zmax1, p.z);
    }
    std::cout << "[offline] ground-zero: z [" << zmin0 << ", " << zmax0
              << "] -> [" << zmin1 << ", " << zmax1 << "]  blocks=" << ground.size() << "\n";
}

// z-only 后端位姿图:
//   节点 = 每个关键帧的 z; 边 = 相邻 z 增量 (原 LIO 轨迹) + 每帧地面 unary
//   (z_k = h_k, h_k 由该帧点云检测的地面高度得到). 只改 z, xy/yaw 不动.
//   这是手持建图正确的"地面 z=0"做法, 不在前端硬锁滤波器.
static void poseGraphGround(const std::vector<Observation>& obs, CloudType::Ptr& cloud,
                            std::vector<io::RobotKfSample>& track) {
    const size_t N = obs.size();
    if (N < 3) return;
    std::vector<double> zold(N, 0.0), h(N, 0.0);
    std::vector<char> valid(N, 0);
    // 先用全局点云的 5m 分块 2% 分位估计局部地面 (比单帧圆锥更稳),
    // 再用 z-only 位姿图把每帧传感器高度 z_k 平滑地约束到这个地面.
    constexpr double B = 5.0;
    auto bkey = [](int bx, int by) -> long long {
        return (static_cast<long long>(bx) << 32) ^ static_cast<unsigned int>(by);
    };
    std::unordered_map<long long, std::vector<float>> zsmap;
    zsmap.reserve(cloud->size() / 8 + 1);
    for (const auto& p : cloud->points) {
        const int bx = static_cast<int>(std::floor(p.x / B));
        const int by = static_cast<int>(std::floor(p.y / B));
        zsmap[bkey(bx, by)].push_back(p.z);
    }
    std::unordered_map<long long, float> bground;
    bground.reserve(zsmap.size());
    for (auto& kv : zsmap) {
        auto& v = kv.second;
        std::sort(v.begin(), v.end());
        bground[kv.first] = v[std::min(v.size() - 1, static_cast<size_t>(0.02 * v.size()))];
    }
    auto getg = [&](double x, double y) -> float {
        const int bx = static_cast<int>(std::floor(x / B));
        const int by = static_cast<int>(std::floor(y / B));
        for (int rr = 0; rr <= 8; ++rr) {
            for (int dx = -rr; dx <= rr; ++dx) {
                for (int dy = -rr; dy <= rr; ++dy) {
                    if (rr > 0 && std::abs(dx) != rr && std::abs(dy) != rr) continue;
                    auto it = bground.find(bkey(bx + dx, by + dy));
                    if (it != bground.end()) return it->second;
                }
            }
        }
        return 0.0f;
    };
    for (size_t i = 0; i < N; ++i) {
        zold[i] = obs[i].origin.z();
        const double hgt = obs[i].origin.z() - getg(obs[i].origin.x(), obs[i].origin.y());
        if (hgt < 0.15 || hgt > 2.0) continue;
        h[i] = hgt;
        valid[i] = 1;
    }
    size_t nvalid = 0;
    for (char v : valid) nvalid += v;
    if (nvalid < 10) {
        std::cout << "[offline] posegraph: too few ground frames (" << nvalid << ")\n";
        return;
    }
    const double wu = 1.0 / 0.05;   // 地面 unary (m)
    const double wo = 1.0 / 0.10;   // 相邻 z 增量 (m)
    const size_t rows = nvalid + (N - 1);
    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(nvalid * 1 + (N - 1) * 2);
    Eigen::VectorXd b(rows);
    b.setZero();
    size_t r = 0;
    for (size_t i = 0; i < N; ++i) {
        if (!valid[i]) continue;
        trips.emplace_back(static_cast<int>(r), static_cast<int>(i), wu);
        b[r] = wu * h[i];
        ++r;
    }
    for (size_t i = 0; i + 1 < N; ++i) {
        trips.emplace_back(static_cast<int>(r), static_cast<int>(i), -wo);
        trips.emplace_back(static_cast<int>(r), static_cast<int>(i + 1), wo);
        b[r] = wo * (zold[i + 1] - zold[i]);
        ++r;
    }
    Eigen::SparseMatrix<double> A(static_cast<int>(rows), static_cast<int>(N));
    A.setFromTriplets(trips.begin(), trips.end());
    Eigen::SparseMatrix<double> ATA = A.transpose() * A;
    Eigen::VectorXd ATb = A.transpose() * b;
    for (size_t i = 0; i < N; ++i) ATA.coeffRef(static_cast<int>(i), static_cast<int>(i)) += 1e-6;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(ATA);
    if (solver.info() != Eigen::Success) {
        std::cout << "[offline] posegraph solve failed\n";
        return;
    }
    Eigen::VectorXd znew = solver.solve(ATb);
    std::vector<float> delta(N, 0.0f);
    for (size_t i = 0; i < N; ++i) delta[i] = static_cast<float>(znew[i] - zold[i]);

    // 每个地图点/轨迹点按最近关键帧 origin 的 delta 平移 z.
    pcl::PointCloud<pcl::PointXYZ>::Ptr origins(new pcl::PointCloud<pcl::PointXYZ>());
    origins->reserve(N);
    for (const auto& o : obs)
        origins->push_back(pcl::PointXYZ(static_cast<float>(o.origin.x()),
                                         static_cast<float>(o.origin.y()), 0.0f));
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(origins);
    std::vector<int> idx(1);
    std::vector<float> d2(1);
    float zmin0 = cloud->points.empty() ? 0.0f : cloud->points[0].z;
    float zmax0 = zmin0;
    for (const auto& p : cloud->points) {
        zmin0 = std::min(zmin0, p.z);
        zmax0 = std::max(zmax0, p.z);
    }
    for (auto& p : cloud->points) {
        pcl::PointXYZ q(p.x, p.y, 0.0f);
        if (kdtree.nearestKSearch(q, 1, idx, d2) > 0) p.z += delta[idx[0]];
    }
    for (auto& s : track) {
        pcl::PointXYZ q(static_cast<float>(s.x), static_cast<float>(s.y), 0.0f);
        if (kdtree.nearestKSearch(q, 1, idx, d2) > 0) s.z += delta[idx[0]];
    }
    float zmin1 = cloud->points.empty() ? 0.0f : cloud->points[0].z;
    float zmax1 = zmin1;
    for (const auto& p : cloud->points) {
        zmin1 = std::min(zmin1, p.z);
        zmax1 = std::max(zmax1, p.z);
    }
    std::cout << "[offline] posegraph: " << nvalid << "/" << N << " ground frames; z ["
              << zmin0 << ", " << zmax0 << "] -> [" << zmin1 << ", " << zmax1 << "]\n";
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
    bool ground_zero = false;
    bool ground_posegraph = false;

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
        else if (a == "--ground-zero") ground_zero = true;
        else if (a == "--ground-posegraph") ground_posegraph = true;
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

    if (ground_posegraph) poseGraphGround(observations, global, track);
    else if (ground_zero) normalizeGround(global, track);

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
