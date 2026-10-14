// tools/localization_eval.cpp
// 离线配准器横向评估工具 (对应建图侧 grid_map_tool):
//   对同一地图/扫描, 依次用各已注册配准器求解, 输出误差/覆盖率/残差/耗时对比,
//   用于在部署前对"同一阶段的不同算法"做定量性能验证.
//
// 用法:
//   localization_eval <map.pcd> [scan.pcd] [选项]
// 选项:
//   --x --y --z --yaw      真值位姿(合成模式) 或 初始猜测(真实模式)
//   --dx --dy --dyaw       合成模式下对初值的扰动(默认 0.3m / 3°)
//   --voxel <r>            配准降采样体素(m)
//   --matcher <name|all>   只测指定配准器 (默认 all)
//   --edge                 开启点到线边缘特征(pp_icp)
//   --no-degeneracy        关闭退化门控(pp_icp)
// 合成模式(不给 scan): 由真值位姿从地图反投影得到"机体坐标"扫描, 可给出绝对误差.
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "algorithms/localization/localization_factory.h"
#include "algorithms/localization/localization_utils.h"

using namespace rosiwit_slam;

namespace {
PoseStamped makePose(double x, double y, double z, double yaw_deg) {
    PoseStamped p;
    p.trans = V3D(x, y, z);
    p.rot = Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, V3D::UnitZ()).toRotationMatrix();
    return p;
}
} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: localization_eval <map.pcd> [scan.pcd] [--x .. --y .. --yaw .. "
                     "--dx .. --dyaw .. --voxel .. --matcher .. --edge --no-degeneracy]\n";
        return 1;
    }
    const std::string map_path = argv[1];
    std::string scan_path;
    if (argc > 2 && argv[2][0] != '-') scan_path = argv[2];

    double gx = 3.0, gy = 3.0, gz = 0.0, gyaw = 8.0;
    double dx = 0.3, dy = 0.0, dyaw = 3.0;
    double voxel = 0.3;
    std::string only = "all";
    bool edge = false, degeneracy = true;

    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](double& out) { if (i + 1 < argc) out = std::atof(argv[++i]); };
        if (a == "--x") next(gx);
        else if (a == "--y") next(gy);
        else if (a == "--z") next(gz);
        else if (a == "--yaw") next(gyaw);
        else if (a == "--dx") next(dx);
        else if (a == "--dy") next(dy);
        else if (a == "--dyaw") next(dyaw);
        else if (a == "--voxel") next(voxel);
        else if (a == "--matcher" && i + 1 < argc) only = argv[++i];
        else if (a == "--edge") edge = true;
        else if (a == "--no-degeneracy") degeneracy = false;
    }

    CloudType::Ptr map(new CloudType());
    if (pcl::io::loadPCDFile(map_path, *map) == -1) {
        std::cerr << "[eval] failed to load map: " << map_path << "\n";
        return 1;
    }

    const bool synthetic = scan_path.empty();
    PoseStamped gt = makePose(gx, gy, gz, gyaw);
    CloudType::Ptr scan(new CloudType());
    PoseStamped guess;
    if (synthetic) {
        Eigen::Matrix4f inv = poseToMatrix(gt).cast<float>().inverse();
        pcl::transformPointCloud(*map, *scan, inv);
        guess = makePose(gx + dx, gy + dy, gz, gyaw + dyaw);
    } else {
        if (pcl::io::loadPCDFile(scan_path, *scan) == -1) {
            std::cerr << "[eval] failed to load scan: " << scan_path << "\n";
            return 1;
        }
        guess = gt;  // 真实模式: 作为初始猜测
    }

    std::cout << "[eval] map=" << map->size() << " pts, scan=" << scan->size()
              << " pts, mode=" << (synthetic ? "synthetic" : "real") << " voxel=" << voxel << "\n";
    std::cout << std::left << std::setw(12) << "matcher" << std::setw(12) << "trans_err_m"
              << std::setw(12) << "yaw_err_deg" << std::setw(10) << "fitness"
              << std::setw(12) << "residual_m" << std::setw(10) << "ms"
              << std::setw(10) << "degen" << "\n";

    std::vector<std::string> names = LocalizationFactory::listMatchers();
    for (const auto& name : names) {
        if (only != "all" && only != name) continue;
        auto matcher = LocalizationFactory::createMatcher(name);
        if (!matcher) continue;
        ScanMatcherConfig cfg;
        cfg.voxel_resolution = voxel;
        cfg.use_edge_feature = edge;
        cfg.degeneracy_gate = degeneracy;
        matcher->configure(cfg);
        matcher->setTarget(map);

        const auto t0 = std::chrono::steady_clock::now();
        MatchResult res;
        const bool ok = matcher->match(scan, guess, res);
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        double terr = -1.0, yerr = -1.0;
        if (synthetic && ok)
            terr = (res.pose.trans - gt.trans).norm(),
            yerr = std::fabs(yawFromRot(gt.rot.transpose() * res.pose.rot)) * 180.0 / M_PI;

        std::cout << std::left << std::setw(12) << name
                  << std::setw(12) << std::fixed << std::setprecision(4) << terr
                  << std::setw(12) << yerr
                  << std::setw(10) << res.fitness
                  << std::setw(12) << res.residual
                  << std::setw(10) << ms
                  << std::setw(10) << (res.degenerate ? res.degenerate_dirs : 0) << "\n";
    }
    return 0;
}
