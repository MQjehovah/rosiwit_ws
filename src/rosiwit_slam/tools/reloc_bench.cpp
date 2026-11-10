// tools/reloc_bench.cpp
//
// 全局重定位基准: 用一张 PCD 地图 + 一条轨迹, 生成 N 个"冷启动"场景,
// 逐个跑全局重定位并统计耗时/召回/误锁。
//
// 场景怎么来 (可复现, 不依赖实车):
//   1. 从轨迹里等间隔取 N 个真值位姿 (真值 = 建图时的位姿);
//   2. 在真值位姿处从地图里裁 ±R 的点, 变换到传感器系 => 合成扫描
//      (可加噪声 / 随机去掉一部分点, 模拟遮挡与变化);
//   3. 全局重定位从零开始求解, 结果与真值比:
//        平移误差 < pos_tol 且 航向误差 < yaw_tol  => 命中
//        报了结果但误差 > false_lock_dist            => 误锁
//   4. 输出 p50/p95/max 耗时、召回率、误锁率。
//
// 用法:
//   reloc_bench <map.pcd> <traj.tum> [选项]
//     --localizer bbs|scan_context|fpfh_ransac|all   (默认 bbs)
//     --scenarios N        场景数 (默认 20)
//     --scan-radius R      合成扫描半径 m (默认 25)
//     --voxel V            扫描降采样 m (默认 0.3)
//     --pos-tol M          命中平移容差 m (默认 0.5)
//     --yaw-tol DEG        命中航向容差 deg (默认 5)
//     --false-lock M       误锁门限 m (默认 2.0)
//     --noise M            点噪声 sigma m (默认 0.02)
//     --drop R             随机丢点比例 (默认 0.2)
//     --topk K             多候选数 (默认 5, 与管线一致)
//     --submap M           精配局部子图半径 m (0 = 全图, 默认 30)
//     --build-sc-db K      先按地图生成 ScanContext 库(K 个关键帧), 供 scan_context 用
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "algorithms/localization/localization_factory.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/scan_context/scan_context.h"

using namespace rosiwit_slam;

namespace {

struct Args {
    std::string map_path;
    std::string traj_path;
    std::string localizer = "bbs";
    int scenarios = 20;
    double scan_radius = 25.0;
    double voxel = 0.3;
    double pos_tol = 0.5;
    double yaw_tol_deg = 5.0;
    double false_lock = 2.0;
    double noise = 0.02;
    double drop = 0.2;
    int topk = 5;
    double submap = 30.0;
    int build_sc_db = 0;
    unsigned seed = 7;
};

struct Scenario {
    PoseStamped gt;
    CloudType::Ptr scan;
};

struct Result {
    double ms = 0.0;
    double err = 0.0;
    double yaw_err_deg = 0.0;
    bool reported = false;
    bool hit = false;
    bool false_lock = false;
    double score = 0.0;
};

double yawOf(const M3D& R) { return std::atan2(R(1, 0), R(0, 0)); }

bool loadTraj(const std::string& path, std::vector<PoseStamped>& out) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        double t, x, y, z, qx, qy, qz, qw;
        if (!(is >> t >> x >> y >> z >> qx >> qy >> qz >> qw)) continue;
        PoseStamped p;
        p.time = t;
        p.trans = V3D(x, y, z);
        Eigen::Quaterniond q(qw, qx, qy, qz);
        p.rot = q.normalized().toRotationMatrix();
        out.push_back(p);
    }
    return !out.empty();
}

CloudType::Ptr cutScan(const pcl::KdTreeFLANN<PointType>& kdt, const CloudType::Ptr& map,
                       const PoseStamped& pose, double radius, double noise, double drop,
                       std::mt19937& rng) {
    CloudType::Ptr scan(new CloudType());
    std::vector<int> ids;
    std::vector<float> sq;
    kdt.radiusSearch(PointType(static_cast<float>(pose.trans.x()),
                               static_cast<float>(pose.trans.y()),
                               static_cast<float>(pose.trans.z())),
                     static_cast<float>(radius), ids, sq);
    if (ids.empty()) return scan;

    std::normal_distribution<double> nd(0.0, noise);
    std::uniform_real_distribution<double> ud(0.0, 1.0);
    const M3D Rinv = pose.rot.transpose();
    scan->reserve(ids.size());
    for (int id : ids) {
        if (drop > 0.0 && ud(rng) < drop) continue;
        const PointType& p = map->points[static_cast<size_t>(id)];
        V3D d(p.x - static_cast<float>(pose.trans.x()),
              p.y - static_cast<float>(pose.trans.y()),
              p.z - static_cast<float>(pose.trans.z()));
        V3D s = Rinv * d;
        if (noise > 0.0) {
            s.x() += nd(rng);
            s.y() += nd(rng);
            s.z() += nd(rng);
        }
        const double r2 = s.squaredNorm();
        if (r2 < 1.0 || r2 > radius * radius) continue;   // 传感器近区/远区
        PointType q;
        q.x = static_cast<float>(s.x());
        q.y = static_cast<float>(s.y());
        q.z = static_cast<float>(s.z());
        q.intensity = p.intensity;
        scan->push_back(q);
    }
    scan->width = static_cast<uint32_t>(scan->size());
    scan->height = 1;
    scan->is_dense = false;
    return scan;
}

void printSummary(const std::string& name, const std::vector<Result>& rs, const Args& a) {
    std::vector<double> lat;
    int hit = 0, reported = 0, fl = 0;
    for (const auto& r : rs) {
        lat.push_back(r.ms);
        if (r.reported) ++reported;
        if (r.hit) ++hit;
        if (r.false_lock) ++fl;
    }
    std::sort(lat.begin(), lat.end());
    auto pct = [&](double p) {
        if (lat.empty()) return 0.0;
        const size_t i = std::min(lat.size() - 1,
                                  static_cast<size_t>(p * (lat.size() - 1) + 0.5));
        return lat[i];
    };
    const double recall = rs.empty() ? 0.0 : 100.0 * hit / rs.size();
    const double flr = rs.empty() ? 0.0 : 100.0 * fl / rs.size();
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "\n[" << name << "] scenarios=" << rs.size()
              << " recall=" << recall << "%"
              << " false_lock=" << flr << "%"
              << " (reported " << reported << ")"
              << " | latency p50=" << pct(0.50) << "ms p95=" << pct(0.95)
              << "ms max=" << pct(1.0) << "ms\n";
    std::cout << "  detail: ";
    for (const auto& r : rs) {
        std::cout << (r.hit ? "H" : (r.false_lock ? "F" : "-"));
    }
    std::cout << "\n  per-scenario ms: ";
    for (const auto& r : rs) {
        std::cout << std::fixed << std::setprecision(0) << r.ms << " ";
    }
    std::cout << "   (H=hit F=false-lock -=miss, tol=" << a.pos_tol << "m/"
              << a.yaw_tol_deg << "deg)\n";
}

}  // namespace

int main(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (s == "--localizer") a.localizer = need("--localizer");
        else if (s == "--scenarios") a.scenarios = std::atoi(need("--scenarios").c_str());
        else if (s == "--scan-radius") a.scan_radius = std::atof(need("--scan-radius").c_str());
        else if (s == "--voxel") a.voxel = std::atof(need("--voxel").c_str());
        else if (s == "--pos-tol") a.pos_tol = std::atof(need("--pos-tol").c_str());
        else if (s == "--yaw-tol") a.yaw_tol_deg = std::atof(need("--yaw-tol").c_str());
        else if (s == "--false-lock") a.false_lock = std::atof(need("--false-lock").c_str());
        else if (s == "--noise") a.noise = std::atof(need("--noise").c_str());
        else if (s == "--drop") a.drop = std::atof(need("--drop").c_str());
        else if (s == "--topk") a.topk = std::atoi(need("--topk").c_str());
        else if (s == "--submap") a.submap = std::atof(need("--submap").c_str());
        else if (s == "--build-sc-db") a.build_sc_db = std::atoi(need("--build-sc-db").c_str());
        else if (s == "--seed") a.seed = static_cast<unsigned>(std::atoi(need("--seed").c_str()));
        else if (s == "--help") {
            std::cout << "usage: reloc_bench <map.pcd> <traj.tum> [--localizer bbs|scan_context|"
                         "fpfh_ransac|all] [--scenarios N] [--submap M]\n";
            return 0;
        } else if (s[0] != '-' && a.map_path.empty()) a.map_path = s;
        else if (s[0] != '-' && a.traj_path.empty()) a.traj_path = s;
    }
    if (a.map_path.empty() || a.traj_path.empty()) {
        std::cerr << "usage: reloc_bench <map.pcd> <traj.tum> [options] (see --help)\n";
        return 2;
    }

    CloudType::Ptr map(new CloudType());
    if (pcl::io::loadPCDFile<PointType>(a.map_path, *map) < 0 || map->empty()) {
        std::cerr << "cannot load map: " << a.map_path << "\n";
        return 2;
    }
    map = filterFinite(map);
    map->width = static_cast<uint32_t>(map->size());
    map->height = 1;

    std::vector<PoseStamped> traj;
    if (!loadTraj(a.traj_path, traj)) {
        std::cerr << "cannot load trajectory: " << a.traj_path << "\n";
        return 2;
    }

    pcl::KdTreeFLANN<PointType> map_kdt;
    map_kdt.setInputCloud(map);

    // 场景: 轨迹上等间隔取样
    std::mt19937 rng(a.seed);
    std::vector<Scenario> scen;
    const size_t n = static_cast<size_t>(std::max(1, a.scenarios));
    for (size_t i = 0; i < n; ++i) {
        const size_t k = traj.size() == 1 ? 0 : i * (traj.size() - 1) / (n - 1 ? n - 1 : 1);
        Scenario sc;
        sc.gt = traj[std::min(k, traj.size() - 1)];
        sc.scan = cutScan(map_kdt, map, sc.gt, a.scan_radius, a.noise, a.drop, rng);
        if (sc.scan->size() < 100) continue;
        scen.push_back(sc);
    }
    std::cout << "map points=" << map->size() << "  scenarios=" << scen.size()
              << "  scan radius=" << a.scan_radius << " m"
              << "  submap=" << a.submap << " m\n";

    GlobalLocalizeConfig cfg;
    cfg.reloc_submap_radius = a.submap;
    cfg.voxel_resolution = a.voxel;
    cfg.top_k = a.topk;

    // 需要 ScanContext 库时, 用地图+轨迹现造一份 (等价于建图时落盘的 .scdb)
    std::string sc_db_base;
    if (a.build_sc_db > 0) {
        ScanContext::Config sc_cfg;
        sc_cfg.num_rings = cfg.sc_num_rings;
        sc_cfg.num_sectors = cfg.sc_num_sectors;
        sc_cfg.max_radius = cfg.sc_max_radius;
        sc_cfg.min_radius = cfg.sc_min_radius;
        ScanContext sc(sc_cfg);
        std::vector<ScanContextDatabaseEntry> db;
        const int kf = std::max(1, a.build_sc_db);
        for (int i = 0; i < kf; ++i) {
            const size_t k = traj.size() == 1 ? 0 : i * (traj.size() - 1) / std::max(1, kf - 1);
            const PoseStamped& p = traj[std::min(k, traj.size() - 1)];
            CloudType::Ptr s = cutScan(map_kdt, map, p, a.scan_radius, 0.0, 0.0, rng);
            if (s->size() < 100) continue;
            ScanContextDatabaseEntry e;
            e.id = "kf" + std::to_string(i);
            e.rot = p.rot;
            e.trans = p.trans;
            e.desc = sc.makeDescriptor(s);
            db.push_back(std::move(e));
        }
        sc_db_base = a.map_path + ".bench";
        if (!saveScanContextDatabase(sc_db_base + ".scdb", db)) {
            std::cerr << "cannot write ScanContext db\n";
            return 2;
        }
        std::cout << "ScanContext db: " << db.size() << " kfs -> " << sc_db_base << ".scdb\n";
    }

    std::vector<std::string> names;
    if (a.localizer == "all") names = {"bbs", "scan_context", "fpfh_ransac"};
    else names = {a.localizer};

    for (const auto& lname : names) {
        auto loc = LocalizationFactory::createGlobalLocalizer(lname);
        if (!loc) {
            std::cerr << "unknown localizer: " << lname << "\n";
            continue;
        }
        loc->configure(cfg);
        const std::string map_arg = (lname == "scan_context" && !sc_db_base.empty())
                                        ? sc_db_base
                                        : std::string();
        const auto t_setmap0 = std::chrono::steady_clock::now();
        const bool ok = loc->setMap(map_arg, map);
        const double setmap_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_setmap0).count();
        if (!ok) {
            std::cout << "[" << lname << "] setMap failed (需要 .scdb 时加 --build-sc-db N)\n";
            continue;
        }
        std::cout << "[" << lname << "] setMap (含建库/建树) " << std::fixed
                  << std::setprecision(0) << setmap_ms << " ms\n";

        std::vector<Result> rs;
        for (const auto& sc : scen) {
            std::vector<GlobalCandidate> cands;
            const auto t0 = std::chrono::steady_clock::now();
            const int nk = loc->localizeTopK(sc.scan, a.topk, cands);
            Result r;
            r.ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - t0).count();
            if (nk > 0 && !cands.empty()) {
                // 取分数最高者, 与管线 top1 一致
                auto best = std::max_element(cands.begin(), cands.end(),
                                             [](const GlobalCandidate& x, const GlobalCandidate& y) {
                                                 return x.score < y.score;
                                             });
                r.reported = true;
                r.score = best->score;
                r.err = (best->pose.trans - sc.gt.trans).norm();
                r.yaw_err_deg = std::fabs(yawOf(best->pose.rot) - yawOf(sc.gt.rot)) * 180.0 / M_PI;
                if (r.yaw_err_deg > 180.0) r.yaw_err_deg = 360.0 - r.yaw_err_deg;
                r.hit = (r.err <= a.pos_tol && r.yaw_err_deg <= a.yaw_tol_deg);
                r.false_lock = !r.hit && r.err > a.false_lock;
            }
            rs.push_back(r);
        }
        printSummary(lname, rs, a);
    }
    return 0;
}
