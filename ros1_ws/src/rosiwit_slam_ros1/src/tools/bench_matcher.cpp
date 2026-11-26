// tools/bench_matcher.cpp — 离线 matcher 基准 (同一输入比耗时/位姿)
//
// 输入: ROSIWIT_DUMP_SEQ=N 转储的帧序列 (/tmp/slamdump_seq/f_%03d/{scan.pcd,poses.txt}
//       + target_<k>.pcd), scan 为 prepareScan 后的匹配器输入 (与线上完全一致)。
// 用法: bench_matcher [dump_dir] [config.yaml] [names] [repeats]
//   默认: /tmp/slamdump_seq  default.yaml  gicp,pp_icp,vgicp,fast_gicp,lio  5
// 输出: 每帧每种 matcher 的 对齐耗时(均值/最大)、setTarget 耗时、fitness、
//       相对参考解 (第一个 matcher) 的位置差/yaw 差、迭代数。
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstdio>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "algorithms/localization/localization_factory.h"
#include "slam_core/localization_stage_types.h"

using namespace rosiwit_slam;

namespace {

template <typename T>
T getv(const YAML::Node& n, const char* key, const T& def) {
    if (n && n[key]) { try { return n[key].as<T>(); } catch (...) {} }
    return def;
}

// 与 LocalizationPipeline::init 的 matcher 段解析保持一致
ScanMatcherConfig loadMatcherCfg(const std::string& yaml) {
    ScanMatcherConfig c;
    YAML::Node root = YAML::LoadFile(yaml);
    auto loc = root["localization"];
    if (!loc) return c;
    auto mc = loc["matcher"];
    if (!mc) return c;
    c.voxel_resolution        = getv<double>(mc, "voxel_resolution", c.voxel_resolution);
    c.max_iterations          = getv<int>(mc, "max_iterations", c.max_iterations);
    c.min_match_points        = getv<int>(mc, "min_match_points", c.min_match_points);
    c.max_correspondence_dist = getv<double>(mc, "max_correspondence_dist", c.max_correspondence_dist);
    c.transformation_epsilon  = getv<double>(mc, "transformation_epsilon", c.transformation_epsilon);
    c.inlier_dist             = getv<double>(mc, "inlier_dist", c.inlier_dist);
    c.use_corr_decay          = getv<bool>(mc, "use_corr_decay", c.use_corr_decay);
    c.corr_dist_init          = getv<double>(mc, "corr_dist_init", c.corr_dist_init);
    c.corr_dist_min           = getv<double>(mc, "corr_dist_min", c.corr_dist_min);
    c.use_huber               = getv<bool>(mc, "use_huber", c.use_huber);
    c.huber_delta             = getv<double>(mc, "huber_delta", c.huber_delta);
    const std::string rk = getv<std::string>(mc, "robust_kernel", "");
    if (rk == "none")   c.robust_kernel = RobustKernel::NONE;
    if (rk == "huber")  c.robust_kernel = RobustKernel::HUBER;
    if (rk == "cauchy") c.robust_kernel = RobustKernel::CAUCHY;
    if (rk == "tukey")  c.robust_kernel = RobustKernel::TUKEY;
    if (rk == "gnc")    c.robust_kernel = RobustKernel::GNC;
    c.gnc_mu0               = getv<double>(mc, "gnc_mu0", c.gnc_mu0);
    c.gnc_decay             = getv<double>(mc, "gnc_decay", c.gnc_decay);
    c.use_edge_feature      = getv<bool>(mc, "use_edge_feature", c.use_edge_feature);
    c.edge_line_ratio       = getv<double>(mc, "edge_line_ratio", c.edge_line_ratio);
    c.edge_valid_thr        = getv<double>(mc, "edge_valid_thr", c.edge_valid_thr);
    c.edge_min_extent       = getv<double>(mc, "edge_min_extent", c.edge_min_extent);
    c.edge_vertical_cos     = getv<double>(mc, "edge_vertical_cos", c.edge_vertical_cos);
    c.degeneracy_gate       = getv<bool>(mc, "degeneracy_gate", c.degeneracy_gate);
    c.degeneracy_eig_ratio  = getv<double>(mc, "degeneracy_eig_ratio", c.degeneracy_eig_ratio);
    c.min_correspondences   = getv<int>(mc, "min_correspondences", c.min_correspondences);
    c.dynamic_filter        = getv<bool>(mc, "dynamic_filter", c.dynamic_filter);
    c.dynamic_dist          = getv<double>(mc, "dynamic_dist", c.dynamic_dist);
    // 转储的扫描已经过 prepareScan 的高度带裁剪: 基准里不再按 guess 裁 (避免口径漂移)
    c.match_z_filter        = false;
    c.min_coverage_points   = getv<int>(mc, "min_coverage_points", c.min_coverage_points);
    c.min_inliers_full_score = getv<int>(mc, "min_inliers_full_score", c.min_inliers_full_score);
    return c;
}

struct Frame {
    std::string name;
    CloudType::Ptr scan;
    CloudType::Ptr target;
    PoseStamped guess, res;
    double fitness = 0.0;
};

bool readPoses(const std::string& path, Frame& fr, std::string& tgt) {
    std::ifstream f(path);
    if (!f) return false;
    std::string k;
    while (f >> k) {
        if (k == "fitness") f >> fr.fitness;
        else if (k == "target") f >> tgt;
        else if (k == "matcher") { std::string m; f >> m; }
        else if (k == "guess_trans") f >> fr.guess.trans.x() >> fr.guess.trans.y() >> fr.guess.trans.z();
        else if (k == "guess_rot" || k == "res_rot") {
            double x, y, z, w; f >> x >> y >> z >> w;
            const Eigen::Matrix3d R = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
            (k == "guess_rot" ? fr.guess : fr.res).rot = R;
        } else if (k == "res_trans") f >> fr.res.trans.x() >> fr.res.trans.y() >> fr.res.trans.z();
        else { std::string rest; std::getline(f, rest); }
    }
    fr.guess.time = fr.res.time = 0.0;
    return true;
}

std::vector<std::string> listFrames(const std::string& dir) {
    std::vector<std::string> out;
    DIR* d = opendir(dir.c_str());
    if (!d) return out;
    while (dirent* e = readdir(d))
        if (std::string(e->d_name).rfind("f_", 0) == 0) out.push_back(e->d_name);
    closedir(d);
    std::sort(out.begin(), out.end());
    return out;
}

double yawOf(const Eigen::Matrix3d& R) { return std::atan2(R(1, 0), R(0, 0)) * 57.2957795; }
// 中位绝对偏差 (MAD): 对少数移动帧鲁棒 —— 静止机器人下反映解的抖动/噪声底
double mad(std::vector<double> v) {
    if (v.size() < 3) return 0.0;
    std::sort(v.begin(), v.end());
    const double med = v[v.size() / 2];
    std::vector<double> d;
    for (double x : v) d.push_back(std::fabs(x - med));
    std::sort(d.begin(), d.end());
    return d[d.size() / 2];
}

} // namespace

int main(int argc, char** argv) {
    const std::string dir    = argc > 1 ? argv[1] : "/tmp/slamdump_seq";
    const std::string yaml   = argc > 2 ? argv[2] : "/opt/ros/melodic/share/rosiwit_slam_ros1/config/default.yaml";
    const std::string names  = argc > 3 ? argv[3] : "gicp,pp_icp,vgicp,fast_gicp,lio";
    const int repeats        = argc > 4 ? std::atoi(argv[4]) : 5;

    std::vector<std::string> mnames;
    {
        std::stringstream ss(names); std::string t;
        while (std::getline(ss, t, ',')) if (!t.empty()) mnames.push_back(t);
    }
    const ScanMatcherConfig cfg = loadMatcherCfg(yaml);
    std::cout << "cfg: voxel=" << cfg.voxel_resolution << " max_it=" << cfg.max_iterations
              << " max_corr=" << cfg.max_correspondence_dist << " eps=" << cfg.transformation_epsilon
              << " inlier=" << cfg.inlier_dist << std::endl;

    // 载入帧
    std::vector<Frame> frames;
    std::map<std::string, CloudType::Ptr> targets;
    for (const auto& fn : listFrames(dir)) {
        Frame fr; fr.name = fn;
        std::string tgt;
        if (!readPoses(dir + "/" + fn + "/poses.txt", fr, tgt)) continue;
        fr.scan.reset(new CloudType());
        if (pcl::io::loadPCDFile(dir + "/" + fn + "/scan.pcd", *fr.scan) < 0) continue;
        if (targets.count(tgt) == 0) {
            CloudType::Ptr t(new CloudType());
            if (pcl::io::loadPCDFile(tgt, *t) < 0) continue;
            targets[tgt] = t;
        }
        fr.target = targets[tgt];
        frames.push_back(fr);
    }
    if (frames.empty()) { std::cerr << "no frames in " << dir << std::endl; return 1; }
    std::cout << "frames: " << frames.size() << "  targets: " << targets.size() << std::endl;

    struct Agg { double ms_sum = 0, ms_max = 0, fit = 0, dt = 0, dy = 0, odt = 0, ody = 0; int n = 0, fail = 0, its = 0; };
    std::map<std::string, Agg> agg;
    std::map<std::string, std::vector<PoseStamped>> results;
    // 扰动恢复测试: guess 偏移 (0.30m, 3°), 看各 matcher 能否回到无扰动解
    const bool do_perturb = (argc > 5 && std::string(argv[5]) == "perturb");
    struct Pert { double err = 0, yaw = 0; int n = 0, fail = 0; };
    std::map<std::string, Pert> pert;

    for (const auto& mn : mnames) {
        auto m = LocalizationFactory::createMatcher(mn);
        if (!m) { std::cout << "[skip] " << mn << " (unavailable)" << std::endl; continue; }
        m->configure(cfg);
        double set_ms_sum = 0;
        const CloudType::Ptr last_target;
        for (size_t fi = 0; fi < frames.size(); ++fi) {
            const Frame& fr = frames[fi];
            // 每帧都 setTarget 一次以单独计时 (线上是缓存命中, 这里量的是"重建一次"的成本)
            const auto ts = std::chrono::steady_clock::now();
            m->setTarget(fr.target);
            const double set_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - ts).count();
            set_ms_sum += set_ms;
            MatchResult r;
            if (!m->match(fr.scan, fr.guess, r) || !r.converged) {
                ++agg[mn].fail; results[mn].push_back(fr.guess); continue;
            }
            double best = 1e9;
            for (int k = 0; k < repeats; ++k) {
                MatchResult rk;
                const auto t0 = std::chrono::steady_clock::now();
                m->match(fr.scan, fr.guess, rk);
                const double ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - t0).count();
                best = std::min(best, ms);
            }
            Agg& a = agg[mn];
            a.ms_sum += best; a.ms_max = std::max(a.ms_max, best); a.fit += r.fitness;
            a.its += r.iterations; ++a.n;
            results[mn].push_back(r.pose);
            // 相对线上解 (poses.txt 的 res_*: 线上 gicp 8 轮的结果)
            agg[mn].odt += (r.pose.trans - fr.res.trans).norm();
            agg[mn].ody += std::fabs(yawOf(r.pose.rot * fr.res.rot.transpose()));
            if (do_perturb) {
                PoseStamped g2 = fr.guess;
                g2.trans.x() += 0.30; g2.trans.y() += 0.15;
                g2.rot = Eigen::AngleAxisd(3.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() * g2.rot;
                MatchResult r2;
                if (!m->match(fr.scan, g2, r2) || !r2.converged) { ++pert[mn].fail; continue; }
                const double ang = std::acos(std::max(-1.0, std::min(1.0,
                    ((r2.pose.rot * r.pose.rot.transpose()).trace() - 1.0) * 0.5))) * 57.2957795;
                pert[mn].err += (r2.pose.trans - r.pose.trans).norm();
                pert[mn].yaw += ang; ++pert[mn].n;
            }
        }
        const Agg& a = agg[mn];
        // 相对参考解 (第一个可用 matcher) 的位姿差
        const std::string ref = mnames.front();
        if (mn != ref && results.count(ref) && results[ref].size() == results[mn].size()) {
            for (size_t i = 0; i < results[mn].size(); ++i) {
                const double dt = (results[mn][i].trans - results[ref][i].trans).norm();
                const double dy = std::fabs(yawOf(results[mn][i].rot * results[ref][i].rot.transpose()));
                if (dt < 5.0) { agg[mn].dt += dt; agg[mn].dy += dy; }
            }
        }
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "[result] " << mn << ": match_ms(mean/max)="
                  << (a.n ? a.ms_sum / a.n : 0.0) << "/" << a.ms_max
                  << "  setTarget_ms(mean)=" << set_ms_sum / frames.size()
                  << "  fitness=" << (a.n ? a.fit / a.n : 0.0)
                  << "  iter=" << (a.n ? double(a.its) / a.n : 0.0)
                  << "  fail=" << a.fail << "/" << frames.size()
                  << "  dpos_vs_online=" << (a.n ? agg[mn].odt / a.n * 100.0 : 0.0) << "cm"
                  << "/dyaw=" << (a.n ? agg[mn].ody / a.n : 0.0) << "deg"
                  << "  dpos_vs_" << ref << "=" << (a.n ? agg[mn].dt / a.n * 100.0 : 0.0) << "cm"
                  << "  dyaw=" << (a.n ? agg[mn].dy / a.n : 0.0) << "deg";
        if (results.count(mn) && results[mn].size() >= 3) {
            std::vector<double> xs, ys, yw;
            for (const auto& p : results[mn]) { xs.push_back(p.trans.x()); ys.push_back(p.trans.y()); yw.push_back(yawOf(p.rot)); }
            // yaw 环绕 ±180: 用与中位数的差
            std::cout << "  jitter(MAD): xy=" << std::max(mad(xs), mad(ys)) * 100.0 << "cm yaw=" << mad(yw) << "deg";
        }
        if (do_perturb) {
            const Pert& q = pert[mn];
            std::cout << "  | perturb(+0.34m,+3deg): recover_err=" << (q.n ? q.err / q.n * 100.0 : -1.0)
                      << "cm yaw=" << (q.n ? q.yaw / q.n : -1.0) << "deg fail=" << q.fail;
        }
        std::cout << std::endl;
    }
    return 0;
}
