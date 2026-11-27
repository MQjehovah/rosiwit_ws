// matcher_ab: 同一份目标, setTarget (原路径: 再降采样一遍) vs setTargetVoxelized (跳过), 匹配结果是否一致。
// 同时核对 setSearchMethodTarget(force_no_recompute) 不改变结果。   matcher_ab <target.pcd> <scan.pcd>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/io/pcd_io.h>
#define protected public
#define private public
#include "algorithms/localization/matcher/gicp_matcher.h"
#undef private
#undef protected
#include <chrono>
#include <cstdio>
using namespace rosiwit_slam;

int main(int argc, char** argv) {
    CloudType::Ptr tgt(new CloudType()), scan(new CloudType());
    pcl::io::loadPCDFile(argv[1], *tgt); pcl::io::loadPCDFile(argv[2], *scan);
    ScanMatcherConfig cfg; cfg.voxel_resolution = 0.3; cfg.max_iterations = 12; cfg.max_correspondence_dist = 1.2;
    cfg.transformation_epsilon = 5e-3; cfg.num_threads = 2; cfg.match_z_filter = false; cfg.dynamic_filter = false; cfg.min_match_points = 50;
    auto make = [&]() { return std::unique_ptr<GicpMatcher>(new GicpMatcher(std::unique_ptr<pcl::Registration<PointType, PointType>>(new fast_gicp::FastGICP<PointType, PointType>()), "fast_gicp")); };
    GicpMatcher::clearTargetCovarianceCache();
    PoseStamped guess; guess.rot = Eigen::Quaterniond(0.5915, 0.2035, 0.2686, 0.7325).normalized().toRotationMatrix(); guess.trans = Eigen::Vector3d(15.0839, -2.4900, 0.9844);
    MatchResult r[3];
    for (int k = 0; k < 3; ++k) {
        auto m = make();
        if (k == 2) {   // 第三次: 去掉 force_no_recompute (让 PCL 建它那棵树), 结果应与 k==1 相同
            m->configure(cfg);
            m->m_gicp->setSearchMethodTarget(m->m_gicp->getSearchMethodTarget(), false);
        } else m->configure(cfg);
        auto t0 = std::chrono::steady_clock::now();
        if (k == 0) m->setTarget(tgt); else m->setTargetVoxelized(tgt);
        double ms_set = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        const size_t n = m->m_target->size();
        auto t1 = std::chrono::steady_clock::now();
        const bool ok = m->match(scan, guess, r[k]);
        double ms_first = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count();
        MatchResult r2; auto t2 = std::chrono::steady_clock::now(); m->match(scan, guess, r2);
        double ms_second = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t2).count();
        std::printf("%s: 目标 %zu 点, setTarget %.0f ms | 第一次 match %.0f ms, 第二次 %.0f ms | ok=%d fit=%.6f iter=%d corr=%d pos=(%.6f, %.6f, %.6f)\n",
                    k == 0 ? "setTarget        (原路径)" : (k == 1 ? "setTargetVoxelized(跳过)  " : "跳过+PCL 建树         "),
                    n, ms_set, ms_first, ms_second, (int)ok, r[k].fitness, r[k].iterations, r[k].correspondences,
                    r[k].pose.trans.x(), r[k].pose.trans.y(), r[k].pose.trans.z());
    }
    for (int k = 1; k < 3; ++k) {
        const double dp = (r[k].pose.trans - r[0].pose.trans).norm();
        const Eigen::Matrix3d dR = r[k].pose.rot * r[0].pose.rot.transpose();
        std::printf("  %d vs 0: 位置差 %.2e m, 姿态差 %.2e deg, fitness 差 %.2e, 内点差 %d\n", k, dp,
                    std::acos(std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) * 0.5))) * 57.2957795, r[k].fitness - r[0].fitness, r[k].inliers - r[0].inliers);
    }
    return 0;
}
