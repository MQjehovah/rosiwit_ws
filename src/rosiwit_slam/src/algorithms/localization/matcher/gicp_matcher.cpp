// src/algorithms/localization/matcher/gicp_matcher.cpp
#include "algorithms/localization/matcher/gicp_matcher.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/registration_metrics.h"
#include <chrono>

namespace rosiwit_slam {

void GicpMatcher::configure(const ScanMatcherConfig& cfg) {
    m_cfg = cfg;
    m_gicp.setMaximumIterations(m_cfg.max_iterations);
    m_gicp.setMaxCorrespondenceDistance(m_cfg.max_correspondence_dist);
    m_gicp.setTransformationEpsilon(m_cfg.transformation_epsilon);
    m_gicp.setEuclideanFitnessEpsilon(1e-6);
}

void GicpMatcher::setTarget(const CloudType::Ptr& target) {
    m_target = downsampleCloud(target, m_cfg.voxel_resolution);
    if (!m_target) m_target.reset(new CloudType());
    // 退化目标守卫: 点数过少时 GICP 协方差估计无意义且会段错误, 置空让 match() 走拒绝分支
    if (m_target->size() < static_cast<size_t>(std::max(1, m_cfg.min_match_points)))
        m_target->clear();
    if (!m_target->empty()) m_kdtree.setInputCloud(m_target);
}

bool GicpMatcher::match(const CloudType::Ptr& scan, const PoseStamped& guess,
                        MatchResult& out) {
    // [MatchProbe] 每 512 次打一条分解计时 —— 定位输出 10Hz(雷达 15.8Hz)瓶颈定位用
    static int probe_n = 0;
    const bool probe = (++probe_n % 512 == 0);
    const auto t0 = std::chrono::steady_clock::now();
    double ms_ds = 0, ms_df = 0, ms_gicp = 0, ms_eval = 0;
    auto ms_since = [](const std::chrono::steady_clock::time_point& t) {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - t).count();
    };

    out = MatchResult();
    if (!m_target || m_target->empty()) return false;
    CloudType::Ptr src = downsampleCloud(scan, m_cfg.voxel_resolution);
    if (!src || src->empty()) return false;
    if (probe) ms_ds = ms_since(t0);
    if (m_cfg.dynamic_filter && m_cfg.dynamic_dist > 0) {
        src = filterByMapProximity(src, m_kdtree, m_cfg.dynamic_dist);
        if (!src || src->empty()) return false;
    }
    if (probe) ms_df = ms_since(t0) - ms_ds;

    // 退化扫描守卫(崩溃根因): 16 点扫描进 PCL GICP = 段错误, 见 ScanMatcherConfig 注释
    if (src->size() < static_cast<size_t>(std::max(1, m_cfg.min_match_points))) return false;

    const auto t1 = std::chrono::steady_clock::now();
    Eigen::Matrix4f g = poseToMatrix(guess).cast<float>();
    m_gicp.setInputSource(src);
    m_gicp.setInputTarget(m_target);
    CloudType aligned;
    m_gicp.align(aligned, g);
    if (probe) ms_gicp = ms_since(t1);

    out.iterations = m_gicp.getMaximumIterations();
    if (!m_gicp.hasConverged()) { out.converged = false; return false; }

    const Eigen::Matrix4d T = m_gicp.getFinalTransformation().cast<double>();
    out.pose = matrixToPose(T, guess.time);
    const auto t2 = std::chrono::steady_clock::now();
    const CoverageStats cs = evaluateCoverage(m_kdtree, *src, T, m_cfg.inlier_dist);
    out.fitness = cs.coverage;
    out.rmse = cs.mean_dist;
    out.inliers = cs.inliers;
    out.correspondences = cs.total;
    out.residual = evaluatePoint2PlaneResidual(m_kdtree, *src, T,
                                               m_cfg.max_correspondence_dist);
    if (probe) {
        ms_eval = ms_since(t2);
        std::cout << "[MatchProbe] n=" << probe_n << " scan=" << scan->size()
                  << " src=" << src->size() << " tgt=" << m_target->size()
                  << " ds=" << ms_ds << " dynf=" << ms_df
                  << " gicp=" << ms_gicp << " eval=" << ms_eval
                  << " total=" << ms_ds + ms_df + ms_gicp + ms_eval << "ms" << std::endl;
    }
    const double s = out.residual > 0 ? out.residual * out.residual
                                      : (out.rmse > 0 ? out.rmse * out.rmse : 1.0);
    out.cov = Eigen::Matrix<double, 6, 6>::Identity() * s;
    out.converged = true;
    return true;
}

double GicpMatcher::evaluateResidual(const CloudType::Ptr& scan, const PoseStamped& pose) {
    if (!m_target || m_target->empty()) return -1.0;
    CloudType::Ptr src = downsampleCloud(scan, m_cfg.voxel_resolution);
    return evaluatePoint2PlaneResidual(m_kdtree, *src, poseToMatrix(pose),
                                       m_cfg.max_correspondence_dist);
}

} // namespace rosiwit_slam
