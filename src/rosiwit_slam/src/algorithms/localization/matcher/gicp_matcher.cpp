// src/algorithms/localization/matcher/gicp_matcher.cpp
#include "algorithms/localization/matcher/gicp_matcher.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/registration_metrics.h"

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
    m_kdtree.setInputCloud(m_target);
}

bool GicpMatcher::match(const CloudType::Ptr& scan, const PoseStamped& guess,
                        MatchResult& out) {
    out = MatchResult();
    if (!m_target || m_target->empty()) return false;
    CloudType::Ptr src = downsampleCloud(scan, m_cfg.voxel_resolution);
    if (!src || src->empty()) return false;

    Eigen::Matrix4f g = poseToMatrix(guess).cast<float>();
    m_gicp.setInputSource(src);
    m_gicp.setInputTarget(m_target);
    CloudType aligned;
    m_gicp.align(aligned, g);

    out.iterations = m_gicp.getMaximumIterations();
    if (!m_gicp.hasConverged()) { out.converged = false; return false; }

    const Eigen::Matrix4d T = m_gicp.getFinalTransformation().cast<double>();
    out.pose = matrixToPose(T, guess.time);
    const CoverageStats cs = evaluateCoverage(m_kdtree, *src, T, m_cfg.inlier_dist);
    out.fitness = cs.coverage;
    out.rmse = cs.mean_dist;
    out.inliers = cs.inliers;
    out.correspondences = cs.total;
    out.residual = evaluatePoint2PlaneResidual(m_kdtree, *src, T,
                                               m_cfg.max_correspondence_dist);
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
