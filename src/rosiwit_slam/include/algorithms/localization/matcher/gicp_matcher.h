// include/algorithms/localization/matcher/gicp_matcher.h
#pragma once
#include <pcl/registration/gicp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_core/i_scan_matcher.h"

namespace rosiwit_slam {

/// PCL GICP 配准器 (基线实现, 作为对照).
class GicpMatcher : public IScanMatcher {
public:
    std::string name() const override { return "gicp"; }
    void configure(const ScanMatcherConfig& cfg) override;
    void setTarget(const CloudType::Ptr& target) override;
    bool match(const CloudType::Ptr& scan, const PoseStamped& guess,
               MatchResult& out) override;
    double evaluateResidual(const CloudType::Ptr& scan, const PoseStamped& pose) override;

private:
    ScanMatcherConfig m_cfg;
    CloudType::Ptr m_target;
    pcl::KdTreeFLANN<PointType> m_kdtree;
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> m_gicp;
};

} // namespace rosiwit_slam
