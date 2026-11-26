// include/algorithms/localization/matcher/gicp_matcher.h
#pragma once
#include <pcl/registration/registration.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <memory>
#include "slam_core/i_scan_matcher.h"

namespace rosiwit_slam {

/// GICP 类配准器: 求解引擎可替换 (PCL GICP / fast_gicp::FastGICP ...), 前后处理与
/// 评分 (覆盖率/内点/残差/可观测性) 完全共用 —— 引擎只决定"怎么解", 不改变
/// fitness 的口径, 便于各实现 A/B。
class GicpMatcher : public IScanMatcher {
public:
    explicit GicpMatcher(std::unique_ptr<pcl::Registration<PointType, PointType>> engine,
                         std::string name = "gicp")
        : m_name(std::move(name)), m_gicp(std::move(engine)) {}
    std::string name() const override { return m_name; }
    void configure(const ScanMatcherConfig& cfg) override;
    void setTarget(const CloudType::Ptr& target) override;
    bool match(const CloudType::Ptr& scan, const PoseStamped& guess,
               MatchResult& out) override;
    double evaluateResidual(const CloudType::Ptr& scan, const PoseStamped& pose) override;

protected:
    /// 求解核心: 从 guess_T 出发把 src 对齐到 m_target。返回 false = 未收敛/失败。
    /// GicpMatcher 用 GICP 引擎; LioMatcher 覆写为 iEKF 点到面。评分在 match() 中共用。
    virtual bool align(const CloudType::Ptr& src, const Eigen::Matrix4d& guess_T,
                       Eigen::Matrix4d& T_out, int& iterations);

    std::string m_name;
    ScanMatcherConfig m_cfg;
    CloudType::Ptr m_target;
    pcl::KdTreeFLANN<PointType> m_kdtree;
    std::unique_ptr<pcl::Registration<PointType, PointType>> m_gicp;
};

} // namespace rosiwit_slam
