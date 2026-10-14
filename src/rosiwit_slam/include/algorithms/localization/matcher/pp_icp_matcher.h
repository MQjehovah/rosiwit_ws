// include/algorithms/localization/matcher/pp_icp_matcher.h
#pragma once
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_core/i_scan_matcher.h"

namespace rosiwit_slam {

/// Point-to-Plane ICP 配准器 (LOAM 风格解析雅可比), 内置工业级鲁棒性:
///   - 迭代内对应点半径衰减 (先大后小, 兼顾收敛域与错配剔除)
///   - Huber 鲁棒核 (抑制动态物体/离群对应)
///   - JtJ 特征值退化方向门控 (长直墙/纯旋转时沿不可观测方向不修正)
///   - 可选点到线边缘特征 (补沿墙切向 + yaw 约束)
class PpIcpMatcher : public IScanMatcher {
public:
    std::string name() const override { return "pp_icp"; }
    void configure(const ScanMatcherConfig& cfg) override { m_cfg = cfg; }
    void setTarget(const CloudType::Ptr& target) override;
    bool match(const CloudType::Ptr& scan, const PoseStamped& guess,
               MatchResult& out) override;
    double evaluateResidual(const CloudType::Ptr& scan, const PoseStamped& pose) override;

private:
    double corrDistForIter(int iter) const;

    ScanMatcherConfig m_cfg;
    CloudType::Ptr m_target;
    pcl::KdTreeFLANN<PointType> m_kdtree;
};

} // namespace rosiwit_slam
