// include/algorithms/localization/matcher/p2p_icp_matcher.h
#pragma once
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_core/i_scan_matcher.h"

namespace rosiwit_slam {

/// 鲁棒点到点 ICP 配准器 (KISS-ICP 风格): 不依赖法向/特征, 直接最小化
/// 变换后源点到目标 1-NN 的残差, 用鲁棒核抑制外点; 内置对应半径衰减 + 退化门控.
/// 作为与 GICP(概率) / PP-ICP(点到面) 并列的第三种基线, 便于横向对比.
class P2pIcpMatcher : public IScanMatcher {
public:
    std::string name() const override { return "p2p_icp"; }
    void configure(const ScanMatcherConfig& cfg) override { m_cfg = cfg; }
    void setTarget(const CloudType::Ptr& target) override;
    void setStore(IMapStore* store) override { m_store = store; }
    bool usesStore() const override { return m_store != nullptr; }
    bool match(const CloudType::Ptr& scan, const PoseStamped& guess,
               MatchResult& out) override;
    double evaluateResidual(const CloudType::Ptr& scan, const PoseStamped& pose) override;

private:
    double corrDistForIter(int iter) const;

    ScanMatcherConfig m_cfg;
    CloudType::Ptr m_target;
    pcl::KdTreeFLANN<PointType> m_kdtree;
    IMapStore* m_store = nullptr;
};

} // namespace rosiwit_slam
