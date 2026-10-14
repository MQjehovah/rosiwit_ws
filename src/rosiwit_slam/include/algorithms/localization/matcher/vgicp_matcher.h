// include/algorithms/localization/matcher/vgicp_matcher.h
#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_core/i_scan_matcher.h"

namespace rosiwit_slam {

/// 体素化 GICP (VGICP-lite): 对目标建体素哈希, 每体素存均值+协方差; 源点做
/// 点-分布 Mahalanobis 残差 (C = C_target + σ_s²I), 复用鲁棒核/退化门控/OpenMP.
/// 相比 PCL GICP: 对应关系为 O(1) 体素查找(无需 KD-tree), 对非均匀点云更稳、更快.
class VgicpMatcher : public IScanMatcher {
public:
    std::string name() const override { return "vgicp"; }
    void configure(const ScanMatcherConfig& cfg) override { m_cfg = cfg; }
    void setTarget(const CloudType::Ptr& target) override;
    bool match(const CloudType::Ptr& scan, const PoseStamped& guess,
               MatchResult& out) override;
    double evaluateResidual(const CloudType::Ptr& scan, const PoseStamped& pose) override;

private:
    struct Voxel {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        Eigen::Matrix3d sum_sq = Eigen::Matrix3d::Zero();
        int n = 0;
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        bool valid = false;
    };
    static int64_t voxelKey(int cx, int cy, int cz);
    Voxel* findVoxel(const Eigen::Vector3d& p);

    double corrDistForIter(int iter) const;

    ScanMatcherConfig m_cfg;
    double m_leaf = 0.5;
    std::unordered_map<int64_t, Voxel> m_voxels;
    CloudType::Ptr m_target;
    pcl::KdTreeFLANN<PointType> m_kdtree;
};

} // namespace rosiwit_slam
