// include/algorithms/localization/matcher/ndt2d_matcher.h
#pragma once
#include <unordered_map>
#include <cstdint>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_core/i_scan_matcher.h"

namespace rosiwit_slam {

/// 2.5D 平面 NDT 配准 (matcher: ndt2d): 在 (x,y,yaw) 上求解, z/roll/pitch 取自初值
/// (随后由地面约束/planar 兜底). 面向货架/狭道等特征匮乏、以平面结构为主的场景.
/// 目标按 z 带投影成 2D 高斯格 (NDT), 点到分布 Mahalanobis 残差, Gauss-Newton.
class Ndt2dMatcher : public IScanMatcher {
public:
    std::string name() const override { return "ndt2d"; }
    void configure(const ScanMatcherConfig& cfg) override { m_cfg = cfg; }
    void setTarget(const CloudType::Ptr& target) override;
    bool match(const CloudType::Ptr& scan, const PoseStamped& guess,
               MatchResult& out) override;
    double evaluateResidual(const CloudType::Ptr& scan, const PoseStamped& pose) override;

private:
    struct Cell {
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        Eigen::Matrix2d cov_inv = Eigen::Matrix2d::Identity();
        int n = 0;
    };
    struct Level {
        std::unordered_map<int64_t, Cell> cells;
        double cell = 0.5;
    };
    static int64_t key(int cx, int cy);
    const Cell* findCell(const Level& lv, double x, double y) const;
    static void buildLevel(Level& lv, const std::vector<Eigen::Vector2d>& pts);

    ScanMatcherConfig m_cfg;
    double m_zmin = -1.0, m_zmax = 1.0;
    std::vector<Level> m_levels;   // 由粗到细
};

} // namespace rosiwit_slam
