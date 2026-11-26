// include/algorithms/localization/matcher/lio_matcher.h
#pragma once
#include <vector>
#include <Eigen/Core>
#include "algorithms/localization/matcher/gicp_matcher.h"

namespace rosiwit_slam {

/// LIO 式配准 (iEKF 点到面) —— 与建图前端同一类求解器, 作为 matcher 策略接入
/// (matcher: lio)。只覆写求解核心与目标预处理, 评分/前后处理与 GicpMatcher 共用。
///
/// 每个源点找最近目标点, 取该目标点所在平面 (法向 n, 中心 c):
///   残差 r = n·(R p + t − c),  雅可比 J = [ q × n ; n ]  (q = R p + t, 左扰动)
/// 累加 H = Σ w J Jᵀ,  b = −Σ w J r; 加先验 P0⁻¹ (来自 IMU/odom 预测),
/// 一次求解 δx = H⁻¹b, T ← Exp(δx)·T, 迭代至收敛 (先验准, 典型 2~4 次)。
///
/// 相比 PCL GICP: 无双侧协方差、无逐点马氏距离、无源点 kNN —— 单核成本低一个
/// 量级 (实机 8xA55 上 GICP 200ms~1s/帧, 预算 100ms)。平面的法向/中心在
/// setTarget 时预计算一次 (随目标在后台建, 不占跟踪线程)。
class LioMatcher : public GicpMatcher {
public:
    LioMatcher() : GicpMatcher(nullptr, "lio") {}
    void configure(const ScanMatcherConfig& cfg) override;
    void setTarget(const CloudType::Ptr& target) override;

protected:
    bool align(const CloudType::Ptr& src, const Eigen::Matrix4d& guess_T,
               Eigen::Matrix4d& T_out, int& iterations) override;

private:
    /// 目标点所在平面 (与 m_target 一一对应); 法向为零 = 无有效平面, 不参与。
    std::vector<Eigen::Vector3f> m_normal;
    std::vector<Eigen::Vector3f> m_center;

    int    m_max_iter    = 5;     // 迭代上限
    double m_prior_trans = 0.30;  // 先验 σ (m):   IMU/odom 预测的可信度
    double m_prior_rot   = 0.05;  // 先验 σ (rad)
    double m_sigma_r     = 0.20;  // 残差尺度 (m), Cauchy 权重 w = 1/(1+r²/σ²)
    int    m_solve_max   = 1500;  // 对齐用源点上限 (步进抽样; 评分仍用全量点)
    double m_conv_t      = 1e-3;  // 收敛: 平移增量 (m)
    double m_conv_r      = 1e-3;  // 收敛: 旋转增量 (rad)
    double m_plane_max_curv = 0.10; // 平面有效性: λmin/Σλ (越小越平)
};

} // namespace rosiwit_slam
