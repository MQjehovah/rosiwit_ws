// include/algorithms/localization/smoother/fixed_lag_smoother.h
#pragma once
#include <Eigen/Eigen>
#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

struct SmootherConfig {
    bool   enabled = false;
    double lag_sec = 2.0;             // 固定滞后窗口(s)
    double motion_trans_sigma = 0.05; // 运动先验平移噪声(std)
    double motion_rot_sigma = 0.02;   // 运动先验旋转噪声(std)
};

/// 固定滞后平滑器: 滑窗因子图融合 运动先验(between) + 扫描匹配(unary, 带协方差),
/// 输出平滑位姿与协方差, 替代逐帧 α 混合/硬替换. 无 GTSAM 时降级为直通(返回未平滑).
class FixedLagSmoother {
public:
    void configure(const SmootherConfig& cfg) { m_cfg = cfg; }
    bool enabled() const { return m_cfg.enabled; }

    void reset(const PoseStamped& init);
    void addFrame(double time, const PoseStamped& pred, bool has_match,
                  const PoseStamped& match, const Eigen::Matrix<double, 6, 6>& match_cov);
    bool getPose(PoseStamped& out, Eigen::Matrix<double, 6, 6>* cov = nullptr);

private:
    SmootherConfig m_cfg;
    long m_idx = 0;
    PoseStamped m_last_pred;
    PoseStamped m_last_est;
    bool m_has_last = false;
    Eigen::Matrix<double, 6, 6> m_last_cov = Eigen::Matrix<double, 6, 6>::Identity();
};

} // namespace rosiwit_slam
