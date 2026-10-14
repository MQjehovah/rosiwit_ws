// include/algorithms/localization/motion_prior/constant_velocity_prior.h
#pragma once
#include "slam_core/i_motion_prior.h"

namespace rosiwit_slam {

/// 常速模型先验: 由最近两次配准位姿估计速度/角速度, 线性外推.
/// 不依赖 IMU, 作为对照/降级方案.
class ConstantVelocityPrior : public IMotionPrior {
public:
    std::string name() const override { return "constant_velocity"; }
    void configure(const MotionPriorConfig& cfg) override { m_cfg = cfg; }
    void reset(const PoseStamped& pose) override;
    PoseStamped predict(double t) override;
    void onMatch(const PoseStamped& pose) override;

private:
    MotionPriorConfig m_cfg;
    PoseStamped m_last_pose;
    double m_last_time = -1.0;
    V3D m_vel = V3D::Zero();
    double m_yaw_rate = 0.0;
    bool m_have_prev = false;
};

} // namespace rosiwit_slam
