// include/algorithms/localization/motion_prior/imu_motion_prior.h
#pragma once
#include "slam_core/i_motion_prior.h"

namespace rosiwit_slam {

/// IMU 航迹推算先验: 角速度积分得到姿态, 位置用上一帧匹配速度常速推进
/// (避免加速度双重积分的漂移). 与旧 GicpLocalization 的滤波路径等价.
class ImuMotionPrior : public IMotionPrior {
public:
    std::string name() const override { return "imu"; }
    void configure(const MotionPriorConfig& cfg) override { m_cfg = cfg; }
    void reset(const PoseStamped& pose) override;
    void onImu(const IMUSample& s) override;
    PoseStamped predict(double t) override;
    void onMatch(const PoseStamped& pose) override;

private:
    MotionPriorConfig m_cfg;
    PoseStamped m_pose;
    double m_last_imu_time = -1.0;
    bool m_first_imu = false;
    V3D m_vel_world = V3D::Zero();
    double m_last_match_time = -1.0;
    V3D m_last_match_trans = V3D::Zero();
};

} // namespace rosiwit_slam
