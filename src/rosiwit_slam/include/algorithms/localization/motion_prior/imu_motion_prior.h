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
    void onOdom(const PoseStamped& odom) override;
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
    // 轮式里程计融合 (打滑门控)
    PoseStamped m_odom;
    double m_odom_time = -1.0;
    double m_odom_yaw_rate = 0.0;
    double m_gyro_yaw_rate = 0.0;
    PoseStamped m_offset;          // map <- odom 偏移 (map_pose ∘ odom^{-1})
    bool m_have_odom = false;
    bool m_have_offset = false;
};

} // namespace rosiwit_slam
