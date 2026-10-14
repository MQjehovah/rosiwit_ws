// src/algorithms/localization/motion_prior/imu_motion_prior.cpp
#include "algorithms/localization/motion_prior/imu_motion_prior.h"
#include <cmath>

namespace rosiwit_slam {

void ImuMotionPrior::reset(const PoseStamped& pose) {
    m_pose = pose;
    m_last_imu_time = -1.0;
    m_first_imu = false;
    m_vel_world.setZero();
    m_last_match_time = pose.time;
    m_last_match_trans = pose.trans;
    m_have_offset = false;
}

void ImuMotionPrior::onImu(const IMUSample& s) {
    if (!m_first_imu) {
        m_last_imu_time = s.time;
        m_first_imu = true;
        return;
    }
    const double dt = s.time - m_last_imu_time;
    if (dt <= 0.0 || dt > m_cfg.max_dt) { m_last_imu_time = s.time; return; }

    const V3D angle_inc = s.gyro * dt;
    const double an = angle_inc.norm();
    if (an > 1e-12)
        m_pose.rot = m_pose.rot * Eigen::AngleAxisd(an, angle_inc / an).toRotationMatrix();

    m_pose.trans += m_vel_world * dt;
    m_pose.vel = m_vel_world;
    m_last_imu_time = s.time;
    m_gyro_yaw_rate = s.gyro.z();   // 机体系偏航角速度(用于打滑判据)
}

void ImuMotionPrior::onOdom(const PoseStamped& odom) {
    if (m_have_odom && odom.time > m_odom_time) {
        const double dt = odom.time - m_odom_time;
        if (dt > 1e-3) {
            const M3D dR = odom.rot * m_odom.rot.transpose();
            m_odom_yaw_rate = std::atan2(dR(1, 0), dR(0, 0)) / dt;
        }
    }
    m_odom = odom;
    m_odom_time = odom.time;
    m_have_odom = true;
}

PoseStamped ImuMotionPrior::predict(double t) {
    PoseStamped p = m_pose;
    p.time = t;
    if (m_cfg.use_odom && m_have_odom && m_have_offset) {
        // 打滑门控: odom 偏航角速度与 IMU 一致时才用 odom 平移
        const double slip = std::fabs(m_odom_yaw_rate - m_gyro_yaw_rate);
        if (slip <= m_cfg.slip_yaw_thresh) {
            p.trans = m_offset.rot * m_odom.trans + m_offset.trans;  // 朝向仍用 IMU 积分
        }
    }
    p.vel = m_vel_world;
    return p;
}

void ImuMotionPrior::onMatch(const PoseStamped& pose) {
    if (m_last_match_time >= 0.0 && pose.time > m_last_match_time) {
        const double dt = pose.time - m_last_match_time;
        if (dt > 1e-3) {
            const V3D inst = (pose.trans - m_last_match_trans) / dt;
            m_vel_world = 0.5 * m_vel_world + 0.5 * inst;
        }
    }
    // map <- odom 偏移, 供 odom 平移预测
    if (m_cfg.use_odom && m_have_odom) {
        m_offset.rot = pose.rot * m_odom.rot.transpose();
        m_offset.trans = pose.trans - m_offset.rot * m_odom.trans;
        m_have_offset = true;
    }
    m_pose = pose;
    m_pose.vel = m_vel_world;
    m_last_match_time = pose.time;
    m_last_match_trans = pose.trans;
}

} // namespace rosiwit_slam
