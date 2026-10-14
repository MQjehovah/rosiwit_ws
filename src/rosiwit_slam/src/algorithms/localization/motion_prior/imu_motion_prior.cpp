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
}

PoseStamped ImuMotionPrior::predict(double t) {
    PoseStamped p = m_pose;
    p.time = t;
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
    m_pose = pose;
    m_pose.vel = m_vel_world;
    m_last_match_time = pose.time;
    m_last_match_trans = pose.trans;
}

} // namespace rosiwit_slam
