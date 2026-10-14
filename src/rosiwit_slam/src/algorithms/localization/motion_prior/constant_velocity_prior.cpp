// src/algorithms/localization/motion_prior/constant_velocity_prior.cpp
#include "algorithms/localization/motion_prior/constant_velocity_prior.h"
#include <cmath>

namespace rosiwit_slam {

void ConstantVelocityPrior::reset(const PoseStamped& pose) {
    m_last_pose = pose;
    m_last_time = pose.time;
    m_vel.setZero();
    m_yaw_rate = 0.0;
    m_have_prev = false;
}

PoseStamped ConstantVelocityPrior::predict(double t) {
    PoseStamped p = m_last_pose;
    p.time = t;
    if (!m_have_prev || m_last_time < 0.0 || t <= m_last_time) return p;
    const double dt = t - m_last_time;
    p.trans = m_last_pose.trans + m_vel * dt;
    p.rot = m_last_pose.rot * Eigen::AngleAxisd(m_yaw_rate * dt, V3D::UnitZ()).toRotationMatrix();
    p.vel = m_vel;
    return p;
}

void ConstantVelocityPrior::onMatch(const PoseStamped& pose) {
    if (m_have_prev && pose.time > m_last_time) {
        const double dt = pose.time - m_last_time;
        if (dt > 1e-3) {
            V3D v = (pose.trans - m_last_pose.trans) / dt;
            if (v.norm() > m_cfg.max_vel) v = v.normalized() * m_cfg.max_vel;
            m_vel = v;
            const M3D dR = pose.rot * m_last_pose.rot.transpose();
            m_yaw_rate = std::atan2(dR(1, 0), dR(0, 0)) / dt;
        }
    }
    m_last_pose = pose;
    m_last_time = pose.time;
    m_have_prev = true;
}

} // namespace rosiwit_slam
