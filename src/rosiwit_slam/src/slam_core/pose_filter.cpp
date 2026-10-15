// src/slam_core/pose_filter.cpp
#include "slam_core/pose_filter.h"
#include <cmath>
#include <algorithm>

namespace rosiwit_slam {

void PoseOutputFilter::update(double t, const PoseStamped& meas) {
    if (!m_init) {
        m_pos = meas.trans;
        m_q = Eigen::Quaterniond(meas.rot).normalized();
        m_vel.setZero();
        m_w.setZero();
        m_t = t;
        m_init = true;
        return;
    }
    const double dt = t - m_t;
    if (dt <= 1e-6) {
        // 时间未前进: 仅做位置/朝向修正
        const V3D e = meas.trans - m_pos;
        if (m_alpha > 0) m_pos += m_alpha * e;
        Eigen::Quaterniond qm(meas.rot);
        if (m_alpha > 0) m_q = m_q.slerp(m_alpha, qm).normalized();
        return;
    }

    // 预测 (常速)
    m_pos += m_vel * dt;
    const double wn = m_w.norm();
    if (wn > 1e-9) m_q = (m_q * Eigen::AngleAxisd(wn * dt, m_w / wn)).normalized();

    // 位置修正 (alpha-beta)
    const V3D e = meas.trans - m_pos;
    m_pos += m_alpha * e;
    m_vel += (m_beta / dt) * e;

    // 姿态修正 (轴向 alpha + 角速度 beta)
    Eigen::Quaterniond qm(meas.rot);
    Eigen::Quaterniond qe = m_q.conjugate() * qm;
    if (qe.w() < 0.0) qe.coeffs() *= -1.0;
    const double ang = 2.0 * std::atan2(qe.vec().norm(), qe.w());
    const V3D axis = qe.vec().norm() > 1e-9 ? qe.vec().normalized() : V3D::Zero();
    if (m_alpha > 0 && ang > 1e-12)
        m_q = (m_q * Eigen::AngleAxisd(m_alpha * ang, axis)).normalized();
    m_w += (m_beta * ang / dt) * axis;

    m_t = t;
}

bool PoseOutputFilter::poseAt(double t, PoseStamped& out) const {
    if (!m_init) return false;
    double dt = t - m_t;
    if (dt < 0.0) dt = 0.0;
    if (dt > m_max_extrap) dt = m_max_extrap;
    out.time = t;
    out.trans = m_pos + m_vel * dt;
    const double wn = m_w.norm();
    Eigen::Quaterniond q = m_q;
    if (wn > 1e-9) q = (q * Eigen::AngleAxisd(wn * dt, m_w / wn)).normalized();
    out.rot = q.toRotationMatrix();
    out.vel = m_vel;
    return true;
}

} // namespace rosiwit_slam
