// src/algorithms/localization/fusion/ekf_pose_fusion.cpp
#include "algorithms/localization/fusion/ekf_pose_fusion.h"
#include <algorithm>
#include <cmath>

namespace rosiwit_slam {

namespace {
inline double normAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
inline Eigen::Vector3d rotToRpy(const Eigen::Matrix3d& R) {
    Eigen::Vector3d rpy;
    rpy.x() = std::atan2(R(2, 1), R(2, 2));
    rpy.y() = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
    rpy.z() = std::atan2(R(1, 0), R(0, 0));
    return rpy;
}
inline Eigen::Matrix3d rpyToRot(const Eigen::Vector3d& rpy) {
    const double cr = std::cos(rpy.x()), sr = std::sin(rpy.x());
    const double cp = std::cos(rpy.y()), sp = std::sin(rpy.y());
    const double cy = std::cos(rpy.z()), sy = std::sin(rpy.z());
    Eigen::Matrix3d R;
    R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
         sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
         -sp,     cp * sr,                cp * cr;
    return R;
}
inline Eigen::Vector3d rotLog(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R);
    return aa.angle() * aa.axis();
}
}  // namespace

void EkfPoseFusion::reset(const PoseStamped& pose) {
    m_p = pose.trans;
    m_R = pose.rot;
    m_theta = rotToRpy(pose.rot);
    m_v.setZero();
    m_v_cov = Eigen::Matrix2d::Identity();
    m_P.setConstant(0.05);
    m_time = pose.time;
    m_valid = true;
    m_have_imu = false;
    m_last_imu_time = -1.0;
    m_gyro_z = 0.0;
    m_have_odom = false;
    m_last_odom_time = -1.0;
    m_have_last_match = false;
    m_odom_yaw = rotToRpy(pose.rot).z();
    m_diag = FusionDiag{};
}

void EkfPoseFusion::onImu(const IMUSample& s) {
    if (!m_valid) return;
    if (!m_have_imu) {
        m_last_imu_time = s.time;
        m_have_imu = true;
        m_gyro_z = s.gyro.z();
        return;
    }
    const double dt = s.time - m_last_imu_time;
    if (dt <= 0.0 || dt > 0.2) {
        m_last_imu_time = s.time;
        m_gyro_z = s.gyro.z();
        return;
    }
    // 姿态: 只积分 yaw; roll/pitch 由匹配与重力约束保持 (v1 简化, 见设计文档)
    m_theta.z() = normAngle(m_theta.z() + (s.gyro.z() - m_gyro_bias_z) * dt);
    m_R = rpyToRot(m_theta);
    // 位置: 体体系速度 -> 世界系
    m_p += m_R * Eigen::Vector3d(m_v.x(), m_v.y(), 0.0) * dt;
    // 协方差扩散
    m_P.head<3>().array() += m_cfg.process_rot * dt;
    m_P.tail<3>().array() += m_cfg.process_trans * dt;
    m_v_cov += Eigen::Matrix2d::Identity() * (0.5 * dt);
    m_last_imu_time = s.time;
    m_time = s.time;
    m_gyro_z = s.gyro.z();
}

void EkfPoseFusion::onOdom(const PoseStamped& odom) {
    if (!m_valid) return;
    if (!m_have_odom) {
        m_last_odom = odom;
        m_last_odom_time = odom.time;
        m_odom_yaw = rotToRpy(odom.rot).z();
        m_have_odom = true;
        return;
    }
    const double dt = odom.time - m_last_odom_time;
    if (dt <= 1e-3) return;
    const V3D dp = odom.trans - m_last_odom.trans;
    const double speed = dp.head<2>().norm() / dt;
    if (m_cfg.max_odom_speed > 0.0 && speed > m_cfg.max_odom_speed) {
        m_last_odom = odom;
        m_last_odom_time = odom.time;
        return;   // 打滑/毛刺样本, 整包丢弃 (与 imu_ekf 先验一致)
    }
    // 体体系速度观测: 用 odom 自己的朝向转 (2026-10-31 已验证 z-v 夹角≈0.7°)
    const double c = std::cos(m_odom_yaw), s = std::sin(m_odom_yaw);
    Eigen::Matrix2d R_odom_T;
    R_odom_T << c, s, -s, c;
    const Eigen::Vector2d z = R_odom_T * dp.head<2>() / dt;
    const double r = 0.15 * 0.15;
    for (int i = 0; i < 2; ++i) {
        const double k = m_v_cov(i, i) / (m_v_cov(i, i) + r);
        m_v(i) += k * (z(i) - m_v(i));
        m_v_cov(i, i) = (1.0 - k) * m_v_cov(i, i);
    }
    m_last_odom = odom;
    m_last_odom_time = odom.time;
    m_odom_yaw = rotToRpy(odom.rot).z();
}

void EkfPoseFusion::onMatch(const PoseStamped& pose,
                            const Eigen::Matrix<double, 6, 6>& cov,
                            const MatchResult& /*res*/) {
    if (!m_valid) {
        reset(pose);
        return;
    }
    // ---- 新息 ----
    const Eigen::Vector3d e_rot = rotLog(pose.rot * m_R.transpose());
    const Eigen::Vector3d e_trans = pose.trans - m_p;
    // ---- 观测噪声 (rot 在前, trans 在后) + 下限 ----
    Eigen::Matrix<double, 6, 1> Rdiag;
    for (int i = 0; i < 3; ++i) {
        Rdiag(i) = std::max(cov(i, i), m_cfg.rot_cov_floor);
        Rdiag(3 + i) = std::max(cov(3 + i, 3 + i), m_cfg.trans_cov_floor);
    }
    // ---- 卡方门控 ----
    double chi2 = 0.0;
    for (int i = 0; i < 3; ++i) chi2 += e_rot(i) * e_rot(i) / (m_P(i) + Rdiag(i));
    for (int i = 0; i < 3; ++i) chi2 += e_trans(i) * e_trans(i) / (m_P(3 + i) + Rdiag(3 + i));
    m_diag.last_chi2 = chi2;
    if (m_cfg.chi2_gate > 0.0 && chi2 > m_cfg.chi2_gate) {
        ++m_diag.rejected_chi2;
        m_P.tail<3>().array() += 0.02;   // 拒收后放宽, 让下一帧更容易通过
        m_P.head<3>().array() += 0.005;
        m_time = pose.time;
        return;
    }
    // ---- 原地旋转: 丢弃平移分量 (旋转时车体位置不应变化) ----
    bool in_place = false;
    if (m_cfg.reject_in_place_trans) {
        const double yaw_now = rotToRpy(pose.rot).z();
        const double yaw_step = m_have_last_match
            ? std::fabs(normAngle(yaw_now - m_last_match_yaw)) : 0.0;
        const bool nearly_static = m_v.norm() < 0.05;
        if (nearly_static && yaw_step > m_cfg.in_place_yaw_step_deg * M_PI / 180.0) {
            in_place = true;
        }
        m_last_match_yaw = yaw_now;
        m_have_last_match = true;
    }
    // ---- 增益 (协方差决定, 上限 gain_*) ----
    Eigen::Matrix<double, 6, 1> gain;
    for (int i = 0; i < 3; ++i) {
        gain(i) = std::min(m_cfg.gain_rot, m_P(i) / (m_P(i) + Rdiag(i)));
    }
    for (int i = 0; i < 3; ++i) {
        gain(3 + i) = std::min(m_cfg.gain_trans, m_P(3 + i) / (m_P(3 + i) + Rdiag(3 + i)));
    }
    Eigen::Vector3d d_rot = gain.head<3>().asDiagonal() * e_rot;
    Eigen::Vector3d d_trans = gain.tail<3>().asDiagonal() * e_trans;
    // ---- 单帧修正限幅 (核心: 一次错配不可能瞬移) ----
    const double max_rot = m_cfg.max_correction_rot_deg * M_PI / 180.0;
    if (d_rot.norm() > max_rot && d_rot.norm() > 1e-12) {
        d_rot *= max_rot / d_rot.norm();
    }
    if (d_trans.norm() > m_cfg.max_correction_trans && d_trans.norm() > 1e-12) {
        d_trans *= m_cfg.max_correction_trans / d_trans.norm();
    }
    if (in_place) {
        d_trans.setZero();
        ++m_diag.rejected_in_place;
    }
    // ---- 应用 ----
    m_p += d_trans;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    if (d_rot.norm() > 1e-12) axis = d_rot / d_rot.norm();
    const Eigen::Matrix3d dR = Eigen::AngleAxisd(d_rot.norm(), axis).toRotationMatrix();
    m_R = dR * m_R;
    m_R = Eigen::Quaterniond(m_R).normalized().toRotationMatrix();
    m_theta = rotToRpy(m_R);
    // ---- 协方差更新 ----
    for (int i = 0; i < 6; ++i) {
        m_P(i) = (1.0 - gain(i)) * m_P(i);
        m_P(i) += (i < 3 ? m_cfg.rot_cov_floor : m_cfg.trans_cov_floor);
    }
    m_time = pose.time;
    ++m_diag.accepted;
    m_diag.last_gain_trans = gain(3);
    m_diag.last_gain_rot = gain(0);
    m_diag.applied_trans = d_trans.norm();
    m_diag.applied_rot_deg = d_rot.norm() * 180.0 / M_PI;
}

PoseStamped EkfPoseFusion::predict(double t) const {
    PoseStamped out;
    out.time = t;
    out.trans = m_p;
    out.rot = m_R;
    out.vel = m_R * Eigen::Vector3d(m_v.x(), m_v.y(), 0.0);
    const double dt = (m_last_imu_time > 0.0) ? (t - m_last_imu_time) : 0.0;
    if (dt > 0.0 && dt < 0.5) {
        out.trans += m_R * Eigen::Vector3d(m_v.x(), m_v.y(), 0.0) * dt;
    }
    return out;
}

bool EkfPoseFusion::getCov(Eigen::Matrix<double, 6, 6>& out) const {
    if (!m_valid) return false;
    out.setZero();
    for (int i = 0; i < 6; ++i) out(i, i) = m_P(i);
    return true;
}

}  // namespace rosiwit_slam
