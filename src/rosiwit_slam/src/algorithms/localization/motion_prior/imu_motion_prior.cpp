// src/algorithms/localization/motion_prior/imu_motion_prior.cpp
#include "algorithms/localization/motion_prior/imu_motion_prior.h"
#include <cmath>
#include <numeric>
#include <algorithm>

namespace rosiwit_slam {

namespace {
inline V3D meanOf(const std::deque<V3D>& d) {
    V3D s = V3D::Zero();
    for (const auto& v : d) s += v;
    return d.empty() ? s : s / static_cast<double>(d.size());
}
inline V3D stdOf(const std::deque<V3D>& d, const V3D& m) {
    V3D s = V3D::Zero();
    for (const auto& v : d) s += (v - m).cwiseProduct(v - m);
    return d.empty() ? s : (s / static_cast<double>(d.size())).cwiseSqrt();
}
} // namespace

void ImuMotionPrior::reset(const PoseStamped& pose) {
    m_pose = pose;
    m_last_imu_time = -1.0;
    m_first_imu = false;
    m_vel_world.setZero();
    m_last_match_time = pose.time;
    m_last_match_trans = pose.trans;
    m_have_offset = false;
    m_gyro_bias.setZero();
    m_acc_bias.setZero();
    m_up_body = V3D::UnitZ();
    m_calibrated = false;
    m_cal_gyro.clear();
    m_cal_acc.clear();
}

void ImuMotionPrior::onImu(const IMUSample& s) {
    // ---- L11 静止标定 (未标定时尝试; 静止成立才写零偏, 否则窗口滑动跳过) ----
    if (!m_calibrated && m_cfg.imu_calib_samples > 0) {
        m_cal_gyro.push_back(s.gyro);
        m_cal_acc.push_back(s.acc);
        if (static_cast<int>(m_cal_gyro.size()) > m_cfg.imu_calib_samples) {
            m_cal_gyro.pop_front();
            m_cal_acc.pop_front();
        }
        if (static_cast<int>(m_cal_gyro.size()) == m_cfg.imu_calib_samples) {
            const V3D mg = meanOf(m_cal_gyro);
            const V3D ma = meanOf(m_cal_acc);
            const V3D sg = stdOf(m_cal_gyro, mg);
            const V3D sa = stdOf(m_cal_acc, ma);
            const bool is_static = (sg.norm() < m_cfg.static_gyro_std) &&
                                   (sa.norm() < m_cfg.static_accel_std) &&
                                   (std::fabs(ma.norm() - m_cfg.gravity) < m_cfg.static_accel_std);
            if (is_static) {
                m_gyro_bias = mg;
                m_up_body = ma.normalized();
                m_calibrated = true;
            } else {
                // 未静止: 滑动一格, 下次继续判 (跳过标定)
                m_cal_gyro.pop_front();
                m_cal_acc.pop_front();
            }
        }
    }

    if (!m_first_imu) {
        m_last_imu_time = s.time;
        m_first_imu = true;
        return;
    }
    const double dt = s.time - m_last_imu_time;
    if (dt <= 0.0 || dt > m_cfg.max_dt) { m_last_imu_time = s.time; return; }

    // ---- 角速度积分 (去零偏) ----
    const V3D angle_inc = (s.gyro - m_gyro_bias) * dt;
    const double an = angle_inc.norm();
    if (an > 1e-12)
        m_pose.rot = m_pose.rot * Eigen::AngleAxisd(an, angle_inc / an).toRotationMatrix();

    // ---- 位置: 常速推进 ----
    m_pose.trans += m_vel_world * dt;
    m_pose.vel = m_vel_world;

    // ---- 重力姿态校正 (roll/pitch, 不改 yaw) ----
    if (m_cfg.use_gravity && m_calibrated) {
        const V3D a = s.acc - m_acc_bias;
        const double anorm = a.norm();
        if (anorm > 1e-3 && std::fabs(anorm - m_cfg.gravity) < m_cfg.static_accel_std) {
            const V3D body_up = a / anorm;
            const V3D up_world = m_pose.rot * body_up;
            if (up_world.z() > 0.1) {
                Eigen::Quaterniond qa;
                qa.setFromTwoVectors(up_world, V3D::UnitZ());
                const Eigen::Quaterniond q_cur(m_pose.rot);
                const Eigen::Quaterniond q_des(qa.toRotationMatrix() * m_pose.rot);
                m_pose.rot = q_cur.slerp(m_cfg.gravity_gain, q_des).normalized().toRotationMatrix();
            }
        }
    }

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

            // L11 零偏在线估计 (dt 限幅: 稀疏匹配时避免过激更新)
            const double dtb = std::min(dt, 0.5);
            const V3D err = pose.trans - m_pose.trans;
            const V3D err_body = m_pose.rot.transpose() * err;
            m_acc_bias -= dtb * m_cfg.acc_bias_gain * err_body;
            m_acc_bias = m_acc_bias.cwiseMax(-m_cfg.acc_bias_max).cwiseMin(m_cfg.acc_bias_max);

            const M3D dR = m_pose.rot.transpose() * pose.rot;  // qe = q_pred^{-1} ⊗ q_meas
            Eigen::Quaterniond qe(dR);
            if (qe.w() < 0.0) qe.coeffs() *= -1.0;
            m_gyro_bias -= dtb * m_cfg.gyro_bias_gain * (qe.w() * qe.vec());
            m_gyro_bias = m_gyro_bias.cwiseMax(-m_cfg.gyro_bias_max).cwiseMin(m_cfg.gyro_bias_max);
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
