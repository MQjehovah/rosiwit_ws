// src/algorithms/localization/motion_prior/ekf_motion_prior.cpp
#include "algorithms/localization/motion_prior/ekf_motion_prior.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <iostream>

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
inline Eigen::Matrix2d yawRot(double th) {
    const double c = std::cos(th), s = std::sin(th);
    Eigen::Matrix2d R;
    R << c, -s, s, c;
    return R;
}
}   // namespace

void EkfMotionPrior::reset(const PoseStamped& pose) {
    m_pose = pose;
    m_v.setZero();
    m_P = Eigen::Matrix2d::Identity() * 1.0;   // 初值不确定度 1 m/s
    m_odom_rejected = 0;
    m_last_imu_time = -1.0;
    m_first_imu = false;
    m_have_odom = false;
    m_odom_time = -1.0;
    m_last_match_time = pose.time;
    m_last_match_trans = pose.trans;
    m_gyro_bias.setZero();
    m_acc_bias.setZero();
    m_up_body = V3D::UnitZ();
    m_calibrated = false;
    m_cal_gyro.clear();
    m_cal_acc.clear();
}

void EkfMotionPrior::onImu(const IMUSample& s) {
    // ---- L11 静止标定 (与 ImuMotionPrior 同款) ----
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

    // ---- 姿态: 陀螺积分(去零偏) ----
    const V3D angle_inc = (s.gyro - m_gyro_bias) * dt;
    const double an = angle_inc.norm();
    if (an > 1e-12)
        m_pose.rot = m_pose.rot * Eigen::AngleAxisd(an, angle_inc / an).toRotationMatrix();

    // ---- 位置: R(θ)·v·dt ----
    m_pose.trans += m_pose.rot * V3D(m_v.x(), m_v.y(), 0.0) * dt;
    m_pose.vel = m_pose.rot * V3D(m_v.x(), m_v.y(), 0.0);

    // ---- 协方差扩散 (速度随机游走) ----
    m_P += Eigen::Matrix2d::Identity() * (m_cfg.ekf_q_v * dt);

    // ---- 重力姿态校正 (roll/pitch, 不改 yaw; 与 ImuMotionPrior 同款) ----
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
    m_gyro_yaw_rate = s.gyro.z();
}

void EkfMotionPrior::onOdom(const PoseStamped& odom) {
    if (!m_cfg.use_odom) return;   // 开关: 关闭时速度只来自激光位移(纯 IMU+EKF 退化为智能常速)
    if (m_have_odom && odom.time > m_odom_time) {
        const double dt = odom.time - m_odom_time;
        if (dt > 1e-3) {
            // 前置物理门 (硬阈值, 卡方之外的保底):
            //   平移速度超物理上限 = 打滑/毛刺样本, 整包丢弃
            const V3D dp_w = odom.trans - m_odom.trans;
            const double speed = dp_w.head<2>().norm() / dt;
            if (m_cfg.max_odom_speed > 0 && speed > m_cfg.max_odom_speed) {
                ++m_odom_rejected;
                return;
            }
            // 前置偏航一致性门: 直行打滑时 yaw 两边都 ~0 挡不住平移, 但转弯打滑
            // (odom 转、机身不转) 能挡住 —— 便宜的先验信息, 不浪费
            const M3D dR = odom.rot * m_odom.rot.transpose();
            m_odom_yaw_rate = std::atan2(dR(1, 0), dR(0, 0)) / dt;
            if (std::fabs(m_odom_yaw_rate - m_gyro_yaw_rate) > m_cfg.slip_yaw_thresh) {
                ++m_odom_rejected;
                return;
            }
            // EKF 更新: 体系速度观测 z = R(θ)ᵀ·Δp/dt
            const Eigen::Vector2d z =
                yawRot(std::atan2(m_pose.rot(1, 0), m_pose.rot(0, 0))).transpose() *
                dp_w.head<2>() / dt;
            velocityUpdate(z, m_cfg.ekf_r_odom * m_cfg.ekf_r_odom);
        }
    }
    m_odom = odom;
    m_odom_time = odom.time;
    m_have_odom = true;
}

void EkfMotionPrior::velocityUpdate(const Eigen::Vector2d& z, double r_var) {
    // H = I; S = P + R; 卡方门控 (2 自由度, 95% = 5.99)
    const Eigen::Matrix2d S = m_P + Eigen::Matrix2d::Identity() * r_var;
    const Eigen::Vector2d nu = z - m_v;
    const double chi2 = nu.transpose() * S.ldlt().solve(nu);
    if (m_cfg.ekf_chi2 > 0 && chi2 > m_cfg.ekf_chi2) {
        ++m_odom_rejected;
        return;   // 新息与协差不一致: 打滑/外力, 拒绝该观测
    }
    const Eigen::Matrix2d K = m_P * S.inverse();
    m_v += K * nu;
    m_P = (Eigen::Matrix2d::Identity() - K) * m_P;
}

PoseStamped EkfMotionPrior::predict(double t) {
    PoseStamped p = m_pose;
    // 外推到 t: 两次 IMU 之间/之后用当前速度推进 (接口契约是"t 时刻位姿";
    // 2026-10-17 单测抓到旧实现只返回最后积分位姿, 不外推)
    const double dt = t - m_last_imu_time;
    if (dt > 0.0 && dt < 0.5) {
        p.trans += m_pose.rot * V3D(m_v.x(), m_v.y(), 0.0) * dt;
    }
    p.time = t;
    p.vel = m_pose.rot * V3D(m_v.x(), m_v.y(), 0.0);
    return p;
}

void EkfMotionPrior::onMatch(const PoseStamped& pose) {
    // 速度弱观测: 激光位移/间隔 -> 体系速度 (大测量噪声, 慢学习)
    if (m_last_match_time >= 0.0 && pose.time > m_last_match_time) {
        const double dt = pose.time - m_last_match_time;
        if (dt > 1e-3) {
            const V3D dp_w = pose.trans - m_last_match_trans;
            const Eigen::Vector2d z =
                yawRot(std::atan2(m_pose.rot(1, 0), m_pose.rot(0, 0))).transpose() *
                dp_w.head<2>() / dt;
            velocityUpdate(z, m_cfg.ekf_r_match * m_cfg.ekf_r_match);

            // 陀螺零偏在线微调 (航向新息; 与 ImuMotionPrior 同款, 该部分实机验证过)
            const double dtb = std::min(dt, 0.5);
            const M3D dR = m_pose.rot.transpose() * pose.rot;
            Eigen::Quaterniond qe(dR);
            if (qe.w() < 0.0) qe.coeffs() *= -1.0;
            m_gyro_bias -= dtb * m_cfg.gyro_bias_gain * (qe.w() * qe.vec());
            m_gyro_bias = m_gyro_bias.cwiseMax(-m_cfg.gyro_bias_max)
                              .cwiseMin(m_cfg.gyro_bias_max);
        }
    }
    // 锚定绝对位姿 (先验只管帧间预测, 绝对位姿归匹配层)
    m_pose = pose;
    m_pose.vel = m_pose.rot * V3D(m_v.x(), m_v.y(), 0.0);
    m_last_match_time = pose.time;
    m_last_match_trans = pose.trans;
}

}   // namespace rosiwit_slam
