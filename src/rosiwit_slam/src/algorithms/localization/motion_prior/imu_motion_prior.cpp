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
    m_odom_speed_bad = false;
    if (m_have_odom && odom.time > m_odom_time) {
        const double dt = odom.time - m_odom_time;
        if (dt > 1e-3) {
            const M3D dR = odom.rot * m_odom.rot.transpose();
            m_odom_yaw_rate = std::atan2(dR(1, 0), dR(0, 0)) / dt;
            // 平移滑移判据: 轮子打滑/里程毛刺会给出物理上不可能的速度.
            // 以前直接 return 丢弃整包 —— 但这样 m_odom_time 不前进, 下一个样本
            // 的 dt 被拉长, 角速度估计连带失真. 现在只打标记, 样本照常入账,
            // 由 predict() 降权.
            const double v_odom = (odom.trans - m_odom.trans).norm() / dt;
            if (m_cfg.max_odom_speed > 0 && v_odom > m_cfg.max_odom_speed) {
                m_odom_speed_bad = true;
            }
        }
    }
    m_odom = odom;
    m_odom_time = odom.time;
    m_have_odom = true;
}

PoseStamped ImuMotionPrior::predict(double t) {
    PoseStamped p = m_pose;
    p.time = t;
    // ---- 轮速 = 带方差的观测, 不是开关 ----
    // 权重由两个打滑判据连续决定 (原来分别是"超阈值整段不用"和"超上限整包丢弃"):
    //   1) |odom_w - gyro_w| / slip_yaw_thresh 的四次方衰减:
    //      slip=0 -> 1.0, slip=阈值 -> 0.5, slip=2x阈值 -> 0.06
    //   2) 平移速度超过物理上限 -> 再乘 0.1 (不丢弃, 保证 odom 时钟连续)
    // 正常直行 slip≈0 -> 权重≈1, 与旧行为完全一致.
    m_odom_weight = 0.0;
    m_odom_slip = false;
    if (m_cfg.use_odom && m_have_odom && m_have_offset) {
        const double thresh = std::max(1e-3, m_cfg.slip_yaw_thresh);
        const double slip = std::fabs(m_odom_yaw_rate - m_gyro_yaw_rate);
        const double ratio = slip / thresh;
        const double r2 = ratio * ratio;
        double w = 1.0 / (1.0 + r2 * r2);
        m_odom_slip = (slip > thresh);
        if (m_odom_speed_bad) w *= 0.1;
        m_odom_weight = std::max(0.0, std::min(1.0, w));

        const V3D odom_mapped = m_offset.rot * m_odom.trans + m_offset.trans;
        p.trans = (1.0 - m_odom_weight) * p.trans + m_odom_weight * odom_mapped;

        static int probe = 0;
        if (++probe % 50 == 0) {
            std::cout << "[PriorProbe] odom weight=" << m_odom_weight
                      << " slip=" << slip << " thresh=" << thresh
                      << " speed_bad=" << (m_odom_speed_bad ? 1 : 0)
                      << " sigma_xy=" << m_trans_sigma << std::endl;
        }
    } else {
        m_odom_weight = 0.0;
    }
    // 等效不确定度: 权重越低越不确定 (同时给下游一个可用的方差).
    m_trans_sigma = 0.02 + 0.60 * (1.0 - m_odom_weight);
    m_yaw_sigma = 0.01 + 0.30 * (1.0 - m_odom_weight);
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
