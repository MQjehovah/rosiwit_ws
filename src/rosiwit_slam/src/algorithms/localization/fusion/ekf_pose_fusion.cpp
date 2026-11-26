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
/// 位移比 -> 连续权重: ratio 落在 [lo, hi] 内=1, 越界线性衰减到 odom_weight_min。
/// |d_odom| 小于 1e-3 时视为"没动", ratio 取 1 (除零保护)。
inline double slipWeight(double d_pose, double d_odom, const PoseFusionConfig& cfg) {
    double ratio = 1.0;
    if (d_odom > 1e-3) ratio = d_pose / d_odom;
    if (!std::isfinite(ratio)) return cfg.odom_weight_min;
    const double lo = cfg.slip_ratio_lo;
    const double hi = cfg.slip_ratio_hi;
    if (ratio >= lo && ratio <= hi) return 1.0;
    const double dev = (ratio < lo) ? (lo - ratio) / std::max(lo, 1e-3)
                                    : (ratio - hi) / std::max(hi, 1e-3);
    return std::max(cfg.odom_weight_min, 1.0 - std::min(1.0, dev));
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
    m_anchor_yaw_next = true;   // 注入/重初始化后: 第一帧高匹配可直接锚定 yaw
    m_have_imu = false;
    m_last_imu_time = -1.0;
    m_gyro_z = 0.0;
    m_have_odom = false;
    m_last_odom_time = -1.0;
    m_have_last_match = false;
    m_lv_have_prev = false;
    m_last_match_time = -1.0;
    m_odom_yaw = rotToRpy(pose.rot).z();
    m_have_rel_prev = false;
    m_rel_prev_state_R = Eigen::Matrix3d::Identity();
    m_have_slip_ref = false;
    m_have_prev_match = false;
    m_odom_slip_weight = 1.0;
    m_diag = FusionDiag{};
    m_hist.clear();
    {
        PoseStamped h;
        h.time = pose.time;
        h.trans = m_p;
        h.rot = m_R;
        m_hist.emplace_back(pose.time, h);
    }
}

void EkfPoseFusion::resetRelative() {
    m_have_rel_prev = false;
    m_rel_prev_state_R = m_R;
}

void EkfPoseFusion::onImu(const IMUSample& s) {
    if (!m_valid) return;
    // 机体 up 方向 (IMU 系): IMU 可能随雷达俯装 (TF/URDF/参数里定义), gyro.z 不是偏航角速度.
    // 偏航角速度必须投影: yaw_rate = ω·up_body (统一走 slam_types.h: imuYawRate, 不允许写死 gyro.z).
    // 首次直接取加速度方向; 之后极慢跟踪, 只跟安装漂移, 不跟运动加速度.
    V3D up;
    if (m_cfg.up_body_valid) {
        // 参数优先: 安装姿态来自 TF/URDF/启动参数, 由定位管线解析后写进 config.
        m_up_body = m_cfg.up_body;
        m_up_valid = true;
    } else if (accelUpBody(s.acc, m_cfg.gravity, m_cfg.gravity_tol, up)) {
        if (!m_up_valid) { m_up_body = up; m_up_valid = true; }
        else m_up_body = (0.9999 * m_up_body + 0.0001 * up).normalized();
    }
    if (!m_up_valid) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            std::cout << "[EkfPoseFusion] WARN up_body unknown (无安装外参且加速度不可用), "
                         "暂时按 IMU z 轴积分" << std::endl;
        }
    }
    const double yaw_rate = imuYawRate(s.gyro, m_up_body);

    if (!m_have_imu) {
        m_last_imu_time = s.time;
        m_have_imu = true;
        m_gyro_z = yaw_rate;
        return;
    }
    const double dt = s.time - m_last_imu_time;
    if (dt <= 0.0 || dt > 0.2) {
        m_last_imu_time = s.time;
        m_gyro_z = yaw_rate;
        return;
    }
    // 姿态: 只积分 yaw; roll/pitch 由匹配与重力约束保持 (v1 简化, 见设计文档)
    m_theta.z() = normAngle(m_theta.z() + (yaw_rate - m_gyro_bias_z) * dt);
    m_diag_yaw_int += yaw_rate * dt;
    {
        static int dn = 0;
        if (++dn % 400 == 0) {   // 2s @200Hz
            std::cout << "[ImuDiag] fusion_yaw=" << m_theta.z() * 57.2957795
                      << " proj_int=" << m_diag_yaw_int * 57.2957795
                      << " gz=" << s.gyro.z() << " yaw_rate=" << yaw_rate
                      << " up=(" << m_up_body.x() << "," << m_up_body.y() << "," << m_up_body.z() << ")"
                      << std::endl;
        }
    }
    m_R = rpyToRot(m_theta);
    // 位置: ṗ_lidar = R·(R_blᵀ·v_base + ω_s×ρ) —— 与 EkfMotionPrior 同款杆臂轨道项
    // (slam_types.h: leverArmSensorFrame). 缺它时原地转 180° 位置误差 ≈0.83m.
    m_w_s = s.gyro;
    {
        const Eigen::Vector3d v_sensor = m_cfg.lever_valid
            ? m_cfg.r_base_lidar.transpose() * Eigen::Vector3d(m_v.x(), m_v.y(), 0.0)
            : Eigen::Vector3d(m_v.x(), m_v.y(), 0.0);
        const Eigen::Vector3d orbit = m_cfg.lever_valid
            ? m_w_s.cross(m_cfg.lever_arm) : Eigen::Vector3d::Zero();
        m_p += m_R * (v_sensor + orbit) * dt;
    }
    // 协方差扩散
    m_P.head<3>().array() += m_cfg.process_rot * dt;
    m_P.tail<3>().array() += m_cfg.process_trans * dt;
    m_v_cov += Eigen::Matrix2d::Identity() * (0.5 * dt);
    m_last_imu_time = s.time;
    m_time = s.time;
    {
        PoseStamped h;
        h.time = s.time;
        h.trans = m_p;
        h.rot = m_R;
        m_hist.emplace_back(s.time, h);
        while (m_hist.size() > 600) m_hist.pop_front();
    }
    m_gyro_z = yaw_rate;
}

void EkfPoseFusion::onOdom(const PoseStamped& odom) {
    if (!m_valid) return;
    if (!m_have_odom) {
        m_last_odom = odom;
        m_last_odom_time = odom.time;
        m_odom_yaw = rotToRpy(odom.rot).z();
        m_have_odom = true;
        if (!m_have_slip_ref) {
            m_slip_ref_odom = odom;
            m_slip_ref_p = m_p;
            m_have_slip_ref = true;
        }
        m_diag.odom_weight = 1.0;
        return;
    }
    const double dt = odom.time - m_last_odom_time;
    if (dt <= 1e-3) return;
    const V3D dp = odom.trans - m_last_odom.trans;
    const double speed = dp.head<2>().norm() / dt;
    if (m_cfg.max_odom_speed > 0.0 && speed > m_cfg.max_odom_speed) {
        m_last_odom = odom;
        m_last_odom_time = odom.time;
        m_diag.odom_weight = 0.0;   // 未参与融合
        return;   // 打滑/毛刺样本, 整包丢弃 (与 imu_ekf 先验一致)
    }
    // ---- 打滑门控: 两路独立对照, 取更保守者 ----
    // 1) 匹配区间对照 (10Hz 更新): 两次 S2M 匹配的位移 vs 同区间 odom 位移。
    //    这条不依赖滤波器自己的速度, 持续打滑也能看出来。
    // 2) plan 语义: 最近一次被接受的匹配位姿 -> 当前融合位置 vs 同区间 odom。
    // 打滑是常态: odom 不删除, 只把权重从 1 线性压到 odom_weight_min。
    double weight = m_odom_slip_weight;
    if (m_have_imu && m_have_slip_ref) {
        // 没有 IMU 时融合位置不会推进, 用"静止的 m_p"对照 odom 只会产生假打滑,
        // 所以这条只在 IMU 真正驱动过位置后启用。
        const double d_odom = (odom.trans - m_slip_ref_odom.trans).head<2>().norm();
        const double d_pose = (m_p - m_slip_ref_p).head<2>().norm();
        weight = std::min(weight, slipWeight(d_pose, d_odom, m_cfg));
    }
    // 横摆率一致性: 打滑时轮速差速给出的转角与陀螺不一致
    if (m_have_imu && m_have_slip_ref && m_cfg.slip_yaw_thresh_deg > 0.0) {
        const double yaw_rate =
            normAngle(rotToRpy(odom.rot).z() - m_odom_yaw) / dt;
        const double thresh = m_cfg.slip_yaw_thresh_deg * M_PI / 180.0;
        if (std::fabs(yaw_rate - m_gyro_z) > thresh) {
            weight = std::max(m_cfg.odom_weight_min, weight * 0.2);
        }
    }
    weight = std::max(m_cfg.odom_weight_min, std::min(1.0, weight));
    m_diag.odom_weight = weight;

    // 体系速度观测 (2026-11-23 修正, 与 EkfMotionPrior::onOdom 同款): ROS 层送来的 odom 是
    // IMU/雷达体在 odom 系的位姿. 有安装外参时换算到 base: 减掉雷达绕 base 的轨道位移,
    // 用 base 在 odom 系的朝向转回体系. (以前按雷达体朝向转, 得到雷达体系速度, 却被
    // 当 base 系速度用于 R_blᵀ·v —— 安装偏航 89.9° 下预测速度指向车尾偏左.)
    Eigen::Vector2d z;
    if (m_cfg.lever_valid) {
        const M3D R_ob_prev = m_last_odom.rot * m_cfg.r_base_lidar.transpose();   // base -> odom
        const M3D R_ob = odom.rot * m_cfg.r_base_lidar.transpose();
        const V3D t_bl = m_cfg.r_base_lidar * m_cfg.lever_arm;                    // 雷达原点 (base 系)
        const V3D dp_b = dp - (R_ob - R_ob_prev) * t_bl;                         // base 原点位移
        const double psi = std::atan2(R_ob_prev(1, 0), R_ob_prev(0, 0));
        const double c = std::cos(psi), s = std::sin(psi);
        z = Eigen::Vector2d(c * dp_b.x() + s * dp_b.y(), -s * dp_b.x() + c * dp_b.y()) / dt;
    } else {
        const double c = std::cos(m_odom_yaw), s = std::sin(m_odom_yaw);
        z = Eigen::Vector2d(c * dp.x() + s * dp.y(), -s * dp.x() + c * dp.y()) / dt;
    }
    // 权重 w 等价于把观测噪声放大 1/w²
    const double r = 0.15 * 0.15 / std::max(weight * weight, 1e-6);
    for (int i = 0; i < 2; ++i) {
        const double k = m_v_cov(i, i) / (m_v_cov(i, i) + r);
        if (!std::isfinite(k)) continue;
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
    // ---- 延迟观测: 先把状态退回到匹配时刻 t_a (2026-11-23) ----
    // IMU 从回调线程实时积分, 匹配到达时状态已在 now = 最新 IMU 时刻, 晚于帧末
    // t_a (处理延迟 0.1~0.4s). 旧实现直接拿 pose(t_a) 与 now 的状态比, 转弯时
    // 新息里混着 ω·(now - t_a) (回放 ~10°), 被当成误差往回拉 -> 融合 yaw 恒落后
    // IMU 积分 ([ImuDiag] 实测少 35°), 发布位姿的 yaw 跟不上转弯.
    // 相对运动与锚点无关: 退回 t_a -> 算新息/修正 -> 把 (t_a, now] 的相对运动接回.
    PoseStamped S_a;                                  // t_a 时刻的滤波器状态
    Eigen::Matrix3d dR_rel = Eigen::Matrix3d::Identity();   // t_a -> now 相对旋转
    Eigen::Vector3d dp_rel = Eigen::Vector3d::Zero();       // t_a -> now 相对位移 (t_a 系)
    const bool delayed = histAt(pose.time, S_a);
    if (delayed) {
        dR_rel = S_a.rot.transpose() * m_R;
        dp_rel = S_a.rot.transpose() * (m_p - S_a.trans);
    } else {
        S_a.trans = m_p;
        S_a.rot = m_R;
    }
    // ---- 新息 (与匹配同时刻) ----
    const Eigen::Vector3d e_rot = rotLog(pose.rot * S_a.rot.transpose());
    const Eigen::Vector3d e_trans = pose.trans - S_a.trans;
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
    // yaw: 注入后的第一帧高匹配全量采纳 (地图一致解), 之后互补滤波小增益。
    // 同一位置: 第一帧是在"无地图一致估计"下的观测 (重定位语义), 且不构成
    // 反馈回路; 之后的帧才受"谷底自由 + 恒定侧偏"棘轮病理影响。
    if (m_anchor_yaw_next) {
        m_anchor_yaw_next = false;   // 只此一次
    } else {
        d_rot.z() = m_cfg.yaw_gain * e_rot.z();
    }

    Eigen::Vector3d d_trans = gain.tail<3>().asDiagonal() * e_trans;
    // ---- 单帧修正限幅 (核心: 一次错配不可能瞬移) ----
    const double max_rot = m_cfg.max_correction_rot_deg * M_PI / 180.0;
    if (d_rot.norm() > max_rot && d_rot.norm() > 1e-12) {
        d_rot *= max_rot / d_rot.norm();
    }
    // 平移限幅随 "距上次匹配的时间内按当前速度能走的距离" 放宽 (2026-11-23): 固定
    // 0.15 m/帧的限幅在持续运动下等价于给输出加一个稳态滞后 (滞后 ≈ 限幅/增益),
    // 越快越落后 —— 实车推车 0.6 m/s 时发布位姿落后匹配 0.5~1.4 m.
    const double dtm = (m_last_match_time > 0.0 && pose.time > m_last_match_time)
                           ? std::min(pose.time - m_last_match_time, 1.0) : 0.0;
    const double max_trans_eff = m_cfg.max_correction_trans + m_v.norm() * dtm;
    if (d_trans.norm() > max_trans_eff && d_trans.norm() > 1e-12) {
        d_trans *= max_trans_eff / d_trans.norm();
    }
    if (in_place) {
        d_trans.setZero();
        ++m_diag.rejected_in_place;
    }
    // ---- 应用: 先修到 t_a 的状态, 再把 (t_a, now] 的相对运动接回来 ----
    const Eigen::Vector3d p_fix = S_a.trans + d_trans;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    if (d_rot.norm() > 1e-12) axis = d_rot / d_rot.norm();
    const Eigen::Matrix3d dR = Eigen::AngleAxisd(d_rot.norm(), axis).toRotationMatrix();
    const Eigen::Matrix3d R_fix = dR * S_a.rot;
    if (delayed) {
        m_p = p_fix + R_fix * dp_rel;
        m_R = R_fix * dR_rel;
    } else {
        m_p = p_fix;
        m_R = R_fix;
    }
    m_R = Eigen::Quaterniond(m_R).normalized().toRotationMatrix();
    m_theta = rotToRpy(m_R);
    // 历史尾部同步重锚 (下一次延迟观测要用修正后的链)
    if (delayed) {
        for (auto& e : m_hist) {
            if (e.first <= pose.time) continue;
            const Eigen::Vector3d rel_p = S_a.rot.transpose() * (e.second.trans - S_a.trans);
            const Eigen::Matrix3d rel_R = S_a.rot.transpose() * e.second.rot;
            e.second.trans = p_fix + R_fix * rel_p;
            e.second.rot = R_fix * rel_R;
        }
        while (!m_hist.empty() && m_hist.front().first < pose.time) m_hist.pop_front();
        // 在缓冲起点补一条"匹配时刻、修正后"的状态: predict(t_a) 必须精确回到 t_a.
        // 否则 t_a 落在起点之前, predict 只能回退到最新状态 = now, 转弯时位姿超前
        // 一个处理延迟 (墙歪的根因).
        PoseStamped at_a;
        at_a.time = pose.time;
        at_a.trans = p_fix;
        at_a.rot = R_fix;
        if (!m_hist.empty() && m_hist.front().first == pose.time) {
            m_hist.front().second = at_a;
        } else {
            m_hist.push_front(std::make_pair(pose.time, at_a));
        }
    }
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
    // ---- 激光位移速度观测 (2026-11-23): 相邻两次被接受匹配的位移 -> 体系速度 ----
    // odom_mode=none 时这是融合层**唯一**的速度来源: 没有它, 位置只能靠限幅修正挪动
    // (0.15 m/帧 ≈ 0.75 m/s), 1~2 m/s 的推行必然落后, 再被马氏门限拒掉 (实车复现).
    // 与 EkfMotionPrior::onMatch 同款: 去杆臂 (雷达绕 base 划弧不是平移), 用 base yaw
    // 转回体系. 原地旋转时杆臂误差会伪装成速度, 跳过. 有 odom 时它的权重自然较低.
    if (m_cfg.match_vel_r > 0.0) {
        if (m_lv_have_prev && !in_place && pose.time > m_lv_prev_pose.time &&
            pose.time - m_lv_prev_pose.time < 1.0) {
            const double dtv = pose.time - m_lv_prev_pose.time;
            V3D dpw = pose.trans - m_lv_prev_pose.trans;
            if (m_cfg.lever_valid)
                dpw -= pose.rot * m_cfg.lever_arm - m_lv_prev_pose.rot * m_cfg.lever_arm;
            const M3D R_mb = m_cfg.lever_valid ? pose.rot * m_cfg.r_base_lidar.transpose()
                                               : pose.rot;
            const double cy = std::atan2(R_mb(1, 0), R_mb(0, 0));
            const double c = std::cos(cy), s = std::sin(cy);
            const Eigen::Vector2d z((c * dpw.x() + s * dpw.y()) / dtv,
                                    (-s * dpw.x() + c * dpw.y()) / dtv);
            const double r = m_cfg.match_vel_r * m_cfg.match_vel_r;
            if (z.allFinite()) {
                for (int i = 0; i < 2; ++i) {
                    const double k = m_v_cov(i, i) / (m_v_cov(i, i) + r);
                    m_v(i) += k * (z(i) - m_v(i));
                    m_v_cov(i, i) = (1.0 - k) * m_v_cov(i, i);
                }
            }
        }
        m_lv_have_prev = true;
        m_lv_prev_pose = pose;
    }
    m_last_match_time = pose.time;
    // ---- 打滑门控锚点 (只有被接受的匹配才更新) ----
    if (m_have_odom) {
        // 独立对照: 两次匹配之间的真实位移 vs 同区间 odom 位移
        if (m_have_prev_match) {
            const double d_match = (pose.trans - m_prev_match_pose.trans).head<2>().norm();
            const double d_odom =
                (m_last_odom.trans - m_prev_match_odom.trans).head<2>().norm();
            m_odom_slip_weight = slipWeight(d_match, d_odom, m_cfg);
        }
        m_have_prev_match = true;
        m_prev_match_pose = pose;
        m_prev_match_odom = m_last_odom;
        // plan 语义锚点: 当前匹配位姿 vs 融合位置
        m_slip_ref_odom = m_last_odom;
        m_slip_ref_p = m_p;
        m_have_slip_ref = true;
    }
}

void EkfPoseFusion::onRelative(const PoseStamped& pose,
                               const Eigen::Matrix<double, 6, 6>& cov) {
    if (!m_valid) return;
    auto anchor = [&]() {
        m_rel_prev_lio = pose;
        m_rel_prev_state_p = m_p;
        m_rel_prev_state_R = m_R;
        m_have_rel_prev = true;
    };
    if (!m_have_rel_prev) {
        // 首帧只做锚定: 相对观测没有绝对信息, 需要一个基准才能比较增量
        anchor();
        return;
    }
    // 连续性保护: LIO 复位/模式切换/长时间丢帧后位姿会跳变, 此时重新锚定,
    // 否则一次跳变会被当成真实位移拉进位姿。
    const double rel_gap = pose.time - m_rel_prev_lio.time;
    const double rel_step = (pose.trans - m_rel_prev_lio.trans).norm();
    if (!(rel_gap > 0.0) || rel_gap > m_cfg.rel_max_gap ||
        !std::isfinite(rel_step) || rel_step > m_cfg.rel_max_step) {
        anchor();
        ++m_diag.relative_reanchored;
        return;
    }
    // 两个增量都在**上一锚点的体体系**里比较, 修正也回到同一参考系,
    // 避免转弯时把"坐标系旋转"误当成横向位移。
    const Eigen::Vector3d d_lio =
        m_rel_prev_lio.rot.transpose() * (pose.trans - m_rel_prev_lio.trans);
    const Eigen::Vector3d d_state =
        m_rel_prev_state_R.transpose() * (m_p - m_rel_prev_state_p);
    const Eigen::Vector3d e = d_lio - d_state;
    if (!e.allFinite()) {
        anchor();
        ++m_diag.relative_reanchored;
        return;
    }

    // 观测噪声: 取协方差平移块的均值 + 下限 (点数少时协方差会被低估)
    double r = 0.0;
    for (int i = 0; i < 3; ++i) {
        r += std::max(cov(3 + i, 3 + i), m_cfg.trans_cov_floor);
    }
    r /= 3.0;
    // 位置协方差均值 -> 标量增益, 上限 gain_trans
    double p = 0.0;
    for (int i = 0; i < 3; ++i) p += m_P(3 + i);
    p /= 3.0;

    // 新息门控: 平滑但错误的 LIO 不能每帧都拉满 max_correction_trans
    const double chi2 = e.squaredNorm() / std::max(p + r, 1e-9);
    m_diag.last_rel_chi2 = chi2;
    if (m_cfg.rel_chi2_gate > 0.0 && (!std::isfinite(chi2) ||
                                      chi2 > m_cfg.rel_chi2_gate)) {
        ++m_diag.relative_rejected;
        return;   // 保持锚点, 等 LIO 回到一致再继续用
    }

    double k = p / (p + r);
    if (!std::isfinite(k)) k = 0.0;
    k = std::min(m_cfg.gain_trans, k);

    Eigen::Vector3d d = k * e;
    if (d.norm() > m_cfg.max_correction_trans && d.norm() > 1e-12) {
        d *= m_cfg.max_correction_trans / d.norm();
    }
    m_p += m_rel_prev_state_R * d;

    ++m_diag.relative_accepted;
    m_diag.last_rel_residual = e.norm();
    m_diag.last_rel_time = pose.time;
    anchor();   // 修正后重新锚定 (等价于滤波器的观测更新)
}

bool EkfPoseFusion::histAt(double t, PoseStamped& out) const {
    if (m_hist.size() < 2) return false;
    if (t < m_hist.front().first || t > m_hist.back().first) return false;
    size_t lo = 0, hi = m_hist.size() - 1;
    while (hi - lo > 1) {
        const size_t mid = (lo + hi) / 2;
        if (m_hist[mid].first <= t) lo = mid; else hi = mid;
    }
    const auto& a = m_hist[lo];
    const auto& b = m_hist[hi];
    const double d = b.first - a.first;
    const double u = (d > 1e-9) ? std::max(0.0, std::min(1.0, (t - a.first) / d)) : 0.0;
    Eigen::Quaterniond qa(a.second.rot), qb(b.second.rot);
    qa.normalize();
    qb.normalize();
    out = a.second;
    out.rot = qa.slerp(u, qb).toRotationMatrix();
    out.trans = (1.0 - u) * a.second.trans + u * b.second.trans;
    out.time = t;
    return true;
}

PoseStamped EkfPoseFusion::predict(double t) const {
    PoseStamped out;
    // **过去时刻: 历史插值** (2026-11-23). 延迟观测修正后状态始终在"最新 IMU 时刻",
    // 帧末 t_a 比它早一个处理延迟; 直接返回当前状态 = 拿"现在"的位姿盖 t_a 的时间戳,
    // 转弯 40°/s × 0.2s 就差 8°. 历史尾部已随观测重锚, 插值出来的就是修正后的位姿.
    bool past = histAt(t, out);
    if (!past && !m_hist.empty() && t <= m_hist.front().first) {
        // t 早于缓冲起点: onMatch 会把 <= 匹配时刻的条目裁掉, 于是"帧末时间戳"经常
        // 刚好落在缓冲起点之前. 这时**必须用起点那条状态**(它离 t 只有一个 IMU 周期),
        // 不能回退到 m_p/m_R: 那是 IMU 的 now, 比帧末晚一个完整处理延迟, 转弯时
        // 直接差出 ω·Δt (现场实测 match_deg=28° @ 0.43 rad/s), 而扫描是按帧末去畸变的,
        // 位姿一超前, 写进会话地图/overlay 就是"转弯墙歪".
        out = m_hist.front().second;
        out.time = t;
        past = true;
    }
    if (!past) {
        out.time = t;
        out.trans = m_p;
        out.rot = m_R;
    }
    const Eigen::Vector3d v_sensor = m_cfg.lever_valid
        ? m_cfg.r_base_lidar.transpose() * Eigen::Vector3d(m_v.x(), m_v.y(), 0.0)
        : Eigen::Vector3d(m_v.x(), m_v.y(), 0.0);
    out.vel = out.rot * v_sensor;
    const double dt = (m_last_imu_time > 0.0) ? (t - m_last_imu_time) : 0.0;
    if (!past && dt > 0.0 && dt < 0.5) {
        const Eigen::Vector3d orbit = m_cfg.lever_valid
            ? m_w_s.cross(m_cfg.lever_arm) : Eigen::Vector3d::Zero();
        // 姿态同样外推: 帧末比最新 IMU 晚几 ms~几十 ms, 不外推 = 每帧固定落后 ω·dt
        const Eigen::Vector3d dw = m_w_s * dt;
        const double an = dw.norm();
        if (an > 1e-12)
            out.rot = m_R * Eigen::AngleAxisd(an, dw / an).toRotationMatrix();
        out.trans += m_R * (v_sensor + orbit) * dt;
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
