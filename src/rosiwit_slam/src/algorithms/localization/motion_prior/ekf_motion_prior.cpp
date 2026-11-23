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
    m_hist.clear();
    m_hist.emplace_back(pose.time, pose);
    m_q_free = Eigen::Quaterniond::Identity();
    m_qhist.clear();
    m_pose = pose;
    m_v.setZero();
    m_P = Eigen::Matrix2d::Identity() * 1.0;   // 初值不确定度 1 m/s
    m_odom_rejected = 0;
    m_last_imu_time = -1.0;
    m_first_imu = false;
    m_have_odom = false;
    m_odom_time = -1.0;
    m_last_w = V3D::Zero();
    m_last_match_rot = pose.rot;
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
    const V3D w_s = s.gyro - m_gyro_bias;   // 传感器系角速度
    const V3D angle_inc = w_s * dt;
    const double an = angle_inc.norm();
    if (an > 1e-12)
        m_pose.rot = m_pose.rot * Eigen::AngleAxisd(an, angle_inc / an).toRotationMatrix();

    // ---- 自由陀螺积分: deskew 帧内相对旋转 (与锚定无关, 见头文件注释) ----
    if (an > 1e-12)
        m_q_free = m_q_free * Eigen::Quaterniond(Eigen::AngleAxisd(an, angle_inc / an));
    else
        m_q_free = m_q_free * Eigen::Quaterniond(1.0, angle_inc.x() * 0.5, angle_inc.y() * 0.5,
                                                 angle_inc.z() * 0.5);
    m_q_free.normalize();
    m_qhist.emplace_back(s.time, m_q_free);
    while (m_qhist.size() > 500) m_qhist.pop_front();

    // ---- 位置: ṗ_lidar = R·(R_blᵀ·v_base + ω_s×ρ) ----
    // v 是体系(base)速度, 需先转到传感器系; ω_s×ρ 是绕 base 自转时雷达绕 base
    // 划弧的轨道项 (slam_types.h: leverArmSensorFrame). 缺它时原地转 180° 位置
    // 误差 2|ρ_xy|≈0.83m 且方向随转向旋转 —— 2026-11-23 "一转弯就丢"根因.
    m_last_w = w_s;
    const V3D v_sensor = m_cfg.lever_valid
                             ? m_cfg.r_base_lidar.transpose() * V3D(m_v.x(), m_v.y(), 0.0)
                             : V3D(m_v.x(), m_v.y(), 0.0);
    const V3D orbit = m_cfg.lever_valid ? w_s.cross(m_cfg.lever_arm) : V3D::Zero();
    m_pose.trans += m_pose.rot * (v_sensor + orbit) * dt;
    m_pose.vel = m_pose.rot * v_sensor;

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
    // 机体系偏航角速度: 必须投影到重力轴 —— 本车 IMU 俯装 ~38°, gyro.z 只有真实转速的 ~81%.
    const V3D up_axis = m_cfg.up_body_valid ? m_cfg.up_body : m_up_body;
    m_gyro_yaw_rate = imuYawRate(s.gyro, up_axis);
    // 姿态历史 (~2s @200Hz): 供 predict(t) 对"过去时刻"插值
    m_hist.emplace_back(s.time, m_pose);
    while (m_hist.size() > 500) m_hist.pop_front();
}

void EkfMotionPrior::onOdom(const PoseStamped& odom) {
    if (!m_cfg.use_odom) return;   // 开关: 关闭时速度只来自激光位移(纯 IMU+EKF 退化为智能常速)

    if (m_have_odom && odom.time > m_odom_time) {
        const double dt = odom.time - m_odom_time;
        if (dt > 1e-3) {
            // 前置物理门 (硬阈值, 卡方之外的保底):
            //   平移速度超物理上限 = 打滑/毛刺样本, 整包丢弃
            const V3D dp_odom = odom.trans - m_odom.trans;
            const double speed = dp_odom.head<2>().norm() / dt;
            if (m_cfg.max_odom_speed > 0 && speed > m_cfg.max_odom_speed) {
                ++m_odom_rejected;
                ++m_odom_rejected_speed;
                m_odom = odom;
                m_odom_time = odom.time;
                return;
            }
            // 前置偏航一致性门: 直行打滑时 yaw 两边都 ~0 挡不住平移, 但转弯打滑
            // (odom 转、机身不转) 能挡住 —— 便宜的先验信息, 不浪费
            const M3D dR = odom.rot * m_odom.rot.transpose();
            m_odom_yaw_rate = std::atan2(dR(1, 0), dR(0, 0)) / dt;
            if (std::fabs(m_odom_yaw_rate - m_gyro_yaw_rate) > m_cfg.slip_yaw_thresh) {
                ++m_odom_rejected;
                ++m_odom_rejected_yaw;
                m_odom = odom;
                m_odom_time = odom.time;
                return;
            }
            // EKF 更新: 体系速度观测.
            // ★ 2026-10-31: 体系速度必须用 odom 自己的朝向转, 与地图无关 (重定位不污染).
            // ★ 2026-11-23 修正: ROS 层送来的 odom 是 **IMU/雷达体**在 odom 系的位姿
            //   (composeRight(p, T_base<-imu)). 按雷达体朝向转回的是**雷达体系**速度;
            //   而传播 (R_blᵀ·v + ω×ρ) 与匹配位移观测都把 m_v 当 **base 系**速度 ——
            //   安装偏航 89.9° 下直行被观测成沿雷达 -y, 再被 R_blᵀ 转一次, 预测速度
            //   指向车尾偏左 (实车: 发布位姿落后匹配且方向偏 15~20°; 转弯每帧位置差
            //   0.414·Δθ). 有安装外参时换算到 base: 减掉雷达绕 base 的轨道位移,
            //   再用 base 在 odom 系的朝向转回体系. 这样也不再把轨道运动重复计入.
            Eigen::Vector2d z;
            if (m_cfg.lever_valid) {
                const M3D R_ob_prev = m_odom.rot * m_cfg.r_base_lidar.transpose();   // base -> odom
                const M3D R_ob = odom.rot * m_cfg.r_base_lidar.transpose();
                const V3D t_bl = m_cfg.r_base_lidar * m_cfg.lever_arm;                // 雷达原点 (base 系)
                const V3D dp_b = dp_odom - (R_ob - R_ob_prev) * t_bl;                 // base 原点位移
                const double psi = std::atan2(R_ob_prev(1, 0), R_ob_prev(0, 0));
                const double cps = std::cos(psi), sps = std::sin(psi);
                z = Eigen::Vector2d(cps * dp_b.x() + sps * dp_b.y(),
                                    -sps * dp_b.x() + cps * dp_b.y()) / dt;
            } else {
                const double odom_yaw = std::atan2(odom.rot(1, 0), odom.rot(0, 0));
                z = yawRot(odom_yaw).transpose() * dp_odom.head<2>() / dt;
            }
            // 诊断: 观测与滤波速度的夹角 (正常行驶应≈0°; 稳定在 ±90° 说明两源不一致)
            {
                const double nz = z.norm(), nv = m_v.norm();
                if (nz > 0.02 && nv > 0.02) {
                    const double c = std::max(-1.0, std::min(1.0, z.dot(m_v) / (nz * nv)));
                    m_odom_z_vs_v_deg = std::acos(c) * 180.0 / M_PI;
                }
            }
            if (velocityUpdate(z, m_cfg.ekf_r_odom * m_cfg.ekf_r_odom)) {
                ++m_odom_accepted;
            } else {
                ++m_odom_rejected;
                ++m_odom_rejected_chi2;
            }
            if (++m_odom_probe % 100 == 0) {
                std::cout << "[odom] acc=" << m_odom_accepted
                          << " rej(speed/yaw/chi2)=" << m_odom_rejected_speed << "/"
                          << m_odom_rejected_yaw << "/" << m_odom_rejected_chi2
                          << " z-v=" << m_odom_z_vs_v_deg << "deg"
                          << " v=(" << m_v.x() << "," << m_v.y() << ")"
                          << " z=(" << z.x() << "," << z.y() << ")"
                          << " map-odom=" << m_odom_offset_yaw_deg << "deg"
                          << std::endl;
            }
        }
    }
    m_odom = odom;
    m_odom_time = odom.time;
    m_have_odom = true;
}

bool EkfMotionPrior::velocityUpdate(const Eigen::Vector2d& z, double r_var) {
    // H = I; S = P + R; 卡方门控 (2 自由度, 95% = 5.99)
    const Eigen::Matrix2d S = m_P + Eigen::Matrix2d::Identity() * r_var;
    const Eigen::Vector2d nu = z - m_v;
    const double chi2 = nu.transpose() * S.ldlt().solve(nu);
    if (m_cfg.ekf_chi2 > 0 && chi2 > m_cfg.ekf_chi2) {
        return false;   // 新息与协差不一致: 打滑/外力, 拒绝该观测 (计数由调用方区分来源)
    }
    const Eigen::Matrix2d K = m_P * S.inverse();
    m_v += K * nu;
    m_P = (Eigen::Matrix2d::Identity() - K) * m_P;
    return true;
}

bool EkfMotionPrior::gyroDelta(double t0, double t1, Eigen::Quaterniond& dq) {
    if (m_qhist.size() < 2 || t1 <= t0) return false;
    if (t0 < m_qhist.front().first || t1 > m_qhist.back().first) return false;
    auto interp = [&](double t, Eigen::Quaterniond& q) -> bool {
        size_t lo = 0, hi = m_qhist.size() - 1;
        while (hi - lo > 1) {
            const size_t mid = (lo + hi) / 2;
            if (m_qhist[mid].first <= t) lo = mid; else hi = mid;
        }
        const auto& a = m_qhist[lo];
        const auto& b = m_qhist[hi];
        const double d = b.first - a.first;
        const double u = (d > 1e-9) ? std::max(0.0, std::min(1.0, (t - a.first) / d)) : 0.0;
        q = a.second.slerp(u, b.second);
        return true;
    };
    Eigen::Quaterniond qa, qb;
    if (!interp(t0, qa) || !interp(t1, qb)) return false;
    dq = (qa.conjugate() * qb).normalized();
    return std::isfinite(dq.w()) && std::isfinite(dq.x()) && std::isfinite(dq.y()) &&
           std::isfinite(dq.z());
}

PoseStamped EkfMotionPrior::predict(double t) {
    // **过去时刻: 用历史插值** (雷达帧 end_time 比最新 IMU 早 0.1~0.2s).
    // 只做向前外推会把"未来姿态"当成帧末姿态: 转弯 45°/s 时超前 0.2s*45=9°,
    // 10m 处错 1.5m -> 匹配度 0.95 -> 0.4 (2026-11-23 根因).
    PoseStamped p;
    if (histAt(t, p)) return p;
    if (!m_hist.empty() && t <= m_hist.front().first) return m_hist.front().second;

    // 未来时刻: 从最新积分位姿外推 (两次 IMU 之间/之后; 接口契约是"t 时刻位姿";
    // 2026-10-17 单测抓到旧实现只返回最后积分位姿, 不外推)
    p = m_pose;
    const double dt = t - m_last_imu_time;
    // 前向外推同样要带杆臂轨道项 (与 onImu 一致), 否则转弯中外推位置脱离圆弧.
    const V3D v_sensor = m_cfg.lever_valid
                             ? m_cfg.r_base_lidar.transpose() * V3D(m_v.x(), m_v.y(), 0.0)
                             : V3D(m_v.x(), m_v.y(), 0.0);
    const V3D orbit = m_cfg.lever_valid ? m_last_w.cross(m_cfg.lever_arm) : V3D::Zero();
    if (dt > 0.0 && dt < 0.5) {
        // 姿态也必须外推 (2026-11-23): 帧末比最新 IMU 晚几 ms~几十 ms 是常态,
        // 姿态原样返回 = 每帧固定落后 ω·dt (45°/s × 20ms ≈ 1°), 白白抬高 GICP 初值误差.
        const V3D dw = m_last_w * dt;
        const double an = dw.norm();
        if (an > 1e-12)
            p.rot = m_pose.rot * Eigen::AngleAxisd(an, dw / an).toRotationMatrix();
        p.trans += m_pose.rot * (v_sensor + orbit) * dt;
    }
    p.time = t;
    p.vel = m_pose.rot * v_sensor;
    return p;
}

bool EkfMotionPrior::histAt(double t, PoseStamped& out) const {
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

void EkfMotionPrior::reanchorHistory(const PoseStamped& pose) {
    // **延迟锚定必须补回 (t_a, now] 的相对运动** (2026-11-23 "一转弯就丢" 根因).
    // IMU 从回调线程实时积分; 一帧处理完 (0.1~0.4s) 再 onMatch 时, 最新 IMU 时刻
    // now 已晚于帧末 t_a. 旧实现直接 m_pose = pose, 等于丢掉这段陀螺积分 ->
    // 下一步预测缺 ω·(now - t_a): 回放逐帧对账 guess 每步只转 +2~4° 而真值
    // -9~-11° (35°/s × 0.3s ≈ 10°/帧), 滞后累积到 GICP 吞不下 -> 拒帧 ->
    // 之后恒差 18° 永久丢失. 处理越慢/转得越快越严重 —— 与现象完全一致.
    // 相对运动与锚点无关, 所以直接把旧链上 (t_a, now] 的相对位姿接到新锚点.
    if (m_hist.size() < 2 || pose.time >= m_hist.back().first) {
        m_pose = pose;   // 锚点不早于最新 IMU: 无需补, 后续 IMU 从锚点起积
        return;
    }
    PoseStamped ref;
    if (!histAt(pose.time, ref)) {
        m_pose = pose;
        return;
    }
    for (auto& e : m_hist) {
        if (e.first <= pose.time) continue;
        const M3D dR = ref.rot.transpose() * e.second.rot;                  // 旧链相对旋转
        const V3D dp = ref.rot.transpose() * (e.second.trans - ref.trans);  // 旧链相对位移
        e.second.rot = pose.rot * dR;
        e.second.trans = pose.trans + pose.rot * dp;
    }
    // t_a 之前的旧链条目与新链在 t_a 处不连续: 丢弃, 过去时刻一律夹到新锚点
    while (!m_hist.empty() && m_hist.front().first < pose.time) m_hist.pop_front();
    m_pose = m_hist.back().second;   // now 时刻 = 新锚点 + 已积入的相对运动
}

void EkfMotionPrior::onMatch(const PoseStamped& pose) {
    // 速度弱观测: 激光位移/间隔 -> 体系速度 (大测量噪声, 慢学习)
    if (m_last_match_time >= 0.0 && pose.time > m_last_match_time) {
        const double dt = pose.time - m_last_match_time;
        if (dt > 1e-3) {
            // 位移换算到 base 原点 (去杆臂): 雷达位移含绕 base 的轨道运动,
            // 转弯时高达 |ρ_xy|·ω ≈ 0.4m/s, 直接当体系速度会把位置往切向推歪.
            V3D dp_w = pose.trans - m_last_match_trans;
            if (m_cfg.lever_valid)
                dp_w -= pose.rot * m_cfg.lever_arm - m_last_match_rot * m_cfg.lever_arm;
            // 体系速度是 base 系的, 旋转也要用 base 的 yaw (雷达 yaw 差安装角).
            const M3D R_mb = m_cfg.lever_valid ? m_pose.rot * m_cfg.r_base_lidar.transpose()
                                               : m_pose.rot;
            const Eigen::Vector2d z =
                yawRot(std::atan2(R_mb(1, 0), R_mb(0, 0))).transpose() *
                dp_w.head<2>() / dt;
            velocityUpdate(z, m_cfg.ekf_r_match * m_cfg.ekf_r_match);

            // 陀螺零偏在线微调 (航向新息; 与 ImuMotionPrior 同款, 该部分实机验证过)
            const double dtb = std::min(dt, 0.5);
            // 新息必须同时刻: 旧实现拿先验"现在"的姿态与 t_a 的匹配姿态比, 两者差
            // ω·(now - t_a) (回放 ~10°), 会被错记成零偏把真实转速吃掉.
            PoseStamped prior_at;
            const M3D R_prior = histAt(pose.time, prior_at) ? prior_at.rot : m_pose.rot;
            const M3D dR = R_prior.transpose() * pose.rot;
            Eigen::Quaterniond qe(dR);
            if (qe.w() < 0.0) qe.coeffs() *= -1.0;
            m_gyro_bias -= dtb * m_cfg.gyro_bias_gain * (qe.w() * qe.vec());
            m_gyro_bias = m_gyro_bias.cwiseMax(-m_cfg.gyro_bias_max)
                              .cwiseMin(m_cfg.gyro_bias_max);
        }
    }
    // 锚定绝对位姿 (先验只管帧间预测, 绝对位姿归匹配层).
    // 延迟锚定: 补回 (t_a, now] 的相对运动, 见 reanchorHistory.
    reanchorHistory(pose);
    m_pose.vel = m_pose.rot * (m_cfg.lever_valid
                                   ? m_cfg.r_base_lidar.transpose() * V3D(m_v.x(), m_v.y(), 0.0)
                                   : V3D(m_v.x(), m_v.y(), 0.0));
    m_last_match_time = pose.time;
    m_last_match_trans = pose.trans;
    m_last_match_rot = pose.rot;
    // 诊断用: map 与 odom 的朝向差 (本机应≈-90°)。只打印, 不参与计算 ——
    // odom 的体系速度用的是 odom 自己的 yaw, 与这个差值无关。
    if (m_cfg.use_odom && m_have_odom) {
        const double map_yaw = std::atan2(m_pose.rot(1, 0), m_pose.rot(0, 0));
        const double odom_yaw = std::atan2(m_odom.rot(1, 0), m_odom.rot(0, 0));
        double d = (map_yaw - odom_yaw) * 180.0 / M_PI;
        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        m_odom_offset_yaw_deg = d;
    }
}

}   // namespace rosiwit_slam
