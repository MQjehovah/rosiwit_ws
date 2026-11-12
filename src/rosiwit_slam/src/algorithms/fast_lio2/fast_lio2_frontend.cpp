#include "algorithms/fast_lio2/fast_lio2_frontend.h"
#include "lidar_processor.h"
#include "commons.h"

#include <cmath>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

namespace rosiwit_slam {

bool FastLio2Frontend::init(const std::string& config_path) {
    if (!parseFastLio2Config(config_path, m_config)) {
        return false;
    }

    m_kf = std::make_shared<IESKF>();
    m_kf->setMaxIter(static_cast<size_t>(m_config.ieskf_max_iter));
    m_kf->setDegenerateEigenRatio(m_config.degenerate_eigen_ratio);
    m_kf->setDegenerateRemoveWeak(m_config.degenerate_remove_weak);
    m_builder = std::make_shared<MapBuilder>(m_config, m_kf);
    m_inited = true;
    return true;
}

bool FastLio2Frontend::estimateGroundNormal(const CloudType::Ptr& cloud, V3D& normal_world,
                                            double& plane_d) {
    if (!cloud || cloud->size() < 100) return false;

    // 选点用**重力方向**而不是位姿: 重力只随姿态变化, 不随位置漂移, 因此不会出现
    // "位姿掉了 -> 选点带跟着掉 -> 地面估计跟着掉" 的正反馈.
    // 同时这也自动处理雷达斜装 42deg: 地面点在 body 系里相对 -g_b 的下视角是稳定的.
    const M3D R = m_kf->x().r_wi;
    const V3D t = m_kf->x().t_wi;
    const V3D down_b = R.transpose() * V3D(0.0, 0.0, -1.0);

    std::vector<V3D> coarse;
    coarse.reserve(cloud->size() / 2);
    for (size_t i = 0; i < cloud->size(); i += 2) {
        const auto& p = cloud->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const V3D pb(p.x, p.y, p.z);
        const double rng = pb.norm();
        if (rng < 1.0 || rng > 18.0) continue;
        const double depress = pb.dot(down_b) / rng;   // 相对"正下方"的余弦
        // 下视角 6~30deg: 既排除车体/近处地面噪声, 也排除墙与远处结构
        if (depress < std::cos(30.0 * M_PI / 180.0) ||
            depress > std::cos(6.0 * M_PI / 180.0)) continue;
        coarse.push_back(R * pb + t);
    }
    if (coarse.size() < 80) return false;

    auto fit = [](const std::vector<V3D>& pts, V3D& n_out) -> bool {
        if (pts.size() < 50) return false;
        V3D mean = V3D::Zero();
        for (const auto& p : pts) mean += p;
        mean /= static_cast<double>(pts.size());
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto& p : pts) { V3D d = p - mean; cov += d * d.transpose(); }
        cov /= static_cast<double>(pts.size());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        if (es.info() != Eigen::Success) return false;
        n_out = es.eigenvectors().col(0);
        if (n_out.z() < 0.0) n_out = -n_out;
        if (es.eigenvalues()(0) > 0.10 * es.eigenvalues()(1) + 1e-6) return false;
        return n_out.allFinite();
    };

    V3D n0;
    if (!fit(coarse, n0)) return false;
    V3D mean = V3D::Zero();
    for (const auto& p : coarse) mean += p;
    mean /= static_cast<double>(coarse.size());
    const double d0 = -n0.dot(mean);
    std::vector<V3D> inliers;
    inliers.reserve(coarse.size());
    for (const auto& p : coarse) {
        if (std::fabs(n0.dot(p) + d0) < 0.12) inliers.push_back(p);
    }
    V3D n1;
    const bool refined = (inliers.size() >= 50 && fit(inliers, n1));
    normal_world = refined ? n1 : n0;
    const std::vector<V3D>& src = refined ? inliers : coarse;
    V3D m2 = V3D::Zero();
    for (const auto& p : src) m2 += p;
    m2 /= static_cast<double>(src.size());
    plane_d = -normal_world.dot(m2);

    if (std::acos(std::max(-1.0, std::min(1.0, normal_world.z()))) > 40.0 * M_PI / 180.0)
        return false;
    if (!m_ground_z_anchor_valid) {
        m_ground_z_anchor = m_kf->x().t_wi.z() - 0.9;
        m_ground_z_anchor_valid = true;
    }
    return true;
}

void FastLio2Frontend::onImu(const IMUSample& s) {
    if (!m_inited) return;
    IMUSample scaled = s;
    scaled.acc = s.acc * m_config.acc_scale;
    // 防御性上限: 正常 onLidar 会清空缓冲; 若雷达长时间不来 (或误在定位模式喂 IMU),
    // 也不能无界增长. 2000 帧 @200Hz ~= 10s, 足够覆盖任何正常帧间隔.
    constexpr size_t kMaxImuBuffer = 2000;
    if (m_imu_buffer.size() >= kMaxImuBuffer) m_imu_buffer.erase(m_imu_buffer.begin());
    m_imu_buffer.push_back(scaled);
}

void FastLio2Frontend::onWheelOdom(const PoseStamped& odom) {
    // /odom 回调在 sensor 组线程, onLidar 在 work 线程: 必须加锁,
    // 否则 vector 扩容时与读端并发, 会 double free / 堆损坏。
    {
        std::lock_guard<std::mutex> lk(m_odom_mu);
        m_odom_buffer.push_back(odom);
        while (m_odom_buffer.size() > 600) m_odom_buffer.erase(m_odom_buffer.begin());
    }
}


void FastLio2Frontend::onLidar(const LidarFrame& f) {
    if (!m_inited) return;

    // IMU 空洞帧: 直接跳过。此前虽然 SlamBase 会丢覆盖不足的帧，但仍有
    // 120~200ms 空洞帧漏进来；空洞期间 IMU 预测会把速度/零偏放大，
    // 随后一帧 LiDAR 把 ba 打到 1e0 量级，长航时直接发散。
    // 空洞帧不在这里硬丢: SlamBase 已按 IMU 覆盖度丢帧; 这里再丢会清空
    // 前端 IMU 缓冲, 让空洞后的第一帧缺少传播基准 (服务器实测 z 被带到 -15m)。
    // 只记录, 继续交给 IMUProcessor 的空洞重置逻辑。
    if (f.imu_gap) {
        static int gap_log = 0;
        if (++gap_log <= 10 || gap_log % 50 == 0) {
            std::cerr << "[FLIO2] lidar frame with IMU gap at " << f.end_time
                      << " (total=" << gap_log << ")" << std::endl;
        }
    }
    m_skip_output = false;

    ::Vec<::IMUData> fast_imus;
    fast_imus.reserve(m_imu_buffer.size());
    while (!m_imu_buffer.empty()) {
        auto& s = m_imu_buffer.front();
        fast_imus.emplace_back(std::move(s.acc), std::move(s.gyro), s.time);
        m_imu_buffer.erase(m_imu_buffer.begin());
    }

    ::SyncPackage fast_pkg;
    fast_pkg.cloud = f.cloud;
    fast_pkg.cloud_start_time = f.start_time;
    fast_pkg.cloud_end_time = f.end_time;
    fast_pkg.imus = std::move(fast_imus);

    // 轮式 odom 先验: 只用相邻两帧的相对位移/yaw 速度, 不依赖 TF 外参.
    if (m_builder && m_builder->lidar_processor()) {
        bool ok = false; double speed = 0.0, yaw_rate = 0.0, lat = 0.0;
        {
            std::lock_guard<std::mutex> lk(m_odom_mu);
            if (m_odom_buffer.size() >= 2) {
                auto pick = [&](double t) -> const PoseStamped* {
                    const PoseStamped* best = nullptr;
                    for (const auto& o : m_odom_buffer) {
                        if (o.time <= t) best = &o;
                    }
                    return best ? best : &m_odom_buffer.front();
                };
                const PoseStamped* b = pick(f.end_time);
                if (b && std::abs(b->time - f.end_time) < 0.5) {
                    speed = b->vel.x();        // 带符号: 倒车不再丢失
                    yaw_rate = b->vel.y();
                    lat = b->vel.z();          // twist.linear.y -> 侧向速度
                    ok = true;
                }
            }
        }
        m_builder->lidar_processor()->setOdomFuse(m_config.odom_fuse);
        m_builder->lidar_processor()->setOdomPrior(ok && (m_config.odom_fuse || m_config.odom_gate), speed, yaw_rate, lat);
    }

    // 地面跟随: 世界系地面平面 (法向 + d). z 用"传感器正下方地面高度"这个绝对量测,
    // 不做 vz 积分 —— 积分在有坡度误估时会 20s 累积出几米 (实测 430s +3.9m)。
    {
        V3D n_world;
        double d = 0.0;
        if (estimateGroundNormal(f.cloud, n_world, d) && n_world.z() > 0.85) {
            const auto& xs = m_kf->x();
            const double ground_z = -(n_world.x() * xs.t_wi.x() +
                                      n_world.y() * xs.t_wi.y() + d) / n_world.z();
            if (!m_ground_h0_valid) {
                m_ground_h0 = xs.t_wi.z() - ground_z;
                m_ground_h0_valid = true;
                std::cout << "[FLIO2] ground h0=" << m_ground_h0 << std::endl;
            }
            m_ground_normal_w = n_world;
            double zt = ground_z + m_ground_h0;
            // 限速: 单帧跳变 >0.3m 视为坏估计, 直接丢弃本帧
            if (m_ground_valid && std::fabs(zt - m_ground_z_target) > 0.3) {
                m_ground_last_ok = f.end_time;
            } else {
                m_ground_z_target = zt;
            }
            m_ground_valid = true;
            m_ground_last_ok = f.end_time;
        } else if (m_ground_valid && (f.end_time - m_ground_last_ok) < 1.0) {
            // hold: 沿用上一帧地面高度
        } else {
            m_ground_valid = false;
        }
        m_builder->lidar_processor()->setGroundPlane(m_ground_valid, m_ground_normal_w,
                                                     m_config.ground_follow_z,
                                                     m_ground_z_target);
        static int gp_log = 0;
        if (++gp_log % 200 == 0) {
            const double slope = std::acos(std::max(-1.0, std::min(1.0, m_ground_normal_w.z())));
            std::cout << "[FLIO2] ground " << (m_ground_valid ? "ok" : "hold")
                      << " slope=" << slope * 180.0 / M_PI
                      << "deg z=" << m_kf->x().t_wi.z()
                      << " z_target=" << m_ground_z_target << std::endl;
        }
    }

    m_builder->process(fast_pkg);

    m_current_cloud = f.cloud;
    m_last_lidar_time = f.end_time;
}

bool FastLio2Frontend::getOdometry(PoseStamped& out) {
    if (!m_inited || m_skip_output || m_builder->status() != BuilderStatus::MAPPING) return false;
    // 输出必须使用扫描时间, 而不是 0 + ROS 层 now(); 处理延迟/离线回放时
    // 否则 odom/TF/path 会被打上处理时刻, 造成 TF 外插和延迟统计错误.
    out.time = m_last_lidar_time;
    out.rot   = m_kf->x().r_wi;
    out.trans = m_kf->x().t_wi;
    out.vel   = m_kf->x().v;
    return true;
}

bool FastLio2Frontend::getClouds(CloudType::Ptr& body, CloudType::Ptr& world) {
    if (!m_inited || m_skip_output || m_builder->status() != BuilderStatus::MAPPING || !m_current_cloud) return false;
    auto lp = m_builder->lidar_processor();
    body  = LidarProcessor::transformCloud(m_current_cloud, m_kf->x().r_il, m_kf->x().t_il);
    world = LidarProcessor::transformCloud(m_current_cloud, lp->r_wl(), lp->t_wl());
    return true;
}

bool FastLio2Frontend::getKeyFrame(KeyFrame& out) {
    if (!m_inited || m_skip_output || m_builder->status() != BuilderStatus::MAPPING || !m_current_cloud) return false;

    // 诊断: 打印**当前实际在用**的 IMU-雷达外参 (rpy 度)。esti_il=true 时这里是 EKF 在线
    // 估计出来的值 (会随运动收敛); false 时就是配置里的定值。约每 100 帧 (≈10s) 一条。
    // 用途: 实机"旋转时叠图"排查 —— 若这里收敛到明显的非 0 角度, 说明原来的 identity 是错的。
    {
        static int est_log_i = 0;
        if (++est_log_i % 100 == 0) {
            const auto& xe = m_kf->x();
            const Eigen::Vector3d rpy = xe.r_il.eulerAngles(2, 1, 0);   // yaw,pitch,roll
            std::cout << "[FLIO2] extrinsic(used) esti_il=" << (m_config.esti_il ? 1 : 0)
                      << " rpy_deg=[" << rpy.z() * 180.0 / M_PI << " "
                      << rpy.y() * 180.0 / M_PI << " " << rpy.x() * 180.0 / M_PI
                      << "] t_il=[" << xe.t_il.transpose() << "]" << std::endl;
        }
    }

    const auto& x = m_kf->x();
    if (m_has_last_kf) {
        double dt = (x.t_wi - m_last_kf_pose.trans).norm();
        // r_wi 经过多次 EKF 乘法更新后会累积 ~1e-8 的正交性误差,
        // 而 Sophus::SO3d(Matrix3d) 的 ensure 容差是 1e-12, 会直接 abort
        // (实机前倾 43 度的雷达外参下必现). 先转成归一化四元数再构造.
        Eigen::Quaterniond dq(x.r_wi * m_last_kf_pose.rot.transpose());
        dq.normalize();
        double drot = Sophus::SO3d(dq).log().norm();
        if (dt < m_keyframe_trans_thresh && drot < m_keyframe_rot_thresh) {
            return false;
        }
    }

    out.pose.time   = m_last_lidar_time;
    out.pose.rot    = x.r_wi;
    out.pose.trans  = x.t_wi;
    out.pose.vel    = x.v;
    out.cloud       = LidarProcessor::transformCloud(m_current_cloud, x.r_il, x.t_il);
    out.id          = "kf_" + std::to_string(m_frame_counter++);
    out.timestamp   = static_cast<int64_t>(m_last_lidar_time * 1e6);

    m_last_kf_pose = out.pose;

    // 附带关键帧时刻的轮速位姿: 前后两帧线性插值. 后端用它构造相邻关键帧的
    // 3-DoF 相对因子 (Δx, Δy, Δyaw), 只取相对量, 与 odom 的绝对坐标无关.
    {
        std::lock_guard<std::mutex> lk(m_odom_mu);
        const PoseStamped* a = nullptr;
        const PoseStamped* b = nullptr;
        for (const auto& o : m_odom_buffer) {
            if (o.time <= m_last_lidar_time) a = &o;
            else { b = &o; break; }
        }
        if (a && b && (b->time - a->time) > 1e-4) {
            const double alpha = (m_last_lidar_time - a->time) / (b->time - a->time);
            out.wheel_pose.time = m_last_lidar_time;
            out.wheel_pose.trans = a->trans + alpha * (b->trans - a->trans);
            const Eigen::Quaterniond qa(a->rot), qb(b->rot);
            out.wheel_pose.rot = qa.slerp(alpha, qb).normalized().toRotationMatrix();
            out.has_wheel_pose = true;
        } else if (a && !b && (m_last_lidar_time - a->time) < 0.5) {
            out.wheel_pose = *a;
            out.has_wheel_pose = true;
        }
    }
    m_has_last_kf = true;
    return true;
}

bool FastLio2Frontend::getGlobalMap(PointVec& out) {
    if (!m_builder || m_builder->status() != BuilderStatus::MAPPING) return false;
    m_builder->lidar_processor()->collectGlobalMap(out);
    return !out.empty();
}

bool FastLio2Frontend::isMapping() const {
    return m_inited && m_builder && m_builder->status() == BuilderStatus::MAPPING;
}

} // namespace rosiwit_slam
