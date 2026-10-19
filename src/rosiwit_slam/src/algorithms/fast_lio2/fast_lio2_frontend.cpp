#include "algorithms/fast_lio2/fast_lio2_frontend.h"
#include "lidar_processor.h"
#include "commons.h"

#include <cmath>
#include <iostream>

namespace rosiwit_slam {

bool FastLio2Frontend::init(const std::string& config_path) {
    if (!parseFastLio2Config(config_path, m_config)) {
        return false;
    }

    m_kf = std::make_shared<IESKF>();
    m_kf->setMaxIter(static_cast<size_t>(m_config.ieskf_max_iter));
    m_builder = std::make_shared<MapBuilder>(m_config, m_kf);
    m_inited = true;
    return true;
}

void FastLio2Frontend::onImu(const IMUSample& s) {
    if (!m_inited) return;
    IMUSample scaled = s;
    scaled.acc = s.acc * m_config.acc_scale;
    // 防御性上限: 正常 onLidar 会清空缓冲; 若雷达长时间不来 (或误在定位模式喂 IMU),
    // 也不能无界增长. 2000 帧 @200Hz ~= 10s, 足够覆盖任何正常帧间隔.
    constexpr size_t kMaxImuBuffer = 2000;
    if (m_imu_buffer.size() >= kMaxImuBuffer) m_imu_buffer.pop_front();
    m_imu_buffer.push_back(scaled);
}

void FastLio2Frontend::onLidar(const LidarFrame& f) {
    if (!m_inited) return;

    ::Vec<::IMUData> fast_imus;
    fast_imus.reserve(m_imu_buffer.size());
    while (!m_imu_buffer.empty()) {
        auto& s = m_imu_buffer.front();
        fast_imus.emplace_back(std::move(s.acc), std::move(s.gyro), s.time);
        m_imu_buffer.pop_front();
    }

    ::SyncPackage fast_pkg;
    fast_pkg.cloud = f.cloud;
    fast_pkg.cloud_start_time = f.start_time;
    fast_pkg.cloud_end_time = f.end_time;
    fast_pkg.imus = std::move(fast_imus);

    m_builder->process(fast_pkg);

    m_current_cloud = f.cloud;
    m_last_lidar_time = f.end_time;
}

bool FastLio2Frontend::getOdometry(PoseStamped& out) {
    if (!m_inited || m_builder->status() != BuilderStatus::MAPPING) return false;
    // 输出必须使用扫描时间, 而不是 0 + ROS 层 now(); 处理延迟/离线回放时
    // 否则 odom/TF/path 会被打上处理时刻, 造成 TF 外插和延迟统计错误.
    out.time = m_last_lidar_time;
    out.rot   = m_kf->x().r_wi;
    out.trans = m_kf->x().t_wi;
    out.vel   = m_kf->x().v;
    return true;
}

bool FastLio2Frontend::getClouds(CloudType::Ptr& body, CloudType::Ptr& world) {
    if (!m_inited || m_builder->status() != BuilderStatus::MAPPING || !m_current_cloud) return false;
    auto lp = m_builder->lidar_processor();
    body  = LidarProcessor::transformCloud(m_current_cloud, m_kf->x().r_il, m_kf->x().t_il);
    world = LidarProcessor::transformCloud(m_current_cloud, lp->r_wl(), lp->t_wl());
    return true;
}

bool FastLio2Frontend::getKeyFrame(KeyFrame& out) {
    if (!m_inited || m_builder->status() != BuilderStatus::MAPPING || !m_current_cloud) return false;

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
        double drot = Sophus::SO3d(x.r_wi * m_last_kf_pose.rot.transpose()).log().norm();
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
