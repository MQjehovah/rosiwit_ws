#include "algorithms/fast_lio2/fast_lio2_frontend.h"
#include "lidar_processor.h"
#include "commons.h"

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
    m_imu_buffer.push_back(s);
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
    out.time = 0;
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
