#include "algorithms/fast_lio2/fast_lio2_frontend.h"
#include "lidar_processor.h"
#include <yaml-cpp/yaml.h>

namespace rosiwit_slam {

bool FastLio2Frontend::init(const std::string& config_path) {
    YAML::Node config = YAML::LoadFile(config_path);
    if (!config) return false;

    m_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
    m_config.lidar_min_range  = config["lidar_min_range"].as<double>();
    m_config.lidar_max_range  = config["lidar_max_range"].as<double>();
    m_config.scan_resolution  = config["scan_resolution"].as<double>();
    m_config.map_resolution   = config["map_resolution"].as<double>();
    m_config.cube_len         = config["cube_len"].as<double>();
    m_config.det_range        = config["det_range"].as<double>();
    m_config.move_thresh      = config["move_thresh"].as<double>();
    m_config.na  = config["na"].as<double>();
    m_config.ng  = config["ng"].as<double>();
    m_config.nba = config["nba"].as<double>();
    m_config.nbg = config["nbg"].as<double>();
    m_config.imu_init_num    = config["imu_init_num"].as<int>();
    m_config.near_search_num = config["near_search_num"].as<int>();
    m_config.ieskf_max_iter  = config["ieskf_max_iter"].as<int>();
    m_config.gravity_align   = config["gravity_align"].as<bool>();
    m_config.esti_il         = config["esti_il"].as<bool>();
    auto t_il = config["t_il"].as<std::vector<double>>();
    auto r_il = config["r_il"].as<std::vector<double>>();
    m_config.t_il << t_il[0], t_il[1], t_il[2];
    m_config.r_il << r_il[0], r_il[1], r_il[2],
                     r_il[3], r_il[4], r_il[5],
                     r_il[6], r_il[7], r_il[8];
    m_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

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
        const auto& s = m_imu_buffer.front();
        double t = s.time;
        fast_imus.emplace_back(s.acc, s.gyro, t);
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
