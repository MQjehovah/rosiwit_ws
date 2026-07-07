// src/algorithms/fast_lio2/fast_lio2_algorithm.cpp
#include "algorithms/fast_lio2/fast_lio2_algorithm.h"
#include "lidar_processor.h"
#include <yaml-cpp/yaml.h>

namespace rosiwit_slam {

bool FastLio2Algorithm::init(const std::string& config_path) {
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

bool FastLio2Algorithm::onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) {
    if (!m_inited) return false;

    // SLAM 层 IMUSample -> FastLIO2 内部 IMUData
    ::Vec<::IMUData> fast_imus;
    fast_imus.reserve(pkg.imus.size());
    for (const auto& s : pkg.imus) {
        double t = s.time;
        fast_imus.emplace_back(s.acc, s.gyro, t);
    }

    // SLAM 层 SyncPackage -> FastLIO2 内部 ::SyncPackage
    ::SyncPackage fast_pkg;
    fast_pkg.cloud           = pkg.frame.cloud;
    fast_pkg.cloud_start_time = pkg.frame.start_time;
    fast_pkg.cloud_end_time   = pkg.frame.end_time;
    fast_pkg.imus            = std::move(fast_imus);

    m_builder->process(fast_pkg);
    if (m_builder->status() != BuilderStatus::MAPPING) return false;

    setState(SlamState::RUNNING);
    out.state       = SlamState::RUNNING;
    out.pose.time   = pkg.frame.end_time;
    out.pose.rot    = m_kf->x().r_wi;
    out.pose.trans  = m_kf->x().t_wi;
    out.pose.vel    = m_kf->x().v;
    auto lp         = m_builder->lidar_processor();
    out.body_cloud  = LidarProcessor::transformCloud(pkg.frame.cloud, m_kf->x().r_il, m_kf->x().t_il);
    out.world_cloud = LidarProcessor::transformCloud(pkg.frame.cloud, lp->r_wl(), lp->t_wl());
    out.has_new_pose = true;
    return true;
}

bool FastLio2Algorithm::getGlobalMap(PointVec& out_points) {
    if (!m_builder || m_builder->status() != BuilderStatus::MAPPING) return false;
    m_builder->lidar_processor()->collectGlobalMap(out_points);
    return !out_points.empty();
}

} // namespace rosiwit_slam
