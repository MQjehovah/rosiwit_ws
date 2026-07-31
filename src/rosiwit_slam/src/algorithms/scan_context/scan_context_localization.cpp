#include "algorithms/scan_context/scan_context_localization.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

bool ScanContextLocalization::init(const std::string& config_path) {
    std::ifstream fin(config_path);
    if (!fin.good()) {
        std::cerr << "[SCLocalization] Config not found, using defaults" << std::endl;
        return true;
    }

#ifndef YAML_CPP_DISABLED
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        auto loc = config["localization"];
        if (loc) {
            m_num_rings     = loc["sc_num_rings"].as<int>(m_num_rings);
            m_num_sectors   = loc["sc_num_sectors"].as<int>(m_num_sectors);
            m_max_radius    = loc["sc_max_radius"].as<double>(m_max_radius);
            m_min_radius    = loc["sc_min_radius"].as<double>(m_min_radius);
            m_dist_thresh   = loc["sc_dist_thresh"].as<double>(m_dist_thresh);
            m_candidate_ratio = loc["sc_candidate_ratio"].as<double>(m_candidate_ratio);
            m_voxel_res         = loc["icp_voxel_res"].as<float>(m_voxel_res);
            m_icp_max_corr_dist = loc["icp_max_corr_dist"].as<float>(m_icp_max_corr_dist);
            m_icp_fitness_thresh = loc["icp_fitness_thresh"].as<float>(m_icp_fitness_thresh);
            m_icp_max_iter       = loc["icp_max_iter"].as<int>(m_icp_max_iter);
        }
    } catch (const std::exception& e) {
        std::cerr << "[SCLocalization] Config parse error: " << e.what() << std::endl;
        return false;
    }
#else
    (void)config_path;
#endif

    ScanContext::Config sc_cfg;
    sc_cfg.num_rings   = m_num_rings;
    sc_cfg.num_sectors = m_num_sectors;
    sc_cfg.max_radius  = m_max_radius;
    sc_cfg.min_radius  = m_min_radius;
    m_sc = ScanContext(sc_cfg);

    m_gicp.setMaximumIterations(m_icp_max_iter);
    m_gicp.setMaxCorrespondenceDistance(m_icp_max_corr_dist);
    m_gicp.setTransformationEpsilon(1e-8);
    m_gicp.setEuclideanFitnessEpsilon(1e-6);

    std::cout << "[SCLocalization] Initialized (rings=" << m_num_rings
              << ", sectors=" << m_num_sectors
              << ", dist_thresh=" << m_dist_thresh << ")" << std::endl;
    return true;
}

void ScanContextLocalization::setMap(const std::string& map_name) {
    m_has_db = false;
    m_status = INIT;
    m_db.clear();
    m_map_cloud.reset();
    m_downsampled_map.reset();

    // 1. 加载 ScanContext 数据库 (约定: map.pcd + ".scdb")
    std::string db_path = map_name + ".scdb";
    if (!loadScanContextDatabase(db_path, m_db) || m_db.empty()) {
        std::cerr << "[SCLocalization] Failed to load SC database: " << db_path << std::endl;
        return;
    }

    // 2. 加载全局点云地图用于 GICP 精配准
    m_map_cloud.reset(new CloudType());
    if (pcl::io::loadPCDFile(map_name, *m_map_cloud) == -1) {
        std::cerr << "[SCLocalization] Failed to load map: " << map_name << std::endl;
        m_db.clear();
        return;
    }
    m_downsampled_map = downsampleCloud(m_map_cloud, m_voxel_res);

    m_has_db = true;
    std::cout << "[SCLocalization] Loaded SC DB: " << m_db.size()
              << " keyframes, map: " << m_map_cloud->size() << " pts" << std::endl;
}

void ScanContextLocalization::setInitPose(const PoseStamped& pose) {
    // ScanContext 全局重定位不需要初始位姿
    m_pose = pose;
}

void ScanContextLocalization::onImu(const IMUSample& s) {
    (void)s;
}

void ScanContextLocalization::onLidar(const LidarFrame& f) {
    m_status = LOST;
    if (!m_has_db || !f.cloud || f.cloud->empty()) return;

    // 1. 当前帧描述子
    ScanContext::Descriptor cur_desc = m_sc.makeDescriptor(f.cloud);

    // 2. RingKey 快速预筛
    struct Candidate { size_t idx; double rk_dist; };
    std::vector<Candidate> candidates;
    candidates.reserve(m_db.size());
    for (size_t i = 0; i < m_db.size(); ++i) {
        double rk = ScanContext::ringKeyDistance(m_db[i].desc.ring_key, cur_desc.ring_key);
        candidates.push_back({i, rk});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.rk_dist < b.rk_dist; });

    size_t num_candidates = std::max<size_t>(1,
        static_cast<size_t>(candidates.size() * m_candidate_ratio));

    // 3. 精确匹配
    int best_idx = -1;
    int best_shift = 0;
    double best_dist = m_dist_thresh;
    for (size_t ci = 0; ci < num_candidates; ++ci) {
        const auto& cand = candidates[ci];
        int shift = 0;
        double d = ScanContext::distance(m_db[cand.idx].desc, cur_desc, &shift);
        if (d < best_dist) {
            best_dist = d;
            best_idx = static_cast<int>(cand.idx);
            best_shift = shift;
        }
    }
    if (best_idx < 0) return;

    // 4. 由匹配关键帧位姿 + 列偏移偏航构建 GICP 初值
    const ScanContextDatabaseEntry& match = m_db[best_idx];
    double yaw = 2.0 * M_PI * best_shift / m_num_sectors;
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    init_guess.block<3,3>(0,0) =
        (match.rot * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()).cast<float>();
    init_guess.block<3,1>(0,3) = match.trans.cast<float>();

    // 5. GICP 对全局地图精配准
    CloudType::Ptr scan_down = downsampleCloud(f.cloud, m_voxel_res);
    if (scan_down->size() < 10) return;

    m_gicp.setInputSource(scan_down);
    m_gicp.setInputTarget(m_downsampled_map);
    CloudType aligned;
    m_gicp.align(aligned, init_guess);

    if (!m_gicp.hasConverged()) return;
    float fitness = m_gicp.getFitnessScore();
    if (fitness > m_icp_fitness_thresh) return;

    Eigen::Matrix4f T = m_gicp.getFinalTransformation();
    m_pose.rot   = T.block<3,3>(0,0).cast<double>();
    m_pose.trans = T.block<3,1>(0,3).cast<double>();
    m_pose.time  = f.end_time;
    m_status = LOCALIZED;

    std::cout << "[SCLocalization] Relocalized: match kf=" << match.id
              << " desc_dist=" << best_dist << " fitness=" << fitness << std::endl;
}

ILocalization::Status ScanContextLocalization::getStatus() {
    return m_status;
}

bool ScanContextLocalization::getPose(PoseStamped& out) {
    if (m_status != LOCALIZED) return false;
    out = m_pose;
    return true;
}

CloudType::Ptr ScanContextLocalization::downsampleCloud(const CloudType::Ptr& cloud, float resolution) {
    CloudType::Ptr filtered(new CloudType());
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(resolution, resolution, resolution);
    vg.filter(*filtered);
    return filtered;
}

} // namespace rosiwit_slam
