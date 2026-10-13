#include "algorithms/pcd_map_manager/pcd_map_manager.h"
#include "algorithms/grid_map/grid_map_factory.h"
#include "algorithms/grid_map/grid_map_io.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

namespace {
// SE3 组合/求逆: poseMul(a,b) = 先 b 后 a
PoseStamped poseMul(const PoseStamped& a, const PoseStamped& b) {
    PoseStamped c;
    c.time = b.time;
    c.rot = a.rot * b.rot;
    c.trans = a.rot * b.trans + a.trans;
    return c;
}
PoseStamped poseInv(const PoseStamped& p) {
    PoseStamped q;
    q.time = p.time;
    q.rot = p.rot.transpose();
    q.trans = -q.rot * p.trans;
    return q;
}
}  // namespace

PcdMapManager::PcdMapManager()
    : global_map_(new CloudType()) {}

bool PcdMapManager::init(const std::string& config_path) {
    std::ifstream fin(config_path);
    if (!fin.good()) {
        std::cerr << "[PcdMapManager] Config not found: " << config_path
                  << ", using defaults" << std::endl;
        map_dir_ = ".";
        grid_gen_ = GridMapFactory::create("height_projection");
        initialized_ = true;
        return true;
    }

#ifndef YAML_CPP_DISABLED
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["map_directory"])
            map_dir_ = config["map_directory"].as<std::string>();
        if (config["map_save_voxel"])
            save_voxel_ = config["map_save_voxel"].as<double>();
        if (config["grid_map_generator"])
            grid_gen_name_ = config["grid_map_generator"].as<std::string>();
        if (config["grid_min_height"])
            grid_cfg_.min_height = config["grid_min_height"].as<double>();
        if (config["grid_max_height"])
            grid_cfg_.max_height = config["grid_max_height"].as<double>();
        if (config["grid_occupied_thresh"])
            grid_cfg_.occupied_thresh = config["grid_occupied_thresh"].as<int>();
        if (config["grid_dilate_radius"])
            grid_cfg_.dilate_radius = config["grid_dilate_radius"].as<int>();
        if (config["grid_max_range"])
            grid_cfg_.max_range = config["grid_max_range"].as<double>();
        if (config["grid_ground_z"])
            grid_cfg_.ground_z = config["grid_ground_z"].as<double>();
        if (config["grid_ground_method"])
            grid_cfg_.ground_method = config["grid_ground_method"].as<std::string>();
        if (config["grid_ransac_dist_thr"])
            grid_cfg_.ransac_dist_thr = config["grid_ransac_dist_thr"].as<double>();
        if (config["grid_ransac_max_iter"])
            grid_cfg_.ransac_max_iter = config["grid_ransac_max_iter"].as<int>();
        if (config["grid_min_hits"])
            grid_cfg_.min_hits = config["grid_min_hits"].as<int>();
        if (config["grid_min_misses"])
            grid_cfg_.min_misses = config["grid_min_misses"].as<int>();
        if (config["grid_mark_radius"])
            grid_cfg_.mark_radius = config["grid_mark_radius"].as<int>();
        if (config["grid_min_component_cells"])
            grid_cfg_.min_component_cells = config["grid_min_component_cells"].as<int>();
        if (config["grid_close_radius"])
            grid_cfg_.close_radius = config["grid_close_radius"].as<int>();
    } catch (const std::exception& e) {
        std::cerr << "[PcdMapManager] Config parse error: " << e.what() << std::endl;
        return false;
    }
    grid_gen_ = GridMapFactory::create(grid_gen_name_);
    if (!grid_gen_) {
        std::cerr << "[PcdMapManager] Unknown grid_map_generator '" << grid_gen_name_
                  << "', fallback to height_projection" << std::endl;
        grid_gen_name_ = "height_projection";
        grid_gen_ = GridMapFactory::create(grid_gen_name_);
    }
#else
    (void)config_path;
    grid_gen_ = GridMapFactory::create(grid_gen_name_);
#endif

    if (map_dir_.empty()) map_dir_ = ".";
    initialized_ = true;
    return true;
}

bool PcdMapManager::saveMap(const std::string& name) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    if (!global_map_ || global_map_->empty()) {
        std::cerr << "[PcdMapManager] Nothing to save" << std::endl;
        return false;
    }

    std::string path = map_dir_ + "/" + name;
    if (pcl::io::savePCDFileBinary(path, *global_map_) == -1) {
        std::cerr << "[PcdMapManager] Save failed: " << path << std::endl;
        return false;
    }
    std::cout << "[PcdMapManager] Saved: " << path
              << " (" << global_map_->size() << " pts)" << std::endl;
    return true;
}

bool PcdMapManager::loadMap(const std::string& name) {
    std::string path = map_dir_ + "/" + name;
    CloudType::Ptr cloud(new CloudType());
    if (pcl::io::loadPCDFile(path, *cloud) == -1) {
        std::cerr << "[PcdMapManager] Load failed: " << path << std::endl;
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex_);
    global_map_ = cloud;
    observations_.clear();   // 纯 PCD 无逐帧原点, 射线类策略退回整体点云
    std::cout << "[PcdMapManager] Loaded: " << path
              << " (" << global_map_->size() << " pts)" << std::endl;
    return true;
}

bool PcdMapManager::getGlobalMap(PointVec& out) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    if (!global_map_ || global_map_->empty()) return false;
    out.clear();
    out.reserve(global_map_->size());
    for (const auto& pt : *global_map_) {
        out.push_back(pt);
    }
    return true;
}

bool PcdMapManager::addSubMap(const KeyFrame& kf) {
    if (!kf.cloud || kf.cloud->empty()) return false;

    std::lock_guard<std::mutex> lock(m_mutex_);
    Submap sm;
    sm.pose = kf.pose;
    sm.cloud = kf.cloud;
    submaps_.push_back(sm);

    // 若后端已发生过回环修正, 新关键帧也要平移到修正坐标系下, 保证与重建后的地图连续
    const PoseStamped eff_pose = poseMul(m_frontier_anchor_, kf.pose);

    CloudType::Ptr transformed(new CloudType());
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = eff_pose.rot.cast<float>();
    T.block<3,1>(0,3) = eff_pose.trans.cast<float>();
    pcl::transformPointCloud(*kf.cloud, *transformed, T);

    if (!global_map_) global_map_.reset(new CloudType());
    *global_map_ += *transformed;

    // 保存逐帧观测(原点=修正后位置), 供射线投射类栅格策略使用
    Observation ob;
    ob.origin = eff_pose.trans;
    ob.cloud = transformed;
    observations_.push_back(ob);

    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(global_map_);
    vg.setLeafSize(save_voxel_, save_voxel_, save_voxel_);
    CloudType::Ptr filtered(new CloudType());
    vg.filter(*filtered);
    global_map_ = filtered;

    return true;
}

void PcdMapManager::onBackendOptimized(const std::vector<PoseStamped>& corrected) {
    if (corrected.empty()) return;

    std::lock_guard<std::mutex> lock(m_mutex_);
    if (submaps_.empty()) return;

    // corrected 与 submaps_ 按同一关键帧交付顺序对齐; 后端可能尚未消化最末几帧.
    // 若后端比本地多出(不应发生), 截断到本地数量.
    const size_t n = std::min(corrected.size(), submaps_.size());
    if (n == 0) return;

    // 前沿锚点 A = G_last * P_last^{-1}: 供 pipeline 把"之后"的原始位姿 R 平移到修正系
    // A∘R, 使 odom/tf/path 与重建后的地图连续. (P_last 取与 G_last 同序号的本地原始位姿)
    const PoseStamped& raw_last = submaps_[n - 1].pose;
    m_frontier_anchor_ = poseMul(corrected[n - 1], poseInv(raw_last));

    // 关键帧直接用后端修正位姿 G_i(均在 odom/图坐标系), 尾部未交付后端的帧用原始位姿占位
    std::vector<PoseStamped> used(n);
    for (size_t i = 0; i < n; ++i) used[i] = corrected[i];

    CloudType::Ptr rebuilt(new CloudType());
    std::vector<Observation> rebuilt_obs;
    rebuilt_obs.reserve(submaps_.size());
    for (size_t i = 0; i < submaps_.size(); ++i) {
        const PoseStamped* pose = (i < used.size()) ? &used[i] : &submaps_[i].pose;
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = pose->rot.cast<float>();
        T.block<3,1>(0,3) = pose->trans.cast<float>();
        CloudType::Ptr transformed(new CloudType());
        pcl::transformPointCloud(*submaps_[i].cloud, *transformed, T);
        *rebuilt += *transformed;

        Observation ob;
        ob.origin = pose->trans;
        ob.cloud = transformed;
        rebuilt_obs.push_back(ob);
    }
    observations_ = std::move(rebuilt_obs);

    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(rebuilt);
    vg.setLeafSize(save_voxel_, save_voxel_, save_voxel_);
    CloudType::Ptr filtered(new CloudType());
    vg.filter(*filtered);
    global_map_ = filtered;

    std::cout << "[PcdMapManager] Rebuilt map from backend poses: " << n << " kfs"
              << " (" << global_map_->size() << " pts), anchor trans="
              << m_frontier_anchor_.trans.norm() << " m" << std::endl;
}

bool PcdMapManager::getCorrectionAnchor(PoseStamped& out) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    out = m_frontier_anchor_;
    return true;
}


bool PcdMapManager::generateGridMap(double resolution) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    if (!global_map_ || global_map_->empty()) return false;
    if (!grid_gen_) grid_gen_ = GridMapFactory::create("height_projection");
    if (!grid_gen_) return false;

    GridMapConfig cfg = grid_cfg_;
    cfg.resolution = resolution;
    cfg.point_spacing = save_voxel_;

    GridInfo info;
    bool ok = !observations_.empty()
        ? grid_gen_->buildFromObservations(observations_, Eigen::Vector3d::Zero(), cfg, info)
        : grid_gen_->build(global_map_, Eigen::Vector3d::Zero(), cfg, info);
    if (!ok) return false;

    m_grid_data = info.data;
    m_grid_w = info.width;
    m_grid_h = info.height;
    m_grid_res = info.resolution;
    m_grid_ox = info.origin_x;
    m_grid_oy = info.origin_y;
    std::cout << "[PcdMapManager] Generated grid map (" << grid_gen_->name() << "): "
              << m_grid_w << "x" << m_grid_h << std::endl;
    return true;
}

bool PcdMapManager::saveGridMap(const std::string& pgm_path, const std::string& yaml_path,
                                  double resolution) {
    return saveGridMap(pgm_path, yaml_path, resolution, "map");
}

bool PcdMapManager::saveGridMap(const std::string& pgm_path, const std::string& yaml_path,
                                  double resolution, const std::string& frame_id) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    if (!global_map_ || global_map_->empty()) return false;
    if (!grid_gen_) grid_gen_ = GridMapFactory::create("height_projection");
    if (!grid_gen_) return false;

    GridMapConfig cfg = grid_cfg_;
    cfg.resolution = resolution;
    cfg.point_spacing = save_voxel_;

    GridInfo info;
    bool ok = !observations_.empty()
        ? grid_gen_->buildFromObservations(observations_, Eigen::Vector3d::Zero(), cfg, info)
        : grid_gen_->build(global_map_, Eigen::Vector3d::Zero(), cfg, info);
    if (!ok) return false;
    if (!saveGridMapPGM(info, pgm_path, yaml_path, frame_id)) return false;

    m_grid_data = info.data;
    m_grid_w = info.width;
    m_grid_h = info.height;
    m_grid_res = info.resolution;
    m_grid_ox = info.origin_x;
    m_grid_oy = info.origin_y;
    std::cout << "[PcdMapManager] Grid map saved (" << grid_gen_->name() << "): " << pgm_path
              << " (" << m_grid_w << "x" << m_grid_h << ")" << std::endl;
    return true;
}

bool PcdMapManager::loadGridMap(const std::string& pgm_path, const std::string& yaml_path) {
    GridInfo info;
    if (!loadGridMapPGM(pgm_path, yaml_path, info)) return false;
    m_grid_data = info.data;
    m_grid_w = info.width;
    m_grid_h = info.height;
    m_grid_res = info.resolution;
    m_grid_ox = info.origin_x;
    m_grid_oy = info.origin_y;
    return true;
}

GridInfo PcdMapManager::getGridInfo() const {
    GridInfo info;
    info.width = m_grid_w;
    info.height = m_grid_h;
    info.resolution = m_grid_res;
    info.origin_x = m_grid_ox;
    info.origin_y = m_grid_oy;
    info.data = m_grid_data;
    return info;
}

bool PcdMapManager::setGridMapGenerator(const std::string& name) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    auto gen = GridMapFactory::create(name);
    if (!gen) {
        std::cerr << "[PcdMapManager] Unknown grid generator: " << name << std::endl;
        return false;
    }
    grid_gen_ = std::move(gen);
    grid_gen_name_ = name;
    std::cout << "[PcdMapManager] grid generator -> " << name << std::endl;
    return true;
}

} // namespace rosiwit_slam
