#include "algorithms/pcd_map_manager/pcd_map_manager.h"
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

PcdMapManager::PcdMapManager()
    : global_map_(new CloudType()) {}

bool PcdMapManager::init(const std::string& config_path) {
    std::ifstream fin(config_path);
    if (!fin.good()) {
        std::cerr << "[PcdMapManager] Config not found: " << config_path
                  << ", using defaults" << std::endl;
        map_dir_ = ".";
        initialized_ = true;
        return true;
    }

#ifndef YAML_CPP_DISABLED
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["map_directory"])
            map_dir_ = config["map_directory"].as<std::string>();
    } catch (const std::exception& e) {
        std::cerr << "[PcdMapManager] Config parse error: " << e.what() << std::endl;
        return false;
    }
#else
    (void)config_path;
#endif

    if (map_dir_.empty()) map_dir_ = ".";
    initialized_ = true;
    return true;
}

bool PcdMapManager::saveMap(const std::string& name) {
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
    global_map_ = cloud;
    std::cout << "[PcdMapManager] Loaded: " << path
              << " (" << global_map_->size() << " pts)" << std::endl;
    return true;
}

bool PcdMapManager::getGlobalMap(PointVec& out) {
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

    CloudType::Ptr transformed(new CloudType());
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = kf.pose.rot.cast<float>();
    T.block<3,1>(0,3) = kf.pose.trans.cast<float>();
    pcl::transformPointCloud(*kf.cloud, *transformed, T);

    *global_map_ += *transformed;

    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(global_map_);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    CloudType::Ptr filtered(new CloudType());
    vg.filter(*filtered);
    global_map_ = filtered;

    return true;
}

bool PcdMapManager::saveGridMap(const std::string& pgm_path, const std::string& yaml_path,
                                  double resolution, const std::string& frame_id) {
    if (!global_map_ || global_map_->empty()) return false;
    OccupancyGridConfig cfg;
    cfg.resolution = resolution;
    OccupancyGridMap grid_map(cfg);
    if (!grid_map.buildFromPointCloud(global_map_)) return false;
    if (!grid_map.saveToPGM(pgm_path, yaml_path, frame_id)) return false;
    m_grid_data = grid_map.getData();
    m_grid_w = grid_map.getWidth();
    m_grid_h = grid_map.getHeight();
    m_grid_res = resolution;
    std::cout << "[PcdMapManager] Grid map saved: " << pgm_path
              << " (" << m_grid_w << "x" << m_grid_h << ")" << std::endl;
    return true;
}

bool PcdMapManager::loadGridMap(const std::string& pgm_path, const std::string& yaml_path) {
    OccupancyGridMap grid_map;
    if (!grid_map.loadFromPGM(pgm_path, yaml_path)) return false;
    m_grid_data = grid_map.getData();
    m_grid_w = grid_map.getWidth();
    m_grid_h = grid_map.getHeight();
    return true;
}

} // namespace rosiwit_slam
