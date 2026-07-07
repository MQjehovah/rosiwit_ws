#include "algorithms/gicp_localization/gicp_localization.h"
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

GicpLocalization::GicpLocalization() {
    gicp_.setMaxCorrespondenceDistance(2.0);
    gicp_.setMaximumIterations(max_iterations_);
    gicp_.setTransformationEpsilon(transformation_epsilon_);
    gicp_.setEuclideanFitnessEpsilon(0.01);
}

bool GicpLocalization::init(const std::string& config_path) {
    std::ifstream fin(config_path);
    if (!fin.good()) {
        std::cerr << "[GicpLocalization] Config not found: " << config_path
                  << ", using defaults" << std::endl;
        return true;
    }

#ifndef YAML_CPP_DISABLED
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["voxel_resolution"])
            voxel_resolution_ = config["voxel_resolution"].as<float>();
        if (config["fitness_threshold"])
            fitness_threshold_ = config["fitness_threshold"].as<float>();
        if (config["max_iterations"])
            max_iterations_ = config["max_iterations"].as<int>();
        if (config["transformation_epsilon"])
            transformation_epsilon_ = config["transformation_epsilon"].as<double>();
    } catch (const std::exception& e) {
        std::cerr << "[GicpLocalization] Config parse error: " << e.what() << std::endl;
        return false;
    }
#else
    (void)config_path;
#endif

    gicp_.setMaximumIterations(max_iterations_);
    gicp_.setTransformationEpsilon(transformation_epsilon_);
    return true;
}

void GicpLocalization::setMap(const std::string& map_name) {
    map_cloud_.reset(new CloudType());
    if (pcl::io::loadPCDFile(map_name, *map_cloud_) == -1) {
        std::cerr << "[GicpLocalization] Failed to load map: " << map_name << std::endl;
        has_map_ = false;
        return;
    }
    std::cout << "[GicpLocalization] Loaded map: " << map_name
              << " (" << map_cloud_->size() << " pts)" << std::endl;
    downsampled_map_ = downsampleCloud(map_cloud_, voxel_resolution_);
    has_map_ = true;
    status_ = INIT;
}

void GicpLocalization::setInitPose(const PoseStamped& pose) {
    current_pose_ = pose;
    first_scan_ = true;
    status_ = INIT;
}

void GicpLocalization::onImu(const IMUSample& s) {
    (void)s;
}

void GicpLocalization::onLidar(const LidarFrame& f) {
    if (!has_map_ || !f.cloud || f.cloud->empty()) {
        status_ = LOST;
        return;
    }

    CloudType::Ptr scan = downsampleCloud(f.cloud, voxel_resolution_);
    if (scan->empty()) {
        status_ = LOST;
        return;
    }

    gicp_.setInputTarget(downsampled_map_);
    gicp_.setInputSource(scan);

    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    guess.block<3,3>(0,0) = current_pose_.rot.cast<float>();
    guess.block<3,1>(0,3) = current_pose_.trans.cast<float>();

    CloudType aligned;
    gicp_.align(aligned, guess);

    if (gicp_.hasConverged()) {
        Eigen::Matrix4f T = gicp_.getFinalTransformation();
        current_pose_.rot = T.block<3,3>(0,0).cast<double>();
        current_pose_.trans = T.block<3,1>(0,3).cast<double>();

        double fitness = gicp_.getFitnessScore();
        if (fitness > fitness_threshold_) {
            status_ = LOST;
        } else {
            status_ = LOCALIZED;
            first_scan_ = false;
        }
    } else {
        if (!first_scan_) status_ = LOST;
    }
}

ILocalization::Status GicpLocalization::getStatus() {
    return status_;
}

bool GicpLocalization::getPose(PoseStamped& out) {
    if (status_ == LOST) return false;
    out = current_pose_;
    return true;
}

CloudType::Ptr GicpLocalization::downsampleCloud(const CloudType::Ptr& cloud, float resolution) {
    CloudType::Ptr filtered(new CloudType());
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(resolution, resolution, resolution);
    vg.filter(*filtered);
    return filtered;
}

} // namespace rosiwit_slam
