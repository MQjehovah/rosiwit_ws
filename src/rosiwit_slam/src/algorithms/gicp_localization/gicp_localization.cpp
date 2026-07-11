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
    m_has_imu_propagation = false;
    m_imu_buffer.clear();
    m_last_imu_time = -1.0;
    m_last_vel = V3D::Zero();
    status_ = INIT;
}

// === IMU 航迹推算: 在 LiDAR 帧间传播位姿估计 ===
// 使用简单的 IMU 积分 (不考虑地球自转):
//   角度: 角速度积分 (二阶龙格-库塔)
//   位置: 加速度双重积分 (body→world 投影后去除重力)
void GicpLocalization::onImu(const IMUSample& s) {
    if (!has_map_) return;

    if (!m_first_imu_received) {
        // 首个 IMU: 初始化时间戳, 不做积分
        m_last_imu_time = s.time;
        m_first_imu_received = true;
        // 缓存 IMU 用于 onLidar
        if (status_ != LOST) m_imu_buffer.push_back(s);
        return;
    }

    // 缓存 IMU 供 onLidar 使用
    if (status_ != LOST) {
        m_imu_buffer.push_back(s);
    }

    const double dt = s.time - m_last_imu_time;
    if (dt <= 0.0 || dt > 0.2) {
        // 时间戳异常或间隔过大, 跳过此帧
        m_last_imu_time = s.time;
        return;
    }

    // 减去零偏
    const V3D gyro = s.gyro - m_gyro_bias;
    const V3D acc  = s.acc - m_acc_bias;

    // 角度积分: 中值法 (梯形积分)
    const V3D angle_inc = gyro * dt;
    const double angle_norm = angle_inc.norm();
    if (angle_norm > 1e-12) {
        const Eigen::AngleAxisd aa(angle_norm, angle_inc / angle_norm);
        current_pose_.rot = current_pose_.rot * aa.toRotationMatrix();
    }

    // 位置-速度积分: 加速度在 world 系下投影
    //   注意: 此处假设初始重力已被车身静态加速度抵消
    //   更准确的做法: 在 setInitPose 时测量重力方向
    const V3D acc_world = current_pose_.rot * acc;
    current_pose_.trans += current_pose_.vel * dt + 0.5 * acc_world * dt * dt;
    current_pose_.vel += acc_world * dt;

    m_last_imu_time = s.time;
    m_has_imu_propagation = true;
}

void GicpLocalization::onLidar(const LidarFrame& f) {
    if (!has_map_ || !f.cloud || f.cloud->empty()) {
        status_ = LOST;
        return;
    }

    // 消费 IMU 缓存: 确保 IMU 数据能覆盖此 LiDAR 帧的时间范围
    // 如果 onImu 已做传播, 可直接使用当前位姿; 否则用缓存做传播
    if (!m_has_imu_propagation && !m_imu_buffer.empty()) {
        // 回放缓存的 IMU 数据做传播
        while (m_imu_buffer.size() >= 2) {
            const auto& imu = m_imu_buffer.front();
            if (imu.time > f.end_time) break;
            onImu(imu);
        }
    }

    CloudType::Ptr scan = downsampleCloud(f.cloud, voxel_resolution_);
    if (scan->empty()) {
        status_ = LOST;
        return;
    }

    gicp_.setInputTarget(downsampled_map_);
    gicp_.setInputSource(scan);

    // 使用 IMU 推算的位姿作为初始猜测, 而非简单的恒等矩阵
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    guess.block<3,3>(0,0) = current_pose_.rot.cast<float>();
    guess.block<3,1>(0,3) = current_pose_.trans.cast<float>();

    CloudType aligned;
    gicp_.align(aligned, guess);

    if (gicp_.hasConverged()) {
        Eigen::Matrix4f T = gicp_.getFinalTransformation();
        current_pose_.rot = T.block<3,3>(0,0).cast<double>();
        current_pose_.trans = T.block<3,1>(0,3).cast<double>();

        const double fitness = gicp_.getFitnessScore();
        if (fitness > fitness_threshold_) {
            status_ = LOST;
        } else {
            status_ = LOCALIZED;
            first_scan_ = false;
            // LiDAR 配准成功后将速度清零 (下一周期由 IMU 重新推算)
            current_pose_.vel = V3D::Zero();
        }
    } else {
        if (!first_scan_) status_ = LOST;
    }

    // 清空已消费的 IMU 缓存
    m_imu_buffer.clear();
    m_has_imu_propagation = false;
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
