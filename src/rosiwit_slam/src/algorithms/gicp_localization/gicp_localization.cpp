#include "algorithms/gicp_localization/gicp_localization.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

GicpLocalization::GicpLocalization() {
    gicp_.setMaxCorrespondenceDistance(max_corr_dist_);
    gicp_.setMaximumIterations(max_iterations_);
    gicp_.setTransformationEpsilon(transformation_epsilon_);
    gicp_.setEuclideanFitnessEpsilon(euclid_eps_);
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
        const YAML::Node config = YAML::LoadFile(config_path);
        // 雷达 -> IMU 外参: ROS1 层注入在 override YAML 的顶层
        if (config["r_il"] && config["r_il"].IsSequence() && config["r_il"].size() == 9) {
            Eigen::Matrix3d R;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    R(r, c) = config["r_il"][r * 3 + c].as<double>();
                }
            }
            Eigen::Vector3d t = Eigen::Vector3d::Zero();
            if (config["t_il"] && config["t_il"].IsSequence() && config["t_il"].size() == 3) {
                t << config["t_il"][0].as<double>(), config["t_il"][1].as<double>(),
                     config["t_il"][2].as<double>();
            }
            m_lidar_to_imu.setIdentity();
            m_lidar_to_imu.block<3, 3>(0, 0) = R.cast<float>();
            m_lidar_to_imu.block<3, 1>(0, 3) = t.cast<float>();
            m_lidar_to_imu_valid = true;
            std::cout << "[GicpLocalization] lidar->imu extrinsic t=[" << t.transpose()
                      << "], scan will be transformed to IMU frame" << std::endl;
        }
        // 标准层级: localization.gicp_localization；兼容旧的 localization.gicp 和根节点写法。
        const YAML::Node gicp = (config["localization"] && config["localization"]["gicp_localization"])
                                   ? config["localization"]["gicp_localization"]
                                   : (config["localization"] && config["localization"]["gicp"])
                                         ? config["localization"]["gicp"]
                                         : config;
        if (gicp["voxel_resolution"])
            voxel_resolution_ = gicp["voxel_resolution"].as<float>();
        if (gicp["fitness_threshold"])
            fitness_threshold_ = gicp["fitness_threshold"].as<float>();
        if (gicp["max_iterations"])
            max_iterations_ = gicp["max_iterations"].as<int>();
        if (gicp["transformation_epsilon"])
            transformation_epsilon_ = gicp["transformation_epsilon"].as<double>();
        if (gicp["max_correspondence_dist"])
            max_corr_dist_ = gicp["max_correspondence_dist"].as<float>();
        else if (gicp["gicp_max_corr_dist"])
            max_corr_dist_ = gicp["gicp_max_corr_dist"].as<float>();
        if (gicp["euclid_epsilon"])
            euclid_eps_ = gicp["euclid_epsilon"].as<double>();
        else if (gicp["gicp_euclid_eps"])
            euclid_eps_ = gicp["gicp_euclid_eps"].as<double>();
        // 新层级 output_filter.*；旧根节点配置沿用 use_pose_filter/pose_filter_*。
        const YAML::Node out = gicp["output_filter"] ? gicp["output_filter"] : gicp;
        if (out["enabled"])
            use_pose_filter_ = out["enabled"].as<bool>();
        else if (out["use_pose_filter"])
            use_pose_filter_ = out["use_pose_filter"].as<bool>();
        if (out["alpha"])
            pose_filter_alpha_ = out["alpha"].as<double>();
        else if (out["pose_filter_alpha"])
            pose_filter_alpha_ = out["pose_filter_alpha"].as<double>();
        if (out["max_xy_m"])
            pose_filter_max_xy_m_ = out["max_xy_m"].as<double>();
        else if (out["pose_filter_max_xy_m"])
            pose_filter_max_xy_m_ = out["pose_filter_max_xy_m"].as<double>();
        if (out["max_yaw_deg"])
            pose_filter_max_yaw_deg_ = out["max_yaw_deg"].as<double>();
        else if (out["pose_filter_max_yaw_deg"])
            pose_filter_max_yaw_deg_ = out["pose_filter_max_yaw_deg"].as<double>();
    } catch (const std::exception& e) {
        std::cerr << "[GicpLocalization] Config parse error: " << e.what() << std::endl;
        return false;
    }
#else
    (void)config_path;
#endif

    gicp_.setMaximumIterations(max_iterations_);
    gicp_.setTransformationEpsilon(transformation_epsilon_);
    gicp_.setMaxCorrespondenceDistance(max_corr_dist_);
    gicp_.setEuclideanFitnessEpsilon(euclid_eps_);
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
    m_vel_world_ = V3D::Zero();
    m_filter_inited_ = false;
    m_last_filter_time_ = -1.0;
    m_last_filtered_trans_ = V3D::Zero();
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

    if (use_pose_filter_) {
        // 滤波路径: 位置用"上一帧估计的世界速度"(常速)推进, 避免加速度双重积分的漂移/抖动;
        // 速度在 onLidar 由匹配位姿差估计并低通得到。
        current_pose_.trans += m_vel_world_ * dt;
        current_pose_.vel = m_vel_world_;
    } else {
        // 原路径: 位置-速度积分, 加速度在 world 系下投影
        //   注意: 此处假设初始重力已被车身静态加速度抵消
        //   更准确的做法: 在 setInitPose 时测量重力方向
        const V3D acc_world = current_pose_.rot * acc;
        current_pose_.trans += current_pose_.vel * dt + 0.5 * acc_world * dt * dt;
        current_pose_.vel += acc_world * dt;
    }

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
    // 雷达系 -> IMU 系 (与建图前端一致; 定位路径本来没做这一步)
    if (m_lidar_to_imu_valid && scan && !scan->empty()) {
        CloudType::Ptr body(new CloudType);
        pcl::transformPointCloud(*scan, *body, m_lidar_to_imu);
        body->header = scan->header;
        scan = body;
    }
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
        const double fitness = gicp_.getFitnessScore();

        if (fitness > fitness_threshold_) {
            status_ = LOST;
        } else if (use_pose_filter_) {
            // ---- 滤波路径: 预测(current_pose_, 由 onImu 推进) + GICP 量测融合 ----
            PoseStamped measured;
            measured.time = f.end_time;
            measured.rot = T.block<3,3>(0,0).cast<double>();
            measured.trans = T.block<3,1>(0,3).cast<double>();

            if (!m_filter_inited_) {
                // 首帧: 直接采用量测, 初始化滤波状态
                current_pose_.rot = measured.rot;
                current_pose_.trans = measured.trans;
                current_pose_.vel.setZero();
                m_vel_world_.setZero();
                m_filter_inited_ = true;
                m_last_filter_time_ = f.end_time;
                status_ = LOCALIZED;
            } else {
                // 残差 = 量测 ⊖ 预测 (旋转取轴角, 平移取差)
                const V3D e_t = measured.trans - current_pose_.trans;
                const Eigen::Matrix3d dR = current_pose_.rot.transpose() * measured.rot;
                const Eigen::AngleAxisd aa(dR);
                const V3D e_r = aa.axis() * aa.angle();
                const double e_yaw_deg = std::fabs(aa.angle()) * 57.2957795;

                const bool outlier = (e_t.norm() > pose_filter_max_xy_m_) ||
                                     (e_yaw_deg > pose_filter_max_yaw_deg_);
                if (!outlier) {
                    const double a = pose_filter_alpha_;
                    // 融合: 只吸收修正量的一部分, 抑制单帧 GICP 噪声
                    current_pose_.rot = current_pose_.rot *
                        (Eigen::AngleAxisd(a * aa.angle(), aa.axis())).toRotationMatrix();
                    current_pose_.trans += a * e_t;

                    // 速度估计: 位置增量/时间, 低通平滑后用于下一帧预测
                    const double dt_l = f.end_time - m_last_filter_time_;
                    if (dt_l > 1e-3) {
                        const V3D inst_vel = (current_pose_.trans - m_last_filtered_trans_) / dt_l;
                        m_vel_world_ = m_vel_world_ * 0.5 + inst_vel * 0.5;
                        current_pose_.vel = m_vel_world_;
                    }
                    m_last_filtered_trans_ = current_pose_.trans;
                    m_last_filter_time_ = f.end_time;
                }
                // 残差超限(离群): 不采信本帧, 保持预测位姿, 避免跳动
                status_ = LOCALIZED;
            }
            first_scan_ = false;
        } else {
            current_pose_.rot = T.block<3,3>(0,0).cast<double>();
            current_pose_.trans = T.block<3,1>(0,3).cast<double>();
            status_ = LOCALIZED;
            first_scan_ = false;
            // LiDAR 配准成功后将速度清零 (下一周期由 IMU 重新推算)
            current_pose_.vel = V3D::Zero();
        }
    } else {
        if (!first_scan_) status_ = LOST;
    }

    // 输出位姿必须带当前扫描时间戳. 否则 setInitPose 之后 current_pose_.time
    // 永远停在初始位姿时间, ROS 层会因相邻输出 stamp 相同而抑制 odom/TF.
    current_pose_.time = f.end_time;

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
