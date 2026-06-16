/**
 * @file config.h
 * @brief FAST-LIO2 SLAM - 配置管理
 * @author AI Development Team
 * @date 2026-04-24
 */

#pragma once

#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "fast_lio2_slam/common/types.h"

namespace fast_lio2_slam {

/**
 * @brief 系统配置参数汇总
 */
struct ConfigParams {
    // ==================== IMU参数 ====================
    ImuParams imu;

    // ==================== LiDAR参数 ====================
    LidarParams lidar;

    // ==================== IEKF参数 ====================
    IekfParams iekf;

    // ==================== 闭环检测参数 ====================
    LoopClosureParams loop_closure;

    // ==================== 里程计融合参数 ====================
    OdomFusionParams odom_fusion;

    // ==================== 全局定位参数 ====================
    LocalizationParams localization;

    // ==================== ROS2接口参数 ====================
    struct RosParams {
        // 输入话题
        std::string lidar_topic = "/lidar_points";
        std::string imu_topic = "/imu/data";
        std::string odom_topic = "/odom";

        // 输出话题
        std::string odom_output_topic = "/odom_estimated";
        std::string path_topic = "/path_estimated";
        std::string map_topic = "/cloud_map";
        std::string keyframe_topic = "/keyframe_cloud";

        // 坐标系
        std::string world_frame = "world";
        std::string map_frame = "map";
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
        std::string lidar_frame = "lidar";
        std::string imu_frame = "imu";

        // 服务
        std::string save_map_service = "/save_map";
        std::string load_map_service = "/load_map";
        std::string clear_map_service = "/clear_map";
        std::string get_map_service = "/get_map";
        std::string save_pcd_service = "/save_pcd";

        // 话题
        std::string global_map_topic = "/global_map";
        std::string local_map_topic = "/local_map";

        // 发布参数
        double map_publish_rate = 2.0;     // 地图发布频率 (Hz)
        double path_publish_rate = 20.0;   // 路径发布频率 (Hz)
        int map_max_points = 1000000;      // 地图最大点数

        // TF
        bool publish_tf = true;
        double tf_publish_rate = 50.0;
    } ros;

    // ==================== 地图管理参数 ====================
    struct MapParams {
        double resolution = 0.2;           // 地图分辨率
        double submap_size = 50.0;          // 子地图大小 (米)
        int max_submap_points = 50000;     // 子地图最大点数
        std::string map_path = "./map";    // 地图保存路径
        bool enable_pcd_save = true;       // 启用PCD保存
        bool enable_submap = true;         // 启用子地图
        double map_leaf_size = 0.2;        // 体素滤波大小
        bool enable_compression = false;   // 启用压缩
        double auto_save_interval = 300.0; // 自动保存间隔 (秒)
        double max_memory_usage = 2048.0;  // 最大内存使用 (MB)
    } map;

    // ==================== 地图质量参数 ====================
    struct MapQualityParams {
        double density_weight = 0.4;       // 密度权重
        double uniformity_weight = 0.3;    // 均匀度权重
        double coverage_weight = 0.3;      // 覆盖权重
        double min_density_threshold = 100.0; // 最小密度阈值
    } map_quality;

    // ==================== 系统参数 ====================
    struct SystemParams {
        bool debug_mode = false;           // 调试模式
        int thread_num = 4;                 // 线程数
        double process_timeout = 0.1;      // 处理超时 (秒)
        std::string log_level = "info";    // 日志级别
    } system;
};

/**
 * @brief 配置管理类
 */
class ConfigManager {
public:
    ConfigManager() = default;
    ~ConfigManager() = default;

    /**
     * @brief 从YAML文件加载配置
     */
    bool loadFromFile(const std::string& config_path) {
        try {
            YAML::Node config = YAML::LoadFile(config_path);

            // 加载IMU参数
            if (config["imu"]) {
                auto imu_node = config["imu"];
                params_.imu.acc_noise = imu_node["acc_noise"].as<double>(0.1);
                params_.imu.gyro_noise = imu_node["gyro_noise"].as<double>(0.01);
                params_.imu.acc_bias_noise = imu_node["acc_bias_noise"].as<double>(0.0001);
                params_.imu.gyro_bias_noise = imu_node["gyro_bias_noise"].as<double>(0.00001);
                params_.imu.gravity_magnitude = imu_node["gravity"].as<double>(9.81);
            }

            // 加载LiDAR参数
            if (config["lidar"]) {
                auto lidar_node = config["lidar"];
                params_.lidar.scan_line = lidar_node["scan_line"].as<int>(16);
                params_.lidar.scan_period = lidar_node["scan_period"].as<double>(0.1);
                params_.lidar.max_range = lidar_node["max_range"].as<double>(100.0);
                params_.lidar.min_range = lidar_node["min_range"].as<double>(0.5);
                params_.lidar.voxel_size = lidar_node["voxel_size"].as<double>(0.2);
            }

            // 加载外参
            if (config["extrinsic"]) {
                auto ext_node = config["extrinsic"];
                params_.lidar.translation = Vector3d(
                    ext_node["translation"][0].as<double>(0.0),
                    ext_node["translation"][1].as<double>(0.0),
                    ext_node["translation"][2].as<double>(0.0)
                );
                params_.lidar.rotation_euler = Vector3d(
                    ext_node["rotation"][0].as<double>(0.0),
                    ext_node["rotation"][1].as<double>(0.0),
                    ext_node["rotation"][2].as<double>(0.0)
                );
            }

            // 加载IEKF参数
            if (config["iekf"]) {
                auto iekf_node = config["iekf"];
                params_.iekf.max_iterations = iekf_node["max_iterations"].as<int>(5);
                params_.iekf.converge_threshold = iekf_node["converge_threshold"].as<double>(0.001);
                params_.iekf.position_noise = iekf_node["position_noise"].as<double>(0.01);
                params_.iekf.rotation_noise = iekf_node["rotation_noise"].as<double>(0.01);
            }

            // 加载闭环检测参数
            if (config["loop_closure"]) {
                auto lc_node = config["loop_closure"];
                params_.loop_closure.enable = lc_node["enable"].as<bool>(true);
                params_.loop_closure.detection_rate = lc_node["detection_rate"].as<double>(0.5);
                params_.loop_closure.min_interval = lc_node["min_interval"].as<int>(50);
                params_.loop_closure.threshold = lc_node["threshold"].as<double>(0.3);
            }

            // 加载ROS参数
            if (config["ros"]) {
                auto ros_node = config["ros"];
                params_.ros.lidar_topic = ros_node["lidar_topic"].as<std::string>("/lidar_points");
                params_.ros.imu_topic = ros_node["imu_topic"].as<std::string>("/imu/data");
                params_.ros.odom_topic = ros_node["odom_topic"].as<std::string>("/odom");
                params_.ros.world_frame = ros_node["world_frame"].as<std::string>("world");
                params_.ros.base_frame = ros_node["base_frame"].as<std::string>("base_link");
            }

            // 加载地图参数
            if (config["map"]) {
                auto map_node = config["map"];
                params_.map.resolution = map_node["resolution"].as<double>(0.2);
                params_.map.submap_size = map_node["submap_size"].as<double>(50.0);
                params_.map.map_path = map_node["map_path"].as<std::string>("./map");
            }

            // ==================== 加载全局定位参数 ====================
            if (config["localization"]) {
                auto loc_node = config["localization"];
                params_.localization.enable = loc_node["enable"].as<bool>(true);
                params_.localization.mode = loc_node["mode"].as<std::string>("manual");

                // Scan Context参数
                if (loc_node["scan_context"]) {
                    auto sc_node = loc_node["scan_context"];
                    params_.localization.scan_context_ring_num = sc_node["ring_num"].as<int>(20);
                    params_.localization.scan_context_sector_num = sc_node["sector_num"].as<int>(60);
                    params_.localization.scan_context_max_range = sc_node["max_range"].as<double>(80.0);
                    params_.localization.scan_context_dist_threshold = sc_node["dist_threshold"].as<double>(0.3);
                    params_.localization.scan_context_candidate_count = sc_node["candidate_count"].as<int>(5);
                }

                // 精配准参数
                if (loc_node["fine_alignment"]) {
                    auto fa_node = loc_node["fine_alignment"];
                    params_.localization.fine_alignment_method = fa_node["method"].as<std::string>("ndt");
                    params_.localization.fine_alignment_max_iterations = fa_node["max_iterations"].as<int>(50);
                    params_.localization.fine_alignment_convergence_threshold = fa_node["convergence_threshold"].as<double>(0.01);
                    params_.localization.fine_alignment_resolution = fa_node["resolution"].as<double>(1.0);
                    params_.localization.fine_alignment_voxel_size = fa_node["voxel_size"].as<double>(0.5);
                }

                // 验证参数
                if (loc_node["validation"]) {
                    auto val_node = loc_node["validation"];
                    params_.localization.validation_min_fitness_score = val_node["min_fitness_score"].as<double>(0.7);
                    params_.localization.validation_min_inlier_ratio = val_node["min_inlier_ratio"].as<double>(0.5);
                    params_.localization.validation_max_position_error = val_node["max_position_error"].as<double>(2.0);
                    params_.localization.validation_max_rotation_error = val_node["max_rotation_error"].as<double>(0.5);
                }

                // 搜索参数
                if (loc_node["search"]) {
                    auto search_node = loc_node["search"];
                    params_.localization.search_max_candidates = search_node["max_candidates"].as<int>(10);
                    params_.localization.use_initial_pose_hint = search_node["use_initial_pose_hint"].as<bool>(true);
                }
            }

            config_loaded_ = true;
            return true;

        } catch (const std::exception& e) {
            return false;
        }
    }

    /**
     * @brief 获取配置参数
     */
    const ConfigParams& getParams() const {
        return params_;
    }

    /**
     * @brief 设置配置参数
     */
    void setParams(const ConfigParams& params) {
        params_ = params;
        config_loaded_ = true;
    }

    /**
     * @brief 检查配置是否已加载
     */
    bool isLoaded() const {
        return config_loaded_;
    }

    /**
     * @brief 保存配置到文件
     */
    bool saveToFile(const std::string& config_path) {
        try {
            YAML::Emitter out;
            out << YAML::BeginMap;

            // IMU
            out << YAML::Key << "imu" << YAML::BeginMap;
            out << YAML::Key << "acc_noise" << YAML::Value << params_.imu.acc_noise;
            out << YAML::Key << "gyro_noise" << YAML::Value << params_.imu.gyro_noise;
            out << YAML::EndMap;

            // LiDAR
            out << YAML::Key << "lidar" << YAML::BeginMap;
            out << YAML::Key << "scan_line" << YAML::Value << params_.lidar.scan_line;
            out << YAML::Key << "max_range" << YAML::Value << params_.lidar.max_range;
            out << YAML::EndMap;

            out << YAML::EndMap;

            std::ofstream fout(config_path);
            fout << out.c_str();
            fout.close();

            return true;
        } catch (const std::exception& e) {
            return false;
        }
    }

private:
    ConfigParams params_;
    bool config_loaded_ = false;
};

} // namespace fast_lio2_slam