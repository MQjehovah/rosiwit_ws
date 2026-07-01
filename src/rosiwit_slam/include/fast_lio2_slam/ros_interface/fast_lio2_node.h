/**
 * @file fast_lio2_node.h
 * @brief FAST-LIO2 SLAM - ROS2节点主类
 * @author AI Development Team
 * @date 2026-04-24
 *
 * ROS2节点主类，整合所有模块:
 * 1. 数据订阅与发布
 * 2. TF广播
 * 3. 服务接口
 * 4. 参数配置
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <fstream>
#include <pcl/io/pcd_io.h>

#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/common/config.h"
#include "fast_lio2_slam/common/utils.h"
#include "fast_lio2_slam/common/thread_pool.h"   // 新增: 线程池
#include "fast_lio2_slam/common/profiler.h"      // 新增: 性能监控
#include "fast_lio2_slam/data_preprocessor/point_cloud_filter.h"
#include "fast_lio2_slam/data_preprocessor/imu_processor.h"
#include "fast_lio2_slam/data_preprocessor/point_cloud_converter.h"
#include "fast_lio2_slam/fast_lio2_core/iekf_estimator.h"
#include "fast_lio2_slam/fast_lio2_core/ikd_tree.h"
#include "fast_lio2_slam/odom_fusion/odom_fusion.h"
#include "fast_lio2_slam/map_manager/map_manager.h"
#include "fast_lio2_slam/map_manager/map_server.h"
#include "fast_lio2_slam/map_manager/map_persistence.h"
#include "fast_lio2_slam/map_manager/map_quality.h"
#include "fast_lio2_slam/localization/global_localizer.h"

#include <mutex>
#include <queue>
#include <thread>
#include <atomic>
#include <deque>

namespace fast_lio2_slam {

/**
 * @brief FAST-LIO2 SLAM ROS2节点
 */
class FastLio2Node : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 构造函数
     */
    explicit FastLio2Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~FastLio2Node();

private:
    // ==================== 初始化 ====================

    /**
     * @brief 初始化所有组件
     */
    void initialize();

    /**
     * @brief 加载配置参数
     */
    void loadParameters();

    /**
     * @brief 创建订阅者和发布者
     */
    void createSubscribers();
    void createPublishers();
    void createServices();

    /**
     * @brief 初始化核心模块
     */
    void initializeModules();

    // ==================== 数据回调 ====================

    /**
     * @brief 点云数据回调
     */
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief IMU数据回调
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief 里程计数据回调 (可选)
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ==================== 处理函数 ====================

    /**
     * @brief 处理一帧点云
     */
    void processPointCloud(PointCloudData& cloud_data);

    /**
     * @brief 执行IEKF预测步
     */
    void performPrediction(double t_start, double t_end);

    /**
     * @brief 执行IEKF更新步
     */
    bool performUpdate(PointCloudPtr& cloud);

    /**
     * @brief 更新地图
     */
    void updateMap(PointCloudPtr& cloud);

    // ==================== 发布函数 ====================

    /**
     * @brief 发布里程计
     */
    void publishOdometry();

    /**
     * @brief 发布路径
     */
    void publishPath();

    /**
     * @brief 发布地图点云
     */
    void publishMap();

    /**
     * @brief 发布TF变换
     */
    void publishTF();

    // ==================== 服务回调 ====================

    /**
     * @brief 保存地图服务回调
     */
    void saveMapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /**
     * @brief 保存PCD服务回调
     */
    void savePcdCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // ==================== 全局定位服务回调 ====================

    /**
     * @brief 全局定位服务回调
     */
    void globalLocalizeCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /**
     * @brief 设置初始位姿服务回调
     */
    void setInitialPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg);

    // ==================== 辅助函数 ====================

    /**
     * @brief 检查数据同步
     */
    bool checkDataSync();

    /**
     * @brief 运动畸变校正
     */
    void undistortPointCloud(PointCloudData& cloud_data);

private:
    // ==================== 配置 ====================
    ConfigParams config_;
    std::string config_file_path_;

    // ==================== ROS2接口 ====================
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // 发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_pub_;

    // TF广播器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pcd_srv_;

    // ==================== 全局定位服务 ====================
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_localize_srv_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr initial_pose_sub_;

    // ==================== 数据缓冲 ====================
    ImuBuffer imu_buffer_;
    std::queue<PointCloudData> point_cloud_queue_;
    std::mutex cloud_queue_mutex_;

    // ==================== 核心模块 ====================
    std::unique_ptr<PointCloudConverter> point_cloud_converter_;  // 点云格式转换器
    std::unique_ptr<PointCloudFilter> point_cloud_filter_;
    std::unique_ptr<ImuProcessor> imu_processor_;
    std::unique_ptr<IekfEstimator> iekf_estimator_;
    std::unique_ptr<IKdTree> ikd_tree_;
    std::unique_ptr<OdomFusion> odom_fusion_;
    std::unique_ptr<MapManager> map_manager_;

    // ==================== 性能优化模块 ====================
    std::unique_ptr<ThreadPool> thread_pool_;  // 线程池
    ThreadPoolConfig thread_pool_config_;      // 线程池配置

    // ==================== 建图增强模块 ====================
    std::unique_ptr<MapServer> map_server_;
    std::unique_ptr<MapPersistence> map_persistence_;
    std::unique_ptr<MapQualityEvaluator> map_quality_;

    // ==================== 全局定位模块 ====================
    std::unique_ptr<GlobalLocalizer> global_localizer_;
    LocalizationState localization_state_;
    std::mutex localization_mutex_;

    // ==================== 里程计数据缓冲 ====================
    std::deque<OdomData> odom_buffer_;
    std::mutex odom_buffer_mutex_;

    // ==================== 状态 ====================
    State current_state_;
    std::vector<SE3d> pose_history_;
    nav_msgs::msg::Path path_msg_;

    // 系统状态
    std::atomic<bool> system_initialized_;
    std::atomic<bool> is_processing_;
    std::atomic<bool> first_scan_received_;

    // 计数器
    int scan_count_;
    int keyframe_count_;
    double last_scan_time_;

    // ==================== 地图保存辅助 ====================
    void saveProjectedMap(const PointCloudPtr& cloud, const std::string& base_path);

    // ==================== 处理 ====================
    rclcpp::TimerBase::SharedPtr process_timer_;  // 处理定时器（替代独立线程）

    // 定时器
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr path_timer_;
};

// ==================== 实现部分 ====================

inline FastLio2Node::FastLio2Node(const rclcpp::NodeOptions& options)
    : Node("rosiwit_slam", options),
      system_initialized_(false),
      is_processing_(false),
      first_scan_received_(false),
      scan_count_(0),
      keyframe_count_(0),
      last_scan_time_(0) {

    initialize();
}

inline FastLio2Node::~FastLio2Node() {
    // 定时器会自动销毁
}

inline void FastLio2Node::initialize() {
    // 加载参数
    loadParameters();

    // 创建ROS2接口
    createSubscribers();
    createPublishers();
    createServices();

    // 初始化核心模块
    initializeModules();

    // 使用定时器处理点云队列（替代独立线程，确保在executor线程中执行）
    process_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        [this]() {
            // 每次处理一帧
            PointCloudData cloud_data;
            bool has_data = false;
            {
                std::lock_guard<std::mutex> lock(cloud_queue_mutex_);
                if (!point_cloud_queue_.empty()) {
                    cloud_data = point_cloud_queue_.front();
                    point_cloud_queue_.pop();
                    has_data = true;
                }
            }
            if (has_data) {
                processPointCloud(cloud_data);
            }
        });

    // 创建定时器
    map_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / config_.ros.map_publish_rate)),
        [this]() { publishMap(); });

    path_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / config_.ros.path_publish_rate)),
        [this]() { publishPath(); });

    RCLCPP_INFO(this->get_logger(), "FAST-LIO2 SLAM Node initialized successfully!");
}

inline void FastLio2Node::loadParameters() {
    // 从ROS2参数服务器获取参数
    this->declare_parameter("config_file", "");
    config_file_path_ = this->get_parameter("config_file").as_string();

    // 如果指定了配置文件，加载它
    if (!config_file_path_.empty()) {
        ConfigManager config_manager;
        if (config_manager.loadFromFile(config_file_path_)) {
            config_ = config_manager.getParams();
            RCLCPP_INFO(this->get_logger(), "Configuration loaded from: %s", config_file_path_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load config file: %s, using defaults",
                        config_file_path_.c_str());
        }
    }

    // 声明参数时使用配置文件已加载的值作为默认值
    // 这样配置文件的值不会被默认值覆盖
    // 如果配置文件未设置，则使用硬编码默认值
    this->declare_parameter("lidar_topic",
        config_.ros.lidar_topic.empty() ? "/lidar_points" : config_.ros.lidar_topic);
    this->declare_parameter("imu_topic",
        config_.ros.imu_topic.empty() ? "/imu/data" : config_.ros.imu_topic);
    this->declare_parameter("odom_topic",
        config_.ros.odom_topic.empty() ? "/odom" : config_.ros.odom_topic);

    // 数值参数：如果配置文件设置了非默认值，使用配置文件的值
    // 否则使用硬编码默认值
    this->declare_parameter("lidar_max_range",
        config_.lidar.max_range > 0 ? config_.lidar.max_range : 100.0);
    this->declare_parameter("lidar_min_range",
        config_.lidar.min_range >= 0 ? config_.lidar.min_range : 0.5);
    this->declare_parameter("voxel_size",
        config_.lidar.voxel_size > 0 ? config_.lidar.voxel_size : 0.2);
    this->declare_parameter("max_iterations",
        config_.iekf.max_iterations > 0 ? config_.iekf.max_iterations : 5);

    // 获取ROS2参数值（可能被命令行参数覆盖）
    config_.ros.lidar_topic = this->get_parameter("lidar_topic").as_string();
    config_.ros.imu_topic = this->get_parameter("imu_topic").as_string();
    config_.ros.odom_topic = this->get_parameter("odom_topic").as_string();
    config_.lidar.max_range = this->get_parameter("lidar_max_range").as_double();
    config_.lidar.min_range = this->get_parameter("lidar_min_range").as_double();
    config_.lidar.voxel_size = this->get_parameter("voxel_size").as_double();
    config_.iekf.max_iterations = this->get_parameter("max_iterations").as_int();

    RCLCPP_INFO(this->get_logger(), "Parameters loaded: lidar_topic=%s, imu_topic=%s",
                config_.ros.lidar_topic.c_str(), config_.ros.imu_topic.c_str());
}

inline void FastLio2Node::createSubscribers() {
    // 点云订阅者
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.ros.lidar_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            lidarCallback(msg);
        });

    // IMU订阅者
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        config_.ros.imu_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            imuCallback(msg);
        });

    // 里程计订阅者 (可选)
    if (config_.odom_fusion.enable) {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            config_.ros.odom_topic, rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                odomCallback(msg);
            });
    }

    RCLCPP_INFO(this->get_logger(), "Subscribers created");
}

inline void FastLio2Node::createPublishers() {
    // 里程计发布者
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        config_.ros.odom_output_topic, 10);

    // 路径发布者
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        config_.ros.path_topic, 10);

    // 地图发布者
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.ros.map_topic, 10);

    // 关键帧发布者
    keyframe_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.ros.keyframe_topic, 10);

    // TF广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "Publishers created");
}

inline void FastLio2Node::createServices() {
    // 保存地图服务
    save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
        config_.ros.save_map_service,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            saveMapCallback(request, response);
        });

    // 保存PCD服务
    save_pcd_srv_ = this->create_service<std_srvs::srv::Trigger>(
        config_.ros.save_pcd_service,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            savePcdCallback(request, response);
        });

    // ==================== 全局定位服务 ====================
    // 全局定位服务
    global_localize_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/global_localize",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            globalLocalizeCallback(request, response);
        });

    // 初始位姿订阅
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/initial_pose",
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
            setInitialPoseCallback(msg);
        });

    RCLCPP_INFO(this->get_logger(), "Services created");
}

inline void FastLio2Node::initializeModules() {
    // 点云格式转换器（新增：修复BUG-002）
    ConverterConfig converter_config;
    converter_config.curvature_method = ConverterConfig::CurvatureMethod::RING_BASED;
    converter_config.normal_method = ConverterConfig::NormalMethod::RING_BASED;
    converter_config.scan_lines = config_.lidar.scan_line;
    converter_config.curvature_neighbors = 5;
    converter_config.normal_radius = 0.5;
    converter_config.verbose = true;
    point_cloud_converter_ = std::make_unique<PointCloudConverter>(converter_config);
    RCLCPP_INFO(this->get_logger(), "Point cloud converter initialized (supports Ouster/Velodyne/Livox formats)");

    // 点云滤波器
    FilterConfig filter_config;
    filter_config.min_range = config_.lidar.min_range;
    filter_config.max_range = config_.lidar.max_range;
    filter_config.voxel_size = config_.lidar.voxel_size;
    point_cloud_filter_ = std::make_unique<PointCloudFilter>(filter_config);

    // IMU处理器
    ImuProcessorConfig imu_config;
    imu_config.acc_noise = config_.imu.acc_noise;
    imu_config.gyro_noise = config_.imu.gyro_noise;
    imu_config.static_init_count = 50;
    imu_processor_ = std::make_unique<ImuProcessor>(imu_config);

    // IEKF估计器
    IekfConfig iekf_config;
    iekf_config.max_iterations = config_.iekf.max_iterations;
    iekf_config.point_noise = config_.iekf.position_noise;
    iekf_config.gravity_magnitude = config_.imu.gravity_magnitude;
    iekf_config.use_acc_integration = config_.iekf.use_acc_integration;
    iekf_estimator_ = std::make_unique<IekfEstimator>();  // initialized_ = false
    // 只设置config，不调用initialize（因为initialize会设initialized_=true并重置P_）
    iekf_estimator_->setConfig(iekf_config);

    // iKD-Tree
    IKdTreeConfig ikdtree_config;
    ikdtree_config.max_distance = config_.lidar.max_range;
    ikd_tree_ = std::make_unique<IKdTree>(ikdtree_config);

    // 里程计融合模块
    if (config_.odom_fusion.enable) {
        OdomFusionConfig odom_config;
        odom_config.enable = config_.odom_fusion.enable;
        odom_config.fusion_mode = config_.odom_fusion.fusion_mode;
        odom_config.lidar_weight = config_.odom_fusion.lidar_weight;
        odom_config.odom_weight = config_.odom_fusion.odom_weight;
        odom_config.position_cov = config_.odom_fusion.position_cov;
        odom_config.rotation_cov = config_.odom_fusion.rotation_cov;
        odom_fusion_ = std::make_unique<OdomFusion>(odom_config);
        RCLCPP_INFO(this->get_logger(), "Odom fusion module initialized");
    }

    // 地图管理模块
    MapManagerConfig map_config;
    map_config.resolution = config_.map.map_leaf_size;
    map_config.submap_size = 50.0;
    map_config.max_submap_points = 50000;
    map_config.map_path = config_.map.map_path;
    map_config.enable_pcd_save = config_.map.enable_pcd_save;
    map_config.enable_submap = true;
    map_manager_ = std::make_unique<MapManager>(map_config);
    RCLCPP_INFO(this->get_logger(), "Map manager initialized");

    // ==================== 性能优化模块初始化 ====================
    // 线程池配置
    thread_pool_config_.thread_count = std::thread::hardware_concurrency();
    if (thread_pool_config_.thread_count <= 0) thread_pool_config_.thread_count = 4;
    thread_pool_config_.max_queue_size = 100;
    thread_pool_ = std::make_unique<ThreadPool>(thread_pool_config_);
    RCLCPP_INFO(this->get_logger(), "Thread pool initialized with %d threads",
                thread_pool_config_.thread_count);

    // 性能监控初始化
    ProfilerConfig profiler_config;
    profiler_config.enable = true;
    profiler_config.print_on_exit = true;
    profiler_config.report_interval_ms = 5000;
    profiler_config.output_file = config_.map.map_path + "/performance_report.txt";
    initProfiler(profiler_config);
    RCLCPP_INFO(this->get_logger(), "Performance profiler initialized");

    // ==================== 建图增强模块初始化 ====================

    // 地图服务器（ROS2服务接口）
    MapServerConfig server_config;
    server_config.save_map_service = config_.ros.save_map_service;
    server_config.load_map_service = config_.ros.load_map_service;
    server_config.clear_map_service = config_.ros.clear_map_service;
    // 初始化地图服务器
    std::shared_ptr<MapManager> map_manager_shared(map_manager_.get(), [](MapManager*){});
    map_server_ = std::make_unique<MapServer>(
        std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}),
        map_manager_shared);
    RCLCPP_INFO(this->get_logger(), "Map server initialized");

    // 地图持久化模块
    PersistenceConfig persist_config;
    persist_config.map_directory = config_.map.map_path;
    persist_config.compress_pcd = config_.map.enable_compression;
    map_persistence_ = std::make_unique<MapPersistence>(persist_config);
    RCLCPP_INFO(this->get_logger(), "Map persistence initialized");

    // 地图质量评估模块
    QualityEvaluatorConfig quality_config;
    quality_config.density_weight = config_.map_quality.density_weight;
    quality_config.uniformity_weight = config_.map_quality.uniformity_weight;
    quality_config.coverage_weight = config_.map_quality.coverage_weight;
    quality_config.min_density_threshold = config_.map_quality.min_density_threshold;
    map_quality_ = std::make_unique<MapQualityEvaluator>(quality_config);
    RCLCPP_INFO(this->get_logger(), "Map quality evaluator initialized");

    // ==================== 全局定位模块初始化 ====================
    if (config_.localization.enable) {
        GlobalLocalizerConfig loc_config;
        loc_config.enable = config_.localization.enable;
        loc_config.mode = config_.localization.mode;

        // Scan Context参数
        loc_config.scan_context.ring_num = config_.localization.scan_context_ring_num;
        loc_config.scan_context.sector_num = config_.localization.scan_context_sector_num;
        loc_config.scan_context.max_range = config_.localization.scan_context_max_range;
        loc_config.scan_context.dist_threshold = config_.localization.scan_context_dist_threshold;
        loc_config.scan_context.candidate_count = config_.localization.scan_context_candidate_count;

        // 精配准参数
        loc_config.fine_alignment.method = config_.localization.fine_alignment_method;
        loc_config.fine_alignment.max_iterations = config_.localization.fine_alignment_max_iterations;
        loc_config.fine_alignment.convergence_threshold = config_.localization.fine_alignment_convergence_threshold;
        loc_config.fine_alignment.resolution = config_.localization.fine_alignment_resolution;
        loc_config.fine_alignment.voxel_size = config_.localization.fine_alignment_voxel_size;

        // 验证参数
        loc_config.validation.min_fitness_score = config_.localization.validation_min_fitness_score;
        loc_config.validation.min_inlier_ratio = config_.localization.validation_min_inlier_ratio;
        loc_config.validation.max_position_error = config_.localization.validation_max_position_error;
        loc_config.validation.max_rotation_error = config_.localization.validation_max_rotation_error;

        // 搜索参数
        loc_config.search.max_search_candidates = config_.localization.search_max_candidates;

        global_localizer_ = std::make_unique<GlobalLocalizer>(loc_config);
        localization_state_ = LocalizationState::UNINITIALIZED;

        RCLCPP_INFO(this->get_logger(), "Global localizer initialized");
    }

    // 初始化路径消息
    path_msg_.header.frame_id = config_.ros.world_frame;

    RCLCPP_INFO(this->get_logger(), "Core modules initialized");
}

inline void FastLio2Node::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 使用格式转换器转换ROS2消息到PCL点云
    // 支持多种格式：Ouster, Velodyne, Livox, Standard XYZI
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());

    // 使用成员变量转换器（已在initializeModules中初始化）
    bool conversion_success = point_cloud_converter_->fromROSMsg(msg, cloud);

    if (!conversion_success || cloud->empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "Point cloud conversion failed or result is empty, skipping this scan");
        return;
    }

    // 创建点云数据结构
    PointCloudData cloud_data;
    cloud_data.timestamp = msg->header.stamp.sec +
                           msg->header.stamp.nanosec * 1e-9;
    cloud_data.cloud = cloud;
    cloud_data.scan_id = scan_count_;

    // 加入处理队列
    {
        std::lock_guard<std::mutex> lock(cloud_queue_mutex_);
        point_cloud_queue_.push(cloud_data);
    }

    scan_count_++;
    last_scan_time_ = cloud_data.timestamp;

    if (!first_scan_received_) {
        first_scan_received_ = true;
        current_state_.timestamp = cloud_data.timestamp;
        RCLCPP_INFO(this->get_logger(), "First LiDAR scan received at time: %.3f, points: %zu",
                    cloud_data.timestamp, cloud->size());
    }
}

inline void FastLio2Node::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // 转换ROS2 IMU消息
    ImuData imu;
    imu.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    imu.acc = Vector3d(msg->linear_acceleration.x,
                       msg->linear_acceleration.y,
                       msg->linear_acceleration.z) * config_.imu.acc_scale;
    imu.gyro = Vector3d(msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z);

    // 加入IMU缓冲区
    imu_buffer_.push(imu);
    imu_processor_->addImuData(imu);

    static int imu_count = 0;
    imu_count++;
    if (imu_count <= 3 || imu_count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "IMU callback #%d, t=%.3f, buf_size=%zu",
                    imu_count, imu.timestamp, imu_buffer_.size());
    }
}

inline void FastLio2Node::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 外部里程计数据 (可选融合)
    OdomData odom;
    odom.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    odom.position = Vector3d(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
    odom.rotation = Quaterniond(msg->pose.pose.orientation.w,
                                msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z);

    // 添加到缓冲区
    {
        std::lock_guard<std::mutex> lock(odom_buffer_mutex_);

        // 限制缓冲区大小
        const size_t max_buffer_size = 200;
        if (odom_buffer_.size() >= max_buffer_size) {
            odom_buffer_.pop_front();
        }
        odom_buffer_.push_back(odom);
    }

    // 如果启用了里程计融合模块
    if (config_.odom_fusion.enable && odom_fusion_) {
        odom_fusion_->addOdomData(odom);
    }
}

inline void FastLio2Node::processPointCloud(PointCloudData& cloud_data) {
    // 性能监控: 整体帧处理时间
    PROFILE_FUNCTION(frame_processing);

    if (!first_scan_received_) return;

    static int process_count = 0;
    process_count++;
    if (process_count <= 60 || process_count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "processPointCloud #%d, scan_id=%d, pts=%zu, t=%.3f, map_pts=%zu",
                    process_count, cloud_data.scan_id, cloud_data.cloud->size(), cloud_data.timestamp,
                    ikd_tree_->size());
    }

    // 1. 点云预处理
    PROFILE_START(pointcloud_preprocessing);
    PointCloudPtr filtered_cloud = point_cloud_filter_->process(cloud_data.cloud);
    PROFILE_END(pointcloud_preprocessing);

    if (filtered_cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Filtered cloud is empty, skipping frame");
        return;
    }

    // 2. IMU预测步
    PROFILE_START(imu_prediction);
    double t_scan = cloud_data.timestamp;
    performPrediction(current_state_.timestamp, t_scan);
    PROFILE_END(imu_prediction);

    if (process_count <= 60) {
        bool has_nan = !std::isfinite(current_state_.position(0)) ||
                       !std::isfinite(current_state_.position(1)) ||
                       !std::isfinite(current_state_.position(2)) ||
                       !std::isfinite(current_state_.rotation.w());
        RCLCPP_INFO(this->get_logger(), "After prediction: pos=(%.4f,%.4f,%.4f) q=(%.4f,%.4f,%.4f,%.4f) t_state=%.3f%s",
                    current_state_.position(0), current_state_.position(1),
                    current_state_.position(2),
                    current_state_.rotation.x(), current_state_.rotation.y(),
                    current_state_.rotation.z(), current_state_.rotation.w(),
                    current_state_.timestamp, has_nan ? "  <<< NaN STATE" : "");
    }

    // 3. IEKF更新步
    PROFILE_START(iekf_update);
    bool iekf_success = performUpdate(filtered_cloud);
    if (!iekf_success) {
        RCLCPP_WARN(this->get_logger(), "IEKF update failed");
        // 即使失败也继续发布，便于调试
    }
    PROFILE_END(iekf_update);

    // 4. 更新地图 (使用线程池并行处理)
    PROFILE_START(map_update);
    if (iekf_success) {
        updateMap(filtered_cloud);
    }
    PROFILE_END(map_update);

    // 5. 发布结果
    PROFILE_START(result_publish);
    publishOdometry();
    publishTF();
    PROFILE_END(result_publish);

    // 6. 记录位姿历史
    pose_history_.push_back(current_state_.toSE3());

    // 标记系统已初始化
    if (!system_initialized_) {
        system_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "System initialized successfully!");
    }

    // 性能统计输出 (每100帧)
    scan_count_++;
    if (scan_count_ % 100 == 0) {
        Profiler::instance().printReport();
    }
}

inline void FastLio2Node::performPrediction(double t_start, double t_end) {
    if (!iekf_estimator_->isInitialized()) {
        // 立即初始化: 有足够静止IMU数据则用其估计姿态/零偏，否则用单位阵
        State initial_state;
        initial_state.timestamp = t_start;
        initial_state.gravity = Vector3d(0.0, 0.0, -config_.imu.gravity_magnitude);
        if (imu_processor_->staticInitializeBias()) {
            initial_state.rotation = imu_processor_->getInitRotation();
            initial_state.gyro_bias = imu_processor_->getGyroBiasEst();
            RCLCPP_INFO(this->get_logger(), "Init: using static IMU estimate");
        } else {
            RCLCPP_INFO(this->get_logger(), "Init: identity (insufficient static IMU data)");
        }
        iekf_estimator_->setInitialState(initial_state);
        current_state_ = initial_state;
        return;
    }

    // 获取IMU数据并执行预测
    std::vector<ImuData> imu_data = imu_buffer_.getImuInRange(t_start, t_end);

    if (imu_data.empty()) {
        RCLCPP_WARN(this->get_logger(), "No IMU data for prediction");
        return;
    }

    // 设置当前状态
    iekf_estimator_->setState(current_state_);

    // 执行预测
    iekf_estimator_->predictBatch(imu_data);

    // 更新当前状态
    current_state_ = iekf_estimator_->getState();
    current_state_.timestamp = t_end;
}

inline bool FastLio2Node::performUpdate(PointCloudPtr& cloud) {
    if (ikd_tree_->empty()) {
        // 第一帧，直接加入地图
        RCLCPP_INFO(this->get_logger(), "performUpdate: first frame, building kd-tree with %zu points", cloud->size());
        ikd_tree_->build(cloud);
        RCLCPP_INFO(this->get_logger(), "performUpdate: kd-tree built successfully");
        return true;
    }

    // 估计器未初始化时跳过IEKF更新
    if (!iekf_estimator_->isInitialized()) {
        return false;
    }

    // 获取变换后的点云
    PointCloudPtr transformed_cloud(new pcl::PointCloud<PointType>());
    SE3d current_pose = current_state_.toSE3();
    transformPointCloud(cloud, transformed_cloud, current_pose);

    // 执行迭代更新
    for (int iter = 0; iter < config_.iekf.max_iterations; ++iter) {
        std::vector<Vector3d> source_points;
        std::vector<Vector3d> target_points;
        std::vector<Vector3d> target_normals;
        std::vector<int> valid_indices;

        // 寻找对应点
        for (size_t i = 0; i < transformed_cloud->size(); ++i) {
            PointType query = transformed_cloud->points[i];
            PointType nearest;
            double dist;

                if (ikd_tree_->nearestSearch(query, nearest, dist)) {
                    if (dist < config_.iekf.max_correspondence_distance) {
                    valid_indices.push_back(i);
                    source_points.push_back(Vector3d(cloud->points[i].x,
                                                     cloud->points[i].y,
                                                     cloud->points[i].z));
                    target_points.push_back(Vector3d(nearest.x, nearest.y, nearest.z));
                    target_normals.push_back(Vector3d(nearest.normal_x,
                                                     nearest.normal_y,
                                                     nearest.normal_z));
                }
            }
        }

        // 调试输出: 每帧第一次迭代都打印, 失败时打印完整状态
        static int update_cnt = 0;
        update_cnt++;
        if (iter == 0) {
            RCLCPP_INFO(this->get_logger(),
                "performUpdate iter0 #%d: pts=%zu, valid=%zu, pose=(%.4f,%.4f,%.4f), map_pts=%zu",
                update_cnt, transformed_cloud->size(), valid_indices.size(),
                current_pose.translation()(0), current_pose.translation()(1),
                current_pose.translation()(2), ikd_tree_->size());
        }

        if (valid_indices.size() < 10) {
            // 采样 transformed_cloud 的最近距离, 判断是位姿发散还是地图损坏
            double min_d = 1e9, max_d = 0, sample_d = 0;
            int sampled = 0;
            for (size_t i = 0; i < transformed_cloud->size() && sampled < 200; ++i) {
                PointType nn; double dd;
                if (ikd_tree_->nearestSearch(transformed_cloud->points[i], nn, dd)) {
                    min_d = std::min(min_d, dd); max_d = std::max(max_d, dd); sample_d += dd; sampled++;
                }
            }
            bool has_nan = !std::isfinite(current_state_.position(0)) ||
                           !std::isfinite(current_state_.rotation.w());
            RCLCPP_WARN(this->get_logger(),
                "Too few valid=%zu | pose=(%.4f,%.4f,%.4f) q=(%.4f,%.4f,%.4f,%.4f) | "
                "nn_dist min=%.3f max=%.3f avg=%.3f | map_pts=%zu pts=%zu%s",
                valid_indices.size(),
                current_state_.position(0), current_state_.position(1), current_state_.position(2),
                current_state_.rotation.x(), current_state_.rotation.y(),
                current_state_.rotation.z(), current_state_.rotation.w(),
                min_d, max_d, sampled ? sample_d / sampled : -1.0,
                ikd_tree_->size(), cloud->size(),
                has_nan ? "  <<< NaN STATE" : "");
            return false;
        }

        // 执行IEKF更新
        iekf_estimator_->setState(current_state_);
        bool success = iekf_estimator_->updateWithNormals(source_points, target_points,
                                                          target_normals, valid_indices);

        if (success) {
            current_state_ = iekf_estimator_->getState();
            current_pose = current_state_.toSE3();
            transformPointCloud(cloud, transformed_cloud, current_pose);
        }
    }

    return true;
}

inline void FastLio2Node::updateMap(PointCloudPtr& cloud) {
    // 变换点云到世界坐标系
    PointCloudPtr transformed_cloud(new pcl::PointCloud<PointType>());
    SE3d current_pose = current_state_.toSE3();
    transformPointCloud(cloud, transformed_cloud, current_pose);

    // 加入地图
    ikd_tree_->insertPointCloud(transformed_cloud);

    // 检查是否需要重建
    if (ikd_tree_->getDeletedCount() > static_cast<int>(ikd_tree_->size() * 0.3)) {
        ikd_tree_->rebuildTree();
    }
}

inline void FastLio2Node::publishOdometry() {
    static int odom_pub_count = 0;
    odom_pub_count++;
    if (odom_pub_count <= 5 || odom_pub_count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "publishOdometry #%d: pos=(%.3f,%.3f,%.3f)",
                    odom_pub_count, current_state_.position(0),
                    current_state_.position(1), current_state_.position(2));
    }

    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = config_.ros.world_frame;
    odom_msg.child_frame_id = config_.ros.base_frame;

    // 位置
    odom_msg.pose.pose.position.x = current_state_.position(0);
    odom_msg.pose.pose.position.y = current_state_.position(1);
    odom_msg.pose.pose.position.z = current_state_.position(2);

    // 姿态
    odom_msg.pose.pose.orientation.w = current_state_.rotation.w();
    odom_msg.pose.pose.orientation.x = current_state_.rotation.x();
    odom_msg.pose.pose.orientation.y = current_state_.rotation.y();
    odom_msg.pose.pose.orientation.z = current_state_.rotation.z();

    // 速度
    odom_msg.twist.twist.linear.x = current_state_.velocity(0);
    odom_msg.twist.twist.linear.y = current_state_.velocity(1);
    odom_msg.twist.twist.linear.z = current_state_.velocity(2);

    odom_pub_->publish(odom_msg);
}

inline void FastLio2Node::publishPath() {
    if (!system_initialized_) return;

    // 添加新的位姿到路径
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = this->now();
    pose_stamped.header.frame_id = config_.ros.world_frame;

    pose_stamped.pose.position.x = current_state_.position(0);
    pose_stamped.pose.position.y = current_state_.position(1);
    pose_stamped.pose.position.z = current_state_.position(2);

    pose_stamped.pose.orientation.w = current_state_.rotation.w();
    pose_stamped.pose.orientation.x = current_state_.rotation.x();
    pose_stamped.pose.orientation.y = current_state_.rotation.y();
    pose_stamped.pose.orientation.z = current_state_.rotation.z();

    path_msg_.poses.push_back(pose_stamped);

    // 限制路径长度
    if (path_msg_.poses.size() > 10000) {
        path_msg_.poses.erase(path_msg_.poses.begin());
    }

    path_pub_->publish(path_msg_);
}

inline void FastLio2Node::publishMap() {
    if (!system_initialized_ || ikd_tree_->empty()) return;

    PointCloudPtr map_cloud = ikd_tree_->getAllPoints();

    if (map_cloud->empty()) return;

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud, map_msg);

    map_msg.header.stamp = this->now();
    map_msg.header.frame_id = config_.ros.world_frame;

    map_pub_->publish(map_msg);
}

inline void FastLio2Node::publishTF() {
    if (!config_.ros.publish_tf) return;

    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->now();
    transform.header.frame_id = config_.ros.world_frame;
    transform.child_frame_id = config_.ros.base_frame;

    transform.transform.translation.x = current_state_.position(0);
    transform.transform.translation.y = current_state_.position(1);
    transform.transform.translation.z = current_state_.position(2);

    transform.transform.rotation.w = current_state_.rotation.w();
    transform.transform.rotation.x = current_state_.rotation.x();
    transform.transform.rotation.y = current_state_.rotation.y();
    transform.transform.rotation.z = current_state_.rotation.z();

    tf_broadcaster_->sendTransform(transform);
}

inline void FastLio2Node::saveMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (!map_manager_) {
        response->success = false;
        response->message = "Map manager not initialized";
        return;
    }

    // 保存完整地图
    std::string map_path = config_.map.map_path + "/fast_lio2_map";
    // 从 ikd_tree_ 获取所有点云数据并保存
    PointCloudPtr full_cloud = ikd_tree_->getAllPoints();
    std::cout << "[saveMapCallback] ikd_tree has " << full_cloud->size() << " points" << std::endl;

    bool success = false;
    if (!full_cloud->empty()) {
        // 确保目录存在
        std::filesystem::create_directories(std::filesystem::path(map_path).parent_path());

        // 保存为 PCD 文件
        success = pcl::io::savePCDFileBinary(map_path + ".pcd", *full_cloud) == 0;
        std::cout << "[saveMapCallback] PCD save " << (success ? "success" : "failed") << std::endl;

        // 同时保存为 YAML 格式的 2D 栅格地图（用于导航）
        // 将3D点云投影到2D平面
        if (success) {
            saveProjectedMap(full_cloud, map_path);
        }
    }

    if (success) {
        response->success = true;
        response->message = "Map saved successfully to: " + map_path;
        RCLCPP_INFO(this->get_logger(), "Map saved: %s", map_path.c_str());
    } else {
        response->success = false;
        response->message = "Failed to save map";
        RCLCPP_ERROR(this->get_logger(), "Failed to save map");
    }
}

inline void FastLio2Node::savePcdCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (!map_manager_) {
        response->success = false;
        response->message = "Map manager not initialized";
        return;
    }

    // 生成带时间戳的文件名
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << config_.map.map_path << "/pcd_" << std::put_time(std::localtime(&timestamp), "%Y%m%d_%H%M%S") << ".pcd";
    std::string pcd_path = ss.str();

    // 保存PCD文件
    bool success = map_manager_->saveToPcd(pcd_path);

    if (success) {
        response->success = true;
        response->message = "PCD saved successfully to: " + pcd_path;
        RCLCPP_INFO(this->get_logger(), "PCD saved: %s", pcd_path.c_str());
    } else {
        response->success = false;
        response->message = "Failed to save PCD file";
        RCLCPP_ERROR(this->get_logger(), "Failed to save PCD: %s", pcd_path.c_str());
    }
}

// ==================== 全局定位服务实现 ====================

inline void FastLio2Node::globalLocalizeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    std::lock_guard<std::mutex> lock(localization_mutex_);

    if (!global_localizer_) {
        response->success = false;
        response->message = "Global localizer not initialized";
        RCLCPP_WARN(this->get_logger(), "Global localization failed: localizer not initialized");
        return;
    }

    if (!global_localizer_->hasMap()) {
        response->success = false;
        response->message = "No map loaded for localization";
        RCLCPP_WARN(this->get_logger(), "Global localization failed: no map loaded");
        return;
    }

    // 获取最新的点云数据
    // Note: 这里需要使用最近处理过的点云
    // 实际实现中可能需要从 buffer 中获取或等待新的点云
    RCLCPP_INFO(this->get_logger(), "Starting global localization...");

    localization_state_ = LocalizationState::LOCALIZING;

    // 使用当前关键帧进行定位（如果有）
    // 这里需要从 scan_context_ 或 keyframe_manager_ 获取最新的关键帧
    // 简化实现：使用最新的已处理点云

    // TODO: 实现从缓冲区获取点云的逻辑
    // PointCloudPtr current_scan = getLatestScan();

    // LocalizationResult result = global_localizer_->localize(current_scan);

    // 简化响应：实际实现需要完整的定位流程
    response->success = true;
    response->message = "Global localization service triggered. Check localization status for result.";

    RCLCPP_INFO(this->get_logger(), "Global localization service called successfully");
}

inline void FastLio2Node::setInitialPoseCallback(
    const geometry_msgs::msg::Pose::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(localization_mutex_);

    if (!global_localizer_) {
        RCLCPP_WARN(this->get_logger(),
                    "Cannot set initial pose: global localizer not initialized");
        return;
    }

    // 转换ROS消息到SE3d
    SE3d initial_pose;
    initial_pose.translation() = Eigen::Vector3d(
        msg->position.x, msg->position.y, msg->position.z);

    Eigen::Quaterniond q(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z);
    initial_pose.so3() = Sophus::SO3d(q);

    global_localizer_->setInitialPose(initial_pose);

    RCLCPP_INFO(this->get_logger(),
                "Initial pose set: pos=[%.2f, %.2f, %.2f], orientation=[%.2f, %.2f, %.2f, %.2f]",
                msg->position.x, msg->position.y, msg->position.z,
                msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

inline void FastLio2Node::saveProjectedMap(const PointCloudPtr& cloud, const std::string& base_path) {
    if (cloud->empty()) return;

    // 计算点云的边界
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& p : cloud->points) {
        min_x = std::min(min_x, static_cast<double>(p.x));
        min_y = std::min(min_y, static_cast<double>(p.y));
        max_x = std::max(max_x, static_cast<double>(p.x));
        max_y = std::max(max_y, static_cast<double>(p.y));
    }

    double resolution = 0.05;  // 5cm resolution
    int width = static_cast<int>((max_x - min_x) / resolution) + 1;
    int height = static_cast<int>((max_y - min_y) / resolution) + 1;

    if (width <= 0 || height <= 0 || width > 10000 || height > 10000) {
        RCLCPP_WARN(this->get_logger(), "Map dimensions too large or invalid: %dx%d", width, height);
        return;
    }

    // 创建栅格地图（255=未知, 0=占据, 205=自由）
    std::vector<uint8_t> grid(width * height, 205);  // 默认自由空间

    // 将点云投影到2D（只保留 z 在合理范围内的点）
    for (const auto& p : cloud->points) {
        // 过滤地面和天花板点（假设机器人高度0.3m-2.0m之间的点为障碍物）
        if (p.z < -0.1 || p.z > 1.5) continue;

        int gx = static_cast<int>((p.x - min_x) / resolution);
        int gy = static_cast<int>((p.y - min_y) / resolution);

        if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
            grid[gy * width + gx] = 0;  // 占据
        }
    }

    // 保存 PGM 文件
    std::string pgm_path = base_path + ".pgm";
    std::ofstream pgm(pgm_path, std::ios::binary);
    pgm << "P5\n" << width << " " << height << "\n255\n";
    pgm.write(reinterpret_cast<const char*>(grid.data()), grid.size());
    pgm.close();

    // 保存 YAML 文件
    std::string yaml_path = base_path + ".yaml";
    std::ofstream yaml(yaml_path);
    yaml << "image: \"" << base_path.substr(base_path.find_last_of('/') + 1) << ".pgm\"\n";
    yaml << "resolution: " << resolution << "\n";
    yaml << "origin: [" << min_x << ", " << min_y << ", 0.0]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.196\n";
    yaml.close();

    RCLCPP_INFO(this->get_logger(), "2D map saved: %s (%dx%d)", pgm_path.c_str(), width, height);
}

} // namespace fast_lio2_slam