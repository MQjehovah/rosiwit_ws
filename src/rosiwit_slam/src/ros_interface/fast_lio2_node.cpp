/**
 * @file fast_lio2_node.cpp
 * @brief FAST-LIO2 SLAM ROS2节点实现
 */

#include "fast_lio2_slam/ros_interface/fast_lio2_node.h"
#include "fast_lio2_slam/common/utils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace fast_lio2_slam {

FastLio2Node::FastLio2Node(const rclcpp::NodeOptions& options)
    : Node("rosiwit_slam", options),
      system_initialized_(false),
      is_processing_(false),
      first_scan_received_(false),
      scan_count_(0),
      keyframe_count_(0),
      last_scan_time_(0),
      is_stationary_(true),
      stationary_frame_count_(0),
      imu_acc_variance_(0.0),
      imu_gyro_variance_(0.0),
      last_imu_acc_mean_(Vector3d::Zero()),
      last_imu_gyro_mean_(Vector3d::Zero()),
      imu_variance_sample_count_(0) {
    initialize();
}

FastLio2Node::~FastLio2Node() = default;

void FastLio2Node::initialize() {
    loadParameters();
    createSubscribers();
    createPublishers();
    createServices();
    initializeModules();

    process_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        [this]() {
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

    map_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / config_.ros.map_publish_rate)),
        [this]() { publishMap(); });

    path_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / config_.ros.path_publish_rate)),
        [this]() { publishPath(); });

    RCLCPP_INFO(this->get_logger(), "FAST-LIO2 SLAM Node initialized successfully!");
}

void FastLio2Node::loadParameters() {
    this->declare_parameter("config_file", "");
    config_file_path_ = this->get_parameter("config_file").as_string();

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

    this->declare_parameter("lidar_topic",
        config_.ros.lidar_topic.empty() ? "/lidar_points" : config_.ros.lidar_topic);
    this->declare_parameter("imu_topic",
        config_.ros.imu_topic.empty() ? "/imu/data" : config_.ros.imu_topic);
    this->declare_parameter("odom_topic",
        config_.ros.odom_topic.empty() ? "/odom" : config_.ros.odom_topic);

    this->declare_parameter("lidar_max_range",
        config_.lidar.max_range > 0 ? config_.lidar.max_range : 100.0);
    this->declare_parameter("lidar_min_range",
        config_.lidar.min_range >= 0 ? config_.lidar.min_range : 0.5);
    this->declare_parameter("voxel_size",
        config_.lidar.voxel_size > 0 ? config_.lidar.voxel_size : 0.2);
    this->declare_parameter("max_iterations",
        config_.iekf.max_iterations > 0 ? config_.iekf.max_iterations : 5);
    this->declare_parameter("use_acc_integration",
        config_.iekf.use_acc_integration);

    config_.ros.lidar_topic = this->get_parameter("lidar_topic").as_string();
    config_.ros.imu_topic = this->get_parameter("imu_topic").as_string();
    config_.ros.odom_topic = this->get_parameter("odom_topic").as_string();
    config_.lidar.max_range = this->get_parameter("lidar_max_range").as_double();
    config_.lidar.min_range = this->get_parameter("lidar_min_range").as_double();
    config_.lidar.voxel_size = this->get_parameter("voxel_size").as_double();
    config_.iekf.max_iterations = this->get_parameter("max_iterations").as_int();
    config_.iekf.use_acc_integration = this->get_parameter("use_acc_integration").as_bool();

    RCLCPP_INFO(this->get_logger(), "Parameters loaded: lidar_topic=%s, imu_topic=%s",
                config_.ros.lidar_topic.c_str(), config_.ros.imu_topic.c_str());
}

void FastLio2Node::createSubscribers() {
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.ros.lidar_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { lidarCallback(msg); });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        config_.ros.imu_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });

    if (config_.odom_fusion.enable) {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            config_.ros.odom_topic, rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });
    }

    RCLCPP_INFO(this->get_logger(), "Subscribers created");
}

void FastLio2Node::createPublishers() {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(config_.ros.odom_output_topic, 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(config_.ros.path_topic, 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(config_.ros.map_topic, 10);
    keyframe_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(config_.ros.keyframe_topic, 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "Publishers created");
}

void FastLio2Node::createServices() {
    save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
        config_.ros.save_map_service,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            saveMapCallback(request, response);
        });

    save_pcd_srv_ = this->create_service<std_srvs::srv::Trigger>(
        config_.ros.save_pcd_service,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            savePcdCallback(request, response);
        });

    global_localize_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/global_localize",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            globalLocalizeCallback(request, response);
        });

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/initial_pose", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) { setInitialPoseCallback(msg); });

    RCLCPP_INFO(this->get_logger(), "Services created");
}

void FastLio2Node::initializeModules() {
    // LiDAR->IMU 外参 (从配置读取, 欧拉角转旋转矩阵)
    R_il_ = eulerToRotationMatrix(config_.lidar.rotation_euler);
    t_il_ = config_.lidar.translation;

    ConverterConfig converter_config;
    converter_config.curvature_method = ConverterConfig::CurvatureMethod::NEIGHBOR_BASED;
    converter_config.normal_method = ConverterConfig::NormalMethod::PCA_BASED;
    converter_config.scan_lines = config_.lidar.scan_line;
    converter_config.curvature_neighbors = 5;
    converter_config.normal_radius = 1.0;
    converter_config.verbose = true;
    point_cloud_converter_ = std::make_unique<PointCloudConverter>(converter_config);
    RCLCPP_INFO(this->get_logger(), "Point cloud converter initialized (supports Ouster/Velodyne/Livox formats)");

    FilterConfig filter_config;
    filter_config.min_range = config_.lidar.min_range;
    filter_config.max_range = config_.lidar.max_range;
    filter_config.voxel_size = config_.lidar.voxel_size;
    point_cloud_filter_ = std::make_unique<PointCloudFilter>(filter_config);

    ImuProcessorConfig imu_config;
    imu_config.acc_noise = config_.imu.acc_noise;
    imu_config.gyro_noise = config_.imu.gyro_noise;
    imu_config.static_init_count = 200;
    imu_processor_ = std::make_unique<ImuProcessor>(imu_config);

    IekfConfig iekf_config;
    iekf_config.max_iterations = config_.iekf.max_iterations;
    iekf_config.point_noise = config_.iekf.position_noise;
    iekf_config.gravity_magnitude = config_.imu.gravity_magnitude;
    iekf_config.use_acc_integration = config_.iekf.use_acc_integration;
    iekf_estimator_ = std::make_unique<IekfEstimator>();
    iekf_estimator_->setConfig(iekf_config);

    IKdTreeConfig ikdtree_config;
    ikdtree_config.max_distance = config_.lidar.max_range;
    ikd_tree_ = std::make_unique<IKdTree>(ikdtree_config);

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

    MapManagerConfig map_config;
    map_config.resolution = config_.map.map_leaf_size;
    map_config.submap_size = 50.0;
    map_config.max_submap_points = 50000;
    map_config.map_path = config_.map.map_path;
    map_config.enable_pcd_save = config_.map.enable_pcd_save;
    map_config.enable_submap = true;
    map_manager_ = std::make_unique<MapManager>(map_config);
    RCLCPP_INFO(this->get_logger(), "Map manager initialized");

    thread_pool_config_.thread_count = std::thread::hardware_concurrency();
    if (thread_pool_config_.thread_count <= 0) thread_pool_config_.thread_count = 4;
    thread_pool_config_.max_queue_size = 100;
    thread_pool_ = std::make_unique<ThreadPool>(thread_pool_config_);
    RCLCPP_INFO(this->get_logger(), "Thread pool initialized with %d threads",
                thread_pool_config_.thread_count);

    ProfilerConfig profiler_config;
    profiler_config.enable = true;
    profiler_config.print_on_exit = true;
    profiler_config.report_interval_ms = 5000;
    profiler_config.output_file = config_.map.map_path + "/performance_report.txt";
    initProfiler(profiler_config);
    RCLCPP_INFO(this->get_logger(), "Performance profiler initialized");

    MapServerConfig server_config;
    server_config.save_map_service = config_.ros.save_map_service;
    server_config.load_map_service = config_.ros.load_map_service;
    server_config.clear_map_service = config_.ros.clear_map_service;
    std::shared_ptr<MapManager> map_manager_shared(map_manager_.get(), [](MapManager*){});
    map_server_ = std::make_unique<MapServer>(
        std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}),
        map_manager_shared);
    RCLCPP_INFO(this->get_logger(), "Map server initialized");

    PersistenceConfig persist_config;
    persist_config.map_directory = config_.map.map_path;
    persist_config.compress_pcd = config_.map.enable_compression;
    map_persistence_ = std::make_unique<MapPersistence>(persist_config);
    RCLCPP_INFO(this->get_logger(), "Map persistence initialized");

    QualityEvaluatorConfig quality_config;
    quality_config.density_weight = config_.map_quality.density_weight;
    quality_config.uniformity_weight = config_.map_quality.uniformity_weight;
    quality_config.coverage_weight = config_.map_quality.coverage_weight;
    quality_config.min_density_threshold = config_.map_quality.min_density_threshold;
    map_quality_ = std::make_unique<MapQualityEvaluator>(quality_config);
    RCLCPP_INFO(this->get_logger(), "Map quality evaluator initialized");

    if (config_.localization.enable) {
        GlobalLocalizerConfig loc_config;
        loc_config.enable = config_.localization.enable;
        loc_config.mode = config_.localization.mode;
        loc_config.scan_context.ring_num = config_.localization.scan_context_ring_num;
        loc_config.scan_context.sector_num = config_.localization.scan_context_sector_num;
        loc_config.scan_context.max_range = config_.localization.scan_context_max_range;
        loc_config.scan_context.dist_threshold = config_.localization.scan_context_dist_threshold;
        loc_config.scan_context.candidate_count = config_.localization.scan_context_candidate_count;
        loc_config.fine_alignment.method = config_.localization.fine_alignment_method;
        loc_config.fine_alignment.max_iterations = config_.localization.fine_alignment_max_iterations;
        loc_config.fine_alignment.convergence_threshold = config_.localization.fine_alignment_convergence_threshold;
        loc_config.fine_alignment.resolution = config_.localization.fine_alignment_resolution;
        loc_config.fine_alignment.voxel_size = config_.localization.fine_alignment_voxel_size;
        loc_config.validation.min_fitness_score = config_.localization.validation_min_fitness_score;
        loc_config.validation.min_inlier_ratio = config_.localization.validation_min_inlier_ratio;
        loc_config.validation.max_position_error = config_.localization.validation_max_position_error;
        loc_config.validation.max_rotation_error = config_.localization.validation_max_rotation_error;
        loc_config.search.max_search_candidates = config_.localization.search_max_candidates;
        global_localizer_ = std::make_unique<GlobalLocalizer>(loc_config);
        localization_state_ = LocalizationState::UNINITIALIZED;
        RCLCPP_INFO(this->get_logger(), "Global localizer initialized");
    }

    // 初始化闭环检测模块
    if (config_.loop_closure.enable) {
        ScanContextConfig sc_config;
        sc_config.ring_num = config_.loop_closure.ring_num;
        sc_config.sector_num = config_.loop_closure.sector_num;
        sc_config.max_range = config_.loop_closure.max_range;
        sc_config.threshold = config_.loop_closure.threshold;
        sc_config.exclude_near_scan = config_.loop_closure.min_interval;
        scan_context_ = std::make_unique<ScanContext>(sc_config);
        RCLCPP_INFO(this->get_logger(), "ScanContext loop closure detector initialized");

        GtsamBackendConfig gtsam_config;
        gtsam_config.max_iterations = config_.loop_closure.max_iterations;
        gtsam_config.relative_pose_noise = config_.loop_closure.relative_pose_noise;
        gtsam_backend_ = std::make_unique<GtsamBackend>(gtsam_config);
        RCLCPP_INFO(this->get_logger(), "GTSAM backend initialized");
    }
    last_loop_detect_frame_ = 0;

    path_msg_.header.frame_id = config_.ros.world_frame;
    RCLCPP_INFO(this->get_logger(), "Core modules initialized");
}

void FastLio2Node::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    bool conversion_success = point_cloud_converter_->fromROSMsg(msg, cloud);

    if (!conversion_success || cloud->empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "Point cloud conversion failed or result is empty, skipping this scan");
        return;
    }

    PointCloudData cloud_data;
    cloud_data.timestamp = msg->header.stamp.sec +
                           msg->header.stamp.nanosec * 1e-9;
    cloud_data.cloud = cloud;
    cloud_data.scan_id = scan_count_;

    {
        std::lock_guard<std::mutex> lock(cloud_queue_mutex_);
        point_cloud_queue_.push(cloud_data);
    }

    scan_count_++;
    last_scan_time_ = cloud_data.timestamp;

    if (!first_scan_received_) {
        first_scan_received_ = true;
        current_state_.timestamp = cloud_data.timestamp;
        RCLCPP_INFO(this->get_logger(), "First LiDAR scan processed at time: %.3f, points: %zu (IMU samples: %zu)",
                    cloud_data.timestamp, cloud->size(), imu_buffer_.size());
    }
}

void FastLio2Node::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    ImuData imu;
    imu.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    imu.acc = Vector3d(msg->linear_acceleration.x,
                       msg->linear_acceleration.y,
                       msg->linear_acceleration.z) * config_.imu.acc_scale;
    imu.gyro = Vector3d(msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z);

    imu_buffer_.push(imu);
    imu_processor_->addImuData(imu);

    static int imu_count = 0;
    imu_count++;
    if (imu_count <= 3 || imu_count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "IMU callback #%d, t=%.3f, buf_size=%zu",
                    imu_count, imu.timestamp, imu_buffer_.size());
    }
}

void FastLio2Node::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    OdomData odom;
    odom.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    odom.position = Vector3d(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
    odom.rotation = Quaterniond(msg->pose.pose.orientation.w,
                                msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z);

    {
        std::lock_guard<std::mutex> lock(odom_buffer_mutex_);
        const size_t max_buffer_size = 200;
        if (odom_buffer_.size() >= max_buffer_size) {
            odom_buffer_.pop_front();
        }
        odom_buffer_.push_back(odom);
    }

    if (config_.odom_fusion.enable && odom_fusion_) {
        odom_fusion_->addOdomData(odom);
    }
}

void FastLio2Node::processPointCloud(PointCloudData& cloud_data) {
    PROFILE_FUNCTION(frame_processing);

    if (!first_scan_received_) return;

    static int process_count = 0;
    process_count++;
    if (process_count <= 60 || process_count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "processPointCloud #%d, scan_id=%d, pts=%zu, t=%.3f, map_pts=%zu",
                    process_count, cloud_data.scan_id, cloud_data.cloud->size(), cloud_data.timestamp,
                    ikd_tree_->size());
    }

    PROFILE_START(pointcloud_preprocessing);
    PointCloudPtr filtered_cloud = point_cloud_filter_->process(cloud_data.cloud);
    PROFILE_END(pointcloud_preprocessing);

    if (filtered_cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Filtered cloud is empty, skipping frame");
        return;
    }

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

    PROFILE_START(iekf_update);
    bool iekf_success = performUpdate(filtered_cloud);
    if (!iekf_success) {
        RCLCPP_WARN(this->get_logger(), "IEKF update failed");
    }
    PROFILE_END(iekf_update);

    PROFILE_START(map_update);
    if (iekf_success) {
        updateMap(filtered_cloud);
    }
    PROFILE_END(map_update);

    // 闭环检测 (每隔一定帧数执行一次)
    if (iekf_success && config_.loop_closure.enable && scan_context_) {
        if (cloud_data.scan_id - last_loop_detect_frame_ >= config_.loop_closure.min_interval) {
            PROFILE_START(loop_closure);
            detectLoopClosure(cloud_data);
            PROFILE_END(loop_closure);
        }
    }

    PROFILE_START(result_publish);
    publishOdometry();
    publishTF();
    PROFILE_END(result_publish);

    pose_history_.push_back(current_state_.toSE3());

    if (!system_initialized_) {
        system_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "System initialized successfully!");
    }

    scan_count_++;
    if (scan_count_ % 100 == 0) {
        Profiler::instance().printReport();
    }
}

void FastLio2Node::performPrediction(double t_start, double t_end) {
    if (!iekf_estimator_->isInitialized()) {
        State initial_state;
        initial_state.timestamp = t_start;
        initial_state.gravity = Vector3d(0.0, 0.0, -config_.imu.gravity_magnitude);
        if (imu_processor_->staticInitializeBias()) {
            initial_state.rotation = imu_processor_->getInitRotation();
            initial_state.gyro_bias = imu_processor_->getGyroBiasEst();
            RCLCPP_INFO(this->get_logger(), "Init: using static IMU estimate (gyro_bias=%.6f,%.6f,%.6f)",
                        initial_state.gyro_bias.x(), initial_state.gyro_bias.y(), initial_state.gyro_bias.z());
        } else {
            RCLCPP_INFO(this->get_logger(), "Init: identity (insufficient static IMU data, have %d/%d samples)",
                        static_cast<int>(imu_processor_->bufferSize()), 200);
        }
        iekf_estimator_->setInitialState(initial_state);
        current_state_ = initial_state;
        return;
    }

    std::vector<ImuData> imu_data = imu_buffer_.getImuInRange(t_start, t_end);

    if (imu_data.empty()) {
        RCLCPP_WARN(this->get_logger(), "No IMU data for prediction");
        return;
    }

    iekf_estimator_->setState(current_state_);
    iekf_estimator_->predictBatch(imu_data);
    current_state_ = iekf_estimator_->getState();
    current_state_.timestamp = t_end;
}

bool FastLio2Node::performUpdate(PointCloudPtr& cloud) {
    if (ikd_tree_->empty()) {
        RCLCPP_INFO(this->get_logger(), "performUpdate: first frame, building kd-tree with %zu points", cloud->size());
        ikd_tree_->build(cloud);
        RCLCPP_INFO(this->get_logger(), "performUpdate: kd-tree built successfully");
        return true;
    }

    if (!iekf_estimator_->isInitialized()) {
        return false;
    }

    const int K = 5;
    const double plane_threshold = 0.1;
    const double score_threshold = 0.9;
    const double max_sq_dist = 5.0;
    const int min_effective_features = 10;

    static int update_cnt = 0;
    update_cnt++;

    for (int iter = 0; iter < config_.iekf.max_iterations; ++iter) {
        SE3d current_pose = current_state_.toSE3();
        const Matrix3d R = current_pose.so3().matrix();
        const Vector3d t = current_pose.translation();

        std::vector<Vector3d> source_points;
        std::vector<Vector3d> plane_normals;
        std::vector<double> plane_dists;
        source_points.reserve(cloud->size());
        plane_normals.reserve(cloud->size());
        plane_dists.reserve(cloud->size());

        for (size_t i = 0; i < cloud->size(); ++i) {
            const PointType& pt_body = cloud->points[i];
            Vector3d p_body(pt_body.x, pt_body.y, pt_body.z);
            // 应用外参: LiDAR -> IMU -> World
            Vector3d p_imu = R_il_ * p_body + t_il_;
            Vector3d p_world = R * p_imu + t;

            PointType query;
            query.x = p_world.x();
            query.y = p_world.y();
            query.z = p_world.z();

            std::vector<PointType> neighbors;
            std::vector<double> sq_dists;
            if (!ikd_tree_->kNearestSearch(query, K, neighbors, sq_dists)) continue;
            if (static_cast<int>(neighbors.size()) < K) continue;
            if (sq_dists[K - 1] > max_sq_dist) continue;

            Eigen::Vector4d pabcd;
            if (!esti_plane(neighbors, plane_threshold, pabcd)) continue;

            double pd2 = pabcd(0) * p_world.x() +
                         pabcd(1) * p_world.y() +
                         pabcd(2) * p_world.z() +
                         pabcd(3);
            double body_norm = p_body.norm();
            if (body_norm < 1e-6) continue;
            double s = 1.0 - 0.9 * std::fabs(pd2) / std::sqrt(body_norm);
            if (s <= score_threshold) continue;

            source_points.push_back(p_imu);
            plane_normals.push_back(Vector3d(pabcd(0), pabcd(1), pabcd(2)));
            plane_dists.push_back(pd2);
        }

        if (iter == 0) {
            RCLCPP_INFO(this->get_logger(),
                "performUpdate #%d iter0: pts=%zu, effect=%zu, pose=(%.4f,%.4f,%.4f), map_pts=%zu",
                update_cnt, cloud->size(), source_points.size(),
                t(0), t(1), t(2), ikd_tree_->size());
        }

        if (static_cast<int>(source_points.size()) < min_effective_features) {
            bool has_nan = !std::isfinite(current_state_.position(0)) ||
                           !std::isfinite(current_state_.rotation.w());
            RCLCPP_WARN(this->get_logger(),
                "Too few effective features=%zu (need %d) | pose=(%.4f,%.4f,%.4f) | map_pts=%zu pts=%zu%s",
                source_points.size(), min_effective_features,
                current_state_.position(0), current_state_.position(1), current_state_.position(2),
                ikd_tree_->size(), cloud->size(),
                has_nan ? "  <<< NaN STATE" : "");
            break;
        }

        iekf_estimator_->setState(current_state_);
        bool success = iekf_estimator_->updateWithNormals(
            source_points, plane_normals, plane_dists, {});

        if (success) {
            current_state_ = iekf_estimator_->getState();
        } else {
            break;
        }
    }

    return true;
}

void FastLio2Node::updateMap(PointCloudPtr& cloud) {
    if (cloud->empty()) return;

    const State& state = current_state_;
    SE3d current_pose = state.toSE3();

    // Transform cloud to world frame (LiDAR → IMU → World)
    SE3d lidar_to_world = current_pose * SE3d(R_il_, t_il_);
    PointCloudPtr world_cloud(new pcl::PointCloud<PointType>());
    transformPointCloud(cloud, world_cloud, lidar_to_world);

    // Per-voxel deduplication (like FAST-LIO2's incrCloudMap)
    double map_res = config_.iekf.map_leaf_size;  // 0.2m
    PointCloudPtr points_to_add(new pcl::PointCloud<PointType>());
    PointCloudPtr points_no_downsample(new pcl::PointCloud<PointType>());

    for (const auto& p_world : world_cloud->points) {
        // Compute voxel center
        double mid_x = std::floor(p_world.x / map_res) * map_res + 0.5 * map_res;
        double mid_y = std::floor(p_world.y / map_res) * map_res + 0.5 * map_res;
        double mid_z = std::floor(p_world.z / map_res) * map_res + 0.5 * map_res;

        // Find nearest neighbor in map
        PointType query = p_world;
        PointType nearest;
        double dist;
        if (!ikd_tree_->nearestSearch(query, nearest, dist)) {
            // No nearby point → add without downsample
            points_no_downsample->push_back(p_world);
            continue;
        }

        // Check if existing point is in the same voxel and closer to center
        double nearest_dist_to_center = std::sqrt(
            (nearest.x - mid_x) * (nearest.x - mid_x) +
            (nearest.y - mid_y) * (nearest.y - mid_y) +
            (nearest.z - mid_z) * (nearest.z - mid_z));
        double point_dist_to_center = std::sqrt(
            (p_world.x - mid_x) * (p_world.x - mid_x) +
            (p_world.y - mid_y) * (p_world.y - mid_y) +
            (p_world.z - mid_z) * (p_world.z - mid_z));

        if (nearest_dist_to_center < point_dist_to_center) {
            // Existing point is better → skip
            continue;
        }

        // This point is better → add with downsample
        points_to_add->push_back(p_world);
    }

    // Insert into ikd-tree
    if (!points_to_add->empty()) {
        ikd_tree_->insertPointCloud(points_to_add);
    }
    if (!points_no_downsample->empty()) {
        ikd_tree_->insertPointCloud(points_no_downsample);
    }

    // Also update MapManager
    if (map_manager_) {
        map_manager_->addPointCloud(world_cloud, current_pose, scan_count_);
    }
}

void FastLio2Node::detectLoopClosure(const PointCloudData& cloud_data) {
    if (!scan_context_ || !gtsam_backend_ || pose_graph_.empty()) return;

    SE3d current_pose = current_state_.toSE3();
    PointCloudPtr transformed_cloud(new pcl::PointCloud<PointType>());
    SE3d ltw = current_pose * SE3d(R_il_, t_il_); transformPointCloud(cloud_data.cloud, transformed_cloud, ltw);

    int keyframe_id = keyframe_count_ - 1;
    if (keyframe_id < 0) return;

    // 使用ScanContext检测闭环
    ScanContextDescriptor query_desc = scan_context_->makeDescriptor(
        transformed_cloud, current_state_.timestamp, keyframe_id, current_pose);

    LoopConstraint constraint;
    if (scan_context_->detectLoop(query_desc, constraint)) {
        RCLCPP_INFO(this->get_logger(),
                    "Loop closure detected! frame=%d -> match=%d, score=%.3f",
                    keyframe_id, constraint.to_id, constraint.score);

        int match_id = constraint.to_id;
        if (match_id >= 0 && match_id < static_cast<int>(pose_graph_.size())) {
            gtsam_backend_->addLoopClosureFactor(match_id, keyframe_id, constraint.relative_pose);

            if (gtsam_backend_->optimize()) {
                RCLCPP_INFO(this->get_logger(), "GTSAM optimization succeeded after loop closure");
                std::vector<SE3d> corrected_poses = gtsam_backend_->getAllOptimizedPoses();
                if (!corrected_poses.empty()) {
                    correctMapWithOptimization(corrected_poses);
                }
            }
        }
    }

    last_loop_detect_frame_ = cloud_data.scan_id;
}

void FastLio2Node::performGlobalOptimization() {
    if (!gtsam_backend_ || !gtsam_backend_->optimize()) return;

    RCLCPP_INFO(this->get_logger(), "Global pose graph optimization completed");
    std::vector<SE3d> corrected_poses = gtsam_backend_->getAllOptimizedPoses();
    if (corrected_poses.size() != pose_graph_.size()) {
        RCLCPP_WARN(this->get_logger(), "Corrected poses count mismatch");
        return;
    }

    correctMapWithOptimization(corrected_poses);
}

void FastLio2Node::correctMapWithOptimization(const std::vector<SE3d>& corrected_poses) {
    if (corrected_poses.size() != pose_graph_.size() || corrected_poses.empty()) return;

    // 更新最新位姿
    SE3d latest_corrected = corrected_poses.back();
    current_state_.position = latest_corrected.translation();
    current_state_.rotation = latest_corrected.unit_quaternion();

    // 更新位姿图
    for (size_t i = 0; i < pose_graph_.size(); ++i) {
        pose_graph_[i].pose = corrected_poses[i];
    }

    RCLCPP_INFO(this->get_logger(), "Map correction applied: new pos=(%.2f, %.2f, %.2f)",
                current_state_.position.x(), current_state_.position.y(),
                current_state_.position.z());
}

bool FastLio2Node::isKeyframe(const SE3d& pose) const {
    if (pose_graph_.empty()) return true;
    const SE3d& last_pose = pose_graph_.back().pose;
    double dist = (pose.translation() - last_pose.translation()).norm();
    return dist > 0.3;
}

bool FastLio2Node::isRobotStationary() const {
    if (imu_variance_sample_count_ < kStationarySampleWindow) return false;
    return imu_gyro_variance_ < kStationaryGyroThreshold &&
           imu_acc_variance_ < kStationaryAccThreshold;
}

void FastLio2Node::updateStationaryState(const ImuData& imu) {
    const double alpha = 0.95;

    if (imu_variance_sample_count_ == 0) {
        last_imu_acc_mean_ = imu.acc;
        last_imu_gyro_mean_ = imu.gyro;
        imu_acc_variance_ = 0.0;
        imu_gyro_variance_ = 0.0;
        imu_variance_sample_count_ = 1;
        return;
    }

    Vector3d acc_diff = imu.acc - last_imu_acc_mean_;
    Vector3d gyro_diff = imu.gyro - last_imu_gyro_mean_;

    last_imu_acc_mean_ = alpha * last_imu_acc_mean_ + (1.0 - alpha) * imu.acc;
    last_imu_gyro_mean_ = alpha * last_imu_gyro_mean_ + (1.0 - alpha) * imu.gyro;

    imu_acc_variance_ = alpha * imu_acc_variance_ + (1.0 - alpha) * acc_diff.squaredNorm();
    imu_gyro_variance_ = alpha * imu_gyro_variance_ + (1.0 - alpha) * gyro_diff.squaredNorm();

    if (imu_variance_sample_count_ < 1000) {
        imu_variance_sample_count_++;
    }
}

void FastLio2Node::addKeyframe(int frame_id, const SE3d& pose, const PointCloudPtr& cloud) {
    pose_graph_.emplace_back(frame_id, pose, current_state_.timestamp, frame_id);

    if (config_.loop_closure.enable && scan_context_) {
        ScanContextDescriptor desc = scan_context_->makeDescriptor(
            cloud, current_state_.timestamp, frame_id, pose);
        scan_context_->addKeyframe(desc);
    }
}

void FastLio2Node::publishOdometry() {
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

    odom_msg.pose.pose.position.x = current_state_.position(0);
    odom_msg.pose.pose.position.y = current_state_.position(1);
    odom_msg.pose.pose.position.z = current_state_.position(2);

    odom_msg.pose.pose.orientation.w = current_state_.rotation.w();
    odom_msg.pose.pose.orientation.x = current_state_.rotation.x();
    odom_msg.pose.pose.orientation.y = current_state_.rotation.y();
    odom_msg.pose.pose.orientation.z = current_state_.rotation.z();

    odom_msg.twist.twist.linear.x = current_state_.velocity(0);
    odom_msg.twist.twist.linear.y = current_state_.velocity(1);
    odom_msg.twist.twist.linear.z = current_state_.velocity(2);

    odom_pub_->publish(odom_msg);
}

void FastLio2Node::publishPath() {
    if (!system_initialized_) return;

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

    if (path_msg_.poses.size() > 10000) {
        path_msg_.poses.erase(path_msg_.poses.begin());
    }

    path_pub_->publish(path_msg_);
}

void FastLio2Node::publishMap() {
    if (!system_initialized_ || ikd_tree_->empty()) return;

    PointCloudPtr map_cloud = ikd_tree_->getAllPoints();
    if (map_cloud->empty()) return;

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud, map_msg);

    map_msg.header.stamp = this->now();
    map_msg.header.frame_id = config_.ros.map_frame;

    map_pub_->publish(map_msg);
}

void FastLio2Node::publishTF() {
    if (!config_.ros.publish_tf) return;

    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->now();
    map_to_odom.header.frame_id = config_.ros.map_frame;
    map_to_odom.child_frame_id = config_.ros.odom_frame;
    map_to_odom.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(map_to_odom);

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

void FastLio2Node::saveMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (!map_manager_) {
        response->success = false;
        response->message = "Map manager not initialized";
        return;
    }

    std::string map_dir = config_.map.map_path;
    std::filesystem::create_directories(std::filesystem::path(map_dir));

    std::string map_path = map_dir + "/fast_lio2_map";
    PointCloudPtr ikd_cloud = ikd_tree_->getAllPoints();
    RCLCPP_INFO(this->get_logger(), "ikd_tree has %zu points, MapManager has %zu points",
                ikd_cloud->size(), map_manager_->pointCount());

    bool success = false;
    PointCloudPtr full_cloud = ikd_cloud;
    if (!full_cloud->empty()) {
        success = pcl::io::savePCDFileBinary(map_path + ".pcd", *full_cloud) == 0;
        if (success) {
            saveProjectedMap(full_cloud, map_path);
            map_manager_->saveToPcd(map_dir + "/map_manager_full.pcd");
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

void FastLio2Node::savePcdCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (!map_manager_) {
        response->success = false;
        response->message = "Map manager not initialized";
        return;
    }

    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << config_.map.map_path << "/pcd_" << std::put_time(std::localtime(&timestamp), "%Y%m%d_%H%M%S") << ".pcd";
    std::string pcd_path = ss.str();

    PointCloudPtr pcd_cloud = ikd_tree_->getAllPoints();
    bool success = false;
    if (!pcd_cloud->empty()) {
        std::filesystem::create_directories(std::filesystem::path(pcd_path).parent_path());
        success = pcl::io::savePCDFileBinary(pcd_path, *pcd_cloud) == 0;
    }

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

void FastLio2Node::globalLocalizeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
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

    RCLCPP_INFO(this->get_logger(), "Starting global localization...");
    localization_state_ = LocalizationState::LOCALIZING;

    response->success = true;
    response->message = "Global localization service triggered. Check localization status for result.";

    RCLCPP_INFO(this->get_logger(), "Global localization service called successfully");
}

void FastLio2Node::setInitialPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(localization_mutex_);

    if (!global_localizer_) {
        RCLCPP_WARN(this->get_logger(),
                    "Cannot set initial pose: global localizer not initialized");
        return;
    }

    SE3d initial_pose;
    initial_pose.translation() = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    initial_pose.so3() = Sophus::SO3d(q);

    global_localizer_->setInitialPose(initial_pose);

    RCLCPP_INFO(this->get_logger(),
                "Initial pose set: pos=[%.2f, %.2f, %.2f], orientation=[%.2f, %.2f, %.2f, %.2f]",
                msg->position.x, msg->position.y, msg->position.z,
                msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

bool FastLio2Node::checkDataSync() { return true; }

void FastLio2Node::undistortPointCloud(PointCloudData& cloud_data) {
    if (!iekf_estimator_ || !iekf_estimator_->isInitialized()) return;
    if (cloud_data.cloud->empty()) return;

    double t_start = current_state_.timestamp;
    double t_end = cloud_data.timestamp;
    if (t_end <= t_start) return;

    // 获取扫描期间的IMU数据
    std::vector<ImuData> imu_data = imu_buffer_.getImuInRange(t_start, t_end);
    if (imu_data.size() < 2) return;

    // 对每个点进行IMU插值去畸变
    double scan_period = t_end - t_start;
    PointCloudPtr undistorted(new pcl::PointCloud<PointType>());
    undistorted->reserve(cloud_data.cloud->size());

    State scan_start_state = iekf_estimator_->getState();
    scan_start_state.timestamp = t_start;

    for (const auto& point : cloud_data.cloud->points) {
        // 点的时间戳 = 扫描开始 + 相对时间(归一化到[0,1])
        double rel_time = point.intensity;
        if (rel_time < 0.0 || rel_time > 1.0) rel_time = 0.5;
        double point_time = t_start + rel_time * scan_period;

        // 用IMU数据插值该时刻的位姿
        State interp_state = scan_start_state;
        double imu_dt = 0.0;
        for (size_t i = 0; i + 1 < imu_data.size(); ++i) {
            if (imu_data[i].timestamp <= point_time && imu_data[i + 1].timestamp >= point_time) {
                double alpha = (point_time - imu_data[i].timestamp) /
                               (imu_data[i + 1].timestamp - imu_data[i].timestamp);
                alpha = std::clamp(alpha, 0.0, 1.0);

                ImuData interp_imu;
                interp_imu.timestamp = point_time;
                interp_imu.acc = (1.0 - alpha) * imu_data[i].acc + alpha * imu_data[i + 1].acc;
                interp_imu.gyro = (1.0 - alpha) * imu_data[i].gyro + alpha * imu_data[i + 1].gyro;

                imu_dt = point_time - scan_start_state.timestamp;
                break;
            }
        }

        // 从扫描开始到该点的相对变换
        Vector3d delta_angle = (interp_state.gyro_bias - scan_start_state.gyro_bias) * imu_dt;
        Quaterniond delta_q = Quaterniond::Identity();
        if (delta_angle.norm() > 1e-12) {
            delta_q = Quaterniond(Eigen::AngleAxisd(delta_angle.norm(), delta_angle.normalized()));
        }

        // 将点从扫描开始时刻变换到扫描结束时刻
        SE3d T_start(scan_start_state.rotation, scan_start_state.position);
        SE3d T_point = T_start * SE3d(delta_q, Vector3d::Zero());

        Vector3d p_point(point.x, point.y, point.z);
        Vector3d p_end = T_point * p_point;

        PointType new_point = point;
        new_point.x = p_end.x();
        new_point.y = p_end.y();
        new_point.z = p_end.z();
        undistorted->push_back(new_point);
    }

    undistorted->width = undistorted->size();
    undistorted->height = 1;
    cloud_data.cloud = undistorted;
}

void FastLio2Node::saveProjectedMap(const PointCloudPtr& cloud, const std::string& base_path) {
    if (cloud->empty()) return;

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

    double resolution = 0.05;
    int width = static_cast<int>((max_x - min_x) / resolution) + 1;
    int height = static_cast<int>((max_y - min_y) / resolution) + 1;

    if (width <= 0 || height <= 0 || width > 10000 || height > 10000) {
        RCLCPP_WARN(this->get_logger(), "Map dimensions too large or invalid: %dx%d", width, height);
        return;
    }

    std::vector<uint8_t> grid(width * height, 205);

    for (const auto& p : cloud->points) {
        if (p.z < -0.1 || p.z > 1.5) continue;
        int gx = static_cast<int>((p.x - min_x) / resolution);
        int gy = static_cast<int>((p.y - min_y) / resolution);
        if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
            grid[gy * width + gx] = 0;
        }
    }

    std::string pgm_path = base_path + ".pgm";
    std::ofstream pgm(pgm_path, std::ios::binary);
    pgm << "P5\n" << width << " " << height << "\n255\n";
    pgm.write(reinterpret_cast<const char*>(grid.data()), grid.size());
    pgm.close();

    std::string yaml_path = base_path + ".yaml";
    std::string pgm_filename = std::filesystem::path(base_path).filename().string() + ".pgm";
    std::ofstream yaml(yaml_path);
    yaml << "image: \"" << pgm_filename << "\"\n";
    yaml << "resolution: " << resolution << "\n";
    yaml << "origin: [" << min_x << ", " << min_y << ", 0.0]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.196\n";
    yaml.close();

    RCLCPP_INFO(this->get_logger(), "2D map saved: %s (%dx%d)", pgm_path.c_str(), width, height);
}

} // namespace fast_lio2_slam
