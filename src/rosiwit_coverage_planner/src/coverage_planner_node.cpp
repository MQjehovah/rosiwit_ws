// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include "coverage_planner/coverage_planner.hpp"
#include "coverage_planner/zigzag_planner.hpp"
#include "coverage_planner/spiral_planner.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace coverage_planner
{

CoveragePlannerNode::CoveragePlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("coverage_planner_node", options),
  map_received_(false),
  start_pose_received_(false)
{
    // 声明参数
    declareParameters();

    // 初始化参数
    initializeParameters();

    // 创建规划上下文
    planner_context_ = std::make_unique<PlannerContext>();

    // 使用单线程执行器避免回调竞态条件
    // 注意：如果需要在多线程环境中运行，需添加mutex保护共享状态

    // 创建订阅者
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(1).reliable(),
        std::bind(&CoveragePlannerNode::mapCallback, this, std::placeholders::_1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",
        rclcpp::QoS(1).reliable(),
        std::bind(&CoveragePlannerNode::initialPoseCallback, this, std::placeholders::_1));

    // 创建发布者
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/coverage_path",
        rclcpp::QoS(1).reliable());

    // 创建服务
    plan_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/plan_coverage",
        std::bind(&CoveragePlannerNode::planServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
        "Coverage Planner Node initialized with mode: %s",
        (coverage_mode_ == PlannerMode::ZIGZAG) ? "zigzag" : "spiral");

    RCLCPP_INFO(this->get_logger(),
        "Robot radius: %.2f m, Coverage resolution: %.2f m",
        planner_config_.robot_radius, planner_config_.coverage_resolution);
}

CoveragePlannerNode::~CoveragePlannerNode()
{
    RCLCPP_INFO(this->get_logger(), "Coverage Planner Node shutting down");
}

void CoveragePlannerNode::declareParameters()
{
    // 声明参数
    this->declare_parameter<std::string>("coverage_mode", "zigzag");
    this->declare_parameter<double>("robot_radius", 0.3);
    this->declare_parameter<double>("coverage_resolution", 0.1);
    this->declare_parameter<double>("inflation_radius", 0.25);
    this->declare_parameter<bool>("enable_optimization", true);
    this->declare_parameter<int>("direction_optimization", 2);
    this->declare_parameter<std::string>("frame_id", "map");

    // 地图尺寸限制（防止内存问题）
    this->declare_parameter<int>("max_map_width", 1000);
    this->declare_parameter<int>("max_map_height", 1000);

    // === P0优化参数 - 分区规划 ===
    this->declare_parameter<bool>("enable_zone_decomposition", false);
    this->declare_parameter<int>("zone_min_area", 100);
    this->declare_parameter<int>("zone_max_count", 20);
    this->declare_parameter<double>("zone_merge_threshold", 0.2);
    this->declare_parameter<int>("connection_search_radius", 5);

    // === P0优化参数 - 转弯优化 ===
    this->declare_parameter<bool>("enable_turn_optimization", false);
    this->declare_parameter<double>("turn_angle_threshold", 0.1);
    this->declare_parameter<double>("turn_merge_distance", 10.0);
    this->declare_parameter<bool>("enable_turn_smoothing", false);
}

void CoveragePlannerNode::initializeParameters()
{
    // 获取参数值
    std::string coverage_mode_str = this->get_parameter("coverage_mode").as_string();

    // 设置规划模式
    if (coverage_mode_str == "zigzag" || coverage_mode_str == "Zigzag") {
        coverage_mode_ = PlannerMode::ZIGZAG;
    } else if (coverage_mode_str == "spiral" || coverage_mode_str == "Spiral") {
        coverage_mode_ = PlannerMode::SPIRAL;
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Invalid coverage_mode '%s', using zigzag as default",
            coverage_mode_str.c_str());
        coverage_mode_ = PlannerMode::ZIGZAG;
    }

    // 设置规划配置
    planner_config_.robot_radius = this->get_parameter("robot_radius").as_double();
    planner_config_.coverage_resolution = this->get_parameter("coverage_resolution").as_double();
    planner_config_.inflation_radius = this->get_parameter("inflation_radius").as_double();
    planner_config_.enable_optimization = this->get_parameter("enable_optimization").as_bool();
    planner_config_.direction_optimization = this->get_parameter("direction_optimization").as_int();

    // 验证参数有效性
    if (planner_config_.robot_radius <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "robot_radius must be positive, using default 0.3m");
        planner_config_.robot_radius = 0.3;
    }

    if (planner_config_.coverage_resolution <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "coverage_resolution must be positive, using default 0.1m");
        planner_config_.coverage_resolution = 0.1;
    }

    // === 读取P0优化参数 ===
    planner_config_.enable_zone_decomposition = this->get_parameter("enable_zone_decomposition").as_bool();
    planner_config_.zone_min_area = this->get_parameter("zone_min_area").as_int();
    planner_config_.zone_max_count = this->get_parameter("zone_max_count").as_int();
    planner_config_.zone_merge_threshold = this->get_parameter("zone_merge_threshold").as_double();

    planner_config_.enable_turn_optimization = this->get_parameter("enable_turn_optimization").as_bool();
    planner_config_.turn_angle_threshold = this->get_parameter("turn_angle_threshold").as_double();
    planner_config_.turn_merge_distance = this->get_parameter("turn_merge_distance").as_double();

    // 输出P0优化状态
    if (planner_config_.enable_zone_decomposition) {
        RCLCPP_INFO(this->get_logger(), "Zone decomposition enabled (P0 optimization)");
    }
    if (planner_config_.enable_turn_optimization) {
        RCLCPP_INFO(this->get_logger(), "Turn optimization enabled (P0 optimization)");
    }

    if (planner_config_.direction_optimization < 0 || planner_config_.direction_optimization > 4) {
        RCLCPP_WARN(this->get_logger(),
            "direction_optimization must be 0-4 (0:horizontal, 1:vertical, 2:auto, 3:PCA, 4:long-edge), using default 4");
        planner_config_.direction_optimization = 4;
    }

    // 输出方向优化模式
    std::string dir_mode_desc;
    switch (planner_config_.direction_optimization) {
        case 0: dir_mode_desc = "horizontal"; break;
        case 1: dir_mode_desc = "vertical"; break;
        case 2: dir_mode_desc = "auto"; break;
        case 3: dir_mode_desc = "PCA-based"; break;
        case 4: dir_mode_desc = "long-edge priority"; break;
        default: dir_mode_desc = "unknown"; break;
    }
    RCLCPP_INFO(this->get_logger(), "Direction optimization mode: %s", dir_mode_desc.c_str());
}

void CoveragePlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // ==================== 安全验证（修复VULN-002和VULN-003） ====================

    // VULN-003修复：验证resolution防止除零崩溃
    if (msg->info.resolution <= 0.0) {
        RCLCPP_ERROR(this->get_logger(),
            "Map resolution (%.6f) is invalid (must be > 0), REJECTING map message",
            msg->info.resolution);
        return;  // 拒绝无效地图
    }

    // VULN-002修复：验证地图尺寸防止资源耗尽DoS
    int max_width = this->get_parameter("max_map_width").as_int();
    int max_height = this->get_parameter("max_map_height").as_int();

    // 整数溢出安全检查（46340x46340是32位int最大安全值）
    if (msg->info.width > 46340 || msg->info.height > 46340) {
        RCLCPP_ERROR(this->get_logger(),
            "Map size (%zu x %zu) exceeds integer overflow safe limit (max 46340x46340), REJECTED",
            msg->info.width, msg->info.height);
        return;  // 拒绝超大地图，防止整数溢出攻击
    }

    // 配置参数限制检查（拒绝，而非仅警告）
    if (msg->info.width > static_cast<size_t>(max_width) ||
        msg->info.height > static_cast<size_t>(max_height)) {
        RCLCPP_ERROR(this->get_logger(),
            "Map size (%zu x %zu) exceeds configured maximum (%d x %d), REJECTED to prevent DoS",
            msg->info.width, msg->info.height, max_width, max_height);
        return;  // 拒绝超大地图，防止资源耗尽DoS攻击
    }

    // ==================== 地图处理 ====================
    current_map_ = *msg;
    map_received_ = true;

    RCLCPP_INFO(this->get_logger(),
        "Map received: %zu x %zu, resolution %.3f m/frame %s",
        msg->info.width, msg->info.height, msg->info.resolution, msg->header.frame_id.c_str());

    // 如果已经收到起始位置，自动触发规划
    if (start_pose_received_) {
        RCLCPP_INFO(this->get_logger(), "Auto-triggering coverage planning...");
        planCoverage();
    }
}

void CoveragePlannerNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    start_pose_ = msg->pose.pose;
    start_pose_received_ = true;

    RCLCPP_INFO(this->get_logger(),
        "Initial pose received: (%.2f, %.2f)",
        start_pose_.position.x, start_pose_.position.y);

    // 如果已经收到地图，自动触发规划
    if (map_received_) {
        RCLCPP_INFO(this->get_logger(), "Auto-triggering coverage planning...");
        planCoverage();
    }
}

void CoveragePlannerNode::planServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;  // 未使用

    RCLCPP_INFO(this->get_logger(), "Coverage planning service triggered");

    bool success = planCoverage();

    response->success = success;

    if (success) {
        response->message = "Coverage planning succeeded. " +
            std::string("Coverage rate: ") + std::to_string(last_result_.coverage_rate * 100) + "%, " +
            std::string("Path length: ") + std::to_string(last_result_.path_length) + "m, " +
            std::string("Turn count: ") + std::to_string(last_result_.turn_count);

        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
        response->message = "Coverage planning failed: " + last_result_.error_message;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

bool CoveragePlannerNode::planCoverage()
{
    // 检查是否收到必要的数据
    if (!map_received_) {
        RCLCPP_ERROR(this->get_logger(), "No map received, cannot plan coverage");
        last_result_.success = false;
        last_result_.error_message = "No map received";
        return false;
    }

    if (!start_pose_received_) {
        RCLCPP_ERROR(this->get_logger(),
            "No initial pose received, using map center as start point");

        // 使用地图中心作为起始位置
        start_pose_.position.x = current_map_.info.origin.position.x +
            (current_map_.info.width * current_map_.info.resolution) / 2.0;
        start_pose_.position.y = current_map_.info.origin.position.y +
            (current_map_.info.height * current_map_.info.resolution) / 2.0;
        start_pose_.position.z = 0.0;
        start_pose_.orientation.w = 1.0;
    }

    // 选择规划器
    IPlanner* planner = planner_context_->selectPlanner(coverage_mode_);

    if (planner == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to select planner");
        last_result_.success = false;
        last_result_.error_message = "Failed to select planner";
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
        "Planning coverage using %s algorithm...",
        planner->getName().c_str());

    // 执行规划
    last_result_ = planner->plan(current_map_, start_pose_, planner_config_);

    if (last_result_.success) {
        // 发布路径
        publishPath(last_result_.path);

        RCLCPP_INFO(this->get_logger(),
            "Coverage planning completed: %.2f%% coverage, %.2f m path length, %d turns",
            last_result_.coverage_rate * 100.0,
            last_result_.path_length,
            last_result_.turn_count);

        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(),
            "Coverage planning failed: %s",
            last_result_.error_message.c_str());

        return false;
    }
}

void CoveragePlannerNode::publishPath(const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
    if (path.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty path, nothing to publish");
        return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = this->get_parameter("frame_id").as_string();
    path_msg.poses = path;

    path_pub_->publish(path_msg);

    RCLCPP_INFO(this->get_logger(),
        "Published coverage path with %zu poses",
        path.size());
}

}  // namespace coverage_planner

// 注册为ROS2组件
RCLCPP_COMPONENTS_REGISTER_NODE(coverage_planner::CoveragePlannerNode)