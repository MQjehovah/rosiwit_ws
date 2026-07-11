// ============================================================
// Diffbot Navigation - 平滑导航节点实现
// ============================================================

#include "rosiwit_navigation/ros_interface/navigation_node.hpp"

#include <memory>
#include <string>
#include <cmath>

#include "path_smoother.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"



namespace rosiwit_navigation
{
namespace navigation
{

using rclcpp_lifecycle::LifecycleNode;
using ros_interface::RosUtils;

SmoothNavigation::SmoothNavigation(const rclcpp::NodeOptions & options)
: LifecycleNode("rosiwit_navigation", options),
  current_state_(NavigationState::IDLE),
  is_navigating_(false),
  is_paused_(false)
{
}

LifecycleNode::CallbackReturn SmoothNavigation::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring rosiwit_navigation node");

  // 加载参数
  loadParameters();

  // 初始化TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 使用NavigationFactory创建规划器策略
  planner_strategy_ = nav_core::NavigationFactory::createPlanner(planner_name_);
  if (planner_strategy_) {
    core::PlannerConfig planner_cfg;
    planner_cfg.name = planner_name_;
    planner_cfg.resolution = 0.05;
    planner_cfg.max_iterations = 50000;
    planner_cfg.timeout = 2.0;
    planner_strategy_->initialize(planner_cfg);
    planner_strategy_->setInflationRadius(inflation_radius_);
    RCLCPP_INFO(get_logger(), "Created planner strategy: %s", planner_name_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Planner '%s' not found, falling back to PathPlanner", planner_name_.c_str());
  }

  // 使用NavigationFactory创建控制器策略
  controller_strategy_ = nav_core::NavigationFactory::createController(controller_name_);
  if (controller_strategy_) {
    core::ControllerConfig ctrl_cfg;
    ctrl_cfg.lookahead_distance = params_.lookahead_distance;
    ctrl_cfg.xy_goal_tolerance = params_.goal_tolerance_xy;
    ctrl_cfg.max_lookahead_distance = params_.max_lookahead_distance;
    ctrl_cfg.min_lookahead_distance = params_.min_lookahead_distance;
    ctrl_cfg.max_linear_velocity = params_.max_velocity_x;
    ctrl_cfg.max_angular_velocity = params_.max_velocity_theta;
    ctrl_cfg.kinematics.max_velocity_x = params_.max_velocity_x;
    ctrl_cfg.kinematics.max_velocity_theta = params_.max_velocity_theta;
    controller_strategy_->initialize(ctrl_cfg);
    RCLCPP_INFO(get_logger(), "Created controller strategy: %s", controller_name_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Controller '%s' not found, using fallback inline controller", controller_name_.c_str());
  }

  // 初始化组件
  trajectory_generator_ = std::make_shared<TrajectoryGenerator>();
  trajectory_generator_->setFrameId(global_frame_);

  // 初始化订阅者和发布者
  initializeSubscribers();
  initializePublishers();

  // 初始化Action服务器
  initializeActionServer();

  RCLCPP_INFO(get_logger(), "rosiwit_navigation node configured successfully");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn SmoothNavigation::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating rosiwit_navigation node");

  // 激活发布者
  cmd_vel_pub_->on_activate();
  path_pub_->on_activate();
  global_path_pub_->on_activate();

  // 激活规划器

  // 创建绑定到当前状态的定时器
  initializeTimers();

  // 设置状态为活跃
  current_state_ = NavigationState::IDLE;

  RCLCPP_INFO(get_logger(), "rosiwit_navigation node activated");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn SmoothNavigation::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating rosiwit_navigation node");

  // 停用定时器
  control_timer_->cancel();
  replanning_timer_->cancel();

  // 停用发布者
  cmd_vel_pub_->on_deactivate();
  path_pub_->on_deactivate();
  global_path_pub_->on_deactivate();

  // 停用规划器

  // 停止当前导航
  stopNavigation();

  current_state_ = NavigationState::IDLE;

  RCLCPP_INFO(get_logger(), "rosiwit_navigation node deactivated");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn SmoothNavigation::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up rosiwit_navigation node");

  // 清理组件
  planner_strategy_.reset();
  controller_strategy_.reset();

  // 重置订阅者和发布者
  odom_sub_.reset();
  map_sub_.reset();
  goal_sub_.reset();
  cmd_vel_pub_.reset();
  path_pub_.reset();
  global_path_pub_.reset();

  tf_buffer_.reset();
  tf_listener_.reset();

  RCLCPP_INFO(get_logger(), "rosiwit_navigation node cleaned up");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn SmoothNavigation::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down rosiwit_navigation node");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

void SmoothNavigation::loadParameters()
{
  this->declare_parameter("controller_frequency", 20.0);
  this->declare_parameter("planner_frequency", 1.0);
  this->declare_parameter("max_velocity_x", 0.5);
  this->declare_parameter("max_velocity_theta", 1.0);
  this->declare_parameter("min_velocity_x", -0.5);
  this->declare_parameter("max_accel_x", 0.5);
  this->declare_parameter("max_accel_theta", 1.0);
  this->declare_parameter("goal_tolerance_xy", 0.1);
  this->declare_parameter("goal_tolerance_yaw", 0.1);
  this->declare_parameter("lookahead_distance", 0.6);
  this->declare_parameter("min_lookahead_distance", 0.3);
  this->declare_parameter("max_lookahead_distance", 0.9);

  this->declare_parameter("global_frame", std::string("odom"));
  this->declare_parameter("robot_frame", std::string("base_link"));

  this->declare_parameter("planner_name", std::string("astar"));
  this->declare_parameter("controller_name", std::string("pure_pursuit"));
  this->declare_parameter("inflation_radius", 0.5);

  get_parameter("controller_frequency", params_.controller_frequency);
  get_parameter("planner_frequency", params_.planner_frequency);
  get_parameter("max_velocity_x", params_.max_velocity_x);
  get_parameter("max_velocity_theta", params_.max_velocity_theta);
  get_parameter("min_velocity_x", params_.min_velocity_x);
  get_parameter("max_accel_x", params_.max_accel_x);
  get_parameter("max_accel_theta", params_.max_accel_theta);
  get_parameter("goal_tolerance_xy", params_.goal_tolerance_xy);
  get_parameter("goal_tolerance_yaw", params_.goal_tolerance_yaw);
  get_parameter("lookahead_distance", params_.lookahead_distance);
  get_parameter("min_lookahead_distance", params_.min_lookahead_distance);
  get_parameter("max_lookahead_distance", params_.max_lookahead_distance);

  get_parameter("global_frame", global_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("planner_name", planner_name_);
  get_parameter("controller_name", controller_name_);
  get_parameter("inflation_radius", inflation_radius_);

  RCLCPP_INFO(get_logger(), "Parameters loaded:");
  RCLCPP_INFO(get_logger(), "  - controller_frequency: %.1f Hz", params_.controller_frequency);
  RCLCPP_INFO(get_logger(), "  - max_velocity_x: %.2f m/s", params_.max_velocity_x);
  RCLCPP_INFO(get_logger(), "  - goal_tolerance_xy: %.2f m", params_.goal_tolerance_xy);
}

void SmoothNavigation::initializeSubscribers()
{
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SensorDataQoS(),
    std::bind(&SmoothNavigation::odometryCallback, this, std::placeholders::_1));

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&SmoothNavigation::mapCallback, this, std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", rclcpp::SystemDefaultsQoS(),
    std::bind(&SmoothNavigation::goalCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribers initialized");
}

void SmoothNavigation::initializePublishers()
{
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/local_path", rclcpp::SystemDefaultsQoS());

  global_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/global_path", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Publishers initialized");
}

void SmoothNavigation::initializeActionServer()
{

  RCLCPP_INFO(get_logger(), "Action server initialized");
}

void SmoothNavigation::initializeTimers()
{
  // 控制循环定时器
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / params_.controller_frequency)),
    std::bind(&SmoothNavigation::controlLoop, this));

  // 重规划检查定时器
  replanning_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / params_.planner_frequency)),
    std::bind(&SmoothNavigation::replanningCheck, this));

  RCLCPP_INFO(get_logger(), "Timers initialized");
}

void SmoothNavigation::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_velocity_ = msg->twist.twist;

  // 更新当前位姿
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose.pose;

  RCLCPP_DEBUG(get_logger(), "Odometry updated: pos=(%.2f, %.2f), vel=(%.2f, %.2f)",
    current_pose_.pose.position.x,
    current_pose_.pose.position.y,
    current_velocity_.linear.x,
    current_velocity_.angular.z);
}

void SmoothNavigation::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = msg;
  if (planner_strategy_) {
    auto grid = std::make_shared<core::CostmapGrid>();
    grid->info.width = msg->info.width;
    grid->info.height = msg->info.height;
    grid->info.resolution = msg->info.resolution;
    grid->info.origin.position.x = msg->info.origin.position.x;
    grid->info.origin.position.y = msg->info.origin.position.y;
    grid->data = msg->data;

    core::Costmap costmap;
    costmap.grid = grid;
    costmap.width = msg->info.width;
    costmap.height = msg->info.height;
    costmap.resolution = msg->info.resolution;
    costmap.origin_x = msg->info.origin.position.x;
    costmap.origin_y = msg->info.origin.position.y;
    costmap.data.assign(msg->data.begin(), msg->data.end());
    planner_strategy_->setCostmap(costmap);
  }
  RCLCPP_INFO(get_logger(), "Map received: %d x %d, res=%.3f",
    msg->info.width, msg->info.height, msg->info.resolution);
}

void SmoothNavigation::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(),
    "Received /goal_pose: (%.2f, %.2f, %.2f)",
    msg->pose.position.x, msg->pose.position.y,
    tf2::getYaw(msg->pose.orientation));

  goal_pose_ = *msg;
  executeNavigation(goal_pose_);
}

void SmoothNavigation::controlLoop()
{
  if (!is_navigating_ || is_paused_) {
    return;
  }

  // 更新当前位姿
  if (!updateCurrentPose()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Cannot get current pose");
    return;
  }

  // 检查是否到达目标
  if (isGoalReached()) {
    RCLCPP_INFO(get_logger(), "Goal reached!");
    current_state_ = NavigationState::GOAL_REACHED;
    stopNavigation();
    return;
  }

  // 计算速度命令
  geometry_msgs::msg::Twist cmd_vel = computeVelocityCommand();

  // 发布速度命令
  publishVelocityCommand(cmd_vel);

  // 发布路径（用于可视化）
  publishPath(current_path_);
}

void SmoothNavigation::replanningCheck()
{
  if (!is_navigating_ || is_paused_) {
    return;
  }

  // 检查是否需要重规划
  // TODO: 实现完整的重规划逻辑
  RCLCPP_DEBUG(get_logger(), "Replanning check");
}


void SmoothNavigation::executeNavigation(const geometry_msgs::msg::PoseStamped & goal)
{
  // 更新当前位姿
  if (!updateCurrentPose()) {
    RCLCPP_ERROR(get_logger(), "Cannot get current pose for planning");
    current_state_ = NavigationState::FAILED;
    return;
  }

  // 规划路径
  RCLCPP_INFO(get_logger(), "Planning path to goal...");
  current_state_ = NavigationState::PLANNING;

  if (planner_strategy_) {
    auto start_pose = RosUtils::toCorePose(current_pose_);
    auto goal_pose_core = RosUtils::toCorePose(goal);
    auto result = planner_strategy_->plan(start_pose, goal_pose_core);
    if (result.is_ok()) {
      current_path_ = RosUtils::toRosPath(result.value(), goal.header.frame_id, now());

      // 平滑路径
      if (current_path_.poses.size() >= 3) {
        planners::SmootherConfig smoother_cfg;
        smoother_cfg.max_iterations = 100;
        smoother_cfg.alpha_start = 0.5;
        smoother_cfg.alpha_end = 0.1;

        // 转换为 core 类型，平滑，再转回 ROS 类型
        core::Path core_path = RosUtils::toCorePath(current_path_);
        // 构建 core::CostmapGrid 用于碰撞检测
        std::shared_ptr<core::CostmapGrid> core_map;
        if (current_map_) {
          core_map = std::make_shared<core::CostmapGrid>();
          core_map->info.width = current_map_->info.width;
          core_map->info.height = current_map_->info.height;
          core_map->info.resolution = current_map_->info.resolution;
          core_map->info.origin.position.x = current_map_->info.origin.position.x;
          core_map->info.origin.position.y = current_map_->info.origin.position.y;
          core_map->data = current_map_->data;
        }
        core_path = planners::smoothPath(core_path, core_map, smoother_cfg);
        current_path_ = RosUtils::toRosPath(core_path, goal.header.frame_id, now());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Planner '%s' failed: %s",
        planner_name_.c_str(), result.error_message().c_str());
      current_state_ = NavigationState::FAILED;
      return;
    }
  }

  // 将路径注入控制器
  if (controller_strategy_) {
    controller_strategy_->setPath(RosUtils::toCorePath(current_path_));
  }

  // 发布全局路径
  global_path_pub_->publish(current_path_);

  RCLCPP_INFO(get_logger(),
    "Path planned with %zu points",
    current_path_.poses.size());

  // 设置导航状态
  is_navigating_ = true;
  is_paused_ = false;
  current_state_ = NavigationState::CONTROLLING;
}

bool SmoothNavigation::updateCurrentPose()
{
  try {
    // 从TF获取当前位姿
    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer_->lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero);

    current_pose_.header.stamp = transform.header.stamp;
    current_pose_.header.frame_id = global_frame_;
    current_pose_.pose.position.x = transform.transform.translation.x;
    current_pose_.pose.position.y = transform.transform.translation.y;
    current_pose_.pose.position.z = transform.transform.translation.z;
    current_pose_.pose.orientation = transform.transform.rotation;

    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Transform error: %s", ex.what());
    return false;
  }
}

bool SmoothNavigation::isGoalReached()
{
  // 优先使用控制器的目标到达判断
  if (controller_strategy_) {
    auto core_pose = RosUtils::toCorePose(current_pose_);
    return controller_strategy_->isGoalReached(core_pose);
  }

  // 后备：使用简单的欧氏距离判断
  double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
  double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  double goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);
  double current_yaw = tf2::getYaw(current_pose_.pose.orientation);
  double angle_diff = std::abs(goal_yaw - current_yaw);

  while (angle_diff > M_PI) {
    angle_diff -= 2.0 * M_PI;
  }
  angle_diff = std::abs(angle_diff);

  bool xy_reached = distance < params_.goal_tolerance_xy;
  bool yaw_reached = angle_diff < params_.goal_tolerance_yaw;

  return xy_reached && yaw_reached;
}

geometry_msgs::msg::Twist SmoothNavigation::computeVelocityCommand()
{
  // 优先使用控制器的速度计算
  if (controller_strategy_) {
    auto core_pose = RosUtils::toCorePose(current_pose_);
    auto core_vel = RosUtils::toCoreVelocity(current_velocity_);
    auto cmd = controller_strategy_->computeVelocityCommand(core_pose, core_vel);
    geometry_msgs::msg::Twist result = RosUtils::toRosTwist(cmd);
    RCLCPP_DEBUG(get_logger(),
      "Controller computed velocity: linear=%.2f, angular=%.2f",
      result.linear.x, result.angular.z);
    return result;
  }

  // 后备：简化的速度计算
  geometry_msgs::msg::Twist cmd_vel;

  double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
  double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  double target_angle = std::atan2(dy, dx);
  double current_yaw = tf2::getYaw(current_pose_.pose.orientation);

  double angle_diff = target_angle - current_yaw;
  while (angle_diff > M_PI) {
    angle_diff -= 2.0 * M_PI;
  }
  while (angle_diff < -M_PI) {
    angle_diff += 2.0 * M_PI;
  }

  if (std::abs(angle_diff) < M_PI / 4) {
    double vel_scale = std::min(distance / 1.0, 1.0);
    cmd_vel.linear.x = vel_scale * params_.max_velocity_x;
  } else {
    cmd_vel.linear.x = 0.0;
  }

  cmd_vel.angular.z = 2.0 * angle_diff;

  cmd_vel.linear.x = std::clamp(cmd_vel.linear.x,
    params_.min_velocity_x, params_.max_velocity_x);
  cmd_vel.angular.z = std::clamp(cmd_vel.angular.z,
    -params_.max_velocity_theta, params_.max_velocity_theta);

  RCLCPP_DEBUG(get_logger(),
    "Computed velocity: linear=%.2f, angular=%.2f",
    cmd_vel.linear.x, cmd_vel.angular.z);

  return cmd_vel;
}

void SmoothNavigation::publishVelocityCommand(const geometry_msgs::msg::Twist & cmd_vel)
{
  cmd_vel_pub_->publish(cmd_vel);
}

void SmoothNavigation::publishPath(const nav_msgs::msg::Path & path)
{
  path_pub_->publish(path);
}

void SmoothNavigation::stopNavigation()
{
  is_navigating_ = false;
  is_paused_ = false;
  current_state_ = NavigationState::IDLE;

  // 发送零速度命令
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(stop_cmd);

  RCLCPP_INFO(get_logger(), "Navigation stopped");
}

void SmoothNavigation::pauseNavigation()
{
  is_paused_ = true;
  current_state_ = NavigationState::IDLE;

  // 发送零速度命令
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(stop_cmd);

  RCLCPP_INFO(get_logger(), "Navigation paused");
}

void SmoothNavigation::resumeNavigation()
{
  if (is_navigating_ && is_paused_) {
    is_paused_ = false;
    current_state_ = NavigationState::CONTROLLING;
    RCLCPP_INFO(get_logger(), "Navigation resumed");
  }
}

} // namespace navigation
} // namespace rosiwit_navigation