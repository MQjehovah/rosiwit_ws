// ============================================================
// Diffbot Navigation - 平滑导航节点实现
// ============================================================

#include "rosiwit_navigation/ros_interface/navigation_node.hpp"

#include <memory>
#include <string>
#include <cmath>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

namespace rosiwit_navigation
{
namespace navigation
{

using rclcpp_lifecycle::LifecycleNode;

SmoothNavigation::SmoothNavigation(const rclcpp::NodeOptions & options)
: LifecycleNode("smooth_navigation", options),
  current_state_(NavigationState::IDLE),
  is_navigating_(false),
  is_paused_(false)
{
}

LifecycleNode::CallbackReturn SmoothNavigation::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring smooth_navigation node");

  // 加载参数
  loadParameters();

  // 初始化TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 初始化组件
  path_planner_ = std::make_shared<PathPlanner>();
  trajectory_generator_ = std::make_shared<TrajectoryGenerator>();
  trajectory_generator_->setFrameId(global_frame_);

  // 初始化订阅者和发布者
  initializeSubscribers();
  initializePublishers();

  // 初始化Action服务器
  initializeActionServer();

  RCLCPP_INFO(get_logger(), "Smooth_navigation node configured successfully");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn SmoothNavigation::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating smooth_navigation node");

  // 激活发布者
  cmd_vel_pub_->on_activate();
  path_pub_->on_activate();
  global_path_pub_->on_activate();

  // 激活规划器
  path_planner_->activate();

  // 创建绑定到当前状态的定时器
  initializeTimers();

  // 设置状态为活跃
  current_state_ = NavigationState::IDLE;

  RCLCPP_INFO(get_logger(), "Smooth_navigation node activated");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn SmoothNavigation::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating smooth_navigation node");

  // 停用定时器
  control_timer_->cancel();
  replanning_timer_->cancel();

  // 停用发布者
  cmd_vel_pub_->on_deactivate();
  path_pub_->on_deactivate();
  global_path_pub_->on_deactivate();

  // 停用规划器
  path_planner_->deactivate();

  // 停止当前导航
  stopNavigation();

  current_state_ = NavigationState::IDLE;

  RCLCPP_INFO(get_logger(), "Smooth_navigation node deactivated");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn SmoothNavigation::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up smooth_navigation node");

  // 清理组件
  path_planner_->cleanup();

  // 重置订阅者和发布者
  odom_sub_.reset();
  map_sub_.reset();
  goal_sub_.reset();
  cmd_vel_pub_.reset();
  path_pub_.reset();
  global_path_pub_.reset();
  action_server_.reset();

  tf_buffer_.reset();
  tf_listener_.reset();

  RCLCPP_INFO(get_logger(), "Smooth_navigation node cleaned up");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn SmoothNavigation::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down smooth_navigation node");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

void SmoothNavigation::loadParameters()
{
  nav2_util::declare_parameter_if_not_declared(this, "controller_frequency", rclcpp::ParameterValue(20.0));
  nav2_util::declare_parameter_if_not_declared(this, "planner_frequency", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(this, "max_velocity_x", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(this, "max_velocity_theta", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(this, "min_velocity_x", rclcpp::ParameterValue(-0.5));
  nav2_util::declare_parameter_if_not_declared(this, "max_accel_x", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(this, "max_accel_theta", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(this, "goal_tolerance_xy", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(this, "goal_tolerance_yaw", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(this, "lookahead_distance", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(this, "min_lookahead_distance", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(this, "max_lookahead_distance", rclcpp::ParameterValue(0.9));

  nav2_util::declare_parameter_if_not_declared(this, "global_frame", rclcpp::ParameterValue(std::string("odom")));
  nav2_util::declare_parameter_if_not_declared(this, "robot_frame", rclcpp::ParameterValue(std::string("base_link")));

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
    "/map", rclcpp::SensorDataQoS(),
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
  action_server_ = std::make_unique<nav2_util::SimpleActionServer<nav2_msgs::action::NavigateToPose>>(
    shared_from_this(),
    "navigate_to_pose",
    std::bind(&SmoothNavigation::navigateToPoseCallback, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

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
  RCLCPP_DEBUG(get_logger(), "Map updated: size=%d x %d",
    msg->info.width, msg->info.height);
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

void SmoothNavigation::navigateToPoseCallback()
{
  auto goal = action_server_->get_current_goal();

  RCLCPP_INFO(get_logger(),
    "Received navigate_to_pose goal: (%.2f, %.2f)",
    goal->pose.pose.position.x,
    goal->pose.pose.position.y);

  // 设置目标位姿
  goal_pose_ = goal->pose;

  // 执行导航
  executeNavigation(goal_pose_);

  // 设置结果
  auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
  action_server_->succeeded_current(result);
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

  current_path_ = path_planner_->createPlan(current_pose_, goal);

  if (current_path_.poses.empty()) {
    RCLCPP_ERROR(get_logger(), "Failed to plan path");
    current_state_ = NavigationState::FAILED;
    return;
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
  // 计算到目标的距离
  double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
  double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // 计算角度差
  double goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);
  double current_yaw = tf2::getYaw(current_pose_.pose.orientation);
  double angle_diff = std::abs(goal_yaw - current_yaw);

  // 规范化角度差
  while (angle_diff > M_PI) {
    angle_diff -= 2.0 * M_PI;
  }
  angle_diff = std::abs(angle_diff);

  // 检查是否满足容差
  bool xy_reached = distance < params_.goal_tolerance_xy;
  bool yaw_reached = angle_diff < params_.goal_tolerance_yaw;

  return xy_reached && yaw_reached;
}

geometry_msgs::msg::Twist SmoothNavigation::computeVelocityCommand()
{
  // 简化的速度计算
  geometry_msgs::msg::Twist cmd_vel;

  // 计算到目标的距离和方向
  double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
  double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  double target_angle = std::atan2(dy, dx);
  double current_yaw = tf2::getYaw(current_pose_.pose.orientation);

  // 计算角度差
  double angle_diff = target_angle - current_yaw;
  while (angle_diff > M_PI) {
    angle_diff -= 2.0 * M_PI;
  }
  while (angle_diff < -M_PI) {
    angle_diff += 2.0 * M_PI;
  }

  // 线速度：根据距离调整
  if (std::abs(angle_diff) < M_PI / 4) {  // 角度误差小于45度时前进
    // 使用简单的速度规划
    double vel_scale = std::min(distance / 1.0, 1.0);  // 距离越远速度越大
    cmd_vel.linear.x = vel_scale * params_.max_velocity_x;
  } else {
    cmd_vel.linear.x = 0.0;  // 角度误差大时停止前进
  }

  // 角速度：根据角度差调整
  cmd_vel.angular.z = 2.0 * angle_diff;  // 简单的比例控制

  // 应用速度限制
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