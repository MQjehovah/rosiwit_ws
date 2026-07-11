// ============================================================
// Diffbot Navigation - 窄道通行规划器实现
// ============================================================

#include "narrow_passage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

namespace rosiwit_navigation
{
namespace narrow_passage
{

using nav2_util::declare_parameter_if_not_declared;

NarrowPassagePlanner::NarrowPassagePlanner()
: current_state_(PassageState::IDLE),
  configured_(false), active_(false),
  speed_limit_(std::numeric_limits<double>::max()),
  speed_limit_percentage_(false)
{
  // 初始化默认配置
  config_.passage_velocity = 0.15;
  config_.max_passage_velocity = 0.2;
  config_.angle_correction_gain = 2.0;
  config_.lateral_correction_gain = 1.5;
  config_.precision_mode = true;
  config_.precision_safety_distance = 0.05;
  config_.min_passage_time = 2.0;
  config_.check_exit_space = true;
  config_.robot_width = 0.4;
  config_.robot_length = 0.5;
  config_.wheel_separation = 0.4;
  config_.safety_margin = 0.1;
  config_.emergency_stop_distance = 0.05;
}

void NarrowPassagePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  planner_name_ = name;
  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  logger_ = node->get_logger();

  // 声明并获取参数
  declare_parameter_if_not_declared(
    node, planner_name_ + ".passage_velocity", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".max_passage_velocity", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".angle_correction_gain", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".lateral_correction_gain", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".robot_width", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".safety_margin", rclcpp::ParameterValue(0.1));

  node->get_parameter(planner_name_ + ".passage_velocity", config_.passage_velocity);
  node->get_parameter(planner_name_ + ".max_passage_velocity", config_.max_passage_velocity);
  node->get_parameter(planner_name_ + ".angle_correction_gain", config_.angle_correction_gain);
  node->get_parameter(planner_name_ + ".lateral_correction_gain", config_.lateral_correction_gain);
  node->get_parameter(planner_name_ + ".robot_width", config_.robot_width);
  node->get_parameter(planner_name_ + ".safety_margin", config_.safety_margin);

  RCLCPP_INFO(
    logger_,
    "Configured narrow_passage_planner with passage_velocity=%.2f, robot_width=%.2f",
    config_.passage_velocity,
    config_.robot_width);

  configured_ = true;
}

void NarrowPassagePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up narrow_passage_planner %s", planner_name_.c_str());
}

void NarrowPassagePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating narrow_passage_planner %s", planner_name_.c_str());
  active_ = true;
  last_time_ = rclcpp::Time(0);
}

void NarrowPassagePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating narrow_passage_planner %s", planner_name_.c_str());
  active_ = false;
}

void NarrowPassagePlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
}

geometry_msgs::msg::TwistStamped NarrowPassagePlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (!configured_) {
    RCLCPP_ERROR(logger_, "Planner not configured");
    return geometry_msgs::msg::TwistStamped();
  }

  if (!active_) {
    RCLCPP_WARN(logger_, "Planner not active");
    return geometry_msgs::msg::TwistStamped();
  }

  // 检查是否需要窄道通行模式
  if (!passage_detector_ || !needsPassageMode(pose, velocity)) {
    current_state_ = PassageState::IDLE;

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = pose.header.stamp;
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // 获取当前窄道
  if (current_state_ == PassageState::IDLE) {
    current_passage_ = passage_detector_->getClosestPassage(pose);
    passage_start_time_ = pose.header.stamp;
    current_state_ = PassageState::APPROACHING;
    RCLCPP_INFO(logger_, "Entering narrow passage at center (%.2f, %.2f)",
      current_passage_.centerline_x, current_passage_.centerline_y);
  }

  // 更新通行状态
  updatePassageState(pose, current_passage_);

  // 根据状态计算控制
  geometry_msgs::msg::Twist cmd_vel;
  switch (current_state_) {
    case PassageState::APPROACHING:
      cmd_vel = computeApproachingControl(pose, velocity, current_passage_);
      break;
    case PassageState::ENTERING:
      cmd_vel = computeEnteringControl(pose, velocity, current_passage_);
      break;
    case PassageState::PASSING:
      cmd_vel = computePassingControl(pose, velocity, current_passage_);
      break;
    case PassageState::EXITING:
      cmd_vel = computeExitingControl(pose, velocity, current_passage_);
      break;
    case PassageState::COMPLETED:
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      break;
    default:
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      break;
  }

  // 检查紧急停止
  if (checkEmergencyStop(pose, velocity, current_passage_)) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    RCLCPP_WARN(logger_, "Emergency stop in narrow passage!");
  }

  // 应用安全约束
  cmd_vel = applySafetyConstraints(cmd_vel, pose, current_passage_);

  // 构建返回消息
  geometry_msgs::msg::TwistStamped cmd_vel_stamped;
  cmd_vel_stamped.header.stamp = pose.header.stamp;
  cmd_vel_stamped.header.frame_id = "base_link";
  cmd_vel_stamped.twist = cmd_vel;

  return cmd_vel_stamped;
}

void NarrowPassagePlanner::setPlan(const nav_msgs::msg::Path & path)
{
  current_path_ = path;
  RCLCPP_INFO(logger_, "Set new plan with %zu points", path.poses.size());
}

void NarrowPassagePlanner::setPassageDetector(std::shared_ptr<NarrowPassageDetector> detector)
{
  passage_detector_ = detector;
}

bool NarrowPassagePlanner::needsPassageMode(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  if (!passage_detector_) {
    return false;
  }

  // 检查是否在窄道中或即将进入窄道
  if (passage_detector_->isInNarrowPassage(pose)) {
    return true;
  }

  // 检查前方是否有窄道
  std::vector<NarrowPassage> front_passages = passage_detector_->detectFrontPassages(pose);

  for (const auto & passage : front_passages) {
    // 检查路径是否经过该通道
    if (passage.is_traversable) {
      return true;
    }
  }

  return false;
}

geometry_msgs::msg::Twist NarrowPassagePlanner::computeApproachingControl(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  const NarrowPassage & passage)
{
  // 接近阶段的控制：减速并对齐通道入口
  geometry_msgs::msg::Twist cmd_vel;

  // 计算到通道入口的距离和方向
  double dx = passage.start_x - pose.pose.position.x;
  double dy = passage.start_y - pose.pose.position.y;
  double dist = std::sqrt(dx * dx + dy * dy);

  // 计算目标方向（通道入口方向）
  double target_angle = std::atan2(dy, dx);

  // 计算当前朝向
  double current_theta = tf2::getYaw(pose.pose.orientation);

  // 计算角度误差
  double angle_error = normalizeAngle(target_angle - current_theta);

  // 线速度：根据距离调整
  if (std::abs(angle_error) < M_PI / 6) {  // 角度误差小于30度时前进
    double vel_scale = std::min(dist / 1.0, 1.0);
    cmd_vel.linear.x = vel_scale * config_.passage_velocity;
  } else {
    cmd_vel.linear.x = 0.0;
  }

  // 角速度：对齐通道入口
  cmd_vel.angular.z = config_.angle_correction_gain * angle_error;

  // 应用速度限制
  cmd_vel.linear.x = std::clamp(cmd_vel.linear.x, 0.0, config_.max_passage_velocity);
  cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -0.5, 0.5);

  RCLCPP_DEBUG(logger_, "Approaching passage: dist=%.2f, angle_error=%.2f, cmd=(%.2f, %.2f)",
    dist, angle_error, cmd_vel.linear.x, cmd_vel.angular.z);

  return cmd_vel;
}

geometry_msgs::msg::Twist NarrowPassagePlanner::computeEnteringControl(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  const NarrowPassage & passage)
{
  // 进入阶段的控制：低速进入并调整姿态
  geometry_msgs::msg::Twist cmd_vel = computeCenterlineTracking(pose, passage);

  // 降低速度进入通道
  cmd_vel.linear.x *= 0.7;

  return cmd_vel;
}

geometry_msgs::msg::Twist NarrowPassagePlanner::computePassingControl(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  const NarrowPassage & passage)
{
  // 通过阶段的控制：中心线跟踪 + 姿态修正
  geometry_msgs::msg::Twist cmd_vel = computeCenterlineTracking(pose, passage);

  // 根据横向误差调整速度
  double lateral_error = computeLateralError(pose, passage);
  double recommended_vel = computeRecommendedVelocity(passage, lateral_error);

  cmd_vel.linear.x = recommended_vel;

  return cmd_vel;
}

geometry_msgs::msg::Twist NarrowPassagePlanner::computeExitingControl(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  const NarrowPassage & passage)
{
  // 离开阶段的控制：保持方向，增加速度
  geometry_msgs::msg::Twist cmd_vel = computeCenterlineTracking(pose, passage);

  // 增加速度离开通道
  cmd_vel.linear.x *= 1.3;

  return cmd_vel;
}

geometry_msgs::msg::Twist NarrowPassagePlanner::computeCenterlineTracking(
  const geometry_msgs::msg::PoseStamped & pose,
  const NarrowPassage & passage)
{
  // 中心线跟踪控制
  geometry_msgs::msg::Twist cmd_vel = computePoseCorrection(pose, passage);

  // 根据通道宽度设置基础速度
  double base_vel = config_.passage_velocity;
  cmd_vel.linear.x = base_vel;

  return cmd_vel;
}

geometry_msgs::msg::Twist NarrowPassagePlanner::computePoseCorrection(
  const geometry_msgs::msg::PoseStamped & pose,
  const NarrowPassage & passage)
{
  // 姿态修正控制
  geometry_msgs::msg::Twist cmd_vel;

  // 计算横向误差（到中心线的距离）
  double lateral_error = computeLateralError(pose, passage);

  // 计算航向误差（与通道方向的偏差）
  double heading_error = computeHeadingError(pose, passage);

  // 线速度：根据横向误差和航向误差调整
  // 如果误差较大，减速修正
  double error_factor = 1.0 - std::min(std::abs(lateral_error) + std::abs(heading_error), 1.0);
  cmd_vel.linear.x = config_.passage_velocity * error_factor;

  // 角速度：修正航向和横向位置
  // heading_correction = k_theta * heading_error
  // lateral_correction = k_y * lateral_error
  cmd_vel.angular.z = config_.angle_correction_gain * heading_error +
    config_.lateral_correction_gain * lateral_error;

  // 应用速度限制
  cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -0.3, 0.3);

  return cmd_vel;
}

double NarrowPassagePlanner::computeLateralError(
  const geometry_msgs::msg::PoseStamped & pose,
  const NarrowPassage & passage)
{
  // 计算到中心线的横向误差
  double dx = pose.pose.position.x - passage.start_x;
  double dy = pose.pose.position.y - passage.start_y;

  // 计算通道方向向量
  double passage_dx = passage.end_x - passage.start_x;
  double passage_dy = passage.end_y - passage.start_y;
  double passage_length = std::sqrt(passage_dx * passage_dx + passage_dy * passage_dy);

  if (passage_length < 0.001) {
    return 0.0;
  }

  // 计算横向误差（使用叉乘）
  // lateral_error = ((dx, dy) x (passage_dx, passage_dy)) / passage_length
  double lateral_error = (dx * passage_dy - dy * passage_dx) / passage_length;

  return lateral_error;
}

double NarrowPassagePlanner::computeHeadingError(
  const geometry_msgs::msg::PoseStamped & pose,
  const NarrowPassage & passage)
{
  // 计算航向误差
  double current_theta = tf2::getYaw(pose.pose.orientation);
  double passage_theta = passage.orientation;

  return normalizeAngle(passage_theta - current_theta);
}

void NarrowPassagePlanner::updatePassageState(
  const geometry_msgs::msg::PoseStamped & pose,
  const NarrowPassage & passage)
{
  // 计算当前位置相对于通道的位置
  double dist_to_start = std::sqrt(
    std::pow(pose.pose.position.x - passage.start_x, 2) +
    std::pow(pose.pose.position.y - passage.start_y, 2));

  double dist_to_end = std::sqrt(
    std::pow(pose.pose.position.x - passage.end_x, 2) +
    std::pow(pose.pose.position.y - passage.end_y, 2));

  // 计算沿通道方向的进度
  double dx = pose.pose.position.x - passage.start_x;
  double dy = pose.pose.position.y - passage.start_y;
  double progress = dx * std::cos(passage.orientation) + dy * std::sin(passage.orientation);

  // 更新状态
  if (progress < 0.0 && dist_to_start > config_.robot_width) {
    current_state_ = PassageState::APPROACHING;
  } else if (progress < config_.robot_length) {
    current_state_ = PassageState::ENTERING;
  } else if (progress < passage.length - config_.robot_length) {
    current_state_ = PassageState::PASSING;
  } else if (progress < passage.length) {
    current_state_ = PassageState::EXITING;
  } else {
    current_state_ = PassageState::COMPLETED;
    RCLCPP_INFO(logger_, "Narrow passage completed");
  }

  RCLCPP_DEBUG(logger_, "Passage state: %d, progress=%.2f", static_cast<int>(current_state_), progress);
}

bool NarrowPassagePlanner::isPassageCompleted(
  const geometry_msgs::msg::PoseStamped & pose,
  const NarrowPassage & passage)
{
  // 检查是否完成通行
  double dist_to_end = std::sqrt(
    std::pow(pose.pose.position.x - passage.end_x, 2) +
    std::pow(pose.pose.position.y - passage.end_y, 2));

  return dist_to_end > config_.robot_width;
}

double NarrowPassagePlanner::computeRecommendedVelocity(
  const NarrowPassage & passage,
  double lateral_error)
{
  // 根据通道宽度和横向误差计算推荐速度
  double width_factor = (passage.width - config_.robot_width) / config_.robot_width;
  double error_factor = 1.0 - std::min(std::abs(lateral_error) / (passage.width / 2.0), 0.5);

  double velocity = config_.passage_velocity * width_factor * error_factor;

  return std::clamp(velocity, 0.05, config_.max_passage_velocity);
}

geometry_msgs::msg::Twist NarrowPassagePlanner::applySafetyConstraints(
  const geometry_msgs::msg::Twist & cmd_vel,
  const geometry_msgs::msg::PoseStamped & pose,
  const NarrowPassage & passage)
{
  geometry_msgs::msg::Twist safe_vel = cmd_vel;

  // 计算到通道边缘的距离
  double lateral_error = std::abs(computeLateralError(pose, passage));
  double margin = (passage.width / 2.0) - lateral_error;

  // 如果距离边缘太近，降低速度
  if (margin < config_.safety_margin) {
    safe_vel.linear.x *= 0.5;
    RCLCPP_WARN(logger_, "Too close to passage edge: margin=%.2f", margin);
  }

  // 应用速度限制
  safe_vel.linear.x = std::clamp(safe_vel.linear.x, 0.0, config_.max_passage_velocity);
  safe_vel.angular.z = std::clamp(safe_vel.angular.z, -0.3, 0.3);

  return safe_vel;
}

double NarrowPassagePlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

bool NarrowPassagePlanner::checkEmergencyStop(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  const NarrowPassage & passage)
{
  // 计算到通道边缘的距离
  double lateral_error = std::abs(computeLateralError(pose, passage));
  double margin = (passage.width / 2.0) - lateral_error;

  // 如果太接近边缘，紧急停止
  return margin < config_.emergency_stop_distance;
}

} // namespace narrow_passage
} // namespace rosiwit_navigation