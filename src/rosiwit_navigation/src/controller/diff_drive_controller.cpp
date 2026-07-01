// ============================================================
// Diffbot Navigation - 差速轮控制器实现
// ============================================================

#include "diffbot_navigation/controller/diff_drive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

namespace diffbot_navigation
{
namespace controller
{

using nav2_util::declare_parameter_if_not_declared;

//
// PIDController::compute() and reset() 已移到 pid_controller.hpp（inline 实现）
// 带完整抗饱和模式支持：None / ConditionalIntegration / Clamping / BackCalculation
//

DiffDriveController::DiffDriveController()
: configured_(false), active_(false),
  speed_limit_(std::numeric_limits<double>::max()),
  speed_limit_percentage_(false),
  closest_point_idx_(0),
  goal_reached_(false),
  prev_linear_output_(0.0),
  prev_angular_output_(0.0)
{
  // 初始化默认配置
  config_.wheel_separation = 0.4;
  config_.wheel_radius = 0.065;
  config_.max_velocity_x = 0.5;
  config_.max_velocity_theta = 1.0;
  config_.min_velocity_x = -0.5;
  config_.min_velocity_theta = -1.0;
  config_.max_accel_x = 0.5;
  config_.max_accel_theta = 1.0;
  config_.min_accel_x = -0.5;
  config_.min_accel_theta = -1.0;
  config_.lookahead_distance = 0.6;
  config_.min_lookahead_distance = 0.3;
  config_.max_lookahead_distance = 0.9;
  config_.lookahead_gain = 0.5;
  config_.k_p_linear = 2.0;
  config_.k_i_linear = 0.0;
  config_.k_d_linear = 0.0;
  config_.k_p_angular = 1.0;
  config_.k_i_angular = 0.0;
  config_.k_d_angular = 0.0;
  config_.xy_goal_tolerance = 0.1;
  config_.yaw_goal_tolerance = 0.1;
  config_.controller_frequency = 20.0;
}

void DiffDriveController::configure(const Config& config)
{
  simple_config_ = config;

  // 将简化配置同步到 PID 控制器
  linear_pid_.k_p = config.kp_linear;
  linear_pid_.k_i = config.ki_linear;
  linear_pid_.k_d = config.kd_linear;
  linear_pid_.integral_limit = config.linear_integral_limit;
  linear_pid_.output_limit = config.max_linear_velocity;
  linear_pid_.setMode(AntiWindupMode::ConditionalIntegration);
  linear_pid_.reset();

  angular_pid_.k_p = config.kp_angular;
  angular_pid_.k_i = config.ki_angular;
  angular_pid_.k_d = config.kd_angular;
  angular_pid_.integral_limit = config.angular_integral_limit;
  angular_pid_.output_limit = config.max_angular_velocity;
  angular_pid_.setMode(AntiWindupMode::ConditionalIntegration);
  angular_pid_.reset();

  prev_linear_output_ = 0.0;
  prev_angular_output_ = 0.0;
}

geometry_msgs::msg::Twist DiffDriveController::compute(double error, double dt)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  // dt <= 0 时返回零输出，不更新内部状态（防止除零和不稳定的导数项）
  if (dt <= 0.0) {
    return cmd;
  }

  // ========================================
  // 线性速度 PID（委托给 PIDController::compute()，含完整抗饱和逻辑）
  // ========================================
  double linear_output = linear_pid_.compute(error, 0.0, dt);
  // 加速度限制（增量裁剪，在 PID 输出限幅之后再叠加）
  if (dt > PIDConstants::kMinDt) {
    double max_delta = simple_config_.max_linear_accel * dt;
    double delta = linear_output - prev_linear_output_;
    delta = std::max(-max_delta, std::min(max_delta, delta));
    linear_output = prev_linear_output_ + delta;
  }

  // ========================================
  // 角速度 PID（委托给 PIDController::compute()，含完整抗饱和逻辑）
  // ========================================
  double angular_output = angular_pid_.compute(error, 0.0, dt);

  // 角速度加速度限制
  if (dt > PIDConstants::kMinDt) {
    double max_delta = simple_config_.max_angular_accel * dt;
    double delta = angular_output - prev_angular_output_;
    delta = std::max(-max_delta, std::min(max_delta, delta));
    angular_output = prev_angular_output_ + delta;
  }

  // 更新状态
  prev_linear_output_ = linear_output;
  prev_angular_output_ = angular_output;

  cmd.linear.x = linear_output;
  cmd.angular.z = angular_output;

  return cmd;
}

geometry_msgs::msg::Twist DiffDriveController::compute(double linear_error, double dt, double angular_error)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  if (dt <= 0.0) {
    return cmd;
  }

  // 线性速度 PID
  double linear_output = linear_pid_.compute(linear_error, 0.0, dt);
  if (dt > PIDConstants::kMinDt) {
    double max_delta = simple_config_.max_linear_accel * dt;
    double delta = linear_output - prev_linear_output_;
    delta = std::max(-max_delta, std::min(max_delta, delta));
    linear_output = prev_linear_output_ + delta;
  }

  // 角速度 PID
  double angular_output = angular_pid_.compute(angular_error, 0.0, dt);
  if (dt > PIDConstants::kMinDt) {
    double max_delta = simple_config_.max_angular_accel * dt;
    double delta = angular_output - prev_angular_output_;
    delta = std::max(-max_delta, std::min(max_delta, delta));
    angular_output = prev_angular_output_ + delta;
  }

  prev_linear_output_ = linear_output;
  prev_angular_output_ = angular_output;

  cmd.linear.x = linear_output;
  cmd.angular.z = angular_output;

  return cmd;
}

void DiffDriveController::reset()
{
  linear_pid_.reset();
  angular_pid_.reset();
  prev_linear_output_ = 0.0;
  prev_angular_output_ = 0.0;
}

void DiffDriveController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  controller_name_ = name;
  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  logger_ = node->get_logger();

  // 声明并获取参数
  declare_parameter_if_not_declared(
    node, controller_name_ + ".wheel_separation", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, controller_name_ + ".wheel_radius", rclcpp::ParameterValue(0.065));
  declare_parameter_if_not_declared(
    node, controller_name_ + ".max_velocity_x", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, controller_name_ + ".max_velocity_theta", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, controller_name_ + ".lookahead_distance", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, controller_name_ + ".lookahead_gain", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, controller_name_ + ".xy_goal_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, controller_name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.1));

  node->get_parameter(controller_name_ + ".wheel_separation", config_.wheel_separation);
  node->get_parameter(controller_name_ + ".wheel_radius", config_.wheel_radius);
  node->get_parameter(controller_name_ + ".max_velocity_x", config_.max_velocity_x);
  node->get_parameter(controller_name_ + ".max_velocity_theta", config_.max_velocity_theta);
  node->get_parameter(controller_name_ + ".lookahead_distance", config_.lookahead_distance);
  node->get_parameter(controller_name_ + ".lookahead_gain", config_.lookahead_gain);
  node->get_parameter(controller_name_ + ".xy_goal_tolerance", config_.xy_goal_tolerance);
  node->get_parameter(controller_name_ + ".yaw_goal_tolerance", config_.yaw_goal_tolerance);

  // 初始化PID控制器
  linear_pid_.k_p = config_.k_p_linear;
  linear_pid_.k_i = config_.k_i_linear;
  linear_pid_.k_d = config_.k_d_linear;
  linear_pid_.output_limit = config_.max_velocity_x;
  linear_pid_.integral_limit = config_.max_velocity_x;

  angular_pid_.k_p = config_.k_p_angular;
  angular_pid_.k_i = config_.k_i_angular;
  angular_pid_.k_d = config_.k_d_angular;
  angular_pid_.output_limit = config_.max_velocity_theta;
  angular_pid_.integral_limit = config_.max_velocity_theta;

  RCLCPP_INFO(
    logger_,
    "Configured diff_drive_controller with lookahead=%.2f, tolerance_xy=%.2f",
    config_.lookahead_distance,
    config_.xy_goal_tolerance);

  configured_ = true;
}

void DiffDriveController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up diff_drive_controller %s", controller_name_.c_str());
}

void DiffDriveController::activate()
{
  RCLCPP_INFO(logger_, "Activating diff_drive_controller %s", controller_name_.c_str());
  active_ = true;
  last_time_ = rclcpp::Time(0);
}

void DiffDriveController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating diff_drive_controller %s", controller_name_.c_str());
  active_ = false;
}

void DiffDriveController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
}

geometry_msgs::msg::TwistStamped DiffDriveController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (!configured_) {
    RCLCPP_ERROR(logger_, "Controller not configured");
    return geometry_msgs::msg::TwistStamped();
  }

  if (!active_) {
    RCLCPP_WARN(logger_, "Controller not active");
    return geometry_msgs::msg::TwistStamped();
  }

  // 计算时间间隔
  auto current_time = pose.header.stamp;
  if (last_time_.nanoseconds() == 0) {
    last_time_ = current_time;
  }
  double dt = (rclcpp::Time(current_time) - last_time_).seconds();
  last_time_ = current_time;

  if (dt < 0.001) {
    dt = 0.05;  // 默认时间步长
  }

  // 检查是否到达目标
  if (goal_reached_) {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = current_time;
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // 找到路径上最近的点
  closest_point_idx_ = findClosestPointOnPath(pose);

  // 计算前视点
  geometry_msgs::msg::PoseStamped lookahead_point = computeLookaheadPoint(pose);

  // 计算Pure Pursuit曲率
  double curvature = computePurePursuitCurvature(pose, lookahead_point);

  // 从曲率计算速度命令
  geometry_msgs::msg::Twist cmd_vel = computeVelocityFromCurvature(curvature);

  // 应用加速度约束
  cmd_vel = applyAccelerationConstraints(cmd_vel, velocity, dt);

  // 应用速度限制
  applySpeedLimit(cmd_vel);

  // 构建返回消息
  geometry_msgs::msg::TwistStamped cmd_vel_stamped;
  cmd_vel_stamped.header.stamp = current_time;
  cmd_vel_stamped.header.frame_id = "base_link";
  cmd_vel_stamped.twist = cmd_vel;

  RCLCPP_DEBUG(
    logger_,
    "Computed velocity: linear=%.2f, angular=%.2f, curvature=%.3f",
    cmd_vel.linear.x, cmd_vel.angular.z, curvature);

  return cmd_vel_stamped;
}

void DiffDriveController::setPlan(const nav_msgs::msg::Path & path)
{
  current_path_ = path;
  closest_point_idx_ = 0;
  goal_reached_ = false;

  // 重置PID控制器
  linear_pid_.reset();
  angular_pid_.reset();

  RCLCPP_INFO(
    logger_,
    "Set new plan with %zu points",
    path.poses.size());
}

void DiffDriveController::resetController()
{
  closest_point_idx_ = 0;
  goal_reached_ = false;
  linear_pid_.reset();
  angular_pid_.reset();
  current_path_ = nav_msgs::msg::Path();
}

bool DiffDriveController::isGoalReached(
  const geometry_msgs::msg::PoseStamped & pose,
  nav2_core::GoalChecker * goal_checker)
{
  if (!goal_checker) {
    // 检查到路径终点的距离
    if (current_path_.poses.empty()) {
      return false;
    }

    double dx = current_path_.poses.back().pose.position.x - pose.pose.position.x;
    double dy = current_path_.poses.back().pose.position.y - pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    return distance < config_.xy_goal_tolerance;
  }

  // 使用GoalChecker
  geometry_msgs::msg::Pose goal_pose = current_path_.poses.back().pose;
  geometry_msgs::msg::Pose current_pose = pose.pose;
  geometry_msgs::msg::Twist current_speed;

  return goal_checker->isGoalReached(goal_pose, current_pose, current_speed);
}

geometry_msgs::msg::PoseStamped DiffDriveController::computeLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose)
{
  return interpolateLookaheadPoint(current_pose, closest_point_idx_);
}

void DiffDriveController::computeDistanceAndAngle(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to,
  double & distance,
  double & angle)
{
  double dx = to.pose.position.x - from.pose.position.x;
  double dy = to.pose.position.y - from.pose.position.y;
  distance = std::sqrt(dx * dx + dy * dy);
  angle = std::atan2(dy, dx);
}

double DiffDriveController::computePurePursuitCurvature(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & lookahead_point)
{
  // 计算当前位置到前视点的向量（在机器人坐标系中）
  double dx = lookahead_point.pose.position.x - current_pose.pose.position.x;
  double dy = lookahead_point.pose.position.y - current_pose.pose.position.y;

  // 转换到机器人坐标系
  double robot_theta = tf2::getYaw(current_pose.pose.orientation);
  double dx_robot = dx * std::cos(robot_theta) + dy * std::sin(robot_theta);
  double dy_robot = -dx * std::sin(robot_theta) + dy * std::cos(robot_theta);

  // 计算曲率
  // k = 2 * y / L^2，其中 y 是前视点在机器人坐标系中的 y 坐标，L 是到前视点的距离
  double L = std::sqrt(dx_robot * dx_robot + dy_robot * dy_robot);

  if (L < 0.001) {
    return 0.0;  // 距离太小，曲率接近无穷
  }

  // 曲率计算（考虑 y 坐标）
  double curvature = 2.0 * dy_robot / (L * L);

  return curvature;
}

geometry_msgs::msg::Twist DiffDriveController::computeVelocityFromCurvature(double curvature)
{
  geometry_msgs::msg::Twist cmd_vel;

  // 基于曲率计算角速度
  // 对于差速轮机器人：omega = v * k
  // 其中 v 是线速度，k 是曲率

  // 根据曲率大小调整线速度
  double curvature_threshold = 1.0;  // 大曲率阈值
  if (std::abs(curvature) > curvature_threshold) {
    // 大曲率时降低线速度
    cmd_vel.linear.x = config_.max_velocity_x * 0.5;
  } else {
    // 小曲率时使用正常线速度
    cmd_vel.linear.x = config_.max_velocity_x;
  }

  // 角速度 = 线速度 * 曲率
  cmd_vel.angular.z = cmd_vel.linear.x * curvature;

  // 应用速度限制
  cmd_vel.linear.x = std::clamp(cmd_vel.linear.x,
    config_.min_velocity_x, config_.max_velocity_x);
  cmd_vel.angular.z = std::clamp(cmd_vel.angular.z,
    -config_.max_velocity_theta, config_.max_velocity_theta);

  return cmd_vel;
}

geometry_msgs::msg::Twist DiffDriveController::applyAccelerationConstraints(
  const geometry_msgs::msg::Twist & cmd_vel,
  const geometry_msgs::msg::Twist & current_vel,
  double dt)
{
  geometry_msgs::msg::Twist constrained_vel = cmd_vel;

  // 线速度加速度约束
  double accel_x = (cmd_vel.linear.x - current_vel.linear.x) / dt;
  if (accel_x > config_.max_accel_x) {
    constrained_vel.linear.x = current_vel.linear.x + config_.max_accel_x * dt;
  } else if (accel_x < -config_.max_accel_x) {
    constrained_vel.linear.x = current_vel.linear.x - config_.max_accel_x * dt;
  }

  // 角速度加速度约束
  double accel_theta = (cmd_vel.angular.z - current_vel.angular.z) / dt;
  if (accel_theta > config_.max_accel_theta) {
    constrained_vel.angular.z = current_vel.angular.z + config_.max_accel_theta * dt;
  } else if (accel_theta < -config_.max_accel_theta) {
    constrained_vel.angular.z = current_vel.angular.z - config_.max_accel_theta * dt;
  }

  return constrained_vel;
}

double DiffDriveController::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

int DiffDriveController::findClosestPointOnPath(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (current_path_.poses.empty()) {
    return 0;
  }

  double min_dist = std::numeric_limits<double>::max();
  int min_idx = closest_point_idx_;  // 从当前最近点开始搜索

  // 搜索范围：从当前索引到路径终点
  for (size_t i = closest_point_idx_; i < current_path_.poses.size(); ++i) {
    double dx = current_path_.poses[i].pose.position.x - pose.pose.position.x;
    double dy = current_path_.poses[i].pose.position.y - pose.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  return min_idx;
}

geometry_msgs::msg::PoseStamped DiffDriveController::interpolateLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & pose,
  int closest_idx)
{
  if (current_path_.poses.empty()) {
    return pose;
  }

  // 计算动态前视距离
  double current_speed = std::sqrt(
    pose.pose.position.x * pose.pose.position.x +
    pose.pose.position.y * pose.pose.position.y);
  double lookahead_dist = config_.lookahead_distance +
    config_.lookahead_gain * current_speed;
  lookahead_dist = std::clamp(lookahead_dist,
    config_.min_lookahead_distance,
    config_.max_lookahead_distance);

  // 从最近点开始搜索前视点
  double accumulated_dist = 0.0;
  for (size_t i = closest_idx; i < current_path_.poses.size() - 1; ++i) {
    double dx = current_path_.poses[i + 1].pose.position.x -
      current_path_.poses[i].pose.position.x;
    double dy = current_path_.poses[i + 1].pose.position.y -
      current_path_.poses[i].pose.position.y;
    double segment_dist = std::sqrt(dx * dx + dy * dy);

    if (accumulated_dist + segment_dist >= lookahead_dist) {
      // 在当前线段上插值找到前视点
      double ratio = (lookahead_dist - accumulated_dist) / segment_dist;

      geometry_msgs::msg::PoseStamped lookahead_point;
      lookahead_point.header = current_path_.header;
      lookahead_point.pose.position.x =
        current_path_.poses[i].pose.position.x + ratio * dx;
      lookahead_point.pose.position.y =
        current_path_.poses[i].pose.position.y + ratio * dy;
      lookahead_point.pose.position.z = 0.0;

      // 简化朝向角计算
      double theta = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      lookahead_point.pose.orientation = tf2::toMsg(q);

      return lookahead_point;
    }

    accumulated_dist += segment_dist;
  }

  // 如果前视距离超过路径长度，返回路径终点
  return current_path_.poses.back();
}

void DiffDriveController::applySpeedLimit(geometry_msgs::msg::Twist & cmd_vel)
{
  if (speed_limit_ > 0.0) {
    if (speed_limit_percentage_) {
      // 百分比限制
      cmd_vel.linear.x *= speed_limit_;
      cmd_vel.angular.z *= speed_limit_;
    } else {
      // 绝对值限制
      if (std::abs(cmd_vel.linear.x) > speed_limit_) {
        cmd_vel.linear.x = std::copysign(speed_limit_, cmd_vel.linear.x);
      }
    }
  }
}

} // namespace controller
} // namespace diffbot_navigation