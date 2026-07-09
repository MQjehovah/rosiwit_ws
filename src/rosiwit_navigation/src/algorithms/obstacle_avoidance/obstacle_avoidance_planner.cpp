// ============================================================
// Diffbot Navigation - 避障规划器实现
// ============================================================

#include "rosiwit_navigation/algorithms/obstacle_avoidance_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

namespace rosiwit_navigation
{
namespace obstacle_avoidance
{

using nav2_util::declare_parameter_if_not_declared;

ObstacleAvoidancePlanner::ObstacleAvoidancePlanner()
: configured_(false), active_(false),
  speed_limit_(std::numeric_limits<double>::max()),
  speed_limit_percentage_(false),
  in_avoidance_mode_(false),
  avoidance_start_time_(0.0)
{
  // 初始化默认配置
  config_.mode = AvoidanceMode::HYBRID;
  config_.avoidance_radius = 0.3;
  config_.avoidance_velocity = 0.2;
  config_.min_avoidance_distance = 0.5;
  config_.allow_reverse = false;
  config_.max_avoidance_time = 10.0;
  config_.enable_prediction = true;
  config_.prediction_time = 1.0;
  config_.prediction_step = 0.1;
  config_.weight_obstacle_distance = 0.5;
  config_.weight_path_alignment = 0.3;
  config_.weight_goal_distance = 0.2;
  config_.weight_velocity_alignment = 0.1;
  config_.weight_acceleration = 0.1;
  config_.max_velocity_x = 0.3;
  config_.max_velocity_theta = 0.5;
  config_.safe_distance = 0.3;
  config_.emergency_stop_distance = 0.2;
}

void ObstacleAvoidancePlanner::configure(
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
    node, planner_name_ + ".avoidance_radius", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".avoidance_velocity", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".safe_distance", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".emergency_stop_distance", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".enable_prediction", rclcpp::ParameterValue(true));

  node->get_parameter(planner_name_ + ".avoidance_radius", config_.avoidance_radius);
  node->get_parameter(planner_name_ + ".avoidance_velocity", config_.avoidance_velocity);
  node->get_parameter(planner_name_ + ".safe_distance", config_.safe_distance);
  node->get_parameter(planner_name_ + ".emergency_stop_distance", config_.emergency_stop_distance);
  node->get_parameter(planner_name_ + ".enable_prediction", config_.enable_prediction);

  RCLCPP_INFO(
    logger_,
    "Configured obstacle_avoidance_planner with avoidance_radius=%.2f, safe_distance=%.2f",
    config_.avoidance_radius,
    config_.safe_distance);

  configured_ = true;
}

void ObstacleAvoidancePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up obstacle_avoidance_planner %s", planner_name_.c_str());
}

void ObstacleAvoidancePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating obstacle_avoidance_planner %s", planner_name_.c_str());
  active_ = true;
  last_time_ = rclcpp::Time(0);
}

void ObstacleAvoidancePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating obstacle_avoidance_planner %s", planner_name_.c_str());
  active_ = false;
}

void ObstacleAvoidancePlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
}

geometry_msgs::msg::TwistStamped ObstacleAvoidancePlanner::computeVelocityCommands(
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

  // 检查是否需要避障
  if (!needsAvoidance(pose, velocity)) {
    in_avoidance_mode_ = false;

    // 返回零速度（避障结束后）
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = pose.header.stamp;
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // 开始避障
  in_avoidance_mode_ = true;
  avoidance_start_time_ = rclcpp::Time(pose.header.stamp).seconds();

  // 根据模式选择避障策略
  geometry_msgs::msg::Twist cmd_vel;
  switch (config_.mode) {
    case AvoidanceMode::DYNAMIC_WINDOW:
      cmd_vel = computeDynamicWindowAvoidance(pose, velocity);
      break;
    case AvoidanceMode::POTENTIAL_FIELD:
      cmd_vel = computePotentialFieldAvoidance(pose, velocity);
      break;
    case AvoidanceMode::HYBRID:
      cmd_vel = computeHybridAvoidance(pose, velocity);
      break;
    default:
      cmd_vel = computeHybridAvoidance(pose, velocity);
      break;
  }

  // 检查紧急停止
  if (checkEmergencyStop(pose, velocity)) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    RCLCPP_WARN(logger_, "Emergency stop triggered!");
  }

  // 构建返回消息
  geometry_msgs::msg::TwistStamped cmd_vel_stamped;
  cmd_vel_stamped.header.stamp = pose.header.stamp;
  cmd_vel_stamped.header.frame_id = "base_link";
  cmd_vel_stamped.twist = cmd_vel;

  return cmd_vel_stamped;
}

void ObstacleAvoidancePlanner::setPlan(const nav_msgs::msg::Path & path)
{
  current_path_ = path;
  RCLCPP_INFO(logger_, "Set new plan with %zu points", path.poses.size());
}

void ObstacleAvoidancePlanner::setObstacleDetector(std::shared_ptr<ObstacleDetector> detector)
{
  obstacle_detector_ = detector;
}

bool ObstacleAvoidancePlanner::needsAvoidance(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  if (!obstacle_detector_ || !obstacle_detector_->hasObstacles()) {
    return false;
  }

  // 获取前方障碍物
  std::vector<Obstacle> obstacles = obstacle_detector_->getObstaclesWithinDistance(
    config_.avoidance_radius);

  for (const auto & obs : obstacles) {
    // 检查是否在安全距离内
    if (obs.distance < config_.safe_distance) {
      RCLCPP_DEBUG(logger_, "Obstacle detected at distance %.2f m", obs.distance);
      return true;
    }

    // 检查是否在前方
    if (std::abs(obs.angle) < M_PI / 4 && obs.distance < config_.avoidance_radius) {
      RCLCPP_DEBUG(logger_, "Front obstacle at angle %.2f, distance %.2f m",
        obs.angle, obs.distance);
      return true;
    }
  }

  return false;
}

geometry_msgs::msg::Twist ObstacleAvoidancePlanner::computeDynamicWindowAvoidance(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  // 生成速度样本
  std::vector<VelocitySample> samples = generateVelocitySamples(velocity);

  // 获取障碍物
  std::vector<Obstacle> obstacles = obstacle_detector_->getObstacles();

  // 预测障碍物位置
  if (config_.enable_prediction) {
    obstacles = predictObstacles(obstacles, config_.prediction_time);
  }

  // 评估每个速度样本
  for (auto & sample : samples) {
    sample.score = evaluateVelocitySample(sample, pose, obstacles);
  }

  // 选择最优速度样本
  VelocitySample best = selectBestVelocitySample(samples);

  // 构建速度命令
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = best.v_x;
  cmd_vel.angular.z = best.v_theta;

  return cmd_vel;
}

geometry_msgs::msg::Twist ObstacleAvoidancePlanner::computePotentialFieldAvoidance(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  // 计算斥力场
  std::vector<Obstacle> obstacles = obstacle_detector_->getObstacles();
  geometry_msgs::msg::Point repulsive_force = computeRepulsiveForce(pose, obstacles);

  // 计算引力场
  if (current_path_.poses.empty()) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.pose = current_path_.poses.back().pose;
  geometry_msgs::msg::Point attractive_force = computeAttractiveForce(pose, goal);

  // 合成力
  double force_x = repulsive_force.x + attractive_force.x;
  double force_y = repulsive_force.y + attractive_force.y;

  // 计算速度
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = std::clamp(force_x, -config_.max_velocity_x, config_.max_velocity_x);
  cmd_vel.angular.z = std::clamp(force_y, -config_.max_velocity_theta, config_.max_velocity_theta);

  return cmd_vel;
}

geometry_msgs::msg::Twist ObstacleAvoidancePlanner::computeHybridAvoidance(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  // 混合模式：结合动态窗口和人工势场法
  geometry_msgs::msg::Twist dwa_vel = computeDynamicWindowAvoidance(pose, velocity);
  geometry_msgs::msg::Twist apf_vel = computePotentialFieldAvoidance(pose, velocity);

  // 权重融合
  double dwa_weight = 0.6;
  double apf_weight = 0.4;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = dwa_weight * dwa_vel.linear.x + apf_weight * apf_vel.linear.x;
  cmd_vel.angular.z = dwa_weight * dwa_vel.angular.z + apf_weight * apf_vel.angular.z;

  // 应用速度限制
  cmd_vel.linear.x = std::clamp(cmd_vel.linear.x, -config_.max_velocity_x, config_.max_velocity_x);
  cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -config_.max_velocity_theta, config_.max_velocity_theta);

  return cmd_vel;
}

std::vector<VelocitySample> ObstacleAvoidancePlanner::generateVelocitySamples(
  const geometry_msgs::msg::Twist & current_velocity)
{
  std::vector<VelocitySample> samples;

  // 生成速度样本网格
  int linear_samples = 5;
  int angular_samples = 7;

  double linear_step = (config_.max_velocity_x - (-config_.max_velocity_x)) / linear_samples;
  double angular_step = (config_.max_velocity_theta - (-config_.max_velocity_theta)) / angular_samples;

  for (int i = 0; i <= linear_samples; ++i) {
    for (int j = 0; j <= angular_samples; ++j) {
      VelocitySample sample;
      sample.v_x = -config_.max_velocity_x + i * linear_step;
      sample.v_theta = -config_.max_velocity_theta + j * angular_step;

      // 排除不合理的样本（如速度过大）
      samples.push_back(sample);
    }
  }

  return samples;
}

double ObstacleAvoidancePlanner::evaluateVelocitySample(
  const VelocitySample & sample,
  const geometry_msgs::msg::PoseStamped & pose,
  const std::vector<Obstacle> & obstacles)
{
  double score = 0.0;

  // 障碍物距离代价
  double obstacle_cost = computeObstacleCost(sample, pose, obstacles);
  score -= config_.weight_obstacle_distance * obstacle_cost;

  // 路径对齐代价
  double path_alignment_cost = computePathAlignmentCost(sample, pose);
  score -= config_.weight_path_alignment * path_alignment_cost;

  // 目标距离代价
  double goal_distance_cost = computeGoalDistanceCost(sample, pose);
  score -= config_.weight_goal_distance * goal_distance_cost;

  // 速度对齐代价
  double velocity_alignment = std::abs(sample.v_theta);
  score -= config_.weight_velocity_alignment * velocity_alignment;

  return score;
}

double ObstacleAvoidancePlanner::computeObstacleCost(
  const VelocitySample & sample,
  const geometry_msgs::msg::PoseStamped & pose,
  const std::vector<Obstacle> & obstacles)
{
  double cost = 0.0;

  // 模拟轨迹并计算与障碍物的最小距离
  double sim_time = config_.prediction_time;
  double sim_step = config_.prediction_step;

  double current_x = pose.pose.position.x;
  double current_y = pose.pose.position.y;
  double current_theta = tf2::getYaw(pose.pose.orientation);

  for (double t = 0; t < sim_time; t += sim_step) {
    // 更新位置
    current_x += sample.v_x * std::cos(current_theta) * sim_step;
    current_y += sample.v_x * std::sin(current_theta) * sim_step;
    current_theta += sample.v_theta * sim_step;

    // 检查与障碍物的距离
    for (const auto & obs : obstacles) {
      double dx = current_x - obs.x;
      double dy = current_y - obs.y;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist < config_.safe_distance) {
        // 碰撞代价很高
        cost += 100.0;
      } else {
        // 根据距离计算代价
        cost += 1.0 / dist;
      }
    }
  }

  return cost;
}

double ObstacleAvoidancePlanner::computePathAlignmentCost(
  const VelocitySample & sample,
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (current_path_.poses.empty()) {
    return 0.0;
  }

  // 计算速度方向与路径方向的对齐程度
  double current_theta = tf2::getYaw(pose.pose.orientation);
  double velocity_angle = current_theta + sample.v_theta * config_.prediction_time;

  // 找到路径上最近点的方向
  double path_direction = 0.0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 1; i < current_path_.poses.size(); ++i) {
    double dx = current_path_.poses[i].pose.position.x - pose.pose.position.x;
    double dy = current_path_.poses[i].pose.position.y - pose.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < min_dist) {
      min_dist = dist;
      path_direction = std::atan2(dy, dx);
    }
  }

  // 计算角度差
  double angle_diff = std::abs(velocity_angle - path_direction);
  return angle_diff;
}

double ObstacleAvoidancePlanner::computeGoalDistanceCost(
  const VelocitySample & sample,
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (current_path_.poses.empty()) {
    return 0.0;
  }

  // 模拟轨迹后的位置到目标的距离
  double sim_time = config_.prediction_time;

  double future_x = pose.pose.position.x + sample.v_x * std::cos(
    tf2::getYaw(pose.pose.orientation)) * sim_time;
  double future_y = pose.pose.position.y + sample.v_x * std::sin(
    tf2::getYaw(pose.pose.orientation)) * sim_time;

  // 计算到目标的距离
  double goal_x = current_path_.poses.back().pose.position.x;
  double goal_y = current_path_.poses.back().pose.position.y;

  double dx = future_x - goal_x;
  double dy = future_y - goal_y;
  double dist = std::sqrt(dx * dx + dy * dy);

  return dist;
}

std::vector<Obstacle> ObstacleAvoidancePlanner::predictObstacles(
  const std::vector<Obstacle> & obstacles,
  double time)
{
  std::vector<Obstacle> predicted = obstacles;

  for (auto & obs : predicted) {
    // 预测障碍物位置
    obs.x += obs.velocity_x * time;
    obs.y += obs.velocity_y * time;
  }

  return predicted;
}

geometry_msgs::msg::Point ObstacleAvoidancePlanner::computeRepulsiveForce(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::vector<Obstacle> & obstacles)
{
  geometry_msgs::msg::Point force;
  force.x = 0.0;
  force.y = 0.0;

  for (const auto & obs : obstacles) {
    double dx = pose.pose.position.x - obs.x;
    double dy = pose.pose.position.y - obs.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < config_.safe_distance) {
      // 强斥力
      double magnitude = 10.0 / (dist * dist);
      force.x += magnitude * dx / dist;
      force.y += magnitude * dy / dist;
    } else if (dist < config_.avoidance_radius) {
      // 中等斥力
      double magnitude = 1.0 / dist;
      force.x += magnitude * dx / dist;
      force.y += magnitude * dy / dist;
    }
  }

  return force;
}

geometry_msgs::msg::Point ObstacleAvoidancePlanner::computeAttractiveForce(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::PoseStamped & goal)
{
  geometry_msgs::msg::Point force;

  double dx = goal.pose.position.x - pose.pose.position.x;
  double dy = goal.pose.position.y - pose.pose.position.y;
  double dist = std::sqrt(dx * dx + dy * dy);

  // 引力大小
  double magnitude = 0.5 * dist;  // 简化的引力公式

  force.x = magnitude * dx / dist;
  force.y = magnitude * dy / dist;

  return force;
}

VelocitySample ObstacleAvoidancePlanner::selectBestVelocitySample(
  const std::vector<VelocitySample> & samples)
{
  if (samples.empty()) {
    return VelocitySample(0.0, 0.0);
  }

  VelocitySample best = samples[0];
  for (const auto & sample : samples) {
    if (sample.score > best.score) {
      best = sample;
    }
  }

  return best;
}

bool ObstacleAvoidancePlanner::checkEmergencyStop(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  if (!obstacle_detector_) {
    return false;
  }

  // 获取非常近的障碍物
  std::vector<Obstacle> obstacles = obstacle_detector_->getObstaclesWithinDistance(
    config_.emergency_stop_distance);

  for (const auto & obs : obstacles) {
    // 检查是否在前方
    if (std::abs(obs.angle) < M_PI / 6) {
      return true;  // 需要紧急停止
    }
  }

  return false;
}

} // namespace obstacle_avoidance
} // namespace rosiwit_navigation