// ============================================================
// Diffbot Navigation - 轨迹生成器实现
// ============================================================

#include "trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>


namespace rosiwit_navigation
{
namespace navigation
{

TrajectoryGenerator::TrajectoryGenerator()
{
  // 初始化默认配置
  config_.max_velocity_x = 0.5;
  config_.max_velocity_theta = 1.0;
  config_.min_velocity_x = -0.5;
  config_.max_accel_x = 0.5;
  config_.max_accel_theta = 1.0;
  config_.sim_time = 1.5;
  config_.sim_granularity = 0.025;
  config_.path_resolution = 0.05;
  config_.xy_goal_tolerance = 0.1;
  config_.yaw_goal_tolerance = 0.1;
}

TrajectoryGenerator::TrajectoryGenerator(const TrajectoryConfig & config)
: config_(config)
{
}

void TrajectoryGenerator::setConfig(const TrajectoryConfig & config)
{
  config_ = config;
}

void TrajectoryGenerator::configure(const Config& config)
{
  Config temp = config;
  // 时间步长安全下限：不低于 0.01s
  if (temp.dt < TrajectoryConstants::kMinDt) {
    temp.dt = TrajectoryConstants::kDefaultDt;
    LOG_WARN(logger_, "dt=%.6f 过小，重置为默认值 %.3f", config.dt, temp.dt);
  }
  // 前视距离合法性检查
  if (temp.min_lookahead_distance > temp.max_lookahead_distance) {
    LOG_WARN(logger_, "min_lookahead > max_lookahead，交换两者");
    std::swap(temp.min_lookahead_distance, temp.max_lookahead_distance);
  }
  simple_config_ = temp;
}

std::vector<TrajectoryPoint> TrajectoryGenerator::generateTrajectory(
  const core::Path & path,
  const core::VelocityCommand & current_velocity)
{
  if (path.points.empty()) {
    LOG_WARN(logger_, "Cannot generate trajectory from empty path");
    return std::vector<TrajectoryPoint>();
  }

  // 检查起始点是否包含 NaN/Inf
  if (std::isnan(path.points[0].pose.x) ||
      std::isnan(path.points[0].pose.y) ||
      std::isinf(path.points[0].pose.x) ||
      std::isinf(path.points[0].pose.y)) {
    LOG_ERROR(logger_,
      "Path start point has NaN/Inf coordinates (%.2f, %.2f)",
      path.points[0].pose.x, path.points[0].pose.y);
    return std::vector<TrajectoryPoint>();
  }

  // 单点路径：返回仅含起点的轨迹，速度为零
  if (path.points.size() == 1) {
    TrajectoryPoint point;
    point.x = path.points[0].pose.x;
    point.y = path.points[0].pose.y;
    point.theta = path.points[0].pose.theta;
    point.vx = 0.0;
    point.vtheta = 0.0;
    point.time = 0.0;
    point.acceleration = 0.0;
    return {point};
  }

  std::vector<TrajectoryPoint> trajectory;

  // 初始化当前点
  core::Pose2D current_pose;
  current_pose.x = path.points[0].pose.x;
  current_pose.y = path.points[0].pose.y;
  current_pose.theta = path.points[0].pose.theta;

  double current_time = 0.0;
  double current_vel_x = current_velocity.linear_x;
  double current_vel_theta = current_velocity.angular_z;

  // 初始速度裁剪（防止从异常速度开始）
  if (std::isnan(current_vel_x) || std::isinf(current_vel_x)) {
    current_vel_x = 0.0;
  }
  if (std::isnan(current_vel_theta) || std::isinf(current_vel_theta)) {
    current_vel_theta = 0.0;
  }
  current_vel_x = std::max(-config_.max_velocity_x,
    std::min(config_.max_velocity_x, current_vel_x));
  current_vel_theta = std::max(-config_.max_velocity_theta,
    std::min(config_.max_velocity_theta, current_vel_theta));

  // 生成轨迹点
  for (size_t i = 1; i < path.points.size(); ++i) {
    // 跳过包含 NaN/Inf 的路径点
    double px = path.points[i].pose.x;
    double py = path.points[i].pose.y;
    if (std::isnan(px) || std::isnan(py) || std::isinf(px) || std::isinf(py)) {
      LOG_WARN(logger_,
        "Skipping path point[%zu] with NaN/Inf coordinates (%.2f, %.2f)",
        i, px, py);
      continue;
    }

    core::Pose2D target_pose;
    target_pose.x = px;
    target_pose.y = py;
    target_pose.theta = path.points[i].pose.theta;

    // 计算到下一个点的距离和方向
    double dx = target_pose.x - current_pose.x;
    double dy = target_pose.y - current_pose.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    double target_angle = std::atan2(dy, dx);

    // 计算角度差
    double angle_diff = normalizeAngle(target_angle - current_pose.theta);

    // 计算目标速度
    double target_vel_x = dist / config_.sim_time;  // 简化的速度计算
    double target_vel_theta = angle_diff / config_.sim_time;

    // 应用速度限制
    target_vel_x = applyVelocityLimit(target_vel_x, config_.max_velocity_x, config_.min_velocity_x);
    target_vel_theta = applyVelocityLimit(target_vel_theta, config_.max_velocity_theta, -config_.max_velocity_theta);

    // 计算移动时间
    double move_time = config_.sim_granularity;
    if (target_vel_x != 0.0 && dist > 0.0) {
      move_time = std::min(dist / target_vel_x, config_.sim_time);
    }

    // 创建轨迹点
    TrajectoryPoint point;
    point.x = current_pose.x;
    point.y = current_pose.y;
    point.theta = current_pose.theta;
    point.vx = target_vel_x;
    point.vtheta = target_vel_theta;
    point.time = current_time;

    trajectory.push_back(point);

    // 更新当前状态
    current_time += move_time;
    current_pose.x += target_vel_x * std::cos(current_pose.theta) * move_time;
    current_pose.y += target_vel_x * std::sin(current_pose.theta) * move_time;
    current_pose.theta = normalizeAngle(current_pose.theta + target_vel_theta * move_time);
    current_vel_x = target_vel_x;
    current_vel_theta = target_vel_theta;
  }

  // 添加终点
  TrajectoryPoint final_point;
  final_point.x = path.points.back().pose.x;
  final_point.y = path.points.back().pose.y;
  final_point.theta = path.points.back().pose.theta;
  final_point.vx = 0.0;
  final_point.vtheta = 0.0;
  final_point.time = current_time;

  trajectory.push_back(final_point);

  // 应用加速度约束
  trajectory = applyAccelerationConstraints(trajectory);

  // DEF-003: 确保极短路径至少有最小数量的轨迹点
  if (trajectory.size() < kMinTrajectoryPoints) {
    // 在起点和终点之间插入等距轨迹点
    const auto & first = trajectory.front();
    const auto & last = trajectory.back();
    const double dt = (last.time - first.time) / kMinTrajectoryPoints;
    trajectory.clear();
    for (size_t i = 0; i < kMinTrajectoryPoints; ++i) {
      TrajectoryPoint pt;
      const double alpha = static_cast<double>(i) / kMinTrajectoryPoints;
      pt.x = first.x + alpha * (last.x - first.x);
      pt.y = first.y + alpha * (last.y - first.y);
      pt.theta = normalizeAngle(first.theta + alpha * normalizeAngle(last.theta - first.theta));
      pt.vx = first.vx + alpha * (last.vx - first.vx);
      pt.vtheta = first.vtheta + alpha * (last.vtheta - first.vtheta);
      pt.time = first.time + static_cast<double>(i) * dt;
      trajectory.push_back(pt);
    }
    // 确保终点在轨迹中
    TrajectoryPoint final_pt = last;
    final_pt.time = first.time + kMinTrajectoryPoints * dt;
    trajectory.push_back(final_pt);
  }

  return trajectory;
}

std::vector<TrajectoryPoint> TrajectoryGenerator::generateTrajectory(
  const core::Pose2D & start,
  const core::Pose2D & goal,
  const core::VelocityCommand & start_vel,
  const core::VelocityCommand & end_vel)
{
  (void)end_vel;
  // 创建简化的路径
  core::Path path;

  // 添加起点
  core::Pose2D start_pose;
  start_pose.x = start.x;
  start_pose.y = start.y;
  start_pose.theta = start.theta;
  path.points.push_back(start_pose);

  // 添加终点
  core::Pose2D goal_pose;
  goal_pose.x = goal.x;
  goal_pose.y = goal.y;
  goal_pose.theta = goal.theta;
  path.points.push_back(goal_pose);

  return generateTrajectory(path, start_vel);
}

std::vector<TrajectoryPoint> TrajectoryGenerator::smoothVelocity(
  const std::vector<TrajectoryPoint> & trajectory)
{
  if (trajectory.size() < 3) {
    return trajectory;
  }

  std::vector<TrajectoryPoint> smoothed = trajectory;

  // 平滑速度曲线（使用简单滤波）
  for (size_t i = 1; i < smoothed.size() - 1; ++i) {
    // 计算平均速度
    double avg_v_x =
      (smoothed[i - 1].vx + smoothed[i].vx + smoothed[i + 1].vx) / 3.0;
    double avg_v_theta =
      (smoothed[i - 1].vtheta + smoothed[i].vtheta + smoothed[i + 1].vtheta) / 3.0;

    smoothed[i].vx = avg_v_x;
    smoothed[i].vtheta = avg_v_theta;
  }

  return smoothed;
}

std::vector<TrajectoryPoint> TrajectoryGenerator::applyAccelerationConstraints(
  const std::vector<TrajectoryPoint> & trajectory)
{
  if (trajectory.size() < 2) {
    return trajectory;
  }

  std::vector<TrajectoryPoint> constrained = trajectory;

  // 计算时间间隔
  double prev_time = trajectory[0].time;
  double prev_v_x = trajectory[0].vx;
  double prev_v_theta = trajectory[0].vtheta;

  for (size_t i = 1; i < constrained.size(); ++i) {
    double dt = constrained[i].time - prev_time;
    if (dt <= 0.0) {
      dt = config_.sim_granularity;  // 默认时间步长
    }

    // 计算所需加速度
    double accel_x = (constrained[i].vx - prev_v_x) / dt;
    double accel_theta = (constrained[i].vtheta - prev_v_theta) / dt;

    // 应用加速度限制
    if (accel_x > config_.max_accel_x) {
      constrained[i].vx = prev_v_x + config_.max_accel_x * dt;
    } else if (accel_x < -config_.max_accel_x) {
      constrained[i].vx = prev_v_x - config_.max_accel_x * dt;
    }

    if (accel_theta > config_.max_accel_theta) {
      constrained[i].vtheta = prev_v_theta + config_.max_accel_theta * dt;
    } else if (accel_theta < -config_.max_accel_theta) {
      constrained[i].vtheta = prev_v_theta - config_.max_accel_theta * dt;
    }

    prev_time = constrained[i].time;
    prev_v_x = constrained[i].vx;
    prev_v_theta = constrained[i].vtheta;
  }

  return constrained;
}

bool TrajectoryGenerator::validateTrajectory(const std::vector<TrajectoryPoint> & trajectory)
{
  if (trajectory.empty()) {
    return false;
  }

  // 检查速度是否在合理范围内
  for (const auto & point : trajectory) {
    if (std::abs(point.vx) > config_.max_velocity_x * 1.5) {
      LOG_WARN(logger_, "Velocity x exceeds limit: %.2f", point.vx);
      return false;
    }

    if (std::abs(point.vtheta) > config_.max_velocity_theta * 1.5) {
      LOG_WARN(logger_, "Velocity theta exceeds limit: %.2f", point.vtheta);
      return false;
    }
  }

  // 检查时间是否递增
  for (size_t i = 1; i < trajectory.size(); ++i) {
    if (trajectory[i].time <= trajectory[i - 1].time) {
      LOG_WARN(logger_, "Time not increasing at point %zu", i);
      return false;
    }
  }

  return true;
}

core::Path TrajectoryGenerator::toPathMsg(const std::vector<TrajectoryPoint> & trajectory)
{
  core::Path path;

  for (const auto & point : trajectory) {
    core::Pose2D pose;
    pose.x = point.x;
    pose.y = point.y;
    


    path.points.push_back(pose);
  }

  return path;
}

double TrajectoryGenerator::distance(
  const core::Pose2D & p1,
  const core::Pose2D & p2)
{
  return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

double TrajectoryGenerator::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double TrajectoryGenerator::computeSteeringAngle(
  const core::Pose2D & current,
  const core::Pose2D & target)
{
  double dx = target.x - current.x;
  double dy = target.y - current.y;
  double target_angle = std::atan2(dy, dx);

  return normalizeAngle(target_angle - current.theta);
}

double TrajectoryGenerator::applyVelocityLimit(double velocity, double max, double min)
{
  if (velocity > max) {
    return max;
  }
  if (velocity < min) {
    return min;
  }
  return velocity;
}

double TrajectoryGenerator::computeStoppingDistance(double velocity)
{
  // 使用最大加速度计算停止距离
  // d = v^2 / (2 * a)
  if (std::abs(velocity) < 0.01) {
    return 0.0;
  }

  return (velocity * velocity) / (2.0 * config_.max_accel_x);
}

} // namespace navigation
} // namespace rosiwit_navigation