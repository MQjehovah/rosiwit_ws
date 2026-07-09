// ============================================================
// Diffbot Navigation - Pure Pursuit 控制器实现
// ============================================================

#include "rosiwit_navigation/algorithms/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rosiwit_navigation
{
namespace controllers
{

PurePursuitController::PurePursuitController()
: controller_name_("PurePursuitController"),
  logger_(rclcpp::get_logger("PurePursuitController")),
  current_waypoint_idx_(0),
  max_linear_velocity_(0.5),
  max_angular_velocity_(1.0),
  lookahead_distance_(0.6),
  goal_tolerance_(0.1),
  initialized_(false),
  goal_reached_(false)
{
}

PurePursuitController::~PurePursuitController()
{
}

bool PurePursuitController::initialize(const core::ControllerConfig& config)
{
    config_ = config;
    lookahead_distance_ = config.lookahead_distance;
    goal_tolerance_ = config.xy_goal_tolerance;
    max_linear_velocity_ = config.kinematics.max_velocity_x;
    max_angular_velocity_ = config.kinematics.max_velocity_theta;
    initialized_ = true;
    
    RCLCPP_INFO(logger_, "Pure Pursuit Controller initialized with lookahead=%.2f, tolerance=%.2f",
                lookahead_distance_, goal_tolerance_);
    return true;
}

void PurePursuitController::setPath(const core::Path& path)
{
    path_ = path;
    current_waypoint_idx_ = 0;
    goal_reached_ = false;
    
    RCLCPP_INFO(logger_, "Path set with %zu waypoints", path_.size());
}

core::VelocityCommand PurePursuitController::computeVelocityCommand(
    const core::Pose2D& current_pose,
    const core::VelocityCommand& current_velocity)
{
    if (!initialized_ || path_.empty()) {
        return core::VelocityCommand(0, 0, 0);
    }

    // 检查是否到达目标
    if (isGoalReached(current_pose)) {
        goal_reached_ = true;
        return core::VelocityCommand(0, 0, 0);
    }

    // 寻找前视点（使用返回索引的版本）
    size_t lookahead_idx = findLookaheadPoint(current_pose);
    const core::PathPoint& lookahead_point = path_.points[lookahead_idx];

    // 使用 Pure Pursuit 算法计算速度命令
    core::VelocityCommand cmd = purePursuitAlgorithm(current_pose, lookahead_point.pose);

    // 根据距离目标调整速度
    double distance_to_goal = current_pose.distanceTo(path_.points.back().pose);
    if (distance_to_goal < config_.slow_down_distance) {
        double slowdown_factor = distance_to_goal / config_.slow_down_distance;
        slowdown_factor = std::max(slowdown_factor, 0.1);
        cmd.linear_x *= slowdown_factor;
    }

    // 应用速度限制
    cmd.linear_x = std::clamp(cmd.linear_x, -max_linear_velocity_, max_linear_velocity_);
    cmd.angular_z = std::clamp(cmd.angular_z, -max_angular_velocity_, max_angular_velocity_);

    return cmd;
}

bool PurePursuitController::isGoalReached(const core::Pose2D& current_pose) const
{
    if (path_.empty()) return true;

    const auto& goal = path_.points.back();
    double distance = current_pose.distanceTo(goal.pose);
    
    return distance < goal_tolerance_;
}

size_t PurePursuitController::getCurrentWaypointIndex() const
{
    return current_waypoint_idx_;
}

double PurePursuitController::getProgress() const
{
    if (path_.empty()) return 1.0;
    return static_cast<double>(current_waypoint_idx_) / static_cast<double>(path_.size());
}

void PurePursuitController::reset()
{
    path_.points.clear();
    current_waypoint_idx_ = 0;
    goal_reached_ = false;
}

void PurePursuitController::setVelocityLimit(double linear_x, double angular_z)
{
    max_linear_velocity_ = linear_x;
    max_angular_velocity_ = angular_z;
}

std::string PurePursuitController::getName() const
{
    return controller_name_;
}

std::string PurePursuitController::getVersion() const
{
    return "1.0.0";
}

void PurePursuitController::setObstacles(const core::ObstacleArray& obstacles)
{
    // Pure Pursuit 不直接使用障碍物信息
    // 可以用于可选的避障调整
    (void)obstacles;
}

// ========== 内部方法实现 ==========

size_t PurePursuitController::findLookaheadPoint(const core::Pose2D& current_pose) const
{
    if (path_.empty()) return 0;

    // 动态调整前视距离
    double dynamic_lookahead = calculateLookaheadDistance(0.3);  // 默认速度
    dynamic_lookahead = std::min(dynamic_lookahead, config_.max_lookahead_distance);
    dynamic_lookahead = std::max(dynamic_lookahead, lookahead_distance_);

    // 从当前路径点开始搜索
    size_t start_index = current_waypoint_idx_;

    for (size_t i = start_index; i < path_.size(); ++i) {
        double distance = current_pose.distanceTo(path_.points[i].pose);

        if (distance >= dynamic_lookahead) {
            return i;
        }
    }

    // 如果没有找到，返回最后一个点（目标）
    return path_.size() - 1;
}

double PurePursuitController::calculateLookaheadDistance(double current_speed) const
{
    // 基于速度动态调整前视距离
    double dynamic_lookahead = config_.lookahead_gain * current_speed + 
                               config_.min_lookahead_distance;
    return std::clamp(dynamic_lookahead, 
                      config_.min_lookahead_distance, 
                      config_.max_lookahead_distance);
}

core::VelocityCommand PurePursuitController::purePursuitAlgorithm(
    const core::Pose2D& current_pose,
    const core::Pose2D& lookahead_point) const
{
    double dx = lookahead_point.x - current_pose.x;
    double dy = lookahead_point.y - current_pose.y;

    // 转换到机器人坐标系
    double local_x = dx * std::cos(current_pose.theta) + 
                     dy * std::sin(current_pose.theta);
    double local_y = -dx * std::sin(current_pose.theta) + 
                     dy * std::cos(current_pose.theta);

    // 计算到前视点的距离
    double L = std::sqrt(local_x * local_x + local_y * local_y);
    
    if (L < 0.01) {
        return core::VelocityCommand(0, 0, 0);
    }

    // 计算曲率
    double curvature = 2.0 * local_y / (L * L);

    // 根据曲率调整线速度
    double linear_vel = max_linear_velocity_;
    if (std::abs(curvature) > 0.1) {
        linear_vel *= (1.0 / (1.0 + 5.0 * std::abs(curvature)));
    }
    linear_vel = std::max(linear_vel, 0.05);

    // 计算角速度
    double angular_vel = curvature * linear_vel;

    // 限制角速度
    angular_vel = std::clamp(angular_vel, -max_angular_velocity_, max_angular_velocity_);

    return core::VelocityCommand(linear_vel, 0.0, angular_vel);
}

double PurePursuitController::normalizeAngle(double angle) const
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace controllers
}  // namespace rosiwit_navigation