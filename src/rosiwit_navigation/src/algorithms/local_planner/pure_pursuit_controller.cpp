#include "rosiwit_navigation/nav_core/logger.hpp"
// ============================================================
// Diffbot Navigation - Pure Pursuit 控制器实现
// ============================================================

#include "pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rosiwit_navigation
{
namespace controllers
{

PurePursuitController::PurePursuitController()
: controller_name_("PurePursuitController"),
  logger_(core::Logger("PurePursuitController")),
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
    // 用 max_linear_velocity (控制器自己的速度上限), 不要用 kinematics 里的
    // 底盘极限: 覆盖模式给的是 coverage_max_speed=0.45, 而 kinematics 是
    // 普通导航的 0.8 -> 以前覆盖实际按 0.8 跑, 行末必然冲过头.
    max_linear_velocity_ = config.max_linear_velocity > 0.0
        ? config.max_linear_velocity : config.kinematics.max_velocity_x;
    max_angular_velocity_ = config.max_angular_velocity > 0.0
        ? config.max_angular_velocity : config.kinematics.max_velocity_theta;
    initialized_ = true;

    LOG_INFO(logger_, "Pure Pursuit Controller initialized with lookahead=%.2f, tolerance=%.2f, vmax=%.2f",
                lookahead_distance_, goal_tolerance_, max_linear_velocity_);
    return true;
}

void PurePursuitController::setPath(const core::Path& path)
{
    path_ = path;
    current_waypoint_idx_ = 0;
    goal_reached_ = false;
    
    LOG_INFO(logger_, "Path set with %zu waypoints", path_.size());
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

size_t PurePursuitController::findLookaheadPoint(const core::Pose2D& current_pose)
{
    if (path_.empty()) return 0;

    // 先找最近路径点，计算局部路径曲率
    size_t nearest_idx = 0;
    double nearest_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_.size(); ++i) {
        double d = current_pose.distanceTo(path_.points[i].pose);
        if (d < nearest_dist) {
            nearest_dist = d;
            nearest_idx = i;
        }
    }
    // 索引只前进不后退: 机器人偏离路径时 "最近点" 可能跳到身后, 允许回退
    // 会让它在原地来回绕圈 (覆盖行段的轨迹就是一团乱线). 用历史进度当下限.
    if (nearest_idx < current_waypoint_idx_) { nearest_idx = current_waypoint_idx_; }

    // 评估前方路径段的曲率: 取最近点前 3 个点的转角
    double path_curvature = 0.0;
    if (nearest_idx + 3 < path_.size()) {
        const auto& p0 = path_.points[nearest_idx].pose;
        const auto& p1 = path_.points[nearest_idx + 1].pose;
        const auto& p2 = path_.points[nearest_idx + 3].pose;
        double dx1 = p1.x - p0.x, dy1 = p1.y - p0.y;
        double dx2 = p2.x - p1.x, dy2 = p2.y - p1.y;
        double angle_diff = std::abs(std::atan2(dy2, dx2) - std::atan2(dy1, dx1));
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
        path_curvature = angle_diff;  // rad/segment
    }

    // 动态前视: 曲率越大前视越短 (0.3~1.2m)
    double base_lookahead = config_.max_lookahead_distance;
    double curvature_factor = 1.0 / (1.0 + 4.0 * path_curvature);
    double dynamic_lookahead = base_lookahead * curvature_factor;
    dynamic_lookahead = std::clamp(dynamic_lookahead,
        config_.min_lookahead_distance, config_.max_lookahead_distance);

    // 从最近路径点开始搜索
    size_t start_index = nearest_idx;

    for (size_t i = start_index; i < path_.size(); ++i) {
        double distance = current_pose.distanceTo(path_.points[i].pose);
        if (distance >= dynamic_lookahead) {
            current_waypoint_idx_ = std::max(current_waypoint_idx_, i);
            return i;
        }
    }

    // 如果没有找到，返回最后一个点（目标）
    current_waypoint_idx_ = path_.size() - 1;
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

    // 根据曲率调整线速度 (系数 2.0, 原始 FAST-LIO2 用的 5.0 过于激进)
    double linear_vel = max_linear_velocity_;
    if (std::abs(curvature) > 0.1) {
        linear_vel *= (1.0 / (1.0 + 2.0 * std::abs(curvature)));
    }
    // 按航向误差减速: 大误差时先原地转, 不要"边转边蹭".
    // 以前这里有一句 `linear_vel = max(linear_vel, 0.10)` —— 航向差再大也
    // 保持 0.1m/s 前进, 于是行末/转角处一直往前蹭并绕圈 (覆盖轨迹成一团).
    const double heading_err = std::fabs(std::atan2(local_y, local_x));
    if (heading_err > 0.9) {
        linear_vel = 0.0;
    } else {
        linear_vel *= std::max(0.15, std::cos(heading_err));
    }

    // 计算角速度. 注意: w 不能直接用 linear_vel 乘 —— 上面"航向差大就
    // 停车原地转"会把 linear_vel 置 0, 那样 w 也变 0, 机器人就永远转不动.
    // 原地转时用 max_linear_velocity_ 的一个保守基值.
    const double w_base = (linear_vel > 1e-3)
        ? linear_vel : std::min(0.25, max_linear_velocity_);
    double angular_vel = curvature * w_base;

    // 限制角速度
    angular_vel = std::clamp(angular_vel, -max_angular_velocity_, max_angular_velocity_);

    // 原地转时给一个最小转向速度, 避免曲率很小时"想转但转不动"
    if (linear_vel < 1e-3 && std::fabs(angular_vel) > 1e-3) {
        const double sign = angular_vel > 0.0 ? 1.0 : -1.0;
        angular_vel = sign * std::max(0.15, std::fabs(angular_vel));
    }

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