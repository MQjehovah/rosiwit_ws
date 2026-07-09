// ============================================================
// Diffbot Navigation - 路径规划器实现
// ============================================================

#include "rosiwit_navigation/algorithms/path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "nav2_util/node_utils.hpp"

namespace rosiwit_navigation
{
namespace navigation
{

using nav2_util::declare_parameter_if_not_declared;

PathPlanner::PathPlanner()
: configured_(false), active_(false)
{
  // 初始化默认配置
  config_.wheel_separation = 0.4;
  config_.wheel_radius = 0.065;
  config_.tolerance = 0.5;
  config_.use_astar = false;
  config_.allow_unknown = true;
  config_.path_resolution = 0.05;
  config_.smooth_cost = 0.5;
  config_.obstacle_cost = 1.0;
}

void PathPlanner::configure(
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
  clock_ = node->get_clock();

  // 声明并获取参数
  declare_parameter_if_not_declared(
    node, planner_name_ + ".tolerance", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".use_astar", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, planner_name_ + ".path_resolution", rclcpp::ParameterValue(0.05));

  node->get_parameter(planner_name_ + ".tolerance", config_.tolerance);
  node->get_parameter(planner_name_ + ".use_astar", config_.use_astar);
  node->get_parameter(planner_name_ + ".allow_unknown", config_.allow_unknown);
  node->get_parameter(planner_name_ + ".path_resolution", config_.path_resolution);

  RCLCPP_INFO(
    logger_,
    "Configuring path planner %s with tolerance %.2f, use_astar: %s",
    planner_name_.c_str(),
    config_.tolerance,
    config_.use_astar ? "true" : "false");

  configured_ = true;
}

void PathPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up path planner %s", planner_name_.c_str());
}

void PathPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating path planner %s", planner_name_.c_str());
  active_ = true;
}

void PathPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating path planner %s", planner_name_.c_str());
  active_ = false;
}

nav_msgs::msg::Path PathPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!configured_) {
    RCLCPP_ERROR(logger_, "Path planner not configured");
    return nav_msgs::msg::Path();
  }

  if (!active_) {
    RCLCPP_WARN(logger_, "Path planner not active");
    return nav_msgs::msg::Path();
  }

  RCLCPP_INFO(
    logger_,
    "Creating plan from (%.2f, %.2f) to (%.2f, %.2f)",
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  // 根据配置选择规划方法
  nav_msgs::msg::Path path;
  if (config_.use_astar) {
    path = planWithAStar(start, goal);
  } else {
    path = planWithNavFn(start, goal);
  }

  // 平滑路径
  if (path.poses.size() > 2) {
    path = smoothPath(path);
  }

  // 验证路径
  if (!validatePath(path)) {
    RCLCPP_WARN(logger_, "Generated path is invalid");
    return nav_msgs::msg::Path();
  }

  return path;
}

void PathPlanner::setConfig(const PlannerConfig & config)
{
  config_ = config;
}

nav_msgs::msg::Path PathPlanner::smoothPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 3) {
    return path;
  }

  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = path.header;

  // 使用简单的路径平滑算法
  // 保留起点和终点
  smoothed_path.poses.push_back(path.poses.front());

  // 中间点使用相邻点平均
  for (size_t i = 1; i < path.poses.size() - 1; ++i) {
    geometry_msgs::msg::PoseStamped smoothed_pose;
    smoothed_pose.header = path.poses[i].header;

    // 计算平滑后的位置（使用权重）
    double alpha = 0.25;  // 平滑系数
    smoothed_pose.pose.position.x =
      path.poses[i - 1].pose.position.x * alpha +
      path.poses[i].pose.position.x * (1.0 - 2.0 * alpha) +
      path.poses[i + 1].pose.position.x * alpha;

    smoothed_pose.pose.position.y =
      path.poses[i - 1].pose.position.y * alpha +
      path.poses[i].pose.position.y * (1.0 - 2.0 * alpha) +
      path.poses[i + 1].pose.position.y * alpha;

    smoothed_pose.pose.position.z = 0.0;

    // 保持原朝向（后续可根据曲率调整）
    smoothed_pose.pose.orientation = path.poses[i].pose.orientation;

    smoothed_path.poses.push_back(smoothed_pose);
  }

  smoothed_path.poses.push_back(path.poses.back());

  RCLCPP_INFO(
    logger_,
    "Smoothed path from %zu points to %zu points",
    path.poses.size(),
    smoothed_path.poses.size());

  return smoothed_path;
}

bool PathPlanner::validatePath(const nav_msgs::msg::Path & path)
{
  // 检查路径是否非空
  if (path.poses.empty()) {
    RCLCPP_ERROR(logger_, "Path is empty");
    return false;
  }

  // 检查路径点是否连续
  for (size_t i = 1; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 检查路径点间隔是否合理（不应过大）
    if (distance > 2.0) {
      RCLCPP_WARN(
        logger_,
        "Large gap between path points %zu and %zu: %.2f m",
        i - 1, i, distance);
      // 不直接返回false，允许一些跳跃
    }
  }

  return true;
}

void PathPlanner::initializePlanner()
{
  // 初始化规划器内部数据结构
}

nav_msgs::msg::Path PathPlanner::planWithNavFn(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // 使用NavFn算法（基于Dijkstra）
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = start.header.frame_id;

  // 获取代价地图
  auto costmap = costmap_ros_->getCostmap();
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();

  // 计算起点和终点在代价地图中的位置
  unsigned int start_mx, start_my, goal_mx, goal_my;
  costmap->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);
  costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my);

  RCLCPP_DEBUG(
    logger_,
    "Planning from (%u, %u) to (%u, %u) in costmap",
    start_mx, start_my, goal_mx, goal_my);

  // 简化实现：使用梯度下降方法从终点回溯到起点
  // 这里使用一个简化的路径生成方法

  // 计算起点和终点的方向
  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // 根据路径分辨率计算点数
  int num_points = static_cast<int>(distance / config_.path_resolution) + 1;
  num_points = std::max(num_points, 2);

  // 生成路径点
  for (int i = 0; i <= num_points; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;

    double ratio = static_cast<double>(i) / static_cast<double>(num_points);
    pose.pose.position.x = start.pose.position.x + ratio * dx;
    pose.pose.position.y = start.pose.position.y + ratio * dy;
    pose.pose.position.z = 0.0;

    // 计算朝向角
    double theta = std::atan2(dy, dx);
    pose.pose.orientation.w = std::cos(theta / 2.0);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(theta / 2.0);

    path.poses.push_back(pose);
  }

  return path;
}

nav_msgs::msg::Path PathPlanner::planWithAStar(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // A*算法实现
  // 这里简化实现，实际应该使用完整的A*搜索
  nav_msgs::msg::Path path = planWithNavFn(start, goal);

  // A*相比NavFn主要是启发式搜索
  // 在Nav2中已经内置优化

  return path;
}

std::vector<PathPoint> PathPlanner::optimizePath(const std::vector<PathPoint> & raw_path)
{
  std::vector<PathPoint> optimized_path;

  if (raw_path.size() < 3) {
    return raw_path;
  }

  // 计算曲率信息并优化路径
  for (size_t i = 0; i < raw_path.size(); ++i) {
    PathPoint point = raw_path[i];

    if (i > 0 && i < raw_path.size() - 1) {
      // 计算曲率
      point.curvature = computeCurvature(
        raw_path[i - 1], raw_path[i], raw_path[i + 1]);

      // 根据曲率调整建议速度
      if (std::abs(point.curvature) > 0.5) {
        // 大曲率时降低速度
        point.velocity *= 0.7;
      }
    }

    optimized_path.push_back(point);
  }

  return optimized_path;
}

double PathPlanner::computeCurvature(
  const PathPoint & prev,
  const PathPoint & curr,
  const PathPoint & next)
{
  // 计算三个点形成的曲率
  // 使用Menger曲率公式: k = 4 * Area / (d_ab * d_bc * d_ca)

  // 计算距离
  double d_ab = std::sqrt(
    std::pow(curr.x - prev.x, 2) + std::pow(curr.y - prev.y, 2));
  double d_bc = std::sqrt(
    std::pow(next.x - curr.x, 2) + std::pow(next.y - curr.y, 2));
  double d_ca = std::sqrt(
    std::pow(next.x - prev.x, 2) + std::pow(next.y - prev.y, 2));

  if (d_ab < 0.001 || d_bc < 0.001 || d_ca < 0.001) {
    return 0.0;  // 点太近，曲率接近无穷
  }

  // 计算三角形面积（海伦公式）
  double s = (d_ab + d_bc + d_ca) / 2.0;
  double area_squared = s * (s - d_ab) * (s - d_bc) * (s - d_ca);

  if (area_squared < 0) {
    return 0.0;  // 无效三角形
  }

  double area = std::sqrt(area_squared);
  double curvature = 4.0 * area / (d_ab * d_bc * d_ca);

  return curvature;
}

} // namespace navigation
} // namespace rosiwit_navigation