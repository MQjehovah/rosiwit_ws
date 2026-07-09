// ============================================================
// Diffbot Navigation - 障碍物检测器实现
// ============================================================

#include "rosiwit_navigation/algorithms/obstacle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

namespace rosiwit_navigation
{
namespace obstacle_avoidance
{

ObstacleDetector::ObstacleDetector()
{
  // 初始化默认配置
  config_.detection_range = 5.0;
  config_.min_obstacle_distance = 0.1;
  config_.safe_distance = 0.3;
  config_.laser_min_range = 0.1;
  config_.laser_max_range = 10.0;
  config_.laser_min_angle = -M_PI;
  config_.laser_max_angle = M_PI;
  config_.point_cloud_min_height = 0.0;
  config_.point_cloud_max_height = 2.0;
  config_.cluster_tolerance = 0.2;
  config_.min_cluster_size = 3;
  config_.max_cluster_size = 100;
  config_.enable_dynamic_detection = true;
  config_.dynamic_velocity_threshold = 0.2;
  config_.enable_filter = true;
  config_.filter_window_size = 5;
}

void ObstacleDetector::configure(const DetectorConfig & config)
{
  config_ = config;
  RCLCPP_INFO(logger_, "Obstacle detector configured with range %.2f m", config_.detection_range);
}

void ObstacleDetector::updateLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  latest_laser_scan_ = scan;

  // 从激光扫描提取障碍物
  std::vector<Obstacle> new_obstacles = extractObstaclesFromLaserScan(scan);

  // 更新障碍物列表
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  obstacles_ = new_obstacles;

  // 检测动态障碍物
  if (config_.enable_dynamic_detection) {
    detectDynamicObstacles();
  }

  // 应用滤波器
  if (config_.enable_filter) {
    applyFilter();
  }

  RCLCPP_DEBUG(logger_, "Updated obstacles from laser scan: %zu obstacles", obstacles_.size());
}

void ObstacleDetector::updatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  latest_point_cloud_ = cloud;

  // 从点云提取障碍物（可选实现）
  // std::vector<Obstacle> new_obstacles = extractObstaclesFromPointCloud(cloud);
}

std::vector<Obstacle> ObstacleDetector::getObstacles() const
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  return obstacles_;
}

std::vector<Obstacle> ObstacleDetector::getFrontObstacles(double angle_range) const
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);

  std::vector<Obstacle> front_obstacles;

  for (const auto & obs : obstacles_) {
    // 检查是否在前方范围内
    if (std::abs(obs.angle) < angle_range) {
      front_obstacles.push_back(obs);
    }
  }

  return front_obstacles;
}

Obstacle ObstacleDetector::getClosestObstacle() const
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);

  if (obstacles_.empty()) {
    return Obstacle();  // 返回空障碍物
  }

  Obstacle closest = obstacles_[0];
  for (const auto & obs : obstacles_) {
    if (obs.distance < closest.distance) {
      closest = obs;
    }
  }

  return closest;
}

std::vector<Obstacle> ObstacleDetector::getObstaclesWithinDistance(double distance) const
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);

  std::vector<Obstacle> nearby_obstacles;

  for (const auto & obs : obstacles_) {
    if (obs.distance < distance) {
      nearby_obstacles.push_back(obs);
    }
  }

  return nearby_obstacles;
}

bool ObstacleDetector::checkPathCollision(
  const std::vector<geometry_msgs::msg::Point> & path) const
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);

  for (const auto & path_point : path) {
    for (const auto & obs : obstacles_) {
      double dx = path_point.x - obs.x;
      double dy = path_point.y - obs.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      // 检查是否碰撞
      if (distance < obs.radius + config_.safe_distance) {
        return true;  // 路径碰撞
      }
    }
  }

  return false;  // 无碰撞
}

bool ObstacleDetector::isPositionSafe(double x, double y) const
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);

  for (const auto & obs : obstacles_) {
    double dx = x - obs.x;
    double dy = y - obs.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < obs.radius + config_.safe_distance) {
      return false;  // 不安全
    }
  }

  return true;  // 安全
}

void ObstacleDetector::clearObstacles()
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  obstacles_.clear();
}

bool ObstacleDetector::hasObstacles() const
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  return !obstacles_.empty();
}

std::vector<Obstacle> ObstacleDetector::extractObstaclesFromLaserScan(
  const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  std::vector<Obstacle> obstacles;

  // 简化的障碍物提取方法：从激光扫描中找到间隙并识别障碍物
  double angle = scan->angle_min;
  double angle_increment = scan->angle_increment;

  // 跳跃检测：当距离突然增大时表示障碍物边缘
  double prev_range = 0.0;
  int obstacle_start_idx = -1;

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];

    // 检查范围有效性
    if (range < scan->range_min || range > scan->range_max) {
      continue;
    }

    // 检查是否在检测范围内
    if (range > config_.detection_range) {
      continue;
    }

    // 检测障碍物跳跃（距离变化超过阈值）
    double range_diff = std::abs(range - prev_range);
    if (range_diff > config_.cluster_tolerance) {
      if (obstacle_start_idx >= 0) {
        // 完成一个障碍物区域，创建障碍物
        Obstacle obs;
        int mid_idx = (obstacle_start_idx + static_cast<int>(i) - 1) / 2;
        double mid_angle = scan->angle_min + mid_idx * angle_increment;
        double mid_range = scan->ranges[mid_idx];

        obs.x = mid_range * std::cos(mid_angle);
        obs.y = mid_range * std::sin(mid_angle);
        obs.z = 0.0;
        obs.distance = mid_range;
        obs.angle = mid_angle;
        obs.radius = config_.cluster_tolerance / 2.0;
        obs.is_dynamic = false;
        obs.confidence = 0.8;

        obstacles.push_back(obs);
      }
      obstacle_start_idx = static_cast<int>(i);
    }

    prev_range = range;
    angle += angle_increment;
  }

  // 处理最后一个障碍物区域
  if (obstacle_start_idx >= 0 && obstacle_start_idx < static_cast<int>(scan->ranges.size())) {
    Obstacle obs;
    int end_idx = static_cast<int>(scan->ranges.size()) - 1;
    int mid_idx = (obstacle_start_idx + end_idx) / 2;
    double mid_angle = scan->angle_min + mid_idx * angle_increment;
    double mid_range = scan->ranges[mid_idx];

    obs.x = mid_range * std::cos(mid_angle);
    obs.y = mid_range * std::sin(mid_angle);
    obs.z = 0.0;
    obs.distance = mid_range;
    obs.angle = mid_angle;
    obs.radius = config_.cluster_tolerance / 2.0;
    obs.is_dynamic = false;
    obs.confidence = 0.8;

    obstacles.push_back(obs);
  }

  return obstacles;
}

std::vector<Obstacle> ObstacleDetector::extractObstaclesFromPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  // 点云处理实现（可选）
  std::vector<Obstacle> obstacles;

  // TODO: 实现点云障碍物提取

  return obstacles;
}

std::vector<std::vector<geometry_msgs::msg::Point>> ObstacleDetector::clusterPoints(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<std::vector<geometry_msgs::msg::Point>> clusters;

  if (points.empty()) {
    return clusters;
  }

  // 简化的聚类方法
  std::vector<bool> visited(points.size(), false);

  for (size_t i = 0; i < points.size(); ++i) {
    if (visited[i]) {
      continue;
    }

    std::vector<geometry_msgs::msg::Point> cluster;
    cluster.push_back(points[i]);
    visited[i] = true;

    // 找到相邻点
    for (size_t j = i + 1; j < points.size(); ++j) {
      if (!visited[j]) {
        double dx = points[j].x - cluster.back().x;
        double dy = points[j].y - cluster.back().y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < config_.cluster_tolerance) {
          cluster.push_back(points[j]);
          visited[j] = true;
        }
      }
    }

    if (static_cast<int>(cluster.size()) >= config_.min_cluster_size &&
      static_cast<int>(cluster.size()) <= config_.max_cluster_size) {
      clusters.push_back(cluster);
    }
  }

  return clusters;
}

double ObstacleDetector::computeObstacleRadius(
  const std::vector<geometry_msgs::msg::Point> & cluster)
{
  if (cluster.empty()) {
    return 0.0;
  }

  // 计算聚类中心
  double center_x = 0.0, center_y = 0.0;
  for (const auto & point : cluster) {
    center_x += point.x;
    center_y += point.y;
  }
  center_x /= cluster.size();
  center_y /= cluster.size();

  // 计算到中心的最大距离
  double max_dist = 0.0;
  for (const auto & point : cluster) {
    double dx = point.x - center_x;
    double dy = point.y - center_y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > max_dist) {
      max_dist = dist;
    }
  }

  return max_dist;
}

void ObstacleDetector::detectDynamicObstacles()
{
  if (previous_obstacles_.empty()) {
    previous_obstacles_ = obstacles_;
    previous_time_ = rclcpp::Clock().now();
    return;
  }

  auto current_time = rclcpp::Clock().now();
  double dt = (current_time - previous_time_).seconds();

  if (dt < 0.001) {
    return;
  }

  // 计算障碍物速度
  for (auto & current_obs : obstacles_) {
    // 找到最接近的先前障碍物
    double min_dist = std::numeric_limits<double>::max();
    Obstacle * closest_prev = nullptr;

    for (auto & prev_obs : previous_obstacles_) {
      double dx = current_obs.x - prev_obs.x;
      double dy = current_obs.y - prev_obs.y;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist < min_dist) {
        min_dist = dist;
        closest_prev = &prev_obs;
      }
    }

    // 如果找到匹配的障碍物，计算速度
    if (closest_prev && min_dist < config_.cluster_tolerance * 2.0) {
      current_obs.velocity_x = (current_obs.x - closest_prev->x) / dt;
      current_obs.velocity_y = (current_obs.y - closest_prev->y) / dt;

      double velocity_magnitude = std::sqrt(
        current_obs.velocity_x * current_obs.velocity_x +
        current_obs.velocity_y * current_obs.velocity_y);

      // 标记动态障碍物
      if (velocity_magnitude > config_.dynamic_velocity_threshold) {
        current_obs.is_dynamic = true;
        RCLCPP_DEBUG(logger_, "Dynamic obstacle detected at (%.2f, %.2f) with speed %.2f m/s",
          current_obs.x, current_obs.y, velocity_magnitude);
      }
    }
  }

  // 更新历史数据
  previous_obstacles_ = obstacles_;
  previous_time_ = current_time;
}

void ObstacleDetector::applyFilter()
{
  // 简化的滤波器实现
  // 使用距离阈值过滤噪声
  std::vector<Obstacle> filtered;

  for (const auto & obs : obstacles_) {
    if (obs.distance >= config_.min_obstacle_distance &&
      obs.confidence > 0.5) {
      filtered.push_back(obs);
    }
  }

  obstacles_ = filtered;
}

void ObstacleDetector::updateObstacleDistances(const geometry_msgs::msg::Pose & robot_pose)
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);

  for (auto & obs : obstacles_) {
    double dx = obs.x - robot_pose.position.x;
    double dy = obs.y - robot_pose.position.y;
    obs.distance = std::sqrt(dx * dx + dy * dy);
    obs.angle = std::atan2(dy, dx);
  }
}

} // namespace obstacle_avoidance
} // namespace rosiwit_navigation