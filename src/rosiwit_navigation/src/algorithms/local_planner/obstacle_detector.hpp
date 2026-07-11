// ============================================================
// Diffbot Navigation - 障碍物检测器
// 基于激光雷达和点云的障碍物检测
// ============================================================

#ifndef ROSIWIT_NAVIGATION__OBSTACLE_AVOIDANCE__OBSTACLE_DETECTOR_HPP_
#define ROSIWIT_NAVIGATION__OBSTACLE_AVOIDANCE__OBSTACLE_DETECTOR_HPP_

#include <memory>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

namespace rosiwit_navigation
{
namespace obstacle_avoidance
{

/**
 * @brief 障碍物信息
 */
struct Obstacle
{
  double x;              // 障碍物x坐标 (m)
  double y;              // 障碍物y坐标 (m)
  double z;              // 障碍物z坐标 (m)
  double distance;       // 到机器人的距离 (m)
  double angle;          // 相对机器人的角度 (rad)
  double radius;         // 障碍物半径估计 (m)
  bool is_dynamic;       // 是否为动态障碍物
  double velocity_x;     // 障碍物速度x (m/s)
  double velocity_y;     // 障碍物速度y (m/s)
  double confidence;     // 检测置信度 [0, 1]

  Obstacle()
  : x(0.0), y(0.0), z(0.0), distance(0.0), angle(0.0),
    radius(0.2), is_dynamic(false), velocity_x(0.0), velocity_y(0.0),
    confidence(0.0) {}
};

/**
 * @brief 检测参数
 */
struct DetectorConfig
{
  // 检测范围
  double detection_range;          // 检测范围 (m)
  double min_obstacle_distance;    // 最小障碍物距离 (m)
  double safe_distance;            // 安全距离 (m)

  // 激光雷达参数
  double laser_min_range;
  double laser_max_range;
  double laser_min_angle;
  double laser_max_angle;

  // 点云参数
  double point_cloud_min_height;
  double point_cloud_max_height;

  // 障碍物聚类参数
  double cluster_tolerance;        // 聚类容差 (m)
  int min_cluster_size;            // 最小聚类点数
  int max_cluster_size;            // 最大聚类点数

  // 动态障碍物检测
  bool enable_dynamic_detection;
  double dynamic_velocity_threshold;  // 动态障碍物速度阈值

  // 滤波参数
  bool enable_filter;
  int filter_window_size;
};

/**
 * @class ObstacleDetector
 * @brief 障碍物检测器
 */
class ObstacleDetector
{
public:
  /**
   * @brief 构造函数
   */
  ObstacleDetector();

  /**
   * @brief 析构函数
   */
  ~ObstacleDetector() = default;

  /**
   * @brief 配置检测器
   */
  void configure(const DetectorConfig & config);

  /**
   * @brief 更新激光扫描数据
   */
  void updateLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief 更新点云数据
   */
  void updatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

  /**
   * @brief 获取所有障碍物
   * @return 障碍物列表
   */
  std::vector<Obstacle> getObstacles() const;

  /**
   * @brief 获取前方障碍物
   * @param angle_range 角度范围 (rad)
   * @return 前方障碍物列表
   */
  std::vector<Obstacle> getFrontObstacles(double angle_range = M_PI / 3) const;

  /**
   * @brief 获取最近障碍物
   * @return 最近的障碍物
   */
  Obstacle getClosestObstacle() const;

  /**
   * @brief 获取指定距离内的障碍物
   * @param distance 距离阈值
   * @return 障碍物列表
   */
  std::vector<Obstacle> getObstaclesWithinDistance(double distance) const;

  /**
   * @brief 检查路径是否碰撞
   * @param path 待检查路径
   * @return 是否碰撞
   */
  bool checkPathCollision(const std::vector<geometry_msgs::msg::Point> & path) const;

  /**
   * @brief 检查位置是否安全
   * @param x x坐标
   * @param y y坐标
   * @return 是否安全
   */
  bool isPositionSafe(double x, double y) const;

  /**
   * @brief 清除障碍物数据
   */
  void clearObstacles();

  /**
   * @brief 是否有障碍物
   */
  bool hasObstacles() const;

  /**
   * @brief 获取障碍物数量
   */
  size_t getObstacleCount() const { return obstacles_.size(); }

private:
  /**
   * @brief 从激光扫描提取障碍物
   */
  std::vector<Obstacle> extractObstaclesFromLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief 从点云提取障碍物
   */
  std::vector<Obstacle> extractObstaclesFromPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

  /**
   * @brief 聚类障碍物点
   */
  std::vector<std::vector<geometry_msgs::msg::Point>> clusterPoints(
    const std::vector<geometry_msgs::msg::Point> & points);

  /**
   * @brief 计算障碍物半径
   */
  double computeObstacleRadius(const std::vector<geometry_msgs::msg::Point> & cluster);

  /**
   * @brief 检测动态障碍物
   */
  void detectDynamicObstacles();

  /**
   * @brief 应用滤波器
   */
  void applyFilter();

  /**
   * @brief 更新障碍物距离
   */
  void updateObstacleDistances(const geometry_msgs::msg::Pose & robot_pose);

  // 成员变量
  DetectorConfig config_;
  std::vector<Obstacle> obstacles_;
  mutable std::mutex obstacles_mutex_;

  // 历史数据（用于动态障碍物检测）
  std::vector<Obstacle> previous_obstacles_;
  rclcpp::Time previous_time_;

  // 传感器数据
  sensor_msgs::msg::LaserScan::SharedPtr latest_laser_scan_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_point_cloud_;

  // 日志
  rclcpp::Logger logger_{rclcpp::get_logger("obstacle_detector")};
};

} // namespace obstacle_avoidance
} // namespace rosiwit_navigation

#endif // ROSIWIT_NAVIGATION__OBSTACLE_AVOIDANCE__OBSTACLE_DETECTOR_HPP_