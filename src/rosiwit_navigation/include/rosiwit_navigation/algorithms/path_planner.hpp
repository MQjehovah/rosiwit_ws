// ============================================================
// Diffbot Navigation - 路径规划器
// 基于Smac Hybrid A*算法的全局路径规划
// ============================================================

#ifndef ROSIWIT_NAVIGATION__NAVIGATION__PATH_PLANNER_HPP_
#define ROSIWIT_NAVIGATION__NAVIGATION__PATH_PLANNER_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace rosiwit_navigation
{
namespace navigation
{

/**
 * @brief 路径规划器配置参数
 */
struct PlannerConfig
{
  // 运动学参数
  double wheel_separation;        // 轮间距 (m)
  double wheel_radius;           // 轮半径 (m)
  
  // 规划参数
  double tolerance;              // 目标容差 (m)
  bool use_astar;                // 是否使用A*
  bool allow_unknown;            // 是否允许未知区域
  
  // 路径优化参数
  double path_resolution;        // 路径分辨率 (m)
  double smooth_cost;            // 平滑代价权重
  double obstacle_cost;          // 障碍物代价权重
};

/**
 * @brief 路径点结构
 */
struct PathPoint
{
  double x;
  double y;
  double theta;          // 朝向角
  double velocity;       // 建议速度
  double curvature;     // 曲率 (用于平滑转向)
};

/**
 * @class PathPlanner
 * @brief 全局路径规划器，支持差速轮运动学约束
 */
class PathPlanner : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief 构造函数
   */
  PathPlanner();

  /**
   * @brief 析构函数
   */
  ~PathPlanner() override = default;

  /**
   * @brief 配置规划器
   * @param parent 生命周期节点
   * @param planner_name 规划器名称
   * @param costmap_ros 代价地图
   * @param frame_id 地图坐标系
   */
  void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg);
    void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 清理资源
   */
  void cleanup() override;

  /**
   * @brief 激活规划器
   */
  void activate() override;

  /**
   * @brief 停用规划器
   */
  void deactivate() override;

  /**
   * @brief 规划从起点到终点的路径
   * @param start 起点
   * @param goal 终点
   * @return 规划的路径
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  /**
   * @brief 设置规划参数
   * @param config 参数配置
   */
  void setConfig(const PlannerConfig & config);

  /**
   * @brief 获取平滑后的路径
   * @param path 原始路径
   * @return 平滑后的路径
   */
  nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path & path);

  /**
   * @brief 检查路径是否有效
   * @param path 待检查路径
   * @return 是否有效
   */
  bool validatePath(const nav_msgs::msg::Path & path);

private:
  /**
   * @brief 初始化规划器
   */
  void initializePlanner();

  /**
   * @brief 使用NavFn规划路径
   */
  nav_msgs::msg::Path planWithNavFn(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief 使用A*规划路径
   */
  nav_msgs::msg::Path planWithAStar(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief 优化路径点，添加曲率信息
   */
  std::vector<PathPoint> optimizePath(const std::vector<PathPoint> & raw_path);

  /**
   * @brief 计算路径点的曲率
   */
  double computeCurvature(
    const PathPoint & prev,
    const PathPoint & curr,
    const PathPoint & next);

  // 成员变量
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string planner_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  PlannerConfig config_;
  bool configured_;
  std::vector<int8_t> map_data_;
  int map_width_ = 0;
  int map_height_ = 0;
  double map_resolution_ = 0.05;
  double map_origin_x_ = 0.0;
  double map_origin_y_ = 0.0;
  bool active_;

  // 日志记录器
  rclcpp::Logger logger_{rclcpp::get_logger("path_planner")};
  rclcpp::Clock::SharedPtr clock_;
};

} // namespace navigation
} // namespace rosiwit_navigation

#endif // ROSIWIT_NAVIGATION__NAVIGATION__PATH_PLANNER_HPP_