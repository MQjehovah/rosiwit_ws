// ============================================================
// Diffbot Navigation - 平滑导航接口
// 提供平滑的单点导航功能
// ============================================================

#ifndef DIFFBOT_NAVIGATION__NAVIGATION__SMOOTH_NAVIGATION_HPP_
#define DIFFBOT_NAVIGATION__NAVIGATION__SMOOTH_NAVIGATION_HPP_

#include <memory>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "diffbot_navigation/navigation/path_planner.hpp"
#include "diffbot_navigation/navigation/trajectory_generator.hpp"

namespace diffbot_navigation
{
namespace navigation
{

/**
 * @brief 导航状态枚举
 */
enum class NavigationState
{
  IDLE,
  PLANNING,
  CONTROLLING,
  OBSTACLE_AVOIDANCE,
  NARROW_PASSAGE,
  GOAL_REACHED,
  FAILED
};

/**
 * @brief 导航参数
 */
struct NavigationParams
{
  // 控制频率
  double controller_frequency;
  double planner_frequency;

  // 速度限制
  double max_velocity_x;
  double max_velocity_theta;
  double min_velocity_x;

  // 加速度限制
  double max_accel_x;
  double max_accel_theta;

  // 目标容差
  double goal_tolerance_xy;
  double goal_tolerance_yaw;

  // 轨迹跟踪参数
  double lookahead_distance;
  double min_lookahead_distance;
  double max_lookahead_distance;

  // 重规划参数
  double replanning_distance;
  double replanning_time;
};

/**
 * @class SmoothNavigation
 * @brief 平滑导航主控制器，协调路径规划和轨迹跟踪
 */
class SmoothNavigation : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief 构造函数
   */
  explicit SmoothNavigation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief 析构函数
   */
  ~SmoothNavigation() override = default;

  /**
   * @brief 配置节点
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief 激活节点
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief 停用节点
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief 清理节点
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief 关闭节点
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief 获取当前导航状态
   */
  NavigationState getState() const { return current_state_; }

  /**
   * @brief 是否正在导航
   */
  bool isNavigating() const { return is_navigating_; }

  /**
   * @brief 停止当前导航
   */
  void stopNavigation();

  /**
   * @brief 暂停当前导航
   */
  void pauseNavigation();

  /**
   * @brief 恢复导航
   */
  void resumeNavigation();

protected:
  /**
   * @brief 初始化订阅者和发布者
   */
  void initializeSubscribers();
  void initializePublishers();

  /**
   * @brief 初始化Action服务器
   */
  void initializeActionServer();

  /**
   * @brief 初始化定时器
   */
  void initializeTimers();

  /**
   * @brief 加载参数
   */
  void loadParameters();

  /**
   * @brief 里程计回调
   */
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief 地图回调
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief 控制循环
   */
  void controlLoop();

  /**
   * @brief 重规划检查
   */
  void replanningCheck();

  /**
   * @brief Action服务器回调
   */
  void navigateToPoseCallback();

  /**
   * @brief 执行导航
   */
  void executeNavigation(const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief 更新当前位姿
   */
  bool updateCurrentPose();

  /**
   * @brief 检查是否到达目标
   */
  bool isGoalReached();

  /**
   * @brief 计算速度命令
   */
  geometry_msgs::msg::Twist computeVelocityCommand();

  /**
   * @brief 发布速度命令
   */
  void publishVelocityCommand(const geometry_msgs::msg::Twist & cmd_vel);

  /**
   * @brief 发布路径（用于可视化）
   */
  void publishPath(const nav_msgs::msg::Path & path);

private:
  // 组件
  std::shared_ptr<PathPlanner> path_planner_;
  std::shared_ptr<TrajectoryGenerator> trajectory_generator_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Action服务器
  std::unique_ptr<nav2_util::SimpleActionServer<nav2_msgs::action::NavigateToPose>>
    action_server_;

  // 订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  // 发布者
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr replanning_timer_;

  // 状态变量
  std::atomic<NavigationState> current_state_;
  std::atomic<bool> is_navigating_;
  std::atomic<bool> is_paused_;

  // 当前位姿和速度
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Twist current_velocity_;

  // 目标位姿
  geometry_msgs::msg::PoseStamped goal_pose_;

  // 当前路径和轨迹
  nav_msgs::msg::Path current_path_;
  std::vector<TrajectoryPoint> current_trajectory_;

  // 地图
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;

  // 参数
  NavigationParams params_;

  // 日志
  rclcpp::Logger logger_{rclcpp::get_logger("smooth_navigation")};
};

} // namespace navigation
} // namespace diffbot_navigation

#endif // DIFFBOT_NAVIGATION__NAVIGATION__SMOOTH_NAVIGATION_HPP_