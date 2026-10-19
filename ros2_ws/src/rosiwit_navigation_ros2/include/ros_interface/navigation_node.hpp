// ============================================================
// Diffbot Navigation - 平滑导航接口
// 提供平滑的单点导航功能
// ============================================================

#ifndef ROSIWIT_NAVIGATION__NAVIGATION__NAVIGATION_NODE_HPP_
#define ROSIWIT_NAVIGATION__NAVIGATION__NAVIGATION_NODE_HPP_

#include <memory>
#include <atomic>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "trajectory_generator.hpp"
#include "rosiwit_navigation/nav_core/navigation_factory.hpp"
#include "ros_interface/ros_utils.hpp"
#include "rosiwit_navigation/nav_core/safety_monitor.hpp"

namespace rosiwit_navigation
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
 * @class NavigationNode
 * @brief 平滑导航主控制器，协调路径规划和轨迹跟踪
 */
class NavigationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief 构造函数
   */
  explicit NavigationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief 析构函数
   */
  ~NavigationNode() override = default;

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
  void loadSafetyParameters();

  /**
   * @brief 里程计回调
   */
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief 地图回调
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void updateCostmap();
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void safetyPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief RViz目标点回调
   */
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

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
  void publishSafetyState(const std::string & state);
  void publishDiagnostics();
  void publishGoalStatus(const std::string & status);

  /**
   * @brief 发布路径（用于可视化）
   */
  void publishPath(const nav_msgs::msg::Path & path);
  void smoothCurrentPath(nav_msgs::msg::Path & path);
  bool pathCollides(const nav_msgs::msg::Path & path,
    const nav_msgs::msg::OccupancyGrid & costmap) const;
  bool adjustGoalToFree(core::Pose2D & goal) const;
  void publishLocalCostmap();
  void republishGlobalCostmap();

private:
  // 组件
  std::shared_ptr<TrajectoryGenerator> trajectory_generator_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


  // 订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr safety_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // 发布者
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_status_pub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr replanning_timer_;
  rclcpp::TimerBase::SharedPtr local_costmap_timer_;
  rclcpp::TimerBase::SharedPtr global_costmap_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  // 最近一次全局 costmap, 由高频定时器重复发布
  nav_msgs::msg::OccupancyGrid global_costmap_msg_;
  bool m_has_global_costmap_ = false;
  double local_costmap_size_ = 8.0;
  int last_local_center_cost_ = -1;
  int last_local_lethal_cells_ = 0;
  int last_local_blue_cells_ = 0;
  geometry_msgs::msg::Twist last_published_cmd_;

  // 状态变量
  std::atomic<NavigationState> current_state_;
  std::atomic<bool> is_navigating_;
  std::atomic<bool> is_paused_;

  // 当前位姿和速度
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Twist current_velocity_;

  // 初始旋转：Pure Pursuit 只前进，目标在后方时先原地掉头
  bool m_need_initial_rotate = false;
  double m_initial_rotate_target = 0.0;
  double m_initial_rotate_start_sec_ = 0.0;

  // 目标位姿
  geometry_msgs::msg::PoseStamped goal_pose_;
  double last_controller_path_sec_ = 0.0;
  double last_costmap_replan_sec_ = -1e9;
  double last_control_sec_ = -1.0;
  double tf_stale_timeout_{0.6};
  double safety_self_filter_radius_{0.20};
  double local_clear_radius_{0.20};
  double last_odom_sec_{-1e9};
  bool odom_pose_valid_{false};
  geometry_msgs::msg::Pose odom_pose_;
  bool last_tf_pose_valid_{false};
  double last_tf_x_{0.0};
  double last_tf_y_{0.0};
  double last_tf_yaw_{0.0};

  // DWA 卡死恢复
  bool recovery_enabled_ = true;
  int recovery_trigger_cycles_ = 40;
  double recovery_duration_ = 1.2;
  double recovery_linear_velocity_ = -0.10;
  double recovery_angular_velocity_ = 0.60;
  int stall_cycles_ = 0;
  int motion_stall_cycles_ = 0;
  bool recovery_active_ = false;
  bool recovery_bypass_safety_ = true;
  double recovery_until_sec_ = 0.0;
  bool safety_stop_active_ = false;
  double safety_stop_since_sec_ = 0.0;
  // 规划失败后随地图更新重试 (探索场景地图会持续变化)
  double m_last_replan_attempt_sec = 0.0;

  // 当前路径和轨迹
  nav_msgs::msg::Path current_path_;
  std::vector<TrajectoryPoint> current_trajectory_;

  // 地图
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  std::vector<geometry_msgs::msg::Point> laser_points_;

  // LiDAR safety layer (Nav2 Collision Monitor style)
  core::SafetyMonitor safety_monitor_;
  std::vector<std::pair<double, double>> safety_points_;
  std::vector<std::pair<double, double>> safety_points_map_;
  bool safety_pose_valid_ = false;
  double safety_pose_x_ = 0.0;
  double safety_pose_y_ = 0.0;
  bool safety_enabled_ = true;
  bool safety_have_cloud_ = false;
  double safety_last_cloud_sec_ = 0.0;
  double safety_source_timeout_ = 2.0;
  bool safety_fail_safe_on_timeout_ = false;
  double safety_min_height_ = 0.05;
  double safety_max_height_ = 1.0;
  std::string safety_base_frame_ = "base_footprint";
  std::string safety_pointcloud_topic_ = "/velodyne_points";
  std::string last_safety_state_;

  // 策略型规划器和控制器（由NavigationFactory创建）
  std::unique_ptr<core::IPlannerStrategy> planner_strategy_;
  std::unique_ptr<core::IControllerStrategy> controller_strategy_;

  // 策略名称参数
  std::string planner_name_;
  std::string controller_name_;
  double inflation_radius_ = 0.5;
  double robot_radius_ = 0.21;
  bool enable_path_smoothing_ = false;

  // 坐标系
  std::string global_frame_ = "odom";
  std::string robot_frame_ = "base_link";
  std::string map_frame_ = "map";

  // 参数
  NavigationParams params_;

  // 日志
  rclcpp::Logger logger_{rclcpp::get_logger("rosiwit_navigation")};
};

} // namespace navigation
} // namespace rosiwit_navigation

#endif // ROSIWIT_NAVIGATION__NAVIGATION__NAVIGATION_NODE_HPP_