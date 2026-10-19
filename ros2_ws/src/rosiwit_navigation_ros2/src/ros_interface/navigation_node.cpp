// ============================================================
// Diffbot Navigation - 平滑导航节点实现
// ============================================================

#include "ros_interface/navigation_node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <cmath>
#include <queue>
#include <unordered_set>
#include <sstream>
#include <iomanip>
#include <utility>

#include "path_smoother.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"



namespace rosiwit_navigation
{
namespace navigation
{

using rclcpp_lifecycle::LifecycleNode;
using ros_interface::RosUtils;

namespace {

// 规划器使用的 costmap:
//   0=free, 1..99=蓝色软代价, 100=紫色致命.
// 蓝色不能直接按 1..99 给小代价, 否则贴着蓝区边缘走只多花 1%,
// A* 会为了抄近路切蓝区. 这里把任何蓝色都抬到 >=128, 让 A* 自己
// 用代价选择全 0 路径; 没有全 0 路径时仍可进入蓝色(窄通道).
core::Costmap makePlannerCostmap(const nav_msgs::msg::OccupancyGrid & msg)
{
  core::Costmap cm;
  auto grid = std::make_shared<core::CostmapGrid>();
  grid->info.width = msg.info.width;
  grid->info.height = msg.info.height;
  grid->info.resolution = msg.info.resolution;
  grid->info.origin.position.x = msg.info.origin.position.x;
  grid->info.origin.position.y = msg.info.origin.position.y;
  grid->data = msg.data;
  cm.grid = grid;
  cm.width = msg.info.width;
  cm.height = msg.info.height;
  cm.resolution = msg.info.resolution;
  cm.origin_x = msg.info.origin.position.x;
  cm.origin_y = msg.info.origin.position.y;
  cm.data.assign(static_cast<size_t>(msg.info.width) * msg.info.height, 0);

  for (size_t i = 0; i < cm.data.size(); ++i) {
    const int8_t v = msg.data[i];
    if (v < 0) {
      cm.data[i] = 254;
    } else if (v >= 100) {
      cm.data[i] = 255;
    } else if (v > 0) {
      // 128..253: 任意蓝格都明显比自由格贵, 但低于致命阈值 254.
      cm.data[i] = static_cast<unsigned char>(128 + static_cast<int>(v) * 125 / 99);
    } else {
      cm.data[i] = 0;
    }
  }
  return cm;
}

}  // namespace

NavigationNode::NavigationNode(const rclcpp::NodeOptions & options)
: LifecycleNode("rosiwit_navigation", options),
  current_state_(NavigationState::IDLE),
  is_navigating_(false),
  is_paused_(false)
{
}

LifecycleNode::CallbackReturn NavigationNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring rosiwit_navigation node");

  // 加载参数
  loadParameters();

  // 初始化TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 使用NavigationFactory创建规划器策略
  planner_strategy_ = nav_core::NavigationFactory::createPlanner(planner_name_);
  if (planner_strategy_) {
    core::PlannerConfig planner_cfg;
    planner_cfg.name = planner_name_;
    planner_cfg.resolution = 0.05;
    // 地图 382x229≈8.7 万格; 旧值 50000 会让不可达目标报
    // "iteration limit exceeded", 看起来像临时失败. 覆盖整张地图.
    planner_cfg.max_iterations = planner_max_iterations_;
    planner_cfg.timeout = planner_timeout_;
    planner_strategy_->initialize(planner_cfg);
    // costmap 已包含安全环 + 蓝色梯度; A* 再膨胀一次会与显示图不一致.
    planner_strategy_->setInflationRadius(0.0);
    // 硬安全缓冲已在 updateCostmap 里预先膨胀到 planner_cm;
    // A* 内部只保留 1 格 (0.05m) 缓冲, 避免双重膨胀把门洞堵死.
    planner_strategy_->setRobotRadius(0.0);
    RCLCPP_INFO(get_logger(), "Created planner strategy: %s", planner_name_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Planner '%s' not found, falling back to PathPlanner", planner_name_.c_str());
  }

  // 独立异步重规划实例: 与 planner_strategy_ 分开发布 costmap, 避免
  // 后台 plan() 与 mapCallback 的 setCostmap 并发读写同一份搜索数组.
  async_planner_ = nav_core::NavigationFactory::createPlanner(planner_name_);
  if (async_planner_) {
    core::PlannerConfig async_cfg;
    async_cfg.name = planner_name_;
    async_cfg.resolution = 0.05;
    async_cfg.max_iterations = planner_max_iterations_;
    async_cfg.timeout = planner_timeout_;
    async_planner_->initialize(async_cfg);
    async_planner_->setInflationRadius(0.0);
    async_planner_->setRobotRadius(0.0);
  }

  // 使用NavigationFactory创建控制器策略
  controller_strategy_ = nav_core::NavigationFactory::createController(controller_name_);
  if (controller_strategy_) {
    core::ControllerConfig ctrl_cfg;
    ctrl_cfg.lookahead_distance = params_.lookahead_distance;
    ctrl_cfg.xy_goal_tolerance = params_.goal_tolerance_xy;
    ctrl_cfg.slow_down_distance = 1.5;
    ctrl_cfg.lookahead_gain = 1.0;
    ctrl_cfg.max_lookahead_distance = params_.max_lookahead_distance;
    ctrl_cfg.min_lookahead_distance = params_.min_lookahead_distance;
    ctrl_cfg.max_linear_velocity = params_.max_velocity_x;
    ctrl_cfg.max_angular_velocity = params_.max_velocity_theta;
    ctrl_cfg.max_accel_x = params_.max_accel_x;
    ctrl_cfg.max_accel_theta = params_.max_accel_theta;
    ctrl_cfg.kinematics.max_velocity_x = params_.max_velocity_x;
    ctrl_cfg.kinematics.max_velocity_theta = params_.max_velocity_theta;
    controller_strategy_->initialize(ctrl_cfg);
    RCLCPP_INFO(get_logger(), "Created controller strategy: %s", controller_name_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Controller '%s' not found, using fallback inline controller", controller_name_.c_str());
  }

  // 初始化订阅者和发布者
  initializeSubscribers();
  initializePublishers();

  // 外部停止服务 (探索节点 disable/finish 时调用)
  stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/stop_navigation",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      stopNavigation();
      response->success = true;
      response->message = "navigation stopped";
    });

  RCLCPP_INFO(get_logger(), "rosiwit_navigation node configured successfully");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn NavigationNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating rosiwit_navigation node");

  // 激活发布者
  cmd_vel_pub_->on_activate();
  path_pub_->on_activate();
  global_path_pub_->on_activate();

  // 激活规划器

  // 创建绑定到当前状态的定时器
  initializeTimers();

  // 设置状态为活跃
  current_state_ = NavigationState::IDLE;

  RCLCPP_INFO(get_logger(), "rosiwit_navigation node activated");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn NavigationNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating rosiwit_navigation node");

  // 先停车, 再停用 publisher; 否则 zero cmd 发不出去.
  if (cmd_vel_pub_ && cmd_vel_pub_->is_activated()) {
    stopNavigation();
  }

  // 停用定时器
  if (control_timer_) control_timer_->cancel();
  if (replanning_timer_) replanning_timer_->cancel();
  if (local_costmap_timer_) local_costmap_timer_->cancel();
  if (diagnostics_timer_) diagnostics_timer_->cancel();

  // 停用发布者
  if (cmd_vel_pub_) cmd_vel_pub_->on_deactivate();
  if (path_pub_) path_pub_->on_deactivate();
  if (global_path_pub_) global_path_pub_->on_deactivate();

  current_state_ = NavigationState::IDLE;

  RCLCPP_INFO(get_logger(), "rosiwit_navigation node deactivated");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn NavigationNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up rosiwit_navigation node");

  if (control_timer_) {control_timer_->cancel(); control_timer_.reset();}
  if (replanning_timer_) {replanning_timer_->cancel(); replanning_timer_.reset();}
  if (local_costmap_timer_) {local_costmap_timer_->cancel(); local_costmap_timer_.reset();}
  if (diagnostics_timer_) {diagnostics_timer_->cancel(); diagnostics_timer_.reset();}
  stop_srv_.reset();

  // 清理组件
  waitForAsyncPlan();
  async_planner_.reset();
  planner_strategy_.reset();
  controller_strategy_.reset();

  // 重置订阅者和发布者
  odom_sub_.reset();
  map_sub_.reset();
  scan_sub_.reset();
  safety_cloud_sub_.reset();
  goal_sub_.reset();
  stop_sub_.reset();
  cmd_vel_pub_.reset();
  path_pub_.reset();
  global_path_pub_.reset();
  global_costmap_pub_.reset();
  local_costmap_pub_.reset();
  diagnostics_pub_.reset();
  goal_status_pub_.reset();
  safety_state_pub_.reset();

  tf_buffer_.reset();
  tf_listener_.reset();

  current_map_.reset();
  m_has_global_costmap_ = false;
  m_has_planner_costmap_ = false;
  m_has_effective_goal_ = false;
  is_navigating_ = false;
  is_paused_ = false;
  last_published_cmd_ = geometry_msgs::msg::Twist();
  current_velocity_ = geometry_msgs::msg::Twist();
  last_odom_sec_ = -1e9;
  last_scan_sec_ = -1e9;
  last_tf_pose_valid_ = false;
  safety_have_cloud_ = false;
  safety_pose_valid_ = false;
  motion_ref_valid_ = false;

  RCLCPP_INFO(get_logger(), "rosiwit_navigation node cleaned up");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn NavigationNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down rosiwit_navigation node");
  // 生命周期 shutdown 不一定先走 deactivate; 主动发零速度, 避免底盘保留
  // 上一条 cmd_vel 继续运动.
  if (cmd_vel_pub_ && cmd_vel_pub_->is_activated()) {
    stopNavigation();
  }
  return LifecycleNode::CallbackReturn::SUCCESS;
}

void NavigationNode::loadParameters()
{
  declareIfNeeded("controller_frequency", 20.0);
  declareIfNeeded("planner_frequency", 1.0);
  declareIfNeeded("max_velocity_x", 0.5);
  declareIfNeeded("max_velocity_theta", 1.0);
  declareIfNeeded("min_velocity_x", -0.5);
  declareIfNeeded("max_accel_x", 0.5);
  declareIfNeeded("max_accel_theta", 1.0);
  declareIfNeeded("goal_tolerance_xy", 0.1);
  declareIfNeeded("goal_tolerance_yaw", 0.1);
  declareIfNeeded("enforce_goal_yaw", false);
  declareIfNeeded("lookahead_distance", 0.6);
  declareIfNeeded("min_lookahead_distance", 0.3);
  declareIfNeeded("max_lookahead_distance", 0.9);

  declareIfNeeded("global_frame", std::string("odom"));
  declareIfNeeded("robot_frame", std::string("base_link"));
  declareIfNeeded("map_frame", std::string("map"));

  declareIfNeeded("planner_name", std::string("astar"));
  declareIfNeeded("controller_name", std::string("pure_pursuit"));
  declareIfNeeded("planner_max_iterations", 200000);
  declareIfNeeded("planner_timeout", 2.0);
  declareIfNeeded("inflation_radius", 0.5);
  declareIfNeeded("robot_radius", 0.21);
  declareIfNeeded("local_costmap_size", 8.0);
  declareIfNeeded("tf_stale_timeout", 0.6);
  declareIfNeeded("local_clear_radius", 0.20);
  declareIfNeeded("local_unknown_as_lethal", false);
  declareIfNeeded("enable_path_smoothing", false);
  declareIfNeeded("recovery_enabled", true);
  declareIfNeeded("recovery_trigger_cycles", 40);
  declareIfNeeded("recovery_duration", 1.2);
  declareIfNeeded("recovery_linear_velocity", -0.10);
  declareIfNeeded("recovery_angular_velocity", 0.60);
  declareIfNeeded("recovery_bypass_safety", true);

  get_parameter("controller_frequency", params_.controller_frequency);
  get_parameter("planner_frequency", params_.planner_frequency);
  get_parameter("max_velocity_x", params_.max_velocity_x);
  get_parameter("max_velocity_theta", params_.max_velocity_theta);
  get_parameter("min_velocity_x", params_.min_velocity_x);
  get_parameter("max_accel_x", params_.max_accel_x);
  get_parameter("max_accel_theta", params_.max_accel_theta);
  get_parameter("goal_tolerance_xy", params_.goal_tolerance_xy);
  get_parameter("goal_tolerance_yaw", params_.goal_tolerance_yaw);
  get_parameter("enforce_goal_yaw", enforce_goal_yaw_);
  get_parameter("lookahead_distance", params_.lookahead_distance);
  get_parameter("min_lookahead_distance", params_.min_lookahead_distance);
  get_parameter("max_lookahead_distance", params_.max_lookahead_distance);

  get_parameter("global_frame", global_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("map_frame", map_frame_);
  get_parameter("planner_name", planner_name_);
  get_parameter("controller_name", controller_name_);
  get_parameter("planner_max_iterations", planner_max_iterations_);
  get_parameter("planner_timeout", planner_timeout_);
  get_parameter("inflation_radius", inflation_radius_);
  get_parameter("robot_radius", robot_radius_);
  get_parameter("local_costmap_size", local_costmap_size_);
  get_parameter("tf_stale_timeout", tf_stale_timeout_);
  get_parameter("local_clear_radius", local_clear_radius_);
  get_parameter("local_unknown_as_lethal", local_unknown_as_lethal_);
  get_parameter("enable_path_smoothing", enable_path_smoothing_);
  get_parameter("recovery_enabled", recovery_enabled_);
  get_parameter("recovery_trigger_cycles", recovery_trigger_cycles_);
  get_parameter("recovery_duration", recovery_duration_);
  get_parameter("recovery_linear_velocity", recovery_linear_velocity_);
  get_parameter("recovery_angular_velocity", recovery_angular_velocity_);
  get_parameter("recovery_bypass_safety", recovery_bypass_safety_);
  loadSafetyParameters();

  RCLCPP_INFO(get_logger(), "Parameters loaded:");
  RCLCPP_INFO(get_logger(), "  - controller_frequency: %.1f Hz", params_.controller_frequency);
  RCLCPP_INFO(get_logger(), "  - max_velocity_x: %.2f m/s", params_.max_velocity_x);
  RCLCPP_INFO(get_logger(), "  - goal_tolerance_xy: %.2f m", params_.goal_tolerance_xy);
}

void NavigationNode::loadSafetyParameters()
{
  declareIfNeeded("safety_enabled", true);
  declareIfNeeded("safety_pointcloud_topic", std::string("/velodyne_points"));
  declareIfNeeded("safety_base_frame", std::string("base_footprint"));
  declareIfNeeded("safety_source_timeout", 2.0);
  declareIfNeeded("safety_self_filter_radius", 0.20);
  declareIfNeeded("safety_fail_safe_on_timeout", false);
  declareIfNeeded("safety_min_height", 0.05);
  declareIfNeeded("safety_max_height", 1.0);
  declareIfNeeded("safety_stop_polygon", std::vector<double>{
    -0.23, -0.21, 0.23, -0.21, 0.23, 0.21, -0.23, 0.21});
  declareIfNeeded("safety_stop_enabled", true);
  declareIfNeeded("safety_stop_radius", 0.0);
  declareIfNeeded("safety_stop_max_points", 2);
  declareIfNeeded("safety_slowdown_polygon", std::vector<double>{
    -0.60, -0.50, 0.70, -0.50, 0.70, 0.50, -0.60, 0.50});
  declareIfNeeded("safety_slowdown_enabled", true);
  declareIfNeeded("safety_slowdown_max_points", 2);
  declareIfNeeded("safety_slowdown_ratio", 0.5);
  declareIfNeeded("safety_approach_enabled", true);
  declareIfNeeded("safety_time_before_collision", 2.0);
  declareIfNeeded("safety_simulation_time_step", 0.1);
  declareIfNeeded("safety_footprint", std::vector<double>{
    -0.25, -0.22, 0.25, -0.22, 0.25, 0.22, -0.25, 0.22});

  get_parameter("safety_enabled", safety_enabled_);
  get_parameter("safety_pointcloud_topic", safety_pointcloud_topic_);
  get_parameter("safety_base_frame", safety_base_frame_);
  get_parameter("safety_source_timeout", safety_source_timeout_);
  get_parameter("safety_self_filter_radius", safety_self_filter_radius_);
  get_parameter("safety_fail_safe_on_timeout", safety_fail_safe_on_timeout_);
  get_parameter("safety_min_height", safety_min_height_);
  get_parameter("safety_max_height", safety_max_height_);

  auto to_polygon = [](const std::vector<double> & flat) {
      std::vector<std::pair<double, double>> polygon;
      for (size_t i = 0; i + 1 < flat.size(); i += 2) {
        polygon.emplace_back(flat[i], flat[i + 1]);
      }
      return polygon;
    };

  core::SafetyMonitorConfig cfg;
  cfg.enabled = safety_enabled_;
  const bool stop_enabled =
    this->get_parameter("safety_stop_enabled").as_bool();
  cfg.stop_polygon = stop_enabled ? to_polygon(
    this->get_parameter("safety_stop_polygon").as_double_array()) :
    std::vector<std::pair<double, double>>{};
  cfg.stop_radius = this->get_parameter("safety_stop_radius").as_double();
  cfg.stop_max_points =
    static_cast<int>(this->get_parameter("safety_stop_max_points").as_int());
  const bool slowdown_enabled =
    this->get_parameter("safety_slowdown_enabled").as_bool();
  cfg.slowdown_polygon = slowdown_enabled ? to_polygon(
    this->get_parameter("safety_slowdown_polygon").as_double_array()) :
    std::vector<std::pair<double, double>>{};
  cfg.slowdown_max_points =
    static_cast<int>(this->get_parameter("safety_slowdown_max_points").as_int());
  cfg.slowdown_ratio = this->get_parameter("safety_slowdown_ratio").as_double();
  cfg.approach_enabled = this->get_parameter("safety_approach_enabled").as_bool();
  cfg.time_before_collision =
    this->get_parameter("safety_time_before_collision").as_double();
  cfg.simulation_time_step =
    this->get_parameter("safety_simulation_time_step").as_double();
  cfg.footprint = to_polygon(
    this->get_parameter("safety_footprint").as_double_array());
  safety_monitor_.configure(cfg);

  RCLCPP_INFO(get_logger(),
    "Safety layer: %s cloud=%s frame=%s stop_pts=%d slow_ratio=%.2f approach=%s",
    safety_enabled_ ? "ON" : "OFF", safety_pointcloud_topic_.c_str(),
    safety_base_frame_.c_str(), cfg.stop_max_points, cfg.slowdown_ratio,
    cfg.approach_enabled ? "ON" : "OFF");
}

void NavigationNode::safetyPointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!safety_enabled_ || msg->header.frame_id.empty()) return;

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      safety_base_frame_, msg->header.frame_id, msg->header.stamp,
      tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException &) {
    try {
      tf = tf_buffer_->lookupTransform(
        safety_base_frame_, msg->header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Safety layer: cannot transform %s -> %s: %s",
        msg->header.frame_id.c_str(), safety_base_frame_.c_str(), ex.what());
      return;
    }
  }

  const auto & q_msg = tf.transform.rotation;
  const auto & t_msg = tf.transform.translation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  tf2::Matrix3x3 rotation(q);
  const tf2::Vector3 translation(t_msg.x, t_msg.y, t_msg.z);

  std::vector<std::pair<double, double>> points;
  points.reserve(static_cast<size_t>(msg->width) * msg->height);
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      tf2::Vector3 point(*iter_x, *iter_y, *iter_z);
      point = rotation * point + translation;
      if (point.z() < safety_min_height_ || point.z() > safety_max_height_) {
        continue;
      }
      // 激光会打到机器人自身/很近的地面噪点; 不滤掉会在局部 costmap
      // 中心形成 100, DWA/MPPI 所有轨迹都判碰撞, 原地 recovery.
      if (std::hypot(point.x(), point.y()) < safety_self_filter_radius_) {
        continue;
      }
      points.emplace_back(point.x(), point.y());
      if (points.size() >= 20000) break;
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Safety layer: failed to parse PointCloud2: %s", ex.what());
    return;
  }

  safety_points_ = std::move(points);
  safety_have_cloud_ = true;
  safety_last_cloud_sec_ = now().seconds();

  // 同步投影到 map 系, 供局部 costmap 和 DWA 障碍层使用.
  safety_points_map_.clear();
  try {
    geometry_msgs::msg::TransformStamped tf_map;
    try {
      tf_map = tf_buffer_->lookupTransform(
        map_frame_, safety_base_frame_, msg->header.stamp, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException &) {
      tf_map = tf_buffer_->lookupTransform(
        map_frame_, safety_base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
    }
    const auto & qm = tf_map.transform.rotation;
    const auto & tm = tf_map.transform.translation;
    tf2::Quaternion q_map(qm.x, qm.y, qm.z, qm.w);
    tf2::Matrix3x3 r_map(q_map);
    const tf2::Vector3 t_map(tm.x, tm.y, tm.z);
    safety_pose_x_ = tm.x;
    safety_pose_y_ = tm.y;
    safety_pose_valid_ = true;
    safety_points_map_.reserve(safety_points_.size());
    for (const auto & p : safety_points_) {
      const tf2::Vector3 pm = r_map * tf2::Vector3(p.first, p.second, 0.0) + t_map;
      safety_points_map_.emplace_back(pm.x(), pm.y());
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
      "Safety layer: map projection unavailable: %s", ex.what());
  }

  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
    "Safety cloud: %zu points in %s", safety_points_.size(),
    safety_base_frame_.c_str());
}

void NavigationNode::initializeSubscribers()
{
  declareIfNeeded("odom_topic", std::string("/odom"));
  std::string odom_topic;
  get_parameter("odom_topic", odom_topic);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&NavigationNode::odometryCallback, this, std::placeholders::_1));

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&NavigationNode::mapCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&NavigationNode::scanCallback, this, std::placeholders::_1));

  if (safety_enabled_) {
    safety_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      safety_pointcloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&NavigationNode::safetyPointCloudCallback, this, std::placeholders::_1));
  }

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigationNode::goalCallback, this, std::placeholders::_1));

  stop_sub_ = create_subscription<std_msgs::msg::String>(
    "/nav_stop", rclcpp::QoS(10),
    std::bind(&NavigationNode::stopCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribers initialized");
}

void NavigationNode::initializePublishers()
{
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/local_path", rclcpp::SystemDefaultsQoS());

  global_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/global_path", rclcpp::SystemDefaultsQoS());

  global_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap", rclcpp::QoS(1).transient_local());
  local_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap", rclcpp::QoS(1).transient_local());
  diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
    "/nav_diagnostics", rclcpp::QoS(10));
  goal_status_pub_ = create_publisher<std_msgs::msg::String>(
    "/goal_status", rclcpp::QoS(10));
  safety_state_pub_ = create_publisher<std_msgs::msg::String>(
    "/safety_state", 10);

  RCLCPP_INFO(get_logger(), "Publishers initialized");
}

void NavigationNode::initializeTimers()
{
  // 控制循环定时器
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / params_.controller_frequency)),
    std::bind(&NavigationNode::controlLoop, this));

  // 重规划检查定时器
  replanning_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / params_.planner_frequency)),
    std::bind(&NavigationNode::replanningCheck, this));

  // 局部 costmap 发布定时器（10Hz）
  local_costmap_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&NavigationNode::publishLocalCostmap, this));

  // 全局 costmap 只在 updateCostmap() 里发布一次; QoS 是 transient_local,
  // 后启动的 RViz 也能收到最后一帧, 不需要 10Hz 重复发送全图.

  diagnostics_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&NavigationNode::publishDiagnostics, this));

  RCLCPP_INFO(get_logger(), "Timers initialized");
}

void NavigationNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_velocity_ = msg->twist.twist;
  last_odom_sec_ = now().seconds();
  odom_pose_ = msg->pose.pose;
  odom_pose_valid_ = true;

  // 更新当前位姿
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose.pose;

  RCLCPP_DEBUG(get_logger(), "Odometry updated: pos=(%.2f, %.2f), vel=(%.2f, %.2f)",
    current_pose_.pose.position.x,
    current_pose_.pose.position.y,
    current_velocity_.linear.x,
    current_velocity_.angular.z);
}

void NavigationNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 将激光点转换到 map 坐标系
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      map_frame_, msg->header.frame_id, msg->header.stamp,
      tf2::durationFromSec(0.1));
  } catch (tf2::TransformException &) { return; }

  last_scan_sec_ = now().seconds();
  laser_points_.clear();
  double angle = msg->angle_min;
  for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max) continue;
    geometry_msgs::msg::PointStamped lp;
    lp.header = msg->header;
    lp.point.x = msg->ranges[i] * std::cos(angle);
    lp.point.y = msg->ranges[i] * std::sin(angle);
    lp.point.z = 0.0;
    geometry_msgs::msg::PointStamped mp;
    tf2::doTransform(lp, mp, tf);
    laser_points_.push_back(mp.point);
  }
  // 激光点只更新实时障碍层. 全局 costmap 由地图回调按地图频率重建;
  // 局部 costmap 由 10Hz 定时器使用这些点重建. 不在 10Hz 扫描回调里
  // 全图 BFS/重规划, 否则会阻塞控制定时器和地图/TF 回调.
}


void NavigationNode::updateCostmap()
{
  if (!current_map_) return;

  unsigned int w = current_map_->info.width;
  unsigned int h = current_map_->info.height;
  float res = current_map_->info.resolution;
  double ox = current_map_->info.origin.position.x;
  double oy = current_map_->info.origin.position.y;

  // ========== 全局 costmap（全图，给规划器） ==========
  // 合并静态地图 + 激光实时障碍物

  if (planner_strategy_) {
    auto grid = std::make_shared<core::CostmapGrid>();
    grid->info.width = w;
    grid->info.height = h;
    grid->info.resolution = res;
    grid->info.origin.position.x = ox;
    grid->info.origin.position.y = oy;
    grid->data = current_map_->data;

    core::Costmap cm;
    cm.grid = grid;
    cm.width = w; cm.height = h; cm.resolution = res;
    cm.origin_x = ox; cm.origin_y = oy;

    // 从静态地图初始化
    cm.data.assign(current_map_->data.size(), 0);
    for (size_t i = 0; i < current_map_->data.size(); ++i) {
      const int8_t v = current_map_->data[i];
      // 254 = unknown (A* 不可穿越, 但不参与膨胀);
      // 255 = occupied (不可穿越, 参与膨胀). 这样前沿目标(自由格紧邻 unknown)
      // 不会被膨胀成障碍, 同时也不会为了抄近路穿过未建图区域.
      if (v < 0) cm.data[i] = 254;
      else if (v >= 100) cm.data[i] = 255;
      else cm.data[i] = 0;
    }

    // 叠加激光实时障碍物; 扫描停止后不能把旧点永久写进规划图.
    const bool laser_fresh = last_scan_sec_ > 0.0 &&
      (now().seconds() - last_scan_sec_ <= 1.0);
    if (laser_fresh) {
      for (const auto& pt : laser_points_) {
        int mx = static_cast<int>(std::floor((pt.x - ox) / res));
        int my = static_cast<int>(std::floor((pt.y - oy) / res));
        if (mx >= 0 && mx < static_cast<int>(w) && my >= 0 && my < static_cast<int>(h))
          cm.data[my * w + mx] = 255;
      }
    }

    // 发布用 costmap 用原始地图 + 安全环; 路径硬边界比它多一格,
    // 避免路径落在显示的 100 区内.
    int inflate_cells = static_cast<int>(std::ceil(inflation_radius_ / res));
    int safety_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_ / res)));
    // 即使 inflation_radius 设得比 robot_radius 小, 也要把硬安全环扩出来.
    const int max_inflate_cells = std::max(inflate_cells, safety_cells);
    std::vector<float> dist(w * h, std::numeric_limits<float>::max());
    std::queue<std::pair<int,int>> q;
    // 只让真实障碍物参与膨胀. unknown 在 planner costmap 里仍是 254
    // (不可穿越), 但不能把它当成障碍物向外膨胀, 否则前沿自由格会被
    // 硬安全环标成 100, explorer 选出来的目标会不断被 nudge/判不可达.
    for (unsigned int y = 0; y < h; ++y)
      for (unsigned int x = 0; x < w; ++x)
        if (cm.data[y * w + x] == 255) { dist[y * w + x] = 0; q.push({x, y}); }
    const int dx4[4] = {-1,0,1,0}, dy4[4] = {0,-1,0,1};
    while (!q.empty()) {
      auto [cx, cy] = q.front(); q.pop();
      float cd = dist[cy * w + cx];
      if (cd >= max_inflate_cells) continue;
      for (int i = 0; i < 4; ++i) {
        int nx = cx + dx4[i], ny = cy + dy4[i];
        if (nx < 0 || nx >= static_cast<int>(w) || ny < 0 || ny >= static_cast<int>(h)) continue;
        if (cd + 1 < dist[ny * w + nx]) { dist[ny * w + nx] = cd + 1; q.push({nx, ny}); }
      }
    }
    nav_msgs::msg::OccupancyGrid costmap_msg;
    costmap_msg.header.frame_id = map_frame_;
    costmap_msg.header.stamp = now();
    costmap_msg.info.resolution = res;
    costmap_msg.info.width = w;
    costmap_msg.info.height = h;
    costmap_msg.info.origin.position.x = ox;
    costmap_msg.info.origin.position.y = oy;
    costmap_msg.info.origin.orientation.w = 1.0;
    costmap_msg.data.resize(w * h);
    const int inv_span = std::max(1, inflate_cells - safety_cells);
    float inv = 1.0f / static_cast<float>(inv_span);
    for (size_t i = 0; i < w * h; ++i) {
      if (cm.data[i] >= 254) { costmap_msg.data[i] = 100; continue; }
      float d = dist[i];
      if (d <= safety_cells) costmap_msg.data[i] = 100;
      else if (d <= inflate_cells) costmap_msg.data[i] = static_cast<int8_t>(
        (1.0f - (d - safety_cells) * inv) * 99);
      else costmap_msg.data[i] = 0;
    }
    global_costmap_msg_ = costmap_msg;
    m_has_global_costmap_ = true;
    global_costmap_pub_->publish(global_costmap_msg_);

    // 规划器使用与 RViz 同一张 costmap 的代价:
    //   0 = 无色自由; 1..99 = 蓝色软膨胀; 100 = 紫色致命.
    // 具体是否把蓝色当墙由规划时决定 (优先全 0 路径).
    planner_costmap_ = makePlannerCostmap(costmap_msg);
    m_has_planner_costmap_ = true;
    planner_strategy_->setCostmap(planner_costmap_);
  }
}

void NavigationNode::publishLocalCostmap()
{
  if (!current_map_) return;
  unsigned int w = current_map_->info.width;
  unsigned int h = current_map_->info.height;
  float res = current_map_->info.resolution;
  double ox = current_map_->info.origin.position.x;
  double oy = current_map_->info.origin.position.y;
  double rx = 0, ry = 0;
  // 局部窗口必须跟随当前机器人, 不能跟随上一帧点云的位姿; 点云已经
  // 变换到 map 系, 只是障碍物层, 窗口中心用当前 TF.
  try {
    auto tf = tf_buffer_->lookupTransform(
      map_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
    rx = tf.transform.translation.x;
    ry = tf.transform.translation.y;
  } catch (tf2::TransformException &) {
    if (safety_pose_valid_) {
      rx = safety_pose_x_;
      ry = safety_pose_y_;
    } else {
      return;
    }
  }
  double win_width = local_costmap_size_;
  int win_half = static_cast<int>(std::ceil(win_width * 0.5 / res));
  int cx = static_cast<int>(std::floor((rx - ox) / res));
  int cy = static_cast<int>(std::floor((ry - oy) / res));
  // 奇数尺寸, 让机器人正好落在中心格 (偶数尺寸会偏半格).
  int lw = win_half * 2 + 1;
  nav_msgs::msg::OccupancyGrid local_costmap;
  local_costmap.header.frame_id = map_frame_;
  local_costmap.header.stamp = now();
  local_costmap.info.resolution = res;
  local_costmap.info.width = lw;
  local_costmap.info.height = lw;
  local_costmap.info.origin.position.x = rx - (win_half + 0.5) * res;
  local_costmap.info.origin.position.y = ry - (win_half + 0.5) * res;
  local_costmap.info.origin.orientation.w = 1.0;
  std::vector<int8_t> raw(lw * lw, 100);  // 地图外默认致命, 防止驶出已知地图
  for (int dy = -win_half; dy <= win_half; ++dy)
    for (int dx = -win_half; dx <= win_half; ++dx) {
      int mx = cx + dx, my = cy + dy;
      if (mx < 0 || mx >= static_cast<int>(w) || my < 0 || my >= static_cast<int>(h)) continue;
      int gx = dx + win_half, gy = dy + win_half;
      const int8_t mv = current_map_->data[my * w + mx];
      raw[gy * lw + gx] = (mv >= 100 || (local_unknown_as_lethal_ && mv < 0))
        ? 100 : 0;
    }
  // real-time 2D laser + 3D LiDAR points in the local costmap window
  auto mark_point = [&](double px, double py) {
      const int mx = static_cast<int>(std::floor((px - ox) / res));
      const int my = static_cast<int>(std::floor((py - oy) / res));
      const int dx = mx - cx;
      const int dy = my - cy;
      if (dx < -win_half || dx > win_half || dy < -win_half || dy > win_half) return;
      raw[(dy + win_half) * lw + (dx + win_half)] = 100;
    };
  const bool laser_fresh = last_scan_sec_ > 0.0 &&
    (now().seconds() - last_scan_sec_ <= 1.0);
  if (laser_fresh) {
    for (const auto& pt : laser_points_) {
      mark_point(pt.x, pt.y);
    }
  }
  const bool safety_fresh = safety_have_cloud_ &&
    (now().seconds() - safety_last_cloud_sec_ <= safety_source_timeout_);
  if (safety_fresh) {
    for (const auto& p : safety_points_map_) {
      mark_point(p.first, p.second);
    }
  }

  // 局部沿用同一语义:
  //   robot_radius 内 -> 100 紫色, DWA 绝对不进入;
  //   robot_radius ~ inflation_radius -> 1..99 蓝色软代价, 窄通道可进入.
  int l_inflate = static_cast<int>(std::ceil(inflation_radius_ / res));
  int l_safety = std::max(1, static_cast<int>(std::ceil(robot_radius_ / res)));
  l_inflate = std::max(l_inflate, l_safety + 1);
  std::vector<float> ldist(lw * lw, std::numeric_limits<float>::max());
  std::queue<std::pair<int,int>> lq;
  for (int y = 0; y < lw; ++y)
    for (int x = 0; x < lw; ++x)
      if (raw[y * lw + x] >= 100) { ldist[y * lw + x] = 0; lq.push({x, y}); }
  const int ddx[4] = {-1,0,1,0}, ddy[4] = {0,-1,0,1};
  while (!lq.empty()) {
    auto [cx2, cy2] = lq.front(); lq.pop();
    float cd = ldist[cy2 * lw + cx2];
    if (cd >= l_inflate) continue;
    for (int i = 0; i < 4; ++i) {
      int nx = cx2 + ddx[i], ny = cy2 + ddy[i];
      if (nx < 0 || nx >= lw || ny < 0 || ny >= lw) continue;
      if (cd + 1 < ldist[ny * lw + nx]) { ldist[ny * lw + nx] = cd + 1; lq.push({nx, ny}); }
    }
  }
  float linv = 1.0f / (l_inflate - l_safety);
  local_costmap.data.resize(lw * lw);
  for (int i = 0; i < lw * lw; ++i) {
    if (raw[i] >= 100) { local_costmap.data[i] = 100; continue; }
    float d = ldist[i];
    if (d <= l_safety) local_costmap.data[i] = 100;
    else if (d <= l_inflate) local_costmap.data[i] = static_cast<int8_t>((1.0f - (d - l_safety) * linv) * 99);
    else local_costmap.data[i] = 0;
  }
  // 记录清除前的中心代价, 便于诊断"机器人是否被墙/自身点云包住".
  last_local_center_cost_ = local_costmap.data[
    static_cast<size_t>(lw / 2) * lw + static_cast<size_t>(lw / 2)];
  // 机器人自身所在的小区域清成自由, 否则贴墙/自身噪点导致局部中心 100,
  // DWA/MPPI 没有可行轨迹; 真正碰撞仍由安全层和紫色硬边界负责.
  if (local_clear_radius_ > 0.0) {
    const int clear_cells = std::max(1, static_cast<int>(
      std::ceil(local_clear_radius_ / res)));
    for (int dy = -clear_cells; dy <= clear_cells; ++dy) {
      for (int dx = -clear_cells; dx <= clear_cells; ++dx) {
        if (dx * dx + dy * dy > clear_cells * clear_cells) continue;
        const int gx = win_half + dx;
        const int gy = win_half + dy;
        if (gx < 0 || gx >= lw || gy < 0 || gy >= lw) continue;
        local_costmap.data[static_cast<size_t>(gy) * lw + gx] = 0;
      }
    }
  }

  last_local_lethal_cells_ = 0;
  last_local_blue_cells_ = 0;
  for (const auto v : local_costmap.data) {
    if (v >= 100) ++last_local_lethal_cells_;
    else if (v > 0) ++last_local_blue_cells_;
  }

  if (controller_strategy_) {
    core::CostmapGrid grid;
    grid.info.width = local_costmap.info.width;
    grid.info.height = local_costmap.info.height;
    grid.info.resolution = local_costmap.info.resolution;
    grid.info.origin.position.x = local_costmap.info.origin.position.x;
    grid.info.origin.position.y = local_costmap.info.origin.position.y;
    grid.data = local_costmap.data;
    controller_strategy_->setLocalCostmap(grid);
  }
  local_costmap_pub_->publish(local_costmap);
}

void NavigationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = msg;
  updateCostmap();
  RCLCPP_INFO(get_logger(), "Map received: %d x %d, res=%.3f",
    msg->info.width, msg->info.height, msg->info.resolution);

  // 重规划由 1Hz 定时器统一执行. 不在地图回调里跑 A*, 避免阻塞地图/TF 回调.
  // 也不在这里对 FAILED 目标自动重试: 失败已经通过 /goal_status 通知
  // explorer, 由上层换目标; 否则旧目标会每秒重试并把过期 failed 状态
  // 混到新目标上.
}

void NavigationNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // inactive/cleanup 状态下控制定时器不存在, 接受目标只会让上层误以为
  // 导航已开始 (accepted) 而车不会动.
  if (get_current_state().label() != "active") {
    RCLCPP_WARN(get_logger(),
      "Ignoring /goal_pose in lifecycle state '%s'",
      get_current_state().label().c_str());
    return;
  }
  RCLCPP_INFO(get_logger(),
    "Received /goal_pose: (%.2f, %.2f, %.2f)",
    msg->pose.position.x, msg->pose.position.y,
    tf2::getYaw(msg->pose.orientation));

  // 目标 frame 必须转换到导航 global frame; 之前直接把 map 坐标当 odom
  // 使用, 在 map->odom 非 identity 时会去错误位置.
  geometry_msgs::msg::PoseStamped goal_msg = *msg;
  if (!goal_msg.header.frame_id.empty() &&
      goal_msg.header.frame_id != global_frame_) {
    try {
      goal_msg = tf_buffer_->transform(
        goal_msg, global_frame_, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(),
        "Cannot transform goal from '%s' to '%s': %s",
        goal_msg.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
      return;
    }
  }
  goal_pose_ = goal_msg;
  executeNavigation(goal_pose_);
}

void NavigationNode::stopCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // 带 goal token 的 stop 只能停止对应目标; 防止 stop 晚到误停新目标.
  if (msg && !msg->data.empty()) {
    const auto & stamp = goal_pose_.header.stamp;
    const int64_t stamp_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL +
      static_cast<int64_t>(stamp.nanosec);
    if (msg->data != std::to_string(stamp_ns)) {
      return;
    }
  }
  RCLCPP_INFO(get_logger(), "Stop request received");
  stopNavigation();
}

void NavigationNode::controlLoop()
{
  if (!is_navigating_ || is_paused_) {
    return;
  }

  // 更新当前位姿; 定位链路失效时必须主动发零速度, 不能保留上一条 cmd_vel.
  if (!updateCurrentPose()) {
    geometry_msgs::msg::Twist stop_cmd;
    publishVelocityCommand(stop_cmd);
    return;
  }
  if (now().seconds() - last_odom_sec_ > tf_stale_timeout_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Odom stale (%.2fs); stopping", now().seconds() - last_odom_sec_);
    geometry_msgs::msg::Twist stop_cmd;
    publishVelocityCommand(stop_cmd);
    return;
  }

  // 到达 XY 后, 若启用了目标朝向约束, 先原地转到目标 yaw.
  if (enforce_goal_yaw_ && !current_path_.poses.empty()) {
    const double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
    const double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
    if (std::hypot(dx, dy) < params_.goal_tolerance_xy) {
      const double goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);
      const double current_yaw = tf2::getYaw(current_pose_.pose.orientation);
      double err = goal_yaw - current_yaw;
      while (err > M_PI) err -= 2.0 * M_PI;
      while (err < -M_PI) err += 2.0 * M_PI;
      if (std::abs(err) > params_.goal_tolerance_yaw) {
        geometry_msgs::msg::Twist rotate;
        rotate.angular.z = std::clamp(
          1.5 * err, -params_.max_velocity_theta, params_.max_velocity_theta);
        publishVelocityCommand(rotate);
        return;
      }
    }
  }

  // 检查是否到达目标
  if (isGoalReached()) {
    RCLCPP_INFO(get_logger(), "Goal reached!");
    publishGoalStatus("reached");
    current_state_ = NavigationState::GOAL_REACHED;
    stopNavigation();
    return;
  }

  // ========== 初始原地旋转：确保机器人面朝路径方向再前进 ==========
  if (m_need_initial_rotate) {
    if (m_initial_rotate_start_sec_ <= 0.0) {
      m_initial_rotate_start_sec_ = now().seconds();
    } else if (now().seconds() - m_initial_rotate_start_sec_ > 4.0) {
      RCLCPP_WARN(get_logger(),
        "Initial rotation timeout; handing control to %s",
        controller_name_.c_str());
      m_need_initial_rotate = false;
    }
    geometry_msgs::msg::Twist rot_cmd;
    double current_yaw = tf2::getYaw(current_pose_.pose.orientation);
    double target_yaw = m_initial_rotate_target;
    double angle_error = target_yaw - current_yaw;
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    if (std::abs(angle_error) < 0.08) {
      m_need_initial_rotate = false;
      RCLCPP_INFO(get_logger(), "Initial rotation done, starting %s",
        controller_name_.c_str());
    } else if (m_need_initial_rotate) {
      rot_cmd.angular.z = std::clamp(1.5 * angle_error, -0.8, 0.8);
      publishVelocityCommand(rot_cmd);
      return;
    }
  }

  // 卡死检测必须基于真实位姿变化, 不能只看 odom.twist.
  // 本项目的 SLAM 之前不填 twist.angular, 机器人原地转/慢慢走时
  // current_velocity_ 仍接近 0, 会每 2s 误触发一次后退旋转恢复.
  if (recovery_enabled_) {
    const double now_sec = now().seconds();
    const double pose_x = current_pose_.pose.position.x;
    const double pose_y = current_pose_.pose.position.y;
    const double pose_yaw = tf2::getYaw(current_pose_.pose.orientation);
    if (!motion_ref_valid_) {
      motion_ref_x_ = pose_x;
      motion_ref_y_ = pose_y;
      motion_ref_yaw_ = pose_yaw;
      motion_ref_sec_ = now_sec;
      motion_ref_valid_ = true;
      motion_stall_cycles_ = 0;
    } else {
      const double moved = std::hypot(pose_x - motion_ref_x_, pose_y - motion_ref_y_);
      double dyaw = pose_yaw - motion_ref_yaw_;
      while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
      while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
      if (moved > 0.03 || std::abs(dyaw) > 0.05) {
        motion_ref_x_ = pose_x;
        motion_ref_y_ = pose_y;
        motion_ref_yaw_ = pose_yaw;
        motion_ref_sec_ = now_sec;
        motion_stall_cycles_ = 0;
      } else if (++motion_stall_cycles_ >= recovery_trigger_cycles_) {
        recovery_active_ = true;
        recovery_until_sec_ = now_sec + recovery_duration_;
        motion_stall_cycles_ = 0;
        motion_ref_valid_ = false;
        RCLCPP_WARN(get_logger(), "Robot not moving; recovery rotation");
      }
    }
  } else {
    motion_stall_cycles_ = 0;
    motion_ref_valid_ = false;
  }

  // 安全层 STOP 持续 > 1s: 原地旋转寻找脱困方向 (安全层不再继续拦截
  // 这个明确的逃逸动作, 避免安全层和 DWA 互相卡死).
  if (safety_stop_active_ && !recovery_active_ &&
    now().seconds() - safety_stop_since_sec_ >= 1.0) {
    recovery_active_ = true;
    recovery_until_sec_ = now().seconds() + recovery_duration_;
    safety_stop_active_ = false;
    stall_cycles_ = 0;
    RCLCPP_WARN(get_logger(), "Safety STOP held; recovery rotation");
  }

  // DWA 无可行轨迹/贴着障碍时, 短暂旋转恢复
  if (recovery_active_) {
    if (now().seconds() < recovery_until_sec_) {
      geometry_msgs::msg::Twist recovery_cmd;
      recovery_cmd.linear.x = recovery_linear_velocity_;
      recovery_cmd.angular.z = recovery_angular_velocity_;
      // 恢复不能盲倒车: 后向小矩形内有 LiDAR 点就只原地旋转.
      const std::vector<std::pair<double, double>> rear_box = {
        {-0.65, -0.35}, {0.0, -0.35}, {0.0, 0.35}, {-0.65, 0.35}};
      if (recovery_cmd.linear.x < 0.0 &&
          core::countPointsInPolygon(safety_points_, rear_box) > 0)
      {
        recovery_cmd.linear.x = 0.0;
      }
      if (recovery_bypass_safety_) {
        cmd_vel_pub_->publish(recovery_cmd);
        last_published_cmd_ = recovery_cmd;
      } else {
        publishVelocityCommand(recovery_cmd);
      }
      return;
    }
    recovery_active_ = false;
    stall_cycles_ = 0;
    motion_ref_valid_ = false;
  }

  // 计算速度命令
  geometry_msgs::msg::Twist cmd_vel = computeVelocityCommand();

  if (recovery_enabled_ &&
    std::abs(cmd_vel.linear.x) < 0.02 && std::abs(cmd_vel.angular.z) < 0.02) {
    if (++stall_cycles_ >= recovery_trigger_cycles_) {
      recovery_active_ = true;
      recovery_until_sec_ = now().seconds() + recovery_duration_;
      stall_cycles_ = 0;
      RCLCPP_WARN(get_logger(), "Navigation stalled; recovery maneuver");
    }
  } else {
    stall_cycles_ = 0;
  }

  // 发布速度命令
  publishVelocityCommand(cmd_vel);

  // 发布路径（用于可视化）
  publishPath(current_path_);
}

void NavigationNode::applyPlanningResult(const core::Result<core::Path> & result)
{
  if (!result.is_ok()) {
    geometry_msgs::msg::Twist stop_cmd;
    publishVelocityCommand(stop_cmd);
    is_navigating_ = false;
    current_state_ = NavigationState::FAILED;
    current_path_.poses.clear();
    publishGoalStatus("failed");
    global_path_pub_->publish(current_path_);
    m_need_initial_rotate = false;
    m_has_effective_goal_ = false;
    recovery_active_ = false;
    motion_ref_valid_ = false;
    if (controller_strategy_) controller_strategy_->reset();
    RCLCPP_WARN(get_logger(), "Replan failed: %s; waiting for new goal",
      result.error_message().c_str());
    return;
  }

  current_path_ = RosUtils::toRosPath(result.value(), goal_pose_.header.frame_id, now());
  smoothCurrentPath(current_path_);
  if (controller_strategy_) {
    // DWA/MPPI 的 setPath 会重置进度; 地图 1Hz 重规划时最多 2Hz 刷给控制器,
    // 中间路径仍发布, 由安全层兜底.
    const double now_sec = now().seconds();
    if (now_sec - last_controller_path_sec_ >= 0.5) {
      controller_strategy_->setPath(RosUtils::toCorePath(current_path_));
      last_controller_path_sec_ = now_sec;
    }
  }
  global_path_pub_->publish(current_path_);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "Replanned path: %zu points", current_path_.poses.size());
}

void NavigationNode::waitForAsyncPlan()
{
  if (!async_plan_future_.valid()) return;
  try {
    (void)async_plan_future_.get();
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Async planning exception: %s", e.what());
  }
}

void NavigationNode::replanningCheck()
{
  if (!is_navigating_ || is_paused_ || !planner_strategy_ || !current_map_) {
    return;
  }
  if (goal_pose_.header.frame_id.empty() || m_need_initial_rotate) {
    return;
  }
  if (!m_has_planner_costmap_) {
    return;
  }
  if (!updateCurrentPose()) {
    geometry_msgs::msg::Twist stop_cmd;
    publishVelocityCommand(stop_cmd);
    return;
  }

  // 后台规划完成: 取结果并应用; 规划期间控制/安全定时器继续运行.
  if (async_plan_future_.valid()) {
    if (async_plan_future_.wait_for(std::chrono::seconds(0)) !=
        std::future_status::ready)
    {
      return;
    }
    core::Result<core::Path> result = core::Result<core::Path>::error(
      core::ErrorCode::PLANNING_FAILED, "async planning exception");
    try {
      result = async_plan_future_.get();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Async planning exception: %s", e.what());
    }
    if (!is_navigating_ || is_paused_) return;
    applyPlanningResult(result);
    return;
  }

  const auto start = RosUtils::toCorePose(current_pose_);
  if (!m_has_effective_goal_) {
    effective_goal_ = RosUtils::toCorePose(goal_pose_);
    adjustGoalToFree(effective_goal_);
    m_has_effective_goal_ = true;
  }
  if (!async_planner_) return;

  // 异步实例独立持有 costmap, 不会和主 planner 的 mapCallback 竞争.
  async_planner_->setCostmap(planner_costmap_);
  const core::Pose2D goal = effective_goal_;
  async_plan_future_ = std::async(std::launch::async,
    [this, start, goal]() {
      return async_planner_->plan(start, goal);
    });
}


void NavigationNode::executeNavigation(const geometry_msgs::msg::PoseStamped & goal)
{
  // 新目标会覆盖旧路径. 规划期间先停车并清掉旧路径, 规划失败时
  // 不能继续沿上一条路径开.
  is_navigating_ = false;
  current_state_ = NavigationState::PLANNING;
  current_path_.poses.clear();
  m_has_effective_goal_ = false;
  m_need_initial_rotate = false;
  recovery_active_ = false;
  stall_cycles_ = 0;
  motion_stall_cycles_ = 0;
  motion_ref_valid_ = false;
  {
    geometry_msgs::msg::Twist stop_cmd;
    publishVelocityCommand(stop_cmd);
  }
  // 新目标先停住并丢弃正在执行的重规划, 防止旧结果在稍后覆盖新路径.
  waitForAsyncPlan();
  global_path_pub_->publish(current_path_);

  // 更新当前位姿
  if (!updateCurrentPose()) {
    RCLCPP_ERROR(get_logger(), "Cannot get current pose for planning");
    publishGoalStatus("failed");
    current_state_ = NavigationState::FAILED;
    return;
  }

  if (!planner_strategy_ || !m_has_planner_costmap_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "No global costmap yet; rejecting goal");
    publishGoalStatus("failed");
    current_state_ = NavigationState::FAILED;
    return;
  }

  // 规划路径
  RCLCPP_INFO(get_logger(), "Planning path to goal...");

  auto start_pose = RosUtils::toCorePose(current_pose_);
  auto goal_pose_core = RosUtils::toCorePose(goal);
  if (!adjustGoalToFree(goal_pose_core)) {
    RCLCPP_WARN(get_logger(),
      "Goal (%.2f, %.2f) has no free cell nearby; rejecting",
      goal_pose_core.x, goal_pose_core.y);
    publishGoalStatus("failed");
    current_state_ = NavigationState::FAILED;
    return;
  }
  effective_goal_ = goal_pose_core;
  m_has_effective_goal_ = true;

  auto result = planner_strategy_->plan(start_pose, effective_goal_);
  if (!result.is_ok()) {
    RCLCPP_ERROR(get_logger(), "Planner '%s' failed: %s",
      planner_name_.c_str(), result.error_message().c_str());
    publishGoalStatus("failed");
    current_state_ = NavigationState::FAILED;
    m_has_effective_goal_ = false;
    if (controller_strategy_) controller_strategy_->reset();
    return;
  }

  current_path_ = RosUtils::toRosPath(result.value(), goal.header.frame_id, now());
  smoothCurrentPath(current_path_);

  // 将路径注入控制器
  if (controller_strategy_) {
    controller_strategy_->setPath(RosUtils::toCorePath(current_path_));
    last_controller_path_sec_ = now().seconds();
  }

  // 发布全局路径
  global_path_pub_->publish(current_path_);

  RCLCPP_INFO(get_logger(),
    "Path planned with %zu points",
    current_path_.poses.size());

  // 告诉上层目标已被接受; explorer 用它判断导航节点是否真的收到了目标.
  publishGoalStatus("accepted");

  // 大角度起步先原地转, 4s 超时后交给控制器 (见 controlLoop).
  // DWA/MPPI 本身也能转向; 这个只用于缩短大角度目标的起步时间.
  m_need_initial_rotate = false;
  if (current_path_.poses.size() > 2) {
    double current_yaw = tf2::getYaw(current_pose_.pose.orientation);
    double dx = current_path_.poses[1].pose.position.x - current_path_.poses[0].pose.position.x;
    double dy = current_path_.poses[1].pose.position.y - current_path_.poses[0].pose.position.y;
    double path_yaw = std::atan2(dy, dx);
    double yaw_diff = path_yaw - current_yaw;
    while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
    if (std::abs(yaw_diff) > 0.35) {
      m_need_initial_rotate = true;
      m_initial_rotate_target = path_yaw;
      m_initial_rotate_start_sec_ = now().seconds();
      RCLCPP_INFO(get_logger(), "Initial rotation needed: %.1f deg", yaw_diff * 180.0 / M_PI);
    }
  }

  // 设置导航状态
  is_navigating_ = true;
  is_paused_ = false;
  current_state_ = NavigationState::CONTROLLING;
}

bool NavigationNode::updateCurrentPose()
{
  try {
    // 从TF获取当前位姿
    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer_->lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero);

    const rclcpp::Time tf_stamp(transform.header.stamp);
    const double tf_age = (this->now() - tf_stamp).seconds();
    if (tf_age > tf_stale_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "TF stale: %s -> %s age=%.2fs (limit %.2fs)",
        global_frame_.c_str(), robot_frame_.c_str(), tf_age, tf_stale_timeout_);
      return false;
    }

    const double x = transform.transform.translation.x;
    const double y = transform.transform.translation.y;
    const double yaw = tf2::getYaw(transform.transform.rotation);

    // SLAM/loop-closure 可能让 TF 停住或突然跳变; 这期间不能继续用旧位姿开车.
    if (last_tf_pose_valid_) {
      const double dx = x - last_tf_x_;
      const double dy = y - last_tf_y_;
      double dyaw = yaw - last_tf_yaw_;
      while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
      while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
      if (std::hypot(dx, dy) > 0.75 || std::abs(dyaw) > 1.0) {
        last_tf_x_ = x; last_tf_y_ = y; last_tf_yaw_ = yaw;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "TF jump ignored: d=%.2fm dyaw=%.2frad", std::hypot(dx, dy), dyaw);
        return false;
      }
    }
    last_tf_x_ = x; last_tf_y_ = y; last_tf_yaw_ = yaw;
    last_tf_pose_valid_ = true;

    // TF 与 odom 应描述同一个底座; 差值异常时判定定位链路不可信.
    if (odom_pose_valid_ && now().seconds() - last_odom_sec_ <= tf_stale_timeout_) {
      const double ox = odom_pose_.position.x;
      const double oy = odom_pose_.position.y;
      double oyaw = tf2::getYaw(odom_pose_.orientation);
      double dyaw = yaw - oyaw;
      while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
      while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
      if (std::hypot(x - ox, y - oy) > 0.6 || std::abs(dyaw) > 0.8) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "TF/odom mismatch: d=%.2fm dyaw=%.2frad; stopping",
          std::hypot(x - ox, y - oy), dyaw);
        return false;
      }
    }

    // 注意: odom.twist 的可信度由 SLAM 的 ROS2 层负责
    // (world->body 线速度 + 四元数差分角速度). 这里不再用相邻 TF 差分
    // 覆写 current_velocity_: TF 跳变/回环时会把巨大速度值混进控制器.
    // 控制器侧同时把上一周期下发的 cmd 作为加速度状态兜底.

    current_pose_.header.stamp = transform.header.stamp;
    current_pose_.header.frame_id = global_frame_;
    current_pose_.pose.position.x = x;
    current_pose_.pose.position.y = y;
    current_pose_.pose.position.z = transform.transform.translation.z;
    current_pose_.pose.orientation = transform.transform.rotation;

    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Transform error: %s", ex.what());
    return false;
  }
}

bool NavigationNode::isGoalReached()
{
  // 优先使用控制器的目标到达判断; 但路径被清空时不能误判到达.
  if (controller_strategy_ && !current_path_.poses.empty()) {
    auto core_pose = RosUtils::toCorePose(current_pose_);
    if (!controller_strategy_->isGoalReached(core_pose)) return false;
    if (enforce_goal_yaw_) {
      const double goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);
      const double current_yaw = tf2::getYaw(current_pose_.pose.orientation);
      double err = goal_yaw - current_yaw;
      while (err > M_PI) err -= 2.0 * M_PI;
      while (err < -M_PI) err += 2.0 * M_PI;
      return std::abs(err) <= params_.goal_tolerance_yaw;
    }
    return true;
  }

  // 后备：使用简单的欧氏距离判断
  double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
  double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  double goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);
  double current_yaw = tf2::getYaw(current_pose_.pose.orientation);
  double angle_diff = std::abs(goal_yaw - current_yaw);

  while (angle_diff > M_PI) {
    angle_diff -= 2.0 * M_PI;
  }
  angle_diff = std::abs(angle_diff);

  bool xy_reached = distance < params_.goal_tolerance_xy;
  bool yaw_reached = angle_diff < params_.goal_tolerance_yaw;

  return xy_reached && yaw_reached;
}

geometry_msgs::msg::Twist NavigationNode::computeVelocityCommand()
{
  // 优先使用控制器的速度计算
  if (controller_strategy_) {
    auto core_pose = RosUtils::toCorePose(current_pose_);
    // 用上一周期真正下发的速度作为加速度状态. odom.twist 可能滞后/缺角速度,
    // 直接作为 DWA/MPPI 动态窗口输入会把首段命令压到 max_accel*dt.
    auto core_vel = RosUtils::toCoreVelocity(last_published_cmd_);
    auto cmd = controller_strategy_->computeVelocityCommand(core_pose, core_vel);
    geometry_msgs::msg::Twist result = RosUtils::toRosTwist(cmd);
    RCLCPP_DEBUG(get_logger(),
      "Controller computed velocity: linear=%.2f, angular=%.2f",
      result.linear.x, result.angular.z);
    return result;
  }

  // 后备：简化的速度计算
  geometry_msgs::msg::Twist cmd_vel;

  double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
  double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  double target_angle = std::atan2(dy, dx);
  double current_yaw = tf2::getYaw(current_pose_.pose.orientation);

  double angle_diff = target_angle - current_yaw;
  while (angle_diff > M_PI) {
    angle_diff -= 2.0 * M_PI;
  }
  while (angle_diff < -M_PI) {
    angle_diff += 2.0 * M_PI;
  }

  if (std::abs(angle_diff) < M_PI / 4) {
    double vel_scale = std::min(distance / 1.0, 1.0);
    cmd_vel.linear.x = vel_scale * params_.max_velocity_x;
  } else {
    cmd_vel.linear.x = 0.0;
  }

  cmd_vel.angular.z = 2.0 * angle_diff;

  cmd_vel.linear.x = std::clamp(cmd_vel.linear.x,
    params_.min_velocity_x, params_.max_velocity_x);
  cmd_vel.angular.z = std::clamp(cmd_vel.angular.z,
    -params_.max_velocity_theta, params_.max_velocity_theta);

  RCLCPP_DEBUG(get_logger(),
    "Computed velocity: linear=%.2f, angular=%.2f",
    cmd_vel.linear.x, cmd_vel.angular.z);

  return cmd_vel;
}

void NavigationNode::publishVelocityCommand(const geometry_msgs::msg::Twist & cmd_vel)
{
  if (!cmd_vel_pub_) return;
  geometry_msgs::msg::Twist out = cmd_vel;
  if (safety_enabled_) {
    const double now_sec = now().seconds();
    const bool sensor_ok = safety_have_cloud_ &&
      (now_sec - safety_last_cloud_sec_ <= safety_source_timeout_);
    if (!sensor_ok) {
      // Nav2 Collision Monitor 默认行为: 传感器失效时"瞎着眼放行".
      if (safety_fail_safe_on_timeout_) out = geometry_msgs::msg::Twist();
      publishSafetyState(safety_fail_safe_on_timeout_ ?
        "STOP_SENSOR_TIMEOUT" : "PASS_SENSOR_STALE");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Safety layer: sensor stale (have_cloud=%s, age=%.2fs)",
        safety_have_cloud_ ? "true" : "false",
        safety_have_cloud_ ? now_sec - safety_last_cloud_sec_ : -1.0);
    } else {
      const core::VelocityCommand core_cmd(
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
      const core::SafetyResult result =
        safety_monitor_.filter(core_cmd, safety_points_);
      out.linear.x = result.command.linear_x;
      out.linear.y = result.command.lateral_y;
      out.angular.z = result.command.angular_z;
      publishSafetyState(result.state);
      const bool stopped = (result.state == "STOP");
      if (stopped && !safety_stop_active_) {
        safety_stop_active_ = true;
        safety_stop_since_sec_ = now_sec;
      } else if (!stopped) {
        safety_stop_active_ = false;
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Safety layer: %s", result.state.c_str());
    }
  } else {
    publishSafetyState("DISABLED");
  }
  last_published_cmd_ = out;
  cmd_vel_pub_->publish(out);
}

void NavigationNode::publishSafetyState(const std::string & state)
{
  if (state == last_safety_state_ || !safety_state_pub_) return;
  last_safety_state_ = state;
  std_msgs::msg::String msg;
  msg.data = state;
  safety_state_pub_->publish(msg);
}

void NavigationNode::publishGoalStatus(const std::string & status)
{
  if (!goal_status_pub_) return;
  std_msgs::msg::String msg;
  msg.data = status;
  // 附带目标时间戳作为 goal token. explorer 用它丢弃上一个目标的
  // 迟到 accepted/failed/reached, 避免旧状态误杀新目标.
  const auto & stamp = goal_pose_.header.stamp;
  if (stamp.sec != 0 || stamp.nanosec != 0) {
    const int64_t stamp_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL +
      static_cast<int64_t>(stamp.nanosec);
    msg.data += ":" + std::to_string(stamp_ns);
  }
  goal_status_pub_->publish(msg);
}

void NavigationNode::publishDiagnostics()
{
  if (!diagnostics_pub_) return;
  const double now_sec = now().seconds();
  const double tf_age = current_pose_.header.stamp.sec > 0 ?
    now_sec - (current_pose_.header.stamp.sec + current_pose_.header.stamp.nanosec * 1e-9) : -1.0;
  const double odom_age = last_odom_sec_ > 0 ? now_sec - last_odom_sec_ : -1.0;
  const double goal_dist = goal_pose_.header.frame_id.empty() ? -1.0 :
    std::hypot(goal_pose_.pose.position.x - current_pose_.pose.position.x,
               goal_pose_.pose.position.y - current_pose_.pose.position.y);

  std::ostringstream os;
  os << std::fixed << std::setprecision(3);
  os << "{"
     << "\"state\":" << static_cast<int>(current_state_.load())
     << ",\"navigating\":" << (is_navigating_.load() ? "true" : "false")
     << ",\"pose\":[" << current_pose_.pose.position.x << ","
     << current_pose_.pose.position.y << ","
     << tf2::getYaw(current_pose_.pose.orientation) << "]"
     << ",\"vel\":[" << current_velocity_.linear.x << ","
     << current_velocity_.angular.z << "]"
     << ",\"cmd\":[" << last_published_cmd_.linear.x << ","
     << last_published_cmd_.angular.z << "]"
     << ",\"tf_age\":" << tf_age
     << ",\"odom_age\":" << odom_age
     << ",\"path_points\":" << current_path_.poses.size()
     << ",\"goal_dist\":" << goal_dist
     << ",\"local_center\":" << last_local_center_cost_
     << ",\"local_lethal\":" << last_local_lethal_cells_
     << ",\"local_blue\":" << last_local_blue_cells_
     << ",\"recovery\":" << (recovery_active_ ? "true" : "false")
     << ",\"initial_rotate\":" << (m_need_initial_rotate ? "true" : "false")
     << ",\"controller\":\"" << controller_name_ << "\""
     << ",\"controller_diag\":\"" <<
        (controller_strategy_ ? controller_strategy_->getDiagnostics() : std::string()) << "\""
     << "}";
  std_msgs::msg::String msg;
  msg.data = os.str();
  diagnostics_pub_->publish(msg);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "diag state=%d pose=(%.2f,%.2f) vel=(%.2f,%.2f) cmd=(%.2f,%.2f) "
    "tf_age=%.2f odom_age=%.2f local_center=%d lethal=%d blue=%d path=%zu rec=%d",
    static_cast<int>(current_state_.load()),
    current_pose_.pose.position.x, current_pose_.pose.position.y,
    current_velocity_.linear.x, current_velocity_.angular.z,
    last_published_cmd_.linear.x, last_published_cmd_.angular.z,
    tf_age, odom_age, last_local_center_cost_, last_local_lethal_cells_,
    last_local_blue_cells_, current_path_.poses.size(), recovery_active_ ? 1 : 0);
}

void NavigationNode::smoothCurrentPath(nav_msgs::msg::Path & path)
{
  if (!enable_path_smoothing_ || path.poses.size() < 3) return;

  planners::SmootherConfig smoother_cfg;
  smoother_cfg.max_iterations = 100;
  smoother_cfg.alpha_start = 0.5;
  smoother_cfg.alpha_end = 0.1;

  std::shared_ptr<core::CostmapGrid> core_map;
  // 优先用已膨胀的 global costmap 做碰撞检测, 防止平滑把路径拉进安全区.
  if (m_has_global_costmap_) {
    core_map = std::make_shared<core::CostmapGrid>();
    core_map->info.width = global_costmap_msg_.info.width;
    core_map->info.height = global_costmap_msg_.info.height;
    core_map->info.resolution = global_costmap_msg_.info.resolution;
    core_map->info.origin.position.x = global_costmap_msg_.info.origin.position.x;
    core_map->info.origin.position.y = global_costmap_msg_.info.origin.position.y;
    core_map->data = global_costmap_msg_.data;
  } else if (current_map_) {
    core_map = std::make_shared<core::CostmapGrid>();
    core_map->info.width = current_map_->info.width;
    core_map->info.height = current_map_->info.height;
    core_map->info.resolution = current_map_->info.resolution;
    core_map->info.origin.position.x = current_map_->info.origin.position.x;
    core_map->info.origin.position.y = current_map_->info.origin.position.y;
    core_map->data = current_map_->data;
  }
  if (!core_map) return;

  const nav_msgs::msg::Path raw_path = path;
  core::Path core_path = RosUtils::toCorePath(path);
  core_path = planners::smoothPath(core_path, core_map, smoother_cfg);
  path = RosUtils::toRosPath(core_path, path.header.frame_id, now());

  // 平滑后再逐段检查一次; 平滑器只保证新顶点/线段不进 100 区, 这里
  // 作为最终安全闸, 失败就退回 A* 原始路径.
  if (m_has_global_costmap_ && pathCollides(path, global_costmap_msg_)) {
    path = raw_path;
    if (pathCollides(path, global_costmap_msg_)) {
      // 不再把路径清空. 新地图/新激光和 A* 规划用的 costmap 可能差一个
      // 更新周期; 清空会让机器人立即停死. 保留 A* 路径, 由下一周期
      // replan 修正, 同时局部 costmap/安全层负责实时避障.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Raw A* path intersects updated costmap; keeping path for replan");
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Smoothed path collides; using raw A* path");
    }
  }
}

bool NavigationNode::adjustGoalToFree(core::Pose2D & goal) const
{
  if (!m_has_global_costmap_ || global_costmap_msg_.data.empty()) return false;
  const auto & cm = global_costmap_msg_;
  const double res = cm.info.resolution;
  const int w = static_cast<int>(cm.info.width);
  const int h = static_cast<int>(cm.info.height);
  auto costAt = [&](int mx, int my) -> int {
    if (mx < 0 || mx >= w || my < 0 || my >= h) return 100;
    return cm.data[static_cast<size_t>(my) * w + mx];
  };
  const int cx = static_cast<int>(std::floor((goal.x - cm.info.origin.position.x) / res));
  const int cy = static_cast<int>(std::floor((goal.y - cm.info.origin.position.y) / res));
  if (costAt(cx, cy) < 100) return true;

  // 前沿目标可能落在未知/激光点形成的紫色格上. 在 0.3m 内统一按
  // costmap 代价选最低的格子; 同代价再选最近, 不再区分 0/蓝两种情况.
  const int max_r = std::max(1, static_cast<int>(std::ceil(0.30 / res)));
  int best_cost = 100;
  double best_d2 = std::numeric_limits<double>::max();
  int best_dx = 0;
  int best_dy = 0;
  for (int dy = -max_r; dy <= max_r; ++dy) {
    for (int dx = -max_r; dx <= max_r; ++dx) {
      const int c = costAt(cx + dx, cy + dy);
      if (c >= 100) continue;
      const double d2 = static_cast<double>(dx * dx + dy * dy);
      if (c < best_cost || (c == best_cost && d2 < best_d2)) {
        best_cost = c;
        best_d2 = d2;
        best_dx = dx;
        best_dy = dy;
      }
    }
  }
  if (best_cost >= 100) return false;
  goal.x = cm.info.origin.position.x + (cx + best_dx + 0.5) * res;
  goal.y = cm.info.origin.position.y + (cy + best_dy + 0.5) * res;
  RCLCPP_WARN(get_logger(),
    "Goal in obstacle; nudged to nearby lowest-cost cell (%.2f, %.2f), cost=%d",
    goal.x, goal.y, best_cost);
  return true;
}

bool NavigationNode::pathCollides(const nav_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & costmap) const
{
  if (path.poses.empty() || costmap.data.empty()) return false;
  const double res = costmap.info.resolution;
  const double ox = costmap.info.origin.position.x;
  const double oy = costmap.info.origin.position.y;
  const auto & start = path.poses.front().pose.position;
  auto lethal = [&](double wx, double wy) -> bool {
    // 机器人起点可能因贴墙/地图抖动落在 100 内; A* 会清出起点小区域.
    // 这里跳过起点近旁的检查, 否则会把刚规划好的路径误清空, 机器人停死.
    if (std::hypot(wx - start.x, wy - start.y) < 0.5) return false;
    const int mx = static_cast<int>(std::floor((wx - ox) / res));
    const int my = static_cast<int>(std::floor((wy - oy) / res));
    if (mx < 0 || mx >= static_cast<int>(costmap.info.width) ||
        my < 0 || my >= static_cast<int>(costmap.info.height)) return true;
    const size_t idx = static_cast<size_t>(my) * costmap.info.width +
                       static_cast<size_t>(mx);
    return costmap.data[idx] >= 100;
  };
  if (path.poses.size() == 1) {
    return lethal(path.poses.front().pose.position.x,
                  path.poses.front().pose.position.y);
  }
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i-1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double dist = std::hypot(b.x - a.x, b.y - a.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / (res * 0.5))));
    for (int s = 0; s <= steps; ++s) {
      const double t = static_cast<double>(s) / steps;
      if (lethal(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) return true;
    }
  }
  return false;
}

void NavigationNode::publishPath(const nav_msgs::msg::Path & path)
{
  path_pub_->publish(path);
}

void NavigationNode::stopNavigation()
{
  is_navigating_ = false;
  is_paused_ = false;
  current_state_ = NavigationState::IDLE;

  // 发送零速度命令 (经过安全层和 last_published_cmd_ 记录)
  geometry_msgs::msg::Twist stop_cmd;
  publishVelocityCommand(stop_cmd);

  current_path_.poses.clear();
  m_has_effective_goal_ = false;
  m_need_initial_rotate = false;
  recovery_active_ = false;
  stall_cycles_ = 0;
  motion_stall_cycles_ = 0;
  motion_ref_valid_ = false;
  safety_stop_active_ = false;

  if (controller_strategy_) controller_strategy_->reset();
  if (path_pub_) path_pub_->publish(current_path_);
  if (global_path_pub_) global_path_pub_->publish(current_path_);

  RCLCPP_INFO(get_logger(), "Navigation stopped");
}

void NavigationNode::pauseNavigation()
{
  is_paused_ = true;
  current_state_ = NavigationState::IDLE;

  // 发送零速度命令
  geometry_msgs::msg::Twist stop_cmd;
  publishVelocityCommand(stop_cmd);
  motion_ref_valid_ = false;

  RCLCPP_INFO(get_logger(), "Navigation paused");
}

void NavigationNode::resumeNavigation()
{
  if (is_navigating_ && is_paused_) {
    is_paused_ = false;
    current_state_ = NavigationState::CONTROLLING;
    RCLCPP_INFO(get_logger(), "Navigation resumed");
  }
}

} // namespace navigation
} // namespace rosiwit_navigation