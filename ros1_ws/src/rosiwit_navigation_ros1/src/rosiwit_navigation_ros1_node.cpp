// ============================================================================
// rosiwit_navigation_ros1_node.cpp
// rosiwit_navigation 的 ROS1 接口层: 仅做 ROS1 <-> 算法核心(NavigationFactory)
// 的桥接. 算法核心完全 ROS 无关, 与 ROS2 节点 (ros_interface/navigation_node)
// 共享同一份代码 (源码由 ../../../src/rosiwit_navigation 内联构建).
//
// 两套入口共用同一控制循环:
//   * 原生: /goal_pose, /nav_stop, /cmd_vel ...
//   * robot_movebase 兼容: /RobotNaviCmdAction + /movebase_cmd_vel 等旧话题
//     (报文解析在 movebase_protocol, ROS 收发在 movebase_interface;
//      运行状态机在算法层 core::ActionTask, 不在本文件里做行为决策).
// ============================================================================
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <fstream>
#include <sstream>
#include <string>
#include <future>
#include <utility>
#include <vector>

#include <boost/bind.hpp>

#include "rosiwit_navigation/nav_core/navigation_factory.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"
#include "rosiwit_navigation/nav_core/safety_monitor.hpp"
#include "rosiwit_navigation/nav_core/obstacle_tracker.hpp"
#include "rosiwit_navigation/nav_core/task_state_machine.hpp"
#include "rosiwit_navigation/nav_core/voxel_layer.hpp"
#include "rosiwit_navigation/nav_core/action_task.hpp"
#include "rosiwit_navigation/nav_core/coverage_grid.hpp"
#include "path_smoother.hpp"
#include "movebase_protocol.h"
#include "movebase_interface.h"

// 算法核心命名空间 (ROS2 薄层同样使用 rosiwit_navigation 前缀).
using namespace rosiwit_navigation;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{

double yawFromQuat(const geometry_msgs::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::Quaternion quatFromYaw(double yaw)
{
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double normalizeAngle(double a)
{
  while (a > M_PI) { a -= 2.0 * M_PI; }
  while (a < -M_PI) { a += 2.0 * M_PI; }
  return a;
}

std::vector<std::pair<double, double>> toPolygon(const std::vector<double> & flat)
{
  std::vector<std::pair<double, double>> polygon;
  for (size_t i = 0; i + 1 < flat.size(); i += 2) {
    polygon.emplace_back(flat[i], flat[i + 1]);
  }
  return polygon;
}

}  // namespace

class RosiwitNavigationRos1Node
{
public:
  RosiwitNavigationRos1Node(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_), is_navigating_(false), is_paused_(false)
  {
    loadParams();
    setupCore();
    setupRos();
    ROS_INFO("[rosiwit_navigation_ros1] node started (planner=%s, controller=%s)",
             planner_name_.c_str(), controller_name_.c_str());
  }

private:
  // ---- 参数 ----
  void loadParams()
  {
    pnh_.param<double>("controller_frequency", controller_frequency_, 20.0);
    pnh_.param<double>("planner_frequency", planner_frequency_, 1.0);
    pnh_.param<double>("max_velocity_x", max_velocity_x_, 0.5);
    pnh_.param<double>("min_velocity_x", min_velocity_x_, -0.5);
    pnh_.param<double>("max_velocity_theta", max_velocity_theta_, 1.0);
    pnh_.param<double>("goal_tolerance_xy", goal_tolerance_xy_, 0.1);
    pnh_.param<double>("goal_tolerance_yaw", goal_tolerance_yaw_, 0.1);
    pnh_.param<double>("lookahead_distance", lookahead_distance_, 0.6);
    pnh_.param<double>("min_lookahead_distance", min_lookahead_distance_, 0.3);
    pnh_.param<double>("max_lookahead_distance", max_lookahead_distance_, 0.9);
    pnh_.param<double>("inflation_radius", inflation_radius_, 0.5);
    pnh_.param<double>("robot_radius", robot_radius_, 0.21);
    pnh_.param<int>("planner_max_iterations", planner_max_iterations_, 200000);
    pnh_.param<double>("planner_timeout", planner_timeout_, 2.0);
    pnh_.param<double>("max_accel_x", max_accel_x_, 0.4);
    pnh_.param<double>("max_accel_theta", max_accel_theta_, 0.8);

    pnh_.param<std::string>("global_frame", global_frame_, "map");
    pnh_.param<std::string>("robot_frame", robot_frame_, "base_link");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("planner_name", planner_name_, "astar");
    pnh_.param<std::string>("controller_name", controller_name_, "pure_pursuit");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    // RViz "2D Nav Goal" 默认发 /move_base_simple/goal (不是 /goal_pose).
    pnh_.param<std::string>("rviz_goal_topic", rviz_goal_topic_,
                            "/move_base_simple/goal");
    pnh_.param<bool>("rviz_goal_align_yaw", rviz_goal_align_yaw_, true);

    pnh_.param<bool>("safety_enabled", safety_enabled_, true);
    pnh_.param<double>("safety_self_filter_radius", safety_self_filter_radius_, 0.25);
    pnh_.param<bool>("safety_fail_safe_on_timeout", safety_fail_safe_on_timeout_, true);
    pnh_.param<double>("safety_source_timeout", safety_source_timeout_, 0.5);
    pnh_.param<double>("safety_slowdown_ratio", safety_slowdown_ratio_, 0.5);
    pnh_.param<bool>("safety_approach_enabled", safety_approach_enabled_, true);
    pnh_.param<double>("safety_time_before_collision", safety_time_before_collision_, 2.0);
    std::vector<double> stop_poly, slow_poly, footprint;
    const std::vector<double> default_stop{-0.55, -0.40, 0.55, -0.40, 0.55, 0.40, -0.55, 0.40};
    const std::vector<double> default_slow{-0.75, -0.55, 0.85, -0.55, 0.85, 0.55, -0.75, 0.55};
    const std::vector<double> default_footprint{-0.25, -0.22, 0.25, -0.22, 0.25, 0.22, -0.25, 0.22};
    pnh_.param<std::vector<double>>("safety_stop_polygon", stop_poly, default_stop);
    pnh_.param<std::vector<double>>("safety_slowdown_polygon", slow_poly, default_slow);
    pnh_.param<std::vector<double>>("safety_footprint", footprint, default_footprint);

    core::SafetyMonitorConfig safety_cfg;
    safety_cfg.enabled = safety_enabled_;
    safety_cfg.stop_polygon = toPolygon(stop_poly);
    safety_cfg.slowdown_polygon = toPolygon(slow_poly);
    safety_cfg.slowdown_ratio = safety_slowdown_ratio_;
    safety_cfg.approach_enabled = safety_approach_enabled_;
    safety_cfg.time_before_collision = safety_time_before_collision_;
    safety_cfg.footprint = toPolygon(footprint);
    safety_monitor_.configure(safety_cfg);

    pnh_.param<bool>("tracking_enabled", tracking_enabled_, true);
    pnh_.param<double>("tracking_horizon", tracking_horizon_, 0.5);
    pnh_.param<double>("tracking_cluster_distance", tracking_cluster_distance_, 0.35);
    pnh_.param<int>("tracking_confirm_hits", tracking_confirm_hits_, 3);
    core::ObstacleTrackerConfig tracking_cfg;
    tracking_cfg.cluster_distance = tracking_cluster_distance_;
    tracking_cfg.confirm_hits = tracking_confirm_hits_;
    tracker_.setConfig(tracking_cfg);

    pnh_.param<bool>("voxel_enabled", voxel_enabled_, true);
    pnh_.param<std::string>("voxel_cloud_topic", voxel_cloud_topic_, "/world_cloud");
    pnh_.param<double>("voxel_resolution", voxel_resolution_, 0.10);
    pnh_.param<double>("voxel_min_height", voxel_min_height_, -0.20);
    pnh_.param<double>("voxel_max_height", voxel_max_height_, 2.00);
    pnh_.param<double>("voxel_decay", voxel_decay_, 2.0);
    pnh_.param<double>("voxel_inflation", voxel_inflation_, 0.15);
    core::VoxelLayerConfig voxel_cfg;
    voxel_cfg.resolution = voxel_resolution_;
    voxel_cfg.min_height = voxel_min_height_;
    voxel_cfg.max_height = voxel_max_height_;
    voxel_cfg.decay_time = voxel_decay_;
    voxel_cfg.inflation = voxel_inflation_;
    voxel_layer_.setConfig(voxel_cfg);

    // ---- robot_movebase 兼容层 ----
    pnh_.param<bool>("movebase_compat_enabled", movebase_compat_enabled_, true);
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    pnh_.param<double>("movebase_default_speed", movebase_default_speed_, 0.0);
    pnh_.param<double>("movebase_back_speed", movebase_back_speed_, 0.1);
    pnh_.param<double>("movebase_rotate_speed", movebase_rotate_speed_, 0.6);
    pnh_.param<double>("movebase_stuck_timeout", movebase_stuck_timeout_, 60.0);
    pnh_.param<double>("movebase_goal_yaw_tolerance", movebase_goal_yaw_tolerance_,
                       goal_tolerance_yaw_);
    pnh_.param<double>("movebase_rotate_tolerance", movebase_rotate_tolerance_, 0.05);
    std::vector<double> movebase_footprint;
    pnh_.param<std::vector<double>>("movebase_robot_footprint", movebase_footprint, footprint);
    movebase_footprint_.clear();
    for (size_t i = 0; i + 1 < movebase_footprint.size(); i += 2) {
      geometry_msgs::Point point;
      point.x = movebase_footprint[i];
      point.y = movebase_footprint[i + 1];
      point.z = 0.0;
      movebase_footprint_.push_back(point);
    }
    if (movebase_default_speed_ <= 0.0) { movebase_default_speed_ = max_velocity_x_; }

    // ---- 动态覆盖清洁 (navi_cmd=20) ----
    pnh_.param<double>("coverage_sweep_width", coverage_sweep_width_, 0.5);
    pnh_.param<double>("coverage_max_speed", coverage_max_speed_, 0.4);
    pnh_.param<double>("coverage_resolution", coverage_resolution_, 0.05);
    pnh_.param<double>("coverage_replan_period", coverage_replan_period_, 10.0);
    pnh_.param<double>("coverage_horizon", coverage_horizon_, 2.5);
    pnh_.param<double>("coverage_retry_timeout", coverage_retry_timeout_, 10.0);
    pnh_.param<double>("coverage_complete_rate", coverage_complete_rate_, 0.995);
    pnh_.param<double>("coverage_blocked_inflation", coverage_blocked_inflation_, 0.10);
    pnh_.param<double>("coverage_plan_inflation", coverage_plan_inflation_, 0.0);
    pnh_.param<double>("coverage_plan_robot_radius", coverage_plan_robot_radius_, 0.0);
    pnh_.param<std::string>("coverage_controller_name", coverage_controller_name_,
                            "pure_pursuit");
    pnh_.param<std::string>("coverage_report_path", coverage_report_path_, "");

    // 长方形车体的 footprint-aware 控制: 控制器按 footprint 采样点查 raw costmap.
    pnh_.param<bool>("footprint_aware_control", footprint_aware_control_, false);
    pnh_.param<double>("footprint_sample_spacing", footprint_sample_spacing_, 0.05);
    configured_footprint_.clear();
    controller_footprint_.clear();  // 实际是否启用由 setupCore 按控制器能力决定
    if (footprint_aware_control_) {
      configured_footprint_ = toPolygon(footprint);
    }

    core::ActionTaskConfig task_cfg;
    task_cfg.goal_yaw_tolerance = movebase_goal_yaw_tolerance_;
    task_cfg.rotate_tolerance = movebase_rotate_tolerance_;
    task_cfg.stuck_timeout = movebase_stuck_timeout_;
    task_cfg.progress_distance = 0.25;
    task_cfg.progress_yaw = 0.15;
    task_cfg.back_speed = movebase_back_speed_;
    task_cfg.rotate_speed = movebase_rotate_speed_;
    action_task_.configure(task_cfg);

    // 实车参数必须能从启动日志一眼核对 (否则很容易又跑成仿真值).
    if (!movebase_footprint_.empty()) {
      double min_x = std::numeric_limits<double>::max();
      double max_x = std::numeric_limits<double>::lowest();
      double min_y = std::numeric_limits<double>::max();
      double max_y = std::numeric_limits<double>::lowest();
      for (const auto & point : movebase_footprint_) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
      }
      ROS_INFO("[rosiwit_navigation_ros1] footprint(%zu pts) x[%.2f,%.2f] y[%.2f,%.2f] "
               "robot_radius=%.2f inflation_radius=%.2f safety_footprint=%zu pts "
               "footprint_aware=%d configured_pts=%zu",
               movebase_footprint_.size(), min_x, max_x, min_y, max_y,
               robot_radius_, inflation_radius_, footprint.size() / 2,
               static_cast<int>(footprint_aware_control_), configured_footprint_.size());
      if (max_x > 0.55 + 0.01 || max_y > 0.34 || min_x < -0.26) {
        ROS_WARN("[rosiwit_navigation_ros1] footprint 与实车 (x[-0.25,0.55] y[-0.33,0.33]) "
                 "不一致, 请确认不是仿真参数");
      }
    }
  }

  // ---- 核心策略 (来自算法核心, ROS 无关) ----
  void setupCore()
  {
    planner_strategy_ = nav_core::NavigationFactory::createPlanner(planner_name_);
    if (planner_strategy_) {
      core::PlannerConfig cfg;
      cfg.name = planner_name_;
      cfg.resolution = 0.05;
      cfg.max_iterations = planner_max_iterations_;
      cfg.timeout = planner_timeout_;
      planner_strategy_->initialize(cfg);
      planner_strategy_->setInflationRadius(inflation_radius_);
      planner_strategy_->setRobotRadius(robot_radius_);
    } else {
      ROS_WARN("[rosiwit_navigation_ros1] planner '%s' not found", planner_name_.c_str());
    }

    // 异步规划实例: 与主 planner 分开，后台 plan() 不会和 updateCostmap 的
    // setCostmap 竞争；控制/安全定时器在规划期间继续运行。
    async_planner_ = nav_core::NavigationFactory::createPlanner(planner_name_);
    if (async_planner_) {
      core::PlannerConfig acfg;
      acfg.name = planner_name_;
      acfg.resolution = 0.05;
      acfg.max_iterations = planner_max_iterations_;
      acfg.timeout = planner_timeout_;
      async_planner_->initialize(acfg);
      async_planner_->setInflationRadius(inflation_radius_);
      async_planner_->setRobotRadius(robot_radius_);
    }

    controller_strategy_ = nav_core::NavigationFactory::createController(controller_name_);
    // footprint-aware 只对真正实现了 footprint 代价的控制器生效;
    // 否则必须保留局部 costmap 的圆形膨胀, 不能接未膨胀地图.
    controller_footprint_.clear();
    if (footprint_aware_control_ && controller_strategy_ &&
        controller_strategy_->supportsFootprint()) {
      controller_footprint_ = configured_footprint_;
    }
    // 覆盖模式单独一个控制器: 覆盖路径是长直线/行段, pure_pursuit 比
    // MPPI 更稳; 正常导航仍用 controller_name 指定的 MPPI.
    const std::string cov_ctrl = coverage_controller_name_.empty() ?
      controller_name_ : coverage_controller_name_;
    coverage_controller_ = nav_core::NavigationFactory::createController(cov_ctrl);
    if (coverage_controller_) {
      core::ControllerConfig ccfg;
      ccfg.lookahead_distance = lookahead_distance_;
      ccfg.xy_goal_tolerance = 0.15;
      ccfg.slow_down_distance = 1.0;
      ccfg.lookahead_gain = 1.0;
      ccfg.max_lookahead_distance = max_lookahead_distance_;
      ccfg.min_lookahead_distance = min_lookahead_distance_;
      ccfg.max_linear_velocity = std::max(0.1, coverage_max_speed_);
      ccfg.max_angular_velocity = max_velocity_theta_;
      ccfg.max_accel_x = max_accel_x_;
      ccfg.max_accel_theta = max_accel_theta_;
      ccfg.kinematics.max_velocity_x = max_velocity_x_;
      ccfg.kinematics.max_velocity_theta = max_velocity_theta_;
      coverage_controller_->initialize(ccfg);
    } else {
      ROS_WARN("[coverage] controller '%s' not found", cov_ctrl.c_str());
    }

    ROS_INFO("[rosiwit_navigation_ros1] footprint_aware: requested=%d active=%d controller=%s",
             static_cast<int>(footprint_aware_control_),
             static_cast<int>(!controller_footprint_.empty()),
             controller_name_.c_str());
    if (controller_strategy_) {
      core::ControllerConfig cfg;
      cfg.lookahead_distance = lookahead_distance_;
      cfg.xy_goal_tolerance = goal_tolerance_xy_;
      cfg.slow_down_distance = 1.5;
      cfg.lookahead_gain = 1.0;
      cfg.max_lookahead_distance = max_lookahead_distance_;
      cfg.min_lookahead_distance = min_lookahead_distance_;
      cfg.max_linear_velocity = max_velocity_x_;
      cfg.max_angular_velocity = max_velocity_theta_;
      cfg.max_accel_x = max_accel_x_;
      cfg.max_accel_theta = max_accel_theta_;
      cfg.kinematics.max_velocity_x = max_velocity_x_;
      cfg.kinematics.max_velocity_theta = max_velocity_theta_;
      cfg.footprint = controller_footprint_;
      cfg.footprint_sample_spacing = footprint_sample_spacing_;
      controller_strategy_->initialize(cfg);
    } else {
      ROS_WARN("[rosiwit_navigation_ros1] controller '%s' not found", controller_name_.c_str());
    }
  }

  void setupRos()
  {
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1);
    global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);
    global_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/global_costmap", 1, true);
    local_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 1, true);
    goal_status_pub_ = nh_.advertise<std_msgs::String>("/goal_status", 1, true);
    diagnostics_pub_ = nh_.advertise<std_msgs::String>("/nav_diagnostics", 1);
    tracked_obs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/tracked_obstacles", 1);

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &RosiwitNavigationRos1Node::odomCallback, this);
    map_sub_ = nh_.subscribe("/map", 1, &RosiwitNavigationRos1Node::mapCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &RosiwitNavigationRos1Node::scanCallback, this);
    if (voxel_enabled_ && !voxel_cloud_topic_.empty()) {
      voxel_sub_ = nh_.subscribe(voxel_cloud_topic_, 1,
                                 &RosiwitNavigationRos1Node::voxelCallback, this);
    }
    goal_sub_ = nh_.subscribe("/goal_pose", 1, &RosiwitNavigationRos1Node::goalCallback, this);
    if (!rviz_goal_topic_.empty() && rviz_goal_topic_ != "/goal_pose") {
      rviz_goal_sub_ = nh_.subscribe(
        rviz_goal_topic_, 1, &RosiwitNavigationRos1Node::rvizGoalCallback, this);
    }
    nav_stop_srv_ = nh_.advertiseService(
      "/nav_stop", &RosiwitNavigationRos1Node::navStopCallback, this);

    control_timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(1.0, controller_frequency_)),
      &RosiwitNavigationRos1Node::controlLoop, this);
    local_costmap_timer_ = nh_.createTimer(
      ros::Duration(0.1), &RosiwitNavigationRos1Node::publishLocalCostmap, this);
    replanning_timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(0.1, planner_frequency_)),
      &RosiwitNavigationRos1Node::replanningCheck, this);
    diagnostics_timer_ = nh_.createTimer(
      ros::Duration(1.0), &RosiwitNavigationRos1Node::diagnosticsCallback, this);

    // robot_movebase 兼容层: action + 旧话题. 未启用时全部是空操作.
    mb_.reset(new movebase::Interface(nh_, pnh_));
    if (mb_->enabled()) {
      mb_->configure(
        boost::bind(&RosiwitNavigationRos1Node::movebaseGoalCallback, this, _1),
        boost::bind(&RosiwitNavigationRos1Node::movebaseCancelCallback, this));
    }
  }

  // ---- 回调 ----
  void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
  {
    last_odom_stamp_ = ros::Time::now();
    current_velocity_ = msg->twist.twist;
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
  {
    current_map_ = msg;
    if (coverage_active_ && coverage_grid_.ready()) {
      coverage_grid_.updateStaticObstacles(msg->data);
    }
    last_global_costmap_update_ = ros::Time(0);
    updateCostmap();
    ROS_INFO("[rosiwit_navigation_ros1] map received: %d x %d, res=%.3f",
             static_cast<int>(msg->info.width), static_cast<int>(msg->info.height),
             msg->info.resolution);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr & msg)
  {
    last_scan_stamp_ = ros::Time::now();
    geometry_msgs::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id,
                                      msg->header.stamp, ros::Duration(0.1));
    } catch (tf2::TransformException &) {
      return;
    }

    const double tyaw = yawFromQuat(tf.transform.rotation);
    const double tx = tf.transform.translation.x;
    const double ty = tf.transform.translation.y;
    const double cy = std::cos(tyaw);
    const double sy = std::sin(tyaw);

    laser_points_.clear();
    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
      const float r = msg->ranges[i];
      if (r < msg->range_min || r > msg->range_max) { continue; }
      const double lx = r * std::cos(angle);
      const double ly = r * std::sin(angle);
      geometry_msgs::Point p;
      p.x = tx + lx * cy - ly * sy;
      p.y = ty + lx * sy + ly * cy;
      p.z = 0.0;
      laser_points_.push_back(p);
      if (tracking_enabled_) {
        core::Point2D tp; tp.x = p.x; tp.y = p.y;
        tracker_points_.push_back(tp);
      }
    }
    if (tracking_enabled_) {
      tracker_.update(tracker_points_, msg->header.stamp.toSec());
      tracker_points_.clear();
      publishTrackedObstacles();
    }
    updateCostmap();
  }

  void voxelCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
  {
    if (!voxel_enabled_) return;
    bool has_xyz = false;
    for (const auto & f : msg->fields) {
      if (f.name == "x") has_xyz = true;
    }
    if (!has_xyz) return;

    double tx = 0.0, ty = 0.0, tz = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      try {
        const auto tf = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id,
                                                  msg->header.stamp, ros::Duration(0.05));
        tx = tf.transform.translation.x;
        ty = tf.transform.translation.y;
        tz = tf.transform.translation.z;
        qx = tf.transform.rotation.x; qy = tf.transform.rotation.y;
        qz = tf.transform.rotation.z; qw = tf.transform.rotation.w;
      } catch (tf2::TransformException &) {
        return;
      }
    }
    const double xx = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double xy = 2.0 * (qx * qy - qz * qw);
    const double xz = 2.0 * (qx * qz + qy * qw);
    const double yx = 2.0 * (qx * qy + qz * qw);
    const double yy = 1.0 - 2.0 * (qx * qx + qz * qz);
    const double yz = 2.0 * (qy * qz - qx * qw);
    const double zx = 2.0 * (qx * qz - qy * qw);
    const double zy = 2.0 * (qy * qz + qx * qw);
    const double zz = 1.0 - 2.0 * (qx * qx + qy * qy);

    std::vector<core::Point3D> points;
    points.reserve(std::min<size_t>(msg->width * msg->height, 200000));
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const double px = *ix, py = *iy, pz = *iz;
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;
      core::Point3D p;
      p.x = xx * px + xy * py + xz * pz + tx;
      p.y = yx * px + yy * py + yz * pz + ty;
      p.z = zx * px + zy * py + zz * pz + tz;
      points.push_back(p);
    }
    voxel_layer_.update(points, msg->header.stamp.toSec());
  }

  bool transformGoalToMap(geometry_msgs::PoseStamped & goal, const char * source)
  {
    if (goal.header.frame_id.empty()) {
      goal.header.frame_id = map_frame_;
      return true;
    }
    if (goal.header.frame_id == map_frame_) { return true; }
    try {
      tf_buffer_.transform(goal, goal, map_frame_, ros::Duration(0.1));
    } catch (tf2::TransformException & ex) {
      ROS_ERROR("[rosiwit_navigation_ros1] %s frame transform failed: %s",
                source, ex.what());
      return false;
    }
    return true;
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
  {
    // 原生 /goal_pose 与 action 共用一套控制循环; 新目标先让旧 action 结束.
    if (action_active_) {
      ROS_WARN("[rosiwit_navigation_ros1] 收到 /goal_pose, 结束进行中的 movebase action");
      clearActionState();
      if (mb_) { mb_->preempted(); }
    }
    geometry_msgs::PoseStamped goal = *msg;
    if (!transformGoalToMap(goal, "goal_pose")) {
      setGoalStatus("REJECTED_FRAME");
      return;
    }
    ROS_INFO("[rosiwit_navigation_ros1] received goal: (%.2f, %.2f) frame=%s",
             goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());
    goal_pose_ = goal;
    setGoalStatus("ACCEPTED");
    executeNavigation(goal_pose_);
  }

  // RViz "2D Nav Goal": 当作一次原生任务交给 core::ActionTask,
  // 到位后按 RViz 箭头方向对齐 (rviz_goal_align_yaw). 没有 action handle,
  // 所以不会回 action result, 但会走同一套进度/卡死/末端航向逻辑.
  void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
  {
    if (action_active_) {
      ROS_WARN("[rosiwit_navigation_ros1] 收到 RViz goal, 结束进行中的 movebase action");
      clearActionState();
      if (mb_) { mb_->preempted(); }
    }
    geometry_msgs::PoseStamped goal = *msg;
    if (!transformGoalToMap(goal, "rviz_goal")) {
      setGoalStatus("REJECTED_FRAME");
      return;
    }
    if (!updateCurrentPose()) {
      setGoalStatus("FAILED_POSE");
      return;
    }

    core::ActionTaskCommand task_command;
    task_command.kind = core::ActionKind::NAVIGATE;
    task_command.final_rotate = rviz_goal_align_yaw_;
    task_command.final_yaw = yawFromQuat(goal.pose.orientation);

    action_active_ = true;
    action_goal_ = movebase::Goal();
    action_traveled_.clear();
    action_traveled_.push_back(current_pose_);
    action_speed_ = movebase_default_speed_;
    if (controller_strategy_) {
      controller_strategy_->setVelocityLimit(action_speed_, max_velocity_theta_);
    }
    if (!action_task_.start(task_command, toCorePose(current_pose_),
                            ros::Time::now().toSec())) {
      clearActionState();
      setGoalStatus("FAILED_PLAN");
      return;
    }

    goal_pose_ = goal;
    setGoalStatus("ACCEPTED");
    ROS_INFO("[rosiwit_navigation_ros1] RViz goal: (%.2f, %.2f, yaw=%.2f) align_yaw=%d",
             goal.pose.position.x, goal.pose.position.y,
             task_command.final_yaw, static_cast<int>(rviz_goal_align_yaw_));
    startPlanning(goal_pose_, /*allow_initial_rotate=*/true, /*restart=*/true);
  }

  bool navStopCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response & res)
  {
    stopNavigation();
    if (action_active_) {
      clearActionState();
      if (mb_) { mb_->preempted(); }
    }
    setGoalStatus("STOPPED");
    res.success = true;
    res.message = "navigation stopped";
    return true;
  }

  // ==========================================================================
  // robot_movebase action 兼容层 (薄)
  //   职责边界:
  //     接口层 (这里): goal 字段 -> 核心命令的映射、ROS 输出、action 生命周期;
  //     算法层 (core::ActionTask): 运行状态、进度、到位判定、末端航向、卡死看门狗.
  //   语义: 成功与失败都 setSucceeded, 失败只体现在 tracking_path_result
  //         (照抄旧实现 src/RobotMovebaseNode.cpp: ExecuteCb()).
  // ==========================================================================
  void publishCommand(const geometry_msgs::Twist & cmd)
  {
    cmd_vel_pub_.publish(cmd);
    if (mb_) { mb_->publishCmdVel(cmd); }
  }

  void movebaseCancelCallback()
  {
    ROS_INFO("[movebase_iface] 取消当前导航");
    stopNavigation();
    clearActionState();
  }

  core::ActionTaskCommand toTaskCommand(const movebase::Decoded & decoded,
                                        const core::Pose2D & target) const
  {
    core::ActionTaskCommand command;
    switch (decoded.command) {
      case movebase::Command::SINGLE_POINT:
        command.kind = core::ActionKind::NAVIGATE;
        command.final_rotate = true;   // 单点导航到位后对齐目标航向
        command.final_yaw = target.theta;
        break;
      case movebase::Command::TWO_POINTS:
        command.kind = core::ActionKind::NAVIGATE;
        command.final_rotate = false;  // 两点导航只保证到达目标点
        command.final_yaw = target.theta;
        break;
      case movebase::Command::FOLLOW_PATH:
        command.kind = core::ActionKind::FOLLOW_PATH;
        break;
      case movebase::Command::ROTATE_180:
        command.kind = core::ActionKind::ROTATE;
        command.rotate_angle = M_PI;
        break;
      case movebase::Command::BACK_OFF:
        command.kind = core::ActionKind::BACK_OFF;
        command.back_distance = decoded.back_distance;
        break;
      default:
        command.kind = core::ActionKind::IDLE;
        break;
    }
    return command;
  }

  void movebaseGoalCallback(const movebase::Goal & goal)
  {
    ROS_INFO("[movebase_iface] 收到 navi_cmd=%d vel=%.2f poses=%zu file='%s'",
             goal.navi_cmd, goal.robot_vel, goal.tracking_path.size(),
             goal.file_path.c_str());

    // actionlib 在收到新 goal 前会先触发 preempt; 这里再清一次, 保证状态干净.
    clearActionState();
    action_traveled_.clear();
    action_goal_ = goal;
    action_cmd_ = movebase::decodeGoal(goal);

    if (!action_cmd_.valid || !action_cmd_.supported) {
      ROS_WARN("[movebase_iface] navi_cmd=%d 当前核心不支持或参数非法, 回 result=%d",
               goal.navi_cmd, action_cmd_.reject_result);
      finishAction(action_cmd_.reject_result);
      return;
    }
    if (action_cmd_.command == movebase::Command::CANCEL) {
      stopNavigation();
      finishAction(movebase::RESULT_SUCCESS);
      return;
    }

    if (!updateCurrentPose()) {
      ROS_ERROR("[movebase_iface] 获取坐标失败");
      failAction(movebase::RESULT_POSE_ERROR, "FAILED_POSE");
      return;
    }
    if (!current_map_) {
      ROS_ERROR("[movebase_iface] 全局地图未就绪");
      failAction(movebase::RESULT_GLOBAL_MAP_ERROR, "FAILED_MAP");
      return;
    }

    action_active_ = true;
    action_traveled_.push_back(current_pose_);
    action_speed_ = action_cmd_.speed > 0.01 ? action_cmd_.speed : movebase_default_speed_;
    if (action_speed_ > max_velocity_x_) {
      ROS_WARN("[movebase_iface] 请求速度 %.2f 超过上限, 收敛到 %.2f",
               action_speed_, max_velocity_x_);
      action_speed_ = max_velocity_x_;
    }
    if (controller_strategy_) {
      controller_strategy_->setVelocityLimit(action_speed_, max_velocity_theta_);
    }

    switch (action_cmd_.command) {
      case movebase::Command::SINGLE_POINT:
      case movebase::Command::TWO_POINTS:
        startActionNavigation();
        break;
      case movebase::Command::FOLLOW_PATH:
        startActionFollowPath();
        break;
      case movebase::Command::ROTATE_180:
        startActionRotate();
        break;
      case movebase::Command::BACK_OFF:
        startActionBackOff();
        break;
      case movebase::Command::COVERAGE:
        startActionCoverage();
        break;
      default:
        failAction(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
        break;
    }
  }

  void startActionNavigation()
  {
    core::Pose2D target = action_cmd_.target;

    // 单点命令的 file_path 分支: 取 [path_segment_index][path_index].
    if (action_cmd_.use_file) {
      std::vector<std::vector<core::Pose2D>> segments;
      std::string error;
      if (!movebase::loadPathFile(action_goal_.file_path, segments, &error)) {
        ROS_ERROR("[movebase_iface] 加载路径文件失败: %s", error.c_str());
        failAction(movebase::RESULT_FILE_ERROR, "FAILED_FILE");
        return;
      }
      if (segments.empty()) {
        failAction(movebase::RESULT_FILE_ERROR, "FAILED_FILE");
        return;
      }
      const size_t seg = std::min<size_t>(
        static_cast<size_t>(action_cmd_.path_segment_index), segments.size() - 1);
      const std::vector<core::Pose2D> tail =
        movebase::slicePath(segments[seg], action_cmd_.path_index);
      if (tail.empty()) {
        failAction(movebase::RESULT_FILE_ERROR, "FAILED_FILE");
        return;
      }
      target = tail.front();
    }

    const core::Pose2D start = action_cmd_.has_start
      ? action_cmd_.start : toCorePose(current_pose_);
    if (!action_task_.start(toTaskCommand(action_cmd_, target), start,
                            ros::Time::now().toSec())) {
      failAction(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
      return;
    }

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = map_frame_;
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = target.x;
    goal.pose.position.y = target.y;
    goal.pose.position.z = 0.0;
    goal.pose.orientation = quatFromYaw(target.theta);
    goal_pose_ = goal;

    if (!current_path_.poses.empty()) { current_path_.poses.clear(); }
    global_path_pub_.publish(current_path_);

    const core::Pose2D * start_override = action_cmd_.has_start ? &action_cmd_.start : nullptr;
    // 两点导航的起点由调用方给定, 与车头朝向无关, 不做起步原地旋转.
    const bool allow_initial_rotate =
      (action_cmd_.command == movebase::Command::SINGLE_POINT);
    startPlanning(goal_pose_, allow_initial_rotate, /*restart=*/true, start_override);
  }

  void startActionFollowPath()
  {
    if (!controller_strategy_) {
      failAction(movebase::RESULT_LOCAL_MAP_ERROR, "FAILED_CONTROLLER");
      return;
    }

    std::vector<core::Pose2D> path = action_cmd_.path;
    if (action_cmd_.use_file) {
      std::vector<std::vector<core::Pose2D>> segments;
      std::string error;
      if (!movebase::loadPathFile(action_goal_.file_path, segments, &error)) {
        ROS_ERROR("[movebase_iface] 加载路径文件失败: %s", error.c_str());
        failAction(movebase::RESULT_FILE_ERROR, "FAILED_FILE");
        return;
      }
      if (segments.empty()) {
        failAction(movebase::RESULT_FILE_ERROR, "FAILED_FILE");
        return;
      }
      const size_t seg = std::min<size_t>(
        static_cast<size_t>(action_cmd_.path_segment_index), segments.size() - 1);
      path = movebase::slicePath(segments[seg], action_cmd_.path_index);
    }
    if (path.size() < 2) {
      ROS_ERROR("[movebase_iface] 跟踪路径点数不足 (%zu)", path.size());
      failAction(movebase::RESULT_POSE_ERROR, "FAILED_POSE");
      return;
    }

    nav_msgs::Path ros_path;
    ros_path.header.frame_id = map_frame_;
    ros_path.header.stamp = ros::Time::now();
    for (const auto & pose : path) {
      geometry_msgs::PoseStamped stamped;
      stamped.header = ros_path.header;
      stamped.pose.position.x = pose.x;
      stamped.pose.position.y = pose.y;
      stamped.pose.orientation = quatFromYaw(pose.theta);
      ros_path.poses.push_back(stamped);
    }
    current_path_ = ros_path;

    // 与规划结果同样做一次平滑; 地图不可用时跳过.
    if (current_path_.poses.size() >= 3 && current_map_) {
      planners::SmootherConfig smoother_cfg;
      smoother_cfg.max_iterations = 100;
      smoother_cfg.alpha_start = 0.5;
      smoother_cfg.alpha_end = 0.1;
      auto core_map = std::make_shared<core::CostmapGrid>();
      core_map->info.width = current_map_->info.width;
      core_map->info.height = current_map_->info.height;
      core_map->info.resolution = current_map_->info.resolution;
      core_map->info.origin.position.x = current_map_->info.origin.position.x;
      core_map->info.origin.position.y = current_map_->info.origin.position.y;
      core_map->data = current_map_->data;
      current_path_ = toRosPath(
        planners::smoothPath(toCorePath(current_path_), core_map, smoother_cfg),
        map_frame_, ros::Time::now());
    }

    core::ActionTaskCommand task_command;
    task_command.kind = core::ActionKind::FOLLOW_PATH;
    if (!action_task_.start(task_command, toCorePose(current_pose_),
                            ros::Time::now().toSec())) {
      failAction(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
      return;
    }

    controller_strategy_->setPath(toCorePath(current_path_));
    global_path_pub_.publish(current_path_);

    planning_active_ = false;
    is_navigating_ = true;
    is_paused_ = false;
    m_need_initial_rotate_ = false;
    task_sm_.handle(core::TaskEvent::PLAN_OK);
    ROS_INFO("[movebase_iface] 开始跟踪路径: %zu 点, %.2f m",
             current_path_.poses.size(), movebase::pathLength(path));
  }

  void startActionRotate()
  {
    core::ActionTaskCommand task_command;
    task_command.kind = core::ActionKind::ROTATE;
    const core::Pose2D pose = toCorePose(current_pose_);
    if (!action_task_.start(task_command, pose, ros::Time::now().toSec())) {
      failAction(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
      return;
    }
    planning_active_ = false;
    is_navigating_ = true;
    is_paused_ = false;
    m_need_initial_rotate_ = false;
    current_path_.poses.clear();
    global_path_pub_.publish(current_path_);
    ROS_INFO("[movebase_iface] 原地旋转 180°: %.2f -> %.2f",
             pose.theta, action_task_.rotationTarget());
  }

  void startActionBackOff()
  {
    core::ActionTaskCommand task_command;
    task_command.kind = core::ActionKind::BACK_OFF;
    task_command.back_distance = action_cmd_.back_distance;
    if (!action_task_.start(task_command, toCorePose(current_pose_),
                            ros::Time::now().toSec())) {
      failAction(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
      return;
    }
    planning_active_ = false;
    is_navigating_ = true;
    is_paused_ = false;
    m_need_initial_rotate_ = false;
    current_path_.poses.clear();
    global_path_pub_.publish(current_path_);
    ROS_INFO("[movebase_iface] 倒车 %.2f m (速度 %.2f m/s)",
             action_cmd_.back_distance, movebase_back_speed_);
  }

  void clearActionState()
  {
    if (coverage_active_) { stopCoverageInternal(); }
    coverage_state_ = "IDLE";
    last_coverage_rate_ = 0.0;
    last_clean_area_ = 0.0;
    action_active_ = false;
    action_task_.reset();
    if (controller_strategy_) {
      controller_strategy_->setVelocityLimit(max_velocity_x_, max_velocity_theta_);
    }
  }

  core::IControllerStrategy * activeController() const
  {
    if (coverage_active_ && coverage_controller_) { return coverage_controller_.get(); }
    return controller_strategy_.get();
  }

  geometry_msgs::Twist coverageCommand()
  {
    if (!coverage_controller_) { return geometry_msgs::Twist(); }
    return toRosTwist(coverage_controller_->computeVelocityCommand(
      toCorePose(current_pose_), toCoreVelocity(current_velocity_)));
  }

  double actionProgress() const
  {
    if (coverage_active_) {
      return coverage_grid_.ready() ? coverage_grid_.stats().coverageRate() : 0.0;
    }
    if (coverage_state_ == "DONE" || coverage_state_ == "STOPPED") {
      return last_coverage_rate_;
    }
    return action_active_ ? action_task_.progress() : 0.0;
  }

  movebase::Feedback actionFeedback() const
  {
    movebase::Feedback feedback;
    feedback.execution_percent = actionProgress();
    feedback.execution_path_segment_index = action_goal_.path_segment_index;
    if (activeController()) {
      feedback.execution_index =
        static_cast<long>(activeController()->getCurrentWaypointIndex());
    }
    if (action_task_.backing()) {
      // 倒车没有路径索引, 用剩余距离的百分比作为下标语义没有意义, 保持 0.
      feedback.execution_index = 0;
    }
    for (const auto & pose : action_traveled_) {
      feedback.execution_path.emplace_back(
        pose.pose.position.x, pose.pose.position.y,
        yawFromQuat(pose.pose.orientation));
    }
    const size_t index = static_cast<size_t>(std::max<long>(0, feedback.execution_index));
    const size_t limit = std::min(current_path_.poses.size(), index + 200);
    for (size_t i = index; i < limit; ++i) {
      feedback.local_path.emplace_back(
        current_path_.poses[i].pose.position.x,
        current_path_.poses[i].pose.position.y,
        yawFromQuat(current_path_.poses[i].pose.orientation));
    }
    feedback.clean_area = last_clean_area_;
    feedback.navi_state =
      (last_safety_state_ != "PASS" || last_safety_scale_ < 0.999 ||
       (coverage_active_ && coverage_grid_.dynamicCount() > 0)) ? 2 : 1;
    return feedback;
  }

  movebase::Outcome actionOutcome(int result_code) const
  {
    movebase::Outcome outcome;
    outcome.tracking_path_result = result_code;
    outcome.execution_percent = actionProgress();
    outcome.execution_path_segment_index = action_goal_.path_segment_index;
    outcome.clean_area = last_clean_area_;
    if (activeController() && !action_task_.backing()) {
      outcome.execution_index =
        static_cast<long>(activeController()->getCurrentWaypointIndex());
    }
    for (const auto & pose : action_traveled_) {
      outcome.execution_path.emplace_back(
        pose.pose.position.x, pose.pose.position.y,
        yawFromQuat(pose.pose.orientation));
    }
    return outcome;
  }

  void finishAction(int result_code)
  {
    if (mb_ && mb_->hasGoal()) {
      mb_->succeed(actionOutcome(result_code));
    }
    clearActionState();
    stopNavigation();
  }

  void failAction(int result_code, const std::string & status)
  {
    if (action_active_ || (mb_ && mb_->hasGoal())) {
      finishAction(result_code);
    } else {
      stopNavigation();
    }
    setGoalStatus(status);
  }

  // 只做"已走过轨迹"的展示采样; 进度/卡死判定在 core::ActionTask.
  void recordActionTraveled()
  {
    if (!action_active_) { return; }
    if (action_traveled_.empty()) {
      action_traveled_.push_back(current_pose_);
      return;
    }
    const auto & last = action_traveled_.back();
    const double moved = std::hypot(
      current_pose_.pose.position.x - last.pose.position.x,
      current_pose_.pose.position.y - last.pose.position.y);
    if (moved > 0.1) {
      action_traveled_.push_back(current_pose_);
      if (action_traveled_.size() > 5000) { action_traveled_.erase(action_traveled_.begin()); }
    }
  }

  void publishActionOutputs()
  {
    if (!action_active_ || !mb_) { return; }
    mb_->publishFeedback(actionFeedback());
    mb_->publishFootprints(current_pose_, current_velocity_, movebase_footprint_);
    mb_->publishTraveledPath(action_traveled_);

    // 参考点: 当前路径上前视 0.6m 的点 (没路径时用车头前方 0.6m 近似)
    geometry_msgs::PointStamped reference;
    reference.header.frame_id = map_frame_;
    reference.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped lookahead;
    if (findLookaheadPose(0.6, lookahead)) {
      reference.point = lookahead.pose.position;
    } else {
      const double yaw = yawFromQuat(current_pose_.pose.orientation);
      reference.point.x = current_pose_.pose.position.x + 0.6 * std::cos(yaw);
      reference.point.y = current_pose_.pose.position.y + 0.6 * std::sin(yaw);
    }
    mb_->publishLookahead(reference);

    // 预测路径: 有参考路径时取前方一段, 否则按当前速度外推 1s.
    nav_msgs::Path prediction;
    prediction.header.frame_id = map_frame_;
    prediction.header.stamp = ros::Time::now();
    if (!current_path_.poses.empty() && activeController()) {
      const size_t index = std::min(
        activeController()->getCurrentWaypointIndex(), current_path_.poses.size() - 1);
      const size_t limit = std::min(current_path_.poses.size(), index + 50);
      for (size_t i = index; i < limit; ++i) {
        prediction.poses.push_back(current_path_.poses[i]);
      }
    } else {
      const double yaw = yawFromQuat(current_pose_.pose.orientation);
      for (int i = 0; i <= 10; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header = prediction.header;
        const double t = 0.1 * i;
        pose.pose.position.x = current_pose_.pose.position.x +
          current_velocity_.linear.x * t * std::cos(yaw);
        pose.pose.position.y = current_pose_.pose.position.y +
          current_velocity_.linear.x * t * std::sin(yaw);
        pose.pose.orientation = quatFromYaw(yaw + current_velocity_.angular.z * t);
        prediction.poses.push_back(pose);
      }
    }
    mb_->publishPredictionPath(prediction);

    if (action_task_.backing()) {
      nav_msgs::Path back_path;
      back_path.header = prediction.header;
      const double yaw = yawFromQuat(current_pose_.pose.orientation);
      for (int i = 0; i <= 10; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header = back_path.header;
        const double distance = -0.1 * i;  // 10 步 x 0.1m
        pose.pose.position.x = current_pose_.pose.position.x + distance * std::cos(yaw);
        pose.pose.position.y = current_pose_.pose.position.y + distance * std::sin(yaw);
        pose.pose.orientation = current_pose_.pose.orientation;
        back_path.poses.push_back(pose);
      }
      mb_->publishBackOffPath(back_path);
    }
  }

  bool findLookaheadPose(double distance, geometry_msgs::PoseStamped & out) const
  {
    if (!activeController() || current_path_.poses.empty()) { return false; }
    size_t index = std::min(
      activeController()->getCurrentWaypointIndex(), current_path_.poses.size() - 1);
    double accumulated = 0.0;
    for (size_t i = index + 1; i < current_path_.poses.size(); ++i) {
      const auto & prev = current_path_.poses[i - 1].pose.position;
      const auto & cur = current_path_.poses[i].pose.position;
      accumulated += std::hypot(cur.x - prev.x, cur.y - prev.y);
      if (accumulated >= distance) {
        out = current_path_.poses[i];
        return true;
      }
    }
    out = current_path_.poses.back();
    return true;
  }

  // 旋转/倒车的速度指令由算法层给出 primitive, 接口层只负责发出去.
  void publishRotationStep()
  {
    geometry_msgs::Twist cmd;
    cmd.angular.z = action_task_.rotationCommand(toCorePose(current_pose_));
    publishCommand(applySafety(cmd));
    recordActionTraveled();
    publishActionOutputs();
  }

  void publishBackOffStep()
  {
    geometry_msgs::Twist cmd;
    cmd.linear.x = action_task_.backOffCommand();
    publishCommand(applySafety(cmd));
    recordActionTraveled();
    publishActionOutputs();
  }


  // ==========================================================================
  // 动态覆盖清洁 (navi_cmd=20)
  //   tracking_path = 区域多边形; robot_vel = 最大速度 (<=0 用参数)
  //   宽度/重规划/超时等全部走私有参数; 不新增 action/service.
  // ==========================================================================
  void startActionCoverage()
  {
    if (!current_map_) {
      failAction(movebase::RESULT_GLOBAL_MAP_ERROR, "FAILED_MAP");
      return;
    }
    if (action_cmd_.polygon.size() < 3) {
      ROS_ERROR("[coverage] 多边形顶点不足 (%zu)", action_cmd_.polygon.size());
      failAction(movebase::RESULT_POSE_ERROR, "FAILED_COVERAGE_POLYGON");
      return;
    }
    if (!updateCurrentPose()) {
      failAction(movebase::RESULT_POSE_ERROR, "FAILED_POSE");
      return;
    }

    core::CostmapGridInfo info;
    info.width = current_map_->info.width;
    info.height = current_map_->info.height;
    info.resolution = current_map_->info.resolution;
    info.origin.position.x = current_map_->info.origin.position.x;
    info.origin.position.y = current_map_->info.origin.position.y;
    if (std::fabs(coverage_resolution_ - info.resolution) > 1e-3) {
      ROS_WARN("[coverage] coverage_resolution=%.3f 与地图分辨率 %.3f 不一致, 按地图分辨率",
               coverage_resolution_, info.resolution);
    }

    coverage_config_.sweep_width = coverage_sweep_width_;
    coverage_config_.obstacle_dilation = coverage_blocked_inflation_;
    // v1: 允许目标到区域边缘, 由下面的“预测位姿越界即停”守卫防止车体中心跑出.
    coverage_config_.center_inset = 0.0;
    if (!coverage_grid_.initialize(info, action_cmd_.polygon, coverage_config_)) {
      failAction(movebase::RESULT_POSE_ERROR, "FAILED_COVERAGE_AREA");
      return;
    }
    coverage_grid_.updateStaticObstacles(current_map_->data);
    const core::Pose2D pose = toCorePose(current_pose_);
    if (!coverage_grid_.contains(pose)) {
      ROS_ERROR("[coverage] 起点 (%.2f, %.2f) 不在区域内或落在障碍上",
                pose.x, pose.y);
      failAction(movebase::RESULT_PLAN_FAILED, "FAILED_COVERAGE_START");
      return;
    }
    coverage_grid_.resetPose(pose);

    coverage_active_ = true;
    coverage_state_ = "COVERING";
    coverage_last_plan_ = ros::Time(0);
    coverage_last_progress_ = ros::Time::now();
    coverage_last_status_ = ros::Time(0);
    coverage_last_dynamic_ = ros::Time(0);
    coverage_last_covered_count_ = 0;
    coverage_last_progress_pose_ = pose;
    coverage_last_motion_pose_ = pose;
    coverage_last_motion_time_ = ros::Time::now();
    coverage_has_target_ = false;
    last_clean_area_ = 0.0;

    action_speed_ = action_cmd_.speed > 0.01 ? action_cmd_.speed : coverage_max_speed_;
    action_speed_ = std::min(action_speed_, max_velocity_x_);
    if (coverage_controller_) {
      coverage_controller_->setVelocityLimit(action_speed_, max_velocity_theta_);
      coverage_controller_->reset();
    }
    current_path_.poses.clear();
    is_navigating_ = true;
    is_paused_ = false;
    planning_active_ = false;

    ROS_INFO("[coverage] start: %zu vertices, sweep=%.2f m, speed=%.2f m/s, horizon=%.1f m",
             action_cmd_.polygon.size(), coverage_sweep_width_, action_speed_,
             coverage_horizon_);
    publishCoverageOutputs(ros::Time::now(), false);
    coveragePlanTick();
  }

  void coverageControlStep()
  {
    if (!coverage_active_) { return; }
    const ros::Time now = ros::Time::now();
    const core::Pose2D pose = toCorePose(current_pose_);

    coverage_grid_.updateCovered(pose);
    recordActionTraveled();

    if (coverage_last_dynamic_.isZero() ||
        (now - coverage_last_dynamic_).toSec() >= 0.2) {
      std::vector<core::Pose2D> dynamic_points;
      dynamic_points.reserve(laser_points_.size());
      for (const auto & p : laser_points_) {
        if (std::hypot(p.x - pose.x, p.y - pose.y) < safety_self_filter_radius_) {
          continue;  // 过滤自身点
        }
        dynamic_points.emplace_back(p.x, p.y, 0.0);
      }
      coverage_grid_.updateDynamicObstacles(dynamic_points);
      coverage_last_dynamic_ = now;
    }

    const auto stats = coverage_grid_.stats();
    if (stats.covered > coverage_last_covered_count_) {
      coverage_last_covered_count_ = stats.covered;
      coverage_last_progress_ = now;
      coverage_last_progress_pose_ = pose;
    } else if (pose.distanceTo(coverage_last_progress_pose_) > 0.3) {
      coverage_last_progress_ = now;
      coverage_last_progress_pose_ = pose;
    }
    if (pose.distanceTo(coverage_last_motion_pose_) > 0.1) {
      coverage_last_motion_pose_ = pose;
      coverage_last_motion_time_ = now;
    }
    last_clean_area_ = stats.coveredArea(current_map_->info.resolution);

    // 最后几个格子可能永远在边界/盲区里打转; 达到完成阈值就收尾,
    // 把剩余未覆盖标成不可达并写进报告.
    if (stats.coverageRate() >= coverage_complete_rate_ &&
        stats.uncovered > 0) {
      coverage_grid_.markAllUncoveredBlocked();
      finishCoverage();
      return;
    }

    if (coverage_last_status_.isZero() ||
        (now - coverage_last_status_).toSec() >= 1.0) {
      publishCoverageOutputs(now, false);
      coverage_last_status_ = now;
    }

    const bool path_done = current_path_.poses.size() < 2 ||
      (coverage_controller_ && coverage_controller_->isGoalReached(pose));
    // 2s 就重规划会把 MPPI 的加速过程反复打断; 给当前段足够时间走完.
    const bool no_motion = current_path_.poses.size() >= 2 &&
      (now - coverage_last_motion_time_).toSec() > 6.0 &&
      (now - coverage_last_plan_).toSec() > 3.0;
    const bool replan_due = coverage_last_plan_.isZero() ||
      (now - coverage_last_plan_).toSec() >= coverage_replan_period_ ||
      no_motion;
    if (path_done || replan_due) {
      coveragePlanTick();
      if (!coverage_active_) { return; }
    }

    if (!coverage_grid_.hasUncoveredReachable(pose, true)) {
      if (coverage_grid_.hasAnyUncovered()) {
        if ((now - coverage_last_progress_).toSec() > coverage_retry_timeout_) {
          ROS_WARN("[coverage] %.0fs 内没有进展, 把剩余未覆盖标为不可达",
                   coverage_retry_timeout_);
          coverage_grid_.markAllUncoveredBlocked();
          finishCoverage();
        } else {
          coverage_state_ = "WAITING_OBSTACLE";
          publishCommand(applySafety(geometry_msgs::Twist()));
          publishActionOutputs();
        }
        return;
      }
      finishCoverage();
      return;
    }

    if (current_path_.poses.size() >= 2) {
      coverage_state_ = "COVERING";
      geometry_msgs::Twist cmd = applySafety(coverageCommand());
      // 预测一步: 如果车体中心会跑出区域/撞进障碍, 停线速度 (保留转向).
      if (coverage_grid_.contains(pose)) {
        const double dt = 0.1;
        core::Pose2D next(pose.x + cmd.linear.x * std::cos(pose.theta) * dt,
                          pose.y + cmd.linear.x * std::sin(pose.theta) * dt,
                          pose.theta + cmd.angular.z * dt);
        if (!coverage_grid_.contains(next)) {
          cmd.linear.x = 0.0;
          coverage_state_ = "BOUNDARY";
        }
      }
      publishCommand(cmd);
      path_pub_.publish(current_path_);
      global_path_pub_.publish(current_path_);
      publishActionOutputs();
    } else {
      publishCommand(applySafety(geometry_msgs::Twist()));
      publishActionOutputs();
    }
  }

  void coveragePlanTick()
  {
    if (!coverage_active_ || !current_map_ || !coverage_grid_.ready() ||
        !planner_strategy_ || !coverage_controller_) {
      return;
    }
    const ros::Time now = ros::Time::now();
    const core::Pose2D pose = toCorePose(current_pose_);
    std::vector<core::Pose2D> segment;
    if (!coverage_grid_.nextCoverageSegment(pose, coverage_horizon_, segment)) {
      coverage_last_plan_ = now;
      if (coverage_grid_.hasAnyUncovered()) {
        if ((now - coverage_last_progress_).toSec() > coverage_retry_timeout_) {
          coverage_grid_.markAllUncoveredBlocked();
          finishCoverage();
        } else {
          coverage_state_ = "WAITING_OBSTACLE";
          current_path_.poses.clear();
          coverage_controller_->reset();
        }
      } else {
        finishCoverage();
      }
      return;
    }
    coverage_target_ = segment.back();
    coverage_has_target_ = true;

    core::Path path;
    if (pose.distanceTo(segment.front()) > 0.3) {
      core::Costmap costmap;
      buildCoverageCostmap(costmap);
      planner_strategy_->setInflationRadius(coverage_plan_inflation_);
      planner_strategy_->setRobotRadius(coverage_plan_robot_radius_);
      planner_strategy_->setCostmap(costmap);
      const core::Result<core::Path> approach =
        planner_strategy_->plan(pose, segment.front());
      planner_strategy_->setInflationRadius(inflation_radius_);
      planner_strategy_->setRobotRadius(robot_radius_);
      coverage_last_plan_ = now;
      if (!approach.is_ok()) {
        ROS_WARN("[coverage] approach to row (%.2f, %.2f) failed: %s",
                 segment.front().x, segment.front().y,
                 approach.error_message().c_str());
        coverage_has_target_ = false;
        current_path_.poses.clear();
        coverage_controller_->reset();
        coverage_state_ = "REPLAN";
        return;
      }
      path = approach.value();
      for (size_t i = 1; i < segment.size(); ++i) {
        core::PathPoint pt;
        pt.pose = segment[i];
        path.points.push_back(pt);
      }
    } else {
      for (const auto & point : segment) {
        core::PathPoint pt;
        pt.pose = point;
        path.points.push_back(pt);
      }
    }
    coverage_last_plan_ = now;
    if (path.points.size() < 2) {
      current_path_.poses.clear();
      coverage_controller_->reset();
      return;
    }

    current_path_ = toRosPath(path, map_frame_, now);
    coverage_controller_->setPath(path);
    coverage_state_ = "COVERING";
    ROS_INFO_THROTTLE(2.0, "[coverage] row=(%.2f,%.2f)->(%.2f,%.2f) path=%zu pts",
                      segment.front().x, segment.front().y,
                      segment.back().x, segment.back().y, path.points.size());
  }

  void buildCoverageCostmap(core::Costmap & costmap) const
  {
    const auto encoded = coverage_grid_.plannerCostmap(current_map_->data);
    auto grid = std::make_shared<core::CostmapGrid>();
    grid->info.width = current_map_->info.width;
    grid->info.height = current_map_->info.height;
    grid->info.resolution = current_map_->info.resolution;
    grid->info.origin.position.x = current_map_->info.origin.position.x;
    grid->info.origin.position.y = current_map_->info.origin.position.y;
    grid->data.assign(encoded.size(), 0);
    costmap.data.assign(encoded.size(), 0);
    for (size_t i = 0; i < encoded.size(); ++i) {
      if (encoded[i] >= 100) {
        grid->data[i] = 100;
        costmap.data[i] = 255;
      }
    }
    // 允许 A* 从当前格起步 (起点可能落在 center_inset 带里).
    const int sx = static_cast<int>(std::floor(
      (current_pose_.pose.position.x - current_map_->info.origin.position.x) /
      current_map_->info.resolution));
    const int sy = static_cast<int>(std::floor(
      (current_pose_.pose.position.y - current_map_->info.origin.position.y) /
      current_map_->info.resolution));
    if (sx >= 0 && sy >= 0 &&
        sx < static_cast<int>(current_map_->info.width) &&
        sy < static_cast<int>(current_map_->info.height)) {
      const size_t idx = static_cast<size_t>(sy) * current_map_->info.width + sx;
      const int recover_cells = std::max(1, static_cast<int>(std::ceil(
        0.25 / current_map_->info.resolution)));
      for (int dy = -recover_cells; dy <= recover_cells; ++dy) {
        for (int dx = -recover_cells; dx <= recover_cells; ++dx) {
          if (dx * dx + dy * dy > recover_cells * recover_cells) { continue; }
          const int nx = sx + dx;
          const int ny = sy + dy;
          if (nx < 0 || ny < 0 ||
              nx >= static_cast<int>(current_map_->info.width) ||
              ny >= static_cast<int>(current_map_->info.height)) { continue; }
          const size_t nidx = static_cast<size_t>(ny) * current_map_->info.width + nx;
          if (nidx < grid->data.size()) {
            grid->data[nidx] = 0;
            costmap.data[nidx] = 0;
          }
        }
      }
    }
    costmap.grid = grid;
    costmap.width = current_map_->info.width;
    costmap.height = current_map_->info.height;
    costmap.resolution = current_map_->info.resolution;
    costmap.origin_x = current_map_->info.origin.position.x;
    costmap.origin_y = current_map_->info.origin.position.y;
  }

  void publishCoverageOutputs(const ros::Time & stamp, bool force_report)
  {
    if (!mb_ || !coverage_grid_.ready() || !current_map_) { return; }
    const auto stats = coverage_grid_.stats();
    const double res = current_map_->info.resolution;
    last_clean_area_ = stats.coveredArea(res);
    last_coverage_rate_ = stats.coverageRate();

    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = stamp;
    map_msg.info = current_map_->info;
    map_msg.data = coverage_grid_.encode();
    mb_->publishCoverageMap(map_msg);
    mb_->publishCoveredArea(last_clean_area_);

    std::ostringstream os;
    os << "{\"state\":\"" << coverage_state_ << "\","
       << "\"coverage_rate\":" << stats.coverageRate() << ","
       << "\"covered_area\":" << last_clean_area_ << ","
       << "\"total_area\":" << stats.total * res * res << ","
       << "\"uncovered_area\":" << stats.uncovered * res * res << ","
       << "\"blocked_area\":" << stats.blocked * res * res << ","
       << "\"obstacle_area\":" << stats.obstacle * res * res << ","
       << "\"dynamic_cells\":" << coverage_grid_.dynamicCount() << ","
       << "\"unreachable\":[";
    const auto regions = coverage_grid_.blockedRegions(1);
    for (size_t r = 0; r < regions.size(); ++r) {
      if (r > 0) { os << ","; }
      os << "[";
      for (size_t i = 0; i < regions[r].size(); ++i) {
        if (i > 0) { os << ","; }
        os << "[" << regions[r][i].x << "," << regions[r][i].y << "]";
      }
      os << "]";
    }
    os << "]}";
    const std::string json = os.str();
    mb_->publishCoverageStatus(json);

    if (!coverage_report_path_.empty() &&
        (force_report || coverage_state_ == "DONE" || coverage_state_ == "STOPPED")) {
      std::ofstream out(coverage_report_path_ + "/coverage_report.json");
      if (out.is_open()) { out << json; }
    }
  }

  void finishCoverage()
  {
    if (!coverage_active_) { return; }
    const core::Pose2D pose = toCorePose(current_pose_);
    coverage_grid_.markUnreachable(pose);
    if (coverage_grid_.hasAnyUncovered()) {
      coverage_grid_.markAllUncoveredBlocked();
    }
    coverage_state_ = "DONE";
    publishCoverageOutputs(ros::Time::now(), true);
    const auto stats = coverage_grid_.stats();
    coverage_active_ = false;
    coverage_has_target_ = false;
    current_path_.poses.clear();
    if (coverage_controller_) { coverage_controller_->reset(); }
    setGoalStatus("SUCCEEDED");
    finishAction(movebase::RESULT_SUCCESS);
    ROS_INFO("[coverage] done: rate=%.3f covered=%.2f m2 blocked=%.2f m2",
             stats.coverageRate(), last_clean_area_,
             stats.blocked * current_map_->info.resolution *
             current_map_->info.resolution);
  }

  void stopCoverageInternal()
  {
    if (!coverage_active_) { return; }
    coverage_active_ = false;
    coverage_state_ = "STOPPED";
    coverage_has_target_ = false;
    current_path_.poses.clear();
    if (coverage_controller_) { coverage_controller_->reset(); }
    publishCoverageOutputs(ros::Time::now(), true);
  }

  void setGoalStatus(const std::string & status)
  {
    goal_status_ = status;
    if (status == "ACCEPTED") {
      task_sm_.handle(core::TaskEvent::GOAL_RECEIVED);
    } else if (status.rfind("FAILED", 0) == 0) {
      task_sm_.handle(core::TaskEvent::PLAN_FAIL);
    } else if (status == "SUCCEEDED") {
      task_sm_.handle(core::TaskEvent::GOAL_REACHED);
    } else if (status.rfind("STOPPED", 0) == 0 || status == "REJECTED_FRAME") {
      task_sm_.handle(core::TaskEvent::STOP);
    }
    std_msgs::String msg;
    msg.data = status;
    if (goal_status_pub_) goal_status_pub_.publish(msg);
  }

  void publishTrackedObstacles()
  {
    if (!tracked_obs_pub_) return;
    visualization_msgs::MarkerArray arr;
    const auto tracks = tracker_.tracked();
    for (const auto & t : tracks) {
      visualization_msgs::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = ros::Time::now();
      m.ns = "tracked_obstacles";
      m.id = t.id;
      m.type = visualization_msgs::Marker::CYLINDER;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position.x = t.x;
      m.pose.position.y = t.y;
      m.pose.position.z = 0.25;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = 2.0 * t.radius;
      m.scale.z = 0.5;
      m.color.a = 0.65;
      m.color.r = 1.0;
      m.color.g = t.speed() > 0.15 ? 0.2 : 0.8;
      m.color.b = 0.2;
      m.lifetime = ros::Duration(0.5);
      arr.markers.push_back(m);
    }
    tracked_obs_pub_.publish(arr);
  }

  void diagnosticsCallback(const ros::TimerEvent &)
  {
    if (!diagnostics_pub_) return;
    std::ostringstream os;
    os << "{\"state\":\"" << (is_navigating_ ? "NAVIGATING" : "IDLE")
       << "\",\"goal_status\":\"" << goal_status_
       << "\",\"safety\":\"" << last_safety_state_
       << "\",\"safety_scale\":" << last_safety_scale_
       << ",\"path_points\":" << current_path_.poses.size()
       << ",\"tracks\":" << tracker_.tracked().size()
       << ",\"task_state\":\"" << task_sm_.stateName() << "\""
       << ",\"voxels\":" << voxel_layer_.size()
       << ",\"odom_age\":" << (last_odom_stamp_.isZero() ? -1.0 : (ros::Time::now() - last_odom_stamp_).toSec())
       << "}";
    std_msgs::String msg;
    msg.data = os.str();
    diagnostics_pub_.publish(msg);
  }

  // ---- costmap ----
  void updateCostmap()
  {
    if (!current_map_) { return; }

    const unsigned int w = current_map_->info.width;
    const unsigned int h = current_map_->info.height;
    const float res = current_map_->info.resolution;
    const double ox = current_map_->info.origin.position.x;
    const double oy = current_map_->info.origin.position.y;

    const bool need_global = last_global_costmap_update_.isZero() ||
      (ros::Time::now() - last_global_costmap_update_).toSec() >= 1.0;
    if (planner_strategy_ && need_global) {
      auto grid = std::make_shared<core::CostmapGrid>();
      grid->info.width = w;
      grid->info.height = h;
      grid->info.resolution = res;
      grid->info.origin.position.x = ox;
      grid->info.origin.position.y = oy;
      grid->data = current_map_->data;

      core::Costmap cm;
      cm.grid = grid;
      cm.width = w;
      cm.height = h;
      cm.resolution = res;
      cm.origin_x = ox;
      cm.origin_y = oy;
      cm.data.assign(current_map_->data.size(), 0);
      for (size_t i = 0; i < current_map_->data.size(); ++i) {
        // unknown 对规划仍是障碍(254), 但不参与膨胀; 真实占据才是 255.
        // 若把 unknown 当 free, 规划会穿未知区域撞墙; 若把 unknown 当 255,
        // 它会向外膨胀, 把已知 free 的走廊也封死.
        cm.data[i] = (current_map_->data[i] >= 100)
                         ? 255
                         : (current_map_->data[i] < 0 ? 254 : 0);
      }
      for (const auto & pt : laser_points_) {
        const int mx = static_cast<int>((pt.x - ox) / res);
        const int my = static_cast<int>((pt.y - oy) / res);
        if (mx >= 0 && mx < static_cast<int>(w) && my >= 0 && my < static_cast<int>(h)) {
          cm.data[my * w + mx] = 255;
        }
      }
      planner_strategy_->setCostmap(cm);
      planner_costmap_ = cm;
      m_has_planner_costmap_ = true;

      // 膨胀并发布供 RViz 可视化
      const int inflate_cells = static_cast<int>(std::ceil(inflation_radius_ / res));
      const int safety_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_ / res)));
      std::vector<float> dist(w * h, std::numeric_limits<float>::max());
      std::queue<std::pair<int, int>> q;
      for (unsigned int y = 0; y < h; ++y) {
        for (unsigned int x = 0; x < w; ++x) {
          // unknown=254 只为 planner 阻挡路径, 不形成膨胀 halo.
          if (cm.data[y * w + x] >= 255) { dist[y * w + x] = 0.0f; q.push({x, y}); }
        }
      }
      const int dx4[4] = {-1, 0, 1, 0};
      const int dy4[4] = {0, -1, 0, 1};
      while (!q.empty()) {
        const std::pair<int, int> c = q.front(); q.pop();
        const float cd = dist[c.second * w + c.first];
        if (cd >= inflate_cells) { continue; }
        for (int i = 0; i < 4; ++i) {
          const int nx = c.first + dx4[i];
          const int ny = c.second + dy4[i];
          if (nx < 0 || nx >= static_cast<int>(w) || ny < 0 || ny >= static_cast<int>(h)) { continue; }
          if (cd + 1 < dist[ny * w + nx]) { dist[ny * w + nx] = cd + 1; q.push({nx, ny}); }
        }
      }

      nav_msgs::OccupancyGrid costmap_msg;
      costmap_msg.header.frame_id = map_frame_;
      costmap_msg.header.stamp = ros::Time::now();
      costmap_msg.info.resolution = res;
      costmap_msg.info.width = w;
      costmap_msg.info.height = h;
      costmap_msg.info.origin.position.x = ox;
      costmap_msg.info.origin.position.y = oy;
      costmap_msg.info.origin.orientation.w = 1.0;
      costmap_msg.data.resize(w * h);
      const float inv = 1.0f / std::max(1, inflate_cells - safety_cells);
      for (size_t i = 0; i < w * h; ++i) {
        if (cm.data[i] >= 254) { costmap_msg.data[i] = 100; continue; }
        const float d = dist[i];
        // 安全缓冲区只显示为高代价(99), 不是 lethal(100). A* 也把它当
        // 高代价而非障碍; 否则 costmap 视觉上会把机器人周围画成黑墙.
        if (d <= safety_cells) { costmap_msg.data[i] = 99; }
        else if (d <= inflate_cells) {
          costmap_msg.data[i] = static_cast<int8_t>((1.0f - (d - safety_cells) * inv) * 99);
        } else { costmap_msg.data[i] = 0; }
      }
      global_costmap_pub_.publish(costmap_msg);
      if (mb_) { mb_->publishPlanMap(costmap_msg); }
      last_global_costmap_update_ = ros::Time::now();
    }

    // 局部障碍物聚类, 供控制器避障
    if (!controller_strategy_) { return; }
    double rx = 0.0, ry = 0.0;
    try {
      const geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
        map_frame_, robot_frame_, ros::Time(0), ros::Duration(0.05));
      rx = tf.transform.translation.x;
      ry = tf.transform.translation.y;
    } catch (tf2::TransformException &) {
      return;
    }

    const double win_width = 6.0;
    const int win_half = static_cast<int>(std::ceil(win_width * 0.5 / res));
    const int cx = static_cast<int>((rx - ox) / res);
    const int cy = static_cast<int>((ry - oy) / res);

    core::ObstacleArray obstacles;
    std::vector<std::vector<bool>> visited(h, std::vector<bool>(w, false));
    const int step = std::max(1, static_cast<int>(0.3 / res));
    // 局部障碍聚类只允许在机器人附近的窗口内扩张. 以前 BFS 从窗口内的墙点
    // 出发会沿整张连通墙一路蔓延到全图 (310k 格), 再被 15Hz /scan 反复触发,
    // 把单线程 spinner 卡在 updateCostmap(), action goal/status 全部饿死.
    const int local_x0 = std::max(0, cx - win_half);
    const int local_x1 = std::min(static_cast<int>(w) - 1, cx + win_half);
    const int local_y0 = std::max(0, cy - win_half);
    const int local_y1 = std::min(static_cast<int>(h) - 1, cy + win_half);

    auto isObstacleCell = [&](int mx, int my) -> bool {
      if (mx < local_x0 || mx > local_x1 || my < local_y0 || my > local_y1) {
        return false;
      }
      if (current_map_->data[my * w + mx] >= 100) { return true; }
      for (const auto & pt : laser_points_) {
        const int lx = static_cast<int>((pt.x - ox) / res);
        const int ly = static_cast<int>((pt.y - oy) / res);
        if (std::abs(lx - mx) <= 1 && std::abs(ly - my) <= 1) { return true; }
      }
      return false;
    };

    for (int dy = -win_half; dy <= win_half; dy += step) {
      for (int dx = -win_half; dx <= win_half; dx += step) {
        const int mx = cx + dx;
        const int my = cy + dy;
        if (mx < 0 || mx >= static_cast<int>(w) || my < 0 || my >= static_cast<int>(h)) { continue; }
        if (visited[my][mx]) { continue; }
        if (!isObstacleCell(mx, my)) { continue; }

        std::queue<std::pair<int, int>> cluster_q;
        cluster_q.push({mx, my});
        visited[my][mx] = true;
        double sum_x = 0.0, sum_y = 0.0;
        int cnt = 0, min_x = mx, max_x = mx, min_y = my, max_y = my;
        while (!cluster_q.empty()) {
          const std::pair<int, int> c = cluster_q.front(); cluster_q.pop();
          sum_x += c.first; sum_y += c.second; ++cnt;
          min_x = std::min(min_x, c.first); max_x = std::max(max_x, c.first);
          min_y = std::min(min_y, c.second); max_y = std::max(max_y, c.second);
          const int nx4[4] = {c.first - 1, c.first + 1, c.first, c.first};
          const int ny4[4] = {c.second, c.second, c.second - 1, c.second + 1};
          for (int i = 0; i < 4; ++i) {
            const int nx = nx4[i];
            const int ny = ny4[i];
            if (nx < 0 || nx >= static_cast<int>(w) || ny < 0 || ny >= static_cast<int>(h)) { continue; }
            if (visited[ny][nx]) { continue; }
            if (isObstacleCell(nx, ny)) { visited[ny][nx] = true; cluster_q.push({nx, ny}); }
          }
        }

        core::Obstacle obs;
        obs.x = ox + (sum_x / cnt + 0.5) * res;
        obs.y = oy + (sum_y / cnt + 0.5) * res;
        const double ww = (max_x - min_x + 1) * res;
        const double hh = (max_y - min_y + 1) * res;
        obs.radius = std::sqrt(ww * ww + hh * hh) * 0.5;
        obstacles.push_back(obs);
      }
    }
    controller_strategy_->setObstacles(obstacles);
  }

  void publishLocalCostmap(const ros::TimerEvent &)
  {
    if (!current_map_) { return; }
    const unsigned int w = current_map_->info.width;
    const unsigned int h = current_map_->info.height;
    const float res = current_map_->info.resolution;
    const double ox = current_map_->info.origin.position.x;
    const double oy = current_map_->info.origin.position.y;

    double rx = 0.0, ry = 0.0;
    try {
      const geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
        map_frame_, robot_frame_, ros::Time(0), ros::Duration(0.05));
      rx = tf.transform.translation.x;
      ry = tf.transform.translation.y;
    } catch (tf2::TransformException &) {
      return;
    }

    const double win_width = 6.0;
    const int win_half = static_cast<int>(std::ceil(win_width * 0.5 / res));
    const int cx = static_cast<int>((rx - ox) / res);
    const int cy = static_cast<int>((ry - oy) / res);
    const int lw = win_half * 2 + 1;

    nav_msgs::OccupancyGrid local_costmap;
    local_costmap.header.frame_id = map_frame_;
    local_costmap.header.stamp = ros::Time::now();
    local_costmap.info.resolution = res;
    local_costmap.info.width = lw;
    local_costmap.info.height = lw;
    local_costmap.info.origin.position.x = rx - (win_half + 0.5) * res;
    local_costmap.info.origin.position.y = ry - (win_half + 0.5) * res;
    local_costmap.info.origin.orientation.w = 1.0;

    // 窗口外默认致命; 融合静态地图 + 实时激光.
    std::vector<int8_t> raw(lw * lw, 100);
    for (int dy = -win_half; dy <= win_half; ++dy) {
      for (int dx = -win_half; dx <= win_half; ++dx) {
        const int mx = cx + dx;
        const int my = cy + dy;
        if (mx < 0 || mx >= static_cast<int>(w) || my < 0 || my >= static_cast<int>(h)) { continue; }
        const int gx = dx + win_half;
        const int gy = dy + win_half;
        const int8_t mv = current_map_->data[my * w + mx];
        // unknown 保持 free; 只有真实占据格才是障碍.
        raw[gy * lw + gx] = (mv >= 100) ? 100 : 0;
      }
    }
    for (const auto& pt : laser_points_) {
      const int mx = static_cast<int>(std::floor((pt.x - ox) / res));
      const int my = static_cast<int>(std::floor((pt.y - oy) / res));
      const int dx = mx - cx;
      const int dy = my - cy;
      if (dx < -win_half || dx > win_half || dy < -win_half || dy > win_half) continue;
      raw[(dy + win_half) * lw + (dx + win_half)] = 100;
    }
    if (tracking_enabled_) {
      const auto predicted = tracker_.predicted(tracking_horizon_);
      for (const auto & t : predicted) {
        const int oxc = static_cast<int>(std::floor((t.x - ox) / res));
        const int oyc = static_cast<int>(std::floor((t.y - oy) / res));
        const int dx = oxc - cx;
        const int dy = oyc - cy;
        const int rr = std::max(1, static_cast<int>(std::ceil(t.radius / res)));
        for (int yy = -rr; yy <= rr; ++yy)
          for (int xx = -rr; xx <= rr; ++xx) {
            if (xx * xx + yy * yy > rr * rr) continue;
            const int gx = dx + xx + win_half;
            const int gy = dy + yy + win_half;
            if (gx >= 0 && gx < lw && gy >= 0 && gy < lw) raw[gy * lw + gx] = 100;
          }
      }
    }

    if (voxel_enabled_) {
      const auto voxel_pts = voxel_layer_.occupancyPoints();
      for (const auto & p : voxel_pts) {
        const int mx = static_cast<int>(std::floor((p.x - ox) / res));
        const int my = static_cast<int>(std::floor((p.y - oy) / res));
        const int dx = mx - cx;
        const int dy = my - cy;
        if (dx < -win_half || dx > win_half || dy < -win_half || dy > win_half) continue;
        raw[(dy + win_half) * lw + (dx + win_half)] = 100;
      }
    }

    // 清掉机器人自身中心附近的自点/噪点; 碰撞仍由安全层和硬边界负责.
    const int clear_cells = std::max(1, static_cast<int>(std::ceil(0.2 / res)));
    for (int dy = -clear_cells; dy <= clear_cells; ++dy)
      for (int dx = -clear_cells; dx <= clear_cells; ++dx) {
        if (dx * dx + dy * dy > clear_cells * clear_cells) continue;
        const int gx = win_half + dx, gy = win_half + dy;
        if (gx >= 0 && gx < lw && gy >= 0 && gy < lw) raw[gy * lw + gx] = 0;
      }

    const int l_inflate = std::max(
      static_cast<int>(std::ceil(inflation_radius_ / res)),
      std::max(1, static_cast<int>(std::ceil(robot_radius_ / res))) + 1);
    const int l_safety = std::max(1, static_cast<int>(std::ceil(robot_radius_ / res)));
    std::vector<float> ldist(lw * lw, std::numeric_limits<float>::max());
    std::queue<std::pair<int, int>> lq;
    for (int y = 0; y < lw; ++y) {
      for (int x = 0; x < lw; ++x) {
        if (raw[y * lw + x] >= 100) { ldist[y * lw + x] = 0.0f; lq.push({x, y}); }
      }
    }
    const int ddx[4] = {-1, 0, 1, 0};
    const int ddy[4] = {0, -1, 0, 1};
    while (!lq.empty()) {
      const std::pair<int, int> c = lq.front(); lq.pop();
      const float cd = ldist[c.second * lw + c.first];
      if (cd >= l_inflate) { continue; }
      for (int i = 0; i < 4; ++i) {
        const int nx = c.first + ddx[i];
        const int ny = c.second + ddy[i];
        if (nx < 0 || nx >= lw || ny < 0 || ny >= lw) { continue; }
        if (cd + 1 < ldist[ny * lw + nx]) { ldist[ny * lw + nx] = cd + 1; lq.push({nx, ny}); }
      }
    }

    const float linv = 1.0f / std::max(1, l_inflate - l_safety);
    local_costmap.data.resize(lw * lw);
    for (int i = 0; i < lw * lw; ++i) {
      if (raw[i] >= 100) { local_costmap.data[i] = 100; continue; }
      const float d = ldist[i];
      // 膨胀安全环显示为 99, 真实障碍 raw 仍是 100.
      if (d <= l_safety) { local_costmap.data[i] = 99; }
      else if (d <= l_inflate) {
        local_costmap.data[i] = static_cast<int8_t>((1.0f - (d - l_safety) * linv) * 99);
      } else { local_costmap.data[i] = 0; }
    }
    local_costmap_pub_.publish(local_costmap);
    if (mb_) { mb_->publishLocalMap(local_costmap); }

    // ROS2 同类节点会把局部 costmap 注入控制器; ROS1 之前漏了这一步,
    // DWA/MPPI 只能看到 RViz, 看不到真实局部障碍.
    if (controller_strategy_) {
      core::CostmapGrid grid;
      grid.info.width = local_costmap.info.width;
      grid.info.height = local_costmap.info.height;
      grid.info.resolution = local_costmap.info.resolution;
      grid.info.origin.position.x = local_costmap.info.origin.position.x;
      grid.info.origin.position.y = local_costmap.info.origin.position.y;
      if (!controller_footprint_.empty()) {
        // footprint-aware: 控制器用未膨胀的实时障碍, 由 footprint 采样点负责
        // 车体形状; 不能再用 0.62m 圆形膨胀, 否则等于把车体算了两遍.
        grid.data = raw;
      } else {
        grid.data = local_costmap.data;
      }
      controller_strategy_->setLocalCostmap(grid);
    }
  }

  // ---- 导航 ----
  void waitForAsyncPlan()
  {
    if (!async_plan_future_.valid()) return;
    try { (void)async_plan_future_.get(); }
    catch (const std::exception & e) {
      ROS_WARN("[rosiwit_navigation_ros1] async planning exception: %s", e.what());
    }
  }

  void launchPlan()
  {
    if (!planning_active_ || async_plan_future_.valid()) return;
    if (!async_planner_ || !m_has_planner_costmap_) {
      ROS_ERROR("[rosiwit_navigation_ros1] planner/costmap not ready");
      planning_active_ = false;
      if (action_active_) {
        failAction(movebase::RESULT_GLOBAL_MAP_ERROR, "FAILED_PLAN");
      } else {
        setGoalStatus("FAILED_PLAN");
        stopNavigation();
      }
      return;
    }
    const core::Pose2D start = pending_has_start_override_
      ? pending_start_override_ : toCorePose(current_pose_);
    const core::Pose2D goal = toCorePose(pending_goal_);
    async_planner_->setCostmap(planner_costmap_);
    active_plan_generation_ = plan_generation_;
    async_plan_future_ = std::async(std::launch::async, [this, start, goal]() {
      return async_planner_->plan(start, goal);
    });
  }

  void startPlanning(const geometry_msgs::PoseStamped & goal, bool allow_initial_rotate,
                     bool restart, const core::Pose2D * start_override = nullptr)
  {
    pending_has_start_override_ = (start_override != nullptr);
    if (start_override != nullptr) { pending_start_override_ = *start_override; }
    // restart=true: 新目标, 先停旧路径; false: 1Hz 重规划, 继续沿旧路径控制.
    if (restart) {
      stopNavigation();
      current_path_.poses.clear();
      global_path_pub_.publish(current_path_);
    } else {
      is_navigating_ = true;  // 重规划期间旧路径继续可用
    }
    if (!updateCurrentPose()) {
      ROS_ERROR("[rosiwit_navigation_ros1] cannot get current pose for planning");
      if (action_active_) {
        failAction(movebase::RESULT_POSE_ERROR, "FAILED_POSE");
      } else {
        setGoalStatus("FAILED_POSE");
        stopNavigation();
      }
      return;
    }
    pending_goal_ = goal;
    pending_allow_initial_rotate_ = allow_initial_rotate;
    ++plan_generation_;
    planning_active_ = true;
    if (async_plan_future_.valid()) {
      // 旧规划仍在跑: 取消并等 pollPlanning 丢弃, 不阻塞控制定时器.
      async_planner_->cancel();
      return;
    }
    launchPlan();
  }

  void pollPlanning()
  {
    if (!async_plan_future_.valid()) {
      if (planning_active_) launchPlan();
      return;
    }
    if (async_plan_future_.wait_for(std::chrono::seconds(0)) !=
        std::future_status::ready) {
      return;
    }
    core::Result<core::Path> result = core::Result<core::Path>::error(
      core::ErrorCode::PLANNING_FAILED, "async planning exception");
    try { result = async_plan_future_.get(); }
    catch (const std::exception & e) {
      ROS_ERROR("[rosiwit_navigation_ros1] async planning exception: %s", e.what());
    }
    planning_active_ = false;

    // 结果属于旧目标: 丢弃并立即为新目标启动规划.
    if (active_plan_generation_ != plan_generation_) {
      if (planning_active_ || !pending_goal_.header.frame_id.empty()) {
        planning_active_ = true;
        launchPlan();
      }
      return;
    }

    if (!result.is_ok()) {
      ROS_ERROR("[rosiwit_navigation_ros1] planner '%s' failed: %s",
                planner_name_.c_str(), result.error_message().c_str());
      if (action_active_) {
        failAction(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
      } else {
        setGoalStatus("FAILED_PLAN");
        stopNavigation();
      }
      return;
    }
    applyPlanningResult(result, pending_allow_initial_rotate_);
  }

  void applyPlanningResult(const core::Result<core::Path> & result, bool allow_initial_rotate)
  {
    current_path_ = toRosPath(result.value(), pending_goal_.header.frame_id, ros::Time::now());

    if (current_path_.poses.size() >= 3) {
      planners::SmootherConfig smoother_cfg;
      smoother_cfg.max_iterations = 100;
      smoother_cfg.alpha_start = 0.5;
      smoother_cfg.alpha_end = 0.1;
      std::shared_ptr<core::CostmapGrid> core_map;
      if (current_map_) {
        core_map = std::make_shared<core::CostmapGrid>();
        core_map->info.width = current_map_->info.width;
        core_map->info.height = current_map_->info.height;
        core_map->info.resolution = current_map_->info.resolution;
        core_map->info.origin.position.x = current_map_->info.origin.position.x;
        core_map->info.origin.position.y = current_map_->info.origin.position.y;
        core_map->data = current_map_->data;
      }
      core::Path core_path = planners::smoothPath(toCorePath(current_path_), core_map, smoother_cfg);
      current_path_ = toRosPath(core_path, pending_goal_.header.frame_id, ros::Time::now());
    }

    if (controller_strategy_) controller_strategy_->setPath(toCorePath(current_path_));
    global_path_pub_.publish(current_path_);

    m_need_initial_rotate_ = false;
    if (allow_initial_rotate && current_path_.poses.size() > 2) {
      const double current_yaw = yawFromQuat(current_pose_.pose.orientation);
      const double dx = current_path_.poses[1].pose.position.x - current_path_.poses[0].pose.position.x;
      const double dy = current_path_.poses[1].pose.position.y - current_path_.poses[0].pose.position.y;
      const double path_yaw = std::atan2(dy, dx);
      if (std::abs(normalizeAngle(path_yaw - current_yaw)) > 0.35) {
        m_need_initial_rotate_ = true;
        m_initial_rotate_target_ = path_yaw;
      }
    }

    task_sm_.handle(core::TaskEvent::PLAN_OK);
    is_navigating_ = true;
    is_paused_ = false;
    ROS_INFO("[rosiwit_navigation_ros1] path planned with %zu points",
             current_path_.poses.size());
  }

  void replanningCheck(const ros::TimerEvent &)
  {
    if (coverage_active_) return;
    if (!is_navigating_ || is_paused_ || m_need_initial_rotate_ || planning_active_) return;
    // 只有"规划到目标点"的任务会周期重规划; 跟踪固定路径/旋转/倒车不重规划.
    if (action_active_ &&
        (action_task_.kind() != core::ActionKind::NAVIGATE || action_task_.rotating())) return;
    if (goal_pose_.header.frame_id.empty() || !m_has_planner_costmap_) return;
    startPlanning(goal_pose_, /*allow_initial_rotate=*/false, /*restart=*/false);
  }

  void executeNavigation(const geometry_msgs::PoseStamped & goal,
                         bool allow_initial_rotate = true)
  {
    startPlanning(goal, allow_initial_rotate, /*restart=*/true);
  }

  void controlLoop(const ros::TimerEvent &)
  {
    if (planning_active_) pollPlanning();
    if (!is_navigating_ || is_paused_) { return; }
    if (last_odom_stamp_.isZero() ||
        (ros::Time::now() - last_odom_stamp_).toSec() > 0.6) {
      if (action_active_) {
        failAction(movebase::RESULT_PERCEPTION_TIMEOUT, "STOPPED_ODOM");
      } else {
        setGoalStatus("STOPPED_ODOM");
        stopNavigation();
      }
      return;
    }
    if (!updateCurrentPose()) {
      // 定位/TF 失效时必须主动发零, 不能保留上一条 cmd_vel.
      if (action_active_) {
        failAction(movebase::RESULT_POSE_ERROR, "FAILED_TF");
      } else {
        setGoalStatus("FAILED_TF");
        stopNavigation();
      }
      return;
    }

    if (coverage_active_) {
      coverageControlStep();
      return;
    }

    // ---- movebase action: 状态/进度/到位/卡死全部交给算法层 ----
    if (action_active_) {
      const bool xy_reached =
        !planning_active_ && !current_path_.poses.empty() && isGoalReached();
      const double controller_progress =
        controller_strategy_ ? controller_strategy_->getProgress() : 0.0;
      const core::ActionTaskStatus task_status = action_task_.update(
        toCorePose(current_pose_), ros::Time::now().toSec(),
        xy_reached, controller_progress);

      if (task_status == core::ActionTaskStatus::SUCCEEDED) {
        ROS_INFO("[rosiwit_navigation_ros1] goal reached");
        setGoalStatus("SUCCEEDED");
        finishAction(movebase::RESULT_SUCCESS);
        return;
      }
      if (task_status == core::ActionTaskStatus::FAILED_STUCK) {
        ROS_ERROR("[movebase_iface] 超过 %.0f 秒没有进展, 判定卡死",
                  movebase_stuck_timeout_);
        failAction(movebase::RESULT_NO_PROGRESS, "FAILED_STUCK");
        return;
      }
      // 原地旋转 (navi_cmd=6 / 单点末端对齐) 与倒车 (navi_cmd=7):
      // 速度 primitive 由算法层给出, 这里只做安全过滤与发布.
      if (action_task_.rotating()) {
        publishRotationStep();
        return;
      }
      if (action_task_.backing()) {
        publishBackOffStep();
        return;
      }
    } else if (isGoalReached()) {
      ROS_INFO("[rosiwit_navigation_ros1] goal reached");
      setGoalStatus("SUCCEEDED");
      stopNavigation();
      return;
    }

    if (m_need_initial_rotate_) {
      geometry_msgs::Twist rot_cmd;
      const double current_yaw = yawFromQuat(current_pose_.pose.orientation);
      const double angle_error = normalizeAngle(m_initial_rotate_target_ - current_yaw);
      if (std::abs(angle_error) < 0.08) {
        m_need_initial_rotate_ = false;
      } else {
        rot_cmd.angular.z = std::max(-0.8, std::min(0.8, 1.5 * angle_error));
        publishCommand(applySafety(rot_cmd));
        recordActionTraveled();
        if (action_active_) { publishActionOutputs(); }
        return;
      }
    }

    publishCommand(applySafety(computeVelocityCommand()));
    path_pub_.publish(current_path_);
    recordActionTraveled();
    if (action_active_) { publishActionOutputs(); }
  }

  bool updateCurrentPose()
  {
    try {
      const geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
        global_frame_, robot_frame_, ros::Time(0), ros::Duration(0.1));
      current_pose_.header.stamp = tf.header.stamp;
      current_pose_.header.frame_id = global_frame_;
      current_pose_.pose.position.x = tf.transform.translation.x;
      current_pose_.pose.position.y = tf.transform.translation.y;
      current_pose_.pose.position.z = tf.transform.translation.z;
      current_pose_.pose.orientation = tf.transform.rotation;
      return true;
    } catch (tf2::TransformException & ex) {
      ROS_WARN_THROTTLE(2.0, "[rosiwit_navigation_ros1] tf error: %s", ex.what());
      return false;
    }
  }

  bool isGoalReached()
  {
    if (controller_strategy_) {
      return controller_strategy_->isGoalReached(toCorePose(current_pose_));
    }
    const double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
    const double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double angle_diff = std::abs(normalizeAngle(
      yawFromQuat(goal_pose_.pose.orientation) - yawFromQuat(current_pose_.pose.orientation)));
    return distance < goal_tolerance_xy_ && angle_diff < goal_tolerance_yaw_;
  }

  geometry_msgs::Twist computeVelocityCommand()
  {
    if (controller_strategy_) {
      const auto core_pose = toCorePose(current_pose_);
      const auto core_vel = toCoreVelocity(current_velocity_);
      const auto cmd = controller_strategy_->computeVelocityCommand(core_pose, core_vel);
      return toRosTwist(cmd);
    }

    geometry_msgs::Twist cmd_vel;
    const double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
    const double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double target_angle = std::atan2(dy, dx);
    const double angle_diff = normalizeAngle(target_angle - yawFromQuat(current_pose_.pose.orientation));
    if (std::abs(angle_diff) < M_PI / 4.0) {
      cmd_vel.linear.x = std::min(distance / 1.0, 1.0) * max_velocity_x_;
    }
    cmd_vel.angular.z = 2.0 * angle_diff;
    cmd_vel.linear.x = std::max(min_velocity_x_, std::min(max_velocity_x_, cmd_vel.linear.x));
    cmd_vel.angular.z = std::max(-max_velocity_theta_, std::min(max_velocity_theta_, cmd_vel.angular.z));
    return cmd_vel;
  }

  geometry_msgs::Twist applySafety(const geometry_msgs::Twist & cmd)
  {
    if (!safety_enabled_) return cmd;
    if (safety_fail_safe_on_timeout_ && !last_scan_stamp_.isZero() &&
        (ros::Time::now() - last_scan_stamp_).toSec() > safety_source_timeout_) {
      last_safety_state_ = "STALE";
      last_safety_scale_ = 0.0;
      return geometry_msgs::Twist();
    }

    const double yaw = yawFromQuat(current_pose_.pose.orientation);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    std::vector<std::pair<double, double>> points;
    points.reserve(laser_points_.size());
    for (const auto & p : laser_points_) {
      const double dx = p.x - current_pose_.pose.position.x;
      const double dy = p.y - current_pose_.pose.position.y;
      const double bx = cy * dx + sy * dy;
      const double by = -sy * dx + cy * dy;
      if (bx * bx + by * by < safety_self_filter_radius_ * safety_self_filter_radius_) continue;
      points.emplace_back(bx, by);
    }

    const core::SafetyResult result = safety_monitor_.filter(toCoreVelocity(cmd), points);
    last_safety_state_ = result.state;
    last_safety_scale_ = result.scale;
    return toRosTwist(result.command);
  }

  void stopNavigation()
  {
    is_navigating_ = false;
    is_paused_ = false;
    // 取消/结束导航时必须同时作废在途的异步规划, 否则 pollPlanning 会把这个
    // 结果 applyPlanningResult() 回来, 让已经停止的任务重新跑起来.
    if (planning_active_ || async_plan_future_.valid()) {
      planning_active_ = false;
      ++plan_generation_;
      if (async_planner_) { async_planner_->cancel(); }
    }
    geometry_msgs::Twist stop_cmd;
    publishCommand(stop_cmd);
  }

  // ---- 类型转换 (ROS <-> core) ----
  static core::Pose2D toCorePose(const geometry_msgs::PoseStamped & pose)
  {
    return core::Pose2D(pose.pose.position.x, pose.pose.position.y,
                        yawFromQuat(pose.pose.orientation));
  }

  static core::VelocityCommand toCoreVelocity(const geometry_msgs::Twist & twist)
  {
    return core::VelocityCommand(twist.linear.x, twist.linear.y, twist.angular.z);
  }

  static geometry_msgs::Twist toRosTwist(const core::VelocityCommand & cmd)
  {
    geometry_msgs::Twist twist;
    twist.linear.x = cmd.linear_x;
    twist.linear.y = cmd.lateral_y;
    twist.angular.z = cmd.angular_z;
    return twist;
  }

  static core::Path toCorePath(const nav_msgs::Path & path)
  {
    core::Path core_path;
    for (const auto & pose : path.poses) {
      core::PathPoint pt;
      pt.pose = toCorePose(pose);
      core_path.points.push_back(pt);
    }
    return core_path;
  }

  static nav_msgs::Path toRosPath(const core::Path & path, const std::string & frame_id,
                                  const ros::Time & stamp)
  {
    nav_msgs::Path ros_path;
    ros_path.header.frame_id = frame_id;
    ros_path.header.stamp = stamp;
    for (const auto & pt : path.points) {
      geometry_msgs::PoseStamped pose;
      pose.header = ros_path.header;
      pose.pose.position.x = pt.pose.x;
      pose.pose.position.y = pt.pose.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = quatFromYaw(pt.pose.theta);
      ros_path.poses.push_back(pose);
    }
    return ros_path;
  }

  // ---- 成员 ----
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double controller_frequency_ = 20.0;
  double planner_frequency_ = 1.0;
  double max_velocity_x_ = 0.5;
  double min_velocity_x_ = -0.5;
  double max_velocity_theta_ = 1.0;
  double goal_tolerance_xy_ = 0.1;
  double goal_tolerance_yaw_ = 0.1;
  double lookahead_distance_ = 0.6;
  double min_lookahead_distance_ = 0.3;
  double max_lookahead_distance_ = 0.9;
  double inflation_radius_ = 0.5;
  double robot_radius_ = 0.21;
  int planner_max_iterations_ = 200000;
  double planner_timeout_ = 2.0;
  double max_accel_x_ = 0.4;
  double max_accel_theta_ = 0.8;

  bool safety_enabled_ = true;
  double safety_self_filter_radius_ = 0.25;
  bool safety_fail_safe_on_timeout_ = true;
  double safety_source_timeout_ = 0.5;
  double safety_slowdown_ratio_ = 0.5;
  bool safety_approach_enabled_ = true;
  double safety_time_before_collision_ = 2.0;
  core::SafetyMonitor safety_monitor_;
  core::ObstacleTracker tracker_;
  core::TaskStateMachine task_sm_;
  core::VoxelLayer voxel_layer_;
  bool voxel_enabled_ = true;
  std::string voxel_cloud_topic_ = "/world_cloud";
  double voxel_resolution_ = 0.10;
  double voxel_min_height_ = -0.20;
  double voxel_max_height_ = 2.00;
  double voxel_decay_ = 2.0;
  double voxel_inflation_ = 0.15;
  bool tracking_enabled_ = true;
  double tracking_horizon_ = 0.5;
  double tracking_cluster_distance_ = 0.35;
  int tracking_confirm_hits_ = 3;
  std::vector<core::Point2D> tracker_points_;
  std::string last_safety_state_ = "PASS";
  double last_safety_scale_ = 1.0;

  std::string global_frame_;
  std::string robot_frame_;
  std::string map_frame_;
  std::string planner_name_;
  std::string controller_name_;
  std::string odom_topic_;
  std::string rviz_goal_topic_ = "/move_base_simple/goal";
  bool rviz_goal_align_yaw_ = true;
  bool footprint_aware_control_ = false;
  double footprint_sample_spacing_ = 0.05;
  core::Footprint configured_footprint_;
  core::Footprint controller_footprint_;

  std::unique_ptr<core::IPlannerStrategy> planner_strategy_;
  std::unique_ptr<core::IPlannerStrategy> async_planner_;
  std::future<core::Result<core::Path>> async_plan_future_;
  bool planning_active_ = false;
  uint64_t plan_generation_ = 0;
  uint64_t active_plan_generation_ = 0;
  bool pending_allow_initial_rotate_ = true;
  geometry_msgs::PoseStamped pending_goal_;
  core::Costmap planner_costmap_;
  bool m_has_planner_costmap_ = false;
  std::unique_ptr<core::IControllerStrategy> controller_strategy_;
  std::unique_ptr<core::IControllerStrategy> coverage_controller_;

  ros::Subscriber odom_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber voxel_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber rviz_goal_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher path_pub_;
  ros::Publisher global_path_pub_;
  ros::Publisher global_costmap_pub_;
  ros::Publisher local_costmap_pub_;
  ros::Publisher goal_status_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Publisher tracked_obs_pub_;
  ros::ServiceServer nav_stop_srv_;
  std::string goal_status_ = "IDLE";
  ros::Time last_odom_stamp_;
  ros::Time last_scan_stamp_;
  ros::Time last_global_costmap_update_;
  ros::Timer control_timer_;
  ros::Timer local_costmap_timer_;
  ros::Timer replanning_timer_;
  ros::Timer diagnostics_timer_;

  nav_msgs::OccupancyGrid::ConstPtr current_map_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::Twist current_velocity_;
  geometry_msgs::PoseStamped goal_pose_;
  nav_msgs::Path current_path_;
  std::vector<geometry_msgs::Point> laser_points_;

  bool is_navigating_;
  bool is_paused_;
  bool m_need_initial_rotate_ = false;
  double m_initial_rotate_target_ = 0.0;

  // ---- robot_movebase 兼容层 (接口层) ----
  std::unique_ptr<movebase::Interface> mb_;
  bool movebase_compat_enabled_ = true;
  std::string cmd_vel_topic_ = "/cmd_vel";
  double movebase_default_speed_ = 0.0;
  double movebase_back_speed_ = 0.1;
  double movebase_rotate_speed_ = 0.6;
  double movebase_stuck_timeout_ = 60.0;
  double movebase_goal_yaw_tolerance_ = 0.1;
  double movebase_rotate_tolerance_ = 0.05;
  std::vector<geometry_msgs::Point> movebase_footprint_;

  // 任务执行语义在算法层; 这里只保留接口层状态.
  core::ActionTask action_task_;
  bool action_active_ = false;

  // ---- 动态覆盖清洁 ----
  bool coverage_active_ = false;
  core::CoverageGrid coverage_grid_;
  core::CoverageConfig coverage_config_;
  ros::Time coverage_last_plan_;
  ros::Time coverage_last_progress_;
  ros::Time coverage_last_status_;
  ros::Time coverage_last_dynamic_;
  size_t coverage_last_covered_count_ = 0;
  bool coverage_has_target_ = false;
  core::Pose2D coverage_target_;
  core::Pose2D coverage_last_progress_pose_;
  core::Pose2D coverage_last_motion_pose_;
  ros::Time coverage_last_motion_time_;
  std::string coverage_state_ = "IDLE";
  double last_clean_area_ = 0.0;
  double last_coverage_rate_ = 0.0;
  double coverage_sweep_width_ = 0.5;
  double coverage_max_speed_ = 0.4;
  double coverage_resolution_ = 0.05;
  double coverage_replan_period_ = 10.0;
  double coverage_horizon_ = 2.5;
  double coverage_retry_timeout_ = 10.0;
  double coverage_complete_rate_ = 0.995;
  double coverage_blocked_inflation_ = 0.10;
  double coverage_plan_inflation_ = 0.0;
  double coverage_plan_robot_radius_ = 0.0;
  std::string coverage_controller_name_ = "pure_pursuit";
  std::string coverage_report_path_;
  movebase::Goal action_goal_;
  movebase::Decoded action_cmd_;
  double action_speed_ = 0.0;
  std::vector<geometry_msgs::PoseStamped> action_traveled_;
  bool pending_has_start_override_ = false;
  core::Pose2D pending_start_override_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "rosiwit_navigation_ros1_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  RosiwitNavigationRos1Node node(nh, pnh);
  ros::spin();
  return 0;
}
