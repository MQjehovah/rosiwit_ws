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
#include <geometry_msgs/PointStamped.h>
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
#include "rosiwit_navigation/nav_core/voxel_layer.hpp"
#include "rosiwit_navigation/nav_core/costmap_layer.hpp"
#include "rosiwit_navigation/nav_core/layered_costmap.hpp"
#include "rosiwit_navigation/nav_core/costmap_layers.hpp"
#include "rosiwit_navigation/nav_core/action_task.hpp"
#include "rosiwit_navigation/nav_core/coverage_grid.hpp"
#include "rosiwit_navigation/nav_core/footprint.hpp"
#include <xzrobot_ros_dependency/SlamInfo.h>
#include <xzrobot_ros_dependency/LocalizationState.h>
#include <xzrobot_ros_dependency/VirtualObstacles.h>
#include "path_smoother.hpp"
#include "trajectory_parameterizer.hpp"
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

bool pointInPolygon(double x, double y, const core::Footprint & polygon)
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const double xi = polygon[i].first, yi = polygon[i].second;
    const double xj = polygon[j].first, yj = polygon[j].second;
    if (((yi > y) != (yj > y)) &&
        (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

double pointSegmentDistance(double px, double py,
                            double ax, double ay, double bx, double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  if (len2 <= 1e-12) { return std::hypot(px - ax, py - ay); }
  double t = ((px - ax) * dx + (py - ay) * dy) / len2;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

double pointPolygonDistance(double x, double y, const core::Footprint & polygon)
{
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    best = std::min(best, pointSegmentDistance(
      x, y, polygon[j].first, polygon[j].second,
      polygon[i].first, polygon[i].second));
  }
  return best;
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
    // 运动 profile: clean(清洁导航, 禁止倒车) | free(自由导航, 允许倒车)
    pnh_.param<std::string>("motion_profile", motion_profile_, "clean");
    pnh_.param<std::string>("free_planner", free_planner_, "hybrid_astar");
    pnh_.param<std::string>("clean_planner", clean_planner_, "astar");
    pnh_.param<bool>("planner_allow_reverse", planner_allow_reverse_, true);
    pnh_.param<double>("planner_max_reverse_distance", planner_max_reverse_distance_, 1.0);
    pnh_.param<double>("max_accel_x", max_accel_x_, 0.4);
    pnh_.param<double>("max_accel_theta", max_accel_theta_, 0.8);
    pnh_.param<int>("mppi_horizon", mppi_horizon_, 20);
    pnh_.param<int>("mppi_samples", mppi_samples_, 160);
    pnh_.param<double>("mppi_dt", mppi_dt_, 0.0);
    pnh_.param<double>("mppi_lambda", mppi_lambda_, 0.8);
    pnh_.param<double>("mppi_sigma_v", mppi_sigma_v_, 0.18);
    pnh_.param<double>("mppi_sigma_w", mppi_sigma_w_, 0.35);
    pnh_.param<double>("mppi_path_weight", mppi_path_weight_, 2.0);
    pnh_.param<double>("mppi_heading_weight", mppi_heading_weight_, 0.4);
    pnh_.param<double>("mppi_obstacle_weight", mppi_obstacle_weight_, 0.03);
    pnh_.param<double>("mppi_control_weight", mppi_control_weight_, 0.03);
    pnh_.param<double>("mppi_speed_weight", mppi_speed_weight_, 2.0);
    pnh_.param<double>("mppi_goal_weight", mppi_goal_weight_, 1.5);

    pnh_.param<std::string>("global_frame", global_frame_, "map");
    pnh_.param<std::string>("robot_frame", robot_frame_, "base_link");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("planner_name", planner_name_, "astar");
    pnh_.param<std::string>("controller_name", controller_name_, "pure_pursuit");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    pnh_.param<std::string>("map_topic", map_topic_, "/map");
    pnh_.param<std::string>("scan_topic", scan_topic_, "/scan");
    pnh_.param<std::string>("goal_topic", goal_topic_, "/goal_pose");
    pnh_.param<std::string>("nav_stop_service", nav_stop_service_, "/nav_stop");
    pnh_.param<std::string>("global_path_topic", global_path_topic_, "/global_path");
    pnh_.param<std::string>("local_path_topic", local_path_topic_, "/local_path");
    pnh_.param<std::string>("global_costmap_topic", global_costmap_topic_, "/global_costmap");
    pnh_.param<std::string>("local_costmap_topic", local_costmap_topic_, "/local_costmap");
    pnh_.param<std::string>("goal_status_topic", goal_status_topic_, "/goal_status");
    pnh_.param<std::string>("diagnostics_topic", diagnostics_topic_, "/nav_diagnostics");
    pnh_.param<std::string>("tracked_obstacles_topic", tracked_obstacles_topic_,
                            "/tracked_obstacles");

    // 传感器 fail-safe
    pnh_.param<double>("map_timeout", map_timeout_, 0.0);
    pnh_.param<double>("scan_timeout", scan_timeout_, 0.5);
    pnh_.param<double>("cloud_timeout", cloud_timeout_, 1.0);
    pnh_.param<bool>("require_scan", require_scan_, true);
    pnh_.param<bool>("require_cloud", require_cloud_, false);

    // 重规划滞回
    pnh_.param<double>("replan_max_interval", replan_max_interval_, 5.0);
    pnh_.param<double>("replan_min_interval", replan_min_interval_, 1.0);
    pnh_.param<bool>("replan_on_blocked", replan_on_blocked_, true);

    // costmap 构建节流 (scan 回调不再每个 scan 都做完整聚类)
    pnh_.param<double>("costmap_update_rate", costmap_update_rate_, 5.0);

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

    // 与 3D 前端相同的车体自过滤: /scan 在 base 系下落在车体包围盒内的点
    // 不进入 costmap/safety. 只过滤车体内部, 车外近障碍仍保留.
    pnh_.param<bool>("scan_self_filter_enabled", scan_self_filter_enabled_, true);
    pnh_.param<double>("scan_self_x_min", scan_self_x_min_, -0.45);
    pnh_.param<double>("scan_self_x_max", scan_self_x_max_, 0.75);
    pnh_.param<double>("scan_self_y_min", scan_self_y_min_, -0.55);
    pnh_.param<double>("scan_self_y_max", scan_self_y_max_, 0.55);

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
    pnh_.param<double>("tracking_max_radius", tracking_max_radius_, 1.2);
    pnh_.param<double>("tracking_min_prediction_speed", tracking_min_prediction_speed_, 0.15);
    core::ObstacleTrackerConfig tracking_cfg;
    tracking_cfg.cluster_distance = tracking_cluster_distance_;
    tracking_cfg.confirm_hits = tracking_confirm_hits_;
    tracking_cfg.max_radius = tracking_max_radius_;
    tracking_cfg.min_prediction_speed = tracking_min_prediction_speed_;
    tracker_.setConfig(tracking_cfg);

    pnh_.param<bool>("voxel_enabled", voxel_enabled_, true);
    pnh_.param<std::string>("voxel_cloud_topic", voxel_cloud_topic_, "/world_cloud");
    pnh_.param<double>("voxel_resolution", voxel_resolution_, 0.10);
    pnh_.param<double>("voxel_min_height", voxel_min_height_, -0.20);
    pnh_.param<double>("voxel_max_height", voxel_max_height_, 2.00);
    pnh_.param<double>("voxel_decay", voxel_decay_, 2.0);
    pnh_.param<double>("voxel_inflation", voxel_inflation_, 0.15);
    pnh_.param<int>("voxel_max_points", voxel_max_points_, 200000);
    core::VoxelLayerConfig voxel_cfg;
    voxel_cfg.resolution = voxel_resolution_;
    voxel_cfg.min_height = voxel_min_height_;
    voxel_cfg.max_height = voxel_max_height_;
    voxel_cfg.decay_time = voxel_decay_;
    voxel_cfg.inflation = voxel_inflation_;
    voxel_layer_.setConfig(voxel_cfg);

    // ---- 分层代价地图 (B1): 同一个层集合, 不同视图 ----
    pnh_.param<bool>("keepout_enabled", keepout_enabled_, true);
    pnh_.param<std::string>("virtual_obstacles_topic", virtual_obstacles_topic_,
                            "/virtual_obstacles");
    pnh_.param<std::string>("keepout_frame", keepout_frame_, "map");
    {
      std::vector<std::string> specs;
      pnh_.param<std::vector<std::string>>(
        "speed_limit_zones", specs, std::vector<std::string>());
      std::vector<core::SpeedLimitLayer::Zone> zones;
      for (const auto & spec : specs) {
        core::SpeedLimitLayer::Zone zone;
        if (core::SpeedLimitLayer::parseRectZone(spec, zone, max_velocity_x_)) {
          zones.push_back(zone);
        } else {
          ROS_WARN("[rosiwit_navigation_ros1] bad speed_limit_zones entry: %s",
                   spec.c_str());
        }
      }
      speed_limit_layer_.setZones(zones);
      if (!zones.empty()) {
        ROS_INFO("[rosiwit_navigation_ros1] speed limit zones: %zu", zones.size());
      }
    }
    keepout_layer_ = std::make_shared<core::KeepoutLayer>();
    keepout_layer_->setActive(keepout_enabled_);
    planner_static_layer_ = std::make_shared<core::StaticLayer>(true);
    local_static_layer_ = std::make_shared<core::StaticLayer>(false);
    local_scan_layer_ = std::make_shared<core::PointsLayer>("scan", 0.0);
    local_track_layer_ = std::make_shared<core::CirclesLayer>("tracked");
    local_voxel_layer_ = std::make_shared<core::PointsLayer>("voxel", 0.0);
    // 规划视图: 静态地图 + 禁行区 (规划器自己按 footprint 膨胀)
    planner_layers_.addLayer(planner_static_layer_);
    planner_layers_.addLayer(keepout_layer_);
    // 控制视图: 静态 + 激光 + 跟踪 + 体素 + 禁行区, 不膨胀 (footprint 负责车体)
    local_layers_.addLayer(local_static_layer_);
    local_layers_.addLayer(local_scan_layer_);
    local_layers_.addLayer(local_track_layer_);
    local_layers_.addLayer(local_voxel_layer_);
    local_layers_.addLayer(keepout_layer_);

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
    pnh_.param<double>("coverage_complete_rate", coverage_complete_rate_, 0.99);
    pnh_.param<double>("coverage_blocked_inflation", coverage_blocked_inflation_, 0.10);
    pnh_.param<double>("coverage_plan_inflation", coverage_plan_inflation_, 0.0);
    pnh_.param<double>("coverage_plan_robot_radius", coverage_plan_robot_radius_, 0.0);
    pnh_.param<std::string>("coverage_controller_name", coverage_controller_name_,
                            "pure_pursuit");
    pnh_.param<std::string>("coverage_report_path", coverage_report_path_, "");

    // 长方形车体的 footprint-aware 控制: 控制器按 footprint 采样点查 raw costmap.
    pnh_.param<bool>("footprint_aware_control", footprint_aware_control_, false);
    pnh_.param<double>("footprint_sample_spacing", footprint_sample_spacing_, 0.05);
    // 负余量: 轮廓内缩后做碰撞检查. 0=严格按车体; 0.02 允许贴边 2cm,
    // 是 0.75m 车身过 0.75m 通道的必要条件 (栅格离散本身占 ~0.1m).
    pnh_.param<double>("footprint_shrink", footprint_shrink_, 0.0);
    // 规划余量: 规划器用"车体轮廓 + margin"生成路径, 保证控制器(车体轮廓 +
    // 实时 scan)跟得上; 0.66m 车体 + 0.06m -> 0.78m 巷道仍可规划.
    pnh_.param<double>("planner_footprint_margin", planner_footprint_margin_, 0.06);
    pnh_.param<double>("path_resample_step", path_resample_step_, 0.08);
    configured_footprint_.clear();
    controller_footprint_.clear();  // 实际是否启用由 setupCore 按控制器能力决定
    // 规划器/控制器统一车体轮廓；优先读显式 footprint，缺失时回退到
    // movebase_robot_footprint / safety_footprint（166 三处目前是同一组点）。
    std::vector<double> robot_footprint;
    pnh_.param<std::vector<double>>("footprint", robot_footprint,
                                    std::vector<double>{});
    if (robot_footprint.empty()) robot_footprint = movebase_footprint;
    if (robot_footprint.empty()) robot_footprint = footprint;
    configured_footprint_ = toPolygon(robot_footprint);
    if (footprint_shrink_ > 1e-6) {
      configured_footprint_ = insetFootprint(configured_footprint_, footprint_shrink_);
      ROS_INFO("[rosiwit_navigation_ros1] footprint inset %.3fm for collision checks "
               "(窄道通过能力: 车身 + 2x栅格离散)", footprint_shrink_);
    }
    // 规划器用外扩轮廓: 路径离墙 >= planner_footprint_margin, 保证控制器
    // (用车身轮廓 + 实时 scan) 能跟得上; 0.66m 车体 + 0.06m 余量 -> 0.78m
    // 巷道仍可规划, 满足 0.8~0.85m 场景.
    planner_footprint_ = insetFootprint(configured_footprint_, -planner_footprint_margin_);
    if (footprint_aware_control_) {
      controller_footprint_ = configured_footprint_;
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

    pnh_.param<bool>("recovery_enabled", recovery_enabled_, true);
    pnh_.param<int>("recovery_max_attempts", recovery_max_attempts_, 2);
    // 恢复只允许规划得到的倒车路径. 不再保留 primitive 后退/旋转后备.
    pnh_.param<double>("recovery_back_distance", recovery_back_distance_, 0.60);
    pnh_.param<double>("recovery_timeout", recovery_timeout_, 20.0);
    // 运动看门狗 / 安全停障恢复 / 定位健康门控
    pnh_.param<double>("nav_no_progress_timeout", no_progress_timeout_, 8.0);
    pnh_.param<double>("nav_safety_stop_timeout", safety_stop_timeout_, 2.0);
    pnh_.param<double>("slam_info_timeout", slam_info_timeout_, 2.0);
    pnh_.param<std::string>("localization_state_topic", localization_state_topic_,
                            "/localization/state");
    pnh_.param<double>("slam_degrade_matching", slam_degrade_matching_, 0.65);
    pnh_.param<double>("slam_degrade_scale", slam_degrade_scale_, 0.5);
    // 恢复规划: 当前 footprint 压住的栅格视为 free, 否则反向 escape
    // 会在起点直接报 Start in obstacle. 正常规划不使用这个副本.
    pnh_.param<bool>("recovery_clear_start_footprint", recovery_clear_start_footprint_, true);
    pnh_.param<double>("recovery_start_clear_margin", recovery_start_clear_margin_, 0.05);
    pnh_.param<double>("recovery_start_relax_radius", recovery_start_relax_radius_, 1.0);
    // Normal planning also ignores the cells already under the current
    // footprint at the start; otherwise a robot parked in a narrow corridor
    // fails with "Start in obstacle" forever.  This does NOT relax obstacles
    // along the rest of the path.
    pnh_.param<bool>("planning_clear_start_footprint", planning_clear_start_footprint_, true);
    pnh_.param<double>("planning_start_clear_margin", planning_start_clear_margin_, 0.05);
    // Start-only footprint relaxation for normal planning: a robot parked
    // closer than its footprint to a mapped wall must still be able to plan
    // out.  This does not relax the rest of the route.
    pnh_.param<double>("planning_start_relax_radius", planning_start_relax_radius_, 0.0);

    // 孤立激光点滤波: 只保留与相邻扫描点足够近的点.
    pnh_.param<bool>("laser_filter_enabled", laser_filter_enabled_, true);
    pnh_.param<double>("laser_filter_radius", laser_filter_radius_, 0.15);
    pnh_.param<int>("laser_filter_min_neighbors", laser_filter_min_neighbors_, 1);
    pnh_.param<int>("laser_filter_window", laser_filter_window_, 2);

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
  // 凸多边形内缩 (负余量): 让车体轮廓在碰撞检查时可以贴近/轻微贴边,
  // 这是"0.75m 车身过 0.75m 通道"的前提 (0.05m 栅格离散本身就需要 ~0.1m
  // 余量). 逆时针多边形, 内法向 = (-dy, dx).
  static core::Footprint insetFootprint(const core::Footprint & poly, double d)
  {
    // d>0 内缩, d<0 外扩 (规划余量)
    if (poly.size() < 3 || std::fabs(d) <= 1e-6) return poly;
    const size_t n = poly.size();
    core::Footprint out;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      const auto & a = poly[(i + n - 1) % n];
      const auto & b = poly[i];
      const auto & c = poly[(i + 1) % n];
      auto line = [&](const std::pair<double, double> & p,
                      const std::pair<double, double> & q) {
        double dx = q.first - p.first, dy = q.second - p.second;
        double len = std::hypot(dx, dy);
        if (len < 1e-9) len = 1e-9;
        dx /= len; dy /= len;
        return std::array<double, 4>{p.first - dy * d, p.second + dx * d, dx, dy};
      };
      const auto l1 = line(a, b);
      const auto l2 = line(b, c);
      const double denom = l1[2] * l2[3] - l1[3] * l2[2];
      if (std::fabs(denom) < 1e-9) { out.push_back(b); continue; }
      const double t = ((l2[0] - l1[0]) * l2[3] - (l2[1] - l1[1]) * l2[2]) / denom;
      out.emplace_back(l1[0] + t * l1[2], l1[1] + t * l1[3]);
    }
    return out;
  }

  void setupCore()
  {
    planner_strategy_ = nav_core::NavigationFactory::createPlanner(planner_name_);
    if (planner_strategy_) {
      core::PlannerConfig cfg;
      cfg.name = planner_name_;
      cfg.resolution = 0.05;
      cfg.max_iterations = planner_max_iterations_;
      cfg.timeout = planner_timeout_;
      cfg.allow_reverse = planner_allow_reverse_;
      cfg.max_reverse_distance = planner_max_reverse_distance_;
      cfg.start_relax_radius = planning_start_relax_radius_;
      planner_strategy_->initialize(cfg);
      planner_strategy_->setInflationRadius(inflation_radius_);
      planner_strategy_->setRobotRadius(robot_radius_);
      if (planner_strategy_->supportsFootprint() && !configured_footprint_.empty()) {
        planner_strategy_->setRobotFootprint(planner_footprint_);
        ROS_INFO("[rosiwit_navigation_ros1] planner '%s' footprint-aware (%zu pts)",
                 planner_name_.c_str(), configured_footprint_.size());
      } else {
        ROS_INFO("[rosiwit_navigation_ros1] planner '%s' circular robot_radius=%.2f",
                 planner_name_.c_str(), robot_radius_);
      }
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
      acfg.allow_reverse = planner_allow_reverse_;
      acfg.max_reverse_distance = planner_max_reverse_distance_;
      acfg.start_relax_radius = planning_start_relax_radius_;
      async_planner_->initialize(acfg);
      async_planner_->setInflationRadius(inflation_radius_);
      async_planner_->setRobotRadius(robot_radius_);
      if (async_planner_->supportsFootprint() && !configured_footprint_.empty()) {
        async_planner_->setRobotFootprint(planner_footprint_);
      }
    }

    // Recovery planner: reverse-only Hybrid A*. It is independent from the
    // normal planner so recovery cannot change the main navigation policy.
    recovery_planner_ = nav_core::NavigationFactory::createPlanner("hybrid_astar");
    if (recovery_planner_) {
      core::PlannerConfig rcfg;
      rcfg.name = "hybrid_astar";
      rcfg.resolution = 0.05;
      rcfg.max_iterations = planner_max_iterations_;
      rcfg.timeout = planner_timeout_ > 0.0 ?
        std::min(planner_timeout_, 2.0) : 2.0;
      rcfg.allow_reverse = true;
      rcfg.reverse_only = true;
      rcfg.start_relax_radius = recovery_start_relax_radius_;
      ROS_INFO("[rosiwit_navigation_ros1] recovery planner config: reverse_only=%d "
               "start_relax_radius=%.2f timeout=%.2f",
               static_cast<int>(rcfg.reverse_only),
               rcfg.start_relax_radius, rcfg.timeout);
      recovery_planner_->initialize(rcfg);
      recovery_planner_->setInflationRadius(inflation_radius_);
      recovery_planner_->setRobotRadius(robot_radius_);
      if (!configured_footprint_.empty()) {
        recovery_planner_->setRobotFootprint(configured_footprint_);
      }
    }

    // Recovery path is reverse-only.  MPPI's local-costmap collision term is
    // intentionally conservative and refuses to follow a recovery path that
    // starts inside the occupied map cells, so recovery execution gets its own
    // path follower.  It still goes through applySafety() (scan stop/slowdown).
    recovery_controller_ = nav_core::NavigationFactory::createController("diff_drive");
    if (recovery_controller_) {
      core::ControllerConfig rcc;
      rcc.lookahead_distance = 0.35;
      rcc.xy_goal_tolerance = 0.12;
      rcc.slow_down_distance = 0.20;
      rcc.max_lookahead_distance = 0.60;
      rcc.min_lookahead_distance = 0.20;
      rcc.lookahead_gain = 1.0;
      rcc.kinematics.max_velocity_x = 0.30;
      rcc.kinematics.max_velocity_theta = max_velocity_theta_;
      rcc.max_linear_velocity = 0.30;
      rcc.max_angular_velocity = std::max(0.35, max_velocity_theta_);
      rcc.path_resample_step = 0.08;
      rcc.allow_reverse = true;
      recovery_controller_->initialize(rcc);
      ROS_INFO("[rosiwit_navigation_ros1] recovery controller: diff_drive "
               "vmax=%.2f lookahead=%.2f", rcc.max_linear_velocity,
               rcc.lookahead_distance);
    }

    controller_strategy_.reset();
    if (!initController(controller_name_, controller_strategy_, true)) {
      ROS_WARN("[rosiwit_navigation_ros1] controller '%s' not found", controller_name_.c_str());
    }
    if (footprint_aware_control_ && controller_name_ == "dwa") {
      ROS_WARN("[rosiwit_navigation_ros1] DWA + footprint 已知会绕圈, "
               "自动回退为圆形膨胀 (DWA 仍可用)");
    }
    // 覆盖模式的 approach 用独立 A*: 正常 planner 可能是 Hybrid A* + footprint,
    // 它会把区域边缘的目标判成 Goal in obstacle.
    coverage_planner_ = nav_core::NavigationFactory::createPlanner("astar");
    if (coverage_planner_) {
      core::PlannerConfig pcfg;
      pcfg.name = "astar";
      pcfg.resolution = 0.05;
      pcfg.max_iterations = planner_max_iterations_;
      pcfg.timeout = planner_timeout_;
      coverage_planner_->initialize(pcfg);
      coverage_planner_->setInflationRadius(0.0);
      coverage_planner_->setRobotRadius(0.0);
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
      ccfg.path_resample_step = path_resample_step_;
      applyMppiParams(ccfg);
      coverage_controller_->initialize(ccfg);
    } else {
      ROS_WARN("[coverage] controller '%s' not found", cov_ctrl.c_str());
    }

    ROS_INFO("[rosiwit_navigation_ros1] footprint_aware: requested=%d active=%d controller=%s",
             static_cast<int>(footprint_aware_control_),
             static_cast<int>(!controller_footprint_.empty()),
             controller_name_.c_str());
    // 按 profile 应用规划器/控制器的倒车策略 (启动即生效)
    applyMotionProfile(motion_profile_);
  }

  // 控制器/规划器的统一装配 (启动与运行时切换共用)
  void applyMppiParams(core::ControllerConfig & c)
  {
    c.mppi_horizon = mppi_horizon_;
    c.mppi_samples = mppi_samples_;
    c.mppi_dt = mppi_dt_ > 0.0 ? mppi_dt_ : 1.0 / std::max(1.0, controller_frequency_);
    c.mppi_lambda = mppi_lambda_;
    c.mppi_sigma_v = mppi_sigma_v_;
    c.mppi_sigma_w = mppi_sigma_w_;
    c.mppi_path_weight = mppi_path_weight_;
    c.mppi_heading_weight = mppi_heading_weight_;
    c.mppi_obstacle_weight = mppi_obstacle_weight_;
    c.mppi_control_weight = mppi_control_weight_;
    c.mppi_speed_weight = mppi_speed_weight_;
    c.mppi_goal_weight = mppi_goal_weight_;
  }

  bool initController(const std::string & name,
                      std::unique_ptr<core::IControllerStrategy> & out,
                      bool allow_reverse)
  {
    out = nav_core::NavigationFactory::createController(name);
    if (!out) return false;
    // footprint-aware 只对真正实现了 footprint 代价的控制器生效.
    controller_footprint_.clear();
    if (footprint_aware_control_ && name != "dwa" && out->supportsFootprint()) {
      controller_footprint_ = configured_footprint_;
    }
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
    cfg.path_resample_step = path_resample_step_;
    cfg.allow_reverse = allow_reverse;
    applyMppiParams(cfg);
    out->initialize(cfg);
    out->setVelocityLimit(max_velocity_x_, max_velocity_theta_);
    return true;
  }

  // ---- 运动 profile / 控制器运行时切换 ----
  // clean: 后面挂清洁组件, 只允许前进 (倒车只在 recovery);
  // free : 自由导航, 允许前进/倒车灵活走位.
  bool switchController(const std::string & name)
  {
    if (name.empty()) return true;
    if (name == controller_name_ && controller_strategy_) return true;
    std::unique_ptr<core::IControllerStrategy> next;
    if (!initController(name, next, motion_profile_ != "clean")) {
      ROS_WARN("[rosiwit_navigation_ros1] unknown controller: %s", name.c_str());
      return false;
    }
    if (!current_path_.poses.empty()) { next->setPath(toCorePath(current_path_)); }
    next->setVelocityLimit(max_velocity_x_, max_velocity_theta_);
    controller_strategy_ = std::move(next);
    controller_name_ = name;
    ROS_WARN("[rosiwit_navigation_ros1] controller -> %s (profile=%s)",
             controller_name_.c_str(), motion_profile_.c_str());
    controller_strategy_->setVelocityLimit(max_velocity_x_, max_velocity_theta_);
    // 能力变化可能改变规划器的双向策略 (例如 MPPI -> RPP 才允许倒车)
    applyMotionProfile(motion_profile_);
    return true;
  }

  bool applyMotionProfile(const std::string & profile)
  {
    if (profile != "free" && profile != "clean") {
      ROS_WARN("[rosiwit_navigation_ros1] unknown profile: %s", profile.c_str());
      return false;
    }
    motion_profile_ = profile;
    // 能力契约: 双向路径只有当控制器支持 cusp 跟踪时才允许规划.
    // MPPI/DWA/pure_pursuit 不支持 -> 规划器只生成单向路径 + 原地旋转.
    const bool ctrl_multi = controller_strategy_ &&
                            controller_strategy_->supportsMultiDirection();
    const bool allow_reverse = (profile != "clean") && ctrl_multi;
    // 规划器: clean 用纯前进 A*; free 用 Hybrid A* (允许倒车)
    const std::string planner = allow_reverse ? free_planner_ : clean_planner_;
    if (async_planner_ &&
        (planner != planner_name_ || allow_reverse != planner_allow_reverse_)) {
      auto next = nav_core::NavigationFactory::createPlanner(planner);
      if (!next) {
        ROS_WARN("[rosiwit_navigation_ros1] planner '%s' not found", planner.c_str());
      } else {
        core::PlannerConfig acfg;
        acfg.name = planner;
        acfg.resolution = 0.05;
        acfg.max_iterations = planner_max_iterations_;
        acfg.timeout = planner_timeout_;
        acfg.allow_reverse = allow_reverse;
        acfg.max_reverse_distance = planner_max_reverse_distance_;
        acfg.start_relax_radius = planning_start_relax_radius_;
        next->initialize(acfg);
        next->setInflationRadius(inflation_radius_);
        next->setRobotRadius(robot_radius_);
        if (next->supportsFootprint() && !configured_footprint_.empty()) {
          next->setRobotFootprint(planner_footprint_);
        }
        if (m_has_planner_costmap_) { next->setCostmap(planner_costmap_); }
        async_planner_ = std::move(next);
        planner_name_ = planner;
        planner_allow_reverse_ = allow_reverse;
      }
    }
    // 控制器重建: allow_reverse 在 initialize 时固化
    std::unique_ptr<core::IControllerStrategy> next_ctrl;
    if (initController(controller_name_, next_ctrl, allow_reverse)) {
      if (!current_path_.poses.empty()) { next_ctrl->setPath(toCorePath(current_path_)); }
      controller_strategy_ = std::move(next_ctrl);
    }    ROS_WARN("[rosiwit_navigation_ros1] profile -> %s (allow_reverse=%d planner=%s)",
             motion_profile_.c_str(), static_cast<int>(allow_reverse), planner_name_.c_str());
    return true;
  }

  void profileCallback(const std_msgs::String::ConstPtr & msg)
  {
    // 格式: "clean" | "free" | "clean,mppi" | ",dwa" (逗号后为控制器名, 可省略)
    std::string s = msg->data;
    std::string profile = s, ctrl;
    const auto comma = s.find(',');
    if (comma != std::string::npos) {
      profile = s.substr(0, comma);
      ctrl = s.substr(comma + 1);
    }
    if (!profile.empty() && !applyMotionProfile(profile)) { return; }
    if (!ctrl.empty()) { switchController(ctrl); }
  }

  void setupRos()
  {
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>(local_path_topic_, 1);
    global_path_pub_ = nh_.advertise<nav_msgs::Path>(global_path_topic_, 1);
    global_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(global_costmap_topic_, 1, true);
    local_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(local_costmap_topic_, 1, true);
    goal_status_pub_ = nh_.advertise<std_msgs::String>(goal_status_topic_, 1, true);
    diagnostics_pub_ = nh_.advertise<std_msgs::String>(diagnostics_topic_, 1);
    tracked_obs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(tracked_obstacles_topic_, 1);
    // 车体轮廓: idle 也按 1Hz 发布, 供 RViz 调试 (与兼容层同话题, 数据一致)
    footprint_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("robot_footprint", 1);

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &RosiwitNavigationRos1Node::odomCallback, this);
    map_sub_ = nh_.subscribe(map_topic_, 1, &RosiwitNavigationRos1Node::mapCallback, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 1, &RosiwitNavigationRos1Node::scanCallback, this);
    // 禁行区 (App/遥控器画的虚拟墙): 多边形列表, 直接进入两个代价地图视图.
    if (!virtual_obstacles_topic_.empty()) {
      virtual_obstacles_sub_ = nh_.subscribe(
        virtual_obstacles_topic_, 1,
        &RosiwitNavigationRos1Node::virtualObstaclesCallback, this);
    }
    // 分区限速: "x,y,w,h,v;x,y,w,h,v" (空串=清空). 与 yaml 的 speed_limit_zones 同格式.
    speed_limit_sub_ = nh_.subscribe(
      "/speed_limit_zones", 1, &RosiwitNavigationRos1Node::speedLimitCallback, this);
    // 定位健康: LOST -> 停车; matching 低 -> 限速
    slam_info_sub_ = nh_.subscribe("/slam/info", 1,
                                   &RosiwitNavigationRos1Node::slamInfoCallback, this);
    // 新契约优先: 位姿/协方差/健康等级/匹配度/地图标识一次拿全.
    localization_state_sub_ = nh_.subscribe(
      localization_state_topic_, 1,
      &RosiwitNavigationRos1Node::localizationStateCallback, this);
    // 运行时切换运动 profile / 控制器: std_msgs/String "clean|free[,controller]"
    profile_sub_ = nh_.subscribe("/set_motion_profile", 1,
                                 &RosiwitNavigationRos1Node::profileCallback, this);
    if (voxel_enabled_ && !voxel_cloud_topic_.empty()) {
      voxel_sub_ = nh_.subscribe(voxel_cloud_topic_, 1,
                                 &RosiwitNavigationRos1Node::voxelCallback, this);
    }
    goal_sub_ = nh_.subscribe(goal_topic_, 1, &RosiwitNavigationRos1Node::goalCallback, this);
    if (!rviz_goal_topic_.empty() && rviz_goal_topic_ != "/goal_pose") {
      rviz_goal_sub_ = nh_.subscribe(
        rviz_goal_topic_, 1, &RosiwitNavigationRos1Node::rvizGoalCallback, this);
    }
    nav_stop_srv_ = nh_.advertiseService(
      nav_stop_service_, &RosiwitNavigationRos1Node::navStopCallback, this);

    control_timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(1.0, controller_frequency_)),
      &RosiwitNavigationRos1Node::controlLoop, this);
    local_costmap_timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(0.5, costmap_update_rate_)),
      &RosiwitNavigationRos1Node::publishLocalCostmap, this);
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
    last_map_stamp_ = ros::Time::now();
    if (coverage_active_ && coverage_grid_.ready()) {
      coverage_grid_.updateStaticObstacles(msg->data);
    }
    last_global_costmap_update_ = ros::Time(0);
    updateCostmap();
    ROS_INFO("[rosiwit_navigation_ros1] map received: %d x %d, res=%.3f",
             static_cast<int>(msg->info.width), static_cast<int>(msg->info.height),
             msg->info.resolution);
  }

  void virtualObstaclesCallback(
    const xzrobot_ros_dependency::VirtualObstacles::ConstPtr & msg)
  {
    if (!keepout_layer_) { return; }
    // VirtualForm.form 是多边形顶点; 少于 3 点无法围成区域.
    bool need_tf = !keepout_frame_.empty() && keepout_frame_ != map_frame_;
    geometry_msgs::TransformStamped tf;
    if (need_tf) {
      try {
        tf = tf_buffer_.lookupTransform(map_frame_, keepout_frame_, ros::Time(0),
                                        ros::Duration(0.1));
      } catch (tf2::TransformException & ex) {
        ROS_WARN_THROTTLE(2.0,
          "[rosiwit_navigation_ros1] keepout tf %s->%s failed: %s (ignore)",
          keepout_frame_.c_str(), map_frame_.c_str(), ex.what());
        return;
      }
    }
    std::vector<std::vector<core::Point2D>> polygons;
    polygons.reserve(msg->list.size());
    for (const auto & form : msg->list) {
      if (form.form.size() < 3) { continue; }
      std::vector<core::Point2D> poly;
      poly.reserve(form.form.size());
      for (const auto & p : form.form) {
        geometry_msgs::Point mp = p;
        if (need_tf) {
          geometry_msgs::PointStamped in, out;
          in.point = p;
          tf2::doTransform(in, out, tf);
          mp = out.point;
        }
        poly.push_back({mp.x, mp.y});
      }
      polygons.push_back(std::move(poly));
    }
    keepout_layer_->setPolygons(polygons);
    // 禁行区变化必须立刻重建规划视图, 否则最多等 1Hz 的常规刷新.
    last_global_costmap_update_ = ros::Time(0);
    updateCostmap();
    ROS_INFO("[rosiwit_navigation_ros1] virtual obstacles -> %zu keepout polygons",
             keepout_layer_->polygonCount());
    if (keepout_layer_->polygonCount() > 0) {
      // stamp 在 updateCostmap() 里执行; 这里读的是刚算完的结果.
      ROS_INFO("[rosiwit_navigation_ros1] keepout marked %u cells (view %ux%u @ %.3f)",
               keepout_layer_->lastMarkedCells(),
               planner_layers_.config().width, planner_layers_.config().height,
               planner_layers_.config().resolution);
    }
  }

  void speedLimitCallback(const std_msgs::String::ConstPtr & msg)
  {
    std::vector<core::SpeedLimitLayer::Zone> zones;
    std::stringstream ss(msg->data);
    std::string item;
    while (std::getline(ss, item, ';')) {
      if (item.empty()) { continue; }
      core::SpeedLimitLayer::Zone zone;
      if (core::SpeedLimitLayer::parseRectZone(item, zone, max_velocity_x_)) {
        zones.push_back(zone);
      } else {
        ROS_WARN("[rosiwit_navigation_ros1] bad speed limit zone: %s", item.c_str());
      }
    }
    speed_limit_layer_.setZones(zones);
    ROS_INFO("[rosiwit_navigation_ros1] speed limit zones -> %zu", zones.size());
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr & msg)  {
    geometry_msgs::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id,
                                      msg->header.stamp, ros::Duration(0.1));
    } catch (tf2::TransformException &) {
      return;
    }
    // 只有成功转换到 map 的扫描才算 fresh; 否则 safety/局部 costmap
    // 会误以为仍有实时数据.
    last_scan_stamp_ = ros::Time::now();

    // /scan 与 3D 前端使用同一套 base 系车体包围盒自过滤.
    geometry_msgs::TransformStamped base_tf;
    if (scan_self_filter_enabled_) {
      try {
        base_tf = tf_buffer_.lookupTransform(robot_frame_, msg->header.frame_id,
                                             msg->header.stamp, ros::Duration(0.05));
      } catch (tf2::TransformException &) {
        return;
      }
    }

    const double tyaw = yawFromQuat(tf.transform.rotation);
    const double tx = tf.transform.translation.x;
    const double ty = tf.transform.translation.y;
    const double cy = std::cos(tyaw);
    const double sy = std::sin(tyaw);

    // /scan 正常约 1000 点，但异常/测试源可能给出超大 ranges。vector::clear()
    // 只清元素不释放容量，一次异常大帧就会把容量永久撑到数 GB。这里做
    // 上限 + stride 抽样，并在容量异常时真正 shrink.
    constexpr size_t kMaxScanPoints = 4000;
    laser_points_.clear();
    tracker_points_.clear();
    if (laser_points_.capacity() > kMaxScanPoints * 4) {
      laser_points_.shrink_to_fit();
    }
    if (tracker_points_.capacity() > kMaxScanPoints * 4) {
      tracker_points_.shrink_to_fit();
    }
    const size_t total_ranges = msg->ranges.size();
    const size_t scan_stride = total_ranges > kMaxScanPoints ?
      std::max<size_t>(1, total_ranges / kMaxScanPoints) : 1;
    double angle = msg->angle_min;
    for (size_t i = 0; i < total_ranges; i += scan_stride,
         angle += msg->angle_increment * static_cast<double>(scan_stride)) {
      const float r = msg->ranges[i];
      if (r < msg->range_min || r > msg->range_max) { continue; }
      const double lx = r * std::cos(angle);
      const double ly = r * std::sin(angle);
      if (scan_self_filter_enabled_) {
        geometry_msgs::PointStamped lp;
        lp.header = msg->header;
        lp.point.x = lx;
        lp.point.y = ly;
        geometry_msgs::PointStamped bp;
        tf2::doTransform(lp, bp, base_tf);
        if (bp.point.x >= scan_self_x_min_ && bp.point.x <= scan_self_x_max_ &&
            bp.point.y >= scan_self_y_min_ && bp.point.y <= scan_self_y_max_) {
          continue;
        }
      }
      geometry_msgs::Point p;
      p.x = tx + lx * cy - ly * sy;
      p.y = ty + lx * sy + ly * cy;
      p.z = 0.0;
      laser_points_.push_back(p);
    }
    // 孤立点滤波: 真实障碍在相邻扫描束上会有邻居; 单点噪声/反光点没有.
    if (laser_filter_enabled_ && laser_points_.size() >= 3) {
      std::vector<geometry_msgs::Point> filtered;
      filtered.reserve(laser_points_.size());
      const int window = std::max(1, laser_filter_window_);
      const int min_neighbors = std::max(1, laser_filter_min_neighbors_);
      const double radius2 = laser_filter_radius_ * laser_filter_radius_;
      for (size_t i = 0; i < laser_points_.size(); ++i) {
        int neighbors = 0;
        for (int k = -window; k <= window && neighbors < min_neighbors; ++k) {
          if (k == 0) { continue; }
          const long j = static_cast<long>(i) + k;
          if (j < 0 || j >= static_cast<long>(laser_points_.size())) { continue; }
          const double dx = laser_points_[i].x - laser_points_[j].x;
          const double dy = laser_points_[i].y - laser_points_[j].y;
          if (dx * dx + dy * dy <= radius2) { ++neighbors; }
        }
        if (neighbors >= min_neighbors) { filtered.push_back(laser_points_[i]); }
      }
      laser_points_.swap(filtered);
    }
    if (tracking_enabled_) {
      tracker_points_.clear();
      tracker_points_.reserve(laser_points_.size());
      for (const auto & p : laser_points_) {
        core::Point2D tp; tp.x = p.x; tp.y = p.y;
        tracker_points_.push_back(tp);
      }
      tracker_.update(tracker_points_, msg->header.stamp.toSec());
      tracker_points_.clear();
      publishTrackedObstacles();
    }
    const ros::Time now = ros::Time::now();
    const double period = costmap_update_rate_ > 0.0 ?
      1.0 / costmap_update_rate_ : 0.0;
    if (period <= 0.0 || last_costmap_build_.isZero() ||
        (now - last_costmap_build_).toSec() >= period) {
      updateCostmap();
      last_costmap_build_ = now;
    }
  }

  void voxelCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
  {
    if (!voxel_enabled_) return;
    bool has_xyz = false;
    for (const auto & f : msg->fields) {
      if (f.name == "x") has_xyz = true;
    }
    if (!has_xyz) return;
    last_cloud_stamp_ = ros::Time::now();

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

    // /world_cloud can be a very large accumulated cloud.  Never materialize
    // the whole cloud: cap the number of points and stride-sample large frames.
    // Otherwise a single callback can allocate GBs and push the whole robot
    // into swap (which also delays the control/safety timers).
    const size_t total_points = static_cast<size_t>(msg->width) * msg->height;
    const size_t max_points = static_cast<size_t>(
      std::max(1000, voxel_max_points_));
    const size_t stride = total_points > max_points ?
      std::max<size_t>(1, total_points / max_points) : 1;
    const size_t reserve_count = std::min(total_points, max_points);
    std::vector<core::Point3D> points;
    points.reserve(reserve_count);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
    size_t index = 0;
    for (; ix != ix.end() && points.size() < max_points;
         ++ix, ++iy, ++iz, ++index) {
      if (stride > 1 && (index % stride) != 0) continue;
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

  // Common navigate-task entry.  /goal_pose and RobotNaviCmdAction both
  // reach this function; the only differences are already encoded in the
  // ActionTaskCommand, the start pose override and the result adapter.
  bool beginNavigationTask(const geometry_msgs::PoseStamped & goal,
                           const core::ActionTaskCommand & command,
                           const core::Pose2D & start,
                           bool allow_initial_rotate,
                           const core::Pose2D * start_override,
                           double speed)
  {
    task_active_ = true;
    recovery_attempts_ = 0;
    action_speed_ = speed > 0.01 ? speed : movebase_default_speed_;
    if (action_speed_ > max_velocity_x_) {
      action_speed_ = max_velocity_x_;
    }
    if (controller_strategy_) {
      controller_strategy_->setVelocityLimit(action_speed_, max_velocity_theta_);
    }
    action_traveled_.clear();
    action_traveled_.push_back(current_pose_);
    if (!action_task_.start(command, start, ros::Time::now().toSec())) {
      failTask(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
      return false;
    }
    goal_pose_ = goal;
    if (!current_path_.poses.empty()) {
      current_path_.poses.clear();
      global_path_pub_.publish(current_path_);
    }
    setGoalStatus("ACCEPTED");
    startPlanning(goal_pose_, allow_initial_rotate, /*restart=*/true, start_override);
    return true;
  }

  // 原生 goal (RViz /goal_pose 或 /move_base_simple/goal) 统一走 ActionTask:
  // 与 action 单点共用进度/卡死/末端航向逻辑, 只是没有 action handle.
  void startNativeGoal(const geometry_msgs::PoseStamped & goal,
                       const char * source, bool align_yaw)
  {
    if (task_active_) {
      ROS_WARN("[rosiwit_navigation_ros1] 收到 %s, 结束进行中的任务", source);
      clearActionState();
      if (mb_) { mb_->preempted(); }
    }
    if (!updateCurrentPose()) {
      setGoalStatus("FAILED_POSE");
      return;
    }

    core::ActionTaskCommand task_command;
    task_command.kind = core::ActionKind::NAVIGATE;
    task_command.final_rotate = align_yaw;
    task_command.final_yaw = yawFromQuat(goal.pose.orientation);

    action_interface_active_ = false;
    action_goal_ = movebase::Goal();
    action_cmd_ = movebase::Decoded();
    ROS_INFO("[rosiwit_navigation_ros1] %s goal: (%.2f, %.2f, yaw=%.2f) align_yaw=%d",
             source, goal.pose.position.x, goal.pose.position.y,
             task_command.final_yaw, static_cast<int>(align_yaw));
    beginNavigationTask(goal, task_command, toCorePose(current_pose_),
                        /*allow_initial_rotate=*/true, nullptr,
                        movebase_default_speed_);
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
  {
    geometry_msgs::PoseStamped goal = *msg;
    if (!transformGoalToMap(goal, "goal_pose")) {
      setGoalStatus("REJECTED_FRAME");
      return;
    }
    startNativeGoal(goal, "goal_pose", /*align_yaw=*/true);
  }

  void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
  {
    geometry_msgs::PoseStamped goal = *msg;
    if (!transformGoalToMap(goal, "rviz_goal")) {
      setGoalStatus("REJECTED_FRAME");
      return;
    }
    startNativeGoal(goal, "rviz_goal", rviz_goal_align_yaw_);
  }

  bool navStopCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response & res)
  {
    stopNavigation();
    if (task_active_) {
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
  void publishCommand(const geometry_msgs::Twist & cmd_in)
  {
    // 分区限速在唯一出口处生效: 控制器/末段精调/恢复/兼容层都走这里.
    const geometry_msgs::Twist cmd = applySpeedLimit(cmd_in);
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
    action_interface_active_ = true;
    action_traveled_.clear();
    action_goal_ = goal;
    action_cmd_ = movebase::decodeGoal(goal);

    if (!action_cmd_.valid || !action_cmd_.supported) {
      ROS_WARN("[movebase_iface] navi_cmd=%d 当前核心不支持或参数非法, 回 result=%d",
               goal.navi_cmd, action_cmd_.reject_result);
      finishTask(action_cmd_.reject_result);
      return;
    }
    if (action_cmd_.command == movebase::Command::CANCEL) {
      stopNavigation();
      finishTask(movebase::RESULT_SUCCESS);
      return;
    }

    if (!updateCurrentPose()) {
      ROS_ERROR("[movebase_iface] 获取坐标失败");
      failTask(movebase::RESULT_POSE_ERROR, "FAILED_POSE");
      return;
    }
    if (!current_map_) {
      ROS_ERROR("[movebase_iface] 全局地图未就绪");
      failTask(movebase::RESULT_GLOBAL_MAP_ERROR, "FAILED_MAP");
      return;
    }

    task_active_ = true;
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
        failTask(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
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
        failTask(movebase::RESULT_FILE_ERROR, "FAILED_FILE");
        return;
      }
      if (segments.empty()) {
        failTask(movebase::RESULT_FILE_ERROR, "FAILED_FILE");
        return;
      }
      const size_t seg = std::min<size_t>(
        static_cast<size_t>(action_cmd_.path_segment_index), segments.size() - 1);
      const std::vector<core::Pose2D> tail =
        movebase::slicePath(segments[seg], action_cmd_.path_index);
      if (tail.empty()) {
        failTask(movebase::RESULT_FILE_ERROR, "FAILED_FILE");
        return;
      }
      target = tail.front();
    }

    const core::Pose2D start = action_cmd_.has_start
      ? action_cmd_.start : toCorePose(current_pose_);
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = map_frame_;
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = target.x;
    goal.pose.position.y = target.y;
    goal.pose.position.z = 0.0;
    goal.pose.orientation = quatFromYaw(target.theta);

    const core::Pose2D * start_override = action_cmd_.has_start ? &action_cmd_.start : nullptr;
    // 两点导航的起点由调用方给定, 与车头朝向无关, 不做起步原地旋转.
    const bool allow_initial_rotate =
      (action_cmd_.command == movebase::Command::SINGLE_POINT);
    beginNavigationTask(goal, toTaskCommand(action_cmd_, target), start,
                        allow_initial_rotate, start_override, action_speed_);
  }

  void startActionFollowPath()
  {
    if (!controller_strategy_) {
      failTask(movebase::RESULT_LOCAL_MAP_ERROR, "FAILED_CONTROLLER");
      return;
    }

    std::vector<core::Pose2D> path = action_cmd_.path;
    if (action_cmd_.use_file) {
      std::vector<std::vector<core::Pose2D>> segments;
      std::string error;
      if (!movebase::loadPathFile(action_goal_.file_path, segments, &error)) {
        ROS_ERROR("[movebase_iface] 加载路径文件失败: %s", error.c_str());
        failTask(movebase::RESULT_FILE_ERROR, "FAILED_FILE");
        return;
      }
      if (segments.empty()) {
        failTask(movebase::RESULT_FILE_ERROR, "FAILED_FILE");
        return;
      }
      const size_t seg = std::min<size_t>(
        static_cast<size_t>(action_cmd_.path_segment_index), segments.size() - 1);
      path = movebase::slicePath(segments[seg], action_cmd_.path_index);
    }
    if (path.size() < 2) {
      ROS_ERROR("[movebase_iface] 跟踪路径点数不足 (%zu)", path.size());
      failTask(movebase::RESULT_POSE_ERROR, "FAILED_POSE");
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

    core::ActionTaskCommand task_command;
    task_command.kind = core::ActionKind::FOLLOW_PATH;
    if (!action_task_.start(task_command, toCorePose(current_pose_),
                            ros::Time::now().toSec())) {
      failTask(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
      return;
    }

    controller_strategy_->setPath(toCorePath(current_path_));
    global_path_pub_.publish(current_path_);

    planning_active_ = false;
    is_navigating_ = true;
    is_paused_ = false;
    m_need_initial_rotate_ = false;
    ROS_INFO("[movebase_iface] 开始跟踪路径: %zu 点, %.2f m",
             current_path_.poses.size(), movebase::pathLength(path));
  }

  void startActionRotate()
  {
    core::ActionTaskCommand task_command;
    task_command.kind = core::ActionKind::ROTATE;
    const core::Pose2D pose = toCorePose(current_pose_);
    if (!action_task_.start(task_command, pose, ros::Time::now().toSec())) {
      failTask(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
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
      failTask(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
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
    task_active_ = false;
    action_interface_active_ = false;
    action_task_.reset();
    // action_goal_/action_cmd_ 可能持有 FOLLOW_PATH/COVERAGE 的整条路径.
    // 任务结束后必须释放, 否则一次异常大 goal 会永久占住内存.
    action_goal_ = movebase::Goal();
    action_cmd_ = movebase::Decoded();
    action_traveled_.clear();
    resetRecovery();
    recovery_attempts_ = 0;
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
    return task_active_ ? action_task_.progress() : 0.0;
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

  // One completion path for /goal_pose and RobotNaviCmdAction.  The action
  // server is only an output adapter; behaviour/state lives in ActionTask.
  void finishTask(int result_code, const std::string & status = "SUCCEEDED")
  {
    if (action_interface_active_ && mb_ && mb_->hasGoal()) {
      mb_->succeed(actionOutcome(result_code));
    }
    setGoalStatus(status);
    clearActionState();
    stopNavigation();
  }

  void failTask(int result_code, const std::string & status)
  {
    finishTask(result_code, status);
  }

  // 只做"已走过轨迹"的展示采样; 进度/卡死判定在 core::ActionTask.
  void recordActionTraveled()
  {
    if (!task_active_) { return; }
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
    if (!action_interface_active_ || !mb_) { return; }
    // footprint 不依赖任务状态: 有订阅者就发, 方便 RViz 在 idle 时也能看到车体轮廓
    mb_->publishFootprints(current_pose_, current_velocity_, movebase_footprint_);
    if (!task_active_) { return; }
    mb_->publishFeedback(actionFeedback());
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
      failTask(movebase::RESULT_GLOBAL_MAP_ERROR, "FAILED_MAP");
      return;
    }
    if (action_cmd_.polygon.size() < 3) {
      ROS_ERROR("[coverage] 多边形顶点不足 (%zu)", action_cmd_.polygon.size());
      failTask(movebase::RESULT_POSE_ERROR, "FAILED_COVERAGE_POLYGON");
      return;
    }
    if (!updateCurrentPose()) {
      failTask(movebase::RESULT_POSE_ERROR, "FAILED_POSE");
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
    // 但**离墙**必须有车体半径的内缩: 否则行会贴着墙选, 车体站不下,
    // 同一行被反复规划直到超时 (实测卡在 y=-0.29 那一行, 覆盖率 13%).
    {
      double half_width = 0.0;
      for (const auto & p : configured_footprint_) {
        half_width = std::max(half_width, std::fabs(p.second));
      }
      if (half_width <= 0.01) { half_width = robot_radius_ * 0.85; }
      coverage_config_.robot_inset = half_width;
      ROS_INFO("[coverage] robot_inset=%.2f m (footprint half-width)", half_width);
    }
    if (!coverage_grid_.initialize(info, action_cmd_.polygon, coverage_config_)) {
      failTask(movebase::RESULT_POSE_ERROR, "FAILED_COVERAGE_AREA");
      return;
    }
    coverage_grid_.updateStaticObstacles(current_map_->data);
    {
      const auto st0 = coverage_grid_.stats();
      const double r0 = current_map_->info.resolution;
      ROS_INFO("[coverage] grid stats: total=%.2f m2 uncovered=%.2f obstacle=%.2f "
               "unknown=%.2f outside=%.2f blocked=%.2f",
               st0.total * r0 * r0, st0.uncovered * r0 * r0, st0.obstacle * r0 * r0,
               st0.unknown * r0 * r0, st0.outside * r0 * r0, st0.blocked * r0 * r0);
    }
    const core::Pose2D pose = toCorePose(current_pose_);
    if (!coverage_grid_.contains(pose)) {
      ROS_ERROR("[coverage] 起点 (%.2f, %.2f) 不在区域内或落在障碍上",
                pose.x, pose.y);
      failTask(movebase::RESULT_PLAN_FAILED, "FAILED_COVERAGE_START");
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
      // 只把**真正在动的目标**当动态障碍. 以前这里喂的是全部激光点, 等于把
      // 静态墙也标成 dynamic_; 而 markDisk() 用 skip_dynamic=true 记录覆盖,
      // 于是靠墙的格子永远标记不成 "已覆盖" -> 同一行反复规划, 最后到
      // 81% 就放弃 (2026-10-25 实测: 同一行被规划 3 次).
      std::vector<core::Pose2D> dynamic_points;
      if (tracking_enabled_) {
        for (const auto & t : tracker_.tracked(true)) {
          dynamic_points.emplace_back(t.x, t.y, 0.0);
        }
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
    if (coverage_last_diag_.isZero() || (now - coverage_last_diag_).toSec() >= 2.0) {
      // 诊断: dynamic_cells 若长期 >0, 说明跟踪器把静态环境当成了动态目标,
      // 而 markDisk(skip_dynamic=true) 会跳过这些格子 -> 扫过也不算覆盖.
      const auto r = current_map_->info.resolution;
      ROS_INFO("[coverage] diag: covered=%.2f/%.2f m2 rate=%.1f%% dynamic=%zu "
               "state=%s pose=(%.2f,%.2f)",
               stats.covered * r * r, stats.total * r * r,
               stats.coverageRate() * 100.0, coverage_grid_.dynamicCount(),
               coverage_state_.c_str(), pose.x, pose.y);
      coverage_last_diag_ = now;
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
      // 预测时域 0.1s 太短 (0.3m/s 只有 3cm), 控制超调后车体早已出界;
      // 而且原来只在 `contains(pose)` 成立时才守 —— 一旦出界就完全不守了
      // (实测跑到 x=2.53, 区域边界 1.5, 然后在区域外原地死等).
      {
        const double dt = 0.6;
        core::Pose2D next(pose.x + cmd.linear.x * std::cos(pose.theta) * dt,
                          pose.y + cmd.linear.x * std::sin(pose.theta) * dt,
                          pose.theta + cmd.angular.z * dt);
        const bool inside_now = coverage_grid_.contains(pose);
        const bool inside_next = coverage_grid_.contains(next);
        if (!inside_now) {
          // 罕见情况: 被超调带到界外. 朝区域内目标"边转边走"(限速), 不要冻结.
          // 界外格子是 OUTSIDE, markDisk 会跳过, 不会把界外算成已清洁.
          const double tx = coverage_target_.x - pose.x;
          const double ty = coverage_target_.y - pose.y;
          const double err = normalizeAngle(std::atan2(ty, tx) - pose.theta);
          cmd.linear.x = (std::fabs(err) < 0.6) ? 0.12 : 0.0;
          cmd.angular.z = (err >= 0.0 ? 1.0 : -1.0) *
            std::min(max_velocity_theta_, std::max(0.25, 1.5 * std::fabs(err)));
          coverage_state_ = "OUT_OF_AREA";
        }
        // 注意: 以前这里还有 "下一步出界就停 v" 的 BOUNDARY 分支 —— 它是死锁源:
        // 机器人正好压在边界上、路径是直线 (w≈0) 时会被冻结在边界上.
        // 现在改由几何保证不出界: 行线内缩 sweep/2 (0.25m) + 端点内缩 0.20m
        // + coverage_max_speed 0.25 (刹车距离 ~0.06m).
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
        !coverage_planner_ || !coverage_controller_) {
      return;
    }
    const ros::Time now = ros::Time::now();
    const core::Pose2D pose = toCorePose(current_pose_);
    std::vector<core::Pose2D> segment;
    if (!coverage_grid_.nextCoverageSegment(pose, coverage_horizon_, segment)) {
      coverage_last_plan_ = now;
      if (coverage_grid_.hasAnyUncovered()) {
        if ((now - coverage_last_progress_).toSec() > coverage_retry_timeout_) {
          // 只把"从当前位置确实到不了"的格子标成 blocked, 可达的先保留.
          // 以前直接 markAllUncoveredBlocked(), 一次把整块区域清空, 报表里
          // blocked 面积失真 (实测 6.49/7.5 m² 全变 blocked).
          {
            const auto st1 = coverage_grid_.stats();
            const double r1 = current_map_->info.resolution;
            ROS_WARN("[coverage] no segment: uncovered=%.2f obstacle=%.2f "
                     "unknown=%.2f blocked=%.2f total=%.2f (m2)",
                     st1.uncovered * r1 * r1, st1.obstacle * r1 * r1,
                     st1.unknown * r1 * r1, st1.blocked * r1 * r1,
                     st1.total * r1 * r1);
          }
          coverage_grid_.markUnreachable(pose);
          if (coverage_grid_.hasAnyUncovered() &&
              coverage_grid_.hasUncoveredReachable(pose, true)) {
            // 还有能到的未覆盖格 (刚才只是这一帧没找到行): 继续尝试
            coverage_state_ = "WAITING_OBSTACLE";
            current_path_.poses.clear();
            coverage_controller_->reset();
            coverage_last_progress_ = now;
          } else {
            finishCoverage();
          }
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
    // 行端点贴着多边形边界时, A* 会把 approach 目标判成 Goal in obstacle
    // (边界/膨胀带), 然后退回 "direct row" —— 路径从 0.8m 外的边界开始跟,
    // 控制器跟不好, 同一行会被反复规划 (实测 3 次) 最后只到 60~80%.
    // 这里把两端沿行方向内缩一点: 端点落回可通行栅格, 而 sweep_width/2 的
    // 清扫半径本来就能覆盖这 0.15m.
    if (segment.size() >= 2) {
      const double inset = std::max(0.20, coverage_blocked_inflation_);
      const double len = std::hypot(segment.back().x - segment.front().x,
                                    segment.back().y - segment.front().y);
      if (len > 3.0 * inset) {
        const double ux = (segment.back().x - segment.front().x) / len;
        const double uy = (segment.back().y - segment.front().y) / len;
        segment.front().x += ux * inset;
        segment.front().y += uy * inset;
        segment.back().x -= ux * inset;
        segment.back().y -= uy * inset;
      }
    }

    // 同一行反复被选中、覆盖量却不增 -> 这一条带确实到不了:
    // 只把它标成 blocked 换下一行, 不要整块放弃 (以前会一直重试到全局超时).
    {
      const int kx = static_cast<int>(std::lround(segment.back().x * 10.0));
      const int ky = static_cast<int>(std::lround(segment.back().y * 10.0));
      const size_t covered_now = coverage_grid_.stats().covered;
      const bool same_row = (kx == coverage_row_key_x_ && ky == coverage_row_key_y_);
      if (!same_row || covered_now > coverage_row_covered_) {
        coverage_row_key_x_ = kx;
        coverage_row_key_y_ = ky;
        coverage_row_covered_ = covered_now;
        coverage_row_since_ = now;
      } else if (!coverage_row_since_.isZero() &&
                 (now - coverage_row_since_).toSec() > coverage_retry_timeout_) {
        ROS_WARN("[coverage] row (%.2f,%.2f) no progress for %.1fs -> mark this "
                 "strip blocked, continue", segment.back().x, segment.back().y,
                 (now - coverage_row_since_).toSec());
        coverage_grid_.markBlockedAround(segment.back(),
                                         std::max(0.20, coverage_sweep_width_ * 0.5));
        coverage_row_since_ = ros::Time();
        coverage_last_progress_ = now;   // 不算"整块没进展", 避免全局放弃
        return;
      }
    }

    coverage_target_ = segment.back();
    coverage_has_target_ = true;

    core::Path path;
    if (pose.distanceTo(segment.front()) > 0.8) {
      core::Costmap costmap;
      buildCoverageCostmap(costmap);
      planner_strategy_->setInflationRadius(coverage_plan_inflation_);
      planner_strategy_->setRobotRadius(coverage_plan_robot_radius_);
      // 覆盖模式使用自己预先膨胀的边界; Hybrid A* 若继续带普通导航 footprint,
      // 会和 coverage_plan_robot_radius=0 的语义冲突. 这里临时清空轮廓,
      // 规划完恢复, 正常导航仍使用 footprint.
      if (planner_strategy_->supportsFootprint())
        planner_strategy_->setRobotFootprint({});
      planner_strategy_->setCostmap(costmap);
      const core::Result<core::Path> approach =
        planner_strategy_->plan(pose, segment.front());
      planner_strategy_->setInflationRadius(inflation_radius_);
      planner_strategy_->setRobotRadius(robot_radius_);
      if (planner_strategy_->supportsFootprint())
        planner_strategy_->setRobotFootprint(configured_footprint_);
      coverage_last_plan_ = now;
      if (!approach.is_ok()) {
        // 区域边缘的短 approach 在 A* 里可能被膨胀/边界误判; 直接沿行段走,
        // 由控制器和安全层兜底, 不要因此把整块标不可达.
        ROS_WARN("[coverage] approach to row (%.2f, %.2f) failed: %s; direct row",
                 segment.front().x, segment.front().y,
                 approach.error_message().c_str());
        path.points.clear();
        for (const auto & point : segment) {
          core::PathPoint pt;
          pt.pose = point;
          path.points.push_back(pt);
        }
      } else {
        path = approach.value();
        for (size_t i = 1; i < segment.size(); ++i) {
          core::PathPoint pt;
          pt.pose = segment[i];
          path.points.push_back(pt);
        }
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
        costmap.data[i] = 100;
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
    finishTask(movebase::RESULT_SUCCESS);
    const double res = current_map_ ? current_map_->info.resolution : 0.05;
    ROS_INFO("[coverage] done: rate=%.3f covered=%.2f m2 blocked=%.2f m2",
             stats.coverageRate(),
             static_cast<double>(stats.covered) * res * res,
             static_cast<double>(stats.blocked) * res * res);
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

  std::string navigationStateName() const
  {
    if (coverage_active_) { return coverage_state_; }
    if (recovery_stage_ != RecoveryStage::NONE) { return "RECOVERY"; }
    if (planning_active_) { return "PLANNING"; }
    if (is_navigating_) { return "NAVIGATING"; }
    if (goal_status_.rfind("FAILED", 0) == 0) { return "FAILED"; }
    if (goal_status_.rfind("STOPPED", 0) == 0) { return "STOPPED"; }
    return "IDLE";
  }

  void setGoalStatus(const std::string & status)
  {
    goal_status_ = status;
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

  void publishFootprintPolygon()
  {
    if (!footprint_pub_ || movebase_footprint_.empty()) { return; }
    geometry_msgs::PolygonStamped msg;
    msg.header = current_pose_.header;
    msg.header.stamp = ros::Time::now();
    const double yaw = yawFromQuat(current_pose_.pose.orientation);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    msg.polygon.points.reserve(movebase_footprint_.size());
    for (const auto & p : movebase_footprint_) {
      geometry_msgs::Point32 q;
      q.x = static_cast<float>(current_pose_.pose.position.x + p.x * cy - p.y * sy);
      q.y = static_cast<float>(current_pose_.pose.position.y + p.x * sy + p.y * cy);
      q.z = 0.0f;
      msg.polygon.points.push_back(q);
    }
    footprint_pub_.publish(msg);
  }

  void diagnosticsCallback(const ros::TimerEvent &)
  {    // footprint 与任务状态无关: 1Hz 直发 (体积极小), RViz 在 idle 时也能看到车体轮廓
    publishFootprintPolygon();
    if (!diagnostics_pub_) return;
    std::ostringstream os;
    os << "{\"state\":\"" << (is_navigating_ ? "NAVIGATING" : "IDLE")
       << "\",\"goal_status\":\"" << goal_status_
       << "\",\"safety\":\"" << last_safety_state_
       << "\",\"safety_scale\":" << last_safety_scale_
       << ",\"path_points\":" << current_path_.poses.size()
       << ",\"planner\":\"" << planner_name_ << "\""
       << ",\"profile\":\"" << motion_profile_ << "\""
       << ",\"controller\":\"" << controller_name_ << "\""
       << ",\"planner_footprint\":" << configured_footprint_.size()
       << ",\"tracks\":" << tracker_.tracked().size()
       << ",\"task_state\":\"" << navigationStateName() << "\""
       << ",\"voxels\":" << voxel_layer_.size()
       << ",\"keepout\":" << (keepout_layer_ ? keepout_layer_->polygonCount() : 0)
       << ",\"speed_zones\":" << speed_limit_layer_.size()
       << ",\"loc_health\":" << static_cast<int>(last_loc_health_)
       << ",\"loc_state\":" << (has_localization_state_ ? 1 : 0)
       << ",\"scan_pts\":" << laser_points_.size()
       << ",\"scan_cap\":" << laser_points_.capacity()
       << ",\"track_pts_cap\":" << tracker_points_.capacity()
       << ",\"map_w\":" << (current_map_ ? static_cast<int>(current_map_->info.width) : -1)
       << ",\"map_h\":" << (current_map_ ? static_cast<int>(current_map_->info.height) : -1)
       << ",\"goal_path\":" << action_goal_.tracking_path.size()
       << ",\"cmd_path\":" << action_cmd_.path.size()
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
    if (w == 0 || h == 0 ||
        static_cast<uint64_t>(w) * static_cast<uint64_t>(h) > 4000000ULL) {
      ROS_ERROR_THROTTLE(2.0,
        "[rosiwit_navigation_ros1] rejecting abnormal map %ux%u for costmap", w, h);
      return;
    }

    const bool need_global = last_global_costmap_update_.isZero() ||
      (ros::Time::now() - last_global_costmap_update_).toSec() >= 1.0;
    if (planner_strategy_ && need_global) {
      if (!planner_static_layer_ || !keepout_layer_) { return; }
      // B1: 规划视图 = StaticLayer + KeepoutLayer.
      // 未配置禁行区时输出与旧实现逐格相同 (0/254/255).
      core::LayerConfig cfg;
      cfg.width = w;
      cfg.height = h;
      cfg.resolution = res;
      cfg.map_origin_x = ox;
      cfg.map_origin_y = oy;
      cfg.window_x = 0;
      cfg.window_y = 0;
      planner_static_layer_->setOccupancy(current_map_->data, w, h);
      keepout_layer_->setActive(keepout_enabled_);
      planner_layers_.configure(cfg, core::kCostFree);
      planner_layers_.update(ros::Time::now().toSec());

      const std::vector<int8_t> & semantic = planner_layers_.combined();

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
      cm.data.assign(semantic.size(), 0);
      for (size_t i = 0; i < semantic.size(); ++i) {
        // 层与 Costmap 同一套语义 (-1/0..100), 直接复制.
        cm.data[i] = semantic[i];
      }
      // /scan 是实时传感器障碍层: 只进入 local costmap/controller/safety,
      // 不能直接写成 global planner 的 lethal。否则 15Hz 的瞬态点会被 A*
      // 当成全局永久障碍, 形成大面积假墙并让全局路径无法生成.
      planner_strategy_->setCostmap(cm);
      planner_costmap_ = cm;
      m_has_planner_costmap_ = true;

      // 膨胀并发布供 RViz 可视化
      const int inflate_cells = static_cast<int>(std::ceil(inflation_radius_ / res));
      const int safety_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_ / res)));
      const std::vector<int8_t> display =
        core::inflatedCostmap(semantic, w, h, safety_cells, inflate_cells);


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
      for (size_t i = 0; i < w * h; ++i) {
        // 层/膨胀输出已经是 nav_msgs 约定 (-1 unknown, 0..100), 直接发布.
        costmap_msg.data[i] = display[i];
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

    // /scan 停止后不能把最后一帧点云永久当成实时障碍; safety 会停车,
    // 局部 costmap/聚类也不能继续使用陈旧点.
    const bool scan_fresh = !last_scan_stamp_.isZero() &&
      (ros::Time::now() - last_scan_stamp_).toSec() <= safety_source_timeout_;

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

    // /scan 点先一次性标记到全图栅格(3x3), 否则 isObstacleCell 每次查询都
    // 全量扫描 ~1000 个点, 聚类 BFS 一次就是几十万次点遍历.
    std::vector<char> scan_cells;
    if (scan_fresh) {
      scan_cells.assign(static_cast<size_t>(w) * h, 0);
      for (const auto & pt : laser_points_) {
        const int lx = static_cast<int>((pt.x - ox) / res);
        const int ly = static_cast<int>((pt.y - oy) / res);
        for (int oy2 = -1; oy2 <= 1; ++oy2) {
          for (int ox2 = -1; ox2 <= 1; ++ox2) {
            const int mx = lx + ox2;
            const int my = ly + oy2;
            if (mx < 0 || mx >= static_cast<int>(w) ||
                my < 0 || my >= static_cast<int>(h)) { continue; }
            scan_cells[static_cast<size_t>(my) * w + mx] = 1;
          }
        }
      }
    }

    auto isObstacleCell = [&](int mx, int my) -> bool {
      if (mx < local_x0 || mx > local_x1 || my < local_y0 || my > local_y1) {
        return false;
      }
      if (current_map_->data[my * w + mx] >= 100) { return true; }
      return scan_fresh && scan_cells[my * w + mx] == 1;
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
    if (!local_static_layer_ || !local_scan_layer_ || !local_track_layer_ ||
        !local_voxel_layer_ || !keepout_layer_) { return; }
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

    // 与 updateCostmap 一致: 超时的 /scan 不再写入实时障碍.
    const bool scan_fresh = !last_scan_stamp_.isZero() &&
      (ros::Time::now() - last_scan_stamp_).toSec() <= safety_source_timeout_;

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

    // B2: 控制视图 = Static(window) + Scan + Tracked + Voxel + Keepout, 不膨胀.
    // 与规划视图共用同一批层, 只是窗口/子集/膨胀参数不同.
    core::LayerConfig cfg;
    cfg.width = static_cast<unsigned int>(lw);
    cfg.height = static_cast<unsigned int>(lw);
    cfg.resolution = res;
    cfg.map_origin_x = ox;
    cfg.map_origin_y = oy;
    cfg.window_x = cx - win_half;
    cfg.window_y = cy - win_half;

    local_static_layer_->setOccupancy(current_map_->data, w, h);
    if (scan_fresh) {
      scan_layer_points_.clear();
      scan_layer_points_.reserve(laser_points_.size());
      for (const auto & p : laser_points_) { scan_layer_points_.push_back({p.x, p.y}); }
      local_scan_layer_->setPoints(scan_layer_points_);
    } else {
      local_scan_layer_->clear();
    }
    if (tracking_enabled_) {
      local_track_layer_->setCircles(tracker_.predicted(tracking_horizon_));
    } else {
      local_track_layer_->clear();
    }
    if (voxel_enabled_) {
      local_voxel_layer_->setPoints(voxel_layer_.occupancyPoints());
    } else {
      local_voxel_layer_->clear();
    }
    keepout_layer_->setActive(keepout_enabled_);

    local_layers_.configure(cfg, core::kCostFree);
    local_layers_.update(ros::Time::now().toSec());
    std::vector<int8_t> raw = local_layers_.combined();

    // 窗口内但在地图外的栅格: 旧实现视为致命 (不可穿越), 保持等价.
    for (int y = 0; y < lw; ++y) {
      const int my = cy - win_half + y;
      for (int x = 0; x < lw; ++x) {
        const int mx = cx - win_half + x;
        if (mx < 0 || mx >= static_cast<int>(w) || my < 0 || my >= static_cast<int>(h)) {
          raw[static_cast<size_t>(y) * lw + x] = core::kCostLethal;
        }
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
    const std::vector<int8_t> ldisplay =
      core::inflatedCostmap(raw, static_cast<unsigned int>(lw),
                            static_cast<unsigned int>(lw), l_safety, l_inflate);

    local_costmap.data.resize(lw * lw);
    for (int i = 0; i < lw * lw; ++i) {
      // 膨胀安全环显示为 99, 真实障碍 raw 仍是 100.
      local_costmap.data[i] = ldisplay[i];
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
        grid.data.assign(raw.begin(), raw.end());
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
      if (task_active_) {
        failTask(movebase::RESULT_GLOBAL_MAP_ERROR, "FAILED_PLAN");
      } else {
        setGoalStatus("FAILED_PLAN");
        stopNavigation();
      }
      return;
    }
    const core::Pose2D start = pending_has_start_override_
      ? pending_start_override_ : toCorePose(current_pose_);
    const core::Pose2D goal = toCorePose(pending_goal_);
    const core::Costmap planning_costmap = buildStartClearedCostmap(
      start, planning_start_clear_margin_, planning_clear_start_footprint_);
    last_planning_costmap_ = planning_costmap;
    async_planner_->setCostmap(planning_costmap);
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
      last_motion_stamp_ = ros::Time::now();
      safety_stop_since_ = ros::Time();
    } else {
      is_navigating_ = true;  // 重规划期间旧路径继续可用
    }
    if (!updateCurrentPose()) {
      ROS_ERROR("[rosiwit_navigation_ros1] cannot get current pose for planning");
      if (task_active_) {
        failTask(movebase::RESULT_POSE_ERROR, "FAILED_POSE");
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
      if (startRecovery()) {
        ROS_WARN("[rosiwit_navigation_ros1] planning failed, reverse escape recovery then retry");
        return;
      }
      if (task_active_) {
        failTask(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
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
    core::Path planned = result.value();
    size_t reverse_points = 0;
    for (const auto & p : planned.points) {
      if (p.velocity_x < 0.0) { ++reverse_points; }
    }
    const bool first_reverse = planned.points.size() > 1 &&
      planned.points[1].velocity_x < 0.0;

    // Smooth positions for every path, then restore each point's heading and
    // forward/reverse sign.  Hybrid A* 路径本身就是圆弧基元, 已经平滑; 再平滑
    // 只会把路径拉进高代价膨胀区 (smoother 只看 raw grid 的 0/100, 看不到
    // 膨胀代价), 表现为"旁边有空地却贴着代价区走".
    bool smoothed = false;
    if (planned.points.size() >= 3 && planner_name_ != "hybrid_astar") {
      const core::Path raw = planned;
      planners::SmootherConfig smoother;
      smoother.max_iterations = 60;
      smoother.alpha_start = 0.45;
      smoother.alpha_end = 0.10;
      smoother.max_cost_increase = 0;
      core::Path candidate = planners::smoothPath(raw, planner_costmap_.grid, smoother);
      if (candidate.points.size() == raw.points.size()) {
        for (size_t i = 0; i < candidate.points.size(); ++i) {
          candidate.points[i].pose.theta = raw.points[i].pose.theta;
          candidate.points[i].velocity_x = raw.points[i].velocity_x;
        }
        planned = candidate;
        smoothed = true;
      }
    }

    current_path_ = toRosPath(planned, pending_goal_.header.frame_id, ros::Time::now());
    current_core_path_ = planned;   // 旋转段执行器读取 velocity_x==0 标记

    // 直接把核心路径交给控制器, 不能用 toCorePath(ROS Path) 转一圈: ROS
    // Path 没有前进/倒车方向字段, 会把 Hybrid A* 的反向段丢成“前进”.
    if (controller_strategy_) controller_strategy_->setPath(planned);
    // A2: 时间参数化 (cusp/旋转段/末段由轨迹接管; 控制器可选消费)
    {
      core::TrajectoryLimits limits;
      limits.max_v = max_velocity_x_;
      limits.max_accel = max_accel_x_;
      limits.max_w = max_velocity_theta_;
      limits.max_alpha = max_accel_theta_;
      limits.max_lat_accel = 0.5;
      const core::TimedTrajectory traj = planners::parameterizeTrajectory(planned, limits);
      if (controller_strategy_ && traj.valid) {
        controller_strategy_->setTrajectory(traj);
      }
    }
    global_path_pub_.publish(current_path_);

    m_need_initial_rotate_ = false;
    if (allow_initial_rotate && planned.points.size() > 2 && !first_reverse) {
      const double current_yaw = yawFromQuat(current_pose_.pose.orientation);
      const double dx = planned.points[1].pose.x - planned.points[0].pose.x;
      const double dy = planned.points[1].pose.y - planned.points[0].pose.y;
      const double path_yaw = std::atan2(dy, dx);
      if (std::abs(normalizeAngle(path_yaw - current_yaw)) > 0.35) {
        m_need_initial_rotate_ = true;
        m_initial_rotate_target_ = path_yaw;
      }
    }

    last_plan_time_ = ros::Time::now();
    is_navigating_ = true;
    is_paused_ = false;
    ROS_INFO("[rosiwit_navigation_ros1] path planned with %zu points "
             "(reverse=%zu smoothed=%d)", current_path_.poses.size(),
             reverse_points, static_cast<int>(smoothed));
  }

  bool pathBlocked() const
  {
    const core::Costmap & cm =
      last_planning_costmap_.data.empty() ? planner_costmap_ : last_planning_costmap_;
    if (current_path_.poses.empty() || cm.data.empty() || cm.resolution <= 0.0) {
      return false;
    }
    const int width = static_cast<int>(cm.width);
    const int height = static_cast<int>(cm.height);
    const size_t stride = std::max<size_t>(1, current_path_.poses.size() / 100);
    for (size_t i = 0; i < current_path_.poses.size(); i += stride) {
      const auto & p = current_path_.poses[i].pose.position;
      const int mx = static_cast<int>(std::floor(
        (p.x - cm.origin_x) / cm.resolution));
      const int my = static_cast<int>(std::floor(
        (p.y - cm.origin_y) / cm.resolution));
      if (mx < 0 || my < 0 || mx >= width || my >= height) { return true; }
      if (cm.data[static_cast<size_t>(my) * width + mx] < 0 ||
          cm.data[static_cast<size_t>(my) * width + mx] >= 100) {
        return true;
      }
    }
    return false;
  }

  void replanningCheck(const ros::TimerEvent &)
  {
    if (coverage_active_) return;
    // 恢复期间禁止周期重规划, 否则会打断 backoff/rotate 并再次报 Start in obstacle.
    if (recovery_stage_ != RecoveryStage::NONE) return;
    if (!is_navigating_ || is_paused_ || m_need_initial_rotate_ || planning_active_) return;
    // 只有"规划到目标点"的任务会周期重规划; 跟踪固定路径/旋转/倒车不重规划.
    if (task_active_ &&
        (action_task_.kind() != core::ActionKind::NAVIGATE || action_task_.rotating())) return;
    if (goal_pose_.header.frame_id.empty() || !m_has_planner_costmap_) return;
    const ros::Time now = ros::Time::now();
    const bool path_empty = current_path_.poses.empty();
    const bool blocked = replan_on_blocked_ && pathBlocked();
    const double since_plan = last_plan_time_.isZero() ? 1e9 :
      (now - last_plan_time_).toSec();
    const bool due = path_empty || since_plan >= replan_max_interval_;
    const bool blocked_due = blocked && since_plan >= replan_min_interval_;
    if (!due && !blocked_due) return;
    startPlanning(goal_pose_, /*allow_initial_rotate=*/false, /*restart=*/false);
  }

  void executeNavigation(const geometry_msgs::PoseStamped & goal,
                         bool allow_initial_rotate = true)
  {
    startPlanning(goal, allow_initial_rotate, /*restart=*/true);
  }

  void resetRecovery()
  {
    recovery_stage_ = RecoveryStage::NONE;
    recovery_core_path_.points.clear();
    if (recovery_controller_) { recovery_controller_->reset(); }
    recovery_started_ = ros::Time(0);
  }

  // Recovery-only costmap: deep-copy the global planner costmap and mark all
  // cells under the current robot footprint as free.  The robot is physically
  // standing there already; if the static map still marks those cells as
  // occupied, a reverse escape path cannot even get a valid start.  Both
  // Costmap::data and the shared grid copy must be cleared because Hybrid A*
  // uses data for cell checks and grid for footprint-corner checks.
  core::Costmap buildStartClearedCostmap(const core::Pose2D & pose,
                                         double margin,
                                         bool enabled)
  {
    core::Costmap cm = planner_costmap_;
    if (!enabled || cm.data.empty() ||
        cm.width == 0 || cm.height == 0 || cm.resolution <= 0.0) {
      return cm;
    }
    if (cm.grid) {
      cm.grid = std::make_shared<core::CostmapGrid>(*cm.grid);
    }

    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    margin = std::max(0.0, margin);
    const double res = cm.resolution;

    auto clear_cell = [&](int ix, int iy) {
      if (ix < 0 || iy < 0 ||
          ix >= static_cast<int>(cm.width) || iy >= static_cast<int>(cm.height)) {
        return;
      }
      const size_t idx = static_cast<size_t>(iy) * cm.width + ix;
      if (idx < cm.data.size()) { cm.data[idx] = 0; }
      if (cm.grid && idx < cm.grid->data.size()) { cm.grid->data[idx] = 0; }
    };

    if (configured_footprint_.size() >= 3) {
      double min_x = std::numeric_limits<double>::max();
      double max_x = std::numeric_limits<double>::lowest();
      double min_y = std::numeric_limits<double>::max();
      double max_y = std::numeric_limits<double>::lowest();
      for (const auto & v : configured_footprint_) {
        const double wx = pose.x + c * v.first - s * v.second;
        const double wy = pose.y + s * v.first + c * v.second;
        min_x = std::min(min_x, wx);
        max_x = std::max(max_x, wx);
        min_y = std::min(min_y, wy);
        max_y = std::max(max_y, wy);
      }
      const int x0 = static_cast<int>(std::floor(
        (min_x - margin - cm.origin_x) / res));
      const int x1 = static_cast<int>(std::ceil(
        (max_x + margin - cm.origin_x) / res));
      const int y0 = static_cast<int>(std::floor(
        (min_y - margin - cm.origin_y) / res));
      const int y1 = static_cast<int>(std::ceil(
        (max_y + margin - cm.origin_y) / res));
      for (int iy = y0; iy <= y1; ++iy) {
        for (int ix = x0; ix <= x1; ++ix) {
          const double wx = cm.origin_x + (ix + 0.5) * res;
          const double wy = cm.origin_y + (iy + 0.5) * res;
          const double dx = wx - pose.x;
          const double dy = wy - pose.y;
          const double lx = c * dx + s * dy;
          const double ly = -s * dx + c * dy;
          if (pointInPolygon(lx, ly, configured_footprint_) ||
              pointPolygonDistance(lx, ly, configured_footprint_) <= margin) {
            clear_cell(ix, iy);
          }
        }
      }
    } else {
      const double radius = std::max(0.0, robot_radius_) + margin;
      const int x0 = static_cast<int>(std::floor(
        (pose.x - radius - cm.origin_x) / res));
      const int x1 = static_cast<int>(std::ceil(
        (pose.x + radius - cm.origin_x) / res));
      const int y0 = static_cast<int>(std::floor(
        (pose.y - radius - cm.origin_y) / res));
      const int y1 = static_cast<int>(std::ceil(
        (pose.y + radius - cm.origin_y) / res));
      for (int iy = y0; iy <= y1; ++iy) {
        for (int ix = x0; ix <= x1; ++ix) {
          const double wx = cm.origin_x + (ix + 0.5) * res;
          const double wy = cm.origin_y + (iy + 0.5) * res;
          if (std::hypot(wx - pose.x, wy - pose.y) <= radius) {
            clear_cell(ix, iy);
          }
        }
      }
    }
    return cm;
  }

  core::Costmap buildRecoveryCostmap()
  {
    return buildStartClearedCostmap(
      toCorePose(current_pose_), recovery_start_clear_margin_,
      recovery_clear_start_footprint_);
  }

  // ---- 反向逃逸规划 (在后台线程执行, 不阻塞控制环) ----
  struct RecoveryPlanOutcome {
    bool ok = false;
    core::Path path;
    size_t candidate = 0;
    double target_distance = 0.0;
    std::string error;
  };

  RecoveryPlanOutcome planReverseEscape(
      const core::Pose2D & pose, const core::Costmap & costmap,
      const std::vector<core::Pose2D> & targets,
      const std::vector<std::pair<double, double>> & footprint_samples,
      double planning_budget)
  {
    RecoveryPlanOutcome out;
    recovery_planner_->setCostmap(costmap);
    auto target_free = [&](const core::Pose2D & target) {
      if (footprint_samples.empty() || !costmap.grid) { return true; }
      return core::footprintCost(*costmap.grid, target, footprint_samples) < 100;
    };
    const ros::Time started = ros::Time::now();
    for (size_t i = 0; i < targets.size(); ++i) {
      if ((ros::Time::now() - started).toSec() > planning_budget) {
        out.error = "planning budget exceeded";
        ROS_WARN("[rosiwit_navigation_ros1] reverse escape planning budget %.1fs "
                 "exceeded after %zu candidates", planning_budget, i);
        break;
      }
      if (!target_free(targets[i])) { continue; }
      const auto result = recovery_planner_->plan(pose, targets[i]);
      if (!result.is_ok()) {
        out.error = result.error_message();
        ROS_WARN("[rosiwit_navigation_ros1] reverse escape candidate=%zu failed: %s",
                 i + 1, result.error_message().c_str());
        continue;
      }
      out.ok = true;
      out.path = result.value();
      out.candidate = i;
      out.target_distance = std::hypot(targets[i].x - pose.x, targets[i].y - pose.y);
      return out;
    }
    if (out.error.empty()) { out.error = "no valid escape target"; }
    return out;
  }

  bool startRecovery()
  {
    if (!recovery_enabled_ || coverage_active_ ||
        recovery_attempts_ >= recovery_max_attempts_) {
      return false;
    }
    // Recovery is for navigation/path tasks only.  This uses the algorithm
    // layer's ActionTask kind, so /goal_pose and RobotNaviCmdAction take the
    // same path; only ROTATE/BACK_OFF primitives are excluded.
    if (task_active_ &&
        action_task_.kind() != core::ActionKind::NAVIGATE &&
        action_task_.kind() != core::ActionKind::FOLLOW_PATH) {
      return false;
    }
    if (!updateCurrentPose()) return false;

    ++recovery_attempts_;
    recovery_started_ = ros::Time::now();
    planning_active_ = false;
    ++plan_generation_;
    if (async_planner_) { async_planner_->cancel(); }
    is_navigating_ = true;
    m_need_initial_rotate_ = false;

    // Prefer a real reverse escape path over a fixed primitive. A short
    // reverse-only Hybrid A* plan can back out of a dead end/start collision
    // while preserving the normal planner's forward policy.
    if (recovery_planner_ && !planner_costmap_.data.empty() && recovery_controller_) {
      const core::Pose2D pose = toCorePose(current_pose_);
      const double back = std::max(0.30, recovery_back_distance_);
      const double side = std::max(0.20, back * 0.55);
      const double wide = std::max(0.45, back);
      // 先在正后方找目标; 再试更近/更远的目标和后左/后右弧线, 让
      // Hybrid A* 用反向路径绕开障碍, 而不是原地后退/旋转.
      auto behind = [&pose](double distance, double lateral) {
        const double cy = std::cos(pose.theta);
        const double sy = std::sin(pose.theta);
        return core::Pose2D(
          pose.x - cy * distance - sy * lateral,
          pose.y - sy * distance + cy * lateral,
          pose.theta);
      };
      auto behind_turn = [&pose](double distance, double lateral, double dyaw) {
        const double cy = std::cos(pose.theta);
        const double sy = std::sin(pose.theta);
        return core::Pose2D(
          pose.x - cy * distance - sy * lateral,
          pose.y - sy * distance + cy * lateral,
          pose.theta + dyaw);
      };
      const double inner = std::max(0.35, back * 0.83);
      std::vector<core::Pose2D> targets;
      targets.push_back(behind(back, 0.0));
      targets.push_back(behind(back * 0.70, 0.0));   // 后方目标太靠墙时退近一点
      targets.push_back(behind(back * 1.35, 0.0));    // 需要更长距离才能让开
      targets.push_back(behind(back * 0.85, side));   // 反向弧线绕左
      targets.push_back(behind(back * 0.85, -side));  // 反向弧线绕右
      // 贴着墙/充电桩时, 终点朝向也必须允许转开. 只要求目标位置和同一
      // 航向会让反向-only Hybrid A* 必须走 S 形, 短距离内几何不可达.
      // 左/右各试 ±0.6/±0.3rad 的终点航向, footprint 必须 free.
      for (double d : {back, back * 1.33, back * 1.66}) {
        for (double lateral : {-inner, inner}) {
          for (double dyaw : {-0.6, 0.6, -0.3, 0.3, 0.0}) {
            targets.push_back(behind_turn(d, lateral, dyaw));
          }
        }
      }
      // 更宽的侧向逃逸目标 (车身长边贴墙时 0.33m 仍不够).
      targets.push_back(behind(back * 0.70, wide));
      targets.push_back(behind(back * 0.70, -wide));
      targets.push_back(behind(back * 1.05, wide));
      targets.push_back(behind(back * 1.05, -wide));
      targets.push_back(behind(back * 2.0, wide));
      targets.push_back(behind(back * 2.0, -wide));

      const core::Costmap recovery_costmap = buildRecoveryCostmap();
      std::vector<std::pair<double, double>> footprint_samples;
      if (!configured_footprint_.empty()) {
        footprint_samples = core::sampleFootprint(
          configured_footprint_, std::max(0.02, footprint_sample_spacing_));
      }
      const double planning_budget = recovery_timeout_ > 0.0
        ? std::min(6.0, recovery_timeout_ * 0.8) : 6.0;
      // 上一次恢复规划还没结束: 复用它, 不再开新线程 (否则 future 赋值会阻塞)
      if (recovery_plan_future_.valid()) {
        if (recovery_plan_future_.wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready) {
          recovery_stage_ = RecoveryStage::PLANNING;
          return true;
        }
        try { (void)recovery_plan_future_.get(); } catch (...) {}
      }
      // 恢复规划放后台线程: 以前在 10Hz 控制定时器里同步跑, 单次最多阻塞
      // 6s, 期间不发 cmd_vel, 底盘继续执行最后一条速度指令 (危险).
      ++recovery_plan_gen_;
      recovery_plan_future_ = std::async(std::launch::async,
        [this, pose, recovery_costmap, targets, footprint_samples, planning_budget]() {
          return planReverseEscape(pose, recovery_costmap, targets,
                                   footprint_samples, planning_budget);
        });
      recovery_stage_ = RecoveryStage::PLANNING;
      ROS_WARN("[rosiwit_navigation_ros1] recovery planning started in background "
               "(%zu candidates)", targets.size());
      return true;
    }
    return false;
  }

  void failRecovery()
  {
    resetRecovery();
    failTask(movebase::RESULT_NO_PROGRESS, "FAILED_RECOVERY");
  }

  void finishRecovery()
  {
    // Resume the same high-level ActionTask regardless of whether the goal
    // came from /goal_pose or the RobotNaviCmdAction interface.  NAVIGATE goals
    // both carry goal_pose_; FOLLOW_PATH re-enters the path follower.
    const core::ActionKind kind = action_task_.kind();
    resetRecovery();
    if (task_active_ && kind == core::ActionKind::FOLLOW_PATH) {
      startActionFollowPath();
      return;
    }
    if (task_active_ && !goal_pose_.header.frame_id.empty()) {
      startPlanning(goal_pose_, /*allow_initial_rotate=*/false, /*restart=*/true);
      return;
    }
    stopNavigation();
  }

  void recoveryControlStep()
  {
    if (recovery_stage_ == RecoveryStage::NONE) return;
    const ros::Time now = ros::Time::now();
    if (recovery_stage_ == RecoveryStage::PLANNING) {
      if (!recovery_plan_future_.valid()) { failRecovery(); return; }
      if (recovery_plan_future_.wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready) {
        // 规划中: 保持停车 (不是保持上一条速度), 控制环不被阻塞
        publishCommand(applySafety(geometry_msgs::Twist()));
        return;
      }
      RecoveryPlanOutcome out;
      try { out = recovery_plan_future_.get(); }
      catch (const std::exception & e) {
        ROS_ERROR("[rosiwit_navigation_ros1] recovery planning exception: %s", e.what());
      }
      if (!out.ok) {
        ROS_ERROR("[rosiwit_navigation_ros1] reverse escape planning failed (%s); "
                  "task will fail", out.error.c_str());
        failRecovery();
        return;
      }
      recovery_core_path_ = out.path;
      current_path_ = toRosPath(recovery_core_path_, map_frame_, ros::Time::now());
      if (recovery_controller_) { recovery_controller_->setPath(recovery_core_path_); }
      recovery_stage_ = RecoveryStage::FOLLOW;
      // 规划耗时不算在执行超时里
      recovery_started_ = now;
      ROS_WARN("[rosiwit_navigation_ros1] recovery %d/%d: planned reverse escape "
               "candidate=%zu target_distance=%.2fm (%zu pts)",
               recovery_attempts_, recovery_max_attempts_, out.candidate + 1,
               out.target_distance, current_path_.poses.size());
      return;
    }
    if (recovery_timeout_ > 0.0 &&
        (now - recovery_started_).toSec() > recovery_timeout_) {
      ROS_WARN("[rosiwit_navigation_ros1] recovery timeout");
      failRecovery();
      return;
    }
    const core::Pose2D pose = toCorePose(current_pose_);

    if (recovery_stage_ == RecoveryStage::FOLLOW) {
      if (!recovery_controller_ || current_path_.poses.empty()) {
        failRecovery();
        return;
      }
      if (recovery_controller_->isGoalReached(pose)) {
        finishRecovery();
        return;
      }
      const auto core_cmd = recovery_controller_->computeVelocityCommand(
        pose, toCoreVelocity(current_velocity_));
      publishCommand(applySafety(toRosTwist(core_cmd)));
      recordActionTraveled();
      if (task_active_) publishActionOutputs();
      return;
    }

  }

  bool sensorFailSafe(std::string & status, int & result_code) const
  {
    const ros::Time now = ros::Time::now();
    if (!current_map_) {
      status = "STOPPED_MAP";
      result_code = movebase::RESULT_GLOBAL_MAP_ERROR;
      return false;
    }
    // 地图是静态产物, 可能只发一次/latch; map_timeout<=0 表示不按新鲜度判.
    if (map_timeout_ > 0.0 && !last_map_stamp_.isZero() &&
        (now - last_map_stamp_).toSec() > map_timeout_) {
      status = "STOPPED_MAP";
      result_code = movebase::RESULT_GLOBAL_MAP_ERROR;
      return false;
    }
    if (require_scan_ &&
        (last_scan_stamp_.isZero() ||
         (now - last_scan_stamp_).toSec() > scan_timeout_)) {
      status = "STOPPED_SCAN";
      result_code = movebase::RESULT_PERCEPTION_TIMEOUT;
      return false;
    }
    if (voxel_enabled_ && require_cloud_ &&
        (last_cloud_stamp_.isZero() ||
         (now - last_cloud_stamp_).toSec() > cloud_timeout_)) {
      status = "STOPPED_CLOUD";
      result_code = movebase::RESULT_PERCEPTION_TIMEOUT;
      return false;
    }
    return true;
  }

  void updateMotionWatch(const ros::Time & now)
  {
    const auto & p = current_pose_.pose.position;
    const double yaw = yawFromQuat(current_pose_.pose.orientation);
    const double moved = std::hypot(p.x - last_motion_pose_.x, p.y - last_motion_pose_.y);
    const double turned = std::fabs(normalizeAngle(yaw - last_motion_yaw_));
    if (last_motion_stamp_.isZero() || moved > 0.03 || turned > 0.05) {
      last_motion_pose_ = p;
      last_motion_yaw_ = yaw;
      last_motion_stamp_ = now;
    }
  }

  void slamInfoCallback(const xzrobot_ros_dependency::SlamInfo::ConstPtr & msg)
  {
    // 迁移期 shim: 一旦收到过 /localization/state, 就完全以新契约为准,
    // 避免两个来源打架 (旧话题还在发布给 App).
    if (has_localization_state_) { return; }
    last_slam_info_stamp_ = ros::Time::now();
    last_slam_status_ = msg->location_status;
    last_slam_matching_ = msg->matching_degree;
  }

  void localizationStateCallback(
    const xzrobot_ros_dependency::LocalizationState::ConstPtr & msg)
  {
    using LocState = xzrobot_ros_dependency::LocalizationState;
    last_slam_info_stamp_ = ros::Time::now();
    has_localization_state_ = true;
    last_loc_health_ = msg->health;
    last_map_id_ = msg->map_id;
    switch (msg->health) {
      case LocState::HEALTH_TRACKING:
      case LocState::HEALTH_DEGRADED:
        last_slam_status_ = 0;   // 仍在地图上; DEGRADED 由 matching 触发限速
        break;
      case LocState::HEALTH_INIT:
        last_slam_status_ = 1;
        break;
      default:
        last_slam_status_ = 2;   // LOST -> 停车
        break;
    }
    last_slam_matching_ = msg->matching;
  }

  bool slamHealthLost() const
  {
    return !last_slam_info_stamp_.isZero() && last_slam_status_ == 2 &&
           (ros::Time::now() - last_slam_info_stamp_).toSec() <= slam_info_timeout_;
  }

  double slamHealthScale() const
  {
    if (last_slam_info_stamp_.isZero() ||
        (ros::Time::now() - last_slam_info_stamp_).toSec() > slam_info_timeout_) {
      return 1.0;   // 没有健康信息时不额外限速 (兼容未接 SLAM 的部署)
    }
    return last_slam_matching_ < slam_degrade_matching_ ? slam_degrade_scale_ : 1.0;
  }

  // 旋转段执行器: Hybrid A* 前进-only 模式用"原地旋转基元"掉头/对正, 路径点
  // 标记 velocity_x==0 且位置不变. MPPI 无法跟踪原地旋转, 这里显式执行.
  // 旋转段执行器: Hybrid A* 前进-only 模式用"原地旋转基元"掉头/对正, 路径点
  // 标记 velocity_x==0 且位置不变. MPPI 无法跟踪原地旋转, 这里显式执行.
  bool executeRotationSegment(const core::Pose2D & pose, geometry_msgs::Twist & out)
  {
    if (current_core_path_.points.size() < 2) return false;
    size_t nearest = 0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < current_core_path_.points.size(); ++i) {
      const auto & q = current_core_path_.points[i].pose;
      const double d = std::hypot(q.x - pose.x, q.y - pose.y);
      if (d < best) { best = d; nearest = i; }
    }
    if (best > 0.25) return false;
    if (std::fabs(current_core_path_.points[nearest].velocity_x) > 0.1) return false;
    const auto & base = current_core_path_.points[nearest].pose;
    size_t end = nearest;
    while (end + 1 < current_core_path_.points.size()) {
      const auto & q = current_core_path_.points[end + 1].pose;
      if (std::hypot(q.x - base.x, q.y - base.y) > 0.03) break;
      ++end;
    }
    // A* 路径点位置连续变化, end==nearest, 不会误判成旋转段
    if (end == nearest) return false;
    const double target = current_core_path_.points[end].pose.theta;
    const double err = normalizeAngle(target - pose.theta);
    if (std::fabs(err) < std::max(0.05, movebase_rotate_tolerance_)) {
      rot_dir_ = 0;
      return false;
    }
    // 方向只在旋转开始时锁存一次, 中途绝不反转; 到位才释放.
    if (rot_dir_ == 0) rot_dir_ = err > 0.0 ? 1 : -1;
    out.linear.x = 0.0;
    // 最小角速度 0.15: 接近目标时 2*err 太小会被底盘死区吃掉, 表现为"转一下停"
    out.angular.z = rot_dir_ * std::min(max_velocity_theta_,
                                        std::max(0.15, 2.0 * std::fabs(err)));
    return true;
  }

  // 局部路径: 只发布机器人前方 ~4m 的切片 (之前把整条全局路径发到
  // /local_path, 而且旋转/恢复/末段分支提前 return 时不刷新 -> 话题冻结).
  void publishLocalPathSlice()
  {
    if (current_core_path_.points.size() < 2) return;
    const core::Pose2D cp = toCorePose(current_pose_);
    size_t nearest = 0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < current_core_path_.points.size(); ++i) {
      const auto & q = current_core_path_.points[i].pose;
      const double d = std::hypot(q.x - cp.x, q.y - cp.y);
      if (d < best) { best = d; nearest = i; }
    }
    nav_msgs::Path out;
    out.header.frame_id = map_frame_;
    out.header.stamp = ros::Time::now();
    double acc = 0.0;
    geometry_msgs::Point prev;
    for (size_t i = nearest; i < current_core_path_.points.size(); ++i) {
      const auto & q = current_core_path_.points[i].pose;
      if (i > nearest) {
        acc += std::hypot(q.x - prev.x, q.y - prev.y);
        if (acc > 4.0) { break; }
      }
      geometry_msgs::PoseStamped ps;
      ps.header = out.header;
      ps.pose.position.x = q.x;
      ps.pose.position.y = q.y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.z = std::sin(q.theta * 0.5);
      ps.pose.orientation.w = std::cos(q.theta * 0.5);
      out.poses.push_back(ps);
      prev.x = q.x;
      prev.y = q.y;
    }
    if (!out.poses.empty()) { path_pub_.publish(out); }
  }

  void controlLoop(const ros::TimerEvent &)
  {
    if (planning_active_) pollPlanning();
    // 丢弃已完成但已过期的恢复规划结果 (resetRecovery 不销毁在途 future)
    if (recovery_stage_ == RecoveryStage::NONE && recovery_plan_future_.valid() &&
        recovery_plan_future_.wait_for(std::chrono::seconds(0)) ==
            std::future_status::ready) {
      try { (void)recovery_plan_future_.get(); } catch (...) {}
    }
    if (!is_navigating_ || is_paused_) { return; }
    if (last_odom_stamp_.isZero() ||
        (ros::Time::now() - last_odom_stamp_).toSec() > 0.6) {
      if (task_active_) {
        failTask(movebase::RESULT_PERCEPTION_TIMEOUT, "STOPPED_ODOM");
      } else {
        setGoalStatus("STOPPED_ODOM");
        stopNavigation();
      }
      return;
    }
    if (!updateCurrentPose()) {
      // 定位/TF 失效时必须主动发零, 不能保留上一条 cmd_vel.
      if (task_active_) {
        failTask(movebase::RESULT_POSE_ERROR, "FAILED_TF");
      } else {
        setGoalStatus("FAILED_TF");
        stopNavigation();
      }
      return;
    }

    updateMotionWatch(ros::Time::now());
    publishLocalPathSlice();   // 每个控制周期都刷新局部路径切片
    // 定位丢失: /lio_odom 会继续发最后有效位姿, 冻结位姿继续走等于盲开.
    if (slamHealthLost()) {
      if (task_active_) {
        failTask(movebase::RESULT_POSE_ERROR, "FAILED_LOC_LOST");
      } else {
        setGoalStatus("FAILED_LOC_LOST");
        stopNavigation();
      }
      return;
    }

    {
      std::string status;
      int result_code = movebase::RESULT_PERCEPTION_TIMEOUT;
      if (!sensorFailSafe(status, result_code)) {
        if (task_active_) {
          failTask(result_code, status);
        } else {
          setGoalStatus(status);
          stopNavigation();
        }
        return;
      }
    }

    // ---- 阻塞兜底: 安全层持续停住 / 有指令但底盘不动 -> 立即恢复 ----
    if (recovery_stage_ == RecoveryStage::NONE && !coverage_active_) {
      const ros::Time now = ros::Time::now();
      const bool safety_stopped = safety_enabled_ && last_safety_scale_ <= 0.01;
      if (safety_stopped) {
        if (safety_stop_since_.isZero()) {
          safety_stop_since_ = now;
        } else if ((now - safety_stop_since_).toSec() > safety_stop_timeout_) {
          safety_stop_since_ = ros::Time();
          ROS_WARN("[rosiwit_navigation_ros1] safety stop %.1fs, start recovery",
                   safety_stop_timeout_);
          if (startRecovery()) { return; }
          if (task_active_) {
            failTask(movebase::RESULT_NO_PROGRESS, "FAILED_SAFETY_STOP");
          } else {
            setGoalStatus("FAILED_SAFETY_STOP");
            stopNavigation();
          }
          return;
        }
      } else {
        safety_stop_since_ = ros::Time();
      }
      if (!last_motion_stamp_.isZero() &&
          (now - last_motion_stamp_).toSec() > no_progress_timeout_) {
        last_motion_stamp_ = now;   // 防止重复触发
        ROS_WARN("[rosiwit_navigation_ros1] no motion for %.1fs, start recovery",
                 no_progress_timeout_);
        if (startRecovery()) { return; }
        if (task_active_) {
          failTask(movebase::RESULT_NO_PROGRESS, "FAILED_NO_MOTION");
        } else {
          setGoalStatus("FAILED_NO_MOTION");
          stopNavigation();
        }
        return;
      }
    }

    if (coverage_active_) {
      coverageControlStep();
      return;
    }

    if (recovery_stage_ != RecoveryStage::NONE) {
      recoveryControlStep();
      return;
    }

    // ---- movebase action: 状态/进度/到位/卡死全部交给算法层 ----
    if (task_active_) {
      // 到点判定只依赖当前位姿/路径终点; 以前加了 !planning_active_ 门控,
      // Hybrid A* 单次规划可达数秒, 期间到点也不停 -> 冲过目标点.
      const bool xy_reached =
        !current_path_.poses.empty() && isGoalReached();
      const double controller_progress =
        controller_strategy_ ? controller_strategy_->getProgress() : 0.0;
      const core::ActionTaskStatus task_status = action_task_.update(
        toCorePose(current_pose_), ros::Time::now().toSec(),
        xy_reached, controller_progress);

      if (task_status == core::ActionTaskStatus::SUCCEEDED) {
        ROS_INFO("[rosiwit_navigation_ros1] goal reached");
        finishTask(movebase::RESULT_SUCCESS);
        return;
      }
      if (task_status == core::ActionTaskStatus::FAILED_STUCK) {
        if (startRecovery()) {
          ROS_WARN("[movebase_iface] 判定卡死, 尝试后退/转向恢复");
          return;
        }
        ROS_ERROR("[movebase_iface] 超过 %.0f 秒没有进展, 判定卡死",
                  movebase_stuck_timeout_);
        failTask(movebase::RESULT_NO_PROGRESS, "FAILED_STUCK");
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
        if (task_active_) { publishActionOutputs(); }
        return;
      }
    }

    // 末段精调 (<0.6m): MPPI 低速小位移的采样分辨率不够, 末段对位会卡.
    // 两段式: 先对正"朝向目标点的方位"(偏差大时原地转, 不前进, 避免斜着顶墙),
    // 再小速度逼近; 进入 xy 容差后原地转到目标朝向.
    if (!coverage_active_ && recovery_stage_ == RecoveryStage::NONE &&
        !goal_pose_.header.frame_id.empty()) {
      const core::Pose2D gp = toCorePose(goal_pose_);
      const core::Pose2D cp = toCorePose(current_pose_);
      const double d = std::hypot(gp.x - cp.x, gp.y - cp.y);
      if (d < 0.6) {
        geometry_msgs::Twist c;
        if (d > goal_tolerance_xy_) {
          const double bearing = std::atan2(gp.y - cp.y, gp.x - cp.x);
          const double head_err = normalizeAngle(bearing - cp.theta);
          if (std::fabs(head_err) > 0.4) {
            c.linear.x = 0.0;   // 先对正行进方向
          } else {
            c.linear.x = std::min(max_velocity_x_, 0.9 * d);
          }
          // 与旋转段执行器同一套锁存逻辑, 避免末段对位来回转
          if (std::fabs(head_err) > 0.08) {
            if (rot_dir_ == 0) rot_dir_ = head_err > 0.0 ? 1 : -1;
            c.angular.z = rot_dir_ * std::min(max_velocity_theta_,
                                              std::max(0.15, 2.0 * std::fabs(head_err)));
          } else {
            rot_dir_ = 0;
            c.angular.z = 0.0;
          }
        } else {
          c.linear.x = 0.0;     // 已到 xy 容差: 原地对末朝向
          const double yaw_err = normalizeAngle(gp.theta - cp.theta);
          c.angular.z = std::max(-max_velocity_theta_,
                                 std::min(max_velocity_theta_, 1.5 * yaw_err));
        }
        publishCommand(applySafety(c));
        recordActionTraveled();
        if (task_active_) { publishActionOutputs(); }
        return;
      }
    }

    auto cmd = computeVelocityCommand();
    // 旋转段: 先原地转到位, 再交回 MPPI 跟踪前进段 (控制器自带原地旋转能力时
    // 由控制器自己执行)
    if (!(controller_strategy_ && controller_strategy_->supportsInPlaceRotation())) {
      geometry_msgs::Twist rot_cmd;
      if (executeRotationSegment(toCorePose(current_pose_), rot_cmd)) {
        rotation_active_ = true;
        publishCommand(applySafety(rot_cmd));
        recordActionTraveled();
        if (task_active_) { publishActionOutputs(); }
        return;
      }
      // 旋转刚结束: MPPI 的进度索引还停在旋转点(旋转时位置不变, 索引不前进),
      // 重新播种整条路径让参考点推进到旋转段之后, 否则会停在半转姿态.
      if (rotation_active_) {
        rotation_active_ = false;
        controller_strategy_->setPath(current_core_path_);
        ROS_INFO("[rosiwit_navigation_ros1] rotation done -> controller re-seeded (%zu pts)",
                 current_core_path_.points.size());
      }
    }
    const double health_scale = slamHealthScale();
    if (health_scale < 1.0) {
      cmd.linear.x *= health_scale;
      cmd.angular.z *= health_scale;
    }
    publishCommand(applySafety(cmd));
    recordActionTraveled();
    if (task_active_) { publishActionOutputs(); }
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

  geometry_msgs::Twist applySpeedLimit(const geometry_msgs::Twist & cmd)
  {
    if (speed_limit_layer_.size() == 0) { return cmd; }
    const double limit = speed_limit_layer_.maxLinearAt(
      current_pose_.pose.position.x, current_pose_.pose.position.y, max_velocity_x_);
    if (limit >= max_velocity_x_ - 1e-6) { return cmd; }
    geometry_msgs::Twist out = cmd;
    out.linear.x = std::max(-limit, std::min(limit, out.linear.x));
    return out;
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
    core::VelocityCommand out = result.command;
    // 接近/减速档只降线速度, 角速度保持原值: 原地旋转不产生位移, 若把 w
    // 也缩到 0.4x, 末段 120° 对位要转十几秒 (实测超过任务超时).
    if (result.scale > 1e-3 && result.scale < 0.999 &&
        std::fabs(cmd.angular.z) > 1e-3) {
      out.angular_z = cmd.angular.z;
    }
    return toRosTwist(out);
  }

  void stopNavigation()
  {
    resetRecovery();
    is_navigating_ = false;
    is_paused_ = false;
    current_core_path_.points.clear();
    last_motion_stamp_ = ros::Time();
    safety_stop_since_ = ros::Time();
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
  bool planner_allow_reverse_ = true;
  double planner_max_reverse_distance_ = 1.0;
  double max_accel_x_ = 0.4;
  double max_accel_theta_ = 0.8;
  int mppi_horizon_ = 20;
  int mppi_samples_ = 160;
  double mppi_dt_ = 0.0;
  double mppi_lambda_ = 0.8;
  double mppi_sigma_v_ = 0.18;
  double mppi_sigma_w_ = 0.35;
  double mppi_path_weight_ = 2.0;
  double mppi_heading_weight_ = 0.4;
  double mppi_obstacle_weight_ = 0.03;
  double mppi_control_weight_ = 0.03;
  double mppi_speed_weight_ = 2.0;
  double mppi_goal_weight_ = 1.5;

  bool safety_enabled_ = true;
  double safety_self_filter_radius_ = 0.25;
  bool scan_self_filter_enabled_ = true;
  double scan_self_x_min_ = -0.45;
  double scan_self_x_max_ = 0.75;
  double scan_self_y_min_ = -0.55;
  double scan_self_y_max_ = 0.55;
  bool safety_fail_safe_on_timeout_ = true;
  double safety_source_timeout_ = 0.5;
  double safety_slowdown_ratio_ = 0.5;
  bool safety_approach_enabled_ = true;
  double safety_time_before_collision_ = 2.0;
  core::SafetyMonitor safety_monitor_;
  core::ObstacleTracker tracker_;
  core::VoxelLayer voxel_layer_;
  bool voxel_enabled_ = true;
  std::string voxel_cloud_topic_ = "/world_cloud";
  int voxel_max_points_ = 200000;
  double voxel_resolution_ = 0.10;
  double voxel_min_height_ = -0.20;
  double voxel_max_height_ = 2.00;
  double voxel_decay_ = 2.0;
  double voxel_inflation_ = 0.15;
  bool tracking_enabled_ = true;
  double tracking_horizon_ = 0.5;
  double tracking_cluster_distance_ = 0.35;
  int tracking_confirm_hits_ = 3;
  double tracking_max_radius_ = 1.2;
  double tracking_min_prediction_speed_ = 0.15;
  std::vector<core::Point2D> tracker_points_;
  std::string last_safety_state_ = "PASS";
  double last_safety_scale_ = 1.0;

  // ---- 分层代价地图 (B): 同一批层, planner/controller 两个视图 ----
  bool keepout_enabled_ = true;
  std::string virtual_obstacles_topic_ = "/virtual_obstacles";
  std::string keepout_frame_ = "map";
  std::shared_ptr<core::KeepoutLayer> keepout_layer_;
  std::shared_ptr<core::StaticLayer> planner_static_layer_;
  std::shared_ptr<core::StaticLayer> local_static_layer_;
  std::shared_ptr<core::PointsLayer> local_scan_layer_;
  std::shared_ptr<core::CirclesLayer> local_track_layer_;
  std::shared_ptr<core::PointsLayer> local_voxel_layer_;
  core::LayeredCostmap planner_layers_;
  core::LayeredCostmap local_layers_;
  std::vector<core::Point2D> scan_layer_points_;
  core::SpeedLimitLayer speed_limit_layer_;
  ros::Subscriber virtual_obstacles_sub_;
  ros::Subscriber speed_limit_sub_;

  std::string global_frame_;
  std::string robot_frame_;
  std::string map_frame_;
  std::string planner_name_;
  std::string controller_name_;
  std::string odom_topic_;
  std::string map_topic_ = "/map";
  std::string scan_topic_ = "/scan";
  std::string goal_topic_ = "/goal_pose";
  std::string nav_stop_service_ = "/nav_stop";
  std::string global_path_topic_ = "/global_path";
  std::string local_path_topic_ = "/local_path";
  std::string global_costmap_topic_ = "/global_costmap";
  std::string local_costmap_topic_ = "/local_costmap";
  std::string goal_status_topic_ = "/goal_status";
  std::string diagnostics_topic_ = "/nav_diagnostics";
  std::string tracked_obstacles_topic_ = "/tracked_obstacles";
  double map_timeout_ = 2.0;
  double scan_timeout_ = 0.5;
  double cloud_timeout_ = 1.0;
  bool require_scan_ = true;
  bool require_cloud_ = false;
  double replan_max_interval_ = 5.0;
  double replan_min_interval_ = 1.0;
  bool replan_on_blocked_ = true;
  double costmap_update_rate_ = 5.0;
  std::string rviz_goal_topic_ = "/move_base_simple/goal";
  bool rviz_goal_align_yaw_ = true;
  bool footprint_aware_control_ = false;
  double footprint_sample_spacing_ = 0.05;
  double footprint_shrink_ = 0.0;   // 轮廓内缩(负余量), 窄道通过用
  double planner_footprint_margin_ = 0.06;   // 规划器轮廓外扩(最小离墙余量)
  core::Footprint planner_footprint_;        // 规划用轮廓 = 车体 + margin
  double path_resample_step_ = 0.08;
  core::Footprint configured_footprint_;
  core::Footprint controller_footprint_;

  std::unique_ptr<core::IPlannerStrategy> planner_strategy_;
  std::unique_ptr<core::IPlannerStrategy> async_planner_;
  std::future<core::Result<core::Path>> async_plan_future_;
  bool planning_active_ = false;
  uint64_t plan_generation_ = 0;
  uint64_t active_plan_generation_ = 0;

  // 恢复规划在后台线程执行; 控制环只 poll, 不再被 plan() 阻塞
  std::future<RecoveryPlanOutcome> recovery_plan_future_;
  uint64_t recovery_plan_gen_ = 0;

  // 运动看门狗: 指令有效但底盘不动 -> 快速判定阻塞 (替代 60s 位移看门狗)
  ros::Time last_motion_stamp_;
  geometry_msgs::Point last_motion_pose_;
  double last_motion_yaw_ = 0.0;
  double no_progress_timeout_ = 8.0;
  ros::Time safety_stop_since_;
  double safety_stop_timeout_ = 2.0;

  // 定位健康: 丢失 -> 停车, 匹配度低 -> 限速
  ros::Subscriber slam_info_sub_;
  ros::Subscriber localization_state_sub_;   // /localization/state (新契约, 优先)
  std::string localization_state_topic_ = "/localization/state";
  bool has_localization_state_ = false;
  uint8_t last_loc_health_ = 0;
  std::string last_map_id_;
  ros::Subscriber profile_sub_;      // /set_motion_profile: "clean|free[,controller]"
  std::string motion_profile_ = "clean";
  std::string free_planner_ = "hybrid_astar";
  std::string clean_planner_ = "hybrid_astar";
  // 当前主路径 (核心坐标系): 供旋转段执行器读取 velocity_x 标记
  core::Path current_core_path_;
  bool rotation_active_ = false;
  int rot_dir_ = 0;   // 旋转方向锁存 (避免 ±π 附近左右摆)
  ros::Time last_slam_info_stamp_;
  int last_slam_status_ = -1;
  double last_slam_matching_ = 1.0;
  double slam_info_timeout_ = 2.0;
  double slam_degrade_matching_ = 0.65;
  double slam_degrade_scale_ = 0.5;
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
  ros::Publisher footprint_pub_;   // /robot_footprint: idle 也发, RViz 调试用
  ros::Publisher tracked_obs_pub_;
  ros::ServiceServer nav_stop_srv_;
  std::string goal_status_ = "IDLE";
  ros::Time last_odom_stamp_;
  ros::Time last_scan_stamp_;
  ros::Time last_map_stamp_;
  ros::Time last_cloud_stamp_;
  ros::Time last_plan_time_;
  ros::Time last_costmap_build_;
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
  bool task_active_ = false;

  // 恢复只使用规划器产生的反向路径 (Hybrid A*, reverse-only).
  enum class RecoveryStage { NONE, PLANNING, FOLLOW };
  RecoveryStage recovery_stage_ = RecoveryStage::NONE;
  std::unique_ptr<core::IPlannerStrategy> recovery_planner_;
  std::unique_ptr<core::IControllerStrategy> recovery_controller_;
  std::unique_ptr<core::IPlannerStrategy> coverage_planner_;
  core::Path recovery_core_path_;
  bool recovery_enabled_ = true;
  int recovery_max_attempts_ = 2;
  int recovery_attempts_ = 0;
  double recovery_back_distance_ = 0.60;
  double recovery_timeout_ = 20.0;
  bool recovery_clear_start_footprint_ = true;
  double recovery_start_clear_margin_ = 0.05;
  double recovery_start_relax_radius_ = 1.0;
  bool planning_clear_start_footprint_ = true;
  double planning_start_clear_margin_ = 0.05;
  double planning_start_relax_radius_ = 0.0;
  core::Costmap last_planning_costmap_;
  bool laser_filter_enabled_ = true;
  double laser_filter_radius_ = 0.15;
  int laser_filter_min_neighbors_ = 1;
  int laser_filter_window_ = 2;
  bool action_interface_active_ = false;
  ros::Time recovery_started_;

  // ---- 动态覆盖清洁 ----
  bool coverage_active_ = false;
  core::CoverageGrid coverage_grid_;
  core::CoverageConfig coverage_config_;
  ros::Time coverage_last_plan_;
  ros::Time coverage_last_progress_;
  // 单行卡死检测: 同一行(0.1m 量化)反复选中且覆盖量不增 -> 只放弃这一条带
  int coverage_row_key_x_ = 0;
  int coverage_row_key_y_ = 0;
  size_t coverage_row_covered_ = 0;
  ros::Time coverage_row_since_;
  ros::Time coverage_last_status_;
  ros::Time coverage_last_dynamic_;
  size_t coverage_last_covered_count_ = 0;
  bool coverage_has_target_ = false;
  core::Pose2D coverage_target_;
  core::Pose2D coverage_last_progress_pose_;
  ros::Time coverage_last_diag_;
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
  double coverage_complete_rate_ = 0.99;
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
  // 多线程 spinner: 控制环(10Hz) 不再被代价地图/规划回调饿死.
  // 单线程时局部代价地图更新会把 controlLoop 挤到几赫兹, 表现为"走得慢".
  int threads = 2;
  pnh.param<int>("spinner_threads", threads, 2);
  threads = std::max(1, std::min(8, threads));
  ros::AsyncSpinner spinner(threads);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
