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

#include <deque>
#include <map>

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
#include "rosiwit_navigation/nav_core/velocity_shaper.hpp"
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
#include "crash_handler.h"
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
    // 加权 A* 倍率: 以前只用了成员默认 2.0, YAML 同名键无效 (改配置不生效)
    pnh_.param<double>("planner_heuristic_weight", planner_heuristic_weight_,
                       planner_heuristic_weight_);
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
    pnh_.param<double>("replan_max_interval", replan_max_interval_, 30.0);
    pnh_.param<double>("replan_min_interval", replan_min_interval_, 1.5);
    pnh_.param<bool>("replan_on_blocked", replan_on_blocked_, true);
    // 阻塞只检查机器人前方一段, 且连续 N 次才重规划 (避免单帧噪声/身后路径触发)
    pnh_.param<double>("replan_lookahead", replan_lookahead_, 3.0);
    pnh_.param<double>("replan_skip_distance", replan_skip_distance_, 0.5);
    pnh_.param<int>("replan_blocked_frames", replan_blocked_frames_, 2);

    // costmap 构建节流 (scan 回调不再每个 scan 都做完整聚类)
    pnh_.param<double>("costmap_update_rate", costmap_update_rate_, 5.0);

    // RViz "2D Nav Goal" 默认发 /move_base_simple/goal (不是 /goal_pose).
    pnh_.param<std::string>("rviz_goal_topic", rviz_goal_topic_,
                            "/move_base_simple/goal");
    pnh_.param<bool>("rviz_goal_align_yaw", rviz_goal_align_yaw_, true);

    pnh_.param<bool>("safety_enabled", safety_enabled_, true);
    // 只做"传感器盲区"过滤 (0.10m). 车身轮廓由 body_polygon 排除 ——
    // 以前 0.25m 的圆 + 大矩形自过滤会把整块安全区一起吃掉, STOP 永不触发.
    pnh_.param<double>("safety_self_filter_radius", safety_self_filter_radius_, 0.10);
    pnh_.param<bool>("safety_fail_safe_on_timeout", safety_fail_safe_on_timeout_, true);
    pnh_.param<double>("safety_source_timeout", safety_source_timeout_, 0.5);
    pnh_.param<double>("safety_slowdown_ratio", safety_slowdown_ratio_, 0.5);
    pnh_.param<bool>("safety_approach_enabled", safety_approach_enabled_, true);
    pnh_.param<double>("safety_time_before_collision", safety_time_before_collision_, 2.0);
    // 滞回 (nav2 trigger/release_consecutive_points): 连续 N 帧才进入/释放,
    // 单帧噪声不会再让 stop/slow 来回跳.
    // 原地转安全半径 = max|footprint 顶点| + 该余量. 0 = 退回矩形旋转区.
    pnh_.param<double>("safety_rotation_radius_margin", safety_rotation_radius_margin_, 0.10);
    // 点数阈值: 以前只在代码里写死 (4/6), YAML 里的同名键完全无效
    pnh_.param<int>("safety_stop_max_points", safety_stop_max_points_, safety_stop_max_points_);
    pnh_.param<int>("safety_slowdown_max_points", safety_slowdown_max_points_,
                    safety_slowdown_max_points_);
    pnh_.param<int>("safety_stop_trigger_frames", safety_stop_trigger_frames_, 2);
    pnh_.param<int>("safety_stop_release_frames", safety_stop_release_frames_, 3);
    pnh_.param<int>("safety_slowdown_trigger_frames", safety_slowdown_trigger_frames_, 1);
    pnh_.param<int>("safety_slowdown_release_frames", safety_slowdown_release_frames_, 2);
    // 动态安全区: 按车体几何 + 制动距离生成, 避免固定多边形与巷道宽度打架
    pnh_.param<bool>("safety_dynamic_geometry", safety_dynamic_geometry_, true);
    pnh_.param<double>("safety_brake_decel", safety_brake_decel_, 0.8);
    pnh_.param<double>("safety_reaction_time", safety_reaction_time_, 0.2);
    pnh_.param<double>("safety_stop_margin", safety_stop_margin_, 0.05);
    pnh_.param<double>("safety_side_margin", safety_side_margin_, 0.03);
    pnh_.param<double>("safety_turn_margin", safety_turn_margin_, 0.05);
    pnh_.param<double>("safety_slowdown_extra", safety_slowdown_extra_, 0.30);
    pnh_.param<double>("safety_scale_slew_rate", safety_scale_slew_rate_, 1.0);
    pnh_.param<double>("safety_scale_dt", safety_scale_dt_, 0.1);
    // 车身轮廓外扩: 只排除轮廓内的点 (nav2 exclusion_zones.robot_body)
    pnh_.param<double>("safety_body_margin", safety_body_margin_, 0.05);
    std::vector<double> stop_poly, slow_poly, footprint;
    // 侧向 0.345 = 车体半宽 0.33 + 15mm: 0.8m 门洞 (±0.40) 能过, 车体扫掠内仍有保护
    std::vector<double> default_stop{0.55, -0.345, 0.95, -0.345, 0.95, 0.345, 0.55, 0.345};
    std::vector<double> default_slow{0.55, -0.55, 1.15, -0.55, 1.15, 0.55, 0.55, 0.55};
    const std::vector<double> default_footprint{-0.25, -0.22, 0.25, -0.22, 0.25, 0.22, -0.25, 0.22};
    pnh_.param<std::vector<double>>("safety_stop_polygon", stop_poly, default_stop);
    pnh_.param<std::vector<double>>("safety_slowdown_polygon", slow_poly, default_slow);
    pnh_.param<std::vector<double>>("safety_footprint", footprint, default_footprint);
    // 按运动状态切换的 STOP 区 (nav2 VelocityPolygon)
    std::vector<double> default_back{-0.75, -0.345, -0.35, -0.345, -0.35, 0.345, -0.75, 0.345};
    std::vector<double> default_rot{-0.50, -0.55, 0.80, -0.55, 0.80, 0.55, -0.50, 0.55};
    std::vector<double> back_poly, rot_poly;
    pnh_.param<std::vector<double>>("safety_stop_polygon_backward", back_poly, default_back);
    pnh_.param<std::vector<double>>("safety_stop_polygon_rotation", rot_poly, default_rot);
    safety_stop_poly_ = stop_poly;
    safety_slow_poly_ = slow_poly;
    safety_back_poly_ = back_poly;
    safety_rot_poly_ = rot_poly;

    // ---- 指令整形 / 死手 / 仲裁 (nav2 velocity_smoother + twist_mux) ----
    pnh_.param<double>("velocity_timeout", velocity_timeout_, 0.6);
    pnh_.param<double>("deadband_velocity_v", deadband_velocity_v_, 0.0);
    pnh_.param<double>("deadband_velocity_w", deadband_velocity_w_, 0.0);
    pnh_.param<std::string>("teleop_topic", teleop_topic_, "");
    pnh_.param<double>("teleop_timeout", teleop_timeout_, 0.5);
    // 恢复: 清图 / 原地转 / 等待 (nav2 行为树的系统级恢复)
    pnh_.param<bool>("recovery_clear_costmaps", recovery_clear_costmaps_, true);
    pnh_.param<double>("recovery_spin_angle_deg", recovery_spin_angle_deg_, 60.0);
    pnh_.param<double>("recovery_spin_timeout", recovery_spin_timeout_, 6.0);
    pnh_.param<double>("recovery_wait_sec", recovery_wait_sec_, 2.0);
    // 配置自检: 生效值与 yaml 不一致/非法时直接报错 (strict 时退出)
    pnh_.param<bool>("strict_config", strict_config_, false);

    // 与 3D 前端相同的车体自过滤: /scan 在 base 系下落在车体包围盒内的点
    // 不进入 costmap/safety. 只过滤车体内部, 车外近障碍仍保留.
    pnh_.param<bool>("scan_self_filter_enabled", scan_self_filter_enabled_, true);

pnh_.param<double>("scan_self_x_min", scan_self_x_min_, -0.45);
    pnh_.param<double>("scan_self_x_max", scan_self_x_max_, 0.75);
    pnh_.param<double>("scan_self_y_min", scan_self_y_min_, -0.55);
    pnh_.param<double>("scan_self_y_max", scan_self_y_max_, 0.55);

    safety_footprint_poly_ = footprint;   // APPROACH 模型用的车体轮廓

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
    // 旋转被安全层挡住多久开始后退脱困 / 退多快多久 / 最多几次
    pnh_.param<double>("coverage_rot_blocked_timeout", cov_rot_blocked_timeout_, 1.2);
    pnh_.param<double>("coverage_backoff_speed", cov_backoff_speed_, 0.12);
    pnh_.param<double>("coverage_backoff_time", cov_backoff_time_, 0.7);
    pnh_.param<double>("coverage_rot_hold_time", cov_rot_hold_time_, 0.25);
    pnh_.param<int>("coverage_backoff_max", cov_backoff_max_, 2);
    pnh_.param<double>("coverage_complete_rate", coverage_complete_rate_, 0.99);
    pnh_.param<double>("coverage_blocked_inflation", coverage_blocked_inflation_, 0.10);
    pnh_.param<double>("coverage_plan_inflation", coverage_plan_inflation_, 0.0);
    pnh_.param<double>("coverage_plan_robot_radius", coverage_plan_robot_radius_, 0.0);
    // 行间掉头: 0 = 原地转; 0.25 = 半圆弧 (R = 行距/2, 需要行端有 R 的转弯余量)
    pnh_.param<double>("coverage_turn_radius", coverage_turn_radius_, 0.0);
    // headland 环宽度: 0 = 不做; = sweep 时先绕边界一圈. 规则矩形区域行线本来
    // 就能贴到边界, 开环只增加 8~10m 空跑; 边带漏的时候才开.
    pnh_.param<double>("coverage_headland_width", coverage_headland_width_, 0.0);
    // 门洞切分: 可通行宽度 <= 该值就切单元. 实测(两室一门洞 26 m²)覆盖率
    // 只 +0.04%、路径 +9%、原地转 16->29 次, 默认关闭; 需要时再开.
    pnh_.param<double>("coverage_doorway_width", coverage_doorway_width_, 0.0);
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
    }

    // 车身排除区 = 真实 footprint 的包围盒外扩 safety_body_margin.
    // 只排除轮廓内的点, 车外近处障碍全部保留 -> STOP 区才可能被触发.
    // 车体圆模型自检: 内切(原点->边最小) / 外接(原点->顶点最大) / 半宽.
    // robot_radius 只是 A* 兜底用的圆模型, 与真实轮廓差太多会分别导致
    // "走廊被判死" 或 "车头方向不够保守".
    {
      double inscribed = 1e9;
      double circum = 0.0;
      double half_w = 0.0;
      for (size_t i = 0; i < configured_footprint_.size(); ++i) {
        const auto & a = configured_footprint_[i];
        const auto & b = configured_footprint_[(i + 1) % configured_footprint_.size()];
        const double ex = b.first - a.first;
        const double ey = b.second - a.second;
        const double l2 = ex * ex + ey * ey;
        double tp = l2 > 1e-12 ? -(a.first * ex + a.second * ey) / l2 : 0.0;
        tp = std::max(0.0, std::min(1.0, tp));
        inscribed = std::min(inscribed, std::hypot(a.first + tp * ex, a.second + tp * ey));
        circum = std::max(circum, std::hypot(a.first, a.second));
        half_w = std::max(half_w, std::fabs(a.second));
      }
      ROS_INFO("[nav] footprint 圆模型: 内切 %.3f / 外接 %.3f / 半宽 %.3f m; "
               "robot_radius=%.3f (A* 兜底用)", inscribed, circum, half_w, robot_radius_);
      if (robot_radius_ + 1e-6 < inscribed) {
        ROS_WARN("[nav] robot_radius < 内切圆: 圆模型会在车体投影内放行, 依赖 footprint 检查兜底");
      } else if (robot_radius_ > half_w + 0.05) {
        ROS_WARN("[nav] robot_radius(%.3f) 明显大于车体半宽(%.3f): 0.8m 级门洞可能被 A* 判死",
                 robot_radius_, half_w);
      }
    }
    buildBodyPolygon();
    configureSafetyLayer();
    {
      core::VelocityShaperConfig scfg;
      scfg.max_v = max_velocity_x_;
      scfg.max_w = max_velocity_theta_;
      scfg.max_acc_v = max_accel_x_;
      scfg.max_acc_w = max_accel_theta_;
      scfg.deadband_v = deadband_velocity_v_;
      scfg.deadband_w = deadband_velocity_w_;
      velocity_shaper_.configure(scfg);
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
    pnh_.param<double>("planner_turn_step_deg", planner_turn_step_deg_, 30.0);
    // unknown(-1) 处理: false = 保持"未知区阻断规划"(旧行为); true = 折成
    // unknown_cost 的高代价, 让规划器尽量绕开但不因为空洞直接拒绝整条路径。
    pnh_.param<bool>("allow_unknown", allow_unknown_, false);
    pnh_.param<int>("unknown_cost", unknown_cost_, 99);
    // 路径轮廓检查的起点豁免半径: 贴墙/门框起步时, 车自身足迹压在墙上属正常。
    pnh_.param<double>("path_footprint_check_skip", path_footprint_check_skip_, 0.5);
    // 两点导航到点后是否对齐目标航向 (旧行为 false: 只到点, 不管车头朝哪)
    pnh_.param<bool>("two_point_final_rotate", two_point_final_rotate_, false);
    // 路径跟踪 (navi_cmd=3) 走完后是否对齐终点位姿的 yaw
    pnh_.param<bool>("path_final_rotate", path_final_rotate_, false);
    // 单点导航末端朝向来源: true = 用 tracking_path[0]->[1] 的方向 (APP 常用它表示
    // "下一段延线往哪走"); false = 用 tracking_path[0].theta 原值。
    pnh_.param<bool>("final_yaw_from_next_point", final_yaw_from_next_point_, false);
    // cmd=1 带多个点时: 用最后一点当目标 (APP 常见的 [当前位置, 目的地] 写法)
    pnh_.param<bool>("single_point_use_last_pose", single_point_use_last_pose_, false);

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
      // 覆盖率控制器以前完全没有车体轮廓/障碍视野, 只能贴几何线走, 全靠安全层兜底
      // (一有人/一贴墙就 STOP -> 6s 无进展 -> 该条带被永久标 BLOCKED)。
      ccfg.footprint = configured_footprint_;
      ccfg.footprint_sample_spacing = footprint_sample_spacing_;
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
    if (!current_core_path_.points.empty()) {
      next->setPath(current_core_path_);   // 已带 velocity_x 方向标记
    } else if (!current_path_.poses.empty()) {
      next->setPath(toCorePath(current_path_));
    }
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
    // 规划在后台跑时不要替换 planner/controller 对象: 虽然 lambda 已按值捕获 planner,
    // controller_strategy_ 仍可能被 setPath 并发使用。等它跑完 (< timeout) 或拒绝。
    if (async_plan_future_.valid() &&
        async_plan_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      ROS_WARN("[nav] refuse profile switch '%s': a plan is still running", profile.c_str());
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
        acfg.turn_step_deg = planner_turn_step_deg_;
        next->initialize(acfg);
        next->setInflationRadius(inflation_radius_);
        next->setRobotRadius(robot_radius_);
        if (next->supportsFootprint() && !configured_footprint_.empty()) {
          next->setRobotFootprint(planner_footprint_);
        }
        if (m_has_planner_costmap_) { next->setCostmap(planner_costmap_); }
        async_planner_ = std::move(next);
        // 窄通道回落: 非 grid 规划器配一个 A* 备用 (栅格 A* 穿窄门是 ms 级)
        if (planner != "astar" && planner != "navfn") {
          auto fb = nav_core::NavigationFactory::createPlanner("astar");
          if (fb) {
            core::PlannerConfig fcfg;
            fcfg.name = "astar";
            fcfg.resolution = 0.05;
            fcfg.max_iterations = planner_max_iterations_;
            fcfg.timeout = planner_timeout_;
            fcfg.allow_reverse = false;
            fcfg.heuristic_weight = planner_heuristic_weight_;
            fb->initialize(fcfg);
            fb->setRobotRadius(robot_radius_);
            fb->setInflationRadius(inflation_radius_);
            fallback_planner_ = std::move(fb);
            ROS_INFO("[plan] fallback planner ready: astar (for narrow passages)");
          }
        }
        planner_name_ = planner;
        planner_allow_reverse_ = allow_reverse;
      }
    }
    // 控制器重建: allow_reverse 在 initialize 时固化
    std::unique_ptr<core::IControllerStrategy> next_ctrl;
    if (initController(controller_name_, next_ctrl, allow_reverse)) {
      if (!current_core_path_.points.empty()) {
        next_ctrl->setPath(current_core_path_);
      } else if (!current_path_.poses.empty()) {
        next_ctrl->setPath(toCorePath(current_path_));
      }
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
    // 死手: 20Hz 检查"还有没有指令在流", 超时就平滑回零
    // (nav2 velocity_smoother 的 velocity_timeout; 以前节点卡死底盘会一直跑最后一条指令)
    command_watchdog_timer_ = nh_.createTimer(
      ros::Duration(0.05), &RosiwitNavigationRos1Node::commandWatchdog, this);
    if (!teleop_topic_.empty()) {
      teleop_sub_ = nh_.subscribe(teleop_topic_, 5,
                                  &RosiwitNavigationRos1Node::teleopCallback, this);
      ROS_INFO("[nav] teleop arbitration enabled: %s (priority over nav, timeout %.2fs)",
               teleop_topic_.c_str(), teleop_timeout_);
    }

    // robot_movebase 兼容层: action + 旧话题. 未启用时全部是空操作.
    mb_.reset(new movebase::Interface(nh_, pnh_));
    if (mb_->enabled()) {
      mb_->configure(
        boost::bind(&RosiwitNavigationRos1Node::movebaseGoalCallback, this, _1),
        boost::bind(&RosiwitNavigationRos1Node::movebaseCancelCallback, this));
    }

    validateConfig();
  }

  // 启动自检: 把"生效配置"打出来, 并对非法/自相矛盾的组合报错.
  // 背景: 真机 yaml 写 hybrid_astar, 但代码默认 motion_profile=clean +
  // clean_planner=astar, 会静默覆盖 -> 真机实际跑 A*. 这类漂移必须启动就暴露.
public:
  ~RosiwitNavigationRos1Node()
  {
    // 规划/逃逸规划的 lambda 都捕获了 this: 关停时必须先等它们结束再析构,
    // 否则正在跑的 plan() 回来会写已释放的成员 (UAF)。先 cancel 再有限等待。
    if (async_planner_) { async_planner_->cancel(); }
    if (async_plan_future_.valid()) {
      if (async_plan_future_.wait_for(std::chrono::seconds(3)) !=
          std::future_status::ready) {
        ROS_WARN("[nav] shutdown: planner still running after 3s (will wait)");
      }
      try { (void)async_plan_future_.get(); } catch (...) {}
    }
    if (recovery_plan_future_.valid()) {
      try { (void)recovery_plan_future_.get(); } catch (...) {}
    }
    // graveyard 里的 future 由 deque 析构等待 (std::async 语义), 这里先显式取一遍
    for (auto & f : recovery_plan_graveyard_) {
      try { (void)f.get(); } catch (...) {}
    }
  }

private:
  void validateConfig()
  {
    const std::string effective_planner =
      (motion_profile_ == "clean") ? clean_planner_ : free_planner_;
    ROS_INFO("[nav] config fingerprint: profile=%s planner=%s(clean=%s free=%s) "
             "controller=%s odom=%s frames=%s/%s footprint_pts=%zu "
             "safety_stop(f/b/r)=%zu/%zu/%zu velocity_timeout=%.2f",
             motion_profile_.c_str(), effective_planner.c_str(),
             clean_planner_.c_str(), free_planner_.c_str(),
             controller_name_.c_str(), odom_topic_.c_str(),
             map_frame_.c_str(), robot_frame_.c_str(), configured_footprint_.size(),
             safety_stop_poly_.size() / 2, safety_back_poly_.size() / 2,
             safety_rot_poly_.size() / 2, velocity_timeout_);

    bool fatal = false;
    if (motion_profile_ != "clean" && motion_profile_ != "free") {
      ROS_ERROR("[nav] motion_profile='%s' 非法 (只支持 clean/free)", motion_profile_.c_str());
      fatal = true;
    }
    const std::string planners[2] = {clean_planner_, free_planner_};
    for (const auto & name : planners) {
      if (!nav_core::NavigationFactory::createPlanner(name)) {
        ROS_ERROR("[nav] planner '%s' 不存在", name.c_str());
        fatal = true;
      }
    }
    if (effective_planner == "astar" && controller_name_ == "rpp") {
      ROS_WARN("[nav] planner=astar + controller=rpp: A* 路径方向已标 +1, 若仍爬行请检查 "
               "trajectory_parameterizer 的 velocity_x 语义");
    }
    if (velocity_timeout_ <= 0.0) {
      ROS_WARN("[nav] velocity_timeout<=0: 死手(静默停车)已关闭, 节点卡死时底盘会保持最后指令");
    }
    if (odom_topic_ == "/odom") {
      ROS_WARN("[nav] odom_topic=/odom: 若定位走 /lio_odom, SLAM 停更时本节点检测不到"
               "(轮速 odom 仍在)");
    }
    if (fatal && strict_config_) {
      ROS_FATAL("[nav] 配置自检失败 (strict_config=true), 退出");
      std::exit(EXIT_FAILURE);
    }
  }

  // ---- 回调 ----
  void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
  {
    last_odom_stamp_ = ros::Time::now();
    current_velocity_ = msg->twist.twist;
    // /odom 是 odom 系: 原样收在 odom_pose_ 里 (里程/打滑/诊断用)。
    // 不能写进 current_pose_: 那个变量的所有消费者 (规划起点/代价地图/
    // robot_footprint 发布) 都按 global_frame_(map) 解释, 不看 frame_id。
    // 本机 map->odom 的 yaw ≈ -90°, 混用就表现为"footprint 与 SLAM 位姿差 90°"。
    odom_pose_.header = msg->header;
    odom_pose_.pose = msg->pose.pose;
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
  {
    current_map_ = msg;
    ++map_seq_;   // 静态层只在有新地图时才需要整图拷贝
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
    // 注意: 时间戳必须在**整帧处理成功之后**再更新。以前在这里先更新、
    // 下面 base_tf 失败却直接 return, 结果 laser_points_ 还是上一帧的旧点,
    // 安全层却用"新位姿 + 旧点"变换 -> 幻影障碍 -> 随机停车。
    // (时间戳更新移到 base_tf 成功之后)
    // 一次性自检: 停止区阈值是**绝对点数**, 但每束雷达在 0.95m 处能采到的点数
    // 取决于 angle_increment。注释里的"0.8m 处 10cm 障碍约 20 点"只在 0.36°/束成立;
    // 换成 0.5~1°/束的雷达后, 细障碍永远凑不够点数 -> 安全层静默漏检。
    if (!scan_geom_logged_ && msg->angle_increment > 1e-6f) {
      scan_geom_logged_ = true;
      const double far = 0.95;   // 前向停止区远端
      const double expect = 0.05 / (far * static_cast<double>(msg->angle_increment));
      ROS_INFO("[safety] scan angle_increment=%.4f rad (%.2f deg): 5cm 障碍在 %.2fm "
               "处约 %.1f 点; 阈值 stop=%d slow=%d (含义: >N 点判停)",
               msg->angle_increment, msg->angle_increment * 57.2957795, far, expect,
               safety_stop_max_points_, safety_slowdown_max_points_);
      if (expect < safety_stop_max_points_) {
        ROS_WARN("[safety] 角分辨率不足: 5cm 障碍在该距离只有 %.1f 点, 而阈值是 %d 点 "
                 "-> 细障碍/区远端会静默漏检 (请按现场雷达降 safety_stop_max_points)",
                 expect, safety_stop_max_points_);
      }
    }

    // /scan 与 3D 前端使用同一套 base 系车体包围盒自过滤.
    geometry_msgs::TransformStamped base_tf;
    if (scan_self_filter_enabled_) {
      try {
        base_tf = tf_buffer_.lookupTransform(robot_frame_, msg->header.frame_id,
                                             msg->header.stamp, ros::Duration(0.05));
      } catch (tf2::TransformException &) {
        // 扫描时刻的 TF 偶发不可用: 先用最新 TF 顶上, 不丢整帧。
        try {
          base_tf = tf_buffer_.lookupTransform(robot_frame_, msg->header.frame_id,
                                               ros::Time(0), ros::Duration(0.05));
          ROS_WARN_THROTTLE(2.0, "[scan] TF at scan stamp unavailable, using latest TF");
        } catch (tf2::TransformException &) {
          // 两个都拿不到: 保留上一帧点云 + 旧时间戳 (二者仍自洽), 不更新 fresh;
          // 超过 scan_timeout 后由 sensorFailSafe/applySafety 统一 fail-safe 停车。
          ROS_WARN_THROTTLE(2.0, "[scan] TF unavailable (stamp+latest); frame skipped");
          return;
        }
      }
    }
    // 整帧确认可用后才更新时间戳 (见上方注释)
    last_scan_stamp_ = ros::Time::now();
    last_scan_msg_stamp_ = msg->header.stamp;

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
        // 只排除车身轮廓内的点 (nav2 exclusion_zones.robot_body).
        // 以前用 [-0.45..0.75, +-0.55] 的大矩形, 把 safety_stop_polygon 整个吃掉了.
        bool is_self = false;
        if (body_polygon_.size() >= 3) {
          is_self = core::pointInPolygon(bp.point.x, bp.point.y, body_polygon_);
        } else {
          is_self = (bp.point.x >= scan_self_x_min_ && bp.point.x <= scan_self_x_max_ &&
                     bp.point.y >= scan_self_y_min_ && bp.point.y <= scan_self_y_max_);
        }
        if (is_self) { continue; }
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
    // 空闲期调 /nav_stop 不应覆盖上一次的 FAILED_* 结论
    const bool was_active = task_active_ || is_navigating_ || planning_active_;
    stopNavigation();
    if (task_active_) {
      clearActionState();
      if (mb_) { mb_->preempted(); }
    }
    if (was_active) { setGoalStatus("STOPPED"); }
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
    // 指令唯一出口, 顺序对齐 nav2:
    //   限速区 -> 遥控仲裁(twist_mux) -> 整形(velocity_smoother) -> 安全层(collision_monitor)
    // 安全层放在最后: 它是硬闸, 不能被加速度限幅拖慢.
    const ros::Time now = ros::Time::now();
    geometry_msgs::Twist cmd = applySpeedLimit(cmd_in);
    if (teleopActive()) {
      cmd = applySpeedLimit(teleop_cmd_);   // 遥控优先 (nav2 twist_mux 的 priority)
    }
    const double dt = last_cmd_stamp_.isZero()
      ? 0.1 : std::max(1e-3, std::min(0.5, (now - last_cmd_stamp_).toSec()));
    last_cmd_stamp_ = now;
    const core::VelocityCommand shaped =
      velocity_shaper_.shape(toCoreVelocity(cmd), dt);
    const geometry_msgs::Twist shaped_ros = toRosTwist(shaped);
    const geometry_msgs::Twist out = applySafety(shaped_ros);
    cmd_vel_pub_.publish(out);
    if (mb_) { mb_->publishCmdVel(out); }
    last_published_cmd_ = out;
    last_published_nonzero_ = (std::fabs(out.linear.x) > 1e-4 ||
                               std::fabs(out.angular.z) > 1e-4);
    // 输入命令本来就是零 (停车/等待/规划中) 时, 安全层的 0 不算"被挡住"
    last_cmd_input_zero_ = (std::fabs(cmd_in.linear.x) < 1e-3 &&
                            std::fabs(cmd_in.angular.z) < 1e-3);
    // 安全层刚做过硬停: 重建整形器状态, 否则恢复时会从"停前速度"继续斜坡
    if (last_safety_hard_stop_) { velocity_shaper_.reset(); }
  }

  // 死手 (nav2 velocity_smoother 的 velocity_timeout): 一段时间没有任何指令 -> 平滑回零.
  // 以前节点卡死/线程饿死时, 底盘会一直执行最后一条非零 cmd_vel.
  void commandWatchdog(const ros::TimerEvent &)
  {
    if (velocity_timeout_ <= 0.0 || last_cmd_stamp_.isZero()) { return; }
    const double idle = (ros::Time::now() - last_cmd_stamp_).toSec();
    if (idle < velocity_timeout_) { return; }
    if (!last_published_nonzero_) { return; }
    const double dt = std::min(0.2, std::max(0.02, idle));
    const core::VelocityCommand z = velocity_shaper_.shape(core::VelocityCommand(), dt);
    const geometry_msgs::Twist out = toRosTwist(z);
    cmd_vel_pub_.publish(out);
    if (mb_) { mb_->publishCmdVel(out); }
    last_published_cmd_ = out;
    if (std::fabs(z.linear_x) < 1e-3 && std::fabs(z.angular_z) < 1e-3) {
      last_published_nonzero_ = false;
    }
  }

  void buildBodyPolygon()
  {
    body_polygon_.clear();
    if (configured_footprint_.size() < 3) { return; }
    double x0 = configured_footprint_[0].first;
    double x1 = x0;
    double y0 = configured_footprint_[0].second;
    double y1 = y0;
    for (const auto & p : configured_footprint_) {
      x0 = std::min(x0, p.first);
      x1 = std::max(x1, p.first);
      y0 = std::min(y0, p.second);
      y1 = std::max(y1, p.second);
    }
    const double m = std::max(0.0, safety_body_margin_);
    body_polygon_ = {{x0 - m, y0 - m}, {x1 + m, y0 - m}, {x1 + m, y1 + m}, {x0 - m, y1 + m}};
    ROS_INFO("[safety] body exclusion box x[%.2f, %.2f] y[%.2f, %.2f] (margin %.2f)",
             x0 - m, x1 + m, y0 - m, y1 + m, m);
  }

  void configureSafetyLayer()
  {
    core::SafetyMonitorConfig cfg;
    cfg.enabled = safety_enabled_;
    cfg.body_polygon = body_polygon_;
    cfg.stop_polygon = toPolygon(safety_stop_poly_);
    cfg.stop_polygon_backward = toPolygon(safety_back_poly_);
    cfg.stop_polygon_rotation = toPolygon(safety_rot_poly_);
    cfg.stop_max_points = safety_stop_max_points_;
    cfg.slowdown_polygon = toPolygon(safety_slow_poly_);
    cfg.slowdown_max_points = safety_slowdown_max_points_;
    cfg.slowdown_ratio = safety_slowdown_ratio_;
    cfg.approach_enabled = safety_approach_enabled_;
    cfg.time_before_collision = safety_time_before_collision_;
    cfg.simulation_time_step = 0.1;
    cfg.footprint = toPolygon(safety_footprint_poly_);
    // 旋转区用扫掠圆: 半径 = 车体最远顶点 + 余量 (矩形会在对角方向多判 0.3m+)
    if (safety_rotation_radius_margin_ > 0.0 && configured_footprint_.size() >= 3) {
      double r = 0.0;
      for (const auto & v : configured_footprint_) {
        r = std::max(r, std::hypot(v.first, v.second));
      }
      cfg.stop_radius_rotation = r + safety_rotation_radius_margin_;
      ROS_INFO("[safety] rotation zone = circle r=%.2f (swept %.2f + margin %.2f)",
               cfg.stop_radius_rotation, r, safety_rotation_radius_margin_);
    }
    cfg.stop_trigger_frames = safety_stop_trigger_frames_;
    cfg.stop_release_frames = safety_stop_release_frames_;
    cfg.slowdown_trigger_frames = safety_slowdown_trigger_frames_;
    cfg.slowdown_release_frames = safety_slowdown_release_frames_;
    cfg.dynamic_geometry = safety_dynamic_geometry_;
    cfg.brake_decel = safety_brake_decel_;
    cfg.reaction_time = safety_reaction_time_;
    cfg.stop_margin = safety_stop_margin_;
    cfg.side_margin = safety_side_margin_;
    cfg.turn_margin = safety_turn_margin_;
    cfg.slowdown_extra = safety_slowdown_extra_;
    cfg.scale_slew_rate = safety_scale_slew_rate_;
    cfg.scale_dt = safety_scale_dt_;
    safety_monitor_.configure(cfg);
    ROS_INFO("[safety] zones: stop_fwd=%zu stop_back=%zu stop_rot=%zu slowdown=%zu "
             "hysteresis=%d/%d body=%zu",
             cfg.stop_polygon.size(), cfg.stop_polygon_backward.size(),
             cfg.stop_polygon_rotation.size(), cfg.slowdown_polygon.size(),
             cfg.stop_trigger_frames, cfg.stop_release_frames, cfg.body_polygon.size());
  }

  bool teleopActive() const
  {
    if (teleop_topic_.empty() || teleop_stamp_.isZero()) { return false; }
    return (ros::Time::now() - teleop_stamp_).toSec() < teleop_timeout_;
  }

  void teleopCallback(const geometry_msgs::Twist::ConstPtr & msg)
  {
    teleop_cmd_ = *msg;
    teleop_stamp_ = ros::Time::now();
  }

  void movebaseCancelCallback()
  {
    ROS_INFO("[movebase_iface] 取消当前导航");
    // 以前这里不写 /goal_status: 取消后状态永远停在 ACCEPTED, 上位机按它判状态会误判。
    setGoalStatus("CANCELED");
    stopNavigation();
    clearActionState();
  }

  // ---- 位姿历史: 安全层要把"扫描时刻"的点云换到 base 系 ----
  struct PoseSample { ros::Time t; double x, y, yaw; };
  std::deque<PoseSample> pose_hist_;
  bool scan_geom_logged_ = false;
  ros::Time last_scan_msg_stamp_;   // /scan 的 header.stamp (安全层对齐用)

  void recordPoseSample(const ros::Time & t)
  {
    PoseSample s;
    s.t = t;
    s.x = current_pose_.pose.position.x;
    s.y = current_pose_.pose.position.y;
    s.yaw = yawFromQuat(current_pose_.pose.orientation);
    pose_hist_.push_back(s);
    while (!pose_hist_.empty() && (t - pose_hist_.front().t).toSec() > 3.0) {
      pose_hist_.pop_front();
    }
  }

  /// 取 t 时刻(或之前最近)的位姿; 找不到返回 false。
  bool poseAt(const ros::Time & t, double & x, double & y, double & yaw) const
  {
    for (auto it = pose_hist_.rbegin(); it != pose_hist_.rend(); ++it) {
      if (it->t <= t) { x = it->x; y = it->y; yaw = it->yaw; return true; }
    }
    return false;
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
        // APP 对"单点导航"往往下发两个点: [到达点, 延线下一点]。真实意图是到位后
        // 让车头朝向**下一段延线方向**, 而 tracking_path[0].theta 常常是 0(未填),
        // 直接用它会让车头正好差 180°(实测 2026-10-31)。开关打开时用 p0→p1 方向。
        if (final_yaw_from_next_point_ && action_goal_.tracking_path.size() >= 2) {
          const core::Pose2D & a = action_goal_.tracking_path[0];
          const core::Pose2D & b = action_goal_.tracking_path[1];
          const double dx = b.x - a.x;
          const double dy = b.y - a.y;
          if (std::hypot(dx, dy) > 0.1) {
            command.final_yaw = std::atan2(dy, dx);
            ROS_INFO("[movebase_iface] final_yaw 取 p0->p1 方向 = %.1fdeg (app theta=%.1fdeg)",
                     command.final_yaw * 57.2957795, target.theta * 57.2957795);
          }
        }
        break;
      case movebase::Command::TWO_POINTS:
        command.kind = core::ActionKind::NAVIGATE;
        // 两点导航历史上只保证"到点", 不对齐末端航向 -> 若最后一段是倒车/大角度切入,
        // 到点时车头可能是反的 (实测 2026-10-31: 末端 yaw 与目标差 ~180°)。
        // 用参数控制: 打开后到点再原地对齐到目标位姿的 yaw。
        command.final_rotate = two_point_final_rotate_;
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
    // 把起点/终点打出来: 只有这样才能区分"APP 让车去远处"和"APP 发了个原地目标"
    // (实测 2026-10-31: 连续几条 navi_cmd 都在同一个点 '到达', 车其实没动)。
    if (!goal.tracking_path.empty()) {
      const core::Pose2D & s = goal.tracking_path.front();
      const core::Pose2D & t = goal.tracking_path.back();
      ROS_INFO("[movebase_iface]   path[0]=(%.2f,%.2f,yaw=%.1fdeg) path[last]=(%.2f,%.2f,yaw=%.1fdeg)",
               s.x, s.y, s.theta * 57.2957795, t.x, t.y, t.theta * 57.2957795);
    }

    // actionlib 在收到新 goal 前会先触发 preempt; 这里再清一次, 保证状态干净.
    clearActionState();
    action_interface_active_ = true;
    action_traveled_.clear();
    action_goal_ = goal;
    action_goal_.single_point_use_last_pose = single_point_use_last_pose_;
    action_cmd_ = movebase::decodeGoal(action_goal_);

    if (!action_cmd_.valid || !action_cmd_.supported) {
      ROS_WARN("[movebase_iface] navi_cmd=%d 当前核心不支持或参数非法, 回 result=%d",
               goal.navi_cmd, action_cmd_.reject_result);
      finishTask(action_cmd_.reject_result, "REJECTED");
      return;
    }
    if (action_cmd_.command == movebase::Command::CANCEL) {
      stopNavigation();
      finishTask(movebase::RESULT_SUCCESS, "CANCELED");   // 结果码是老契约, 状态串要如实
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
    // ★ 最终朝向必须在**规划时**就定下来, 而不是到位后再原地转:
    //   Hybrid A* 的状态含 θ (goal 接受判据 dtheta<=0.45rad, 且末点 pose 直接
    //   吸附到 goal), 所以把 final_yaw 塞进 goal.orientation, 路径会自己收尾到
    //   目标朝向; 否则会先按 app 给的 theta(=0) 规划, 到位后再原地转 180°
    //   (实测 2026-10-31: 15 s 原地旋转)。
    core::ActionTaskCommand task_cmd = toTaskCommand(action_cmd_, target);
    if (task_cmd.final_rotate) {
      target.theta = task_cmd.final_yaw;
      goal.pose.orientation = quatFromYaw(task_cmd.final_yaw);
    }
    beginNavigationTask(goal, task_cmd, start,
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
    // 换路径必须同时刷新 core 版本: 否则下面"优先用 current_core_path_"会拿到
    // 上一条 NAVIGATE 的旧路径 (控制器跟着旧路径跑, 而 /global_path 发的是新路径)。
    current_core_path_ = toCorePath(current_path_);

    core::ActionTaskCommand task_command;
    task_command.kind = core::ActionKind::FOLLOW_PATH;
    // 走完路径后的朝向: 以前完全没设 final_yaw, 到点就用"当时车头"收尾
    // (实测 2026-10-31: 路径跟随后 yaw 与终点朝向差 125°, 任务却报 SUCCEEDED)。
    task_command.final_rotate = path_final_rotate_;
    task_command.final_yaw = path.back().theta;
    if (!action_task_.start(task_command, toCorePose(current_pose_),
                            ros::Time::now().toSec())) {
      failTask(movebase::RESULT_PLAN_FAILED, "FAILED_PLAN");
      return;
    }

    if (!current_core_path_.points.empty()) {
      controller_strategy_->setPath(current_core_path_);
    } else {
      controller_strategy_->setPath(toCorePath(current_path_));
    }
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
    publishCommand(cmd);
    recordActionTraveled();
    publishActionOutputs();
  }

  void publishBackOffStep()
  {
    geometry_msgs::Twist cmd;
    cmd.linear.x = action_task_.backOffCommand();
    publishCommand(cmd);
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
    coverage_config_.turn_radius = coverage_turn_radius_;
    coverage_config_.headland_width = coverage_headland_width_;
    coverage_config_.doorway_width = coverage_doorway_width_;
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
      // 清扫宽度必须 >= 2*内缩: 否则墙边那条 (2*inset - sweep) 宽的带永远标记不到,
      // 覆盖率天花板 = 1 - (周长*带宽)/面积 (5x5m 房间约 93%), coverage_complete_rate
      // 设 0.99 永远到不了, 只能靠 retry 超时收场。
      const double need = 2.0 * coverage_config_.robot_inset;
      if (coverage_config_.sweep_width < need) {
        ROS_WARN("[coverage] sweep_width=%.2f < 2*robot_inset=%.2f -> raise to %.2f",
                 coverage_config_.sweep_width, need, need);
        coverage_config_.sweep_width = need;
      }
      ROS_INFO("[coverage] robot_inset=%.2f m, sweep_width=%.2f m (footprint half-width)",
               coverage_config_.robot_inset, coverage_config_.sweep_width);
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

    coverage_route_.clear();
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

    // 全图可达性 BFS 不必 20Hz 重跑: 栅格只在覆盖/障碍更新时变化, 而"不可达"的
    // 处理路径本来就由 coverage_retry_timeout_ (秒级) 兜底。缓存 0.5s 或车移动
    // 超过 0.15m 再重算, 把每帧 O(栅格) 的 BFS 降到 2Hz。
    if (coverage_reach_check_time_.isZero() ||
        (now - coverage_reach_check_time_).toSec() >= 0.5 ||
        pose.distanceTo(coverage_reach_pose_) > 0.15) {
      coverage_reach_check_time_ = now;
      coverage_reach_pose_ = pose;
      coverage_reach_result_ = coverage_grid_.hasUncoveredReachable(pose, true);
    }
    if (!coverage_reach_result_) {
      if (coverage_grid_.hasAnyUncovered()) {
        if ((now - coverage_last_progress_).toSec() > coverage_retry_timeout_) {
          ROS_WARN("[coverage] %.0fs 内没有进展, 把剩余未覆盖标为不可达",
                   coverage_retry_timeout_);
          coverage_grid_.markAllUncoveredBlocked();
          finishCoverage();
        } else {
          coverage_state_ = "WAITING_OBSTACLE";
          publishCommand(geometry_msgs::Twist());
          publishActionOutputs();
        }
        return;
      }
      finishCoverage();
      return;
    }

    // 行末"停一拍": 旋转点即使不需要转 (朝向已对齐), 也要先把速度收掉,
    // 否则 pure_pursuit 会直接切向下一段的远点 -> 行末过冲 0.2~0.4m,
    // 实测冲出覆盖区域后贴在障碍上, 旋转扫掠圆 (0.64m) 里就有东西 -> 卡死.
    if (!cov_rot_hold_until_.isZero() && now < cov_rot_hold_until_) {
      coverage_state_ = "ROT_HOLD";
      publishCommand(geometry_msgs::Twist());
      publishActionOutputs();
      return;
    }

    // 脱困后退优先: 转不开时先沿车尾方向退一点 (仍在安全层管辖下)
    if (!cov_backoff_until_.isZero() && now < cov_backoff_until_) {
      geometry_msgs::Twist back;
      back.linear.x = -std::fabs(cov_backoff_speed_);
      coverage_state_ = "BACKOFF";
      publishCommand(back);
      publishActionOutputs();
      return;
    }

    if (current_path_.poses.size() >= 2) {
      // 旋转点优先: 显式停-转, 不要交给 pure_pursuit 抹成弧线
      double w_rot = 0.0;
      if (executeCoverageRotation(pose, w_rot)) {
        geometry_msgs::Twist rot;
        rot.linear.x = 0.0;
        rot.angular.z = w_rot;
        coverage_state_ = "ROTATING";
        publishCommand(rot);
        // 行末贴障碍时, 车体旋转扫掠圆 (0.64m) 可能真的转不开; 安全层会 STOP.
        // 一直干等只会 "no progress -> 标 blocked -> 重规划" 死循环,
        // 所以先沿原方向退一点再转 (nav2 的 BackUp-then-Spin 同理).
        if (last_safety_state_ == "STOP" || last_safety_state_ == "STALE") {
          if (cov_rot_blocked_since_.isZero()) { cov_rot_blocked_since_ = now; }
          if (cov_backoff_count_ < cov_backoff_max_ &&
              (now - cov_rot_blocked_since_).toSec() > cov_rot_blocked_timeout_) {
            ROS_WARN("[coverage] 原地转被挡住 %.1fs -> 后退 %.2fs 脱困 (#%d)",
                     (now - cov_rot_blocked_since_).toSec(), cov_backoff_time_,
                     cov_backoff_count_ + 1);
            cov_backoff_until_ = now + ros::Duration(cov_backoff_time_);
            cov_rot_blocked_since_ = ros::Time(0);
            cov_rot_engaged_ = false;      // 退完重新对正
            cov_rot_dir_ = 0;
            ++cov_backoff_count_;
          }
        } else {
          cov_rot_blocked_since_ = ros::Time(0);
        }
        publishActionOutputs();
        return;
      }
      cov_rot_dir_ = 0;
      cov_rot_blocked_since_ = ros::Time(0);

      coverage_state_ = "COVERING";
      geometry_msgs::Twist cmd = coverageCommand();
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
      publishCommand(geometry_msgs::Twist());
      publishActionOutputs();
    }
  }

  // 覆盖路线里的原地转点执行器.
  //
  // 为什么需要: pure_pursuit 只跟踪位置, 不认识"位置不变、朝向变化"的旋转点,
  // 于是"停->转->走"被它抹成弧线 (实测轨迹里的小环). 差速轮可以原地转
  // (R=0), 长方形车体的限制由安全层的 footprint 保证. 这里显式执行旋转:
  // 到点 -> 停线速度 -> 按锁存方向转到位 -> 交回跟踪.
  bool executeCoverageRotation(const core::Pose2D & pose, double & w_out)
  {
    const auto & pts = coverage_path_core_.points;
    if (pts.size() < 3) return false;

    // 已占用: 一直转到位才交回跟踪.
    // 为什么必须锁存: 旋转执行器(按路径顺序找 0.30m 内的旋转点)和纯跟踪会**互相抢**.
    // 纯跟踪在旋转点附近算出的 w 和这里方向常常相反 (实测 +0.2 与 -0.7 交替),
    // 平均角速度≈0, 机器人在原地抖着不动, 覆盖率卡死. 锁存后旋转期间只有它发令.
    if (cov_rot_engaged_) {
      const double err = normalizeAngle(cov_rot_target_ - pose.theta);
      if (std::fabs(err) < 0.06) {
        cov_rot_engaged_ = false;
        cov_rot_dir_ = 0;
        cov_rot_done_ = cov_rot_idx_ + 1;
        return false;
      }
      if (cov_rot_dir_ == 0) { cov_rot_dir_ = err > 0.0 ? 1 : -1; }
      w_out = cov_rot_dir_ * std::min(max_velocity_theta_,
                                      std::max(0.20, 2.0 * std::fabs(err)));
      return true;
    }

    // 未占用: 按**路径顺序**取第一个够得着的旋转点 (不是全局最近点).
    // 全局最近点会被后面某段转场/别的行端点劫持, 目标朝向在两个旋转点之间来回跳.
    size_t nearest = pts.size();
    for (size_t i = cov_rot_done_; i < pts.size(); ++i) {
      if (std::fabs(pts[i].velocity_x) > 0.1) continue;
      if (std::hypot(pts[i].pose.x - pose.x, pts[i].pose.y - pose.y) <= 0.30) {
        nearest = i;
        break;
      }
    }
    if (nearest >= pts.size()) return false;

    // 同一位置的连续点构成旋转段, 目标朝向取这一段最后一点
    size_t end = nearest;
    const auto & base = pts[nearest].pose;
    while (end + 1 < pts.size()) {
      const auto & q = pts[end + 1].pose;
      if (std::hypot(q.x - base.x, q.y - base.y) > 0.03) break;
      ++end;
    }
    const double target = pts[end].pose.theta;
    const double err = normalizeAngle(target - pose.theta);
    if (std::fabs(err) < 0.06) {
      cov_rot_done_ = end + 1;   // 已经对正, 这段不用转
      cov_rot_hold_until_ = ros::Time::now() + ros::Duration(cov_rot_hold_time_);
      return false;
    }
    cov_rot_engaged_ = true;
    cov_rot_idx_ = end;
    cov_rot_target_ = target;
    cov_rot_dir_ = err > 0.0 ? 1 : -1;
    w_out = cov_rot_dir_ * std::min(max_velocity_theta_,
                                    std::max(0.20, 2.0 * std::fabs(err)));
    return true;
  }

  // 覆盖路线: 一次生成整条剩余路线 (boustrophedon), 不再"每行一次规划".
  //
  // 原因: nav2 的跟踪律 (弧长前视 + 曲率限速 + 接近收缩 + 原地转) 假设一条
  // **稳定的**路径; 逐行重规划会让前视点跳到相邻行/approach 段上, 机器人跟
  // 一条不存在的线 (实测轨迹 162m, 覆盖率 22%).
  // 参考 fields2cover (SwathGenerator + Route) / opennav_coverage (一次出 route).
  void coveragePlanTick()
  {
    if (!coverage_active_ || !current_map_ || !coverage_grid_.ready() ||
        !coverage_controller_) {
      return;
    }
    const ros::Time now = ros::Time::now();
    const core::Pose2D pose = toCorePose(current_pose_);

    std::vector<core::Pose2D> route;

    // ---- 已有活动路线: 先决定"继续跟"还是"重建" ----
    // 注意: 以前 buildRoute() 是每个 tick 都调一次再把结果丢掉 (一次要跑
    // BFS+chamfer+单元分解, 几十 ms), 现在只在真正需要重建时才算.
    if (!coverage_route_.empty()) {
      double best = std::numeric_limits<double>::max();
      for (const auto & rp : coverage_route_) {
        best = std::min(best, pose.distanceTo(rp));
      }
      const bool finished = coverage_route_.size() > 1 &&
        pose.distanceTo(coverage_route_.back()) < 0.15;
      if (!finished && best <= 0.8) {
        // 路线还在, 但已经完全没进展 -> 只把"前方那一条带"标 blocked, 下次重建绕开它
        if ((now - coverage_last_progress_).toSec() > coverage_retry_timeout_) {
          const size_t ahead = std::min(coverage_route_.size() - 1, static_cast<size_t>(20));
          ROS_WARN("[coverage] no progress for %.1fs -> mark strip ahead (%.2f,%.2f) blocked",
                   (now - coverage_last_progress_).toSec(),
                   coverage_route_[ahead].x, coverage_route_[ahead].y);
          coverage_grid_.markBlockedAround(coverage_route_[ahead],
                                           std::max(0.20, coverage_sweep_width_ * 0.5));
          coverage_last_progress_ = now;
          coverage_route_.clear();
          current_path_.poses.clear();
          coverage_controller_->reset();
        }
        return;
      }
      coverage_route_.clear();
    }

    const bool have_route = coverage_grid_.buildRoute(pose, route) && route.size() >= 2;

    if (!have_route) {
      coverage_last_plan_ = now;
      if (coverage_grid_.hasAnyUncovered()) {
        if ((now - coverage_last_progress_).toSec() > coverage_retry_timeout_) {
          // 到这时确实没有可执行的路线: 标掉不可达的格子后收尾.
          // 以前这里"若还有可达未覆盖就继续等", 而 buildRoute 又生成不出行段,
          // 结果永远在 WAITING_OBSTACLE 空转 (实测卡在 98%).
          coverage_grid_.markUnreachable(pose);
          coverage_state_ = "WAITING_OBSTACLE";
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

    core::Path path;
    path.points.reserve(route.size());
    core::Pose2D prev = pose;
    for (const auto & p : route) {
      core::PathPoint pt;
      pt.pose = p;
      // 位置不变 = 原地转 (velocity_x=0), 其余前进 (覆盖不倒车)
      const double step = std::hypot(p.x - prev.x, p.y - prev.y);
      pt.velocity_x = (step < 1e-3) ? 0.0 : 1.0;
      path.points.push_back(pt);
      prev = p;
    }
    if (path.points.size() < 2) {
      current_path_.poses.clear();
      coverage_controller_->reset();
      return;
    }

    coverage_last_plan_ = now;
    coverage_route_ = route;
    coverage_path_core_ = path;
    cov_rot_engaged_ = false;
    cov_rot_done_ = 0;
    cov_rot_dir_ = 0;
    cov_rot_blocked_since_ = ros::Time(0);
    cov_backoff_until_ = ros::Time(0);
    cov_backoff_count_ = 0;
    cov_rot_hold_until_ = ros::Time(0);
    coverage_target_ = route.back();
    coverage_has_target_ = true;
    current_path_ = toRosPath(path, map_frame_, now);
    coverage_controller_->setPath(path);
    coverage_state_ = "COVERING";
    ROS_INFO("[coverage] route: %zu pts (turns=%zu), target=(%.2f,%.2f)", path.points.size(),
             static_cast<size_t>(std::count_if(path.points.begin(), path.points.end(),
               [](const core::PathPoint & p) { return p.velocity_x < 0.5; })),
             coverage_target_.x, coverage_target_.y);
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
    // 必须与 /local_costmap、/global_costmap 同系 (map), 否则叠图会差 map->odom 的旋转。
    if (current_pose_.header.frame_id != global_frame_) { return; }
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
    // 位姿只认 TF: idle 时控制环不跑, 这里补一次刷新, 保证 footprint 是最新的 map 位姿。
    updateCurrentPose();
    publishFootprintPolygon();
    if (!diagnostics_pub_) return;
    std::ostringstream os;
    os << "{\"state\":\"" << (is_navigating_ ? "NAVIGATING" : "IDLE")
       << "\",\"goal_status\":\"" << goal_status_
       << "\",\"safety\":\"" << last_safety_state_
       << "\",\"safety_scale\":" << last_safety_scale_
       << ",\"replan_count\":" << replan_count_
       << ",\"replan_fail\":" << replan_fail_count_
       << ",\"plan_ms\":" << last_plan_ms_
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
       // odom 系位姿 (frame_id 会打出 "odom"): 与 map 系位姿对照, 便于现场区分"两系混用"
       << ",\"odom_pose\":{\"frame\":\"" << odom_pose_.header.frame_id << "\""
       << ",\"x\":" << odom_pose_.pose.position.x
       << ",\"y\":" << odom_pose_.pose.position.y
       << ",\"yaw\":" << (yawFromQuat(odom_pose_.pose.orientation) * 57.2957795) << "}"
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
      // StaticLayer::setOccupancy 是**整图 vector 拷贝**; 以前 1Hz(规划)+5Hz(局部)
      // 各拷一次, 大地图上是几十 MB/s 的无谓 memcpy。按 map 版本去重。
      if (planner_static_seq_ != map_seq_) {
        planner_static_layer_->setOccupancy(current_map_->data, w, h);
        planner_static_seq_ = map_seq_;
      }
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
      // unknown(-1) 策略: 以前整条链路都按"100=碰撞"处理, 只要路径/足迹碰到一块
      // 没扫到的地方就整条路径作废 (实测 2026-10-31: 机器人停在未建图的口袋旁边,
      // 每次规划都在"第 5 点车体压到 cost=-1"处被拒 -> FAILED_PATH_FOOTPRINT)。
      // 语义上 -1 只是"没有数据", 既不是墙也不是代价; 这里按参数把它折成高代价
      // (默认 99), 规划器会尽量避免, 但不会因为一块空洞就拒绝整条路径。
      // 真实障碍仍由 100 与安全层(实时 /scan)兜底。
      const int8_t unknown_repl = static_cast<int8_t>(
        std::max(1, std::min(99, unknown_cost_)));
      const auto fixUnknown = [&](std::vector<int8_t> & v) {
        if (!allow_unknown_) { return; }
        for (auto & c : v) { if (c < 0) { c = unknown_repl; } }
      };
      for (size_t i = 0; i < semantic.size(); ++i) {
        // 层与 Costmap 同一套语义 (-1/0..100), 直接复制.
        cm.data[i] = semantic[i];
      }
      // 规划器两套数组都要改: hybrid 的 isObstacle 读 data(), 足迹检查读 grid->data.
      fixUnknown(cm.data);
      if (grid) { fixUnknown(grid->data); }
      // /scan 是实时传感器障碍层: 只进入 local costmap/controller/safety,
      // 不能直接写成 global planner 的 lethal。否则 15Hz 的瞬态点会被 A*
      // 当成全局永久障碍, 形成大面积假墙并让全局路径无法生成.
      planner_strategy_->setCostmap(cm);
      planner_costmap_ = cm;
      m_has_planner_costmap_ = true;

      // 膨胀并发布供 RViz 可视化
      const int inflate_cells = static_cast<int>(std::ceil(inflation_radius_ / res));
      const int safety_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_ / res)));
      std::vector<int8_t> display =
        core::inflatedCostmap(semantic, w, h, safety_cells, inflate_cells);
      // 显示/平滑视图与规划器保持同一套数值: 否则 /makeplan_map 上还是一片 -1,
      // 现场看到的"未知区"和规划器实际用的"99 代价"不一致, 排查时会被误导。
      fixUnknown(display);

      // 平滑器只看得见传进去的栅格: 以前传的是 raw (0/100) 的 planner_costmap_.grid,
      // 于是它把回退 A* 的折线"拉直"进 99 膨胀区 —— 用户看到的就是"规划穿过代价区"。
      // 这里存一份膨胀后的图 (display, 99=内切/膨胀带), 平滑必须在这上面判定。
      {
        auto smooth_grid = std::make_shared<core::CostmapGrid>();
        smooth_grid->info = grid->info;
        smooth_grid->data = display;
        planner_smooth_grid_ = std::move(smooth_grid);
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
    // 1 字节/格; 以前是 vector<vector<bool>>(h 个 vector), 每次 updateCostmap 都会
    // 分配 4M 个 vector 头 (上限 4M 格 ≈ 100MB+/次, 5Hz) —— 单线程 spinner 下正是
    // 让 commandWatchdog 饿死的元凶之一。
    std::vector<uint8_t> visited(static_cast<size_t>(w) * h, 0);
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
      // 按**米**膨胀 (5cm 物理半径), 不能按格写死 3x3: res=0.05 时每点扩 0.15m,
      // res=0.10 时就是 0.30m —— 0.8m 门洞两侧各被吃掉 0.15m, 直接过不去。
      // 用 floor 而不是 round: res=0.1 时 0.05/0.1=0.5 会被 round 成 1 格 (0.10m),
      // 又回到"按格放大"的老问题; floor 才是"最多扩到 5cm 半径"。
      const int scan_dilate = std::max(0, static_cast<int>(std::floor(0.05 / res)));
      for (const auto & pt : laser_points_) {
        const int lx = static_cast<int>((pt.x - ox) / res);
        const int ly = static_cast<int>((pt.y - oy) / res);
        for (int oy2 = -scan_dilate; oy2 <= scan_dilate; ++oy2) {
          for (int ox2 = -scan_dilate; ox2 <= scan_dilate; ++ox2) {
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
        if (visited[static_cast<size_t>(my) * w + mx]) { continue; }
        if (!isObstacleCell(mx, my)) { continue; }

        std::queue<std::pair<int, int>> cluster_q;
        cluster_q.push({mx, my});
        visited[static_cast<size_t>(my) * w + mx] = 1;
        // 每个连通域再按 0.5m 分块, 每块一个小圆。
        // 以前一个连通域只出一个"质心 + 半径=包围盒半对角"的圆: 6m 长的一整面墙
        // 会变成 r≈3m 的巨圆, DWA 的 clearance = |p-obs| - r 直接变负 -> 候选全丢,
        // 表现就是"实车用 dwa 时原地不动"。
        const double kBlockM = 0.5;
        const int block_cells = std::max(1, static_cast<int>(std::lround(kBlockM / res)));
        struct BlockAcc { int x0 = 1 << 30, x1 = -1, y0 = 1 << 30, y1 = -1; };
        std::map<std::pair<int, int>, BlockAcc> blocks;
        while (!cluster_q.empty()) {
          const std::pair<int, int> c = cluster_q.front(); cluster_q.pop();
          BlockAcc & acc = blocks[std::make_pair(c.first / block_cells, c.second / block_cells)];
          acc.x0 = std::min(acc.x0, c.first); acc.x1 = std::max(acc.x1, c.first);
          acc.y0 = std::min(acc.y0, c.second); acc.y1 = std::max(acc.y1, c.second);
          const int nx4[4] = {c.first - 1, c.first + 1, c.first, c.first};
          const int ny4[4] = {c.second, c.second, c.second - 1, c.second + 1};
          for (int i = 0; i < 4; ++i) {
            const int nx = nx4[i];
            const int ny = ny4[i];
            // 聚类必须限制在机器人附近的窗口内: 整面连通墙会一路蔓延到全图
            // (实测 30 万格), 分块 map 再叠加一次, 单线程 spinner 会被拖住。
            if (nx < local_x0 || nx > local_x1 || ny < local_y0 || ny > local_y1) { continue; }
            if (visited[static_cast<size_t>(ny) * w + nx]) { continue; }
            if (isObstacleCell(nx, ny)) { visited[static_cast<size_t>(ny) * w + nx] = 1; cluster_q.push({nx, ny}); }
          }
        }

        for (const auto & kv : blocks) {
          const BlockAcc & b = kv.second;
          core::Obstacle obs;
          obs.x = ox + (0.5 * (b.x0 + b.x1) + 0.5) * res;
          obs.y = oy + (0.5 * (b.y0 + b.y1) + 0.5) * res;
          const double ww = (b.x1 - b.x0 + 1) * res;
          const double hh = (b.y1 - b.y0 + 1) * res;
          obs.radius = 0.5 * std::hypot(ww, hh);
          obstacles.push_back(obs);
        }
      }
    }
    // 分块后数量可能上百: 只保留离车最近的 128 个, 避免 DWA 每采样 × 每障碍的
    // 三重循环把 10Hz 控制环吃满。
    const size_t kMaxObstacles = 128;
    if (obstacles.size() > kMaxObstacles) {
      std::sort(obstacles.begin(), obstacles.end(),
                [rx, ry](const core::Obstacle & a, const core::Obstacle & b) {
                  const double da = std::hypot(a.x - rx, a.y - ry) - a.radius;
                  const double db = std::hypot(b.x - rx, b.y - ry) - b.radius;
                  return da < db;
                });
      obstacles.resize(kMaxObstacles);
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

    if (local_static_seq_ != map_seq_) {
      local_static_layer_->setOccupancy(current_map_->data, w, h);
      local_static_seq_ = map_seq_;
    }
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
      // 覆盖率执行用的是另一个控制器实例, 同一份局部代价地图也要给它
      if (coverage_controller_ && coverage_active_) {
        coverage_controller_->setLocalCostmap(grid);
      }
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
    if (fallback_planner_) {
      fallback_planner_->setCostmap(planning_costmap);
      if (!configured_footprint_.empty()) {
        fallback_planner_->setRobotFootprint(planner_footprint_);
      }
    }
    active_plan_generation_ = plan_generation_;
    // 按值捕获 planner 与名字: 之后即使 async_planner_ 被替换/析构, 本任务仍持有对象。
    auto planner = async_planner_;
    auto fallback = fallback_planner_;
    const std::string pname = planner_name_;
    const core::Footprint tight_fp = configured_footprint_;
    const core::Footprint full_fp = planner_footprint_;
    // A* 是圆模型(无航向维度), 单个圆无法同时满足"安全"与"过窄门":
    //   外接圆 r≈0.58: 任何朝向都不碰, 但需要 ~1.16m 通道 -> 0.8m 门洞必然无解;
    //   内切圆 r=robot_radius(0.33): 能过门, 但车头(0.55)转弯时可插进墙 ~0.22m。
    // 所以先用外接圆(安全), 无解再退内切圆, 由上层 pathFootprintClear() 逐点校验。
    double a_star_circum = 0.0;
    for (const auto & v : tight_fp) {
      a_star_circum = std::max(a_star_circum, std::hypot(v.first, v.second));
    }
    a_star_circum += std::max(0.0, footprint_shrink_);   // tight_fp 是内缩过的
    const double a_star_inner = robot_radius_;
    async_plan_future_ = std::async(std::launch::async,
      [this, planner, fallback, pname, start, goal, tight_fp, full_fp,
       a_star_circum, a_star_inner]() {
      if (!planner) {
        return core::Result<core::Path>::error(
          core::ErrorCode::PLANNING_FAILED, "planner gone");
      }
      auto primary = planner->plan(start, goal);
      if (primary.is_ok()) {
        last_plan_used_fallback_.store(false);
        return primary;
      }
      // 第二档: 用**实际车体轮廓**(configured, 不含 0.06 安全余量)再试一次。
      // 很多 "No valid path found" 是余量把窄处判死, 而不是真的过不去;
      // 实车轮廓本身仍是正确的几何约束。
      if (planner->supportsFootprint() && tight_fp.size() >= 3) {
        planner->setRobotFootprint(tight_fp);
        auto retry = planner->plan(start, goal);
        planner->setRobotFootprint(full_fp);   // 复原, 不影响下一次
        if (retry.is_ok()) {
          last_plan_used_fallback_.store(false);
          return retry;
        }
      }
      if (!fallback) {
        last_plan_used_fallback_.store(false);
        return primary;
      }
      // 回退 A*: 先用**外接圆** (任何朝向都安全, 但窄门洞必然无解),
      // 无解再退**内切圆** (能过窄处, 但车头可能压墙) —— 后者由上层
      // pathFootprintClear() 用车体轮廓逐点校验, 不合格直接拒绝。
      if (a_star_circum > 1e-3) {
        fallback->setRobotRadius(a_star_circum);
        auto safe = fallback->plan(start, goal);
        if (safe.is_ok()) {
          ROS_WARN("[plan] %s failed (%s), fallback to A* r=%.3f (外接圆: 任何朝向都安全)",
                   pname.c_str(), primary.error_message().c_str(), a_star_circum);
          last_plan_used_fallback_.store(true);
          return safe;
        }
        ROS_WARN("[plan] A* 外接圆 r=%.3f 无解 (窄处必然), 退内切圆 r=%.3f + 轮廓校验",
                 a_star_circum, a_star_inner);
      }
      fallback->setRobotRadius(a_star_inner);
      ROS_WARN("[plan] %s failed (%s), fallback to A* r=%.3f (内切圆, 需轮廓校验)",
               pname.c_str(), primary.error_message().c_str(), a_star_inner);
      last_plan_used_fallback_.store(true);
      return fallback->plan(start, goal);
    });
  }

  void startPlanning(const geometry_msgs::PoseStamped & goal, bool allow_initial_rotate,
                     bool restart, const core::Pose2D * start_override = nullptr)
  {
    // restart=true: 新目标, 先停旧路径; false: 周期重规划, 继续沿旧路径控制.
    planning_is_replan_ = !restart;
    if (restart) {
      safety_monitor_.reset();
      replan_blocked_count_ = 0;
    }
    if (planning_is_replan_) ++replan_count_;
    plan_start_time_ = ros::Time::now().toSec();
    if (restart) {
      stopNavigation();   // ⚠ 它会清 pending_has_start_override_, 所以 override 必须在它之后写
      current_path_.poses.clear();
      global_path_pub_.publish(current_path_);
      last_motion_stamp_ = ros::Time::now();
      safety_stop_since_ = ros::Time();
    } else {
      is_navigating_ = true;  // 重规划期间旧路径继续可用
    }
    pending_has_start_override_ = (start_override != nullptr);
    if (start_override != nullptr) { pending_start_override_ = *start_override; }
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

    // 结果属于旧目标: 丢弃。只有"仍在导航且有新目标"时才对最新目标重启规划,
    // 否则 /nav_stop 之后的旧结果会把车重新开出去 (见 stopNavigation 注释)。
    if (active_plan_generation_ != plan_generation_) {
      if (is_navigating_ && !pending_goal_.header.frame_id.empty()) {
        planning_active_ = true;
        launchPlan();
      }
      return;
    }

    if (!result.is_ok()) {
      ROS_ERROR("[rosiwit_navigation_ros1] planner '%s' failed: %s",
                planner_name_.c_str(), result.error_message().c_str());
      // 周期重规划失败 != 当前路径失效。旧路径仍可安全跟踪时, 只记录并延后
      // 下一次重规划; 以前这里直接 recovery/FAILED_PLAN, 表现为"走一半突然
      // 后退重规划/整单失败", 是现场"经常重规划、规划失败"的主要来源之一。
      if (planning_is_replan_ && !current_core_path_.points.empty()) {
        ROS_WARN("[plan] replan failed (%s); keep current path, retry later",
                 result.error_message().c_str());
        ++replan_fail_count_;
        last_plan_ms_ = (ros::Time::now().toSec() - plan_start_time_) * 1000.0;
        last_plan_time_ = ros::Time::now();   // 避免 blocked 每 1s 连续重试
        planning_active_ = false;
        return;
      }
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
    last_plan_ms_ = (ros::Time::now().toSec() - plan_start_time_) * 1000.0;
    applyPlanningResult(result, pending_allow_initial_rotate_);
  }

  void applyPlanningResult(const core::Result<core::Path> & result, bool allow_initial_rotate)
  {
    // 规划结果可能来自已被取消的那一代 (stopNavigation / 新目标 / 位姿跳变都 ++generation)。
    // pollPlanning 已挡一层, 这里再挡一层: 结果一旦过期, 绝不能落到 current_path_。
    if (active_plan_generation_ != plan_generation_) {
      ROS_WARN("[nav] drop stale planning result (gen %lu != %lu)",
               static_cast<unsigned long>(active_plan_generation_),
               static_cast<unsigned long>(plan_generation_));
      return;
    }
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
    bool smoothing_rejected = false;   // 平滑会把代价抬高 -> 保留原路径, 不能谎报 smoothed
    // 回退到 A* 时必须平滑: A* 是栅格折线, 不平滑会一路切角; 而 planner_name_ 仍是
    // 配置里的 hybrid_astar, 以前这个条件把回退路径的平滑整个跳过了 (smoothed=0)。
    if (planned.points.size() >= 3 &&
        (planner_name_ != "hybrid_astar" || last_plan_used_fallback_.load())) {
      const core::Path raw = planned;
      planners::SmootherConfig smoother;
      smoother.max_iterations = 60;
      smoother.alpha_start = 0.45;
      smoother.alpha_end = 0.10;
      smoother.max_cost_increase = 0;
      const auto smooth_map =
        planner_smooth_grid_ ? planner_smooth_grid_ : planner_costmap_.grid;
      core::Path candidate = planners::smoothPath(raw, smooth_map, smoother);
      // 双保险: 平滑结果的最高代价不得高于原始路径 (99=膨胀带, 100=障碍)。
      // smoothPath 内部的 max_cost_increase 是按段比的, 这里按整条路径再兜一层。
      if (candidate.points.size() == raw.points.size() && smooth_map) {
        const auto maxCost = [&smooth_map](const core::Path & p) -> int {
          int worst = 0;
          const auto & gi = smooth_map->info;
          for (const auto & pt : p.points) {
            const int gx = static_cast<int>((pt.pose.x - gi.origin.position.x) / gi.resolution);
            const int gy = static_cast<int>((pt.pose.y - gi.origin.position.y) / gi.resolution);
            if (gx < 0 || gy < 0 || gx >= static_cast<int>(gi.width) ||
                gy >= static_cast<int>(gi.height)) {
              continue;
            }
            const int v = static_cast<int>(
              smooth_map->data[static_cast<size_t>(gy) * gi.width + gx]);
            // 与 path_smoother 的 cellCost 同一语义: 负数(unknown)按阻断算,
            // 否则"平滑进未知区"不会被这道校验拦住。
            worst = std::max(worst, v < 0 ? 100 : v);
          }
          return worst;
        };
        const int raw_cost = maxCost(raw);
        const int new_cost = maxCost(candidate);
        if (new_cost > raw_cost) {
          ROS_WARN("[plan] 平滑会把路径代价从 %d 抬到 %d (99=膨胀带): 放弃平滑, 保留原始路径",
                   raw_cost, new_cost);
          candidate = raw;
          smoothing_rejected = true;
        } else if (raw_cost >= 99) {
          ROS_WARN("[plan] 原始路径已穿过膨胀带 (cost=%d, 99=内切/膨胀): "
                   "这是回退规划器(A*)的能力边界, 建议保留 hybrid_astar", raw_cost);
        }
      }
      if (!smoothing_rejected && candidate.points.size() == raw.points.size()) {
        for (size_t i = 0; i < candidate.points.size(); ++i) {
          candidate.points[i].pose.theta = raw.points[i].pose.theta;
          candidate.points[i].velocity_x = raw.points[i].velocity_x;
        }
        planned = candidate;
        smoothed = true;
      }
    }

    // 最后一道闸: 路径不得让**实际车体轮廓**压到真实障碍。
    // (hybrid 本来就按 planner_footprint_=configured+0.06 检查, 能过;
    //  会失败的基本只有回退 A* 的圆模型路径 —— 实测它会让车头插进墙里。)
    std::string fp_why;
    if (!pathFootprintClear(planned, &fp_why)) {
      // 重规划(1Hz)被拒 != 任务失败: 旧路径仍然可用, 直接丢掉这一版继续走,
      // 下一拍再规划。以前每条被拒的重规划都会进恢复并最终 FAILED_PATH_FOOTPRINT
      // (实测 2026-10-31: 走了 1 m 之后被重规划拒掉, 整单结束 -> "走一半停了")。
      if (planning_is_replan_ && !current_core_path_.points.empty()) {
        ROS_WARN("[plan] 重规划被轮廓检查拒绝 (%s), 保留旧路径继续走", fp_why.c_str());
        planning_active_ = false;
        ++plan_generation_;
        return;
      }
      ROS_ERROR("[plan] 拒绝执行: 路径让车体轮廓压到障碍 (%s) -> 后退重规划",
                fp_why.c_str());
      planning_active_ = false;
      ++plan_generation_;
      // 只允许"后退重规划"一次: 以前每次拒绝都进恢复, 而回退 A* 的路径在窄处
      // 反正过不了闸 -> 恢复 -> 再拒绝 -> 无限循环, 表现为"下目标没反应"。
      if (recovery_attempts_ < 1 && startRecovery()) { return; }
      if (task_active_) {
        failTask(movebase::RESULT_PLAN_FAILED, "FAILED_PATH_FOOTPRINT");
      } else {
        setGoalStatus("FAILED_PATH_FOOTPRINT");
        stopNavigation();
      }
      return;
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
    // 起步原地旋转只在"路还长"时值得做: 目标就在车边上 (实测 cmd=1 的
    // tracking_path[0] 距车 0.2 m) 时先转 180° 再挪 20 cm, 纯属浪费 7 秒,
    // 而且会和末端对齐叠成"原地绕很久"。
    double path_span = 0.0;
    if (planned.points.size() >= 2) {
      const auto & a = planned.points.front().pose;
      const auto & b = planned.points.back().pose;
      path_span = std::hypot(b.x - a.x, b.y - a.y);
    }
    if (allow_initial_rotate && planned.points.size() > 2 && !first_reverse &&
        path_span > 0.6) {
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
    const double rx = current_pose_.pose.position.x;
    const double ry = current_pose_.pose.position.y;
    // 只检查机器人前方 [skip, lookahead] 一段: 身后/已走过的点没有意义,
    // 也避免定位小跳变让旧路径段落进障碍 -> 误判 blocked 每分钟重规划。
    for (size_t i = 0; i < current_path_.poses.size(); i += 1) {
      const auto & p = current_path_.poses[i].pose.position;
      const double d = std::hypot(p.x - rx, p.y - ry);
      if (d < replan_skip_distance_) { continue; }
      if (d > replan_lookahead_) { continue; }
      const int mx = static_cast<int>(std::floor(
        (p.x - cm.origin_x) / cm.resolution));
      const int my = static_cast<int>(std::floor(
        (p.y - cm.origin_y) / cm.resolution));
      if (mx < 0 || my < 0 || mx >= width || my >= height) { return true; }
      const int8_t c = cm.data[static_cast<size_t>(my) * width + mx];
      // unknown(-1) 在 allow_unknown 时已被折成高代价(默认99), 这里只把
      // 真正的 lethal(>=100) 当 blocked; 否则未知区会每 1s 触发一次重规划。
      if (c >= 100) { return true; }
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
    const bool blocked_raw = replan_on_blocked_ && pathBlocked();
    if (blocked_raw) {
      ++replan_blocked_count_;
    } else {
      replan_blocked_count_ = 0;
    }
    const bool blocked = replan_blocked_count_ >= std::max(1, replan_blocked_frames_);
    const double since_plan = last_plan_time_.isZero() ? 1e9 :
      (now - last_plan_time_).toSec();
    // 快到位时不再按周期重规划: 末段换路径会让 MPPI 参考点跳变, 表现为
    // 到点前"蹭/抖"。只有真的 blocked 才重规划。
    const double gx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
    const double gy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
    const bool near_goal = std::hypot(gx, gy) < 0.8;
    const bool due = path_empty || (!near_goal && since_plan >= replan_max_interval_);
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
    // 作废所有在飞的逃逸规划: 它们的起点位姿/地图坐标系可能已经失效
    // (切图、重定位、新目标都会走到这里)。只递增 generation 不够 —— future 还活着,
    // launchReverseEscape 以前会直接复用它。
    ++recovery_plan_gen_;
    discardRecoveryPlan();
    recovery_stage_ = RecoveryStage::NONE;
    recovery_core_path_.points.clear();
    if (recovery_controller_) { recovery_controller_->reset(); }
    recovery_started_ = ros::Time(0);
  }

  /// 把在飞的逃逸规划移进 graveyard: 不能在控制环里 get() 等它 (最长 ~6s),
  /// 也不能原地析构 (async future 的析构会阻塞). 已完成的顺手回收。
  void discardRecoveryPlan()
  {
    if (recovery_plan_future_.valid()) {
      recovery_plan_graveyard_.push_back(std::move(recovery_plan_future_));
    }
    for (auto it = recovery_plan_graveyard_.begin();
         it != recovery_plan_graveyard_.end();) {
      if (it->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try { (void)it->get(); } catch (...) {}
        it = recovery_plan_graveyard_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // Recovery-only costmap: deep-copy the global planner costmap and mark all
  // cells under the current robot footprint as free.  The robot is physically
  // standing there already; if the static map still marks those cells as
  // occupied, a reverse escape path cannot even get a valid start.  Both
  // Costmap::data and the shared grid copy must be cleared because Hybrid A*
  // uses data for cell checks and grid for footprint-corner checks.
  /// 用车体轮廓逐点验证路径。回退规划器 A* 只有圆形模型 (robot_radius=0.33),
  /// 而车体是 0.55 前 / 0.25 后 —— 它给的路径可能让车头插进墙里 (实测撞上过)。
  /// 这种路径必须拒绝: 宁可后退重规划, 也不能下发。
  bool pathFootprintClear(const core::Path & path, std::string * why = nullptr) const
  {
    if (path.points.size() < 2 || configured_footprint_.size() < 3 ||
        planner_costmap_.data.empty() || planner_costmap_.resolution <= 0.0) {
      return true;   // 缺数据时保持旧行为, 不额外拦
    }
    const auto & cm = planner_costmap_;
    const double res = cm.resolution;
    // 采样粒度按"路径来源"区分:
    //   hybrid 路径: 规划器自己已按 planner_footprint_ + 0.20m 粗采样验证过可行性,
    //     这里用同样的粒度复核即可。以前统一用 0.05m 细采样, 等于比规划器更严,
    //     会把 hybrid 明明可行的路径否掉 (实测 2026-10-31: 第 35 点(距起点 136cm)
    //     cost=100 -> 恢复 -> FAILED_PATH_FOOTPRINT, "走一半停了")。
    //   A* 回退路径: 只有圆模型, 必须细采样兜住切角。
    const bool from_fallback = last_plan_used_fallback_.load();
    const double sample_step = from_fallback ? 0.05 : 0.20;
    const core::Footprint & check_fp =
      (from_fallback || planner_footprint_.size() < 3) ? configured_footprint_ : planner_footprint_;
    const std::vector<std::pair<double, double>> samples =
      core::sampleFootprint(check_fp, sample_step);
    if (samples.empty()) { return true; }
    // 起点邻域豁免: 机器人可能正贴着墙/门框, 它**自身**的足迹本来就压在 100 上;
    // 规划器已经用"起点清障盒"把这一小片清成 free 并验证过起点可达, 这里若再拿
    // 未清障的图去判, 就会"起步即被自己否掉"(实测 2026-10-31: 第 0 点 cost=100 ->
    // 恢复 -> FAILED_PATH_FOOTPRINT)。起点附近 skip 距离内跳过轮廓检查。
    const double skip = std::max(0.0, path_footprint_check_skip_);
    const core::Pose2D & p_start = path.points.front().pose;
    for (size_t i = 0; i < path.points.size(); ++i) {
      const core::Pose2D & p = path.points[i].pose;
      const double from_start = std::hypot(p.x - p_start.x, p.y - p_start.y);
      // 起点自身也算在内: 车贴墙/贴门框时, 起点足迹本来就压在 100 上, 而规划器
      // 已用"起点清障盒"验证过起点可达 (否则会在起点处永远失败)。
      if (skip > 0.0 && from_start < skip) { continue; }
      const double c = std::cos(p.theta), s = std::sin(p.theta);
      for (const auto & q : samples) {
        const double wx = p.x + c * q.first - s * q.second;
        const double wy = p.y + s * q.first + c * q.second;
        const int gx = static_cast<int>(std::floor((wx - cm.origin_x) / res));
        const int gy = static_cast<int>(std::floor((wy - cm.origin_y) / res));
        if (gx < 0 || gy < 0 || gx >= static_cast<int>(cm.width) ||
            gy >= static_cast<int>(cm.height)) {
          continue;
        }
        const int v = cm.data[static_cast<size_t>(gy) * cm.width + gx];
        // 100=真实障碍; 负数=unknown(规划和本检查同一套数值, 已按 allow_unknown 折成代价)。
        if (v >= 100 || v < 0) {
          if (why != nullptr) {
            *why = "第 " + std::to_string(i) + " 点(距起点 " +
                   std::to_string(static_cast<int>(from_start * 100)) +
                   "cm)车体压到 cost=" + std::to_string(v);
          }
          return false;
        }
      }
    }
    return true;
  }

  core::Costmap buildStartClearedCostmap(const core::Pose2D & pose,
                                         double margin,
                                         bool enabled)
  {
    core::Costmap cm = planner_costmap_;
    if (!enabled || cm.data.empty() ||
        cm.width == 0 || cm.height == 0 || cm.resolution <= 0.0) {
      return cm;
    }
    // 清除盒必须覆盖**规划器实际使用的轮廓** (planner_footprint_ = configured 外扩
    // planner_footprint_margin_), 再加半格取整余量。以前用 configured_footprint_ + 0.05,
    // 而规划器按 configured + 0.06 判定, 所以保险杠距墙 5~6cm 时 start 必被判进障碍。
    const std::vector<std::pair<double, double>> & clear_fp =
      planner_footprint_.size() >= 3 ? planner_footprint_ : configured_footprint_;
    // 多边形本身已经含 planner_footprint_margin_(0.06), 这里只补半格取整误差;
    // 再叠加 planning_start_clear_margin_(0.05) 会把真实墙多挖掉 5cm, 反而给
    // "起步就贴着墙穿过去"开了口子。margin(带 0.05) 只留给下面的圆模型兜底。
    const double poly_margin = 0.5 * cm.resolution;
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

    if (clear_fp.size() >= 3) {
      double min_x = std::numeric_limits<double>::max();
      double max_x = std::numeric_limits<double>::lowest();
      double min_y = std::numeric_limits<double>::max();
      double max_y = std::numeric_limits<double>::lowest();
      for (const auto & v : clear_fp) {
        const double wx = pose.x + c * v.first - s * v.second;
        const double wy = pose.y + s * v.first + c * v.second;
        min_x = std::min(min_x, wx);
        max_x = std::max(max_x, wx);
        min_y = std::min(min_y, wy);
        max_y = std::max(max_y, wy);
      }
      const int x0 = static_cast<int>(std::floor(
        (min_x - poly_margin - cm.origin_x) / res));
      const int x1 = static_cast<int>(std::ceil(
        (max_x + poly_margin - cm.origin_x) / res));
      const int y0 = static_cast<int>(std::floor(
        (min_y - poly_margin - cm.origin_y) / res));
      const int y1 = static_cast<int>(std::ceil(
        (max_y + poly_margin - cm.origin_y) / res));
      for (int iy = y0; iy <= y1; ++iy) {
        for (int ix = x0; ix <= x1; ++ix) {
          const double wx = cm.origin_x + (ix + 0.5) * res;
          const double wy = cm.origin_y + (iy + 0.5) * res;
          const double dx = wx - pose.x;
          const double dy = wy - pose.y;
          const double lx = c * dx + s * dy;
          const double ly = -s * dx + c * dy;
          if (pointInPolygon(lx, ly, clear_fp) ||
              pointPolygonDistance(lx, ly, clear_fp) <= poly_margin) {
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
    uint64_t gen = 0;      // 生成它的 recovery_plan_gen_, 回来时要校验
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
    recovery_start_pose_ = toCorePose(current_pose_);
    planning_active_ = false;
    ++plan_generation_;
    if (async_planner_) { async_planner_->cancel(); }
    is_navigating_ = true;
    m_need_initial_rotate_ = false;

    // Prefer a real reverse escape path over a fixed primitive. A short
    // reverse-only Hybrid A* plan can back out of a dead end/start collision
    // while preserving the normal planner's forward policy.
    if (!(recovery_planner_ && !planner_costmap_.data.empty() && recovery_controller_)) {
      // 恢复器不可用: 这一轮没有真正开始, 把刚消耗的预算退回去
      --recovery_attempts_;
      return false;
    }
    // 先做 nav2 式系统级恢复: 清代价地图 -> 原地转 -> 等待,
    // 然后才用反向 Hybrid A* 找逃逸路径 (原来的唯一手段).
    recovery_spin_angle_ = recovery_spin_angle_deg_ * M_PI / 180.0;
    recovery_stage_ = RecoveryStage::CLEAR;
    recovery_phase_since_ = ros::Time::now();
    ROS_WARN("[recovery] %d/%d: clear costmaps -> spin %.0fdeg -> wait %.1fs -> reverse escape",
             recovery_attempts_, recovery_max_attempts_,
             recovery_spin_angle_ * 180.0 / M_PI, recovery_wait_sec_);
    return true;
  }

  void clearRecoveryCostmaps()
  {
    // 清掉"动态层残留": 体素、跟踪目标、scan 缓存. 规划视图本身是 static+keepout,
    // 重建一次即可 (可能上一次因地图更新没刷到).
    tracker_.clear();
    voxel_layer_.clear();
    laser_points_.clear();
    last_global_costmap_update_ = ros::Time(0);
    updateCostmap();
    publishLocalCostmap(ros::TimerEvent());
    ROS_WARN("[recovery] costmaps cleared (voxel/tracked/scan) + planner view rebuilt");
  }

  // 倒车前用实时点云校验倒车走廊 (nav2 BackUp 的 simulate_ahead_time 思路).
  bool rearClearForBackoff(double dist)
  {
    if (laser_points_.empty()) { return true; }
    const double yaw = yawFromQuat(current_pose_.pose.orientation);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double x_far = -(dist + 0.10);
    const double x_near = -0.28;
    int hits = 0;
    for (const auto & p : laser_points_) {
      const double dx = p.x - current_pose_.pose.position.x;
      const double dy = p.y - current_pose_.pose.position.y;
      const double bx = cy * dx + sy * dy;
      const double by = -sy * dx + cy * dy;
      if (bx <= x_near && bx >= x_far && std::fabs(by) <= 0.40) {
        if (++hits > 3) { return false; }
      }
    }
    return true;
  }

  bool launchReverseEscape()
  {
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

      if (!rearClearForBackoff(back)) {
        ROS_WARN("[recovery] 倒车走廊被实时点云挡住 (%.2fm), 放弃反向逃逸", back);
        return false;
      }
      const core::Costmap recovery_costmap = buildRecoveryCostmap();
      std::vector<std::pair<double, double>> footprint_samples;
      if (!configured_footprint_.empty()) {
        footprint_samples = core::sampleFootprint(
          configured_footprint_, std::max(0.02, footprint_sample_spacing_));
      }
      const double planning_budget = recovery_timeout_ > 0.0
        ? std::min(6.0, recovery_timeout_ * 0.8) : 6.0;
      // 旧规划一律作废, 不复用: 它可能是上一轮恢复、甚至切图/重定位前的位姿算出来的。
      // (以前这里"复用在飞 future", 于是 out.gen 校验对这条路径完全失效。)
      discardRecoveryPlan();
      // 恢复规划放后台线程: 以前在 10Hz 控制定时器里同步跑, 单次最多阻塞
      // 6s, 期间不发 cmd_vel, 底盘继续执行最后一条速度指令 (危险).
      ++recovery_plan_gen_;
      const uint64_t plan_gen = recovery_plan_gen_;
      recovery_plan_future_ = std::async(std::launch::async,
        [this, pose, recovery_costmap, targets, footprint_samples, planning_budget, plan_gen]() {
          RecoveryPlanOutcome out = planReverseEscape(pose, recovery_costmap, targets,
                                                      footprint_samples, planning_budget);
          out.gen = plan_gen;
          return out;
        });
      recovery_stage_ = RecoveryStage::PLANNING;
      ROS_WARN("[rosiwit_navigation_ros1] recovery planning started in background "
               "(%zu candidates)", targets.size());
      return true;
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
    // 恢复成功: 让任务状态机重新计时并解除可能已锁存的 FAILED_STUCK
    // (update() 对非 RUNNING 是直接返回, 不清就会永久"判卡死 -> 恢复"循环)。
    const core::Pose2D now_pose = toCorePose(current_pose_);
    action_task_.noteMotion(now_pose, ros::Time::now().toSec());
    // 预算只在"真的换了个位置"时清零: 逃逸本身是 0.4 m, 所以 >=0.5 m 才算进展。
    // 否则原地打转 (恢复->判卡死->恢复) 会无限循环而不是如实报失败。
    const double moved = std::hypot(now_pose.x - recovery_start_pose_.x,
                                    now_pose.y - recovery_start_pose_.y);
    // 逃逸动作本身是 recovery_back_distance_ (仿真实测 0.4m): 只有明显超过它才算
    // "真的换了位置"。阈值跟着配置走, 不再写死 0.5 (仿真 0.4 时永远清不了零)。
    const double progress_needed = std::max(0.30, 1.25 * recovery_back_distance_);
    if (!recovery_started_.isZero() && moved >= progress_needed) {
      recovery_attempts_ = 0;
    }
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
    // 恢复期间的倒车/原地转也是"在动": 明确告诉任务状态机, 否则卡死看门狗
    // 会把恢复耗时算成"无进展", 恢复一结束就判 FAILED_STUCK。
    if (task_active_ && !last_odom_stamp_.isZero()) {
      action_task_.noteMotion(toCorePose(current_pose_), now.toSec());
    }
    // 总超时必须在 stage 分派**之前**: 各 stage 分支都是自己 return, 放在后面只覆盖
    // 最后一个 stage (以前 PLANNING 等后台 future 时没有任何截止时间, 一旦规划线程
    // 卡住就永远停在这一步; noteMotion 又让任务看门狗不响)。
    if (recovery_timeout_ > 0.0 &&
        (now - recovery_started_).toSec() > recovery_timeout_) {
      ROS_WARN("[rosiwit_navigation_ros1] recovery timeout after %.1fs (stage=%d)",
               (now - recovery_started_).toSec(), static_cast<int>(recovery_stage_));
      failRecovery();
      return;
    }
    if (recovery_stage_ == RecoveryStage::CLEAR) {
      if (recovery_clear_costmaps_) { clearRecoveryCostmaps(); }
      recovery_spin_target_ = normalizeAngle(
        toCorePose(current_pose_).theta + recovery_spin_angle_);
      recovery_phase_since_ = now;
      recovery_stage_ = RecoveryStage::SPIN;
      publishCommand(geometry_msgs::Twist());
      return;
    }
    if (recovery_stage_ == RecoveryStage::SPIN) {
      const double err = normalizeAngle(
        recovery_spin_target_ - toCorePose(current_pose_).theta);
      const bool done = std::fabs(err) < 0.08 ||
        (now - recovery_phase_since_).toSec() > recovery_spin_timeout_;
      if (done) {
        recovery_stage_ = RecoveryStage::WAIT;
        recovery_phase_since_ = now;
        publishCommand(geometry_msgs::Twist());
        ROS_WARN("[recovery] spin done (residual %.2f rad)", err);
        return;
      }
      geometry_msgs::Twist cmd;
      // 安全层在出口处统一做旋转区判定 (车体扫掠面), 这里只管转
      cmd.angular.z = std::max(-max_velocity_theta_,
                               std::min(max_velocity_theta_, 1.2 * err));
      publishCommand(cmd);
      // 旋转被安全层按住 (窄通道里扫掠圆 0.66m 撞墙): 别等满 recovery_spin_timeout_,
      // 直接进入等待->反向逃逸, 否则每次恢复白烧 6s。
      if (last_safety_scale_ <= 0.01) {
        if (recovery_rot_blocked_since_.isZero()) {
          recovery_rot_blocked_since_ = now;
        } else if ((now - recovery_rot_blocked_since_).toSec() > 0.4) {
          recovery_rot_blocked_since_ = ros::Time();
          recovery_stage_ = RecoveryStage::WAIT;
          recovery_phase_since_ = now;
          ROS_WARN("[recovery] spin blocked by safety -> skip to wait");
        }
      } else {
        recovery_rot_blocked_since_ = ros::Time();
      }
      return;
    }
    if (recovery_stage_ == RecoveryStage::WAIT) {
      publishCommand(geometry_msgs::Twist());
      if ((now - recovery_phase_since_).toSec() >= recovery_wait_sec_) {
        recovery_stage_ = RecoveryStage::NONE;
        ROS_WARN("[recovery] wait done -> plan reverse escape");
        if (!launchReverseEscape()) { failRecovery(); }
      }
      return;
    }
    if (recovery_stage_ == RecoveryStage::PLANNING) {
      if (!recovery_plan_future_.valid()) { failRecovery(); return; }
      if (recovery_plan_future_.wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready) {
        // 规划中: 保持停车 (不是保持上一条速度), 控制环不被阻塞
        publishCommand(geometry_msgs::Twist());
        return;
      }
      RecoveryPlanOutcome out;
      try { out = recovery_plan_future_.get(); }
      catch (const std::exception & e) {
        ROS_ERROR("[rosiwit_navigation_ros1] recovery planning exception: %s", e.what());
      }
      if (out.gen != recovery_plan_gen_) {
        // 上一轮恢复的规划结果: 位姿/代价地图都已变, 不能拿它当本轮路径。
        ROS_WARN("[recovery] discard stale reverse-escape plan (gen %lu != %lu)",
                 static_cast<unsigned long>(out.gen),
                 static_cast<unsigned long>(recovery_plan_gen_));
        recovery_stage_ = RecoveryStage::NONE;
        if (!launchReverseEscape()) { failRecovery(); }
        return;
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
      publishCommand(toRosTwist(core_cmd));
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
    // SlamInfo.slam_state: 0=MAPPING / 1=LOCALIZATION / 2=ERROR。
    // 用来区分"没有定位(建图)"与"定位中/定位退化": 只凭 health/status 无法区分
    // (建图模式下 LocalizationState 的 health 恒为 INIT)，以前按状态停车会永久停住。
    last_slam_pipeline_mode_ = msg->slam_state;
  }

  void localizationStateCallback(
    const xzrobot_ros_dependency::LocalizationState::ConstPtr & msg)
  {
    using LocState = xzrobot_ros_dependency::LocalizationState;
    // 位姿跳变检测 (契约: relocalize_count 变化 / map_id 变化 => 位姿坐标系跳了).
    // 必须在更新缓存的同一拍完成: 跳变后旧计划/旧跟踪状态立即作废。
    const bool map_changed = has_reloc_baseline_ && !msg->map_id.empty() &&
      msg->map_id != last_map_id_;
    const bool jumped = has_reloc_baseline_ &&
      msg->relocalize_count != last_relocalize_count_;
    if (map_changed || jumped) {
      // 只登记: 本节点用 AsyncSpinner(2), 回调和控制环可能并发。
      // 计划/控制器状态只在控制环里改, 避免两条线程同时动 current_core_path_。
      pending_jump_seq_.store(msg->relocalize_count);
      pending_jump_reason_.store(map_changed ? 1 : 0);
      pose_jump_pending_.store(true);
    }
    last_relocalize_count_ = msg->relocalize_count;
    has_reloc_baseline_ = true;

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

  /// 位姿跳变 (切图/重定位/手动初始位姿): 旧计划在新坐标系下已经没有意义。
  /// 只作废"计划与跟踪状态", 不清目标 —— 目标是地图坐标, 跳变后仍然有效。
  void onPoseJump(uint32_t seq, const std::string & why)
  {
    ROS_WARN("[nav] pose jump (%s, seq=%u): invalidate plan, stop and replan",
             why.c_str(), seq);
    // 1) 作废在途异步规划: 否则旧坐标系下的结果会被 applyPlanningResult 装回来
    if (planning_active_ || async_plan_future_.valid()) {
      planning_active_ = false;
      ++plan_generation_;
      if (async_planner_) { async_planner_->cancel(); }
    }
    // 2) 清计划/跟踪状态, 控制器从当前位姿重新起算 (避免旧 crosstrack 积分)
    current_core_path_.points.clear();
    current_path_.poses.clear();
    if (controller_strategy_) { controller_strategy_->reset(); }
    resetRecovery();
    safety_monitor_.reset();
    replan_blocked_count_ = 0;
    // 3) 先停车: 不能让跳变前一拍算出的指令继续推机器人
    geometry_msgs::Twist stop_cmd;
    publishCommand(stop_cmd);
    // 4) 下一拍立刻重规划 (replanningCheck 看 since_plan)
    last_plan_time_ = ros::Time();
    last_pose_jump_ = ros::Time::now();
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
    // 单线程 spinner: 代价地图/规划/存图与看门狗共用一条回调队列, 某次回调超时
    // 时看门狗也一起被饿死, 底盘会继续执行最后一条指令。这里在**恢复的第一拍**
    // 立刻补一条硬停, 把"被拖住的这段时间"造成的风险限制在一拍内。
    {
      const ros::Time t0 = ros::Time::now();
      if (!last_control_tick_.isZero()) {
        const double gap = (t0 - last_control_tick_).toSec();
        const double period = 1.0 / std::max(1.0, controller_frequency_);
        if (gap > std::max(0.5, 3.0 * period) && last_published_nonzero_) {
          ROS_WARN_THROTTLE(1.0,
            "[nav] control tick late by %.2fs (spinner blocked?); force zero", gap);
          publishHardStop();
        }
      }
      last_control_tick_ = t0;
    }
    if (pose_jump_pending_.exchange(false)) {
      onPoseJump(pending_jump_seq_.load(),
                 pending_jump_reason_.load() == 1 ? std::string("map_switched")
                                                 : std::string("relocalized"));
    }
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

    // 注意 (2026-10-28 回退): 曾把 HEALTH_INIT 当成"不可行驶"在这里停车 + WAITING_LOC。
    // 实测同路线 3/3 全部 TIMEOUT:WAITING_LOC —— 因为 `last_slam_status_` 被**两个**
    // 回调写 (/slam/info 的 location_status 与 LocalizationState 的 health, 语义不同),
    // 运行中会出现 location_status=1 而 health=0(TRACKING) 的组合, 门控就把正常行驶
    // 永久停住。要收这个口子必须先统一状态来源, 不能只看一个数字。
    // 现状: INIT 仍按"可继续行驶"处理; 真正的丢失由 slamHealthLost() (status==2) 兜底。

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
      const bool safety_stopped = safety_enabled_ && last_safety_scale_ <= 0.01 &&
                                  !last_cmd_input_zero_;
      if (safety_stopped) {
        if (safety_stop_since_.isZero()) {
          safety_stop_since_ = now;
        } else if ((now - safety_stop_since_).toSec() > safety_stop_timeout_) {
          safety_stop_since_ = ros::Time();
          // 定位质量差的时候不要进恢复: 安全层是拿"当前位姿"把 /scan 换到 map 系的,
          // 位姿偏 20~30cm 时真墙会落到地图的空白处 -> 幻影障碍 -> 停 -> 恢复也改不了
          // 位姿 -> 预算烧完直接 FAILED_SAFETY_STOP (实测 matching=0.635 那次就是这样,
          // 而 matching>=0.78 的两次都是 3/3)。这里改为"保持停车等定位恢复"。
          // matching==0 表示根本没有定位 (纯建图), 不能算退化 —— 否则建图模式会永远停住。
          // 只在 LOCALIZATION 模式判: MAPPING 下 health 恒 INIT / matching 恒 0,
          // 按状态停车会让建图模式永久停住 (本轮回归前踩过一次)。
          const bool loc_fresh = !last_slam_info_stamp_.isZero() &&
            (now - last_slam_info_stamp_).toSec() <= slam_info_timeout_;
          const bool loc_degraded = loc_fresh && last_slam_pipeline_mode_ == 1 &&
            (last_slam_status_ != 0 ||
             (last_slam_matching_ > 0.0 && last_slam_matching_ < slam_degrade_matching_));
          if (loc_degraded) {
            ROS_WARN_THROTTLE(2.0,
              "[nav] safety stop while localization degraded (matching=%.2f < %.2f): "
              "hold and wait for localization instead of recovery",
              last_slam_matching_, slam_degrade_matching_);
            publishCommand(geometry_msgs::Twist());
            setGoalStatus("WAITING_LOC");
            return;
          }
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
        // 打印末端航向与目标航向, 以后"到点朝向不对"能直接从日志看出来。
        const double cur_yaw = yawFromQuat(current_pose_.pose.orientation);
        const double goal_yaw = yawFromQuat(goal_pose_.pose.orientation);
        ROS_INFO("[rosiwit_navigation_ros1] goal reached: pos=(%.2f,%.2f) yaw=%.1fdeg "
                 "goal_yaw=%.1fdeg err=%.1fdeg",
                 current_pose_.pose.position.x, current_pose_.pose.position.y,
                 cur_yaw * 57.2957795, goal_yaw * 57.2957795,
                 normalizeAngle(goal_yaw - cur_yaw) * 57.2957795);
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
        publishCommand(rot_cmd);
        recordActionTraveled();
        if (task_active_) { publishActionOutputs(); }
        return;
      }
    }

    // 末端完全交给 MPPI: 不再用 <0.6m 的独立 P 控制器切换
    // (会造成前进/停/转反复, 且与 MPPI 的速度/加速度约束不一致)。
    auto cmd = computeVelocityCommand();
    // 旋转段: 先原地转到位, 再交回 MPPI 跟踪前进段 (控制器自带原地旋转能力时
    // 由控制器自己执行)
    if (!(controller_strategy_ && controller_strategy_->supportsInPlaceRotation())) {
      geometry_msgs::Twist rot_cmd;
      if (executeRotationSegment(toCorePose(current_pose_), rot_cmd)) {
        rotation_active_ = true;
        publishCommand(rot_cmd);
        recordActionTraveled();
        if (task_active_) { publishActionOutputs(); }
        return;
      }
      // 旋转刚结束: MPPI 的进度索引还停在旋转点(旋转时位置不变, 索引不前进),
      // 重新播种整条路径让参考点推进到旋转段之后, 否则会停在半转姿态.
      if (rotation_active_) {
        rotation_active_ = false;
        if (!current_core_path_.points.empty()) {
          controller_strategy_->setPath(current_core_path_);
        }
        ROS_INFO("[rosiwit_navigation_ros1] rotation done -> controller re-seeded (%zu pts)",
                 current_core_path_.points.size());
      }
    }
    const double health_scale = slamHealthScale();
    if (health_scale < 1.0) {
      cmd.linear.x *= health_scale;
      cmd.angular.z *= health_scale;
    }
    publishCommand(cmd);
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
      recordPoseSample(ros::Time::now());
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
    // 位姿还没建立 (启动/切图瞬间) 时不能用默认 (0,0) 去变换扫描点,
    // 否则会把真实障碍映射到错误位置并锁存 STOP。控制环在无位姿时本来就会
    // 走 stopNavigation, 这里直接放行由上层兜底。
    if (current_pose_.header.frame_id != global_frame_) return cmd;
    // 零速指令没有运动风险, 不走 STOP/SLOWDOWN 判定: 否则停车/规划等待时
    // 会被近处点云锁存 STOP, 诊断长期显示 safety=STOP, 下一次起步还要等释放。
    if (std::fabs(cmd.linear.x) < 1e-4 && std::fabs(cmd.angular.z) < 1e-4) {
      last_safety_state_ = "PASS";
      last_safety_scale_ = 1.0;
      last_safety_hard_stop_ = false;
      return cmd;
    }
    if (safety_fail_safe_on_timeout_ && !last_scan_stamp_.isZero() &&
        (ros::Time::now() - last_scan_stamp_).toSec() > safety_source_timeout_) {
      ROS_WARN_THROTTLE(1.0, "[safety] STALE scan age=%.2fs (timeout %.2fs)",
                        (ros::Time::now() - last_scan_stamp_).toSec(), safety_source_timeout_);
      last_safety_state_ = "STALE";
      last_safety_scale_ = 0.0;
      return geometry_msgs::Twist();
    }

    // 点云是 map 系、比控制拍早 50~150ms: 必须用**扫描时刻**的位姿换到 base 系。
    // 用最新位姿会在转弯/旋转时把整个安全区相对点云转偏 (0.8rad/s × 100ms ≈ 4.6°,
    // 在 0.95m 处 ≈ 7.6cm, 而前向停止区的横向余量只有 1.5cm)。
    double px = current_pose_.pose.position.x;
    double py = current_pose_.pose.position.y;
    double pyaw = yawFromQuat(current_pose_.pose.orientation);
    const ros::Time scan_t =
      !last_scan_msg_stamp_.isZero() ? last_scan_msg_stamp_ : last_scan_stamp_;
    if (!scan_t.isZero()) {
      double sx = 0.0, sy2 = 0.0, syaw = 0.0;
      if (poseAt(scan_t, sx, sy2, syaw)) { px = sx; py = sy2; pyaw = syaw; }
    }
    const double yaw = pyaw;
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    std::vector<std::pair<double, double>> points;
    points.reserve(laser_points_.size());
    for (const auto & p : laser_points_) {
      const double dx = p.x - px;
      const double dy = p.y - py;
      const double bx = cy * dx + sy * dy;
      const double by = -sy * dx + cy * dy;
      if (bx * bx + by * by < safety_self_filter_radius_ * safety_self_filter_radius_) continue;
      points.emplace_back(bx, by);
    }

    const core::SafetyResult result = safety_monitor_.filter(toCoreVelocity(cmd), points);
    if (result.scale <= 0.0 && result.state == "STOP") {
      // 诊断: 打印落入 STOP 区的前几个点 (base 系), 确认是自身回波还是真障碍
      const std::vector<std::pair<double, double>> * zone = nullptr;
      const auto & cfg = safety_monitor_.config();
      if (std::fabs(cmd.linear.x) < 0.05 && !cfg.stop_polygon_rotation.empty()) {
        zone = &cfg.stop_polygon_rotation;
      } else if (cmd.linear.x < 0.0 && !cfg.stop_polygon_backward.empty()) {
        zone = &cfg.stop_polygon_backward;
      } else {
        zone = &cfg.stop_polygon;
      }
      std::ostringstream os;
      int n = 0;
      for (const auto & pt : points) {
        if (core::pointInPolygon(pt.first, pt.second, *zone)) {
          // base 系 + 反算回 map 系, 判断是"刚性挂在车上"还是"换算偏移"
          const double wx = current_pose_.pose.position.x + (cy * pt.first - sy * pt.second);
          const double wy = current_pose_.pose.position.y + (sy * pt.first + cy * pt.second);
          os << " b(" << pt.first << "," << pt.second << ") w(" << wx << "," << wy << ")";
          if (++n >= 2) { break; }
        }
      }
      ROS_WARN_THROTTLE(1.0, "[safety] STOP hits%s pose=(%.2f,%.2f,%.1fdeg) npts=%zu",
                        os.str().c_str(), current_pose_.pose.position.x,
                        current_pose_.pose.position.y, yaw * 180.0 / M_PI, points.size());
    }
    last_safety_hard_stop_ = (result.scale <= 0.0);
    if (result.scale <= 0.0) {
      ROS_WARN_THROTTLE(1.0, "[safety] %s: pts=%zu cmd(v=%.2f w=%.2f) -> zero",
                        result.state.c_str(), points.size(),
                        cmd.linear.x, cmd.angular.z);
    }
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
    // 安全层滞回状态 (stop_active_/slow_active_) 不能跨任务残留:
    // 上一次停车/切图时的 STOP 会让新目标前几拍继续零速, 表现为"起步一顿"。
    safety_monitor_.reset();
    is_navigating_ = false;
    is_paused_ = false;
    current_core_path_.points.clear();
    last_motion_stamp_ = ros::Time();
    safety_stop_since_ = ros::Time();
    // 取消/结束导航时必须同时作废在途的异步规划, 否则 pollPlanning 会把这个
    // 结果 applyPlanningResult() 回来, 让已经停止的任务重新跑起来.
    // ⚠ 2026-10-28: 只作废 generation 不够 —— pollPlanning 在"generation 不匹配"分支里
    // 看到 pending_goal_ 非空还会 launchPlan() 重启它, 于是 /nav_stop 之后车照样开到
    // 被取消的目标。必须同时清掉"待执行目标"。
    pending_goal_ = geometry_msgs::PoseStamped();
    pending_has_start_override_ = false;
    if (planning_active_ || async_plan_future_.valid()) {
      planning_active_ = false;
      ++plan_generation_;
      if (async_planner_) { async_planner_->cancel(); }
    }
    publishHardStop();
  }

  /// 取消/结束时的停车: **不能**走速度整形器 —— 整形器对"目标 0"也做加速度限幅
  /// (0.8m/s 时单拍只降到 0.77), 而控制环接着就 return, 0.6s 内一条指令都不发,
  /// 合计约 1.3s / 0.75m 才真停。这里直接把 0 发给底盘。
  void publishHardStop()
  {
    velocity_shaper_.reset();
    geometry_msgs::Twist z;
    cmd_vel_pub_.publish(z);
    if (mb_) { mb_->publishCmdVel(z); }
    last_published_cmd_ = z;
    last_published_nonzero_ = false;
    last_cmd_stamp_ = ros::Time::now();
    last_safety_scale_ = 1.0;
    last_safety_hard_stop_ = false;
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
      // ⚠ PathPoint 默认 velocity_x=0, 而 rpp 把 |velocity_x|<0.1 解释成
      // "原地旋转段" (dirAt()==0 -> 只返回角速度)。ROS Path 里没有方向信息,
      // 从外部灌进来的路径必须显式标成前进, 否则 linear.x 恒为 0: 车只转不走。
      pt.velocity_x = 1.0;
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
  double planner_heuristic_weight_ = 2.0;
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
  double safety_self_filter_radius_ = 0.10;
      // ---- 安全层 (nav2_collision_monitor 机制) ----
  std::vector<double> safety_stop_poly_;
  std::vector<double> safety_slow_poly_;
  std::vector<double> safety_back_poly_;
  std::vector<double> safety_rot_poly_;
  std::vector<double> safety_footprint_poly_;
  std::vector<std::pair<double, double>> body_polygon_;   // 车身排除区 (只排除轮廓内)
  double safety_body_margin_ = 0.05;
  double safety_rotation_radius_margin_ = 0.10;
  int safety_stop_max_points_ = 4;   // nav2 collision_monitor 默认
  int safety_slowdown_max_points_ = 6;
  int safety_stop_trigger_frames_ = 2;
  int safety_stop_release_frames_ = 3;
  int safety_slowdown_trigger_frames_ = 1;
  int safety_slowdown_release_frames_ = 2;
  // 动态安全区 (车体+制动距离)
  bool safety_dynamic_geometry_ = true;
  double safety_brake_decel_ = 0.8;
  double safety_reaction_time_ = 0.2;
  double safety_stop_margin_ = 0.05;
  double safety_side_margin_ = 0.03;
  double safety_turn_margin_ = 0.05;
  double safety_slowdown_extra_ = 0.30;
  double safety_scale_slew_rate_ = 1.0;
  double safety_scale_dt_ = 0.1;

  // ---- 指令整形 / 死手 / 遥控仲裁 ----
  core::VelocityShaper velocity_shaper_;
  ros::Time last_cmd_stamp_;
  ros::Time last_control_tick_;   // 上一拍控制开始时刻 (延迟保护)
  uint64_t map_seq_ = 0;          // /map 版本号 (静态层整图拷贝去重)
  uint64_t planner_static_seq_ = 0;
  uint64_t local_static_seq_ = 0;
  double velocity_timeout_ = 0.6;
  double deadband_velocity_v_ = 0.0;
  double deadband_velocity_w_ = 0.0;
  std::string teleop_topic_;
  double teleop_timeout_ = 0.5;
  geometry_msgs::Twist teleop_cmd_;
  ros::Time teleop_stamp_;
  ros::Subscriber teleop_sub_;
  bool strict_config_ = false;
  geometry_msgs::Twist last_published_cmd_;
  bool last_published_nonzero_ = false;
  ros::Timer command_watchdog_timer_;

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
  double replan_max_interval_ = 30.0;
  double replan_min_interval_ = 1.5;
  bool replan_on_blocked_ = true;
  double replan_lookahead_ = 3.0;
  double replan_skip_distance_ = 0.5;
  int replan_blocked_frames_ = 2;
  int replan_blocked_count_ = 0;
  int replan_count_ = 0;
  int replan_fail_count_ = 0;
  double plan_start_time_ = 0.0;
  double last_plan_ms_ = -1.0;
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
  // 规划在 std::async 线程里跑; 这两个必须 shared_ptr + 按值捕获,
  // 否则换 profile 时 unique_ptr 析构会把正在执行的 planner 拆掉 (UAF)。
  std::shared_ptr<core::IPlannerStrategy> async_planner_;
  // 窄通道回落: hybrid A* 超时/失败时用栅格 A* (见 launchPlan)
  std::shared_ptr<core::IPlannerStrategy> fallback_planner_;
  std::future<core::Result<core::Path>> async_plan_future_;
  bool planning_active_ = false;
  uint64_t plan_generation_ = 0;
  uint64_t active_plan_generation_ = 0;

  // 恢复规划在后台线程执行; 控制环只 poll, 不再被 plan() 阻塞
  std::future<RecoveryPlanOutcome> recovery_plan_future_;
  uint64_t recovery_plan_gen_ = 0;
  // 被作废但还没结束的逃逸规划 (async future 析构会阻塞, 只能顺带回收)
  std::deque<std::future<RecoveryPlanOutcome>> recovery_plan_graveyard_;

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
  uint32_t last_relocalize_count_ = 0;   // 契约字段, 变化 = 位姿跳变
  std::atomic<bool> pose_jump_pending_{false};   // 回调登记, 控制环消费
  std::atomic<uint32_t> pending_jump_seq_{0};
  std::atomic<int> pending_jump_reason_{0};      // 0=重定位 1=切图
  bool has_reloc_baseline_ = false;      // 首帧不算跳变
  ros::Time last_pose_jump_;             // 最近一次跳变时刻 (诊断用)
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
  int last_slam_pipeline_mode_ = -1;   // SlamInfo.slam_state: 0=MAPPING 1=LOCALIZATION 2=ERROR
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
  // ---- 位姿: 两个来源、两个坐标系, 分开存 ----
  // 坐标系链 (实测):
  //   map --(SLAM 发布, 重定位/建图原点决定)--> odom --(底盘轮速)--> base_footprint
  //   base_footprint --(静态)--> base_link --(静态, rpy≈43°/-0.3°/90°)--> lidar / laser
  // 本机实测 map->odom 的 yaw ≈ -90° (地图 origin 锚在建图起始朝向), 所以
  // 把 odom 系的数当成 map 用, 等价于整体差 90°。
  // current_pose_: **只**由 updateCurrentPose() 从 TF(global_frame_<-robot_frame_) 写,
  //   永远是 global_frame_(map); 规划起点/代价地图/RViz 多边形都用它。
  // odom_pose_: /odom 原样 (frame_id=odom), 只用于里程/打滑/诊断, 不许进 map 系计算。
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::PoseStamped odom_pose_;
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
  // CLEAR/SPIN/WAIT 是 nav2 行为树里的系统级恢复 (清图/原地转/等待),
  // 之后才进我们自己的反向逃逸规划 (PLANNING -> FOLLOW).
  enum class RecoveryStage { NONE, CLEAR, SPIN, WAIT, PLANNING, FOLLOW };
  RecoveryStage recovery_stage_ = RecoveryStage::NONE;
  ros::Time recovery_phase_since_;
  double recovery_spin_target_ = 0.0;
  double recovery_spin_angle_ = 1.05;   // 60deg
  bool recovery_clear_costmaps_ = true;
  double recovery_spin_angle_deg_ = 60.0;
  double recovery_spin_timeout_ = 6.0;
  double recovery_wait_sec_ = 2.0;
  // 本次规划是否用了回退规划器 (A*): 决定要不要做节点层平滑
  std::atomic<bool> last_plan_used_fallback_{false};
  // 平滑器用的代价图 (膨胀后, 99=膨胀带): 只做平滑, 不改规划器的判据
  std::shared_ptr<core::CostmapGrid> planner_smooth_grid_;
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
  double planner_turn_step_deg_ = 30.0;   // hybrid 原地转向步长 (deg)
  bool allow_unknown_ = false;            // unknown(-1) 是否按代价放行 (见 updateCostmap)
  int unknown_cost_ = 99;                 // allow_unknown 时给 -1 的代价
  double path_footprint_check_skip_ = 0.5;  // 路径轮廓检查跳过起点附近该半径内 (m)
  bool two_point_final_rotate_ = false;   // 两点导航到点后是否原地对齐目标航向
  bool path_final_rotate_ = false;        // 路径跟踪走完后是否原地对齐终点航向
  bool final_yaw_from_next_point_ = false;  // 单点导航: 用 p0->p1 方向当末端朝向
  bool single_point_use_last_pose_ = false; // 单点导航给了多点时用最后一点当目标
  bool planning_is_replan_ = false;         // 本次在途规划是不是 1Hz 重规划(非新目标)
  core::Costmap last_planning_costmap_;
  bool laser_filter_enabled_ = true;
  double laser_filter_radius_ = 0.15;
  int laser_filter_min_neighbors_ = 1;
  int laser_filter_window_ = 2;
  bool action_interface_active_ = false;
  ros::Time recovery_started_;
  core::Pose2D recovery_start_pose_;   // 本轮恢复开始时的位姿 (判断是否真有进展)


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
  // 全图可达性 BFS 缓存 (见 coverage 执行处的注释)
  ros::Time coverage_reach_check_time_;
  core::Pose2D coverage_reach_pose_;
  bool coverage_reach_result_ = true;
  // 当前活动覆盖路线: 有它就不再周期重建 (每 10s 重建会让机器人反复重跑同一段)
  std::vector<core::Pose2D> coverage_route_;
  // 覆盖路线 (core::Path, 带 velocity_x: 0=原地转 1=前进), 供旋转段执行器读取
  core::Path coverage_path_core_;
  int cov_rot_dir_ = 0;   // 覆盖旋转方向锁存 (避免 ±π 附近左右摆)
  // 旋转执行器锁存: 触发后独占, 直到转到位 (否则和纯跟踪互相抢, 净角速度≈0)
  bool cov_rot_engaged_ = false;
  size_t cov_rot_idx_ = 0;
  double cov_rot_target_ = 0.0;
  size_t cov_rot_done_ = 0;   // 已处理到的旋转点下标 (按路径顺序单调推进)
  // 旋转被挡 -> 后退脱困
  ros::Time cov_rot_blocked_since_;
  ros::Time cov_backoff_until_;
  ros::Time cov_rot_hold_until_;
  double cov_rot_hold_time_ = 0.25;
  double cov_rot_blocked_timeout_ = 1.2;
  double cov_backoff_speed_ = 0.12;
  double cov_backoff_time_ = 0.7;
  int cov_backoff_max_ = 2;
  int cov_backoff_count_ = 0;
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
  double coverage_turn_radius_ = 0.0;
  double coverage_headland_width_ = 0.0;
  double coverage_doorway_width_ = 0.0;
  std::string coverage_controller_name_ = "pure_pursuit";
  std::string coverage_report_path_;
  movebase::Goal action_goal_;
  movebase::Decoded action_cmd_;
  double action_speed_ = 0.0;
  std::vector<geometry_msgs::PoseStamped> action_traveled_;
  bool pending_has_start_override_ = false;
  bool last_cmd_input_zero_ = false;      // 上一条"输入"命令是否为零
  bool last_safety_hard_stop_ = false;    // 上一条命令是否被安全层硬停
  ros::Time recovery_rot_blocked_since_;  // 恢复旋转被安全层按住的起始时刻
  core::Pose2D pending_start_override_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "rosiwit_navigation_ros1_node");
  // 崩溃时打印带符号的栈 (roswit 现场排查: exit code -11 不能只有一行 died)
  rosiwit_navigation_ros1::installCrashHandler();
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  RosiwitNavigationRos1Node node(nh, pnh);
  // **默认单线程**: 本节点的全部共享状态 (planner_costmap_ / current_map_ /
  // current_pose_ / 路径 / 恢复状态机) 都没有锁, 2 条 spinner 线程会让订阅回调与
  // 定时器并发读写同一份数据 -> 数据竞争 (实测有 exit code -11 的崩溃)。
  // 异步重活已经不在 spinner 上 (规划走 async_planner_ / recovery_plan_future_),
  // 单线程不会把控制环饿死; 需要多线程时必须先给共享状态加锁再调大这个参数。
  int threads = 1;
  pnh.param<int>("spinner_threads", threads, 1);
  threads = std::max(1, std::min(8, threads));
  ros::AsyncSpinner spinner(threads);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
