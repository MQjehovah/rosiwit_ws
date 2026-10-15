// ============================================================================
// rosiwit_navigation_ros1_node.cpp
// rosiwit_navigation 的 ROS1 接口层: 仅做 ROS1 <-> 算法核心(NavigationFactory)
// 的桥接. 算法核心完全 ROS 无关, 与 ROS2 节点 (ros_interface/navigation_node)
// 共享同一份代码 (源码由 ../../../src/rosiwit_navigation 内联构建).
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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rosiwit_navigation/nav_core/navigation_factory.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"
#include "path_smoother.hpp"

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

    pnh_.param<std::string>("global_frame", global_frame_, "odom");
    pnh_.param<std::string>("robot_frame", robot_frame_, "base_link");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("planner_name", planner_name_, "astar");
    pnh_.param<std::string>("controller_name", controller_name_, "pure_pursuit");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
  }

  // ---- 核心策略 (来自算法核心, ROS 无关) ----
  void setupCore()
  {
    planner_strategy_ = nav_core::NavigationFactory::createPlanner(planner_name_);
    if (planner_strategy_) {
      core::PlannerConfig cfg;
      cfg.name = planner_name_;
      cfg.resolution = 0.05;
      cfg.max_iterations = 50000;
      cfg.timeout = 2.0;
      planner_strategy_->initialize(cfg);
      planner_strategy_->setInflationRadius(inflation_radius_);
      planner_strategy_->setRobotRadius(robot_radius_);
    } else {
      ROS_WARN("[rosiwit_navigation_ros1] planner '%s' not found", planner_name_.c_str());
    }

    controller_strategy_ = nav_core::NavigationFactory::createController(controller_name_);
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
      cfg.kinematics.max_velocity_x = max_velocity_x_;
      cfg.kinematics.max_velocity_theta = max_velocity_theta_;
      controller_strategy_->initialize(cfg);
    } else {
      ROS_WARN("[rosiwit_navigation_ros1] controller '%s' not found", controller_name_.c_str());
    }
  }

  void setupRos()
  {
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1);
    global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);
    global_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/global_costmap", 1, true);
    local_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 1, true);

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &RosiwitNavigationRos1Node::odomCallback, this);
    map_sub_ = nh_.subscribe("/map", 1, &RosiwitNavigationRos1Node::mapCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &RosiwitNavigationRos1Node::scanCallback, this);
    goal_sub_ = nh_.subscribe("/goal_pose", 1, &RosiwitNavigationRos1Node::goalCallback, this);

    control_timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(1.0, controller_frequency_)),
      &RosiwitNavigationRos1Node::controlLoop, this);
    local_costmap_timer_ = nh_.createTimer(
      ros::Duration(0.1), &RosiwitNavigationRos1Node::publishLocalCostmap, this);
  }

  // ---- 回调 ----
  void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
  {
    current_velocity_ = msg->twist.twist;
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
  {
    current_map_ = msg;
    updateCostmap();
    ROS_INFO("[rosiwit_navigation_ros1] map received: %d x %d, res=%.3f",
             static_cast<int>(msg->info.width), static_cast<int>(msg->info.height),
             msg->info.resolution);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr & msg)
  {
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
    }
    updateCostmap();
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
  {
    ROS_INFO("[rosiwit_navigation_ros1] received /goal_pose: (%.2f, %.2f)",
             msg->pose.position.x, msg->pose.position.y);
    goal_pose_ = *msg;
    executeNavigation(goal_pose_);
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
      cm.width = w;
      cm.height = h;
      cm.resolution = res;
      cm.origin_x = ox;
      cm.origin_y = oy;
      cm.data.assign(current_map_->data.size(), 0);
      for (size_t i = 0; i < current_map_->data.size(); ++i) {
        cm.data[i] = (current_map_->data[i] >= 100) ? 255 : 0;
      }
      for (const auto & pt : laser_points_) {
        const int mx = static_cast<int>((pt.x - ox) / res);
        const int my = static_cast<int>((pt.y - oy) / res);
        if (mx >= 0 && mx < static_cast<int>(w) && my >= 0 && my < static_cast<int>(h)) {
          cm.data[my * w + mx] = 255;
        }
      }
      planner_strategy_->setCostmap(cm);

      // 膨胀并发布供 RViz 可视化
      const int inflate_cells = static_cast<int>(std::ceil(inflation_radius_ / res));
      const int safety_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_ / res)));
      std::vector<float> dist(w * h, std::numeric_limits<float>::max());
      std::queue<std::pair<int, int>> q;
      for (unsigned int y = 0; y < h; ++y) {
        for (unsigned int x = 0; x < w; ++x) {
          if (cm.data[y * w + x] >= 254) { dist[y * w + x] = 0.0f; q.push({x, y}); }
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
        if (d <= safety_cells) { costmap_msg.data[i] = 100; }
        else if (d <= inflate_cells) {
          costmap_msg.data[i] = static_cast<int8_t>((1.0f - (d - safety_cells) * inv) * 99);
        } else { costmap_msg.data[i] = 0; }
      }
      global_costmap_pub_.publish(costmap_msg);
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

    auto isObstacleCell = [&](int mx, int my) -> bool {
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
    const int lw = win_half * 2;

    nav_msgs::OccupancyGrid local_costmap;
    local_costmap.header.frame_id = map_frame_;
    local_costmap.header.stamp = ros::Time::now();
    local_costmap.info.resolution = res;
    local_costmap.info.width = lw;
    local_costmap.info.height = lw;
    local_costmap.info.origin.position.x = rx - win_width / 2.0;
    local_costmap.info.origin.position.y = ry - win_width / 2.0;
    local_costmap.info.origin.orientation.w = 1.0;

    std::vector<int8_t> raw(lw * lw, 0);
    for (int dy = -win_half; dy < win_half; ++dy) {
      for (int dx = -win_half; dx < win_half; ++dx) {
        const int mx = cx + dx;
        const int my = cy + dy;
        if (mx < 0 || mx >= static_cast<int>(w) || my < 0 || my >= static_cast<int>(h)) { continue; }
        const int gx = dx + win_half;
        const int gy = dy + win_half;
        raw[gy * lw + gx] = (current_map_->data[my * w + mx] >= 100) ? 100 : 0;
      }
    }

    const int l_inflate = static_cast<int>(std::ceil(inflation_radius_ / res));
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
        if (cd + 1 < ldist[ny * lw + nx]) { ldist[ny * lw + nx] = cd + 1; q.push({nx, ny}); }
      }
    }

    const float linv = 1.0f / std::max(1, l_inflate - l_safety);
    local_costmap.data.resize(lw * lw);
    for (int i = 0; i < lw * lw; ++i) {
      if (raw[i] >= 100) { local_costmap.data[i] = 100; continue; }
      const float d = ldist[i];
      if (d <= l_safety) { local_costmap.data[i] = 100; }
      else if (d <= l_inflate) {
        local_costmap.data[i] = static_cast<int8_t>((1.0f - (d - l_safety) * linv) * 99);
      } else { local_costmap.data[i] = 0; }
    }
    local_costmap_pub_.publish(local_costmap);
  }

  // ---- 导航 ----
  void executeNavigation(const geometry_msgs::PoseStamped & goal)
  {
    if (!updateCurrentPose()) {
      ROS_ERROR("[rosiwit_navigation_ros1] cannot get current pose for planning");
      return;
    }
    if (!planner_strategy_) {
      ROS_ERROR("[rosiwit_navigation_ros1] no planner available");
      return;
    }

    const core::Pose2D start = toCorePose(current_pose_);
    const core::Pose2D goal_core = toCorePose(goal);
    auto result = planner_strategy_->plan(start, goal_core);
    if (!result.is_ok()) {
      ROS_ERROR("[rosiwit_navigation_ros1] planner '%s' failed: %s",
                planner_name_.c_str(), result.error_message().c_str());
      return;
    }

    current_path_ = toRosPath(result.value(), goal.header.frame_id, ros::Time::now());

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
      current_path_ = toRosPath(core_path, goal.header.frame_id, ros::Time::now());
    }

    if (controller_strategy_) {
      controller_strategy_->setPath(toCorePath(current_path_));
    }
    global_path_pub_.publish(current_path_);

    m_need_initial_rotate_ = false;
    if (current_path_.poses.size() > 2) {
      const double current_yaw = yawFromQuat(current_pose_.pose.orientation);
      const double dx = current_path_.poses[1].pose.position.x - current_path_.poses[0].pose.position.x;
      const double dy = current_path_.poses[1].pose.position.y - current_path_.poses[0].pose.position.y;
      const double path_yaw = std::atan2(dy, dx);
      const double yaw_diff = normalizeAngle(path_yaw - current_yaw);
      if (std::abs(yaw_diff) > 0.35) {
        m_need_initial_rotate_ = true;
        m_initial_rotate_target_ = path_yaw;
      }
    }

    is_navigating_ = true;
    is_paused_ = false;
    ROS_INFO("[rosiwit_navigation_ros1] path planned with %zu points",
             current_path_.poses.size());
  }

  void controlLoop(const ros::TimerEvent &)
  {
    if (!is_navigating_ || is_paused_) { return; }
    if (!updateCurrentPose()) { return; }

    if (isGoalReached()) {
      ROS_INFO("[rosiwit_navigation_ros1] goal reached");
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
        cmd_vel_pub_.publish(rot_cmd);
        return;
      }
    }

    geometry_msgs::Twist cmd_vel = computeVelocityCommand();
    cmd_vel_pub_.publish(cmd_vel);
    path_pub_.publish(current_path_);
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

  void stopNavigation()
  {
    is_navigating_ = false;
    is_paused_ = false;
    geometry_msgs::Twist stop_cmd;
    cmd_vel_pub_.publish(stop_cmd);
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

  std::string global_frame_;
  std::string robot_frame_;
  std::string map_frame_;
  std::string planner_name_;
  std::string controller_name_;
  std::string odom_topic_;

  std::unique_ptr<core::IPlannerStrategy> planner_strategy_;
  std::unique_ptr<core::IControllerStrategy> controller_strategy_;

  ros::Subscriber odom_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher path_pub_;
  ros::Publisher global_path_pub_;
  ros::Publisher global_costmap_pub_;
  ros::Publisher local_costmap_pub_;
  ros::Timer control_timer_;
  ros::Timer local_costmap_timer_;

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
