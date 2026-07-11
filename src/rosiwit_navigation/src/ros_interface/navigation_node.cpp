// ============================================================
// Diffbot Navigation - 平滑导航节点实现
// ============================================================

#include "rosiwit_navigation/ros_interface/navigation_node.hpp"

#include <memory>
#include <string>
#include <cmath>
#include <queue>
#include <utility>

#include "path_smoother.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"



namespace rosiwit_navigation
{
namespace navigation
{

using rclcpp_lifecycle::LifecycleNode;
using ros_interface::RosUtils;

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
    planner_cfg.max_iterations = 50000;
    planner_cfg.timeout = 2.0;
    planner_strategy_->initialize(planner_cfg);
    planner_strategy_->setInflationRadius(inflation_radius_);
    RCLCPP_INFO(get_logger(), "Created planner strategy: %s", planner_name_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Planner '%s' not found, falling back to PathPlanner", planner_name_.c_str());
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
    ctrl_cfg.kinematics.max_velocity_x = params_.max_velocity_x;
    ctrl_cfg.kinematics.max_velocity_theta = params_.max_velocity_theta;
    controller_strategy_->initialize(ctrl_cfg);
    RCLCPP_INFO(get_logger(), "Created controller strategy: %s", controller_name_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Controller '%s' not found, using fallback inline controller", controller_name_.c_str());
  }

  // 初始化组件
  trajectory_generator_ = std::make_shared<TrajectoryGenerator>();
  trajectory_generator_->setFrameId(global_frame_);

  // 初始化订阅者和发布者
  initializeSubscribers();
  initializePublishers();

  // 初始化Action服务器
  initializeActionServer();

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

  // 停用定时器
  control_timer_->cancel();
  replanning_timer_->cancel();

  // 停用发布者
  cmd_vel_pub_->on_deactivate();
  path_pub_->on_deactivate();
  global_path_pub_->on_deactivate();

  // 停用规划器

  // 停止当前导航
  stopNavigation();

  current_state_ = NavigationState::IDLE;

  RCLCPP_INFO(get_logger(), "rosiwit_navigation node deactivated");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn NavigationNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up rosiwit_navigation node");

  // 清理组件
  planner_strategy_.reset();
  controller_strategy_.reset();

  // 重置订阅者和发布者
  odom_sub_.reset();
  map_sub_.reset();
  scan_sub_.reset();
  goal_sub_.reset();
  cmd_vel_pub_.reset();
  path_pub_.reset();
  global_path_pub_.reset();

  tf_buffer_.reset();
  tf_listener_.reset();

  RCLCPP_INFO(get_logger(), "rosiwit_navigation node cleaned up");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

LifecycleNode::CallbackReturn NavigationNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down rosiwit_navigation node");
  return LifecycleNode::CallbackReturn::SUCCESS;
}

void NavigationNode::loadParameters()
{
  this->declare_parameter("controller_frequency", 20.0);
  this->declare_parameter("planner_frequency", 1.0);
  this->declare_parameter("max_velocity_x", 0.5);
  this->declare_parameter("max_velocity_theta", 1.0);
  this->declare_parameter("min_velocity_x", -0.5);
  this->declare_parameter("max_accel_x", 0.5);
  this->declare_parameter("max_accel_theta", 1.0);
  this->declare_parameter("goal_tolerance_xy", 0.1);
  this->declare_parameter("goal_tolerance_yaw", 0.1);
  this->declare_parameter("lookahead_distance", 0.6);
  this->declare_parameter("min_lookahead_distance", 0.3);
  this->declare_parameter("max_lookahead_distance", 0.9);

  this->declare_parameter("global_frame", std::string("odom"));
  this->declare_parameter("robot_frame", std::string("base_link"));

  this->declare_parameter("planner_name", std::string("astar"));
  this->declare_parameter("controller_name", std::string("pure_pursuit"));
  this->declare_parameter("inflation_radius", 0.5);

  get_parameter("controller_frequency", params_.controller_frequency);
  get_parameter("planner_frequency", params_.planner_frequency);
  get_parameter("max_velocity_x", params_.max_velocity_x);
  get_parameter("max_velocity_theta", params_.max_velocity_theta);
  get_parameter("min_velocity_x", params_.min_velocity_x);
  get_parameter("max_accel_x", params_.max_accel_x);
  get_parameter("max_accel_theta", params_.max_accel_theta);
  get_parameter("goal_tolerance_xy", params_.goal_tolerance_xy);
  get_parameter("goal_tolerance_yaw", params_.goal_tolerance_yaw);
  get_parameter("lookahead_distance", params_.lookahead_distance);
  get_parameter("min_lookahead_distance", params_.min_lookahead_distance);
  get_parameter("max_lookahead_distance", params_.max_lookahead_distance);

  get_parameter("global_frame", global_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("planner_name", planner_name_);
  get_parameter("controller_name", controller_name_);
  get_parameter("inflation_radius", inflation_radius_);

  RCLCPP_INFO(get_logger(), "Parameters loaded:");
  RCLCPP_INFO(get_logger(), "  - controller_frequency: %.1f Hz", params_.controller_frequency);
  RCLCPP_INFO(get_logger(), "  - max_velocity_x: %.2f m/s", params_.max_velocity_x);
  RCLCPP_INFO(get_logger(), "  - goal_tolerance_xy: %.2f m", params_.goal_tolerance_xy);
}

void NavigationNode::initializeSubscribers()
{
  std::string odom_topic = this->declare_parameter("odom_topic", "/odom");
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&NavigationNode::odometryCallback, this, std::placeholders::_1));

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&NavigationNode::mapCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&NavigationNode::scanCallback, this, std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigationNode::goalCallback, this, std::placeholders::_1));

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

  RCLCPP_INFO(get_logger(), "Publishers initialized");
}

void NavigationNode::initializeActionServer()
{

  RCLCPP_INFO(get_logger(), "Action server initialized");
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

  RCLCPP_INFO(get_logger(), "Timers initialized");
}

void NavigationNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_velocity_ = msg->twist.twist;

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
      "map", msg->header.frame_id, msg->header.stamp,
      tf2::durationFromSec(0.1));
  } catch (tf2::TransformException &) { return; }

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
  // 激光数据更新后重新生成 costmap（含障碍物层）
  updateCostmap();
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
    for (size_t i = 0; i < current_map_->data.size(); ++i)
      cm.data[i] = (current_map_->data[i] >= 100) ? 255 : 0;

    // 叠加激光实时障碍物
    for (const auto& pt : laser_points_) {
      int mx = static_cast<int>((pt.x - ox) / res);
      int my = static_cast<int>((pt.y - oy) / res);
      if (mx >= 0 && mx < static_cast<int>(w) && my >= 0 && my < static_cast<int>(h))
        cm.data[my * w + mx] = 255;
    }

    planner_strategy_->setCostmap(cm);
  }

  // ========== 局部 costmap（机器人周围窗口，给控制器） ==========
  if (!controller_strategy_) return;

  // 获取机器人当前位置
  double rx = 0, ry = 0;
  try {
    auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.05));
    rx = tf.transform.translation.x;
    ry = tf.transform.translation.y;
  } catch (tf2::TransformException &) { return; }

  // 局部窗口参数
  double win_width = 6.0;   // 6x6m 窗口
  int win_half = static_cast<int>(std::ceil(win_width * 0.5 / res));
  int cx = static_cast<int>((rx - ox) / res);
  int cy = static_cast<int>((ry - oy) / res);

  // 提取窗口内的障碍物（静态地图 + 激光）
  core::ObstacleArray obstacles;
  std::vector<std::vector<bool>> visited(h, std::vector<bool>(w, false));

  // BFS 聚类 + 提取窗口内障碍物
  int step = std::max(1, static_cast<int>(0.3 / res));
  for (int dy = -win_half; dy <= win_half; dy += step) {
    for (int dx = -win_half; dx <= win_half; dx += step) {
      int mx = cx + dx, my = cy + dy;
      if (mx < 0 || mx >= static_cast<int>(w) || my < 0 || my >= static_cast<int>(h)) continue;
      if (visited[my][mx]) continue;

      bool is_obs = (current_map_->data[my * w + mx] >= 100);
      // 也检查激光点
      for (const auto& pt : laser_points_) {
        int lx = static_cast<int>((pt.x - ox) / res);
        int ly = static_cast<int>((pt.y - oy) / res);
        if (std::abs(lx - mx) <= 1 && std::abs(ly - my) <= 1) { is_obs = true; break; }
      }
      if (!is_obs) continue;

      // BFS 聚类
      std::queue<std::pair<int,int>> cluster_q;
      cluster_q.push({mx, my});
      visited[my][mx] = true;
      double sum_x = 0, sum_y = 0;
      int cnt = 0, min_x = mx, max_x = mx, min_y = my, max_y = my;

      while (!cluster_q.empty()) {
        auto [cx2, cy2] = cluster_q.front(); cluster_q.pop();
        sum_x += cx2; sum_y += cy2; ++cnt;
        min_x = std::min(min_x, cx2); max_x = std::max(max_x, cx2);
        min_y = std::min(min_y, cy2); max_y = std::max(max_y, cy2);

        for (int i = 0; i < 4; ++i) {
          int nx = cx2 + (i==0 ? -1 : i==1 ? 1 : 0);
          int ny = cy2 + (i==2 ? -1 : i==3 ? 1 : 0);
          if (nx >= 0 && nx < static_cast<int>(w) && ny >= 0 && ny < static_cast<int>(h) && !visited[ny][nx]) {
            bool neighbor_obs = (current_map_->data[ny * w + nx] >= 100);
            if (!neighbor_obs) {
              for (const auto& pt : laser_points_) {
                int lx = static_cast<int>((pt.x - ox) / res);
                int ly = static_cast<int>((pt.y - oy) / res);
                if (nx == lx && ny == ly) { neighbor_obs = true; break; }
              }
            }
            if (neighbor_obs) {
              visited[ny][nx] = true;
              cluster_q.push({nx, ny});
            }
          }
        }
      }

      core::Obstacle obs;
      obs.x = ox + (sum_x / cnt + 0.5) * res;
      obs.y = oy + (sum_y / cnt + 0.5) * res;
      double w_ = (max_x - min_x + 1) * res;
      double h_ = (max_y - min_y + 1) * res;
      obs.radius = std::sqrt(w_ * w_ + h_ * h_) * 0.5;
      obstacles.push_back(obs);
    }
  }

  controller_strategy_->setObstacles(obstacles);
}

void NavigationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = msg;
  updateCostmap();
  RCLCPP_INFO(get_logger(), "Map received: %d x %d, res=%.3f",
    msg->info.width, msg->info.height, msg->info.resolution);
}

void NavigationNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(),
    "Received /goal_pose: (%.2f, %.2f, %.2f)",
    msg->pose.position.x, msg->pose.position.y,
    tf2::getYaw(msg->pose.orientation));

  goal_pose_ = *msg;
  executeNavigation(goal_pose_);
}

void NavigationNode::controlLoop()
{
  if (!is_navigating_ || is_paused_) {
    return;
  }

  // 更新当前位姿
  if (!updateCurrentPose()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Cannot get current pose");
    return;
  }

  // 检查是否到达目标
  if (isGoalReached()) {
    RCLCPP_INFO(get_logger(), "Goal reached!");
    current_state_ = NavigationState::GOAL_REACHED;
    stopNavigation();
    return;
  }

  // ========== 初始原地旋转：确保机器人面朝路径方向再前进 ==========
  if (m_need_initial_rotate) {
    geometry_msgs::msg::Twist rot_cmd;
    double current_yaw = tf2::getYaw(current_pose_.pose.orientation);
    double target_yaw = m_initial_rotate_target;
    double angle_error = target_yaw - current_yaw;
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    if (std::abs(angle_error) < 0.08) {
      m_need_initial_rotate = false;
      RCLCPP_INFO(get_logger(), "Initial rotation done, starting Pure Pursuit");
    } else {
      rot_cmd.angular.z = std::clamp(1.5 * angle_error, -0.8, 0.8);
      publishVelocityCommand(rot_cmd);
      return;
    }
  }

  // 计算速度命令
  geometry_msgs::msg::Twist cmd_vel = computeVelocityCommand();

  // 发布速度命令
  publishVelocityCommand(cmd_vel);

  // 发布路径（用于可视化）
  publishPath(current_path_);
}

void NavigationNode::replanningCheck()
{
  if (!is_navigating_ || is_paused_) {
    return;
  }

  // 检查是否需要重规划
  // TODO: 实现完整的重规划逻辑
  RCLCPP_DEBUG(get_logger(), "Replanning check");
}


void NavigationNode::executeNavigation(const geometry_msgs::msg::PoseStamped & goal)
{
  // 更新当前位姿
  if (!updateCurrentPose()) {
    RCLCPP_ERROR(get_logger(), "Cannot get current pose for planning");
    current_state_ = NavigationState::FAILED;
    return;
  }

  // 规划路径
  RCLCPP_INFO(get_logger(), "Planning path to goal...");
  current_state_ = NavigationState::PLANNING;

  if (planner_strategy_) {
    auto start_pose = RosUtils::toCorePose(current_pose_);
    auto goal_pose_core = RosUtils::toCorePose(goal);
    auto result = planner_strategy_->plan(start_pose, goal_pose_core);
    if (result.is_ok()) {
      current_path_ = RosUtils::toRosPath(result.value(), goal.header.frame_id, now());

      // 平滑路径
      if (current_path_.poses.size() >= 3) {
        planners::SmootherConfig smoother_cfg;
        smoother_cfg.max_iterations = 100;
        smoother_cfg.alpha_start = 0.5;
        smoother_cfg.alpha_end = 0.1;

        // 转换为 core 类型，平滑，再转回 ROS 类型
        core::Path core_path = RosUtils::toCorePath(current_path_);
        // 构建 core::CostmapGrid 用于碰撞检测
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
        core_path = planners::smoothPath(core_path, core_map, smoother_cfg);
        current_path_ = RosUtils::toRosPath(core_path, goal.header.frame_id, now());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Planner '%s' failed: %s",
        planner_name_.c_str(), result.error_message().c_str());
      current_state_ = NavigationState::FAILED;
      return;
    }
  }

  // 将路径注入控制器
  if (controller_strategy_) {
    controller_strategy_->setPath(RosUtils::toCorePath(current_path_));
  }

  // 发布全局路径
  global_path_pub_->publish(current_path_);

  RCLCPP_INFO(get_logger(),
    "Path planned with %zu points",
    current_path_.poses.size());

  // 检查是否需要先原地旋转（Pure Pursuit 只能前进）
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

    current_pose_.header.stamp = transform.header.stamp;
    current_pose_.header.frame_id = global_frame_;
    current_pose_.pose.position.x = transform.transform.translation.x;
    current_pose_.pose.position.y = transform.transform.translation.y;
    current_pose_.pose.position.z = transform.transform.translation.z;
    current_pose_.pose.orientation = transform.transform.rotation;

    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Transform error: %s", ex.what());
    return false;
  }
}

bool NavigationNode::isGoalReached()
{
  // 优先使用控制器的目标到达判断
  if (controller_strategy_) {
    auto core_pose = RosUtils::toCorePose(current_pose_);
    return controller_strategy_->isGoalReached(core_pose);
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
    auto core_vel = RosUtils::toCoreVelocity(current_velocity_);
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
  cmd_vel_pub_->publish(cmd_vel);
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

  // 发送零速度命令
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(stop_cmd);

  RCLCPP_INFO(get_logger(), "Navigation stopped");
}

void NavigationNode::pauseNavigation()
{
  is_paused_ = true;
  current_state_ = NavigationState::IDLE;

  // 发送零速度命令
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(stop_cmd);

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