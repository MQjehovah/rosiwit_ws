// ============================================================================
// movebase_interface.cpp — 见 movebase_interface.h
// ============================================================================
#include "movebase_interface.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>

#include <actionlib_msgs/GoalStatus.h>

namespace rosiwit_navigation
{
namespace movebase
{
namespace
{

geometry_msgs::Pose2D toRosPose2D(const core::Pose2D & pose)
{
  geometry_msgs::Pose2D out;
  out.x = pose.x;
  out.y = pose.y;
  out.theta = pose.theta;
  return out;
}

std::vector<geometry_msgs::Pose2D> toRosPose2DArray(const std::vector<core::Pose2D> & poses,
                                                    size_t max_size = 10000)
{
  std::vector<geometry_msgs::Pose2D> out;
  const size_t count = std::min(poses.size(), max_size);
  out.reserve(count);
  for (size_t i = 0; i < count; ++i) { out.push_back(toRosPose2D(poses[i])); }
  return out;
}

// ServerGoalHandle 没有 isActive(); 用 GoalStatus 判断.
bool isGoalActive(
  const actionlib::ActionServer<xzrobot_ros_dependency::NaviCmdActionAction>::GoalHandle & handle)
{
  return handle.isValid() &&
         handle.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE;
}

// 收到 cancel 后 actionlib 会先把状态改成 PREEMPTING 再调回调,
// 此时仍然必须 setCanceled() 才能把 result 发回客户端.
bool isGoalPending(
  const actionlib::ActionServer<xzrobot_ros_dependency::NaviCmdActionAction>::GoalHandle & handle)
{
  if (!handle.isValid()) { return false; }
  const auto status = handle.getGoalStatus().status;
  return status == actionlib_msgs::GoalStatus::ACTIVE ||
         status == actionlib_msgs::GoalStatus::PREEMPTING;
}

double yawFromQuat(const geometry_msgs::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

Interface::Interface(ros::NodeHandle & nh, ros::NodeHandle & pnh)
: nh_(nh), pnh_(pnh)
{
  pnh_.param<bool>("movebase_compat_enabled", enabled_, true);
  pnh_.param<double>("movebase_prediction_horizon", prediction_horizon_, 1.0);
  pnh_.param<std::string>("movebase_action_name", action_name_, "RobotNaviCmdAction");
  pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
  pnh_.param<std::string>("movebase_cmd_vel_topic", movebase_cmd_vel_topic_,
                          "/movebase_cmd_vel");
  pnh_.param<std::string>("map_frame", map_frame_, "map");

  if (!enabled_) {
    ROS_INFO("[movebase_iface] 兼容层已关闭 (movebase_compat_enabled=false)");
    return;
  }

  // ---- 话题: 与 robot_movebase 同名, 相对名字在根命名空间落地 ----
  movebase_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(movebase_cmd_vel_topic_, 1);
  local_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1);
  makeplan_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("makeplan_map", 1);
  robot_footprint_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("robot_footprint", 1);
  prediction_footprint_pub_ =
      nh_.advertise<geometry_msgs::PolygonStamped>("prediction_robot_footprint", 1);
  prediction_path_pub_ = nh_.advertise<nav_msgs::Path>("prediction_path", 1);
  // 注意: 旧包这个话题就叫 movebae_robot_path (拼写错误), 下游按原名订阅.
  robot_path_pub_ = nh_.advertise<nav_msgs::Path>("movebae_robot_path", 1);
  back_off_path_pub_ = nh_.advertise<nav_msgs::Path>("back_off_path", 1);
  docking_path_pub_ = nh_.advertise<nav_msgs::Path>("docking_path", 1);
  re_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("re_point", 1);
  // 以下 4 个话题旧实现只在清洁/区域业务里发, 这里只保证接口存在.
  area_type_pub_ = nh_.advertise<std_msgs::String>("area_type", 1);
  clean_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("clean_map", 1);
  clean_area_pub_ = nh_.advertise<std_msgs::Float64>("clean_area", 1);
  clean_part_status_pub_ = nh_.advertise<std_msgs::String>("clean_part_status", 1);

  publishDockingPathOnce();

  // ---- action (异步 GoalHandle, 由控制定时器负责完成) ----
  server_.reset(new Server(nh_, action_name_, false));
  server_->registerGoalCallback(boost::bind(&Interface::goalCallback, this, _1));
  server_->registerCancelCallback(boost::bind(&Interface::cancelCallback, this, _1));
  server_->start();
  ROS_INFO("[movebase_iface] action 服务端已启动: /%s (velocity: %s + %s)",
           action_name_.c_str(), cmd_vel_topic_.c_str(), movebase_cmd_vel_topic_.c_str());
}

Interface::~Interface()
{
  // ActionServer 的析构会停掉内部定时器/订阅; 它没有显式 shutdown().
}

void Interface::configure(const GoalCallback & on_goal, const CancelCallback & on_cancel)
{
  on_goal_ = on_goal;
  on_cancel_ = on_cancel;
}

void Interface::goalCallback(GoalHandle handle)
{
  if (!enabled_) { return; }

  // 新 goal 覆盖旧 goal: 照旧包"新目标先 preempt 旧目标"的行为.
  if (has_goal_ && isGoalPending(current_goal_)) {
    Server::Result canceled;
    current_goal_.setCanceled(canceled);
    has_goal_ = false;
    if (on_cancel_) { on_cancel_(); }
  }

  current_goal_ = handle;
  current_goal_.setAccepted();
  has_goal_ = true;

  if (!on_goal_) {
    ROS_ERROR("[movebase_iface] 未注册 goal 回调, 拒绝 navi_cmd=%d", handle.getGoal()->navi_cmd);
    Outcome outcome;
    outcome.tracking_path_result = RESULT_PLAN_FAILED;
    succeed(outcome);
    return;
  }

  const auto msg = handle.getGoal();
  Goal goal;
  goal.navi_cmd = msg->navi_cmd;
  goal.robot_vel = msg->robot_vel;
  goal.path_index = msg->path_index;
  goal.path_segment_index = msg->path_segment_index;
  // 现场 xzrobot_ros_dependency 版本没有 set_navi_mode 字段
  // (robot_movebase 源码里的 .action 比现场消息包新), 不能访问它.
  goal.backing_dist = msg->backing_dist;
  // A malformed/very large tracking_path would otherwise be copied several
  // times (action message -> Goal -> action_goal_ -> Decoded::path) and can
  // exhaust the 7.7GB robot.  Keep a hard bound on what this node accepts.
  constexpr size_t kMaxGoalPathPoints = 20000;
  const size_t incoming_points = msg->tracking_path.size();
  if (incoming_points > kMaxGoalPathPoints) {
    ROS_ERROR("[movebase_iface] tracking_path has %zu points (max %zu); truncating",
              incoming_points, kMaxGoalPathPoints);
  }
  const size_t copy_points = std::min(incoming_points, kMaxGoalPathPoints);
  goal.tracking_path.reserve(copy_points);
  for (size_t i = 0; i < copy_points; ++i) {
    const auto & pose = msg->tracking_path[i];
    goal.tracking_path.emplace_back(pose.x, pose.y, pose.theta);
  }
  goal.file_path = msg->file_path;

  // goal 回调在 ros::spin 线程里; on_goal_ 只做解码/启动, 不阻塞.
  on_goal_(goal);
}

void Interface::cancelCallback(GoalHandle /*handle*/)
{
  if (!enabled_) { return; }
  ROS_INFO("[movebase_iface] 收到 cancel, 停止当前导航");
  if (on_cancel_) { on_cancel_(); }
  Server::Result canceled;
  if (isGoalPending(current_goal_)) {
    current_goal_.setCanceled(canceled);
  }
  has_goal_ = false;
  current_goal_ = GoalHandle();
}

void Interface::publishCmdVel(const geometry_msgs::Twist & cmd)
{
  if (!enabled_) { return; }
  // 旧包里 movebase_cmd_vel 才是底盘真正消费的控制量 (cmd_vel 是对桩用).
  if (movebase_cmd_vel_pub_) { movebase_cmd_vel_pub_.publish(cmd); }
}

void Interface::publishLocalMap(const nav_msgs::OccupancyGrid & map)
{
  if (enabled_ && local_map_pub_) { local_map_pub_.publish(map); }
}

void Interface::publishPlanMap(const nav_msgs::OccupancyGrid & map)
{
  if (enabled_ && makeplan_map_pub_) { makeplan_map_pub_.publish(map); }
}

void Interface::publishFootprints(const geometry_msgs::PoseStamped & pose,
                                  const geometry_msgs::Twist & velocity,
                                  const std::vector<geometry_msgs::Point> & footprint)
{
  if (!enabled_ || footprint.empty()) { return; }
  // 周期发布 (1Hz, 体积极小): 不依赖任务状态, RViz 在 idle 时也能看到车体轮廓.

  const double yaw = yawFromQuat(pose.pose.orientation);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const auto transform = [&](geometry_msgs::PolygonStamped & out) {
    out.header = pose.header;
    out.polygon.points.clear();
    out.polygon.points.reserve(footprint.size());
    for (const auto & point : footprint) {
      geometry_msgs::Point32 p;
      p.x = static_cast<float>(pose.pose.position.x + point.x * cy - point.y * sy);
      p.y = static_cast<float>(pose.pose.position.y + point.x * sy + point.y * cy);
      p.z = 0.0f;
      out.polygon.points.push_back(p);
    }
  };

  geometry_msgs::PolygonStamped current;
  transform(current);
  current.header.stamp = ros::Time::now();
  if (robot_footprint_pub_) { robot_footprint_pub_.publish(current); }

  // 预测框: 按当前速度外推 prediction_horizon 秒.
  geometry_msgs::PolygonStamped predicted;
  predicted.header.frame_id = pose.header.frame_id;
  predicted.header.stamp = current.header.stamp;
  predicted.polygon.points.clear();
  predicted.polygon.points.reserve(footprint.size());
  const double horizon = std::max(0.0, prediction_horizon_);
  const double px = pose.pose.position.x + velocity.linear.x * horizon * cy;
  const double py = pose.pose.position.y + velocity.linear.x * horizon * sy;
  const double pyaw = yaw + velocity.angular.z * horizon;
  const double pcy = std::cos(pyaw);
  const double psy = std::sin(pyaw);
  for (const auto & point : footprint) {
    geometry_msgs::Point32 p;
    p.x = static_cast<float>(px + point.x * pcy - point.y * psy);
    p.y = static_cast<float>(py + point.x * psy + point.y * pcy);
    p.z = 0.0f;
    predicted.polygon.points.push_back(p);
  }
  if (prediction_footprint_pub_) { prediction_footprint_pub_.publish(predicted); }
}

void Interface::publishPredictionPath(const nav_msgs::Path & path)
{
  if (enabled_ && prediction_path_pub_) { prediction_path_pub_.publish(path); }
}

void Interface::publishTraveledPath(const std::vector<geometry_msgs::PoseStamped> & poses)
{
  if (!enabled_ || !robot_path_pub_) { return; }
  nav_msgs::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = ros::Time::now();
  path.poses = poses;
  robot_path_pub_.publish(path);
}

void Interface::publishLookahead(const geometry_msgs::PointStamped & point)
{
  if (enabled_ && re_point_pub_) { re_point_pub_.publish(point); }
}

void Interface::publishBackOffPath(const nav_msgs::Path & path)
{
  if (enabled_ && back_off_path_pub_) { back_off_path_pub_.publish(path); }
}

void Interface::publishCoverageMap(const nav_msgs::OccupancyGrid & map)
{
  if (enabled_ && clean_map_pub_) { clean_map_pub_.publish(map); }
}

void Interface::publishCoveredArea(double area_m2)
{
  if (!enabled_ || !clean_area_pub_) { return; }
  std_msgs::Float64 msg;
  msg.data = area_m2;
  clean_area_pub_.publish(msg);
}

void Interface::publishCoverageStatus(const std::string & json)
{
  if (!enabled_ || !clean_part_status_pub_) { return; }
  std_msgs::String msg;
  msg.data = json;
  clean_part_status_pub_.publish(msg);
}

void Interface::publishFeedback(const Feedback & feedback)
{
  if (!enabled_ || !has_goal_ || !isGoalActive(current_goal_)) {
    return;
  }
  Server::Feedback msg;
  msg.execution_percent = static_cast<float>(clampPercent(feedback.execution_percent));
  msg.execution_index = feedback.execution_index;
  msg.execution_path_segment_index = feedback.execution_path_segment_index;
  msg.navi_state = static_cast<int8_t>(feedback.navi_state);
  msg.execution_path = toRosPose2DArray(feedback.execution_path);
  msg.local_path = toRosPose2DArray(feedback.local_path);
  // 现场消息包的清洁/闸机字段: 本包没有清洁业务, 回中性值.
  msg.clean_area = static_cast<float>(feedback.clean_area);
  msg.access_turnstile_id.clear();
  current_goal_.publishFeedback(msg);
}

void Interface::succeed(const Outcome & outcome)
{
  if (!enabled_ || !server_) { return; }
  if (!has_goal_) {
    ROS_WARN("[movebase_iface] 无活动 goal, 丢弃结果码 %d", outcome.tracking_path_result);
    return;
  }
  Server::Result msg;
  msg.tracking_path_result = static_cast<int8_t>(outcome.tracking_path_result);
  msg.execution_percent = static_cast<float>(clampPercent(outcome.execution_percent));
  msg.execution_index = outcome.execution_index;
  msg.execution_path_segment_index = outcome.execution_path_segment_index;
  msg.execution_path = toRosPose2DArray(outcome.execution_path);
  msg.clean_area = static_cast<float>(outcome.clean_area);
  has_goal_ = false;
  if (isGoalActive(current_goal_)) {
    // 旧实现对成功和失败都调用 setSucceeded, 失败信息只在 tracking_path_result 里.
    current_goal_.setSucceeded(msg);
  }
  // GoalHandle may retain the complete goal message (including a huge
  // FOLLOW_PATH tracking_path) after the result.  Release it explicitly.
  current_goal_ = GoalHandle();
  ROS_INFO("[movebase_iface] 导航结束: result=%d percent=%.2f",
           outcome.tracking_path_result, outcome.execution_percent);
}

void Interface::preempted()
{
  if (!enabled_ || !server_) { return; }
  Server::Result canceled;
  if (isGoalPending(current_goal_)) {
    current_goal_.setCanceled(canceled);
  }
  has_goal_ = false;
  current_goal_ = GoalHandle();
}

void Interface::publishDockingPathOnce()
{
  // 旧实现只在进入对桩流程时发布; 这里发一条空路径, 让 RViz/订阅端知道话题存在.
  if (!enabled_ || !docking_path_pub_) { return; }
  nav_msgs::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = ros::Time::now();
  docking_path_pub_.publish(path);
}

}  // namespace movebase
}  // namespace rosiwit_navigation
