// ============================================================================
// movebase_interface.cpp — 见 movebase_interface.h
// ============================================================================
#include "movebase_interface.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>

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

  // ---- action ----
  server_.reset(new Server(nh_, action_name_,
                           boost::bind(&Interface::goalCallback, this, _1), false));
  server_->registerPreemptCallback(boost::bind(&Interface::preemptCallback, this));
  server_->start();
  ROS_INFO("[movebase_iface] action 服务端已启动: /%s (velocity: %s + %s)",
           action_name_.c_str(), cmd_vel_topic_.c_str(), movebase_cmd_vel_topic_.c_str());
}

Interface::~Interface()
{
  if (server_) { server_->shutdown(); }
}

void Interface::configure(const GoalCallback & on_goal, const CancelCallback & on_cancel)
{
  on_goal_ = on_goal;
  on_cancel_ = on_cancel;
}

void Interface::goalCallback(const Server::GoalConstPtr & msg)
{
  if (!enabled_) { return; }
  has_goal_ = true;
  if (!on_goal_) {
    ROS_ERROR("[movebase_iface] 未注册 goal 回调, 拒绝 navi_cmd=%d", msg->navi_cmd);
    Outcome outcome;
    outcome.tracking_path_result = RESULT_PLAN_FAILED;
    succeed(outcome);
    return;
  }

  Goal goal;
  goal.navi_cmd = msg->navi_cmd;
  goal.robot_vel = msg->robot_vel;
  goal.path_index = msg->path_index;
  goal.path_segment_index = msg->path_segment_index;
  goal.set_navi_mode = msg->set_navi_mode;
  goal.backing_dist = msg->backing_dist;
  goal.tracking_path.reserve(msg->tracking_path.size());
  for (const auto & pose : msg->tracking_path) {
    goal.tracking_path.emplace_back(pose.x, pose.y, pose.theta);
  }
  goal.file_path = msg->file_path;

  has_goal_ = true;
  on_goal_(goal);
}

void Interface::preemptCallback()
{
  ROS_INFO("[movebase_iface] 收到 cancel, 停止当前导航");
  if (on_cancel_) { on_cancel_(); }
  // 旧实现 preemptCB(): 立即 setPreempted(), 不等控制循环.
  preempted();
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

void Interface::publishFeedback(const Feedback & feedback)
{
  if (!enabled_ || !server_ || !has_goal_ || !server_->isActive()) { return; }
  Server::Feedback msg;
  msg.execution_percent = static_cast<float>(clampPercent(feedback.execution_percent));
  msg.execution_index = feedback.execution_index;
  msg.execution_path_segment_index = feedback.execution_path_segment_index;
  msg.navi_state = static_cast<int8_t>(feedback.navi_state);
  msg.execution_path = toRosPose2DArray(feedback.execution_path);
  msg.local_path = toRosPose2DArray(feedback.local_path);
  server_->publishFeedback(msg);
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
  has_goal_ = false;
  if (server_->isActive()) {
    // 旧实现对成功和失败都调用 setSucceeded, 失败信息只在 tracking_path_result 里.
    server_->setSucceeded(msg);
  }
  ROS_INFO("[movebase_iface] 导航结束: result=%d percent=%.2f",
           outcome.tracking_path_result, outcome.execution_percent);
}

void Interface::preempted()
{
  if (!enabled_ || !server_) { return; }
  has_goal_ = false;
  if (server_->isActive()) { server_->setPreempted(); }
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
