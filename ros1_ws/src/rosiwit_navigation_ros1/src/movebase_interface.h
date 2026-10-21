// ============================================================================
// movebase_interface.h
// robot_movebase 协议适配层 (ROS 侧):
//   * action 服务端 /RobotNaviCmdAction (xzrobot_ros_dependency/NaviCmdActionAction)
//   * robot_movebase 的调试/反馈话题: movebase_cmd_vel / local_map / makeplan_map /
//     robot_footprint / prediction_robot_footprint / prediction_path /
//     movebae_robot_path / back_off_path / docking_path / re_point
//   * 仅占位的话题: area_type / clean_map / clean_area / clean_part_status
//     (旧实现只在清洁/区域业务下才发; 当前核心没有这些业务)
//
// 不做的事情 (有意为之, 见 docs/movebase-interface-compat.md):
//   * /RobotNaviState (robot_movebase/NaviState): 旧节点只 advertise、从不 publish,
//     依赖它需要引入 robot_movebase 消息包, 与"替换旧包"的目标冲突, 故不实现.
//   * /cmd_vel 订阅 (旧节点用它做打滑/怼桩判断), 会与自身发布形成回环, 不实现.
//   * /driver/chassis/info (xzrobot_ros_dependency/ChassisInfo): 只服务对桩, 不实现.
// ============================================================================
#ifndef ROSIWIT_NAVIGATION_ROS1__MOVEBASE_INTERFACE_H_
#define ROSIWIT_NAVIGATION_ROS1__MOVEBASE_INTERFACE_H_

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <xzrobot_ros_dependency/NaviCmdActionAction.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <boost/function.hpp>

#include "rosiwit_navigation/nav_core/types.hpp"
#include "movebase_protocol.h"

namespace rosiwit_navigation
{
namespace movebase
{

// action 反馈/结果里与运动进度有关的部分 (ROS 无关, 由节点填充)
struct Feedback
{
  double execution_percent = 0.0;
  long execution_index = 0;
  long execution_path_segment_index = 0;
  std::vector<core::Pose2D> execution_path;  // 已走过的轨迹
  std::vector<core::Pose2D> local_path;      // 剩余/局部路径
  int navi_state = 1;                        // 1 正常导航, 2 避障中
};

struct Outcome
{
  int tracking_path_result = RESULT_PLAN_FAILED;
  double execution_percent = 0.0;
  long execution_index = 0;
  long execution_path_segment_index = 0;
  std::vector<core::Pose2D> execution_path;
};

class Interface
{
public:
  // 用 ActionServer(GoalHandle) 而不是 SimpleActionServer:
  // 我们的运动由定时器驱动, goal 回调必须能立即返回,
  // 而 SimpleActionServer 会在 execute 回调返回时把 goal 自动 abort.
  using Server = actionlib::ActionServer<xzrobot_ros_dependency::NaviCmdActionAction>;
  using GoalHandle = Server::GoalHandle;
  using GoalCallback = boost::function<void(const Goal &)>;
  using CancelCallback = boost::function<void()>;

  Interface(ros::NodeHandle & nh, ros::NodeHandle & pnh);
  ~Interface();

  // 注册节点侧回调. 未注册时收到 goal 直接按失败结束.
  void configure(const GoalCallback & on_goal, const CancelCallback & on_cancel);

  bool enabled() const { return enabled_; }
  bool hasGoal() const { return has_goal_; }

  // ---- 输出 (未启用时全部是空操作) ----
  void publishCmdVel(const geometry_msgs::Twist & cmd);
  void publishLocalMap(const nav_msgs::OccupancyGrid & map);
  void publishPlanMap(const nav_msgs::OccupancyGrid & map);
  void publishFootprints(const geometry_msgs::PoseStamped & pose,
                         const geometry_msgs::Twist & velocity,
                         const std::vector<geometry_msgs::Point> & footprint);
  void publishPredictionPath(const nav_msgs::Path & path);
  void publishTraveledPath(const std::vector<geometry_msgs::PoseStamped> & poses);
  void publishLookahead(const geometry_msgs::PointStamped & point);
  void publishBackOffPath(const nav_msgs::Path & path);

  // ---- action 生命周期 ----
  void publishFeedback(const Feedback & feedback);
  void succeed(const Outcome & outcome);   // 旧实现成功/失败都用 setSucceeded + 结果码
  void preempted();                        // 对应旧实现 preemptCB 的 setPreempted()

private:
  void goalCallback(GoalHandle handle);
  void cancelCallback(GoalHandle handle);
  void publishDockingPathOnce();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  bool enabled_ = true;
  bool has_goal_ = false;
  double prediction_horizon_ = 1.0;
  std::string action_name_ = "RobotNaviCmdAction";
  std::string cmd_vel_topic_ = "/cmd_vel";
  std::string movebase_cmd_vel_topic_ = "/movebase_cmd_vel";
  std::string map_frame_ = "map";

  std::unique_ptr<Server> server_;
  GoalHandle current_goal_;
  GoalCallback on_goal_;
  CancelCallback on_cancel_;

  ros::Publisher movebase_cmd_vel_pub_;
  ros::Publisher local_map_pub_;
  ros::Publisher makeplan_map_pub_;
  ros::Publisher robot_footprint_pub_;
  ros::Publisher prediction_footprint_pub_;
  ros::Publisher prediction_path_pub_;
  ros::Publisher robot_path_pub_;
  ros::Publisher back_off_path_pub_;
  ros::Publisher docking_path_pub_;
  ros::Publisher re_point_pub_;
  ros::Publisher area_type_pub_;
  ros::Publisher clean_map_pub_;
  ros::Publisher clean_area_pub_;
  ros::Publisher clean_part_status_pub_;
};

}  // namespace movebase
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION_ROS1__MOVEBASE_INTERFACE_H_
