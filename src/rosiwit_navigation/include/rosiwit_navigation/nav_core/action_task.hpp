// ============================================================================
// action_task.hpp — 单目标任务状态机 (ROS 无关)
//
// 定位: 这是"算法层的任务执行语义", 不是接口层。
//   * 接口层 (ROS1/ROS2 薄层) 只负责: 报文解析、话题/action 收发、TF/地图订阅。
//   * 本状态机负责: 单点/路径跟随/原地旋转/倒车的运行状态、进度、到位判定、
//     卡死看门狗, 以及"位置到位后再对齐目标航向"这条旧导航的行为。
// 两种接口层 (robot_movebase action / 原生 goal) 共用同一份实现。
// ============================================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE__ACTION_TASK_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__ACTION_TASK_HPP_

#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

enum class ActionKind
{
  IDLE = 0,
  NAVIGATE,     // 规划到目标点 (可带末端航向对齐)
  FOLLOW_PATH,  // 直接跟踪一条已有路径
  ROTATE,       // 原地旋转 (默认 180°)
  BACK_OFF,     // 直线倒车
};

enum class ActionTaskStatus
{
  IDLE = 0,
  RUNNING,
  SUCCEEDED,
  FAILED_STUCK,
};

struct ActionTaskConfig
{
  double goal_yaw_tolerance = 0.10;   // 位置到位后允许的航向误差 (rad)
  double rotate_tolerance = 0.05;     // 原地旋转完成阈值 (rad)
  double stuck_timeout = 60.0;        // 无进展超时 (s), <=0 关闭
  double progress_distance = 0.25;    // 判定"有进展"的位移 (m)
  double progress_yaw = 0.15;         // 判定"有进展"的转角 (rad)
  double back_speed = 0.10;           // 倒车线速度 (m/s)
  double rotate_speed = 0.60;         // 旋转角速度上限 (rad/s)
};

struct ActionTaskCommand
{
  ActionKind kind = ActionKind::IDLE;
  bool final_rotate = false;     // 位置到位后是否对齐 final_yaw
  double final_yaw = 0.0;
  double rotate_angle = 3.14159265358979323846;  // ROTATE: 相对当前航向的角度
  double back_distance = 0.0;    // BACK_OFF: 倒车距离 (m)
};

class ActionTask
{
public:
  ActionTask() = default;

  void configure(const ActionTaskConfig & config) { config_ = config; }
  const ActionTaskConfig & config() const { return config_; }

  // 开始任务. 返回 false 表示命令为空 (IDLE), 调用方应直接结束.
  bool start(const ActionTaskCommand & command, const Pose2D & pose, double now);

  // 每个控制周期调用一次.
  //   xy_reached     : 控制器是否已到达路径终点 (NAVIGATE/FOLLOW_PATH)
  //   path_progress  : 控制器给出的 0..1 进度 (NAVIGATE/FOLLOW_PATH)
  ActionTaskStatus update(const Pose2D & pose, double now,
                          bool xy_reached, double path_progress);

  void reset();

  ActionKind kind() const { return command_.kind; }
  ActionTaskStatus status() const { return status_; }
  bool active() const { return status_ == ActionTaskStatus::RUNNING; }
  double progress() const;

  // 原地旋转 (ROTATE, 或 NAVIGATE 的末端航向对齐)
  bool rotating() const { return rotating_; }
  double rotationTarget() const { return rotation_target_; }
  double rotationCommand(const Pose2D & pose) const;

  // 倒车
  bool backing() const { return backing_; }
  double backRemaining() const;
  double backOffCommand() const;

private:
  double angleError(double target, double current) const;
  void anchorProgress(const Pose2D & pose, double now);
  bool rotationDone(const Pose2D & pose) const;

  ActionTaskConfig config_;
  ActionTaskCommand command_;
  ActionTaskStatus status_ = ActionTaskStatus::IDLE;

  bool rotating_ = false;
  double rotation_target_ = 0.0;

  bool backing_ = false;
  double back_target_ = 0.0;
  double back_traveled_ = 0.0;
  Pose2D back_last_pose_;

  Pose2D progress_pose_;
  double progress_yaw_ = 0.0;
  double progress_time_ = 0.0;
  double progress_ = 0.0;
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__ACTION_TASK_HPP_
