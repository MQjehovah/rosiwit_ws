// ============================================================
// 导航任务状态机 —— 行为树的最小可验证内核
//
// 状态: IDLE -> PLANNING -> FOLLOWING -> SUCCEEDED / FAILED
//                              \-> RECOVERING -> PLANNING
// 该状态机不依赖 ROS，ROS1/ROS2 薄层用它统一 goal_status、恢复行为和诊断。
// ============================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE_TASK_STATE_MACHINE_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE_TASK_STATE_MACHINE_HPP_

#include <string>

namespace rosiwit_navigation
{
namespace core
{

enum class TaskState
{
  IDLE = 0,
  PLANNING,
  FOLLOWING,
  RECOVERING,
  SUCCEEDED,
  FAILED,
  STOPPED
};

enum class TaskEvent
{
  GOAL_RECEIVED = 0,
  PLAN_OK,
  PLAN_FAIL,
  PATH_BLOCKED,
  GOAL_REACHED,
  RECOVERY_STARTED,
  RECOVERY_DONE,
  STOP,
  RESET
};

class TaskStateMachine
{
public:
  TaskStateMachine() = default;

  bool handle(TaskEvent event);
  TaskState state() const { return state_; }
  std::string stateName() const;
  std::string lastEventName() const;
  int transitionCount() const { return transitions_; }

  bool isActive() const
  {
    return state_ == TaskState::PLANNING || state_ == TaskState::FOLLOWING ||
           state_ == TaskState::RECOVERING;
  }

private:
  TaskState state_ = TaskState::IDLE;
  TaskEvent last_event_ = TaskEvent::RESET;
  int transitions_ = 0;
};

std::string toString(TaskState state);
std::string toString(TaskEvent event);

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE_TASK_STATE_MACHINE_HPP_
