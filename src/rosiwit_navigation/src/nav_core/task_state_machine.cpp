#include "rosiwit_navigation/nav_core/task_state_machine.hpp"

namespace rosiwit_navigation
{
namespace core
{

std::string toString(TaskState state)
{
  switch (state) {
    case TaskState::IDLE: return "IDLE";
    case TaskState::PLANNING: return "PLANNING";
    case TaskState::FOLLOWING: return "FOLLOWING";
    case TaskState::RECOVERING: return "RECOVERING";
    case TaskState::SUCCEEDED: return "SUCCEEDED";
    case TaskState::FAILED: return "FAILED";
    case TaskState::STOPPED: return "STOPPED";
  }
  return "UNKNOWN";
}

std::string toString(TaskEvent event)
{
  switch (event) {
    case TaskEvent::GOAL_RECEIVED: return "GOAL_RECEIVED";
    case TaskEvent::PLAN_OK: return "PLAN_OK";
    case TaskEvent::PLAN_FAIL: return "PLAN_FAIL";
    case TaskEvent::PATH_BLOCKED: return "PATH_BLOCKED";
    case TaskEvent::GOAL_REACHED: return "GOAL_REACHED";
    case TaskEvent::RECOVERY_STARTED: return "RECOVERY_STARTED";
    case TaskEvent::RECOVERY_DONE: return "RECOVERY_DONE";
    case TaskEvent::STOP: return "STOP";
    case TaskEvent::RESET: return "RESET";
  }
  return "UNKNOWN";
}

bool TaskStateMachine::handle(TaskEvent event)
{
  TaskState next = state_;
  switch (event) {
    case TaskEvent::GOAL_RECEIVED:
      next = TaskState::PLANNING;
      break;
    case TaskEvent::PLAN_OK:
      if (state_ == TaskState::PLANNING || state_ == TaskState::RECOVERING) {
        next = TaskState::FOLLOWING;
      }
      break;
    case TaskEvent::PLAN_FAIL:
      if (state_ == TaskState::PLANNING || state_ == TaskState::RECOVERING) {
        next = TaskState::FAILED;
      }
      break;
    case TaskEvent::PATH_BLOCKED:
    case TaskEvent::RECOVERY_STARTED:
      if (state_ == TaskState::FOLLOWING) {
        next = TaskState::RECOVERING;
      }
      break;
    case TaskEvent::GOAL_REACHED:
      if (state_ == TaskState::FOLLOWING) {
        next = TaskState::SUCCEEDED;
      }
      break;
    case TaskEvent::RECOVERY_DONE:
      if (state_ == TaskState::RECOVERING) {
        next = TaskState::PLANNING;
      }
      break;
    case TaskEvent::STOP:
      next = TaskState::STOPPED;
      break;
    case TaskEvent::RESET:
      next = TaskState::IDLE;
      break;
  }

  last_event_ = event;
  if (next != state_) {
    state_ = next;
    ++transitions_;
    return true;
  }
  return false;
}

std::string TaskStateMachine::stateName() const
{
  return toString(state_);
}

std::string TaskStateMachine::lastEventName() const
{
  return toString(last_event_);
}

}  // namespace core
}  // namespace rosiwit_navigation
