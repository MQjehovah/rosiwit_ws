// ============================================================================
// action_task.cpp — 见 action_task.hpp
// ============================================================================
#include "rosiwit_navigation/nav_core/action_task.hpp"

#include <algorithm>
#include <cmath>

namespace rosiwit_navigation
{
namespace core
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi) { angle -= 2.0 * kPi; }
  while (angle < -kPi) { angle += 2.0 * kPi; }
  return angle;
}

double clamp01(double value)
{
  if (!std::isfinite(value)) { return 0.0; }
  return std::max(0.0, std::min(1.0, value));
}

}  // namespace

void ActionTask::reset()
{
  status_ = ActionTaskStatus::IDLE;
  command_ = ActionTaskCommand();
  rotating_ = false;
  rotation_target_ = 0.0;
  backing_ = false;
  back_target_ = 0.0;
  back_traveled_ = 0.0;
  back_last_pose_ = Pose2D();
  progress_pose_ = Pose2D();
  progress_yaw_ = 0.0;
  progress_time_ = 0.0;
  progress_ = 0.0;
}

bool ActionTask::start(const ActionTaskCommand & command, const Pose2D & pose, double now)
{
  reset();
  command_ = command;
  progress_pose_ = pose;
  progress_yaw_ = pose.theta;
  progress_time_ = now;

  switch (command_.kind) {
    case ActionKind::IDLE:
      status_ = ActionTaskStatus::IDLE;
      return false;

    case ActionKind::ROTATE:
      rotating_ = true;
      rotation_target_ = normalizeAngle(pose.theta + command_.rotate_angle);
      status_ = ActionTaskStatus::RUNNING;
      return true;

    case ActionKind::BACK_OFF:
      backing_ = true;
      back_target_ = std::max(0.0, command_.back_distance);
      back_traveled_ = 0.0;
      back_last_pose_ = pose;
      if (back_target_ <= 1e-6) {
        status_ = ActionTaskStatus::SUCCEEDED;
      } else {
        status_ = ActionTaskStatus::RUNNING;
      }
      return true;

    case ActionKind::NAVIGATE:
    case ActionKind::FOLLOW_PATH:
      status_ = ActionTaskStatus::RUNNING;
      return true;
  }
  status_ = ActionTaskStatus::IDLE;
  return false;
}

ActionTaskStatus ActionTask::update(const Pose2D & pose, double now,
                                    bool xy_reached, double path_progress)
{
  if (status_ != ActionTaskStatus::RUNNING) { return status_; }

  // ---- 卡死看门狗: 位移或转角足够大才算"有进展" ----
  const double moved = std::hypot(pose.x - progress_pose_.x, pose.y - progress_pose_.y);
  const double turned = std::fabs(angleError(pose.theta, progress_yaw_));
  if (moved > config_.progress_distance || turned > config_.progress_yaw) {
    anchorProgress(pose, now);
  } else if (config_.stuck_timeout > 0.0 && now - progress_time_ > config_.stuck_timeout) {
    status_ = ActionTaskStatus::FAILED_STUCK;
    return status_;
  }

  switch (command_.kind) {
    case ActionKind::ROTATE:
      progress_ = clamp01(1.0 - std::fabs(angleError(rotation_target_, pose.theta)) / kPi);
      if (rotationDone(pose)) {
        progress_ = 1.0;
        status_ = ActionTaskStatus::SUCCEEDED;
      }
      break;

    case ActionKind::BACK_OFF: {
      const double step = std::hypot(
        pose.x - back_last_pose_.x, pose.y - back_last_pose_.y);
      back_traveled_ += step;
      back_last_pose_ = pose;
      progress_ = back_target_ > 1e-6 ? clamp01(back_traveled_ / back_target_) : 1.0;
      if (back_traveled_ >= back_target_ - 0.02) {
        progress_ = 1.0;
        status_ = ActionTaskStatus::SUCCEEDED;
      }
      break;
    }

    case ActionKind::NAVIGATE:
    case ActionKind::FOLLOW_PATH:
      if (!rotating_) {
        progress_ = clamp01(path_progress);
        if (xy_reached) {
          const double yaw_error = std::fabs(angleError(command_.final_yaw, pose.theta));
          if (command_.final_rotate && yaw_error > config_.goal_yaw_tolerance) {
            rotating_ = true;
            rotation_target_ = normalizeAngle(command_.final_yaw);
          } else {
            progress_ = 1.0;
            status_ = ActionTaskStatus::SUCCEEDED;
          }
        }
      } else {
        // 末端原地对齐: 进度从 0 重新开始, 避免回退到路径进度造成跳变.
        progress_ = clamp01(1.0 - std::fabs(angleError(rotation_target_, pose.theta)) / kPi);
        if (rotationDone(pose)) {
          progress_ = 1.0;
          status_ = ActionTaskStatus::SUCCEEDED;
        }
      }
      break;

    case ActionKind::IDLE:
      status_ = ActionTaskStatus::IDLE;
      break;
  }
  return status_;
}

double ActionTask::progress() const
{
  return clamp01(progress_);
}

double ActionTask::rotationCommand(const Pose2D & pose) const
{
  if (!rotating_) { return 0.0; }
  const double error = angleError(rotation_target_, pose.theta);
  if (std::fabs(error) < config_.rotate_tolerance) { return 0.0; }
  const double limit = std::max(0.05, config_.rotate_speed);
  return std::max(-limit, std::min(limit, 1.5 * error));
}

double ActionTask::backRemaining() const
{
  if (!backing_) { return 0.0; }
  return std::max(0.0, back_target_ - back_traveled_);
}

double ActionTask::backOffCommand() const
{
  if (!backing_) { return 0.0; }
  const double remaining = backRemaining();
  if (remaining <= 0.0) { return 0.0; }
  const double speed = std::max(0.01, config_.back_speed);
  return -std::min(speed, std::max(0.02, remaining));
}

double ActionTask::angleError(double target, double current) const
{
  return normalizeAngle(target - current);
}

void ActionTask::anchorProgress(const Pose2D & pose, double now)
{
  progress_pose_ = pose;
  progress_yaw_ = pose.theta;
  progress_time_ = now;
}

bool ActionTask::rotationDone(const Pose2D & pose) const
{
  return std::fabs(angleError(rotation_target_, pose.theta)) < config_.rotate_tolerance;
}

}  // namespace core
}  // namespace rosiwit_navigation
