#include "slam_core/calibration_controller.h"

#include <algorithm>
#include <cmath>

namespace rosiwit_slam
{

namespace
{
double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}
}  // namespace

CalibrationDecision CalibrationController::update(
  const TimeCalibrationResult & time, const YawCalibrationResult & yaw)
{
  decision_.applied = false;
  decision_.rolled_back = false;

  if (!config_.enabled) {
    decision_.state = "DISABLED";
    return decision_;
  }

  const bool time_ok = time.valid && time.confidence >= config_.min_confidence &&
                       std::abs(time.offset) <= config_.max_time_offset;
  const bool yaw_ok = yaw.valid && std::abs(yaw.yaw_offset) <= config_.max_yaw_offset;

  if (!time_ok && !yaw_ok) {
    ++fail_count_;
    if (decision_.time_offset != 0.0 || decision_.yaw_offset != 0.0) {
      if (fail_count_ >= config_.rollback_windows) {
        decision_.time_offset = 0.0;
        decision_.yaw_offset = 0.0;
        decision_.applied = true;
        decision_.rolled_back = true;
        decision_.state = "ROLLED_BACK";
        stable_count_ = 0;
        fail_count_ = 0;
      } else {
        decision_.state = "APPLIED";
      }
    } else {
      decision_.state = time.valid || yaw.valid ? "REJECTED" : "COLLECTING";
    }
    return decision_;
  }

  fail_count_ = 0;
  const double next_time = time_ok ? time.offset : candidate_time_;
  const double next_yaw = yaw_ok ? yaw.yaw_offset : candidate_yaw_;

  // 连续窗口必须落在 max_step 内，避免一次噪声估计直接改标定。
  if (stable_count_ == 0 ||
      std::abs(next_time - candidate_time_) > config_.max_step ||
      std::abs(next_yaw - candidate_yaw_) > config_.max_step) {
    candidate_time_ = next_time;
    candidate_yaw_ = next_yaw;
    stable_count_ = 1;
  } else {
    candidate_time_ = next_time;
    candidate_yaw_ = next_yaw;
    ++stable_count_;
  }

  if (stable_count_ < std::max(1, config_.stable_windows)) {
    decision_.state = "COLLECTING";
    return decision_;
  }

  const double desired_time = clamp(candidate_time_, -config_.max_time_offset,
                                    config_.max_time_offset);
  const double desired_yaw = clamp(candidate_yaw_, -config_.max_yaw_offset,
                                   config_.max_yaw_offset);
  const double dt = clamp(desired_time - decision_.time_offset, -config_.max_step,
                          config_.max_step);
  const double dy = clamp(desired_yaw - decision_.yaw_offset, -config_.max_step,
                          config_.max_step);
  decision_.time_offset += dt;
  decision_.yaw_offset += dy;
  decision_.applied = true;
  decision_.state = "APPLIED";
  return decision_;
}

CalibrationDecision CalibrationController::current() const
{
  return decision_;
}

void CalibrationController::reset()
{
  decision_ = CalibrationDecision{};
  candidate_time_ = 0.0;
  candidate_yaw_ = 0.0;
  stable_count_ = 0;
  fail_count_ = 0;
}

}  // namespace rosiwit_slam
