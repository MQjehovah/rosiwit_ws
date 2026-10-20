// ============================================================
// 在线标定闭环控制器
//
// 估计值不能直接写回系统：必须先过 confidence/稳定窗口/幅度门限，
// 应用后持续监控，置信度下降或估计跳变则回滚。该策略与实车
// "先观察、再小步应用、异常回滚" 的标定流程一致。
// ============================================================
#ifndef ROSIWIT_SLAM__CALIBRATION_CONTROLLER_H_
#define ROSIWIT_SLAM__CALIBRATION_CONTROLLER_H_

#include <string>

#include "slam_core/online_calibrator.h"

namespace rosiwit_slam
{

struct CalibrationApplyConfig
{
  bool enabled = false;
  double min_confidence = 0.60;
  int stable_windows = 3;        // 连续多少个估计窗口落在 step 内才应用
  int rollback_windows = 3;      // 连续多少个窗口失败后回滚
  double max_time_offset = 0.05; // 允许应用的时间偏移上限 (s)
  double max_yaw_offset = 0.15;  // 允许应用的 yaw 上限 (rad)
  double max_step = 0.01;        // 每次最多修正量
};

struct CalibrationDecision
{
  bool applied = false;
  bool rolled_back = false;
  double time_offset = 0.0;   // 已应用的累计时间偏移
  double yaw_offset = 0.0;    // 已应用的累计 yaw 偏移
  std::string state = "DISABLED";  // DISABLED/COLLECTING/APPLIED/ROLLED_BACK/REJECTED
};

class CalibrationController
{
public:
  CalibrationController() = default;
  explicit CalibrationController(const CalibrationApplyConfig & config) : config_(config) {}

  void setConfig(const CalibrationApplyConfig & config) { config_ = config; }
  const CalibrationApplyConfig & config() const { return config_; }

  CalibrationDecision update(const TimeCalibrationResult & time,
                             const YawCalibrationResult & yaw);

  CalibrationDecision current() const;
  void reset();

private:
  CalibrationApplyConfig config_;
  CalibrationDecision decision_;
  double candidate_time_ = 0.0;
  double candidate_yaw_ = 0.0;
  int stable_count_ = 0;
  int fail_count_ = 0;
};

}  // namespace rosiwit_slam

#endif  // ROSIWIT_SLAM__CALIBRATION_CONTROLLER_H_
