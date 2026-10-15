// include/slam_core/trajectory_metrics.h
// 轨迹评估: ATE (绝对轨迹误差) / RPE (相对位姿误差), 用于定位性能量化与回归.
#pragma once
#include <vector>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

struct TrajMetrics {
    double ate_rmse = 0.0;         // 绝对轨迹误差 RMSE (m)
    double ate_mean = 0.0;         // 绝对轨迹误差均值 (m)
    double rpe_trans_rmse = 0.0;   // 相对位姿(平移)误差 RMSE (m)
    double rpe_rot_rmse_deg = 0.0; // 相对位姿(旋转)误差 RMSE (deg)
    int    n = 0;                  // 参与统计的位姿数
};

/// 评估轨迹: est 与 gt 按**索引**对齐(长度不齐取较小者). 无时间对齐/尺度对齐.
TrajMetrics evaluateTrajectory(const std::vector<PoseStamped>& est,
                               const std::vector<PoseStamped>& gt);

} // namespace rosiwit_slam
