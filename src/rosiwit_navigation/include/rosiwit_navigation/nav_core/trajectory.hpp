// 时间参数化轨迹 —— 把几何路径升级为 (s, t, v, w) 参考.
// 目的: cusp / 原地旋转 / 末段减速由轨迹生成器统一处理, 控制器只负责跟踪.
// 注: types.hpp 已有 core::TrajectoryPoint (历史结构), 这里用 TimedPoint 避免冲突.
#pragma once

#include <vector>

#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

struct TrajectoryLimits
{
  double max_v = 0.8;         // m/s
  double max_accel = 0.8;     // m/s^2
  double max_w = 0.8;         // rad/s
  double max_alpha = 0.6;     // rad/s^2
  double max_lat_accel = 0.5; // 横向加速度上限 (曲率限速 v <= sqrt(a_lat/kappa))
};

struct TimedPoint
{
  Pose2D pose;
  double s = 0.0;   // 累计弧长
  double t = 0.0;   // 到达时刻
  double v = 0.0;   // 参考线速度 (含方向符号)
  double w = 0.0;   // 参考角速度
};

struct TimedTrajectory
{
  std::vector<TimedPoint> points;
  double duration = 0.0;
  bool valid = false;
};

}  // namespace core
}  // namespace rosiwit_navigation
