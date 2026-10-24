#pragma once

#include "rosiwit_navigation/nav_core/trajectory.hpp"

namespace rosiwit_navigation
{
namespace planners
{

/// 路径 -> 时间参数化轨迹.
/// 处理: 曲率限速、双向加速度积分、cusp 处 v=0、原地旋转段、末段减速.
core::TimedTrajectory parameterizeTrajectory(const core::Path & path,
                                             const core::TrajectoryLimits & limits);

}  // namespace planners
}  // namespace rosiwit_navigation
