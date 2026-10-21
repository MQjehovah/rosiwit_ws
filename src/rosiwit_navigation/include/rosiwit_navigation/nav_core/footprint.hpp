// ============================================================================
// footprint.hpp — 长方形/多边形车体的 costmap 代价查询
//
// 控制器 (DWA/MPPI) 原来的碰撞检查只看机器人中心点; 对长方形车体会漏掉
// 车头/车角。这里把 footprint 离散成采样点, 在每个候选位姿下取最大代价。
// 没有 footprint 时调用方传空 samples, 行为与旧版完全一致 (只看中心)。
// ============================================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE__FOOTPRINT_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__FOOTPRINT_HPP_

#include <utility>
#include <vector>

#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

// 把机器人坐标系多边形离散成采样点: 顶点 + 每条边按 spacing 加密 + 中心 (0,0)。
// footprint 少于 3 个点 / spacing <= 0 时返回空 (调用方回退到中心点检查).
std::vector<std::pair<double, double>> sampleFootprint(const Footprint & footprint,
                                                       double spacing);

// costmap 单点代价. 越界按 100 (致命) 处理; -1 (unknown) 也按 100.
int costAt(const CostmapGrid & costmap, double x, double y);

// footprint 在 pose 下的最大代价 (最保守的角点).
// samples 为空时返回中心点代价.
int footprintCost(const CostmapGrid & costmap, const Pose2D & pose,
                  const std::vector<std::pair<double, double>> & samples);

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__FOOTPRINT_HPP_
