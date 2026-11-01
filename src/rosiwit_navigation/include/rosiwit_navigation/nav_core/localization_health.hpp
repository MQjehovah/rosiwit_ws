// ============================================================================
// localization_health.hpp — 定位质量 -> 速度缩放 (纯函数, 可单测)
//
// 为什么单独抽出来: 这个判断原先散落在节点里 (slamHealthScale / 安全停车 /
// 定位告警各写一遍), 其中一处漏了"没有匹配分"的情况, 结果建图模式
// (matching_degree == 0) 被判成"定位退化", 整条导航指令乘 0.5:
//   2026-11-01 仿真 10m 直走廊: v 0.8 -> 0.4, ω 0.8 -> 0.4, 42s -> 31s.
// 语义约定:
//   * 没有健康信息 (未接 SLAM / 数据超时) -> 不限速
//   * matching <= 0 -> 没有匹配分 (建图模式 / SLAM 不提供该字段) -> 不限速
//   * 0 < matching < degrade_matching -> 退化, 乘 degrade_scale
// ============================================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE__LOCALIZATION_HEALTH_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__LOCALIZATION_HEALTH_HPP_

namespace rosiwit_navigation
{
namespace core
{

/// @param info_fresh       /slam/info 是否新鲜 (未超时)
/// @param matching         匹配度 (0~1); <=0 表示没有这个分数
/// @param degrade_matching 低于此值算退化 (例如 0.65)
/// @param degrade_scale    退化时的速度缩放 (例如 0.5)
inline double localizationSpeedScale(bool info_fresh, double matching,
                                     double degrade_matching, double degrade_scale)
{
  if (!info_fresh) { return 1.0; }
  // 用 !(x > 0) 而不是 x <= 0: 同时把 NaN 归到"没有分数"这一侧, 不能因为
  // 一个 NaN 让车永远半速.
  if (!(matching > 0.0)) { return 1.0; }
  if (matching < degrade_matching) { return degrade_scale; }
  return 1.0;
}

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__LOCALIZATION_HEALTH_HPP_
