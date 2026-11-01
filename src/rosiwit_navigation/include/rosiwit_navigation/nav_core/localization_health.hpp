// ============================================================================
// localization_health.hpp — 定位质量 -> 速度缩放 (纯函数, 可单测)
//
// 为什么单独抽出来: 这个判断原先散落在节点里 (slamHealthScale / 安全停车 /
// 定位告警各写一遍), 其中一处漏了"没有匹配分"的情况, 结果建图模式
// (matching_degree == 0) 被判成"定位退化", 整条导航指令乘 0.5:
//   2026-11-01 仿真 10m 直走廊: v 0.8 -> 0.4, ω 0.8 -> 0.4, 42s -> 31s.
//
// 但要区分两种"没有匹配分":
//   * 建图/未接入定位 (slam_state == 0, 或 -1 = 未知) -> 本来就没有匹配语义, 不限速
//   * 定位模式 (slam_state == 1) 却拿不到分 -> 位姿还没有匹配依据, 必须降速
// 只按 matching 判断会把后者也放行 (2026-11-01 评审 finding 5)。
// ============================================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE__LOCALIZATION_HEALTH_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__LOCALIZATION_HEALTH_HPP_

namespace rosiwit_navigation
{
namespace core
{

enum class SlamMode
{
  Unknown = -1,   ///< 没有 /slam/info 或未提供 slam_state
  Mapping = 0,    ///< 建图: 没有匹配分是正常的
  Localization = 1,
  Error = 2,
};

/// @param info_fresh       /slam/info 是否新鲜 (未超时)
/// @param matching         匹配度 (0~1); <=0 表示没有这个分数
/// @param degrade_matching 低于此值算退化 (例如 0.65)
/// @param degrade_scale    退化时的速度缩放 (例如 0.5)
/// @param mode             slam_state (建图/定位/错误)
inline double localizationSpeedScale(bool info_fresh, double matching,
                                     double degrade_matching, double degrade_scale,
                                     SlamMode mode)
{
  if (!info_fresh) { return 1.0; }
  // 用 !(x > 0) 而不是 x <= 0: 同时把 NaN 归到"没有分数"这一侧
  if (!(matching > 0.0)) {
    // 建图 / 未知模式: 没有匹配语义 -> 不限速
    if (mode == SlamMode::Mapping || mode == SlamMode::Unknown) { return 1.0; }
    // 定位模式却没有分数: 位姿没有匹配依据 -> 按退化处理
    return degrade_scale;
  }
  if (matching < degrade_matching) { return degrade_scale; }
  return 1.0;
}

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__LOCALIZATION_HEALTH_HPP_
