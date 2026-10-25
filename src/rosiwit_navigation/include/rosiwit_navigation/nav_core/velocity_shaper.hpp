// ============================================================================
// velocity_shaper.hpp —— 速度指令整形 (对齐 nav2_velocity_smoother 的机制)
//
// 为什么要有它: 加速度限制原来写在每个控制器内部, 且 dt 硬编码 0.1s ——
// controller_frequency 一改, 实际加速度就跟着翻倍. 这里把
//   * 绝对限幅 (max_v / max_w)
//   * 加速度限幅 (按真实 dt)
//   * deadband (小于阈值置零, 防止憋电流/抖)
// 统一放在"指令唯一出口"里做, 控制器只管出理想速度.
// 超时回零(deadman)由调用方的看门狗负责: 没指令时反复调 shape(零) 即可平滑停.
// ============================================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE__VELOCITY_SHAPER_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__VELOCITY_SHAPER_HPP_

#include <algorithm>
#include <cmath>

#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

struct VelocityShaperConfig
{
  double max_v = 1.0;         // 绝对上限 (m/s)
  double max_w = 1.5;         // 绝对上限 (rad/s)
  double max_acc_v = 0.5;     // m/s^2
  double max_acc_w = 1.0;     // rad/s^2
  double deadband_v = 0.0;    // m/s
  double deadband_w = 0.0;    // rad/s
};

class VelocityShaper
{
public:
  void configure(const VelocityShaperConfig & config) { config_ = config; }

  const VelocityShaperConfig & config() const { return config_; }

  /// 按 dt 秒限幅. dt <= 0 视为 1 个控制周期.
  VelocityCommand shape(const VelocityCommand & cmd, double dt)
  {
    const double step = (dt > 1e-4) ? dt : 0.1;
    VelocityCommand out;
    out.linear_x = limit(cmd.linear_x, last_.linear_x, config_.max_acc_v * step, config_.max_v);
    out.lateral_y = cmd.lateral_y;
    out.angular_z = limit(cmd.angular_z, last_.angular_z, config_.max_acc_w * step, config_.max_w);
    if (std::fabs(out.linear_x) < config_.deadband_v) { out.linear_x = 0.0; }
    if (std::fabs(out.angular_z) < config_.deadband_w) { out.angular_z = 0.0; }
    last_ = out;
    return out;
  }

  void reset() { last_ = VelocityCommand(); }

  const VelocityCommand & last() const { return last_; }

private:
  static double limit(double target, double current, double max_delta, double abs_max)
  {
    target = std::max(-abs_max, std::min(abs_max, target));
    const double lo = current - max_delta;
    const double hi = current + max_delta;
    return std::max(lo, std::min(hi, target));
  }

  VelocityShaperConfig config_;
  VelocityCommand last_;
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__VELOCITY_SHAPER_HPP_
