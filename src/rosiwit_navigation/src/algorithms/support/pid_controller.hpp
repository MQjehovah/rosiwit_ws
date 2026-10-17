// ============================================================
// Diffbot Navigation - PID 控制器（独立模块，无 ROS/Nav2 依赖）
// 
// 设计目标：
//   - 纯 C++ 实现，可用于单元测试而不需要 ROS 环境
//   - 支持多种抗饱和模式（条件积分/钳位/反算）
//   - 所有参数可在构造时或运行时配置
// ============================================================

#ifndef ROSIWIT_NAVIGATION__CONTROLLER__PID_CONTROLLER_HPP_
#define ROSIWIT_NAVIGATION__CONTROLLER__PID_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rosiwit_navigation
{
namespace controller
{

// ---------------------------------------------------------------------------
// PID 控制器常量
// ---------------------------------------------------------------------------
namespace PIDConstants {
    constexpr double kMinDt = 1e-6;               // 最小时间步长（防止除零）
    constexpr double kDefaultIntegralLimit = 1.0;  // 默认积分限幅
    constexpr double kDefaultOutputLimit = 1.0;    // 默认输出限幅
}

// ---------------------------------------------------------------------------
// 抗饱和模式枚举
// ---------------------------------------------------------------------------
enum class AntiWindupMode : uint8_t
{
  None = 0,                   // 无抗饱和（积分始终累加，适合测试基本功能）
  ConditionalIntegration = 1, // 条件积分：仅当 PD 输出未饱和时积分
  Clamping = 2,               // 钳位：直接限制积分项范围
  BackCalculation = 3         // 反算：超出限幅时按比例削减积分（更平滑）
};

/**
 * @brief PID控制器——支持多种抗饱和模式
 *
 * 使用方式：
 *   PIDController pid;
 *   pid.k_p = 1.0; pid.k_i = 0.1; pid.k_d = 0.01;
 *   pid.setMode(AntiWindupMode::ConditionalIntegration);
 *   double output = pid.compute(setpoint, current, dt);
 */
struct PIDController
{
  // 比例/积分/微分增益
  double k_p;
  double k_i;
  double k_d;

  // 内部状态
  double integral_error;
  double previous_error;
  double integral_limit;
  double output_limit;
  bool output_saturated;

  // 抗饱和模式
  AntiWindupMode anti_windup_mode;

  PIDController() : k_p(0.0), k_i(0.0), k_d(0.0),
                    integral_error(0.0), previous_error(0.0),
                    integral_limit(PIDConstants::kDefaultIntegralLimit),
                    output_limit(PIDConstants::kDefaultOutputLimit),
                    output_saturated(false),
                    anti_windup_mode(AntiWindupMode::ConditionalIntegration)
  {}

  /**
   * @brief 设置抗饱和模式
   * @param mode 新的抗饱和模式
   */
  void setMode(AntiWindupMode mode) noexcept
  {
    anti_windup_mode = mode;
  }

  /**
   * @brief 获取当前抗饱和模式
   */
  AntiWindupMode getMode() const noexcept
  {
    return anti_windup_mode;
  }

  /**
   * @brief 计算控制输出
   * @param setpoint 目标值
   * @param current  当前值
   * @param dt       时间步长
   * @return 控制输出（限幅后）
   */
  double compute(double setpoint, double current, double dt)
  {
    // 防止 dt 过小导致数值不稳定
    if (dt < PIDConstants::kMinDt) {
      integral_error = 0.0;
      previous_error = 0.0;
      return 0.0;
    }

    double error = setpoint - current;
    double derivative_error = (error - previous_error) / dt;
    previous_error = error;

    double p_term = k_p * error;
    double d_term = k_d * derivative_error;
    double pd_output = p_term + d_term;

    switch (anti_windup_mode) {
      case AntiWindupMode::None:
        // 无条件积分——用于基准测试
        integral_error += error * dt;
        output_saturated = false;
        break;

      case AntiWindupMode::ConditionalIntegration:
        // 条件积分：仅在 PD 未饱和或误差正拉回积分时累积
        if (std::abs(pd_output) < output_limit ||
            (error > 0.0 && integral_error < 0.0) ||
            (error < 0.0 && integral_error > 0.0)) {
          integral_error += error * dt;
        }
        output_saturated = (std::abs(pd_output + k_i * integral_error) >= output_limit);
        break;

      case AntiWindupMode::Clamping:
        // 直接钳位——简单有效
        integral_error += error * dt;
        integral_error = std::clamp(integral_error, -integral_limit, integral_limit);
        output_saturated = (std::abs(pd_output + k_i * integral_error) >= output_limit);
        break;

      case AntiWindupMode::BackCalculation:
        // 反算——用饱和差值按比例削减积分
        {
          integral_error += error * dt;
          double raw_output = pd_output + k_i * integral_error;
          if (std::abs(raw_output) > output_limit) {
            double saturated_output = std::clamp(raw_output, -output_limit, output_limit);
            double excess = raw_output - saturated_output;
            // 按比例削减积分
            if (std::abs(k_i) > PIDConstants::kMinDt) {
              integral_error -= excess / k_i;
            }
          }
          output_saturated = (std::abs(raw_output) >= output_limit);
        }
        break;
    }

    // 积分限幅（安全保护）
    integral_error = std::clamp(integral_error, -integral_limit, integral_limit);

    // 组合输出并限幅
    double total_output = pd_output + k_i * integral_error;
    return std::clamp(total_output, -output_limit, output_limit);
  }

  /**
   * @brief 重置所有内部状态
   */
  void reset() noexcept
  {
    integral_error = 0.0;
    previous_error = 0.0;
    output_saturated = false;
  }

  /**
   * @brief 检查控制器是否处于饱和状态
   */
  bool isSaturated() const noexcept
  {
    return output_saturated;
  }
};

}  // namespace controller
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__CONTROLLER__PID_CONTROLLER_HPP_
