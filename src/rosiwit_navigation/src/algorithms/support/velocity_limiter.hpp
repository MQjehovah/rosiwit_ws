// ============================================================
// Diffbot Navigation - 速度限制器
// 提供速度和加速度限制功能
// ============================================================

#ifndef ROSIWIT_NAVIGATION__CONTROLLER__VELOCITY_LIMITER_HPP_
#define ROSIWIT_NAVIGATION__CONTROLLER__VELOCITY_LIMITER_HPP_

#include <memory>
#include "rosiwit_navigation/nav_core/logger.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace rosiwit_navigation
{
namespace controller
{

/**
 * @brief 速度限制参数
 */
struct VelocityLimits
{
  double max_velocity_x;
  double max_velocity_theta;
  double min_velocity_x;
  double min_velocity_theta;

  double max_accel_x;
  double max_accel_theta;
  double min_accel_x;
  double min_accel_theta;

  double max_decel_x;
  double max_decel_theta;

  VelocityLimits()
  : max_velocity_x(0.5), max_velocity_theta(1.0),
    min_velocity_x(-0.5), min_velocity_theta(-1.0),
    max_accel_x(0.5), max_accel_theta(1.0),
    min_accel_x(-0.5), min_accel_theta(-1.0),
    max_decel_x(0.5), max_decel_theta(1.0) {}
};

/**
 * @class VelocityLimiter
 * @brief 速度限制器，确保速度命令符合物理约束
 */
class VelocityLimiter
{
public:
  /// DEF-001: 最小有效时间步长，防止除零
  static constexpr double kMinTimeStep = 1e-9;

  /**
   * @brief 构造函数
   */
  VelocityLimiter();

  /**
   * @brief 构造函数，带参数
   */
  explicit VelocityLimiter(const VelocityLimits & limits);

  /**
   * @brief 析构函数
   */
  ~VelocityLimiter() = default;

  /**
   * @brief 设置速度限制
   */
  void setLimits(const VelocityLimits & limits);

  /**
   * @brief 获取当前限制
   */
  const VelocityLimits & getLimits() const { return limits_; }

  /**
   * @brief 应用速度限制
   * @param velocity 输入速度
   * @return 限制后的速度
   */
  core::VelocityCommand limitVelocity(
    const core::VelocityCommand & velocity);

  /**
   * @brief 应用加速度限制
   * @param current_velocity 当前速度
   * @param target_velocity 目标速度
   * @param dt 时间间隔
   * @return 限制后的速度
   */
  core::VelocityCommand limitAcceleration(
    const core::VelocityCommand & current_velocity,
    const core::VelocityCommand & target_velocity,
    double dt);

  /**
   * @brief 应用速度和加速度限制
   */
  core::VelocityCommand limitVelocityAndAcceleration(
    const core::VelocityCommand & current_velocity,
    const core::VelocityCommand & target_velocity,
    double dt);

  /**
   * @brief 计算停止距离
   * @param current_velocity 当前速度
   * @return 停止距离
   */
  double computeStoppingDistance(const core::VelocityCommand & current_velocity);

  /**
   * @brief 计算停止时间
   * @param current_velocity 当前速度
   * @return 停止时间
   */
  double computeStoppingTime(const core::VelocityCommand & current_velocity);

  /**
   * @brief 检查速度是否有效
   */
  bool isVelocityValid(const core::VelocityCommand & velocity);

  /**
   * @brief 应用速度比例限制
   * @param velocity 输入速度
   * @param scale 比例因子 (0-1)
   * @return 缩放后的速度
   */
  core::VelocityCommand scaleVelocity(
    const core::VelocityCommand & velocity,
    double scale);

private:
  /**
   * @brief 裁剪值到指定范围
   */
  double clamp(double value, double min_value, double max_value);

  // 成员变量
  VelocityLimits limits_;
  core::Logger logger_{core::Logger("velocity_limiter")};
};

} // namespace controller
} // namespace rosiwit_navigation

#endif // ROSIWIT_NAVIGATION__CONTROLLER__VELOCITY_LIMITER_HPP_