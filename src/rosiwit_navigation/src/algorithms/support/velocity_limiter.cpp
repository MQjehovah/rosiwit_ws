// ============================================================
// Diffbot Navigation - 速度限制器实现
// ============================================================

#include "velocity_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace rosiwit_navigation
{
namespace controller
{

VelocityLimiter::VelocityLimiter()
{
  // 初始化默认限制
}

VelocityLimiter::VelocityLimiter(const VelocityLimits & limits)
: limits_(limits)
{
}

void VelocityLimiter::setLimits(const VelocityLimits & limits)
{
  limits_ = limits;
}

geometry_msgs::msg::Twist VelocityLimiter::limitVelocity(
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::Twist limited = velocity;

  // 应用线速度限制
  limited.linear.x = clamp(velocity.linear.x,
    limits_.min_velocity_x, limits_.max_velocity_x);

  // 应用角速度限制
  limited.angular.z = clamp(velocity.angular.z,
    limits_.min_velocity_theta, limits_.max_velocity_theta);

  return limited;
}

geometry_msgs::msg::Twist VelocityLimiter::limitAcceleration(
  const geometry_msgs::msg::Twist & current_velocity,
  const geometry_msgs::msg::Twist & target_velocity,
  double dt)
{
  // DEF-001: 零时间步长保护 — 当 dt 无效时直接返回当前速度
  if (dt <= kMinTimeStep) {
    return current_velocity;
  }

  geometry_msgs::msg::Twist limited = target_velocity;

  // 计算所需加速度
  double accel_x = (target_velocity.linear.x - current_velocity.linear.x) / dt;
  double accel_theta = (target_velocity.angular.z - current_velocity.angular.z) / dt;

  // 应用加速度限制
  if (accel_x > limits_.max_accel_x) {
    limited.linear.x = current_velocity.linear.x + limits_.max_accel_x * dt;
  } else if (accel_x < limits_.min_accel_x) {
    limited.linear.x = current_velocity.linear.x + limits_.min_accel_x * dt;
  }

  if (accel_theta > limits_.max_accel_theta) {
    limited.angular.z = current_velocity.angular.z + limits_.max_accel_theta * dt;
  } else if (accel_theta < limits_.min_accel_theta) {
    limited.angular.z = current_velocity.angular.z + limits_.min_accel_theta * dt;
  }

  return limited;
}

geometry_msgs::msg::Twist VelocityLimiter::limitVelocityAndAcceleration(
  const geometry_msgs::msg::Twist & current_velocity,
  const geometry_msgs::msg::Twist & target_velocity,
  double dt)
{
  // 首先应用加速度限制
  geometry_msgs::msg::Twist limited = limitAcceleration(current_velocity, target_velocity, dt);

  // 然后应用速度限制
  limited = limitVelocity(limited);

  return limited;
}

double VelocityLimiter::computeStoppingDistance(const geometry_msgs::msg::Twist & current_velocity)
{
  // 使用最大减速计算停止距离
  // d = v^2 / (2 * a)
  double v = std::abs(current_velocity.linear.x);
  double a = limits_.max_decel_x;

  if (v < 0.001 || a <= 0.0) {
    return 0.0;
  }

  return (v * v) / (2.0 * a);
}

double VelocityLimiter::computeStoppingTime(const geometry_msgs::msg::Twist & current_velocity)
{
  // 使用最大减速计算停止时间
  // t = v / a
  double v = std::abs(current_velocity.linear.x);
  double a = limits_.max_decel_x;

  if (v < 0.001 || a <= 0.0) {
    return 0.0;
  }

  return v / a;
}

bool VelocityLimiter::isVelocityValid(const geometry_msgs::msg::Twist & velocity)
{
  // 检查速度是否在合理范围内
  bool linear_valid = velocity.linear.x >= limits_.min_velocity_x &&
    velocity.linear.x <= limits_.max_velocity_x;

  bool angular_valid = velocity.angular.z >= limits_.min_velocity_theta &&
    velocity.angular.z <= limits_.max_velocity_theta;

  return linear_valid && angular_valid;
}

geometry_msgs::msg::Twist VelocityLimiter::scaleVelocity(
  const geometry_msgs::msg::Twist & velocity,
  double scale)
{
  geometry_msgs::msg::Twist scaled = velocity;

  // 应用比例缩放
  scale = clamp(scale, 0.0, 1.0);

  scaled.linear.x *= scale;
  scaled.angular.z *= scale;

  // 确保缩放后的速度在限制范围内
  scaled = limitVelocity(scaled);

  return scaled;
}

double VelocityLimiter::clamp(double value, double min_value, double max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

} // namespace controller
} // namespace rosiwit_navigation