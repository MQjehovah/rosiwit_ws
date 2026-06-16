// ============================================================
// Diffbot Navigation - 速度限制器单元测试
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include "diffbot_navigation/controller/velocity_limiter.hpp"

using namespace diffbot_navigation::controller;

class VelocityLimiterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 使用默认参数初始化
    VelocityLimits limits;
    limits.max_velocity_x = 0.5;
    limits.max_velocity_theta = 1.0;
    limits.min_velocity_x = -0.5;
    limits.min_velocity_theta = -1.0;
    limits.max_accel_x = 0.5;
    limits.max_accel_theta = 1.0;
    limits.min_accel_x = -0.5;
    limits.min_accel_theta = -1.0;
    limiter_ = std::make_unique<VelocityLimiter>(limits);
  }

  std::unique_ptr<VelocityLimiter> limiter_;
};

// ==================== 速度限制测试 ====================

TEST_F(VelocityLimiterTest, LimitLinearVelocityWithinBounds)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.3;  // 在限制范围内
  velocity.angular.z = 0.5;

  auto result = limiter_->limitVelocity(velocity);

  EXPECT_DOUBLE_EQ(result.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(result.angular.z, 0.5);
}

TEST_F(VelocityLimiterTest, LimitLinearVelocityExceedsMax)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 1.0;  // 超过最大值
  velocity.angular.z = 0.0;

  auto result = limiter_->limitVelocity(velocity);

  EXPECT_DOUBLE_EQ(result.linear.x, 0.5);  // 应该被限制到0.5
}

TEST_F(VelocityLimiterTest, LimitLinearVelocityBelowMin)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = -1.0;  // 低于最小值
  velocity.angular.z = 0.0;

  auto result = limiter_->limitVelocity(velocity);

  EXPECT_DOUBLE_EQ(result.linear.x, -0.5);  // 应该被限制到-0.5
}

TEST_F(VelocityLimiterTest, LimitAngularVelocityExceedsMax)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.0;
  velocity.angular.z = 2.0;  // 超过最大值

  auto result = limiter_->limitVelocity(velocity);

  EXPECT_DOUBLE_EQ(result.angular.z, 1.0);  // 应该被限制到1.0
}

TEST_F(VelocityLimiterTest, LimitAngularVelocityBelowMin)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.0;
  velocity.angular.z = -2.0;  // 低于最小值

  auto result = limiter_->limitVelocity(velocity);

  EXPECT_DOUBLE_EQ(result.angular.z, -1.0);  // 应该被限制到-1.0
}

TEST_F(VelocityLimiterTest, LimitBothVelocitiesSimultaneously)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 2.0;    // 超过最大值
  velocity.angular.z = -1.5;  // 低于最小值

  auto result = limiter_->limitVelocity(velocity);

  EXPECT_DOUBLE_EQ(result.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(result.angular.z, -1.0);
}

// ==================== 加速度限制测试 ====================

TEST_F(VelocityLimiterTest, LimitAccelerationWithinBounds)
{
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  geometry_msgs::msg::Twist target_velocity;
  target_velocity.linear.x = 0.1;  // 加速度在范围内
  target_velocity.angular.z = 0.2;

  double dt = 0.1;  // 100ms
  auto result = limiter_->limitAcceleration(current_velocity, target_velocity, dt);

  // 加速度为 1.0 m/s²，在限制范围内
  EXPECT_NEAR(result.linear.x, 0.1, 1e-6);
  EXPECT_NEAR(result.angular.z, 0.2, 1e-6);
}

TEST_F(VelocityLimiterTest, LimitAccelerationExceedsMax)
{
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  geometry_msgs::msg::Twist target_velocity;
  target_velocity.linear.x = 1.0;  // 需要非常大的加速度
  target_velocity.angular.z = 0.0;

  double dt = 0.1;  // 100ms
  auto result = limiter_->limitAcceleration(current_velocity, target_velocity, dt);

  // 最大加速度为0.5 m/s²，dt=0.1s，所以最大变化为0.05
  EXPECT_DOUBLE_EQ(result.linear.x, 0.05);
}

TEST_F(VelocityLimiterTest, LimitDecelerationExceedsMin)
{
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.5;
  current_velocity.angular.z = 0.0;

  geometry_msgs::msg::Twist target_velocity;
  target_velocity.linear.x = 0.0;  // 急停
  target_velocity.angular.z = 0.0;

  double dt = 0.1;  // 100ms
  auto result = limiter_->limitAcceleration(current_velocity, target_velocity, dt);

  // 最大减速度为-0.5 m/s²，dt=0.1s，所以最大减速为0.05
  EXPECT_DOUBLE_EQ(result.linear.x, 0.45);
}

TEST_F(VelocityLimiterTest, LimitAngularAcceleration)
{
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  geometry_msgs::msg::Twist target_velocity;
  target_velocity.linear.x = 0.0;
  target_velocity.angular.z = 2.0;  // 需要非常大的角加速度

  double dt = 0.05;  // 50ms
  auto result = limiter_->limitAcceleration(current_velocity, target_velocity, dt);

  // 最大角加速度为1.0 rad/s²，dt=0.05s，所以最大变化为0.05
  EXPECT_DOUBLE_EQ(result.angular.z, 0.05);
}

// ==================== 综合限制测试 ====================

TEST_F(VelocityLimiterTest, LimitVelocityAndAccelerationCombined)
{
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.2;
  current_velocity.angular.z = 0.3;

  geometry_msgs::msg::Twist target_velocity;
  target_velocity.linear.x = 1.0;  // 超过速度和加速度限制
  target_velocity.angular.z = 2.0;  // 超过速度和加速度限制

  double dt = 0.1;
  auto result = limiter_->limitVelocityAndAcceleration(current_velocity, target_velocity, dt);

  // 先应用加速度限制，再应用速度限制
  // 加速度限制后: linear.x = 0.2 + 0.5*0.1 = 0.25
  //              angular.z = 0.3 + 1.0*0.1 = 0.4
  // 速度限制：都在范围内
  EXPECT_NEAR(result.linear.x, 0.25, 1e-6);
  EXPECT_NEAR(result.angular.z, 0.4, 1e-6);
}

// ==================== 边界条件测试 ====================

TEST_F(VelocityLimiterTest, ZeroVelocityInput)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.0;
  velocity.angular.z = 0.0;

  auto result = limiter_->limitVelocity(velocity);

  EXPECT_DOUBLE_EQ(result.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular.z, 0.0);
}

TEST_F(VelocityLimiterTest, VerySmallVelocityInput)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 1e-10;  // 非常小的值
  velocity.angular.z = 1e-10;

  auto result = limiter_->limitVelocity(velocity);

  EXPECT_NEAR(result.linear.x, 1e-10, 1e-15);
  EXPECT_NEAR(result.angular.z, 1e-10, 1e-15);
}

TEST_F(VelocityLimiterTest, LargeVelocityInput)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 1000.0;  // 非常大的值
  velocity.angular.z = -1000.0;

  auto result = limiter_->limitVelocity(velocity);

  EXPECT_DOUBLE_EQ(result.linear.x, 0.5);  // 应该被限制
  EXPECT_DOUBLE_EQ(result.angular.z, -1.0);
}

TEST_F(VelocityLimiterTest, ZeroDeltaTime)
{
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.1;
  current_velocity.angular.z = 0.2;

  geometry_msgs::msg::Twist target_velocity;
  target_velocity.linear.x = 0.3;
  target_velocity.angular.z = 0.4;

  double dt = 0.0;  // 时间间隔为0
  // 应该不崩溃，返回当前速度或目标速度
  EXPECT_NO_THROW({
    auto result = limiter_->limitAcceleration(current_velocity, target_velocity, dt);
  });
}

TEST_F(VelocityLimiterTest, NegativeDeltaTime)
{
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.1;
  current_velocity.angular.z = 0.2;

  geometry_msgs::msg::Twist target_velocity;
  target_velocity.linear.x = 0.3;
  target_velocity.angular.z = 0.4;

  double dt = -0.1;  // 负时间间隔
  // 应该处理异常情况
  EXPECT_NO_THROW({
    auto result = limiter_->limitAcceleration(current_velocity, target_velocity, dt);
  });
}

// ==================== 参数设置测试 ====================

TEST_F(VelocityLimiterTest, SetNewLimits)
{
  VelocityLimits new_limits;
  new_limits.max_velocity_x = 1.0;
  new_limits.max_velocity_theta = 2.0;
  new_limits.min_velocity_x = -1.0;
  new_limits.min_velocity_theta = -2.0;
  new_limits.max_accel_x = 1.0;
  new_limits.max_accel_theta = 2.0;
  new_limits.min_accel_x = -1.0;
  new_limits.min_accel_theta = -2.0;

  limiter_->setLimits(new_limits);

  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 1.5;  // 超过新限制
  velocity.angular.z = 2.5;

  auto result = limiter_->limitVelocity(velocity);

  EXPECT_DOUBLE_EQ(result.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(result.angular.z, 2.0);
}

TEST_F(VelocityLimiterTest, GetLimits)
{
  auto limits = limiter_->getLimits();

  EXPECT_DOUBLE_EQ(limits.max_velocity_x, 0.5);
  EXPECT_DOUBLE_EQ(limits.max_velocity_theta, 1.0);
  EXPECT_DOUBLE_EQ(limits.min_velocity_x, -0.5);
  EXPECT_DOUBLE_EQ(limits.min_velocity_theta, -1.0);
}

// ==================== 默认构造函数测试 ====================

TEST(VelocityLimiterDefaultTest, DefaultConstructor)
{
  VelocityLimiter default_limiter;
  auto limits = default_limiter.getLimits();

  // 检查默认值
  EXPECT_DOUBLE_EQ(limits.max_velocity_x, 0.5);
  EXPECT_DOUBLE_EQ(limits.max_velocity_theta, 1.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}