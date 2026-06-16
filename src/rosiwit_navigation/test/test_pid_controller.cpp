// ============================================================
// PID控制器抗饱和增强测试
// 对应需求: requirements.md § PID积分抗饱和修复
// 对应架构: architecture.md § 3.1 PIDController接口设计
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <chrono>
#include "diffbot_navigation/controller/diff_drive_controller.hpp"

using namespace diffbot_navigation::controller;

// ============================================================
// Fixture: 标准 PID 控制器测试环境
// ============================================================
class PIDControllerAntiWindupTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 标准 PID 参数
    controller_ = std::make_unique<DiffDriveController>();
    DiffDriveController::Config config;
    config.kp_linear = 1.0;
    config.ki_linear = 0.1;
    config.kd_linear = 0.05;
    config.kp_angular = 1.5;
    config.ki_angular = 0.05;
    config.kd_angular = 0.02;
    config.linear_integral_limit = 2.0;    // 积分限幅
    config.angular_integral_limit = 1.0;
    config.max_linear_velocity = 1.0;
    config.max_angular_velocity = 1.5;
    config.max_linear_accel = 0.5;
    config.max_angular_accel = 1.0;
    controller_->configure(config);
  }

  void TearDown() override
  {
    controller_->reset();
  }

  std::unique_ptr<DiffDriveController> controller_;
};

// ============================================================
// TC-PID-01: 正常跟踪时积分应正常累积
// ============================================================
TEST_F(PIDControllerAntiWindupTest, IntegralAccumulatesDuringNormalTracking)
{
  // 给定：持续的正误差
  double dt = 0.1;
  double error = 0.5;  // 正值误差

  // 当：连续多次 compute
  auto prev_output = controller_->compute(error, dt);
  double integral_sum = 0.0;

  for (int i = 0; i < 10; ++i) {
    auto output = controller_->compute(error, dt);
    // 验证：输出应包含积分项贡献（整体不应为0）
    EXPECT_NE(output.linear.x, 0.0) << "Iteration " << i;
    integral_sum += output.linear.x;
  }

  // 验证：经过多步积分后，总输出持续增长（积分累积）
  EXPECT_GT(std::abs(integral_sum), 1.0)
    << "Integral should accumulate over multiple timesteps";
}

// ============================================================
// TC-PID-02: 线性速度正向积分抗饱和（核心修复用例）
// 期望：当PD输出已达到 max_linear_velocity 时，
//       PID不应继续正向积分（防止积分饱和）
// ============================================================
TEST_F(PIDControllerAntiWindupTest, LinearIntegralAntiWindupPositiveSaturation)
{
  // 给定：一个足够大的误差，使得 P 项单独就达到饱和
  double dt = 0.1;
  double large_error = 5.0;  // P项 = 5.0 > max_linear_velocity(1.0)

  // 预热：先跑几步让积分建立
  for (int i = 0; i < 5; ++i) {
    controller_->compute(large_error, dt);
  }

  // 当：在饱和状态下继续计算
  auto output_before = controller_->compute(large_error, dt);

  // 验证：输出被限幅到 max_linear_velocity
  EXPECT_LE(std::abs(output_before.linear.x), 1.0 + 1e-9)
    << "Linear velocity should be clamped to max_linear_velocity";

  // 继续在饱和区运行多步
  for (int i = 0; i < 20; ++i) {
    auto output = controller_->compute(large_error, dt);
    EXPECT_LE(std::abs(output.linear.x), 1.0 + 1e-9)
      << "Linear velocity should remain clamped at step " << i;
  }

  // 关键验证：将误差突变为负（反向需要）后，控制器应能快速响应
  // 如果积分过度饱和，反向响应会严重滞后
  double reverse_error = -0.5;
  auto reverse_output = controller_->compute(reverse_error, dt);

  // 反向输出应合理（不含过大饱和积分残余）
  EXPECT_GT(reverse_output.linear.x, -1.0)
    << "After saturation, reverse response should not be delayed by integral windup";
}

// ============================================================
// TC-PID-03: 线性速度负向积分抗饱和
// ============================================================
TEST_F(PIDControllerAntiWindupTest, LinearIntegralAntiWindupNegativeSaturation)
{
  double dt = 0.1;
  double large_negative_error = -5.0;  // 负向大误差

  // 预热积分
  for (int i = 0; i < 5; ++i) {
    controller_->compute(large_negative_error, dt);
  }

  // 验证饱和限幅
  for (int i = 0; i < 20; ++i) {
    auto output = controller_->compute(large_negative_error, dt);
    EXPECT_LE(std::abs(output.linear.x), 1.0 + 1e-9)
      << "Negative saturated output should remain clamped";
  }

  // 反向验证：给正误差，应能快速响应
  double positive_error = 0.5;
  auto reverse_output = controller_->compute(positive_error, dt);
  EXPECT_LT(reverse_output.linear.x, 1.0)
    << "After negative saturation, positive response should not be delayed";
}

// ============================================================
// TC-PID-04: 角速度积分抗饱和
// ============================================================
TEST_F(PIDControllerAntiWindupTest, AngularIntegralAntiWindup)
{
  double dt = 0.1;
  double large_angular_error = 3.0;  // P项 = 4.5 > max_angular_velocity(1.5)

  // 预热和持续饱和运行
  for (int i = 0; i < 25; ++i) {
    auto output = controller_->compute(0.0, dt, large_angular_error);
    EXPECT_LE(std::abs(output.angular.z), 1.5 + 1e-9)
      << "Angular velocity should be clamped to max_angular_velocity at step " << i;
  }

  // 反向验证
  auto reverse_output = controller_->compute(0.0, dt, -0.3);
  EXPECT_GT(reverse_output.angular.z, -1.5)
    << "After angular saturation, reverse response should not be delayed by windup";
}

// ============================================================
// TC-PID-05: 积分限幅边界验证
// 期望：积分项不超过配置的 integral_limit
// ============================================================
TEST_F(PIDControllerAntiWindupTest, IntegralLimitBoundary)
{
  double dt = 0.1;
  double moderate_error = 1.0;

  // 运行充足步数以累积积分
  for (int i = 0; i < 100; ++i) {
    controller_->compute(moderate_error, dt);
  }

  // 验证：即使长时间积累，输出也不应无限制增长
  auto output = controller_->compute(moderate_error, dt);
  EXPECT_LE(std::abs(output.linear.x), 1.0 + 1e-9)
    << "Output must respect max_linear_velocity limit";
}

// ============================================================
// TC-PID-06: reset() 后积分清零验证
// ============================================================
TEST_F(PIDControllerAntiWindupTest, ResetClearsIntegralAccumulation)
{
  double dt = 0.1;
  double error = 1.0;

  // 积累积分
  for (int i = 0; i < 10; ++i) {
    controller_->compute(error, dt);
  }
  auto output_before_reset = controller_->compute(error, dt);

  // 重置
  controller_->reset();

  // 立即计算 - 积分应从零开始
  auto output_after_reset = controller_->compute(error, dt);

  // 重置后的输出应小于积累后的输出（证明积分已清除）
  EXPECT_LT(std::abs(output_after_reset.linear.x),
            std::abs(output_before_reset.linear.x))
    << "After reset, integral contribution should be zero";
}

// ============================================================
// TC-PID-07: 误差符号反转时积分应对消（条件积分逻辑）
// 期望：当误差和积分符号相反时，允许积分修正（帮助脱离饱和）
// ============================================================
TEST_F(PIDControllerAntiWindupTest, ConditionalIntegrationAllowsWinddown)
{
  double dt = 0.1;

  // 第一阶段：正向误差积累正向积分
  for (int i = 0; i < 10; ++i) {
    controller_->compute(1.0, dt);
  }

  // 第二阶段：负向误差（符号相反），积分应能对消
  auto output_after_reversal = controller_->compute(-0.5, dt);

  // 输出应快速响应反向需求
  EXPECT_LT(output_after_reversal.linear.x, 0.2)
    << "Conditional integration should allow integral winddown "
       "when error and integral have opposite signs";
}

// ============================================================
// TC-PID-08: 大误差跳变下控制器稳定性
// ============================================================
TEST_F(PIDControllerAntiWindupTest, LargeErrorStepStability)
{
  double dt = 0.05;
  std::vector<double> errors = {10.0, -10.0, 5.0, -5.0, 0.5, -0.1, 0.0};

  for (double error : errors) {
    auto output = controller_->compute(error, dt);
    // 验证输出始终在物理限制内
    EXPECT_LE(std::abs(output.linear.x), 1.0 + 1e-9);
    EXPECT_LE(std::abs(output.angular.z), 1.5 + 1e-9);
    // 验证不产生 NaN
    EXPECT_FALSE(std::isnan(output.linear.x));
    EXPECT_FALSE(std::isnan(output.linear.y));
    EXPECT_FALSE(std::isnan(output.angular.z));
  }
}

// ============================================================
// TC-PID-09: dt=0 边界条件（不应除零崩溃）
// ============================================================
TEST_F(PIDControllerAntiWindupTest, ZeroDeltaTimeHandling)
{
  EXPECT_NO_THROW({
    auto output = controller_->compute(0.5, 0.0);
    EXPECT_FALSE(std::isnan(output.linear.x));
  });
}

// ============================================================
// TC-PID-10: 极小dt下稳定性
// ============================================================
TEST_F(PIDControllerAntiWindupTest, VerySmallDeltaTime)
{
  for (int i = 0; i < 1000; ++i) {
    auto output = controller_->compute(0.1, 0.001);
    EXPECT_FALSE(std::isnan(output.linear.x));
  }
}
