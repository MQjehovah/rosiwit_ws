#include <gtest/gtest.h>

#include "rosiwit_navigation/nav_core/velocity_shaper.hpp"

namespace
{
using rosiwit_navigation::core::VelocityCommand;
using rosiwit_navigation::core::VelocityShaper;
using rosiwit_navigation::core::VelocityShaperConfig;

VelocityShaperConfig MakeConfig()
{
  VelocityShaperConfig cfg;
  cfg.max_v = 0.8;
  cfg.max_w = 1.5;
  cfg.max_acc_v = 0.5;
  cfg.max_acc_w = 1.0;
  cfg.deadband_v = 0.0;
  cfg.deadband_w = 0.0;
  return cfg;
}

}  // namespace

TEST(VelocityShaper, AccelLimitPerStep)
{
  VelocityShaper shaper;
  shaper.configure(MakeConfig());
  // 从 0 起步, 目标 0.8, dt=0.1, 加速度 0.5 -> 每步 0.05
  auto v = shaper.shape(VelocityCommand(0.8, 0.0, 0.0), 0.1);
  EXPECT_NEAR(v.linear_x, 0.05, 1e-9);
  v = shaper.shape(VelocityCommand(0.8, 0.0, 0.0), 0.1);
  EXPECT_NEAR(v.linear_x, 0.10, 1e-9);
}

TEST(VelocityShaper, DtIndependent)
{
  VelocityShaper a;
  VelocityShaper b;
  a.configure(MakeConfig());
  b.configure(MakeConfig());
  // 两次 0.05 与一次 0.1 应该等价
  a.shape(VelocityCommand(0.8, 0.0, 0.0), 0.05);
  const auto va = a.shape(VelocityCommand(0.8, 0.0, 0.0), 0.05);
  const auto vb = b.shape(VelocityCommand(0.8, 0.0, 0.0), 0.1);
  EXPECT_NEAR(va.linear_x, vb.linear_x, 1e-9);
}

TEST(VelocityShaper, AbsoluteClamp)
{
  VelocityShaper shaper;
  auto cfg = MakeConfig();
  cfg.max_acc_v = 100.0;   // 不限制加速度, 只看绝对上限
  cfg.max_acc_w = 100.0;
  shaper.configure(cfg);
  const auto v = shaper.shape(VelocityCommand(5.0, 0.0, -5.0), 0.1);
  EXPECT_DOUBLE_EQ(v.linear_x, cfg.max_v);
  EXPECT_DOUBLE_EQ(v.angular_z, -cfg.max_w);
}

TEST(VelocityShaper, DeadbandZeroesSmallCommands)
{
  VelocityShaper shaper;
  auto cfg = MakeConfig();
  cfg.max_acc_v = 100.0;
  cfg.deadband_v = 0.05;
  cfg.deadband_w = 0.05;
  shaper.configure(cfg);
  const auto v = shaper.shape(VelocityCommand(0.02, 0.0, 0.03), 0.1);
  EXPECT_DOUBLE_EQ(v.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(v.angular_z, 0.0);
}

TEST(VelocityShaper, RampDownToZeroIsSmooth)
{
  VelocityShaper shaper;
  auto cfg = MakeConfig();
  cfg.max_acc_v = 0.5;
  shaper.configure(cfg);
  shaper.shape(VelocityCommand(0.25, 0.0, 0.0), 0.5);   // 直接到 0.25
  const auto v = shaper.shape(VelocityCommand(0.0, 0.0, 0.0), 0.1);
  EXPECT_NEAR(v.linear_x, 0.20, 1e-9);                  // 只降 0.05
  EXPECT_GT(v.linear_x, 0.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
