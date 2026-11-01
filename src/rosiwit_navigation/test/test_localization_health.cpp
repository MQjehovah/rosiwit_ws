#include <gtest/gtest.h>

#include "rosiwit_navigation/nav_core/localization_health.hpp"

namespace
{
using rosiwit_navigation::core::localizationSpeedScale;

// 生产改动会让这些测试失败的情形:
//   * 去掉 "matching <= 0 -> 1.0" 守卫 -> MappingZeroMatchingIsNotDegraded 失败
//   * 把 fresh 判断写反 -> NoInfoMeansFullSpeed 失败
//   * 边界改成 <= / < 写错 -> AtThresholdIsHealthy 失败
TEST(LocalizationHealth, NoInfoMeansFullSpeed)
{
  // 没有 /slam/info 或已超时: 不能凭"没数据"就把车限速
  EXPECT_DOUBLE_EQ(localizationSpeedScale(false, 0.10, 0.65, 0.5), 1.0);
}

TEST(LocalizationHealth, MappingZeroMatchingIsNotDegraded)
{
  // matching_degree == 0 = 建图模式没有定位分, 不是退化 (2026-11-01 仿真:
  // 这里误判导致整条导航被乘 0.5, 10m 走廊 42s -> 31s)
  EXPECT_DOUBLE_EQ(localizationSpeedScale(true, 0.0, 0.65, 0.5), 1.0);
}

TEST(LocalizationHealth, LowMatchingDegrades)
{
  EXPECT_DOUBLE_EQ(localizationSpeedScale(true, 0.40, 0.65, 0.5), 0.5);
}

TEST(LocalizationHealth, AtThresholdIsHealthy)
{
  EXPECT_DOUBLE_EQ(localizationSpeedScale(true, 0.65, 0.65, 0.5), 1.0);
}

TEST(LocalizationHealth, GoodMatchingFullSpeed)
{
  // 实车当前状态: matching_degree ≈ 0.97
  EXPECT_DOUBLE_EQ(localizationSpeedScale(true, 0.97, 0.65, 0.5), 1.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
