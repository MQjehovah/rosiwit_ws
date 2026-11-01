#include <gtest/gtest.h>

#include "rosiwit_navigation/nav_core/localization_health.hpp"

namespace
{
using rosiwit_navigation::core::localizationSpeedScale;
using rosiwit_navigation::core::SlamMode;

// 生产改动会让这些测试失败的情形:
//   * 去掉 "mapping/unknown + matching<=0 -> 1.0" 守卫 -> MappingZeroMatchingIsNotDegraded 失败
//   * 把 "localization + matching<=0" 也放行 -> LocalizationWithoutScoreDegrades 失败
//   * 把 fresh 判断写反 -> NoInfoMeansFullSpeed 失败
//   * 边界 < / <= 写错 -> AtThresholdIsHealthy 失败
TEST(LocalizationHealth, NoInfoMeansFullSpeed)
{
  EXPECT_DOUBLE_EQ(
    localizationSpeedScale(false, 0.10, 0.65, 0.5, SlamMode::Localization), 1.0);
}

TEST(LocalizationHealth, MappingZeroMatchingIsNotDegraded)
{
  // 建图模式没有定位分, 不是退化 (2026-11-01: 这里误判导致整条导航被乘 0.5)
  EXPECT_DOUBLE_EQ(
    localizationSpeedScale(true, 0.0, 0.65, 0.5, SlamMode::Mapping), 1.0);
  // 没有 slam_state (老部署 / 未接入) 也按"没有匹配语义"处理
  EXPECT_DOUBLE_EQ(
    localizationSpeedScale(true, 0.0, 0.65, 0.5, SlamMode::Unknown), 1.0);
}

TEST(LocalizationHealth, LocalizationWithoutScoreDegrades)
{
  // 定位模式下 matching 还是 0 (刚启动/重定位) -> 位姿没有匹配依据, 必须降速
  EXPECT_DOUBLE_EQ(
    localizationSpeedScale(true, 0.0, 0.65, 0.5, SlamMode::Localization), 0.5);
}

TEST(LocalizationHealth, LowMatchingDegrades)
{
  EXPECT_DOUBLE_EQ(
    localizationSpeedScale(true, 0.40, 0.65, 0.5, SlamMode::Localization), 0.5);
}

TEST(LocalizationHealth, AtThresholdIsHealthy)
{
  EXPECT_DOUBLE_EQ(
    localizationSpeedScale(true, 0.65, 0.65, 0.5, SlamMode::Localization), 1.0);
}

TEST(LocalizationHealth, GoodMatchingFullSpeed)
{
  // 实车当前状态: matching_degree ≈ 0.97
  EXPECT_DOUBLE_EQ(
    localizationSpeedScale(true, 0.97, 0.65, 0.5, SlamMode::Localization), 1.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
