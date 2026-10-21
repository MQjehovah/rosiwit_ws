// ============================================================================
// test_footprint.cpp — 长方形车体 footprint 代价检查的回归测试
//
// 目标: 证明“中心点安全”不等于“车体安全”.
//   * 车头角压在障碍上时, 只看中心会漏, footprintCost 必须返回致命代价
//   * 旋转后同一个世界障碍不再落在车体内
//   * 空 footprint 保持旧行为 (只看中心)
// ============================================================================
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "rosiwit_navigation/nav_core/footprint.hpp"

using namespace rosiwit_navigation::core;

namespace
{

// 实车 footprint: x[-0.25,0.55] y[-0.33,0.33]
Footprint realFootprint()
{
  return {
    {-0.25, -0.33}, {0.45, -0.33}, {0.55, -0.20},
    {0.55, 0.20}, {0.45, 0.33}, {-0.25, 0.33}};
}

CostmapGrid makeGrid()
{
  CostmapGrid grid;
  grid.info.width = 200;
  grid.info.height = 200;
  grid.info.resolution = 0.05;
  grid.info.origin.position.x = -5.0;
  grid.info.origin.position.y = -5.0;
  grid.data.assign(200 * 200, 0);
  return grid;
}

void setOccupied(CostmapGrid & grid, double x, double y, int value = 100)
{
  const int mx = static_cast<int>(std::floor(
    (x - grid.info.origin.position.x) / grid.info.resolution));
  const int my = static_cast<int>(std::floor(
    (y - grid.info.origin.position.y) / grid.info.resolution));
  ASSERT_GE(mx, 0);
  ASSERT_GE(my, 0);
  ASSERT_LT(mx, static_cast<int>(grid.info.width));
  ASSERT_LT(my, static_cast<int>(grid.info.height));
  grid.data[static_cast<size_t>(my) * grid.info.width + mx] = value;
}

Pose2D pose(double x, double y, double yaw)
{
  return Pose2D(x, y, yaw);
}

}  // namespace

TEST(Footprint, SamplesCoverEdgesAndCenter)
{
  const auto samples = sampleFootprint(realFootprint(), 0.10);
  ASSERT_FALSE(samples.empty());
  // 6 个顶点 + 边加密 + 中心
  EXPECT_GT(samples.size(), realFootprint().size());
  const bool has_center = std::any_of(
    samples.begin(), samples.end(),
    [](const auto & p) { return std::fabs(p.first) < 1e-9 && std::fabs(p.second) < 1e-9; });
  EXPECT_TRUE(has_center);
}

TEST(Footprint, EmptyOrDegenerateFootprintFallsBackToCenter)
{
  const auto samples = sampleFootprint(Footprint{}, 0.1);
  EXPECT_TRUE(samples.empty());
  EXPECT_TRUE(sampleFootprint({{0, 0}, {1, 0}}, 0.1).empty());

  CostmapGrid grid = makeGrid();
  setOccupied(grid, 0.5, 0.25);
  // 没有 footprint 采样: 只看中心, 角上的障碍不影响
  EXPECT_EQ(footprintCost(grid, pose(0, 0, 0), samples), 0);
  // 中心有障碍时才致命
  setOccupied(grid, 0.0, 0.0);
  EXPECT_EQ(footprintCost(grid, pose(0, 0, 0), samples), 100);
}

TEST(Footprint, FrontCornerObstacleIsCaught)
{
  CostmapGrid grid = makeGrid();
  const auto samples = sampleFootprint(realFootprint(), 0.05);

  // 障碍在 (0.5, 0.25): 中心 (0,0) 安全, 但落在右前角车体内
  setOccupied(grid, 0.5, 0.25);
  EXPECT_EQ(costAt(grid, 0.0, 0.0), 0);
  EXPECT_EQ(footprintCost(grid, pose(0, 0, 0), samples), 100);
}

TEST(Footprint, RotationMovesTheCornerAway)
{
  CostmapGrid grid = makeGrid();
  const auto samples = sampleFootprint(realFootprint(), 0.05);
  setOccupied(grid, 0.5, 0.25);

  // 车体逆时针转 90° 后, 该世界点不再在 footprint 内
  EXPECT_EQ(footprintCost(grid, pose(0, 0, M_PI / 2.0), samples), 0);
}

TEST(Footprint, ThinObstacleOnEdgeIsCaught)
{
  CostmapGrid grid = makeGrid();
  const auto samples = sampleFootprint(realFootprint(), 0.10);
  // 下边 y=-0.33 上、x=0.15 的一个单格薄障碍
  setOccupied(grid, 0.15, -0.33);
  EXPECT_EQ(footprintCost(grid, pose(0, 0, 0), samples), 100);
}

TEST(Footprint, TakesWorstCornerNotAverage)
{
  CostmapGrid grid = makeGrid();
  const auto samples = sampleFootprint(realFootprint(), 0.05);
  setOccupied(grid, 0.45, 0.33, 80);   // 右前角软代价
  setOccupied(grid, -0.25, 0.33, 100); // 左后角致命
  EXPECT_EQ(footprintCost(grid, pose(0, 0, 0), samples), 100);
}

TEST(Footprint, OutOfBoundsIsLethal)
{
  CostmapGrid grid = makeGrid();
  EXPECT_EQ(costAt(grid, 100.0, 100.0), 100);
  const auto samples = sampleFootprint(realFootprint(), 0.05);
  EXPECT_EQ(footprintCost(grid, pose(100.0, 100.0, 0.0), samples), 100);
}

TEST(Footprint, UnknownCellIsLethal)
{
  CostmapGrid grid = makeGrid();
  setOccupied(grid, 0.5, 0.25, -1);
  const auto samples = sampleFootprint(realFootprint(), 0.05);
  EXPECT_EQ(footprintCost(grid, pose(0, 0, 0), samples), 100);
}

TEST(Footprint, ControlPathUsesRealRobotShape)
{
  // 对比: 同样的障碍, 点模型通过, 长方形车体判定碰撞.
  CostmapGrid grid = makeGrid();
  setOccupied(grid, 1.5, 0.33);  // 车体右侧边缘会被扫到的位置

  const auto samples = sampleFootprint(realFootprint(), 0.05);
  const Pose2D straight(0.0, 0.0, 0.0);
  EXPECT_EQ(footprintCost(grid, straight, {}), 0);          // 中心线安全
  // 车体中心在 (1.5,0) 时, 右侧边缘压在障碍上
  EXPECT_EQ(footprintCost(grid, pose(1.5, 0.0, 0.0), samples), 100);
}
