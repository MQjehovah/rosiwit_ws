// ============================================================================
// test_coverage_grid.cpp — 动态覆盖栅格的回归测试
//   重点:
//     * 覆盖是按"走过的轨迹 + 清扫宽度"累计, 不是按规划路径
//     * 静态障碍切断区域 -> 不可达区域可被标记/输出
//     * 动态障碍只临时挡路, 不直接变成永久不可达
// ============================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rosiwit_navigation/nav_core/coverage_grid.hpp"

using namespace rosiwit_navigation::core;

namespace
{

CostmapGridInfo makeInfo(int width = 200, int height = 200, double res = 0.05,
                         double ox = -5.0, double oy = -5.0)
{
  CostmapGridInfo info;
  info.width = static_cast<unsigned int>(width);
  info.height = static_cast<unsigned int>(height);
  info.resolution = res;
  info.origin.position.x = ox;
  info.origin.position.y = oy;
  return info;
}

std::vector<Pose2D> rect(double x0, double y0, double x1, double y1)
{
  return {{x0, y0, 0}, {x1, y0, 0}, {x1, y1, 0}, {x0, y1, 0}};
}

std::vector<int8_t> makeMap(const CostmapGridInfo & info)
{
  return std::vector<int8_t>(
    static_cast<size_t>(info.width) * info.height, 0);
}

void setCell(std::vector<int8_t> & data, const CostmapGridInfo & info,
             double x, double y, int8_t value)
{
  const int mx = static_cast<int>(std::floor(
    (x - info.origin.position.x) / info.resolution));
  const int my = static_cast<int>(std::floor(
    (y - info.origin.position.y) / info.resolution));
  if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) ||
      my >= static_cast<int>(info.height)) {
    return;
  }
  data[static_cast<size_t>(my) * info.width + mx] = value;
}

CoverageConfig config(double sweep = 0.5, double dilation = 0.1)
{
  CoverageConfig c;
  c.sweep_width = sweep;
  c.obstacle_dilation = dilation;
  return c;
}

}  // namespace

TEST(CoverageGrid, InitializesAreaAndRejectsBadPolygon)
{
  CoverageGrid grid;
  const auto info = makeInfo();
  EXPECT_FALSE(grid.initialize(info, {{0, 0, 0}, {1, 0, 0}}, config()));

  ASSERT_TRUE(grid.initialize(info, rect(-2, -2, 2, 2), config()));
  const CoverageStats stats = grid.stats();
  EXPECT_EQ(stats.total, 6400u);  // 4m x 4m @ 0.05
  EXPECT_EQ(stats.uncovered, 6400u);
  EXPECT_EQ(stats.covered, 0u);
  EXPECT_DOUBLE_EQ(stats.coverageRate(), 0.0);
  EXPECT_EQ(stats.outside, 40000u - 6400u);
}

TEST(CoverageGrid, CoveredIsAccumulatedFromTrajectoryAndWidth)
{
  CoverageGrid grid;
  const auto info = makeInfo();
  ASSERT_TRUE(grid.initialize(info, rect(-2, -2, 2, 2), config(0.5)));

  grid.updateCovered({-1.0, 0.0, 0.0});
  const auto after_first = grid.stats();
  EXPECT_GT(after_first.covered, 0u);
  EXPECT_LT(after_first.covered, after_first.total);

  grid.updateCovered({1.0, 0.0, 0.0});
  const auto after_line = grid.stats();
  EXPECT_GT(after_line.covered, after_first.covered);
  // 清扫宽度 0.5m, 2m 长的轨迹大约覆盖 1m^2 => 400 格
  EXPECT_GT(after_line.covered, 300u);
  EXPECT_LT(after_line.covered, 700u);
}

TEST(CoverageGrid, ObstacleWallMakesFarSideUnreachable)
{
  CoverageGrid grid;
  const auto info = makeInfo();
  ASSERT_TRUE(grid.initialize(info, rect(-2, -2, 2, 2), config(0.2)));

  auto map = makeMap(info);
  // 墙要伸到区域外, 否则栅格化后边界会留下可绕过的缝
  for (double y = -3.0; y <= 3.0; y += info.resolution) {
    setCell(map, info, 0.0, y, 100);
  }
  grid.updateStaticObstacles(map);

  const Pose2D robot(-1.0, 0.0, 0.0);
  EXPECT_TRUE(grid.hasUncoveredReachable(robot, false));
  grid.markUnreachable(robot);

  const auto stats = grid.stats();
  EXPECT_GT(stats.blocked, 2000u);  // 墙右侧约 80x80 格
  EXPECT_GT(stats.obstacle, 0u);
  const auto regions = grid.blockedRegions();
  ASSERT_FALSE(regions.empty());
  // 不可达区域应该主要在 x>0
  for (const auto & poly : regions) {
    ASSERT_EQ(poly.size(), 4u);
    EXPECT_GT((poly[0].x + poly[2].x) * 0.5, -0.1);
  }
}

TEST(CoverageGrid, DynamicObstacleBlocksTargetTemporarily)
{
  CoverageGrid grid;
  const auto info = makeInfo();
  // 区域 x[-1,1] y[-0.25,0.25]; 清扫宽度 1.5m 先把左半边盖满
  ASSERT_TRUE(grid.initialize(info, rect(-1, -0.25, 1, 0.25), config(1.5)));

  const Pose2D robot(-0.75, 0.0, 0.0);
  grid.updateCovered(robot);  // 左半边覆盖到 x≈0
  grid.updateDynamicObstacles({{0.0, -0.25, 0}, {0.0, 0.0, 0}, {0.0, 0.25, 0}});

  EXPECT_FALSE(grid.hasUncoveredReachable(robot, true));   // 动态墙挡住
  EXPECT_TRUE(grid.hasUncoveredReachable(robot, false));   // 忽略动态仍可达
  Pose2D target;
  EXPECT_FALSE(grid.nearestUncovered(robot, target, true));
  // 动态障碍消失后又能过去
  grid.updateDynamicObstacles({});
  EXPECT_TRUE(grid.nearestUncovered(robot, target, true));
}

TEST(CoverageGrid, MarkAllUncoveredBlockedAndEncode)
{
  CoverageGrid grid;
  const auto info = makeInfo();  // 5m x 5m
  ASSERT_TRUE(grid.initialize(info, rect(-1, -1, 1, 1), config(0.5)));
  grid.updateCovered({0.0, 0.0, 0.0});
  grid.markAllUncoveredBlocked();

  const auto stats = grid.stats();
  EXPECT_EQ(stats.uncovered, 0u);
  EXPECT_GT(stats.blocked, 0u);

  const auto encoded = grid.encode();
  ASSERT_EQ(encoded.size(), static_cast<size_t>(info.width) * info.height);
  bool has_covered = false;
  bool has_blocked = false;
  bool has_outside = false;
  for (const auto v : encoded) {
    if (v == 50) { has_covered = true; }
    if (v == 80) { has_blocked = true; }
    if (v == -1) { has_outside = true; }
  }
  EXPECT_TRUE(has_covered);
  EXPECT_TRUE(has_blocked);
  EXPECT_TRUE(has_outside);
}

TEST(CoverageGrid, FarthestTargetBeatsAdjacentCell)
{
  CoverageGrid grid;
  const auto info = makeInfo();
  ASSERT_TRUE(grid.initialize(info, rect(-2, -2, 2, 2), config(0.3)));
  grid.updateCovered({-1.5, 0.0, 0.0});

  Pose2D nearest;
  Pose2D farthest;
  ASSERT_TRUE(grid.nearestUncovered({-1.5, 0.0, 0.0}, nearest, false));
  ASSERT_TRUE(grid.farthestUncovered({-1.5, 0.0, 0.0}, farthest, 2.5, false));
  const double near_d = std::hypot(nearest.x + 1.5, nearest.y);
  const double far_d = std::hypot(farthest.x + 1.5, farthest.y);
  EXPECT_GT(far_d, near_d + 0.5);
  EXPECT_LE(far_d, 2.6);
}

TEST(CoverageGrid, NextCoverageSegmentIsARowNearTheRobot)
{
  CoverageGrid grid;
  const auto info = makeInfo();
  ASSERT_TRUE(grid.initialize(info, rect(-2, -2, 2, 2), config(0.5)));
  grid.updateCovered({-1.5, 0.0, 0.0});

  std::vector<Pose2D> segment;
  ASSERT_TRUE(grid.nextCoverageSegment({-1.5, 0.0, 0.0}, 2.5, segment));
  ASSERT_GE(segment.size(), 2u);
  EXPECT_LE(std::fabs(segment.front().y), 0.1);
  EXPECT_LE(std::fabs(segment.back().y), 0.1);
  double length = 0.0;
  for (size_t i = 1; i < segment.size(); ++i) {
    length += segment[i].distanceTo(segment[i - 1]);
  }
  EXPECT_GT(length, 1.0);
  EXPECT_LE(length, 2.6);
  EXPECT_LE(std::fabs(segment.front().x), 2.0);
  EXPECT_LE(std::fabs(segment.back().x), 2.0);
}

TEST(CoverageGrid, PlannerCostmapTreatsAreaBoundaryAsHard)
{
  CoverageGrid grid;
  const auto info = makeInfo();
  ASSERT_TRUE(grid.initialize(info, rect(-1, -1, 1, 1), config(0.5)));
  auto map = makeMap(info);
  // 区域外的一格墙 (x=2.5), 区域内全部自由
  setCell(map, info, 2.5, 0.0, 100);
  grid.updateStaticObstacles(map);

  const auto plan = grid.plannerCostmap(map);
  ASSERT_EQ(plan.size(), static_cast<size_t>(info.width) * info.height);
  // 区域外对车体中心是硬边界 (工具可以伸出去, 中心不能出去)
  const int outer_x = static_cast<int>(std::floor((2.0 - info.origin.position.x) / info.resolution));
  const int outer_y = static_cast<int>(std::floor((0.0 - info.origin.position.y) / info.resolution));
  EXPECT_EQ(plan[static_cast<size_t>(outer_y) * info.width + outer_x], 100);
  // 区域外的墙仍然致命
  const int wall_x = static_cast<int>(std::floor((2.5 - info.origin.position.x) / info.resolution));
  EXPECT_EQ(plan[static_cast<size_t>(outer_y) * info.width + wall_x], 100);
  // 区域内的障碍也致命
  auto map2 = map;
  setCell(map2, info, 0.0, 0.0, 100);
  grid.updateStaticObstacles(map2);
  const auto plan2 = grid.plannerCostmap(map2);
  const int cx = static_cast<int>(std::floor((0.0 - info.origin.position.x) / info.resolution));
  const int cy = static_cast<int>(std::floor((0.0 - info.origin.position.y) / info.resolution));
  EXPECT_EQ(plan2[static_cast<size_t>(cy) * info.width + cx], 100);
}

TEST(CoverageGrid, NearestTargetIsInsideAreaAndUncovered)
{
  CoverageGrid grid;
  const auto info = makeInfo();
  ASSERT_TRUE(grid.initialize(info, rect(-2, -2, 2, 2), config(0.5)));
  grid.updateCovered({0.0, 0.0, 0.0});

  Pose2D target;
  ASSERT_TRUE(grid.nearestUncovered({0.0, 0.0, 0.0}, target, false));
  EXPECT_LE(std::fabs(target.x), 2.0);
  EXPECT_LE(std::fabs(target.y), 2.0);
  // 目标格子不应该在已覆盖圆盘中心
  EXPECT_GT(std::hypot(target.x, target.y), 0.2);
}
