#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "path_smoother.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace
{
using rosiwit_navigation::core::CostmapGrid;
using rosiwit_navigation::core::Path;
using rosiwit_navigation::core::PathPoint;
using rosiwit_navigation::core::Pose2D;
using rosiwit_navigation::planners::SmootherConfig;
using rosiwit_navigation::planners::smoothPath;

std::shared_ptr<CostmapGrid> makeGrid(
  unsigned int width, unsigned int height, double resolution,
  const std::vector<int8_t> & cells)
{
  auto grid = std::make_shared<CostmapGrid>();
  grid->info.width = width;
  grid->info.height = height;
  grid->info.resolution = resolution;
  grid->info.origin.position.x = 0.0;
  grid->info.origin.position.y = 0.0;
  grid->data = cells;
  return grid;
}

Path makePath(const std::vector<Pose2D> & poses)
{
  Path path;
  for (const auto & pose : poses) {
    path.points.emplace_back(pose);
  }
  return path;
}

int cellCost(const CostmapGrid & grid, double x, double y)
{
  const int mx = static_cast<int>(
    (x - grid.info.origin.position.x) / grid.info.resolution);
  const int my = static_cast<int>(
    (y - grid.info.origin.position.y) / grid.info.resolution);
  if (mx < 0 || mx >= static_cast<int>(grid.info.width) ||
      my < 0 || my >= static_cast<int>(grid.info.height)) {
    return 100;
  }
  return grid.data[static_cast<size_t>(my) * grid.info.width + mx];
}

int maxPathCost(const CostmapGrid & grid, const Path & path)
{
  if (path.points.empty()) return 0;
  int max_cost = cellCost(grid, path.points.front().pose.x, path.points.front().pose.y);
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & a = path.points[i - 1].pose;
    const auto & b = path.points[i].pose;
    const double distance = std::hypot(b.x - a.x, b.y - a.y);
    const int steps = std::max(1, static_cast<int>(
      std::ceil(distance / (grid.info.resolution * 0.5))));
    for (int step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / steps;
      max_cost = std::max(max_cost, cellCost(
        grid, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
    }
  }
  return max_cost;
}

Path makeUCourse()
{
  // U 形绕行: 中间是障碍/蓝色区域, 原始路径在外侧自由区.
  return makePath({
    Pose2D(0.5, 0.5, 0.0),
    Pose2D(0.5, 2.5, 0.0),
    Pose2D(2.5, 2.5, 0.0),
    Pose2D(2.5, 0.5, 0.0),
  });
}

TEST(PathSmoother, KeepsSegmentsOutOfLethalCells)
{
  const unsigned int width = 60;
  const unsigned int height = 40;
  std::vector<int8_t> cells(width * height, 0);
  for (unsigned int y = 0; y <= 20; ++y) {
    for (unsigned int x = 10; x <= 12; ++x) {
      cells[y * width + x] = 100;
    }
  }
  auto grid = makeGrid(width, height, 0.1, cells);
  const Path raw = makeUCourse();

  const Path smoothed = smoothPath(raw, grid, SmootherConfig());
  ASSERT_EQ(smoothed.points.size(), raw.points.size());
  EXPECT_LT(maxPathCost(*grid, smoothed), 100);
  EXPECT_DOUBLE_EQ(smoothed.points.front().pose.x, raw.points.front().pose.x);
  EXPECT_DOUBLE_EQ(smoothed.points.front().pose.y, raw.points.front().pose.y);
  EXPECT_DOUBLE_EQ(smoothed.points.back().pose.x, raw.points.back().pose.x);
  EXPECT_DOUBLE_EQ(smoothed.points.back().pose.y, raw.points.back().pose.y);
}

TEST(PathSmoother, DoesNotCutThroughHigherCostBlueArea)
{
  const unsigned int width = 60;
  const unsigned int height = 40;
  std::vector<int8_t> cells(width * height, 0);
  for (unsigned int y = 0; y <= 20; ++y) {
    for (unsigned int x = 10; x <= 12; ++x) {
      cells[y * width + x] = 50;
    }
  }
  auto grid = makeGrid(width, height, 0.1, cells);
  const Path raw = makeUCourse();
  ASSERT_EQ(maxPathCost(*grid, raw), 0);

  const Path smoothed = smoothPath(raw, grid, SmootherConfig());
  EXPECT_EQ(maxPathCost(*grid, smoothed), 0);
}

TEST(PathSmoother, ShortPathIsReturnedUnchanged)
{
  auto grid = makeGrid(20, 20, 0.1, std::vector<int8_t>(400, 0));
  const Path raw = makePath({Pose2D(0.5, 0.5, 0.0), Pose2D(1.5, 0.5, 0.0)});

  const Path smoothed = smoothPath(raw, grid, SmootherConfig());
  ASSERT_EQ(smoothed.points.size(), 2u);
  EXPECT_DOUBLE_EQ(smoothed.points[0].pose.x, 0.5);
  EXPECT_DOUBLE_EQ(smoothed.points[1].pose.x, 1.5);
}

}  // namespace
