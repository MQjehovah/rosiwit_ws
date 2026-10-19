#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "astar_planner.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace
{
using rosiwit_navigation::core::Costmap;
using rosiwit_navigation::core::CostmapGrid;
using rosiwit_navigation::core::Path;
using rosiwit_navigation::core::Pose2D;
using rosiwit_navigation::planners::AStarPlanner;

Costmap makeCostmap(
  unsigned int width, unsigned int height, double resolution,
  const std::vector<unsigned char> & cells)
{
  auto grid = std::make_shared<CostmapGrid>();
  grid->info.width = width;
  grid->info.height = height;
  grid->info.resolution = resolution;
  grid->info.origin.position.x = 0.0;
  grid->info.origin.position.y = 0.0;
  grid->data.assign(cells.begin(), cells.end());

  Costmap costmap;
  costmap.grid = grid;
  costmap.width = width;
  costmap.height = height;
  costmap.resolution = resolution;
  costmap.origin_x = 0.0;
  costmap.origin_y = 0.0;
  costmap.data = cells;
  return costmap;
}

int costAt(const Costmap & costmap, double x, double y)
{
  const int mx = static_cast<int>((x - costmap.origin_x) / costmap.resolution);
  const int my = static_cast<int>((y - costmap.origin_y) / costmap.resolution);
  if (mx < 0 || mx >= static_cast<int>(costmap.width) ||
      my < 0 || my >= static_cast<int>(costmap.height)) {
    return 255;
  }
  return costmap.data[static_cast<size_t>(my) * costmap.width + mx];
}

bool pathTouchesLethal(const Costmap & costmap, const Path & path)
{
  for (const auto & point : path.points) {
    if (costAt(costmap, point.pose.x, point.pose.y) >= 254) {
      return true;
    }
  }
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & a = path.points[i - 1].pose;
    const auto & b = path.points[i].pose;
    const double distance = std::hypot(b.x - a.x, b.y - a.y);
    const int steps = std::max(1, static_cast<int>(
      std::ceil(distance / (costmap.resolution * 0.5))));
    for (int step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / steps;
      if (costAt(costmap, a.x + (b.x - a.x) * t,
          a.y + (b.y - a.y) * t) >= 254) {
        return true;
      }
    }
  }
  return false;
}

int maxCostOnPath(const Costmap & costmap, const Path & path)
{
  int max_cost = 0;
  for (const auto & point : path.points) {
    max_cost = std::max(max_cost, costAt(costmap, point.pose.x, point.pose.y));
  }
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & a = path.points[i - 1].pose;
    const auto & b = path.points[i].pose;
    const double distance = std::hypot(b.x - a.x, b.y - a.y);
    const int steps = std::max(1, static_cast<int>(
      std::ceil(distance / (costmap.resolution * 0.5))));
    for (int step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / steps;
      max_cost = std::max(max_cost, costAt(costmap,
        a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
    }
  }
  return max_cost;
}

AStarPlanner makePlanner(const Costmap & costmap)
{
  AStarPlanner planner;
  rosiwit_navigation::core::PlannerConfig config;
  config.name = "astar_test";
  config.resolution = costmap.resolution;
  config.max_iterations = 50000;
  config.timeout = 2.0;
  EXPECT_TRUE(planner.initialize(config));
  planner.setInflationRadius(0.0);
  planner.setRobotRadius(0.0);
  planner.setCostmap(costmap);
  return planner;
}

TEST(AStarPlanner, AvoidsLethalCellsAndUsesGap)
{
  const unsigned int width = 40;
  const unsigned int height = 40;
  std::vector<unsigned char> cells(width * height, 0);
  for (unsigned int y = 0; y <= 30; ++y) {
    cells[y * width + 20] = 255;
  }
  const Costmap costmap = makeCostmap(width, height, 0.1, cells);
  auto planner = makePlanner(costmap);

  const auto result = planner.plan(Pose2D(0.5, 0.5, 0.0), Pose2D(3.5, 0.5, 0.0));
  ASSERT_TRUE(result.is_ok()) << result.error_message();
  EXPECT_FALSE(pathTouchesLethal(costmap, result.value()));
}

TEST(AStarPlanner, PrefersZeroCostOverBlueDetour)
{
  const unsigned int width = 60;
  const unsigned int height = 40;
  std::vector<unsigned char> cells(width * height, 0);

  // 27*16 的蓝色软障碍, 直线会穿过它; 上方有完整自由空间.
  for (unsigned int y = 0; y <= 15; ++y) {
    for (unsigned int x = 20; x <= 40; ++x) {
      cells[y * width + x] = 128;
    }
  }
  const Costmap costmap = makeCostmap(width, height, 0.1, cells);
  auto planner = makePlanner(costmap);

  const auto result = planner.plan(Pose2D(0.5, 0.5, 0.0), Pose2D(5.5, 0.5, 0.0));
  ASSERT_TRUE(result.is_ok()) << result.error_message();
  ASSERT_FALSE(result.value().points.empty());

  for (const auto & point : result.value().points) {
    EXPECT_EQ(costAt(costmap, point.pose.x, point.pose.y), 0)
      << "path point entered blue inflation at "
      << point.pose.x << ", " << point.pose.y;
  }
}

TEST(AStarPlanner, PlansThroughOneCellGap)
{
  const unsigned int width = 40;
  const unsigned int height = 40;
  std::vector<unsigned char> cells(width * height, 0);
  for (unsigned int y = 0; y < height; ++y) {
    cells[y * width + 20] = 255;
  }
  cells[20 * width + 20] = 0;
  const Costmap costmap = makeCostmap(width, height, 0.1, cells);
  auto planner = makePlanner(costmap);

  const auto result = planner.plan(Pose2D(0.5, 0.5, 0.0), Pose2D(3.5, 0.5, 0.0));
  ASSERT_TRUE(result.is_ok()) << result.error_message();
  EXPECT_FALSE(pathTouchesLethal(costmap, result.value()));
}

TEST(AStarPlanner, BlueIsTraversableWhenNoFreeRouteExists)
{
  const unsigned int width = 40;
  const unsigned int height = 20;
  std::vector<unsigned char> cells(width * height, 0);
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 10; x <= 30; ++x) {
      cells[y * width + x] = 50;
    }
  }
  const Costmap costmap = makeCostmap(width, height, 0.1, cells);
  auto planner = makePlanner(costmap);

  const auto result = planner.plan(Pose2D(0.5, 0.5, 0.0), Pose2D(3.5, 0.5, 0.0));
  ASSERT_TRUE(result.is_ok()) << result.error_message();
  ASSERT_FALSE(result.value().points.empty());
  EXPECT_GT(maxCostOnPath(costmap, result.value()), 0);
  EXPECT_FALSE(pathTouchesLethal(costmap, result.value()));
}

TEST(AStarPlanner, AppliesConfiguredIterationLimit)
{
  const unsigned int width = 40;
  const unsigned int height = 40;
  std::vector<unsigned char> cells(width * height, 0);
  const Costmap costmap = makeCostmap(width, height, 0.1, cells);

  AStarPlanner planner;
  rosiwit_navigation::core::PlannerConfig config;
  config.name = "astar_iteration_test";
  config.resolution = 0.1;
  config.max_iterations = 1;
  config.timeout = 2.0;
  ASSERT_TRUE(planner.initialize(config));
  planner.setInflationRadius(0.0);
  planner.setRobotRadius(0.0);
  planner.setCostmap(costmap);

  const auto limited = planner.plan(
    Pose2D(0.5, 0.5, 0.0), Pose2D(3.5, 3.5, 0.0));
  EXPECT_FALSE(limited.is_ok());
  EXPECT_EQ(limited.error_code(),
            rosiwit_navigation::core::ErrorCode::TIMEOUT);

  // 同一个 planner 用正常上限重新初始化, 应能恢复并完成规划;
  // 这也覆盖失败/超时后搜索状态必须被清理.
  config.max_iterations = 50000;
  ASSERT_TRUE(planner.initialize(config));
  const auto normal = planner.plan(
    Pose2D(0.5, 0.5, 0.0), Pose2D(3.5, 3.5, 0.0));
  ASSERT_TRUE(normal.is_ok()) << normal.error_message();
}

}  // namespace
