#include <gtest/gtest.h>

#include "rosiwit_navigation/nav_core/navigation_factory.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

using namespace rosiwit_navigation;

namespace
{
core::Costmap makeMap(int w, int h, double res)
{
  core::Costmap cm;
  cm.grid = std::make_shared<core::CostmapGrid>();
  cm.grid->info.width = w;
  cm.grid->info.height = h;
  cm.grid->info.resolution = res;
  cm.grid->info.origin.position.x = 0.0;
  cm.grid->info.origin.position.y = 0.0;
  cm.grid->data.assign(static_cast<size_t>(w) * h, 0);
  cm.width = w;
  cm.height = h;
  cm.resolution = res;
  cm.origin_x = 0.0;
  cm.origin_y = 0.0;
  cm.data.assign(static_cast<size_t>(w) * h, 0);
  return cm;
}
}  // namespace

TEST(HybridAStar, OpenSpacePlan)
{
  auto planner = nav_core::NavigationFactory::createPlanner("hybrid_astar");
  ASSERT_NE(planner, nullptr);
  core::PlannerConfig cfg;
  cfg.name = "hybrid_astar";
  cfg.resolution = 0.05;
  cfg.max_iterations = 100000;
  cfg.timeout = 2.0;
  ASSERT_TRUE(planner->initialize(cfg));

  auto cm = makeMap(200, 200, 0.05);
  planner->setCostmap(cm);
  planner->setRobotRadius(0.20);
  const auto result = planner->plan({0.5, 0.5, 0.0}, {7.5, 7.5, 1.0});
  ASSERT_TRUE(result.is_ok()) << result.error_message();
  EXPECT_GE(result.value().points.size(), 2u);
  EXPECT_NEAR(result.value().points.front().pose.x, 0.5, 0.2);
  EXPECT_NEAR(result.value().points.back().pose.x, 7.5, 0.2);
}

TEST(HybridAStar, AvoidsWallThroughGap)
{
  auto planner = nav_core::NavigationFactory::createPlanner("hybrid_astar");
  ASSERT_NE(planner, nullptr);
  core::PlannerConfig cfg;
  cfg.name = "hybrid_astar";
  cfg.resolution = 0.05;
  cfg.max_iterations = 200000;
  cfg.timeout = 15.0;
  ASSERT_TRUE(planner->initialize(cfg));

  auto cm = makeMap(240, 200, 0.05);
  // 竖墙 x=6m，上方留 6m 通道；通道足够宽，测试关注算法本身而不是极限掉头。
  const int wall_x = 120;
  for (int y = 0; y < 80; ++y) {
    cm.grid->data[static_cast<size_t>(y) * 240 + wall_x] = 100;
    cm.data[static_cast<size_t>(y) * 240 + wall_x] = 255;
  }
  planner->setCostmap(cm);
  planner->setRobotRadius(0.18);

  const auto result = planner->plan({0.5, 0.5, 0.0}, {7.0, 5.0, 0.0});
  ASSERT_TRUE(result.is_ok()) << result.error_message();
  bool crossed = false;
  for (const auto & pt : result.value().points) {
    if (pt.pose.x > 6.0) crossed = true;
  }
  EXPECT_TRUE(crossed);
}

TEST(HybridAStar, RejectsGoalInObstacle)
{
  auto planner = nav_core::NavigationFactory::createPlanner("hybrid_astar");
  ASSERT_NE(planner, nullptr);
  core::PlannerConfig cfg;
  cfg.name = "hybrid_astar";
  cfg.resolution = 0.05;
  ASSERT_TRUE(planner->initialize(cfg));
  auto cm = makeMap(100, 100, 0.05);
  cm.grid->data[50 * 100 + 50] = 100;
  cm.data[50 * 100 + 50] = 255;
  planner->setCostmap(cm);
  const auto result = planner->plan({0.5, 0.5, 0.0}, {2.5, 2.5, 0.0});
  EXPECT_TRUE(result.is_error());
}

