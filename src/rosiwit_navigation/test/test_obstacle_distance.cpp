#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include "rosiwit_navigation/nav_core/obstacle_distance.hpp"

namespace
{
using rosiwit_navigation::core::CostmapGrid;
using rosiwit_navigation::core::ObstacleDistance;
using rosiwit_navigation::core::Pose2D;

CostmapGrid makeGrid(int w, int h, double res)
{
  CostmapGrid g;
  g.info.width = static_cast<unsigned>(w);
  g.info.height = static_cast<unsigned>(h);
  g.info.resolution = res;
  g.info.origin.position.x = 0.0;
  g.info.origin.position.y = 0.0;
  g.data.assign(static_cast<size_t>(w) * h, 0);
  return g;
}

// 生产改动会让这些测试失败的情形:
//   * EDT 单位没乘分辨率 -> DistanceToSingleCell 失败 (格 vs 米)
//   * unknown(-1) 当成自由 -> UnknownCountsAsObstacle 失败 (TEB 会贴着未知区走)
//   * 窗口外返回大数而不是 -1 -> OutsideWindowIsMinusOne 失败 (把"不知道"当成"很安全")
TEST(ObstacleDistance, EmptyGridIsNotReady)
{
  ObstacleDistance od;
  EXPECT_FALSE(od.ready());
  EXPECT_LT(od.distanceAt(0.0, 0.0), 0.0);
}

TEST(ObstacleDistance, DistanceToSingleCell)
{
  CostmapGrid g = makeGrid(20, 20, 0.1);
  g.data[10 * 20 + 10] = 100;             // (10,10) lethal, 格中心 (1.05, 1.05)
  ObstacleDistance od;
  od.update(g);
  ASSERT_TRUE(od.ready());
  EXPECT_NEAR(od.distanceAt(1.05, 1.05), 0.0, 1e-6);
  EXPECT_NEAR(od.distanceAt(1.15, 1.05), 0.1, 1e-3);   // 右邻格
  EXPECT_NEAR(od.distanceAt(1.25, 1.05), 0.2, 1e-3);   // 右第二格
}

TEST(ObstacleDistance, UnknownCountsAsObstacle)
{
  CostmapGrid g = makeGrid(10, 10, 0.1);
  g.data[5 * 10 + 5] = -1;                // unknown 也要当成障碍(保守)
  ObstacleDistance od;
  od.update(g);
  EXPECT_NEAR(od.distanceAt(0.55, 0.55), 0.0, 1e-6);
}

TEST(ObstacleDistance, OutsideWindowIsMinusOne)
{
  CostmapGrid g = makeGrid(10, 10, 0.1);
  ObstacleDistance od;
  od.update(g);
  EXPECT_LT(od.distanceAt(-1.0, 0.5), 0.0);
  EXPECT_LT(od.distanceAt(5.0, 0.5), 0.0);
}

TEST(ObstacleDistance, FootprintClearanceShrinksNearObstacle)
{
  CostmapGrid g = makeGrid(40, 20, 0.1);
  for (int y = 0; y < 20; ++y) { g.data[y * 40 + 30] = 100; }   // x=3.05 处一堵墙
  ObstacleDistance od;
  od.update(g);

  const std::vector<std::pair<double, double>> fp = {{-0.2, -0.2}, {0.2, -0.2}, {0.2, 0.2}, {-0.2, 0.2}};
  Pose2D far_pose;
  far_pose.x = 2.0;
  far_pose.y = 1.0;
  Pose2D near_pose;
  near_pose.x = 2.6;
  near_pose.y = 1.0;
  const double far_c = od.footprintClearance(far_pose, fp);
  const double near_c = od.footprintClearance(near_pose, fp);
  EXPECT_GT(far_c, near_c);
  EXPECT_GE(far_c, 0.0);

  // 足迹任一点出窗口 -> -1 (调用方据此拒绝该顶点, 而不是当成安全)
  Pose2D outside;
  outside.x = 0.1;
  outside.y = 1.0;
  EXPECT_LT(od.footprintClearance(outside, fp), 0.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
