#include <gtest/gtest.h>

#include <cmath>

#include "rosiwit_navigation/nav_core/teb_types.hpp"

namespace
{
using rosiwit_navigation::core::EdgeAccumulator;
using rosiwit_navigation::core::edgeAccel;
using rosiwit_navigation::core::edgeKinematics;
using rosiwit_navigation::core::edgeObstacle;
using rosiwit_navigation::core::edgePath;
using rosiwit_navigation::core::edgeTime;
using rosiwit_navigation::core::edgeVelocity;
using rosiwit_navigation::core::TebVertex;
using rosiwit_navigation::core::TebWeights;

rosiwit_navigation::core::Path straightPath()
{
  rosiwit_navigation::core::Path p;
  for (int i = 0; i <= 10; ++i) {
    rosiwit_navigation::core::Pose2D q;
    q.x = 0.5 * i;   // 沿 x 轴 0..5 m
    q.y = 0.0;
    q.theta = 0.0;
    rosiwit_navigation::core::PathPoint pp(q);
    pp.velocity_x = 1.0;
    p.points.push_back(pp);
  }
  return p;
}

TebWeights weights()
{
  TebWeights w;
  w.path = 1.0;
  w.obstacle = 8.0;
  w.kinematics = 6.0;
  w.velocity = 4.0;
  w.accel = 3.0;
  w.time = 1.0;
  w.shortest = 1.0;
  w.min_obstacle_dist = 0.12;
  w.inflation_dist = 0.30;
  w.max_deviation = 0.60;
  return w;
}

// 生产改动会让这些测试失败的情形:
//   * 路径代价符号错 (把"偏离"算成奖励) -> PathCostZeroOnPath / PathCostGrowsWithOffset 失败
//   * 障碍代价阈值写反 -> ObstacleCostZeroBeyondMinDist 失败
//   * 障碍梯度方向反 -> ObstacleGradientPointsTowardObstacle 失败
//   * 运动学残差用错角度 -> KinematicsZeroWhenStraight 失败
TEST(TebTypes, PathCostZeroOnPath)
{
  const auto p = straightPath();
  TebVertex v{2.0, 0.0, 0.0, 0.3};
  const EdgeAccumulator e = edgePath(v, p, weights().path);
  EXPECT_NEAR(e.cost, 0.0, 1e-9);
  EXPECT_NEAR(e.grad[0], 0.0, 1e-6);
  EXPECT_NEAR(e.grad[1], 0.0, 1e-6);
}

TEST(TebTypes, PathCostGrowsWithOffset)
{
  const auto p = straightPath();
  const TebWeights w = weights();
  TebVertex near_v{2.0, 0.1, 0.0, 0.3};
  TebVertex far_v{2.0, 0.4, 0.0, 0.3};
  const double c_near = edgePath(near_v, p, w.path).cost;
  const double c_far = edgePath(far_v, p, w.path).cost;
  EXPECT_GT(c_near, 0.0);
  EXPECT_GT(c_far, c_near * 4.0);   // 二次: 0.4^2 / 0.1^2 = 16
  // 梯度指向"回到路径"的方向: v 在 +y 偏移, 代价对 y 的梯度应为正
  const EdgeAccumulator e = edgePath(far_v, p, w.path);
  EXPECT_GT(e.grad[1], 0.0);
}

TEST(TebTypes, ObstacleCostZeroBeyondMinDist)
{
  const TebWeights w = weights();
  TebVertex v{0.0, 0.0, 0.0, 0.3};
  EXPECT_NEAR(edgeObstacle(v, 0.50, 1.0, 0.0, w).cost, 0.0, 1e-12);
  EXPECT_NEAR(edgeObstacle(v, 0.30, 1.0, 0.0, w).cost, 0.0, 1e-12);
  EXPECT_GT(edgeObstacle(v, 0.10, 1.0, 0.0, w).cost, 0.0);
}

TEST(TebTypes, ObstacleCostMonotonicInDistance)
{
  const TebWeights w = weights();
  TebVertex v{0.0, 0.0, 0.0, 0.3};
  const double c1 = edgeObstacle(v, 0.02, 1.0, 0.0, w).cost;
  const double c2 = edgeObstacle(v, 0.08, 1.0, 0.0, w).cost;
  EXPECT_GT(c1, c2);
}

TEST(TebTypes, ObstacleGradientPointsTowardObstacle)
{
  // 代价随"靠近障碍"上升, 所以对位置的梯度应指向障碍方向
  const TebWeights w = weights();
  TebVertex v{0.0, 0.0, 0.0, 0.3};
  const EdgeAccumulator e = edgeObstacle(v, 0.05, 0.0, 1.0, w);   // 障碍在 +y
  EXPECT_GT(e.grad[1], 0.0);
  EXPECT_NEAR(e.grad[0], 0.0, 1e-9);
}

TEST(TebTypes, KinematicsZeroWhenStraight)
{
  const TebWeights w = weights();
  TebVertex a{0.0, 0.0, 0.0, 0.3};
  TebVertex b{0.3, 0.0, 0.0, 0.3};
  EXPECT_NEAR(edgeKinematics(a, b, w.kinematics).cost, 0.0, 1e-9);
}

TEST(TebTypes, KinematicsPenalisesSidewaysAndTurn)
{
  const TebWeights w = weights();
  TebVertex a{0.0, 0.0, 0.0, 0.3};
  TebVertex sideways{0.0, 0.3, 0.0, 0.3};   // 朝 +x 却横向移动
  EXPECT_GT(edgeKinematics(a, sideways, w.kinematics).cost, 0.0);
  TebVertex turned{0.3, 0.0, 1.0, 0.3};      // 位置向前但朝向偏了 1 rad
  EXPECT_GT(edgeKinematics(a, turned, w.kinematics).cost, 0.0);
}

TEST(TebTypes, TimeCostIsWeightedDt)
{
  TebWeights w = weights();
  w.time = 2.5;
  TebVertex v{0.0, 0.0, 0.0, 0.4};
  EXPECT_NEAR(edgeTime(v, w.time).cost, 1.0, 1e-9);
}

TEST(TebTypes, VelocityZeroWithinLimit)
{
  const TebWeights w = weights();
  TebVertex a{0.0, 0.0, 0.0, 0.5};
  TebVertex b{0.2, 0.0, 0.0, 0.5};           // 0.2m / 0.5s = 0.4 m/s
  EXPECT_NEAR(edgeVelocity(a, b, w, 0.8).cost, 0.0, 1e-9);
  TebVertex c{0.9, 0.0, 0.0, 0.5};           // 0.9 m 位移 > 0.8 上限
  EXPECT_GT(edgeVelocity(a, c, w, 0.8).cost, 0.0);
}

TEST(TebTypes, AccelZeroWithinLimit)
{
  const TebWeights w = weights();
  TebVertex a{0.0, 0.0, 0.0, 0.3};
  TebVertex b{0.1, 0.0, 0.0, 0.3};           // v=0.33
  TebVertex c{0.2, 0.0, 0.0, 0.3};           // v=0.33, dv=0
  EXPECT_NEAR(edgeAccel(a, b, c, w, 0.8).cost, 0.0, 1e-9);
  TebVertex c_fast{1.5, 0.0, 0.0, 0.1};      // 位移增量 1.3 > 0.8 上限
  EXPECT_GT(edgeAccel(a, b, c_fast, w, 0.8).cost, 0.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
