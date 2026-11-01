#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "rosiwit_navigation/nav_core/types.hpp"
#include "teb_controller.hpp"

namespace
{
using rosiwit_navigation::controllers::TebController;
using rosiwit_navigation::core::ControllerConfig;
using rosiwit_navigation::core::CostmapGrid;
using rosiwit_navigation::core::Path;
using rosiwit_navigation::core::PathPoint;
using rosiwit_navigation::core::Pose2D;
using rosiwit_navigation::core::VelocityCommand;

ControllerConfig makeConfig()
{
  ControllerConfig cfg;
  cfg.max_linear_velocity = 0.8;
  cfg.max_angular_velocity = 0.8;
  cfg.max_accel_x = 0.8;
  cfg.xy_goal_tolerance = 0.15;
  cfg.footprint = {{-0.25, -0.20}, {0.25, -0.20}, {0.25, 0.20}, {-0.25, 0.20}};
  cfg.footprint_sample_spacing = 0.05;
  return cfg;
}

Path straightPath(double length, double dir, double y = 0.0)
{
  Path p;
  const int n = 20;
  for (int i = 0; i <= n; ++i) {
    Pose2D q;
    q.x = length * static_cast<double>(i) / n;
    q.y = y;
    q.theta = 0.0;
    PathPoint pp(q);
    pp.velocity_x = dir;
    p.points.push_back(pp);
  }
  return p;
}

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
//   * 起点没锚到当前位姿 -> StraightPathMovesForward 拿不到正向速度
//   * 全不可行时仍输出速度 -> BlockedWindowStops 失败 (会撞)
//   * 倒车方向标记被忽略 -> ReversePathMovesBackward 失败
TEST(TebController, MovesForwardAlongStraightPath)
{
  TebController c;
  ASSERT_TRUE(c.initialize(makeConfig()));
  c.setPath(straightPath(3.0, 1.0));
  const VelocityCommand cmd = c.computeVelocityCommand(Pose2D(0.0, 0.0, 0.0), VelocityCommand());
  EXPECT_GT(cmd.linear_x, 0.0);
  EXPECT_LE(cmd.linear_x, 0.8 + 1e-6);
  EXPECT_TRUE(std::isfinite(cmd.angular_z));
  EXPECT_NE(c.getDiagnostics().find("infeasible=0"), std::string::npos)
    << c.getDiagnostics();
}

TEST(TebController, MovesBackwardWhenPathFlagIsNegative)
{
  TebController c;
  ASSERT_TRUE(c.initialize(makeConfig()));
  c.setPath(straightPath(2.0, -1.0));
  // 车身朝 +x, 路径向 +x 延伸但标记倒车
  const VelocityCommand cmd = c.computeVelocityCommand(Pose2D(0.0, 0.0, 0.0), VelocityCommand());
  EXPECT_LT(cmd.linear_x, 0.0);
}

TEST(TebController, StopsWhenWholeWindowIsLethal)
{
  TebController c;
  ASSERT_TRUE(c.initialize(makeConfig()));
  CostmapGrid g = makeGrid(120, 120, 0.1);   // 12m x 12m, 全 lethal
  for (auto & v : g.data) { v = 100; }
  c.setLocalCostmap(g);
  c.setPath(straightPath(3.0, 1.0, 2.0));
  const VelocityCommand cmd = c.computeVelocityCommand(Pose2D(0.5, 2.0, 0.0), VelocityCommand());
  EXPECT_NEAR(cmd.linear_x, 0.0, 1e-9);
  EXPECT_NEAR(cmd.angular_z, 0.0, 1e-9);
  EXPECT_NE(c.getDiagnostics().find("infeasible=1"), std::string::npos)
    << c.getDiagnostics();
}

TEST(TebController, KeepsMovingWithObstacleNearPath)
{
  TebController c;
  ASSERT_TRUE(c.initialize(makeConfig()));
  CostmapGrid g = makeGrid(120, 60, 0.1);   // 12m x 6m, 机器人 y=1.0
  // 在 (1.5m, 1.0m) 附近放一个 0.3m 的障碍块, 挡在路径正中
  for (int iy = 8; iy <= 12; ++iy) {
    for (int ix = 14; ix <= 16; ++ix) { g.data[iy * 120 + ix] = 100; }
  }
  c.setLocalCostmap(g);
  c.setPath(straightPath(3.0, 1.0));
  const VelocityCommand cmd = c.computeVelocityCommand(Pose2D(0.5, 1.0, 0.0), VelocityCommand());
  // 有障碍时仍应给出可行解 (弹性带绕开), 不能直接判死
  EXPECT_NE(c.getDiagnostics().find("infeasible=0"), std::string::npos)
    << c.getDiagnostics();
  EXPECT_GT(cmd.linear_x, 0.0);
}

TEST(TebController, WaypointIndexStaysInsidePath)
{
  TebController c;
  ASSERT_TRUE(c.initialize(makeConfig()));
  const Path p = straightPath(3.0, 1.0);
  c.setPath(p);
  for (int k = 0; k < 5; ++k) {
    c.computeVelocityCommand(Pose2D(0.1 * k, 0.0, 0.0), VelocityCommand());
    EXPECT_LT(c.getCurrentWaypointIndex(), p.points.size());
    const double prog = c.getProgress();
    EXPECT_GE(prog, 0.0);
    EXPECT_LE(prog, 1.0);
  }
}

TEST(TebController, ReachesGoalWithinTolerance)
{
  TebController c;
  ASSERT_TRUE(c.initialize(makeConfig()));
  c.setPath(straightPath(1.0, 1.0));
  EXPECT_TRUE(c.isGoalReached(Pose2D(0.95, 0.0, 0.0)));
  EXPECT_FALSE(c.isGoalReached(Pose2D(0.0, 0.0, 0.0)));
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
