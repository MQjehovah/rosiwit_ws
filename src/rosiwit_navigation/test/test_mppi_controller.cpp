#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "mppi_controller.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace
{
using rosiwit_navigation::controllers::MppiController;
using rosiwit_navigation::core::ControllerConfig;
using rosiwit_navigation::core::CostmapGrid;
using rosiwit_navigation::core::Path;
using rosiwit_navigation::core::Pose2D;
using rosiwit_navigation::core::VelocityCommand;

MppiController makeController()
{
  ControllerConfig config;
  config.max_linear_velocity = 0.5;
  config.max_angular_velocity = 1.0;
  config.max_accel_x = 0.5;
  config.max_accel_theta = 0.8;
  config.xy_goal_tolerance = 0.2;
  config.kinematics.max_velocity_x = 0.5;
  config.kinematics.max_velocity_theta = 1.0;

  MppiController controller;
  EXPECT_TRUE(controller.initialize(config));
  return controller;
}

Path makeStraightPath()
{
  Path path;
  for (double x = 0.0; x <= 2.0 + 1e-9; x += 0.1) {
    path.points.emplace_back(Pose2D(x, 0.0, 0.0));
  }
  return path;
}

CostmapGrid makeLocalGrid(int value)
{
  CostmapGrid grid;
  grid.info.width = 200;
  grid.info.height = 200;
  grid.info.resolution = 0.05;
  grid.info.origin.position.x = -5.0;
  grid.info.origin.position.y = -5.0;
  grid.data.assign(200 * 200, static_cast<int8_t>(value));
  return grid;
}

TEST(MppiController, EmptyPathIsNotGoalReached)
{
  auto controller = makeController();
  EXPECT_FALSE(controller.isGoalReached(Pose2D(0.0, 0.0, 0.0)));
  const auto command = controller.computeVelocityCommand(
    Pose2D(0.0, 0.0, 0.0), VelocityCommand(0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

TEST(MppiController, MovesForwardOnFreeStraightPath)
{
  auto controller = makeController();
  controller.setPath(makeStraightPath());
  controller.setLocalCostmap(makeLocalGrid(0));

  const auto command = controller.computeVelocityCommand(
    Pose2D(0.0, 0.0, 0.0), VelocityCommand(0.0, 0.0, 0.0));
  EXPECT_GT(command.linear_x, 0.03);
  EXPECT_LE(std::abs(command.angular_z), 1.0 + 1e-6);
}

TEST(MppiController, BlueLocalInflationIsTraversable)
{
  auto controller = makeController();
  controller.setPath(makeStraightPath());
  controller.setLocalCostmap(makeLocalGrid(50));

  const auto command = controller.computeVelocityCommand(
    Pose2D(0.0, 0.0, 0.0), VelocityCommand(0.0, 0.0, 0.0));
  EXPECT_GT(command.linear_x, 0.03);
}

TEST(MppiController, SoftBlueInflationDoesNotForceCrawl)
{
  // 0.9m 膨胀半径下, 窄走廊中心也可能有 40-60 的蓝色代价. 这些格子
  // 可通行, 不应把速度压到 0.05m/s.
  auto controller = makeController();
  controller.setPath(makeStraightPath());
  controller.setLocalCostmap(makeLocalGrid(50));

  std::vector<double> speeds;
  for (int i = 0; i < 10; ++i) {
    const auto command = controller.computeVelocityCommand(
      Pose2D(0.0, 0.0, 0.0), VelocityCommand(0.0, 0.0, 0.0));
    speeds.push_back(command.linear_x);
  }
  EXPECT_GT(speeds.back(), 0.10);
}

TEST(MppiController, RampsWhenMeasuredVelocityIsZero)
{
  // 复现仿真里 odom.twist 恒为 0 的情况: 如果只看当前速度,
  // MPPI 每个周期都被限制在 max_accel*dt, 机器人会一直 0.04m/s 爬行.
  auto controller = makeController();
  controller.setPath(makeStraightPath());
  controller.setLocalCostmap(makeLocalGrid(0));

  std::vector<double> speeds;
  for (int i = 0; i < 10; ++i) {
    const auto command = controller.computeVelocityCommand(
      Pose2D(0.0, 0.0, 0.0), VelocityCommand(0.0, 0.0, 0.0));
    speeds.push_back(command.linear_x);
  }
  ASSERT_EQ(speeds.size(), 10u);
  EXPECT_GT(speeds.back(), speeds.front() + 0.02);
  EXPECT_GT(speeds.back(), 0.10);
}

TEST(MppiController, ReportsGoalReachedAtPathEnd)
{
  auto controller = makeController();
  controller.setPath(makeStraightPath());
  EXPECT_TRUE(controller.isGoalReached(Pose2D(2.0, 0.0, 0.0)));
  EXPECT_FALSE(controller.isGoalReached(Pose2D(1.0, 0.0, 0.0)));
}

}  // namespace
