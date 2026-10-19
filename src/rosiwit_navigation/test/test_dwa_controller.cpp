#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "dwa_controller.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace
{
using rosiwit_navigation::controllers::DwaController;
using rosiwit_navigation::core::ControllerConfig;
using rosiwit_navigation::core::CostmapGrid;
using rosiwit_navigation::core::Path;
using rosiwit_navigation::core::PathPoint;
using rosiwit_navigation::core::Pose2D;
using rosiwit_navigation::core::VelocityCommand;

DwaController makeController()
{
  ControllerConfig config;
  config.max_linear_velocity = 0.5;
  config.max_angular_velocity = 1.0;
  config.xy_goal_tolerance = 0.2;
  config.kinematics.max_velocity_x = 0.5;
  config.kinematics.max_velocity_theta = 1.0;

  DwaController controller;
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

TEST(DwaController, EmptyPathIsNotGoalReached)
{
  auto controller = makeController();
  EXPECT_FALSE(controller.isGoalReached(Pose2D(0.0, 0.0, 0.0)));

  const auto command = controller.computeVelocityCommand(
    Pose2D(0.0, 0.0, 0.0), VelocityCommand(0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

TEST(DwaController, BlueLocalInflationIsTraversable)
{
  auto controller = makeController();
  controller.setPath(makeStraightPath());
  // 50 = 蓝色软膨胀, 不是碰撞; 机器人必须还能往前走.
  controller.setLocalCostmap(makeLocalGrid(50));

  const auto command = controller.computeVelocityCommand(
    Pose2D(0.0, 0.0, 0.0), VelocityCommand(0.0, 0.0, 0.0));
  EXPECT_GT(command.linear_x, 0.0);
  EXPECT_FALSE(controller.isGoalReached(Pose2D(0.0, 0.0, 0.0)));
}

TEST(DwaController, LethalLocalCostmapBlocksMotion)
{
  auto controller = makeController();
  controller.setPath(makeStraightPath());
  controller.setLocalCostmap(makeLocalGrid(100));

  const auto command = controller.computeVelocityCommand(
    Pose2D(0.0, 0.0, 0.0), VelocityCommand(0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
}

TEST(DwaController, ReportsGoalReachedAtPathEnd)
{
  auto controller = makeController();
  controller.setPath(makeStraightPath());
  EXPECT_TRUE(controller.isGoalReached(Pose2D(2.0, 0.0, 0.0)));
  EXPECT_FALSE(controller.isGoalReached(Pose2D(1.0, 0.0, 0.0)));
}

TEST(DwaController, AngularCommandRespectsAccelerationLimit)
{
  ControllerConfig config;
  config.max_linear_velocity = 0.5;
  config.max_angular_velocity = 1.0;
  config.max_accel_x = 0.5;
  config.max_accel_theta = 0.2;
  config.xy_goal_tolerance = 0.2;
  config.kinematics.max_velocity_x = 0.5;
  config.kinematics.max_velocity_theta = 1.0;

  DwaController controller;
  ASSERT_TRUE(controller.initialize(config));
  Path path;
  for (double x = 0.0; x <= 2.0 + 1e-9; x += 0.1) {
    path.points.emplace_back(Pose2D(x, x, M_PI / 4.0));
  }
  controller.setPath(path);
  const auto command = controller.computeVelocityCommand(
    Pose2D(0.0, 0.0, 0.0), VelocityCommand(0.0, 0.0, 0.0));
  EXPECT_LE(std::abs(command.angular_z), 0.2 + 1e-6);
}

}  // namespace
