#include <gtest/gtest.h>

#include "diff_drive_controller.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace
{
using rosiwit_navigation::controllers::DiffDriveController;
using rosiwit_navigation::core::ControllerConfig;
using rosiwit_navigation::core::Path;
using rosiwit_navigation::core::PathPoint;
using rosiwit_navigation::core::Pose2D;
using rosiwit_navigation::core::VelocityCommand;

ControllerConfig makeConfig()
{
  ControllerConfig config{};
  config.lookahead_distance = 0.35;
  config.xy_goal_tolerance = 0.10;
  config.slow_down_distance = 0.20;
  config.max_linear_velocity = 0.30;
  config.max_angular_velocity = 0.50;
  config.kinematics.max_velocity_x = 0.30;
  config.kinematics.max_velocity_theta = 0.50;
  config.allow_reverse = true;
  return config;
}

Path reverseArc()
{
  // 机器人朝 +x, 路径在后方并向 +y 弯曲 (倒车左转).
  Path path;
  for (int i = 0; i < 10; ++i) {
    PathPoint point;
    point.pose = Pose2D(-0.05 * i, 0.02 * i, 0.0);
    point.velocity_x = -1.0;
    path.points.push_back(point);
  }
  return path;
}

TEST(DiffDrive, ReversePathUsesNegativeVelocityAndYawRate)
{
  DiffDriveController controller;
  ASSERT_TRUE(controller.initialize(makeConfig()));
  controller.setPath(reverseArc());
  const VelocityCommand cmd = controller.computeVelocityCommand(
    Pose2D(0.0, 0.0, 0.0), VelocityCommand(0.0, 0.0, 0.0));
  EXPECT_LT(cmd.linear_x, -0.01);
  EXPECT_LT(cmd.angular_z, -0.01);
}

}  // namespace
