#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "dwa_controller.hpp"
#include "mppi_controller.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace
{
using rosiwit_navigation::controllers::DwaController;
using rosiwit_navigation::controllers::MppiController;
using rosiwit_navigation::core::ControllerConfig;
using rosiwit_navigation::core::CostmapGrid;
using rosiwit_navigation::core::Path;
using rosiwit_navigation::core::Pose2D;
using rosiwit_navigation::core::VelocityCommand;

struct SimResult {
  bool reached{false};
  double elapsed{0.0};
  double max_speed{0.0};
  double speed_at_1s{0.0};
  double final_x{0.0};
};

ControllerConfig controllerConfig()
{
  ControllerConfig config;
  config.max_linear_velocity = 0.35;
  config.max_angular_velocity = 0.8;
  config.max_accel_x = 0.4;
  config.max_accel_theta = 0.8;
  config.xy_goal_tolerance = 0.25;
  config.kinematics.max_velocity_x = 0.35;
  config.kinematics.max_velocity_theta = 0.8;
  return config;
}

CostmapGrid freeLocalGrid()
{
  CostmapGrid grid;
  grid.info.width = 400;
  grid.info.height = 400;
  grid.info.resolution = 0.05;
  grid.info.origin.position.x = -10.0;
  grid.info.origin.position.y = -10.0;
  grid.data.assign(400 * 400, 0);
  return grid;
}

Path straightPath(double length = 3.0)
{
  Path path;
  for (double x = 0.0; x <= length + 1e-9; x += 0.05) {
    path.points.emplace_back(Pose2D(x, 0.0, 0.0));
  }
  return path;
}

Path remainingPath(const Path & base, const Pose2D & pose)
{
  // 模拟导航层 replan: 路径从机器人当前位置重新生成, 终点不变.
  Path out;
  out.points.emplace_back(pose);
  double best = std::numeric_limits<double>::max();
  size_t best_i = 0;
  for (size_t i = 0; i < base.points.size(); ++i) {
    const double dx = base.points[i].pose.x - pose.x;
    const double dy = base.points[i].pose.y - pose.y;
    const double d = dx * dx + dy * dy;
    if (d < best) { best = d; best_i = i; }
  }
  for (size_t i = best_i; i < base.points.size(); ++i) {
    out.points.push_back(base.points[i]);
  }
  return out;
}

template<typename Controller>
SimResult runClosedLoop(
  Controller & controller, const Path & path, double seconds, bool replan)
{
  const double dt = 0.05;
  Pose2D pose(0.0, 0.0, 0.0);
  VelocityCommand actual(0.0, 0.0, 0.0);
  SimResult result;
  const auto goal = path.points.back().pose;

  for (int step = 0; step < static_cast<int>(seconds / dt); ++step) {
    if (replan && step % 10 == 0) {
      controller.setPath(remainingPath(path, pose));
    }
    const auto cmd = controller.computeVelocityCommand(pose, actual);
    actual = cmd;
    pose.x += cmd.linear_x * std::cos(pose.theta) * dt;
    pose.y += cmd.linear_x * std::sin(pose.theta) * dt;
    pose.theta += cmd.angular_z * dt;
    result.elapsed = (step + 1) * dt;
    result.max_speed = std::max(result.max_speed, std::abs(cmd.linear_x));
    if (result.elapsed >= 1.0 && result.speed_at_1s == 0.0) {
      result.speed_at_1s = std::abs(cmd.linear_x);
    }
    result.final_x = pose.x;
    if (pose.distanceTo(goal) < 0.25) {
      result.reached = true;
      break;
    }
  }
  return result;
}

TEST(ControllerClosedLoop, MppiReachesStraightGoal)
{
  MppiController controller;
  ASSERT_TRUE(controller.initialize(controllerConfig()));
  const Path path = straightPath();
  controller.setPath(path);
  controller.setLocalCostmap(freeLocalGrid());

  const auto result = runClosedLoop(controller, path, 12.0, false);
  EXPECT_TRUE(result.reached) << "final_x=" << result.final_x
                              << " max_speed=" << result.max_speed;
  EXPECT_GT(result.max_speed, 0.15);
  EXPECT_GT(result.speed_at_1s, 0.08);
}

TEST(ControllerClosedLoop, MppiSurvivesFrequentReplanning)
{
  MppiController controller;
  ASSERT_TRUE(controller.initialize(controllerConfig()));
  const Path path = straightPath();
  controller.setPath(path);
  controller.setLocalCostmap(freeLocalGrid());

  const auto result = runClosedLoop(controller, path, 12.0, true);
  EXPECT_TRUE(result.reached) << "final_x=" << result.final_x
                              << " max_speed=" << result.max_speed;
  EXPECT_GT(result.max_speed, 0.15);
}

TEST(ControllerClosedLoop, DwaReachesStraightGoal)
{
  DwaController controller;
  ASSERT_TRUE(controller.initialize(controllerConfig()));
  const Path path = straightPath();
  controller.setPath(path);
  controller.setLocalCostmap(freeLocalGrid());

  const auto result = runClosedLoop(controller, path, 12.0, false);
  EXPECT_TRUE(result.reached) << "final_x=" << result.final_x;
  EXPECT_GT(result.max_speed, 0.15);
}

TEST(ControllerClosedLoop, DwaSurvivesFrequentReplanning)
{
  DwaController controller;
  ASSERT_TRUE(controller.initialize(controllerConfig()));
  const Path path = straightPath();
  controller.setPath(path);
  controller.setLocalCostmap(freeLocalGrid());

  const auto result = runClosedLoop(controller, path, 12.0, true);
  EXPECT_TRUE(result.reached) << "final_x=" << result.final_x;
  EXPECT_GT(result.max_speed, 0.15);
}

}  // namespace
