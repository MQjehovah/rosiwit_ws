#include <gtest/gtest.h>

#include "rosiwit_navigation/nav_core/safety_monitor.hpp"

namespace
{
using rosiwit_navigation::core::SafetyMonitor;
using rosiwit_navigation::core::SafetyMonitorConfig;
using rosiwit_navigation::core::VelocityCommand;
using Point = std::pair<double, double>;
}  // namespace

TEST(SafetyMonitor, PointInPolygon)
{
  const std::vector<Point> square{{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}};
  EXPECT_TRUE(rosiwit_navigation::core::pointInPolygon(0.0, 0.0, square));
  EXPECT_FALSE(rosiwit_navigation::core::pointInPolygon(2.0, 0.0, square));
  EXPECT_FALSE(rosiwit_navigation::core::pointInPolygon(0.0, -2.0, square));
}

TEST(SafetyMonitor, StopZoneZerosCommand)
{
  SafetyMonitorConfig cfg;
  cfg.enabled = true;
  cfg.stop_polygon = {{-0.3, -0.3}, {0.3, -0.3}, {0.3, 0.3}, {-0.3, 0.3}};
  cfg.stop_max_points = 2;
  cfg.slowdown_polygon = {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}};
  cfg.slowdown_max_points = 100;
  cfg.footprint = {{-0.2, -0.2}, {0.2, -0.2}, {0.2, 0.2}, {-0.2, 0.2}};

  SafetyMonitor monitor;
  monitor.configure(cfg);
  const std::vector<Point> points{{0.1, 0.0}, {0.15, 0.05}, {0.2, 0.0}};
  const auto result = monitor.filter(VelocityCommand(0.5, 0.0, 0.0), points);
  EXPECT_EQ(result.state, "STOP");
  EXPECT_DOUBLE_EQ(result.scale, 0.0);
  EXPECT_DOUBLE_EQ(result.command.linear_x, 0.0);
}

TEST(SafetyMonitor, SlowdownScalesAllComponents)
{
  SafetyMonitorConfig cfg;
  cfg.enabled = true;
  cfg.stop_polygon = {{-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}};
  cfg.stop_max_points = 100;
  cfg.slowdown_polygon = {{-0.6, -0.6}, {0.6, -0.6}, {0.6, 0.6}, {-0.6, 0.6}};
  cfg.slowdown_max_points = 1;
  cfg.slowdown_ratio = 0.5;
  cfg.approach_enabled = false;

  SafetyMonitor monitor;
  monitor.configure(cfg);
  const std::vector<Point> points{{0.4, 0.0}, {0.45, 0.1}};
  const auto result = monitor.filter(VelocityCommand(0.4, 0.0, 0.2), points);
  EXPECT_EQ(result.state, "SLOWDOWN");
  EXPECT_DOUBLE_EQ(result.scale, 0.5);
  EXPECT_DOUBLE_EQ(result.command.linear_x, 0.2);
  EXPECT_DOUBLE_EQ(result.command.angular_z, 0.1);
}

TEST(SafetyMonitor, ApproachKeepsTimeMargin)
{
  SafetyMonitorConfig cfg;
  cfg.enabled = true;
  cfg.stop_polygon = {{-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}};
  cfg.stop_max_points = 100;
  cfg.slowdown_polygon = {{-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}};
  cfg.slowdown_max_points = 100;
  cfg.footprint = {{-0.2, -0.2}, {0.2, -0.2}, {0.2, 0.2}, {-0.2, 0.2}};
  cfg.approach_enabled = true;
  cfg.time_before_collision = 2.0;
  cfg.simulation_time_step = 0.1;

  SafetyMonitor monitor;
  monitor.configure(cfg);
  // Point at 0.5 m ahead; footprint front is 0.2 m, so contact at ~0.6 s.
  const std::vector<Point> points{{0.5, 0.0}};
  const auto result = monitor.filter(VelocityCommand(0.5, 0.0, 0.0), points);
  EXPECT_EQ(result.state, "APPROACH");
  EXPECT_GT(result.scale, 0.2);
  EXPECT_LT(result.scale, 0.4);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
