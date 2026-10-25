#include <gtest/gtest.h>

#include <vector>

#include "rosiwit_navigation/nav_core/safety_monitor.hpp"

namespace
{
using rosiwit_navigation::core::SafetyMonitor;
using rosiwit_navigation::core::SafetyMonitorConfig;
using rosiwit_navigation::core::VelocityCommand;
using Point = std::pair<double, double>;

// 车身轮廓: x -0.25..0.55, y +-0.33 (实车)
std::vector<Point> BodyBox()
{
  return {{-0.27, -0.35}, {0.57, -0.35}, {0.57, 0.35}, {-0.27, 0.35}};
}
std::vector<Point> ForwardStopZone()
{
  return {{0.55, -0.40}, {0.95, -0.40}, {0.95, 0.40}, {0.55, 0.40}};
}
std::vector<Point> RotationStopZone()
{
  return {{-0.50, -0.55}, {0.80, -0.55}, {0.80, 0.55}, {-0.50, 0.55}};
}
std::vector<Point> BackwardStopZone()
{
  return {{-0.75, -0.40}, {-0.35, -0.40}, {-0.35, 0.40}, {-0.75, 0.40}};
}

SafetyMonitorConfig MakeConfig()
{
  SafetyMonitorConfig cfg;
  cfg.enabled = true;
  cfg.body_polygon = BodyBox();
  cfg.stop_polygon = ForwardStopZone();
  cfg.stop_polygon_backward = BackwardStopZone();
  cfg.stop_polygon_rotation = RotationStopZone();
  cfg.stop_max_points = 2;
  cfg.slowdown_polygon = {{0.55, -0.55}, {1.15, -0.55}, {1.15, 0.55}, {0.55, 0.55}};
  cfg.slowdown_max_points = 100;
  cfg.approach_enabled = false;
  cfg.footprint = {{-0.25, -0.33}, {0.55, -0.33}, {0.55, 0.33}, {-0.25, 0.33}};
  cfg.stop_trigger_frames = 1;
  cfg.stop_release_frames = 1;
  return cfg;
}

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
  const std::vector<Point> points{{0.5, 0.0}};
  const auto result = monitor.filter(VelocityCommand(0.5, 0.0, 0.0), points);
  EXPECT_EQ(result.state, "APPROACH");
  EXPECT_GT(result.scale, 0.2);
  EXPECT_LT(result.scale, 0.4);
}

// ---- 新增: 车身排除 / 三区切换 / 滞回 ----

TEST(SafetyMonitor, BodyExclusionKeepsSelfPointsOut)
{
  SafetyMonitor monitor;
  monitor.configure(MakeConfig());
  // 车身内的点 (自己): 不该触发 STOP (以前大矩形自过滤把车身外 0.75m 也吞了,
  // 而车身内的点又会一直被当障碍 -> 两头都不对)
  const std::vector<Point> self_points{{0.10, 0.0}, {0.30, 0.20}, {0.56, 0.30}};
  auto r = monitor.filter(VelocityCommand(0.3, 0.0, 0.0), self_points);
  EXPECT_EQ(r.state, "PASS");
  EXPECT_DOUBLE_EQ(r.command.linear_x, 0.3);
}

TEST(SafetyMonitor, ForwardZoneStopsBeforeBodyEdge)
{
  SafetyMonitor monitor;
  monitor.configure(MakeConfig());
  // 车头前 0.7m 处 3 个点 -> 进前进 STOP 区
  const std::vector<Point> points{{0.70, 0.0}, {0.72, 0.05}, {0.68, -0.05}};
  auto r = monitor.filter(VelocityCommand(0.3, 0.0, 0.0), points);
  EXPECT_EQ(r.state, "STOP");
  EXPECT_DOUBLE_EQ(r.command.linear_x, 0.0);
}

TEST(SafetyMonitor, RotationZoneOnlyAppliesWhenRotating)
{
  SafetyMonitor monitor;
  auto cfg = MakeConfig();
  cfg.stop_trigger_frames = 1;
  monitor.configure(cfg);
  // 侧向 0.45m 的点: 原地旋转时必须停 (车体扫掠), 前进时不停 (前进区只覆盖前方)
  const std::vector<Point> side{{0.10, 0.45}, {0.12, 0.47}, {0.08, 0.44}};
  auto r_forward = monitor.filter(VelocityCommand(0.3, 0.0, 0.0), side);
  EXPECT_EQ(r_forward.state, "PASS");

  monitor.reset();
  auto r_rotate = monitor.filter(VelocityCommand(0.0, 0.0, 0.6), side);
  EXPECT_EQ(r_rotate.state, "STOP");
  EXPECT_DOUBLE_EQ(r_rotate.command.angular_z, 0.0);
}

TEST(SafetyMonitor, BackwardZoneAppliesWhenReversing)
{
  SafetyMonitor monitor;
  monitor.configure(MakeConfig());
  const std::vector<Point> behind{{-0.55, 0.0}, {-0.58, 0.05}, {-0.52, -0.05}};
  auto r = monitor.filter(VelocityCommand(-0.2, 0.0, 0.0), behind);
  EXPECT_EQ(r.state, "STOP");
  EXPECT_DOUBLE_EQ(r.command.linear_x, 0.0);
}

TEST(SafetyMonitor, TriggerHysteresisNeedsTwoFrames)
{
  auto cfg = MakeConfig();
  cfg.stop_trigger_frames = 2;
  cfg.stop_release_frames = 3;
  SafetyMonitor monitor;
  monitor.configure(cfg);
  const std::vector<Point> points{{0.70, 0.0}, {0.72, 0.05}, {0.68, -0.05}};

  auto r1 = monitor.filter(VelocityCommand(0.3, 0.0, 0.0), points);
  EXPECT_EQ(r1.state, "PASS");            // 第 1 帧不触发
  EXPECT_FALSE(monitor.stopActive());
  auto r2 = monitor.filter(VelocityCommand(0.3, 0.0, 0.0), points);
  EXPECT_EQ(r2.state, "STOP");            // 第 2 帧触发
  EXPECT_TRUE(monitor.stopActive());
}

TEST(SafetyMonitor, ReleaseHysteresisHoldsStop)
{
  auto cfg = MakeConfig();
  cfg.stop_trigger_frames = 1;
  cfg.stop_release_frames = 3;
  SafetyMonitor monitor;
  monitor.configure(cfg);
  const std::vector<Point> points{{0.70, 0.0}, {0.72, 0.05}, {0.68, -0.05}};
  (void)monitor.filter(VelocityCommand(0.3, 0.0, 0.0), points);
  ASSERT_TRUE(monitor.stopActive());

  const std::vector<Point> empty;
  // 障碍消失后仍要连续 3 帧才释放, 避免噪声抖动
  EXPECT_EQ(monitor.filter(VelocityCommand(0.3, 0.0, 0.0), empty).state, "STOP");
  EXPECT_EQ(monitor.filter(VelocityCommand(0.3, 0.0, 0.0), empty).state, "STOP");
  EXPECT_EQ(monitor.filter(VelocityCommand(0.3, 0.0, 0.0), empty).state, "PASS");
  EXPECT_FALSE(monitor.stopActive());
}

TEST(SafetyMonitor, SingleFrameNoiseDoesNotFlipState)
{
  auto cfg = MakeConfig();
  cfg.stop_trigger_frames = 2;
  cfg.stop_release_frames = 2;
  SafetyMonitor monitor;
  monitor.configure(cfg);
  const std::vector<Point> hit{{0.70, 0.0}, {0.72, 0.05}, {0.68, -0.05}};
  const std::vector<Point> empty;
  // 单帧噪声: 命中 -> 空 -> 命中 -> 空, 不应进入 STOP
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(monitor.filter(VelocityCommand(0.3, 0.0, 0.0), hit).state, "PASS");
    EXPECT_EQ(monitor.filter(VelocityCommand(0.3, 0.0, 0.0), empty).state, "PASS");
  }
  EXPECT_FALSE(monitor.stopActive());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
