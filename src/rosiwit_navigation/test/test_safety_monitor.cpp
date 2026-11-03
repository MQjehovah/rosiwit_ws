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

TEST(SafetyMonitor, DynamicStopUsesBrakingDistance)
{
  SafetyMonitorConfig cfg;
  cfg.enabled = true;
  cfg.dynamic_geometry = true;
  cfg.brake_decel = 0.8;
  cfg.reaction_time = 0.2;
  cfg.stop_margin = 0.05;
  cfg.side_margin = 0.03;
  cfg.turn_margin = 0.05;
  cfg.footprint = {{-0.25, -0.33}, {0.55, -0.33}, {0.55, 0.33}, {-0.25, 0.33}};
  cfg.stop_max_points = 2;
  cfg.stop_trigger_frames = 1;
  cfg.stop_release_frames = 1;
  cfg.slowdown_max_points = 100;
  cfg.approach_enabled = false;

  SafetyMonitor monitor;
  monitor.configure(cfg);
  const std::vector<Point> points{{1.0, 0.0}, {1.02, 0.01}, {1.03, -0.01}};

  // v=0.8: front = 0.55 + 0.8^2/(2*0.8) + 0.8*0.2 + 0.05 = 1.16 -> 1.0m 在区内
  const auto fast = monitor.filter(VelocityCommand(0.8, 0.0, 0.0), points);
  EXPECT_EQ(fast.state, "STOP");

  // v=0.2: front = 0.55 + 0.025 + 0.04 + 0.05 = 0.665 -> 1.0m 在区外
  monitor.reset();
  const auto slow = monitor.filter(VelocityCommand(0.2, 0.0, 0.0), points);
  EXPECT_NE(slow.state, "STOP");
}

TEST(SafetyMonitor, DynamicStopSideAllowsNarrowAisle)
{
  SafetyMonitorConfig cfg;
  cfg.enabled = true;
  cfg.dynamic_geometry = true;
  cfg.brake_decel = 0.8;
  cfg.reaction_time = 0.2;
  cfg.stop_margin = 0.05;
  cfg.side_margin = 0.03;
  cfg.footprint = {{-0.25, -0.33}, {0.55, -0.33}, {0.55, 0.33}, {-0.25, 0.33}};
  cfg.stop_max_points = 2;
  cfg.stop_trigger_frames = 1;
  cfg.stop_release_frames = 1;
  cfg.slowdown_max_points = 100;
  cfg.approach_enabled = false;

  SafetyMonitor monitor;
  monitor.configure(cfg);
  // 0.8m 巷道: 侧墙约 0.40; 动态侧边界 = 0.33+0.03=0.36 -> 不应停车
  const std::vector<Point> wall_far{{0.2, 0.40}, {0.21, 0.40}, {0.22, 0.40}};
  const auto pass = monitor.filter(VelocityCommand(0.3, 0.0, 0.0), wall_far);
  EXPECT_NE(pass.state, "STOP");

  // 侧墙 0.34 -> 已进入 0.36 -> 停车
  monitor.reset();
  const std::vector<Point> wall_near{{0.2, 0.34}, {0.21, 0.34}, {0.22, 0.34}};
  const auto stop = monitor.filter(VelocityCommand(0.3, 0.0, 0.0), wall_near);
  EXPECT_EQ(stop.state, "STOP");
}

TEST(SafetyMonitor, ScaleSlewLimitsRecovery)
{
  SafetyMonitorConfig cfg;
  cfg.enabled = true;
  cfg.dynamic_geometry = false;
  cfg.stop_polygon = {{-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}};
  cfg.stop_max_points = 100;
  cfg.slowdown_polygon = {{-0.6, -0.6}, {0.6, -0.6}, {0.6, 0.6}, {-0.6, 0.6}};
  cfg.slowdown_max_points = 1;
  cfg.slowdown_ratio = 0.5;
  cfg.approach_enabled = false;
  cfg.scale_slew_rate = 1.0;
  cfg.scale_dt = 0.1;

  SafetyMonitor monitor;
  monitor.configure(cfg);
  const std::vector<Point> points{{0.4, 0.0}, {0.45, 0.1}};

  // 降速应立即生效 (安全优先)
  const auto slow = monitor.filter(VelocityCommand(0.4, 0.0, 0.0), points);
  EXPECT_EQ(slow.state, "SLOWDOWN");
  EXPECT_NEAR(slow.scale, 0.5, 1e-9);

  // 障碍消失后只允许按 slew 回升: 0.5 -> 0.6 -> 0.7
  const std::vector<Point> empty;
  const auto up1 = monitor.filter(VelocityCommand(0.4, 0.0, 0.0), empty);
  EXPECT_NEAR(up1.scale, 0.6, 1e-9);
  const auto up2 = monitor.filter(VelocityCommand(0.4, 0.0, 0.0), empty);
  EXPECT_NEAR(up2.scale, 0.7, 1e-9);
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

TEST(SafetyMonitor, RotationZoneCircleIgnoresDiagonalOverCover)
{
  // 实测回归: 矩形旋转区 (对角 0.97m) 会把 0.83m 外、车体扫掠只到 0.64m 的
  // 物体判成障碍 -> 原地转被永久 STOP, 覆盖率卡死. 用扫掠圆后应放行.
  auto cfg = MakeConfig();
  cfg.stop_trigger_frames = 1;
  cfg.stop_radius_rotation = 0.74;   // 车体最远顶点 0.64 + 0.10
  SafetyMonitor monitor;
  monitor.configure(cfg);
  // 该点在矩形里 (x<=0.80, |y|<=0.55), 但距车心 0.80 > 0.74 (扫掠圆外);
  // 且必须在车身排除盒 (x<=0.57, |y|<=0.35) 之外, 否则会被当自身点丢掉.
  const std::vector<Point> diag{{0.64, -0.47}, {0.65, -0.46}, {0.63, -0.48}};
  auto r = monitor.filter(VelocityCommand(0.0, 0.0, 0.8), diag);
  EXPECT_EQ(r.state, "PASS");
  EXPECT_DOUBLE_EQ(r.command.angular_z, 0.8);

  // 换成"只有矩形"的配置: 同样三个点必须停 -> 证明这条回归是圆判据修掉的
  monitor.reset();
  cfg.stop_radius_rotation = 0.0;
  monitor.configure(cfg);
  auto r2 = monitor.filter(VelocityCommand(0.0, 0.0, 0.8), diag);
  EXPECT_EQ(r2.state, "STOP");
}

TEST(SafetyMonitor, StoppedCommandUsesForwardZoneNotRotationCircle)
{
  // 回归: v~0 且 w~0 (蠕行/停住) 时必须用前进区, 不能套旋转圆.
  // 否则 0.74m 内任何方向的物体都会把车按住, 行末再也动不了.
  auto cfg = MakeConfig();
  cfg.stop_trigger_frames = 1;
  cfg.stop_radius_rotation = 0.74;
  SafetyMonitor monitor;
  monitor.configure(cfg);
  // 侧后方 0.70m 的点: 在旋转圆内, 但不在前进区
  const std::vector<Point> behind_side{{0.20, -0.68}, {0.22, -0.67}, {0.18, -0.69}};
  auto r = monitor.filter(VelocityCommand(0.0, 0.0, 0.0), behind_side);
  EXPECT_EQ(r.state, "PASS");
  // 同一个点, 真的在转 (w=0.6) -> 必须停
  monitor.reset();
  auto r2 = monitor.filter(VelocityCommand(0.0, 0.0, 0.6), behind_side);
  EXPECT_EQ(r2.state, "STOP");
}

TEST(SafetyMonitor, RotationCircleStillStopsRealObstacle)
{
  auto cfg = MakeConfig();
  cfg.stop_trigger_frames = 1;
  cfg.stop_radius_rotation = 0.74;
  SafetyMonitor monitor;
  monitor.configure(cfg);
  // 0.61m 处的物体 (车身排除盒之外 x>0.57): 车体扫掠 (0.64) 会撞上 -> 必须停
  const std::vector<Point> close{{0.60, 0.10}, {0.61, 0.08}, {0.59, 0.12}};
  auto r = monitor.filter(VelocityCommand(0.0, 0.0, 0.5), close);
  EXPECT_EQ(r.state, "STOP");
}


TEST(SafetyMonitor, SweptSlowdownZoneSlowsBeforeStop)
{
  // 同样的 footprint, SLOWDOWN 扫掠更久: 障碍在"减速区但不在停止区"时应减速不停车
  SafetyMonitorConfig cfg;
  cfg.enabled = true;
  cfg.dynamic_geometry = true;
  cfg.brake_decel = 0.8;
  cfg.reaction_time = 0.2;
  cfg.stop_margin = 0.05;
  cfg.side_margin = 0.03;
  cfg.footprint = {{-0.25, -0.33}, {0.55, -0.33}, {0.55, 0.33}, {-0.25, 0.33}};
  cfg.stop_max_points = 2;
  cfg.slowdown_max_points = 2;
  cfg.slowdown_ratio = 0.5;
  cfg.slowdown_lookahead = 0.8;
  cfg.stop_trigger_frames = 1;
  cfg.stop_release_frames = 1;
  cfg.slowdown_trigger_frames = 1;
  cfg.slowdown_release_frames = 1;
  cfg.approach_enabled = false;

  SafetyMonitor monitor;
  monitor.configure(cfg);
  // v=0.5: stop 扫掠到车头+0.51m (0.55+0.41); slow 扫掠再多 0.4m
  const std::vector<Point> far_points{{1.05, 0.0}, {1.06, 0.05}, {1.04, -0.05}};
  const auto slow = monitor.filter(VelocityCommand(0.5, 0.0, 0.0), far_points);
  EXPECT_NE(slow.state, "STOP");
  EXPECT_LT(slow.scale, 1.0) << slow.state;

  // 换成 0.5m 处: 落进 STOP 扫掠区 -> 停
  monitor.reset();
  const std::vector<Point> near_points{{0.70, 0.0}, {0.71, 0.05}, {0.69, -0.05}};
  const auto stop = monitor.filter(VelocityCommand(0.5, 0.0, 0.0), near_points);
  EXPECT_EQ(stop.state, "STOP");
}


TEST(SafetyMonitor, SlowdownKeepsMinimumSpeed)
{
  // 减速比 0.1 会把 0.6 m/s 压到 0.06 -> 低于下限, 应抬到 slowdown_min_speed
  SafetyMonitorConfig cfg;
  cfg.enabled = true;
  cfg.dynamic_geometry = true;
  cfg.brake_decel = 0.8;
  cfg.reaction_time = 0.2;
  cfg.stop_margin = 0.05;
  cfg.side_margin = 0.03;
  cfg.footprint = {{-0.25, -0.33}, {0.55, -0.33}, {0.55, 0.33}, {-0.25, 0.33}};
  cfg.stop_max_points = 2;
  cfg.slowdown_max_points = 2;
  cfg.slowdown_ratio = 0.1;
  cfg.slowdown_min_speed = 0.15;
  cfg.slowdown_lookahead = 0.8;
  cfg.stop_trigger_frames = 1;
  cfg.stop_release_frames = 1;
  cfg.slowdown_trigger_frames = 1;
  cfg.slowdown_release_frames = 1;
  cfg.approach_enabled = false;

  SafetyMonitor monitor;
  monitor.configure(cfg);
  // v=0.6: stop 扫掠到 1.12m, slow 扫掠到 1.60m -> 1.30m 只落在减速区
  const std::vector<Point> pts{{1.30, 0.0}, {1.31, 0.05}, {1.29, -0.05}};
  const auto r = monitor.filter(VelocityCommand(0.6, 0.0, 0.0), pts);
  EXPECT_NE(r.state, "STOP");
  EXPECT_GE(std::fabs(r.command.linear_x), 0.15 - 1e-6);
}

// ============================================================================
// 硬指标回归: 0.66m 车体过 0.75m 门洞 (门框内表面 x = ±0.375)
//
// 这些测试锁住"安全层几何不能把窄门算窄"这条契约:
//   * 余量必须各向异性: 前/后 stop_margin, 左右 side_margin
//   * 侧向余量一旦被 stop_margin(0.05) 污染, 侧向半宽 0.38 > 0.375 -> 永远停
// ============================================================================
SafetyMonitorConfig DoorConfig(double side_margin)
{
  SafetyMonitorConfig cfg;
  cfg.enabled = true;
  cfg.dynamic_geometry = true;
  cfg.brake_decel = 0.8;
  cfg.reaction_time = 0.2;
  cfg.stop_margin = 0.05;
  cfg.side_margin = side_margin;
  cfg.footprint = {{-0.25, -0.33}, {0.55, -0.33}, {0.55, 0.33}, {-0.25, 0.33}};
  cfg.stop_max_points = 4;
  cfg.slowdown_max_points = 1000;   // 只看 STOP, 避免减速干扰结论
  cfg.stop_trigger_frames = 1;
  cfg.stop_release_frames = 1;
  cfg.slowdown_trigger_frames = 1;
  cfg.slowdown_release_frames = 1;
  cfg.approach_enabled = false;
  return cfg;
}

// 门框: 两片墙在 y = ±half，x 从 0.30 到 1.60 (base 系, 车在原点朝 +x)
std::vector<Point> DoorFrame(double half, double shift_y)
{
  std::vector<Point> pts;
  for (double x = 0.30; x <= 1.60; x += 0.05) {
    pts.push_back({x, shift_y + half});
    pts.push_back({x, shift_y - half});
  }
  return pts;
}

TEST(SafetyMonitor, SweptZonePasses075DoorWhenCentered)
{
  // 0.75m 门洞、车体 0.66m、居中 -> 必须放行 (以前必停: 径向外扩 0.05 把侧向
  // 撑到 0.38 > 0.375)
  SafetyMonitor monitor;
  monitor.configure(DoorConfig(0.02));
  const auto r = monitor.filter(VelocityCommand(0.2, 0.0, 0.0), DoorFrame(0.375, 0.0));
  EXPECT_NE(r.state, "STOP");
}

TEST(SafetyMonitor, SweptZoneStopsWhenDoorOffsetExceedsSideMargin)
{
  // 同一个门, 但车体横向偏 6cm: 0.33 + 0.02 = 0.35 的侧向边界已经越过门框
  // (0.375 - 0.06 = 0.315) -> 必须停
  SafetyMonitor monitor;
  monitor.configure(DoorConfig(0.02));
  const auto r = monitor.filter(VelocityCommand(0.2, 0.0, 0.0), DoorFrame(0.375, -0.06));
  EXPECT_EQ(r.state, "STOP");
}

TEST(SafetyMonitor, SweptZoneLateralMarginIsSideMarginNotStopMargin)
{
  // 侧墙 0.365: 侧向边界 0.33+0.02=0.35 < 0.365 -> 放行。
  // 若侧向误用 stop_margin(0.05) -> 0.38 > 0.365 -> 会被判停 (这个测试就是防回归)。
  SafetyMonitor monitor;
  monitor.configure(DoorConfig(0.02));
  const auto pass = monitor.filter(VelocityCommand(0.2, 0.0, 0.0), DoorFrame(0.365, 0.0));
  EXPECT_NE(pass.state, "STOP");

  // 侧墙 0.34 -> 进入 0.35 侧向边界 -> 停
  SafetyMonitor monitor2;
  monitor2.configure(DoorConfig(0.02));
  const auto stop = monitor2.filter(VelocityCommand(0.2, 0.0, 0.0), DoorFrame(0.34, 0.0));
  EXPECT_EQ(stop.state, "STOP");
}

// 倒车脱困: 前方障碍不应阻止倒车指令。
// 2026-11-03 实车: 前方 0.16m 有墙, recovery 倒车 v=-0.03 每拍被安全层 STOP,
// 20s 后 FAILED_NO_PROGRESS。动态停区的 stop_margin 只能加在行进方向。
TEST(SafetyMonitor, ReverseEscapeNotBlockedByFrontMargin)
{
  SafetyMonitorConfig cfg = MakeConfig();
  cfg.dynamic_geometry = true;
  cfg.stop_margin = 0.15;
  cfg.side_margin = 0.12;
  cfg.brake_decel = 0.8;
  cfg.reaction_time = 0.2;
  cfg.approach_enabled = false;

  // 障碍在车头前 0.68m (footprint 前缘 0.55, 前进 margin 0.15 -> 0.70)
  const std::vector<Point> pts{{0.68, 0.0}, {0.69, 0.02}, {0.70, -0.02}};

  SafetyMonitor fwd_monitor;
  fwd_monitor.configure(cfg);
  const auto fwd = fwd_monitor.filter(VelocityCommand(0.30, 0.0, 0.0), pts);
  EXPECT_EQ(fwd.state, "STOP");

  SafetyMonitor rev_monitor;
  rev_monitor.configure(cfg);
  const auto rev = rev_monitor.filter(VelocityCommand(-0.10, 0.0, 0.0), pts);
  EXPECT_NE(rev.state, "STOP");
  EXPECT_GT(rev.scale, 0.0);
}

// 低速前进 + 小角度修正不能被当成"原地旋转"套用 ±0.55 旋转停区:
// 实车 2026-11-03 21:56 在走廊里 (侧墙 0.55m) 每拍 STOP, 车被钉死.
TEST(SafetyMonitor, SlowForwardBesideWallIsNotRotationStop)
{
  SafetyMonitorConfig cfg = MakeConfig();
  cfg.dynamic_geometry = true;
  cfg.stop_margin = 0.15;
  cfg.side_margin = 0.12;
  cfg.brake_decel = 0.8;
  cfg.reaction_time = 0.2;
  cfg.approach_enabled = false;

  // 右侧墙 0.54m (旋转区侧向是 ±0.55, 行进区是 0.35+0.12=0.47)
  const std::vector<Point> wall{{0.20, -0.54}, {0.30, -0.54}, {0.40, -0.54}};

  SafetyMonitor slow;
  slow.configure(cfg);
  const auto fwd = slow.filter(VelocityCommand(0.03, 0.0, 0.08), wall);
  EXPECT_NE(fwd.state, "STOP");

  SafetyMonitor spin;
  spin.configure(cfg);
  const auto rot = spin.filter(VelocityCommand(0.0, 0.0, 0.50), wall);
  EXPECT_EQ(rot.state, "STOP");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
