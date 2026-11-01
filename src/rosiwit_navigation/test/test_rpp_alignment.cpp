#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "rosiwit_navigation/nav_core/rpp_controller.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace
{
using rosiwit_navigation::controllers::RppController;
using rosiwit_navigation::core::ControllerConfig;
using rosiwit_navigation::core::Path;
using rosiwit_navigation::core::PathPoint;
using rosiwit_navigation::core::Pose2D;
using rosiwit_navigation::core::VelocityCommand;

// 0.75m 门洞场景的几何: 起点 (0,0) 朝 +x, 路径沿 +y 直上 (门在 y≈2.9)。
// 车体 0.66x0.80, 门洞半宽 0.375 -> 车体斜置必须 < 2.3° 才塞得进去。
Path straightPathY(double length, double step = 0.05)
{
  Path p;
  for (double y = 0.0; y <= length + 1e-9; y += step) {
    PathPoint pp(Pose2D(0.0, y, M_PI / 2.0));
    pp.velocity_x = 0.30;
    p.points.push_back(pp);
  }
  return p;
}

std::unique_ptr<RppController> makeController()
{
  ControllerConfig cfg;
  cfg.max_linear_velocity = 0.45;
  cfg.max_angular_velocity = 0.8;
  cfg.max_accel_x = 0.8;
  cfg.max_accel_theta = 0.6;
  cfg.xy_goal_tolerance = 0.10;
  cfg.slow_down_distance = 0.0;
  cfg.min_lookahead_distance = 0.18;
  cfg.max_lookahead_distance = 0.50;
  cfg.lookahead_gain = 0.8;
  cfg.use_velocity_scaled_lookahead_dist = true;
  cfg.lookahead_time = 1.5;
  cfg.path_resample_step = 0.05;
  cfg.use_rotate_to_heading = true;
  cfg.rotate_to_heading_min_angle = 0.785;
  std::unique_ptr<RppController> c(new RppController());
  c->initialize(cfg);
  return c;
}

struct LoopResult {
  std::vector<Pose2D> trace;
  bool reached{false};
  double max_abs_x_after_1m{0.0};   // 走完 1m 之后的横向偏差 (起步瞬态不算)
  double heading_err_at_door{0.0};  // 穿过 y=2.85 时的航向误差 (rad)
  double yaw_deg_at_start{0.0};
};

LoopResult run(const Path & path, const Pose2D & start, double max_time = 60.0)
{
  std::unique_ptr<RppController> c = makeController();
  c->setPath(path);
  LoopResult r;
  Pose2D pose = start;
  const double dt = 0.1;
  bool crossed = false;
  for (double t = 0.0; t < max_time; t += dt) {
    const VelocityCommand cmd = c->computeVelocityCommand(pose, VelocityCommand(0.0, 0.0, 0.0));
    r.trace.push_back(pose);
    pose.x += cmd.linear_x * std::cos(pose.theta) * dt;
    pose.y += cmd.linear_x * std::sin(pose.theta) * dt;
    pose.theta += cmd.angular_z * dt;
    if (!crossed && pose.y > 2.85) {
      crossed = true;
      r.heading_err_at_door = std::fabs(std::atan2(std::sin(pose.theta - M_PI / 2.0),
                                                  std::cos(pose.theta - M_PI / 2.0)));
    }
    if (path.points.size() > 1) {
      const auto & g = path.points.back().pose;
      if (std::hypot(g.x - pose.x, g.y - pose.y) < 0.12) { r.reached = true; break; }
    }
    if (pose.y > 1.0) { r.max_abs_x_after_1m = std::max(r.max_abs_x_after_1m, std::fabs(pose.x)); }
  }
  return r;
}

// 起步必须"先转后走": 航向差 90° 时直接开走会切出 0.2m 的弧, 到门前收不回来
TEST(RppAlignment, RotatesInPlaceBeforeDriving)
{
  std::unique_ptr<RppController> c = makeController();
  c->setPath(straightPathY(3.8));
  const VelocityCommand cmd = c->computeVelocityCommand(Pose2D(0.0, 0.0, 0.0),
                                                        VelocityCommand(0.0, 0.0, 0.0));
  EXPECT_LT(std::fabs(cmd.linear_x), 1e-6) << "起步就往前冲 -> 会切弧";
  EXPECT_GT(std::fabs(cmd.angular_z), 0.3) << "没有原地对准";
}

// 纯跟踪质量: 起步已对准 (朝 +y), 路径沿 +y 直行 3.8m。
// 这是 nav2 RPP 的标准工况: 横向偏差应保持在厘米级, 门前航向误差 < 2.3°。
TEST(RppAlignment, StraightLineTrackingStaysCentered)
{
  const LoopResult r = run(straightPathY(3.8), Pose2D(0.0, 0.0, M_PI / 2.0));
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.heading_err_at_door, 0.04)
      << "door heading err = " << r.heading_err_at_door * 180.0 / M_PI << " deg";
  EXPECT_LT(r.max_abs_x_after_1m, 0.03) << "lateral excursion = " << r.max_abs_x_after_1m;
}

// 起步朝 +x (需转 90°): 用 nav2 rotation_shim 的语义先原地转 (航向差 > 0.785)。
// 起步瞬态允许有横向摆动 (真实 nav2 也是这样, 由代价/碰撞调速兜住),
// 但**穿过门平面时必须已经对准** (< 2.3°), 否则 0.66m 车体塞不进 0.75m 门。
TEST(RppAlignment, MisalignedStartCrossesDoorAligned)
{
  const LoopResult r = run(straightPathY(3.8), Pose2D(0.0, 0.0, 0.0));
  EXPECT_TRUE(r.reached);
  EXPECT_LT(r.heading_err_at_door, 0.04)
      << "door heading err = " << r.heading_err_at_door * 180.0 / M_PI << " deg";
}
}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
