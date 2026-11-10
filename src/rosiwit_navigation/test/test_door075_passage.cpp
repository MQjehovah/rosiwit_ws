// ============================================================================
// 0.75m 门洞端到端离线复现 (不依赖 Gazebo)
//
// 组成: rpp 控制器 + core::SafetyMonitor (扫掠安全区) + 门框点云 + 单极模型积分
// 目的: 把"0.75m 门洞能不能过"变成秒级可重复的断言, 而不是每次 4 分钟仿真。
//
// 几何 (与 world/doorway.world 一致):
//   门框: y ∈ [2.90, 3.10], |x| ≥ 0.39   (门洞宽 0.78m ≈ 车宽 676 + 10cm)
//   车体: 官方图纸 906.23 x 676 (x -0.262..0.644, y ±0.338)
// ============================================================================
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <vector>

#include "rosiwit_navigation/nav_core/navigation_factory.hpp"
#include "rosiwit_navigation/nav_core/rpp_controller.hpp"
#include "rosiwit_navigation/nav_core/safety_monitor.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace
{
using rosiwit_navigation::controllers::RppController;
using rosiwit_navigation::core::Path;
using rosiwit_navigation::core::PathPoint;
using rosiwit_navigation::core::Pose2D;
using rosiwit_navigation::core::SafetyMonitor;
using rosiwit_navigation::core::SafetyMonitorConfig;
using rosiwit_navigation::core::SafetyResult;
using rosiwit_navigation::core::VelocityCommand;

constexpr double kDoorY0 = 2.90;
constexpr double kDoorY1 = 3.10;
constexpr double kDoorHalf = 0.39;   // 0.78m 门洞 = 车宽 676 + 10cm(776) 留 2mm

// 门框内表面的采样点 (世界系) —— 相当于 /scan 看到的东西
std::vector<std::pair<double, double>> doorSurfacePoints()
{
  std::vector<std::pair<double, double>> pts;
  const double step = 0.02;
  // 面向 -y 的两个内表面
  for (double x = -3.0; x <= -kDoorHalf; x += step) { pts.emplace_back(x, kDoorY0); }
  for (double x = kDoorHalf; x <= 3.0; x += step) { pts.emplace_back(x, kDoorY0); }
  // 门洞两侧的侧壁 (厚度方向)
  for (double y = kDoorY0; y <= kDoorY1; y += step) {
    pts.emplace_back(-kDoorHalf, y);
    pts.emplace_back(kDoorHalf, y);
  }
  return pts;
}

std::vector<std::pair<double, double>> toBaseFrame(
  const std::vector<std::pair<double, double>> & world, const Pose2D & pose, double range)
{
  std::vector<std::pair<double, double>> out;
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  for (const auto & p : world) {
    const double dx = p.first - pose.x;
    const double dy = p.second - pose.y;
    if (dx * dx + dy * dy > range * range) { continue; }
    out.emplace_back(c * dx + s * dy, -s * dx + c * dy);
  }
  return out;
}

SafetyMonitorConfig safetyConfig()
{
  SafetyMonitorConfig cfg;
  cfg.enabled = true;
  cfg.dynamic_geometry = true;
  cfg.heading_aware = true;
  cfg.brake_decel = 0.8;
  cfg.heading_brake_decel = 0.8;
  cfg.reaction_time = 0.2;
  cfg.sweep_samples = 8;
  cfg.sweep_max_time = 1.0;
  cfg.stop_margin = 0.05;
  cfg.side_margin = 0.02;
  cfg.turn_margin = 0.05;
  // 官方图纸: 总长 906.23 / 车宽 676 / 驱动轮到后端 261.74
  cfg.footprint = {{-0.262, -0.338}, {0.644, -0.338}, {0.644, 0.338}, {-0.262, 0.338}};
  // 车身排除盒 = 图纸 + 0.02 (safety_body_margin)
  cfg.body_polygon = {{-0.282, -0.358}, {0.664, -0.358}, {0.664, 0.358}, {-0.282, 0.358}};
  cfg.stop_max_points = 4;
  cfg.slowdown_max_points = 6;
  cfg.slowdown_ratio = 0.30;
  cfg.slowdown_min_speed = 0.15;
  cfg.slowdown_lookahead = 1.2;
  cfg.stop_trigger_frames = 2;
  cfg.stop_release_frames = 3;
  cfg.slowdown_trigger_frames = 1;
  cfg.slowdown_release_frames = 2;
  cfg.approach_enabled = true;
  cfg.time_before_collision = 2.0;
  cfg.simulation_time_step = 0.1;
  return cfg;
}

double envOr(const char * key, double def)
{
  const char * v = getenv(key);
  return (v != nullptr && *v != '\0') ? atof(v) : def;
}

std::unique_ptr<RppController> makeController()
{
  rosiwit_navigation::core::ControllerConfig cfg;
  cfg.max_linear_velocity = envOr("DOOR_VMAX", 0.45);
  cfg.max_angular_velocity = 0.8;
  cfg.max_accel_x = 0.8;
  cfg.max_accel_theta = 0.6;
  cfg.xy_goal_tolerance = 0.15;
  cfg.slow_down_distance = 0.0;
  cfg.min_lookahead_distance = envOr("DOOR_LA_MIN", 0.30);
  cfg.max_lookahead_distance = envOr("DOOR_LA_MAX", 0.90);
  cfg.lookahead_gain = 0.8;
  cfg.use_velocity_scaled_lookahead_dist = true;
  cfg.lookahead_time = envOr("DOOR_LA_TIME", 1.5);
  cfg.use_rotate_to_heading = true;
  cfg.rotate_to_heading_min_angle = envOr("DOOR_ROT_ANG", 0.03);
  cfg.regulated_linear_scaling_min_speed = envOr("DOOR_MIN_SPEED", 0.2);
  cfg.path_resample_step = 0.05;
  std::unique_ptr<RppController> c(new RppController());
  c->initialize(cfg);
  return c;
}

Path straightPathY(double length, double x = 0.0, double step = 0.05)
{
  Path p;
  for (double y = 0.0; y <= length + 1e-9; y += step) {
    PathPoint pp(Pose2D(x, y, M_PI / 2.0));
    pp.velocity_x = 0.25;
    p.points.push_back(pp);
  }
  return p;
}

struct DoorRun {
  bool reached{false};
  bool stopped_in_gate{false};   // 门带内出现过 STOP (= 真机上就是 FAILED_SAFETY_STOP)
  bool clipped{false};           // 车体角点真的进了门框
  double max_skew_deg{0.0};
  double max_omega_in_gate{0.0};   // 门带内发出的 |ω| (窄口 lane 应把它压到 ~0)
  double final_y{0.0};
  double worst_corner_x{0.0};    // 门带内车体角点的最大 |x| (相对门洞半宽 0.375)
};

// 车体 4 角 (世界系)
std::vector<std::pair<double, double>> bodyCorners(const Pose2D & pose)
{
  const std::vector<std::pair<double, double>> fp = {
    {-0.262, -0.338}, {0.644, -0.338}, {0.644, 0.338}, {-0.262, 0.338}};
  std::vector<std::pair<double, double>> out;
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  for (const auto & q : fp) {
    out.emplace_back(pose.x + c * q.first - s * q.second,
                     pose.y + s * q.first + c * q.second);
  }
  return out;
}

// 门框的代价地图 (0.05m): 与点云同几何, 供控制器 arcClearanceScale 使用
rosiwit_navigation::core::CostmapGrid doorCostmap()
{
  rosiwit_navigation::core::CostmapGrid g;
  const double res = 0.05;
  g.info.width = 160;      // 8 m
  g.info.height = 160;
  g.info.resolution = res;
  g.info.origin.position.x = -4.0;
  g.info.origin.position.y = -4.0;
  g.data.assign(static_cast<size_t>(g.info.width) * g.info.height, 0);
  for (unsigned r = 0; r < g.info.height; ++r) {
    const double wy = g.info.origin.position.y + (r + 0.5) * res;
    if (wy < kDoorY0 || wy > kDoorY1) { continue; }
    for (unsigned c = 0; c < g.info.width; ++c) {
      const double wx = g.info.origin.position.x + (c + 0.5) * res;
      if (std::fabs(wx) > kDoorHalf) {
        g.data[static_cast<size_t>(r) * g.info.width + c] = 100;
      }
    }
  }
  return g;
}

DoorRun runDoor(const Path & path, const Pose2D & start, double max_time = 45.0, bool trace = false)
{
  std::unique_ptr<RppController> ctrl = makeController();
  ctrl->setPath(path);
  ctrl->setLocalCostmap(doorCostmap());
  SafetyMonitor monitor;
  monitor.configure(safetyConfig());
  std::ofstream tf;
  if (trace) { tf.open("/tmp/door_trace.txt"); }
  const auto world_pts = doorSurfacePoints();

  DoorRun r;
  Pose2D pose = start;
  const double dt = 0.1;
  for (double t = 0.0; t < max_time; t += dt) {
    VelocityCommand cmd = ctrl->computeVelocityCommand(pose, VelocityCommand(0.0, 0.0, 0.0));
    if (pose.y > kDoorY0 - 0.15 && pose.y < kDoorY1 + 0.05) {
      r.max_omega_in_gate = std::max(r.max_omega_in_gate, std::fabs(cmd.angular_z));
    }
    double scale = 1.0;
    if (std::fabs(cmd.linear_x) > 1e-4 || std::fabs(cmd.angular_z) > 1e-4) {
      const SafetyResult sr = monitor.filter(cmd, toBaseFrame(world_pts, pose, 3.0));
      cmd = sr.command;
      scale = sr.scale;
      if (sr.scale <= 0.0 && pose.y > kDoorY0 - 1.0) { r.stopped_in_gate = true; }
    }
    if (trace) {
      tf << t << " y=" << pose.y << " x=" << pose.x
         << " yaw=" << pose.theta * 57.2958 << " v=" << cmd.linear_x
         << " w=" << cmd.angular_z << " sc=" << scale << "\n";
    }
    pose.x += cmd.linear_x * std::cos(pose.theta) * dt;
    pose.y += cmd.linear_x * std::sin(pose.theta) * dt;
    pose.theta += cmd.angular_z * dt;

    // 物理贴边检查: 门带内任何车体角点都不能越过门框
    for (const auto & c : bodyCorners(pose)) {
      if (c.second >= kDoorY0 && c.second <= kDoorY1) {
        r.worst_corner_x = std::max(r.worst_corner_x, std::fabs(c.first));
        if (std::fabs(c.first) > kDoorHalf) { r.clipped = true; }
      }
    }
    const double skew = std::fabs(std::atan2(std::sin(pose.theta - M_PI / 2.0),
                                             std::cos(pose.theta - M_PI / 2.0)));
    if (pose.y > kDoorY0 - 0.8 && pose.y < kDoorY1 + 0.3) {
      r.max_skew_deg = std::max(r.max_skew_deg, skew * 180.0 / M_PI);
    }
    r.final_y = pose.y;
    if (pose.y > kDoorY1 + 0.10) { r.reached = true; break; }
  }
  return r;
}

// 起点正对门洞、路径居中: 必须一路通过, 门带内不得触发 STOP, 车体不得越界
TEST(Door075, StraightCenteredPassage)
{
  const DoorRun r = runDoor(straightPathY(4.0), Pose2D(0.0, 0.0, M_PI / 2.0));
  EXPECT_FALSE(r.clipped) << "worst corner |x| = " << r.worst_corner_x;
  EXPECT_FALSE(r.stopped_in_gate) << "门带内触发 STOP (真机 = FAILED_SAFETY_STOP)";
  EXPECT_TRUE(r.reached) << "final_y = " << r.final_y
                         << " max_skew = " << r.max_skew_deg << " deg";
}

// 起点朝 +x (要转 90°): 起步必须先原地对准, 不能把 0.2m 横漂带进门口
TEST(Door075, StartMisalignedRotatesThenPasses)
{
  const DoorRun r = runDoor(straightPathY(4.0), Pose2D(0.0, 0.0, 0.0));
  EXPECT_FALSE(r.clipped) << "worst corner |x| = " << r.worst_corner_x;
  EXPECT_FALSE(r.stopped_in_gate) << "门带内触发 STOP; max_skew = " << r.max_skew_deg << " deg";
  EXPECT_TRUE(r.reached) << "final_y = " << r.final_y
                         << " max_skew = " << r.max_skew_deg << " deg";
}

// 起点带 5cm 横向偏差 (定位误差): 允许, 但必须在门口之前收敛
TEST(Door075, StartOffsetFiveCmConverges)
{
  const DoorRun r = runDoor(straightPathY(4.0), Pose2D(0.05, 0.0, M_PI / 2.0));
  EXPECT_FALSE(r.clipped) << "worst corner |x| = " << r.worst_corner_x;
  EXPECT_FALSE(r.stopped_in_gate) << "门带内触发 STOP; max_skew = " << r.max_skew_deg << " deg";
  EXPECT_TRUE(r.reached) << "final_y = " << r.final_y;
}

// 起始同时有 4cm 横偏 + 2° 航向差 (定位/对齐误差): 控制器一路在修正,
// 门带内不许触发 STOP —— 这条锁住"弧线贴墙时抑制 ω"(tight_arc_straighten):
// 否则 0.5m 行程内 ω 会把车体斜置撑到 4°+, 扫掠区 >0.375m, 必停。
TEST(Door075, StartOffsetWithHeadingErrorConverges)
{
  const DoorRun r = runDoor(straightPathY(4.0), Pose2D(0.04, 0.0, M_PI / 2.0 + 0.035));
  EXPECT_FALSE(r.clipped) << "worst corner |x| = " << r.worst_corner_x;
  EXPECT_FALSE(r.stopped_in_gate) << "门带内触发 STOP; max_skew = " << r.max_skew_deg << " deg";
  EXPECT_TRUE(r.reached) << "final_y = " << r.final_y;
}
// 窄口 lane: 门带内不许"边转边走" (ω 冻结到 lane_max_omega), 且能干净通过。
// 起点故意给 8cm 横偏 + 4° 航向差 (比真机定位误差还大), 收敛必须在 lane 前的
// 1m 里完成。
TEST(Door075, NarrowLaneFreezesTurning)
{
  const DoorRun r = runDoor(straightPathY(4.0), Pose2D(0.08, 0.0, M_PI / 2.0 + 0.07));
  EXPECT_FALSE(r.clipped) << "worst corner |x| = " << r.worst_corner_x;
  EXPECT_FALSE(r.stopped_in_gate) << "门带内触发 STOP; max_skew = " << r.max_skew_deg << " deg";
  EXPECT_TRUE(r.reached) << "final_y = " << r.final_y;
  EXPECT_LT(r.max_omega_in_gate, 0.05)
      << "门带内仍在转向 omega=" << r.max_omega_in_gate << " (lane 冻结失效)";
}

// 参数扫描用: 打印严苛起步 (4cm 横偏 + 2° 航向差) 下门带内的斜置/越界指标。
// 由 DOOR_* 环境变量驱动, 只打印不断言 (供离线台秒级扫参)。
TEST(Door075, PrintMetricsForSweep)
{
  const DoorRun r = runDoor(straightPathY(4.0), Pose2D(0.04, 0.0, M_PI / 2.0 + 0.035));
  printf("SWEEP reached=%d stopped=%d clipped=%d skew=%.2f worst_x=%.4f y=%.3f\n",
         static_cast<int>(r.reached), static_cast<int>(r.stopped_in_gate),
         static_cast<int>(r.clipped), r.max_skew_deg, r.worst_corner_x, r.final_y);
  EXPECT_TRUE(true);
}
}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
