// 导航质量体检 (离线, 无 ROS):
//   1) 横向跟踪: 直线 + 初始横偏 0.30m, 各控制器闭环 6s, 统计 max/末段横向误差
//   2) 窄道规划: 两道墙留 0.80m 通道, 试不同 planner footprint / robot_radius
// 用法: nav_quality_check
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "rosiwit_navigation/nav_core/i_controller.hpp"
#include "rosiwit_navigation/nav_core/i_planner.hpp"
#include "rosiwit_navigation/nav_core/navigation_factory.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

using namespace rosiwit_navigation;
using core::ControllerConfig;
using nav_core::NavigationFactory;
using core::Costmap;
using core::CostmapGrid;
using core::Path;
using core::PathPoint;
using core::PlannerConfig;
using core::Pose2D;
using core::VelocityCommand;

namespace
{

Path MakeStraight(double length, double step)
{
  Path p;
  for (double x = 0.0; x <= length + 1e-9; x += step) {
    PathPoint pt;
    pt.pose = Pose2D(x, 0.0, 0.0);
    pt.velocity_x = 1.0;      // 前进
    p.points.push_back(pt);
  }
  return p;
}

void CrosstrackTest(const std::string & name, double vmax, double lookahead,
                    double max_lookahead, double min_lookahead)
{
  auto ctrl = nav_core::NavigationFactory::createController(name);
  if (!ctrl) {
    std::printf("%-14s  (factory 不支持)\n", name.c_str());
    return;
  }
  ControllerConfig cfg;
  cfg.lookahead_distance = lookahead;
  cfg.min_lookahead_distance = min_lookahead;
  cfg.max_lookahead_distance = max_lookahead;
  cfg.lookahead_gain = 1.0;
  cfg.xy_goal_tolerance = 0.12;
  cfg.slow_down_distance = 0.3;
  cfg.max_linear_velocity = vmax;
  cfg.max_angular_velocity = 0.8;
  cfg.kinematics.max_velocity_x = vmax;
  cfg.kinematics.max_velocity_theta = 0.8;
  cfg.max_accel_x = 0.8;
  cfg.max_accel_theta = 0.8;
  cfg.path_resample_step = 0.08;

  // 空的世界: 全自由 10m x 10m
  Costmap cm;
  auto grid = std::make_shared<CostmapGrid>();
  grid->info.width = 200;
  grid->info.height = 200;
  grid->info.resolution = 0.05;
  grid->info.origin.position.x = -5.0;
  grid->info.origin.position.y = -5.0;
  grid->data.assign(200 * 200, 0);
  cm.grid = grid;
  cm.data = grid->data;

  ctrl->initialize(cfg);
  ctrl->setPath(MakeStraight(4.0, 0.05));
  ctrl->setLocalCostmap(*grid);

  Pose2D pose(0.0, 0.30, 0.0);          // 初始横偏 0.30m
  VelocityCommand cmd;
  double max_err = 0.0;
  double sum_err_tail = 0.0;
  int tail_n = 0;
  const double dt = 0.1;
  for (int i = 0; i < 60; ++i) {        // 6s
    cmd = ctrl->computeVelocityCommand(pose, cmd);
    // 简单运动学积分 (差速)
    pose.x += cmd.linear_x * std::cos(pose.theta) * dt;
    pose.y += cmd.linear_x * std::sin(pose.theta) * dt;
    pose.theta += cmd.angular_z * dt;
    const double err = std::fabs(pose.y);
    max_err = std::max(max_err, err);
    if (i >= 30) { sum_err_tail += err; ++tail_n; }
  }
  std::printf("%-14s vmax=%.2f lookahead=%.2f/%.2f  max|y|=%.3f  tail|y|=%.3f  "
              "end=(%.2f,%.2f)\n", name.c_str(), vmax, lookahead, max_lookahead,
              max_err, tail_n ? sum_err_tail / tail_n : 0.0, pose.x, pose.y);
}

Costmap MakeCorridor(double gap)
{
  // 10m x 6m; x=4.0..4.2 是一道**贯通整幅地图**的墙, 只在 y∈[3-gap/2, 3+gap/2]
  // 留门洞 -> 唯一通路就是那个 gap. (第一版墙没贯通, 规划器直接绕着墙走, 测不出来)
  const double res = 0.05;
  const int w = 200;
  const int h = 120;
  Costmap cm;
  auto grid = std::make_shared<CostmapGrid>();
  grid->info.width = w;
  grid->info.height = h;
  grid->info.resolution = res;
  grid->info.origin.position.x = 0.0;
  grid->info.origin.position.y = 0.0;
  grid->data.assign(w * h, 0);
  const double y_lo = 3.0 - gap / 2.0;
  const double y_hi = 3.0 + gap / 2.0;
  for (int iy = 0; iy < h; ++iy) {
    const double y = (iy + 0.5) * res;
    if (y > y_lo && y < y_hi) { continue; }        // 门洞
    for (int ix = 80; ix < 84; ++ix) {             // 墙 x = 4.00..4.20, 贯通
      grid->data[iy * w + ix] = 100;
    }
  }
  cm.grid = grid;
  cm.data = grid->data;
  return cm;
}

core::Footprint Grow(const core::Footprint & fp, double m)
{
  core::Footprint out;
  for (const auto & v : fp) {
    const double s = std::hypot(v.first, v.second);
    if (s < 1e-6) { out.push_back(v); continue; }
    out.push_back({v.first + m * v.first / s, v.second + m * v.second / s});
  }
  return out;
}

void NarrowTest(const char * planner, double gap, const core::Footprint & fp,
                double robot_radius, bool use_fp, double inflation = 0.0)
{
  Costmap cm = MakeCorridor(gap);
  auto p = nav_core::NavigationFactory::createPlanner(planner);
  if (!p) { std::printf("%-14s (factory 不支持)\n", planner); return; }
  PlannerConfig pc;
  pc.name = planner;
  pc.resolution = 0.05;
  pc.max_iterations = 800000;
  pc.timeout = 15.0;
  pc.allow_reverse = true;
  p->initialize(pc);
  p->setCostmap(cm);
  p->setRobotRadius(robot_radius);
  p->setInflationRadius(inflation);
  if (use_fp && p->supportsFootprint()) { p->setRobotFootprint(fp); }

  const Pose2D start(1.0, 3.0, 0.0);
  const Pose2D goal(8.0, 3.0, 0.0);
  const auto res = p->plan(start, goal);
  if (res.is_ok()) {
    std::printf("  gap=%.2f fp=%s r=%.2f infl=%.2f -> OK   (%zu pts)\n", gap,
                use_fp ? "yes" : "no", robot_radius, inflation, res.value().points.size());
  } else {
    std::printf("  gap=%.2f fp=%s r=%.2f infl=%.2f -> FAIL (%s)\n", gap,
                use_fp ? "yes" : "no", robot_radius, inflation,
                res.error_message().c_str());
  }
}

}  // namespace

int main()
{
  std::printf("=== 1) 横向跟踪 (直线, 初始横偏 0.30m) ===\n");
  CrosstrackTest("pure_pursuit", 0.25, 0.35, 0.70, 0.25);
  CrosstrackTest("rpp", 0.25, 0.35, 0.70, 0.25);
  CrosstrackTest("dwa", 0.25, 0.35, 0.70, 0.25);
  CrosstrackTest("mppi", 0.25, 0.35, 0.70, 0.25);

  std::printf("\n=== 2) 窄道规划 (通道 0.80m / 0.70m, 车宽 0.66m) ===\n");
  // 实车轮廓
  core::Footprint real{{-0.25, -0.33}, {0.45, -0.33}, {0.55, -0.20},
                       {0.55, 0.20}, {0.45, 0.33}, {-0.25, 0.33}};
  const core::Footprint grown = Grow(real, 0.06);   // planner_footprint_margin
  for (double gap : {0.80, 0.70}) {
    std::printf("hybrid_astar:\n");
    NarrowTest("hybrid_astar", gap, grown, 0.39, true);
    NarrowTest("hybrid_astar", gap, real, 0.39, true);
    NarrowTest("hybrid_astar", gap, real, 0.33, true);
    NarrowTest("hybrid_astar", gap, grown, 0.39, false);
    std::printf("hybrid_astar + inflation 0.62 (实车配置):\n");
    NarrowTest("hybrid_astar", gap, grown, 0.39, true, 0.62);
    NarrowTest("hybrid_astar", gap, real, 0.33, true, 0.62);
    std::printf("astar + inflation 0.62:\n");
    NarrowTest("astar", gap, grown, 0.39, false, 0.62);
    NarrowTest("astar", gap, real, 0.39, false, 0.62);
  }
  return 0;
}
