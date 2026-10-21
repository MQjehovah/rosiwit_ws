// ============================================================================
// bench_footprint_ab.cpp — DWA/MPPI 的 footprint-aware A/B 离线基准
//
// 场景: 8m 直线路径 + 两侧障碍 + 1m 窄门; 车体用实车长方形 footprint.
// 对比四种组合:
//   dwa_point / dwa_footprint / mppi_point / mppi_footprint
// 评价:
//   reached    是否到达终点
//   collisions 真实 footprint 压在障碍格上的控制步数
//   min_clr    真实 footprint 到障碍的最小距离 (m)
//   lat_rms    横向跟踪误差 RMS (m)
//   avg_v      平均线速度 (m/s)
//   rms_w      角速度 RMS (rad/s)
//   avg_ms     单次 computeVelocityCommand 平均耗时 (ms)
//
// 编译 (机器人上):
//   g++ -std=c++17 -O2 -pthread -I<core>/include bench_footprint_ab.cpp \
//       -L<build> -lrosiwit_navigation -Wl,-rpath,<build> -o bench_ab
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rosiwit_navigation/nav_core/footprint.hpp"
#include "rosiwit_navigation/nav_core/navigation_factory.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

using namespace rosiwit_navigation;

namespace
{

constexpr double kRes = 0.05;
constexpr int kWidth = 240;   // 12m
constexpr int kHeight = 160;  // 8m
constexpr double kOriginX = -1.0;
constexpr double kOriginY = -4.0;
constexpr double kDt = 0.1;

core::Footprint realFootprint()
{
  return {
    {-0.25, -0.33}, {0.45, -0.33}, {0.55, -0.20},
    {0.55, 0.20}, {0.45, 0.33}, {-0.25, 0.33}};
}

core::CostmapGrid makeMap(const std::string & scenario = "full")
{
  core::CostmapGrid grid;
  grid.info.width = kWidth;
  grid.info.height = kHeight;
  grid.info.resolution = kRes;
  grid.info.origin.position.x = kOriginX;
  grid.info.origin.position.y = kOriginY;
  grid.data.assign(static_cast<size_t>(kWidth) * kHeight, 0);

  const auto fill = [&grid](double x0, double x1, double y0, double y1) {
    for (int my = 0; my < kHeight; ++my) {
      const double y = kOriginY + (my + 0.5) * kRes;
      if (y < y0 || y > y1) { continue; }
      for (int mx = 0; mx < kWidth; ++mx) {
        const double x = kOriginX + (mx + 0.5) * kRes;
        if (x < x0 || x > x1) { continue; }
        grid.data[static_cast<size_t>(my) * kWidth + mx] = 100;
      }
    }
  };

  if (scenario == "empty") { return grid; }
  // 两侧障碍: 直线路径中心安全, 但长方形车体会压到
  fill(2.40, 2.80, 0.25, 0.55);
  if (scenario == "one_block") { return grid; }
  fill(5.20, 5.60, -0.55, -0.25);
  if (scenario == "no_gate") { return grid; }
  // 1.5m 门 (x=4.0~4.2), 车宽 0.66m, 单侧余量 0.42m
  fill(4.00, 4.20, 0.75, 3.00);
  fill(4.00, 4.20, -3.00, -0.75);
  return grid;
}

core::Path makePath()
{
  core::Path path;
  for (int i = 0; i <= 160; ++i) {
    core::PathPoint p;
    p.pose = core::Pose2D(0.05 * i, 0.0, 0.0);
    path.points.push_back(p);
  }
  return path;
}

std::vector<float> buildDistanceField(const core::CostmapGrid & grid)
{
  const float inf = std::numeric_limits<float>::max();
  std::vector<float> dist(static_cast<size_t>(kWidth) * kHeight, inf);
  std::queue<std::pair<int, int>> queue;
  for (int my = 0; my < kHeight; ++my) {
    for (int mx = 0; mx < kWidth; ++mx) {
      const int v = grid.data[static_cast<size_t>(my) * kWidth + mx];
      if (v >= 100 || v < 0) {
        dist[static_cast<size_t>(my) * kWidth + mx] = 0.0f;
        queue.push({mx, my});
      }
    }
  }
  const int dx[4] = {-1, 0, 1, 0};
  const int dy[4] = {0, -1, 0, 1};
  while (!queue.empty()) {
    const auto [cx, cy] = queue.front();
    queue.pop();
    const float d = dist[static_cast<size_t>(cy) * kWidth + cx];
    for (int i = 0; i < 4; ++i) {
      const int nx = cx + dx[i];
      const int ny = cy + dy[i];
      if (nx < 0 || nx >= kWidth || ny < 0 || ny >= kHeight) { continue; }
      const size_t idx = static_cast<size_t>(ny) * kWidth + nx;
      if (d + 1.0f < dist[idx]) {
        dist[idx] = d + 1.0f;
        queue.push({nx, ny});
      }
    }
  }
  return dist;
}

double clearanceAt(const std::vector<float> & dist, double x, double y)
{
  const int mx = static_cast<int>(std::floor((x - kOriginX) / kRes));
  const int my = static_cast<int>(std::floor((y - kOriginY) / kRes));
  if (mx < 0 || mx >= kWidth || my < 0 || my >= kHeight) { return 0.0; }
  const float d = dist[static_cast<size_t>(my) * kWidth + mx];
  if (d >= std::numeric_limits<float>::max()) { return 100.0; }
  return static_cast<double>(d) * kRes;
}

struct Result
{
  std::string name;
  bool reached = false;
  int steps = 0;
  int collisions = 0;
  double min_clearance = 100.0;
  double lateral_sq_sum = 0.0;
  double speed_sum = 0.0;
  double ang_sq_sum = 0.0;
  double calc_ms = 0.0;
  double final_x = 0.0;
};

Result run(const std::string & controller_name, bool use_footprint,
           double max_accel_theta,
           const core::CostmapGrid & grid, const core::Path & path,
           const std::vector<float> & distance_field)
{
  core::ControllerConfig cfg;
  cfg.lookahead_distance = 0.6;
  cfg.xy_goal_tolerance = 0.1;
  cfg.slow_down_distance = 1.5;
  cfg.lookahead_gain = 1.0;
  cfg.max_lookahead_distance = 0.9;
  cfg.min_lookahead_distance = 0.3;
  cfg.max_linear_velocity = 0.8;
  cfg.max_angular_velocity = 0.5;
  cfg.max_accel_x = 0.3;
  cfg.max_accel_theta = max_accel_theta;
  cfg.kinematics.max_velocity_x = 0.8;
  cfg.kinematics.max_velocity_theta = 0.5;
  if (use_footprint) {
    cfg.footprint = realFootprint();
    cfg.footprint_sample_spacing = 0.05;
  }

  auto controller = nav_core::NavigationFactory::createController(controller_name);
  Result result;
  result.name = controller_name + (use_footprint ? "_footprint" : "_point");
  if (use_footprint && max_accel_theta > 0.21) {
    char suffix[32];
    std::snprintf(suffix, sizeof(suffix), "_wacc%.1f", max_accel_theta);
    result.name += suffix;
  }
  if (!controller) {
    result.name += " (NOT FOUND)";
    return result;
  }
  controller->initialize(cfg);
  controller->setPath(path);
  controller->setLocalCostmap(grid);

  const auto true_samples = core::sampleFootprint(realFootprint(), 0.05);
  core::Pose2D pose(0.0, 0.0, 0.0);
  core::VelocityCommand velocity(0.0, 0.0, 0.0);
  const core::Pose2D goal = path.points.back().pose;

  const int kMaxSteps = 800;
  double total_ms = 0.0;
  for (int step = 0; step < kMaxSteps; ++step) {
    const auto t0 = std::chrono::steady_clock::now();
    const core::VelocityCommand cmd = controller->computeVelocityCommand(pose, velocity);
    const auto t1 = std::chrono::steady_clock::now();
    total_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

    pose.x += cmd.linear_x * std::cos(pose.theta) * kDt;
    pose.y += cmd.linear_x * std::sin(pose.theta) * kDt;
    pose.theta += cmd.angular_z * kDt;
    while (pose.theta > M_PI) { pose.theta -= 2.0 * M_PI; }
    while (pose.theta < -M_PI) { pose.theta += 2.0 * M_PI; }
    velocity = cmd;
    ++result.steps;

    // 真实车体碰撞/间隙
    const int footprint_cell = core::footprintCost(grid, pose, true_samples);
    if (footprint_cell >= 100) { ++result.collisions; }
    for (const auto & sample : true_samples) {
      const double wx = pose.x + std::cos(pose.theta) * sample.first -
                        std::sin(pose.theta) * sample.second;
      const double wy = pose.y + std::sin(pose.theta) * sample.first +
                        std::cos(pose.theta) * sample.second;
      result.min_clearance = std::min(result.min_clearance, clearanceAt(distance_field, wx, wy));
    }

    const double dx = pose.x - goal.x;
    const double dy = pose.y - goal.y;
    result.lateral_sq_sum += pose.y * pose.y;
    result.speed_sum += std::max(0.0, cmd.linear_x);
    result.ang_sq_sum += cmd.angular_z * cmd.angular_z;
    result.final_x = pose.x;
    if (std::getenv("TRACE") != nullptr && result.name == "dwa_footprint" &&
        step % 20 == 0) {
      std::printf("  trace step=%d pose=(%.2f,%.2f,%.2f) cmd=(%.2f,%.2f) fp=%d\n",
                  step, pose.x, pose.y, pose.theta, cmd.linear_x, cmd.angular_z,
                  footprint_cell);
    }

    if (std::hypot(dx, dy) < 0.15) {
      result.reached = true;
      break;
    }
    if (step > 100 && std::hypot(cmd.linear_x, cmd.angular_z) < 1e-4 &&
        std::hypot(dx, dy) < 0.5) {
      break;
    }
  }
  result.calc_ms = result.steps > 0 ? total_ms / result.steps : 0.0;
  result.lateral_sq_sum /= std::max(1, result.steps);
  result.speed_sum /= std::max(1, result.steps);
  result.ang_sq_sum /= std::max(1, result.steps);
  return result;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string scenario = (argc > 1) ? argv[1] : "full";
  const core::CostmapGrid grid = makeMap(scenario);
  const core::Path path = makePath();
  const std::vector<float> distance_field = buildDistanceField(grid);

  std::printf("%-18s %7s %7s %9s %9s %8s %8s %8s %9s\n",
              "controller", "reached", "steps", "collisions", "min_clr",
              "lat_rms", "avg_v", "final_x", "avg_ms");
  struct Case { std::string name; bool footprint; double wacc; };
  for (const Case & entry : std::vector<Case>{
         {"dwa", false, 0.2}, {"dwa", true, 0.2}, {"dwa", true, 0.8},
         {"mppi", false, 0.2}, {"mppi", true, 0.2}, {"mppi", true, 0.3},
         {"mppi", true, 0.4}, {"mppi", true, 0.5}, {"mppi", true, 0.6},
         {"mppi", true, 0.8}}) {
    const Result r = run(entry.name, entry.footprint, entry.wacc, grid, path, distance_field);
    std::printf("%-18s %7d %7d %9d %9.3f %8.3f %8.3f %8.2f %9.3f\n",
                r.name.c_str(), static_cast<int>(r.reached), r.steps,
                r.collisions, r.min_clearance,
                std::sqrt(r.lateral_sq_sum), r.speed_sum, r.final_x, r.calc_ms);
  }
  return 0;
}
