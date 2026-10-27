#include "rosiwit_navigation/nav_core/safety_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace rosiwit_navigation
{
namespace core
{

namespace
{

/// 滞回更新 (nav2 的 trigger/release_consecutive_points 等价物).
bool updateHysteresis(
  bool & active, int & trigger, int & release, bool hit,
  int trigger_frames, int release_frames)
{
  const int tf = std::max(1, trigger_frames);
  const int rf = std::max(1, release_frames);
  if (active) {
    if (hit) {
      release = 0;
    } else if (++release >= rf) {
      active = false;
      release = 0;
    }
  } else if (hit) {
    if (++trigger >= tf) {
      active = true;
      trigger = 0;
    }
  } else {
    trigger = 0;
  }
  return active;
}

}  // namespace

bool pointInPolygon(
  double x, double y,
  const std::vector<std::pair<double, double>> & polygon)
{
  if (polygon.size() < 3) return false;
  bool inside = false;
  const size_t n = polygon.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = polygon[i].first;
    const double yi = polygon[i].second;
    const double xj = polygon[j].first;
    const double yj = polygon[j].second;
    const bool crosses = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (crosses) inside = !inside;
  }
  return inside;
}

int countPointsInPolygon(
  const std::vector<std::pair<double, double>> & points,
  const std::vector<std::pair<double, double>> & polygon)
{
  if (polygon.size() < 3) return 0;
  int count = 0;
  for (const auto & p : points) {
    if (pointInPolygon(p.first, p.second, polygon)) ++count;
  }
  return count;
}

double approachScale(  const std::vector<std::pair<double, double>> & points,
  double vx, double vy, double wz,
  const std::vector<std::pair<double, double>> & footprint,
  double max_time, double time_step)
{
  if (max_time <= 0.0 || time_step <= 0.0 || footprint.size() < 3) return 1.0;
  if (std::hypot(vx, vy) < 1e-6 && std::abs(wz) < 1e-6) return 1.0;
  if (points.empty()) return 1.0;

  double radius = 0.0;
  for (const auto & p : footprint) {
    radius = std::max(radius, std::hypot(p.first, p.second));
  }
  const double max_range = std::hypot(vx, vy) * max_time + radius + 0.2;

  std::vector<std::pair<double, double>> candidates;
  candidates.reserve(points.size());
  for (const auto & p : points) {
    if (std::hypot(p.first, p.second) <= max_range) candidates.push_back(p);
  }
  if (candidates.empty()) return 1.0;
  if (candidates.size() > 2000) candidates.resize(2000);

  const int steps = std::max(1, static_cast<int>(std::ceil(max_time / time_step)));
  double x = 0.0, y = 0.0, theta = 0.0;
  for (int step = 1; step <= steps; ++step) {
    x += (vx * std::cos(theta) - vy * std::sin(theta)) * time_step;
    y += (vx * std::sin(theta) + vy * std::cos(theta)) * time_step;
    theta += wz * time_step;
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    for (const auto & p : candidates) {
      const double dx = p.first - x;
      const double dy = p.second - y;
      const double bx = dx * c + dy * s;
      const double by = -dx * s + dy * c;
      if (pointInPolygon(bx, by, footprint)) {
        return std::max(0.0, std::min(1.0, (step * time_step) / max_time));
      }
    }
  }
  return 1.0;
}

SafetyResult SafetyMonitor::filter(
  const VelocityCommand & cmd,
  const std::vector<std::pair<double, double>> & points) const
{
  SafetyResult result;
  result.command = cmd;

  if (!config_.enabled) {
    result.state = "DISABLED";
    return result;
  }

  // 1) 车身排除: 只丢掉车身轮廓内的点 (nav2 exclusion_zones.robot_body).
  //    以前是在 node 里用一个大矩形过滤, 结果把安全区一起吞了.
  const auto & body = config_.body_polygon;
  std::vector<std::pair<double, double>> pts;
  if (body.size() < 3) {
    pts = points;
  } else {
    pts.reserve(points.size());
    for (const auto & p : points) {
      if (!pointInPolygon(p.first, p.second, body)) pts.push_back(p);
    }
  }

  // nav2 VelocityPolygon 的语义: rotation 区只在"真的在转"时生效
  // (|v| 小 **且** |w| 明显).
  // ⚠ 2026-10-28: 以前**区选择**只看 |v|<0.05, 而半径分支才看 |w|, 于是
  // "命令为 0(停车/等待/规划中)"时也按旋转区判定 —— 侧后方的墙会命中
  // 旋转矩形并锁存 STOP, 2 s 后误判 blocked 触发 recovery, 最终 FAILED_SAFETY_STOP。
  // 现在两处用同一个 predicate。
  const bool rotating = std::fabs(cmd.linear_x) < 0.05 &&
                        std::fabs(cmd.angular_z) > 0.05;

  // 2) 按运动状态选 STOP 区: 原地旋转 -> 旋转区; 后退 -> 后退区; 否则前进区.
  const std::vector<std::pair<double, double>> * stop_poly = &config_.stop_polygon;
  if (rotating && !config_.stop_polygon_rotation.empty()) {
    stop_poly = &config_.stop_polygon_rotation;
  } else if (cmd.linear_x < 0.0 && !config_.stop_polygon_backward.empty()) {
    stop_poly = &config_.stop_polygon_backward;
  }
  int stop_count = 0;
  double stop_radius = config_.stop_radius;
  if (rotating && config_.stop_radius_rotation > 0.0) {
    stop_radius = config_.stop_radius_rotation;   // 原地转: 用扫掠圆
  }
  if (stop_radius > 0.0) {
    const double r2 = stop_radius * stop_radius;
    for (const auto & p : pts) {
      if (p.first * p.first + p.second * p.second <= r2) ++stop_count;
    }
  } else {
    stop_count = countPointsInPolygon(pts, *stop_poly);
  }

  // 3) 滞回: 连续 trigger 帧才进入, 连续 release 帧才释放.
  const bool stop_hit = stop_count > config_.stop_max_points;
  const bool stop_now = updateHysteresis(
    stop_active_, stop_trigger_, stop_release_, stop_hit,
    config_.stop_trigger_frames, config_.stop_release_frames);
  if (stop_now) {
    result.command = VelocityCommand{};
    result.scale = 0.0;
    result.state = "STOP";
    return result;
  }

  // 4) SLOWDOWN 同样带滞回.
  const bool slow_hit =
    countPointsInPolygon(pts, config_.slowdown_polygon) > config_.slowdown_max_points;
  const bool slow_now = updateHysteresis(
    slowdown_active_, slow_trigger_, slow_release_, slow_hit,
    config_.slowdown_trigger_frames, config_.slowdown_release_frames);

  double scale = 1.0;
  std::string state = "PASS";
  if (slow_now) {
    scale = std::min(scale, config_.slowdown_ratio);
    state = "SLOWDOWN";
  }

  if (config_.approach_enabled) {
    const double approach = approachScale(
      pts, cmd.linear_x, cmd.lateral_y, cmd.angular_z, config_.footprint,
      config_.time_before_collision, config_.simulation_time_step);
    if (approach < scale) {
      scale = approach;
      state = "APPROACH";
    }
  }

  if (scale <= 0.0) {
    result.command = VelocityCommand{};
    result.scale = 0.0;
    result.state = "STOP";
    return result;
  }

  result.command.linear_x = cmd.linear_x * scale;
  result.command.lateral_y = cmd.lateral_y * scale;
  result.command.angular_z = cmd.angular_z * scale;
  result.scale = scale;
  result.state = state;
  return result;
}

}  // namespace core
}  // namespace rosiwit_navigation
