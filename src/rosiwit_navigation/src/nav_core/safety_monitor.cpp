#include "rosiwit_navigation/nav_core/safety_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace rosiwit_navigation
{
namespace core
{

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

double approachScale(
  const std::vector<std::pair<double, double>> & points,
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
  if (std::abs(cmd.linear_x) < 1e-9 && std::abs(cmd.lateral_y) < 1e-9 &&
    std::abs(cmd.angular_z) < 1e-9)
  {
    result.state = "PASS";
    return result;
  }

  if (countPointsInPolygon(points, config_.stop_polygon) >
    config_.stop_max_points)
  {
    result.command = VelocityCommand{};
    result.scale = 0.0;
    result.state = "STOP";
    return result;
  }

  double scale = 1.0;
  std::string state = "PASS";

  if (countPointsInPolygon(points, config_.slowdown_polygon) >
    config_.slowdown_max_points)
  {
    scale = std::min(scale, config_.slowdown_ratio);
    state = "SLOWDOWN";
  }

  if (config_.approach_enabled) {
    const double approach = approachScale(
      points, cmd.linear_x, cmd.lateral_y, cmd.angular_z, config_.footprint,
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
