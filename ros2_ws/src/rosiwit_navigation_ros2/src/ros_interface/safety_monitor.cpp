#include "ros_interface/safety_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace rosiwit_navigation
{
namespace navigation
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

geometry_msgs::msg::Twist SafetyMonitor::filter(
  const geometry_msgs::msg::Twist & cmd,
  const std::vector<std::pair<double, double>> & points,
  std::string * state) const
{
  auto set_state = [state](const char * s) {
      if (state != nullptr) *state = s;
    };

  if (!config_.enabled) {
    set_state("DISABLED");
    return cmd;
  }

  const double vx = cmd.linear.x;
  const double vy = cmd.linear.y;
  const double wz = cmd.angular.z;
  if (std::abs(vx) < 1e-9 && std::abs(vy) < 1e-9 && std::abs(wz) < 1e-9) {
    set_state("PASS");
    return cmd;
  }

  if (countPointsInPolygon(points, config_.stop_polygon) >
    config_.stop_max_points)
  {
    set_state("STOP");
    return geometry_msgs::msg::Twist();
  }

  double scale = 1.0;
  const char * state_name = "PASS";

  if (countPointsInPolygon(points, config_.slowdown_polygon) >
    config_.slowdown_max_points)
  {
    scale = std::min(scale, config_.slowdown_ratio);
    state_name = "SLOWDOWN";
  }

  if (config_.approach_enabled) {
    const double approach = approachScale(
      points, vx, vy, wz, config_.footprint,
      config_.time_before_collision, config_.simulation_time_step);
    if (approach < scale) {
      scale = approach;
      state_name = "APPROACH";
    }
  }

  if (scale <= 0.0) {
    set_state("STOP");
    return geometry_msgs::msg::Twist();
  }

  set_state(state_name);
  geometry_msgs::msg::Twist out = cmd;
  out.linear.x = vx * scale;
  out.linear.y = vy * scale;
  out.linear.z = cmd.linear.z * scale;
  out.angular.x = cmd.angular.x * scale;
  out.angular.y = cmd.angular.y * scale;
  out.angular.z = wz * scale;
  return out;
}

}  // namespace navigation
}  // namespace rosiwit_navigation
