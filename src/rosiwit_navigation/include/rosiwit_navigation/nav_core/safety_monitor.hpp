#ifndef ROSIWIT_NAVIGATION__NAV_CORE__SAFETY_MONITOR_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__SAFETY_MONITOR_HPP_

#include <string>
#include <utility>
#include <vector>

#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

struct SafetyMonitorConfig
{
  bool enabled = true;
  std::vector<std::pair<double, double>> stop_polygon;
  double stop_radius = 0.0;  // >0: 用圆形安全区, 优先于 polygon
  int stop_max_points = 2;
  std::vector<std::pair<double, double>> slowdown_polygon;
  int slowdown_max_points = 2;
  double slowdown_ratio = 0.35;
  bool approach_enabled = true;
  double time_before_collision = 2.0;
  double simulation_time_step = 0.1;
  std::vector<std::pair<double, double>> footprint;
};

struct SafetyResult
{
  VelocityCommand command;
  double scale = 1.0;
  std::string state = "PASS";
};

/// ROS-independent collision monitor, equivalent to Nav2 Collision Monitor's
/// STOP / SLOWDOWN / APPROACH behaviour.
class SafetyMonitor
{
public:
  void configure(const SafetyMonitorConfig & config) { config_ = config; }

  const SafetyMonitorConfig & config() const { return config_; }

  SafetyResult filter(
    const VelocityCommand & cmd,
    const std::vector<std::pair<double, double>> & points) const;

private:
  SafetyMonitorConfig config_;
};

bool pointInPolygon(
  double x, double y,
  const std::vector<std::pair<double, double>> & polygon);

int countPointsInPolygon(
  const std::vector<std::pair<double, double>> & points,
  const std::vector<std::pair<double, double>> & polygon);

double approachScale(
  const std::vector<std::pair<double, double>> & points,
  double vx, double vy, double wz,
  const std::vector<std::pair<double, double>> & footprint,
  double max_time, double time_step);

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__SAFETY_MONITOR_HPP_
