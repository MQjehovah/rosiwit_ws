#ifndef ROSIWIT_NAVIGATION__ROS_INTERFACE__SAFETY_MONITOR_HPP_
#define ROSIWIT_NAVIGATION__ROS_INTERFACE__SAFETY_MONITOR_HPP_

#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"

namespace rosiwit_navigation
{
namespace navigation
{

struct SafetyMonitorConfig
{
  bool enabled = true;
  std::vector<std::pair<double, double>> stop_polygon;
  int stop_max_points = 2;
  std::vector<std::pair<double, double>> slowdown_polygon;
  int slowdown_max_points = 2;
  double slowdown_ratio = 0.35;
  bool approach_enabled = true;
  double time_before_collision = 2.0;
  double simulation_time_step = 0.1;
  std::vector<std::pair<double, double>> footprint;
};

/// Pure, sensor-independent safety filter for cmd_vel.
///
/// The navigation node owns the LiDAR subscription and TF transform; this
/// module only receives 2D points in base_frame and returns the filtered
/// command. It mirrors Nav2 Collision Monitor's STOP / SLOWDOWN / APPROACH.
class SafetyMonitor
{
public:
  void configure(const SafetyMonitorConfig & config) { config_ = config; }

  const SafetyMonitorConfig & config() const { return config_; }

  geometry_msgs::msg::Twist filter(
    const geometry_msgs::msg::Twist & cmd,
    const std::vector<std::pair<double, double>> & points,
    std::string * state = nullptr) const;

private:
  SafetyMonitorConfig config_;
};

bool pointInPolygon(
  double x, double y,
  const std::vector<std::pair<double, double>> & polygon);

int countPointsInPolygon(
  const std::vector<std::pair<double, double>> & points,
  const std::vector<std::pair<double, double>> & polygon);

/// Velocity scale needed to keep `max_time` seconds of collision margin.
/// Returns 0 when already touching, 1 when the footprint is clear.
double approachScale(
  const std::vector<std::pair<double, double>> & points,
  double vx, double vy, double wz,
  const std::vector<std::pair<double, double>> & footprint,
  double max_time, double time_step);

}  // namespace navigation
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__ROS_INTERFACE__SAFETY_MONITOR_HPP_
