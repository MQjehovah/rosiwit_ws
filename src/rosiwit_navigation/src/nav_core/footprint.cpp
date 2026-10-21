// ============================================================================
// footprint.cpp — 见 footprint.hpp
// ============================================================================
#include "rosiwit_navigation/nav_core/footprint.hpp"

#include <algorithm>
#include <cmath>

namespace rosiwit_navigation
{
namespace core
{

std::vector<std::pair<double, double>> sampleFootprint(const Footprint & footprint,
                                                       double spacing)
{
  std::vector<std::pair<double, double>> samples;
  if (footprint.size() < 3 || !std::isfinite(spacing) || spacing <= 0.0) {
    return samples;
  }
  const double step = std::max(0.02, spacing);

  for (size_t i = 0; i < footprint.size(); ++i) {
    const auto & from = footprint[i];
    const auto & to = footprint[(i + 1) % footprint.size()];
    const double dx = to.first - from.first;
    const double dy = to.second - from.second;
    const double length = std::hypot(dx, dy);
    const int segments = std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int k = 0; k < segments; ++k) {
      const double ratio = static_cast<double>(k) / static_cast<double>(segments);
      samples.emplace_back(from.first + ratio * dx, from.second + ratio * dy);
    }
  }
  // 中心点: 保证空 footprint 的回退行为仍然覆盖车体中心.
  samples.emplace_back(0.0, 0.0);
  return samples;
}

int costAt(const CostmapGrid & costmap, double x, double y)
{
  const auto & info = costmap.info;
  if (info.width == 0 || info.height == 0 || info.resolution <= 0.0 ||
      costmap.data.size() < static_cast<size_t>(info.width) * info.height) {
    return 0;
  }
  const int mx = static_cast<int>(std::floor((x - info.origin.position.x) / info.resolution));
  const int my = static_cast<int>(std::floor((y - info.origin.position.y) / info.resolution));
  if (mx < 0 || my < 0 ||
      mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height)) {
    return 100;
  }
  const int value = costmap.data[
    static_cast<size_t>(my) * info.width + static_cast<size_t>(mx)];
  return value < 0 ? 100 : value;
}

int footprintCost(const CostmapGrid & costmap, const Pose2D & pose,
                  const std::vector<std::pair<double, double>> & samples)
{
  if (samples.empty()) {
    return costAt(costmap, pose.x, pose.y);
  }
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  int worst = 0;
  for (const auto & sample : samples) {
    const double wx = pose.x + c * sample.first - s * sample.second;
    const double wy = pose.y + s * sample.first + c * sample.second;
    worst = std::max(worst, costAt(costmap, wx, wy));
    if (worst >= 100) { return 100; }
  }
  return worst;
}

}  // namespace core
}  // namespace rosiwit_navigation
