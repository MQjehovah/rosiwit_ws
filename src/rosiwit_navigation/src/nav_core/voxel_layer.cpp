#include "rosiwit_navigation/nav_core/voxel_layer.hpp"

#include <algorithm>
#include <cmath>

namespace rosiwit_navigation
{
namespace core
{

VoxelLayer::VoxelLayer(const VoxelLayerConfig & config)
  : config_(config)
{
}

int64_t VoxelLayer::keyOf(int ix, int iy, int iz) const
{
  // 21-bit 有符号坐标打包；覆盖 +/-1e5 个体素，足够局部/城市级地图。
  const int64_t mask = 0x1fffff;
  return ((static_cast<int64_t>(ix) & mask) << 42) |
         ((static_cast<int64_t>(iy) & mask) << 21) |
         (static_cast<int64_t>(iz) & mask);
}

void VoxelLayer::indexOf(double x, double y, double z, int & ix, int & iy, int & iz) const
{
  const double r = std::max(0.01, config_.resolution);
  ix = static_cast<int>(std::floor(x / r));
  iy = static_cast<int>(std::floor(y / r));
  iz = static_cast<int>(std::floor(z / r));
}

void VoxelLayer::update(const std::vector<Point3D> & points, double stamp)
{
  prune(stamp);
  if (points.empty()) return;

  const double r = std::max(0.01, config_.resolution);
  for (const auto & p : points) {
    if (p.z < config_.min_height || p.z > config_.max_height) continue;
    int ix = 0, iy = 0, iz = 0;
    indexOf(p.x, p.y, p.z, ix, iy, iz);
    const int64_t key = keyOf(ix, iy, iz);
    auto it = voxels_.find(key);
    if (it == voxels_.end()) {
      if (voxels_.size() >= config_.max_voxels) continue;
      Voxel v;
      v.x = (ix + 0.5) * r;
      v.y = (iy + 0.5) * r;
      v.z = (iz + 0.5) * r;
      v.stamp = stamp;
      v.hits = 1;
      voxels_.emplace(key, v);
    } else {
      it->second.stamp = stamp;
      ++it->second.hits;
    }
  }
}

void VoxelLayer::prune(double now)
{
  if (config_.decay_time <= 0.0) return;
  const double cutoff = now - config_.decay_time;
  for (auto it = voxels_.begin(); it != voxels_.end();) {
    if (it->second.stamp < cutoff) it = voxels_.erase(it);
    else ++it;
  }
}

std::vector<Point2D> VoxelLayer::occupancyPoints() const
{
  std::vector<Point2D> out;
  out.reserve(voxels_.size());
  for (const auto & entry : voxels_) {
    Point2D p;
    p.x = entry.second.x;
    p.y = entry.second.y;
    out.push_back(p);
  }
  return out;
}

void VoxelLayer::clear()
{
  voxels_.clear();
}

}  // namespace core
}  // namespace rosiwit_navigation
