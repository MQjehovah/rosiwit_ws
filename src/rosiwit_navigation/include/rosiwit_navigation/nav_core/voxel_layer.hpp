// ============================================================
// 3D 体素/STVL 障碍层
//
// Spatio-Temporal Voxel Layer: 把 3D 点写入体素，按时间衰减，
// 再投影到 2D 局部代价地图。比纯 2D 激光层更适合斜坡、悬空障碍、
// 多层货架等场景；decay<=0 时退化为普通 3D 体素层。
// ============================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE_VOXEL_LAYER_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE_VOXEL_LAYER_HPP_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "rosiwit_navigation/nav_core/obstacle_tracker.hpp"

namespace rosiwit_navigation
{
namespace core
{

struct Point3D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct VoxelLayerConfig
{
  double resolution = 0.10;      // 体素边长
  double min_height = -0.20;     // 相对 map 的 z 过滤
  double max_height = 2.00;
  double decay_time = 2.0;       // <=0: 不衰减
  double inflation = 0.15;       // 投影到 2D 后额外膨胀
  size_t max_voxels = 500000;
};

class VoxelLayer
{
public:
  explicit VoxelLayer(const VoxelLayerConfig & config = VoxelLayerConfig());

  void setConfig(const VoxelLayerConfig & config) { config_ = config; }
  const VoxelLayerConfig & config() const { return config_; }

  /// 插入一帧 map 系 3D 点。
  void update(const std::vector<Point3D> & points, double stamp);
  /// 删除过期体素。
  void prune(double now);
  /// 投影为 2D 障碍中心点（每个体素一个）。
  std::vector<Point2D> occupancyPoints() const;
  /// 当前体素数 / 时间范围，供诊断。
  size_t size() const { return voxels_.size(); }
  void clear();

private:
  struct Voxel
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double stamp = 0.0;
    int hits = 0;
  };

  int64_t keyOf(int ix, int iy, int iz) const;
  void indexOf(double x, double y, double z, int & ix, int & iy, int & iz) const;

  VoxelLayerConfig config_;
  std::unordered_map<int64_t, Voxel> voxels_;
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE_VOXEL_LAYER_HPP_
