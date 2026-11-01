// ============================================================
// int64 网格体素降采样（质心 + 颜色平均）
//
// 为什么不用 pcl::VoxelGrid：
//   它用 int32 网格索引，当 (extent/leaf)^3 超过 int32 时索引溢出，
//   结果会塌成极少几个点（比"返回原始点云"更糟，地图直接看不见）。
//   实测一段 181 s 的 NTU 序列（场景约 100 m、leaf 0.1 m）会从 25 M 点
//   塌到 3893 点。
//
// 这里每个轴 21 bit（|index| <= 2^20-1，1 cm 叶节点约 ±10 km），
// 三个轴打包进一个 int64 key，不会溢出；非有限值直接丢弃。
// ============================================================
#ifndef ROSIWIT_SLAM__VOXEL_DOWNSAMPLE_H_
#define ROSIWIT_SLAM__VOXEL_DOWNSAMPLE_H_

#include <cmath>
#include <cstdint>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rosiwit_slam {

namespace voxel_detail {

struct VoxelAccum
{
  double x = 0.0, y = 0.0, z = 0.0;
  double r = 0.0, g = 0.0, b = 0.0;
  uint32_t n = 0;
};

inline int64_t voxelKey(int64_t ix, int64_t iy, int64_t iz)
{
  constexpr int64_t kMask = (1LL << 21) - 1;
  return ((ix & kMask) << 42) | ((iy & kMask) << 21) | (iz & kMask);
}

}  // namespace voxel_detail

/// 体素降采样（每个体素取质心与平均颜色）。leaf <= 0 时原样拷贝。
inline void voxelDownsampleRGB(const pcl::PointCloud<pcl::PointXYZRGB> & in,
                               double leaf,
                               pcl::PointCloud<pcl::PointXYZRGB> & out)
{
  out.clear();
  if (leaf <= 0.0 || in.empty()) {
    out = in;
    return;
  }

  const double inv_leaf = 1.0 / leaf;
  const int64_t limit = (1LL << 20) - 1;
  std::unordered_map<int64_t, voxel_detail::VoxelAccum> voxels;
  voxels.reserve(in.size() / 4 + 1);

  for (const auto & p : in.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    const int64_t ix = static_cast<int64_t>(std::floor(p.x * inv_leaf));
    const int64_t iy = static_cast<int64_t>(std::floor(p.y * inv_leaf));
    const int64_t iz = static_cast<int64_t>(std::floor(p.z * inv_leaf));
    if (ix < -limit || ix > limit || iy < -limit || iy > limit || iz < -limit || iz > limit) {
      continue;
    }
    voxel_detail::VoxelAccum & a = voxels[voxel_detail::voxelKey(ix, iy, iz)];
    a.x += p.x;
    a.y += p.y;
    a.z += p.z;
    a.r += p.r;
    a.g += p.g;
    a.b += p.b;
    ++a.n;
  }

  out.reserve(voxels.size());
  for (const auto & kv : voxels) {
    const voxel_detail::VoxelAccum & a = kv.second;
    if (a.n == 0) {continue;}
    pcl::PointXYZRGB p;
    p.x = static_cast<float>(a.x / a.n);
    p.y = static_cast<float>(a.y / a.n);
    p.z = static_cast<float>(a.z / a.n);
    p.r = static_cast<uint8_t>(a.r / a.n);
    p.g = static_cast<uint8_t>(a.g / a.n);
    p.b = static_cast<uint8_t>(a.b / a.n);
    out.push_back(p);
  }
  out.width = static_cast<uint32_t>(out.size());
  out.height = 1;
  out.is_dense = false;
}

}  // namespace rosiwit_slam

#endif  // ROSIWIT_SLAM__VOXEL_DOWNSAMPLE_H_
