// test/test_voxel_downsample.cpp
#include <gtest/gtest.h>

#include <cmath>

#include "slam_core/voxel_downsample.h"

using rosiwit_slam::voxelDownsampleRGB;

namespace {

pcl::PointXYZRGB pt(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
{
  pcl::PointXYZRGB p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.r = r;
  p.g = g;
  p.b = b;
  return p;
}

}  // namespace

TEST(VoxelDownsample, MergesPointsInSameVoxel)
{
  pcl::PointCloud<pcl::PointXYZRGB> in;
  in.push_back(pt(0.01f, 0.01f, 0.01f, 100, 20, 30));
  in.push_back(pt(0.03f, 0.03f, 0.03f, 200, 40, 50));
  pcl::PointCloud<pcl::PointXYZRGB> out;
  voxelDownsampleRGB(in, 0.1, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].x, 0.02, 1e-5);
  EXPECT_NEAR(out[0].y, 0.02, 1e-5);
  EXPECT_NEAR(out[0].z, 0.02, 1e-5);
  EXPECT_EQ(out[0].r, 150);
  EXPECT_EQ(out[0].g, 30);
  EXPECT_EQ(out[0].b, 40);
}

TEST(VoxelDownsample, KeepsDistantPointsSeparate)
{
  pcl::PointCloud<pcl::PointXYZRGB> in;
  in.push_back(pt(0.0f, 0.0f, 0.0f, 10, 10, 10));
  in.push_back(pt(1.0f, 0.0f, 0.0f, 20, 20, 20));
  pcl::PointCloud<pcl::PointXYZRGB> out;
  voxelDownsampleRGB(in, 0.1, out);
  EXPECT_EQ(out.size(), 2u);
}

TEST(VoxelDownsample, DropsNonFinitePoints)
{
  pcl::PointCloud<pcl::PointXYZRGB> in;
  in.push_back(pt(0.0f, 0.0f, 0.0f, 1, 2, 3));
  in.push_back(pt(std::nanf(""), 0.0f, 0.0f, 4, 5, 6));
  in.push_back(pt(std::numeric_limits<float>::infinity(), 0.0f, 0.0f, 7, 8, 9));
  pcl::PointCloud<pcl::PointXYZRGB> out;
  voxelDownsampleRGB(in, 0.1, out);
  EXPECT_EQ(out.size(), 1u);
}

// 回归: pcl::VoxelGrid 在这个尺度上会因 int32 索引溢出把点几乎全丢.
// 这里用一个 2 km 跨度的稀疏点云验证 int64 key 仍能保住每个体素.
TEST(VoxelDownsample, LargeExtentDoesNotCollapse)
{
  pcl::PointCloud<pcl::PointXYZRGB> in;
  const int n = 50;
  for (int i = 0; i < n; ++i) {
    in.push_back(pt(static_cast<float>(i) * 40.0f, 0.0f, 0.0f, 50, 60, 70));
  }
  pcl::PointCloud<pcl::PointXYZRGB> out;
  voxelDownsampleRGB(in, 0.05, out);
  EXPECT_EQ(out.size(), static_cast<size_t>(n));
}

TEST(VoxelDownsample, NonPositiveLeafIsCopy)
{
  pcl::PointCloud<pcl::PointXYZRGB> in;
  in.push_back(pt(0.0f, 0.0f, 0.0f, 1, 1, 1));
  in.push_back(pt(0.0f, 0.0f, 0.0f, 2, 2, 2));
  pcl::PointCloud<pcl::PointXYZRGB> out;
  voxelDownsampleRGB(in, 0.0, out);
  EXPECT_EQ(out.size(), 2u);
}
