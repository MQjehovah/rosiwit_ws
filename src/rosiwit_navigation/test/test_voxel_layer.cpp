#include <gtest/gtest.h>

#include "rosiwit_navigation/nav_core/voxel_layer.hpp"

using namespace rosiwit_navigation::core;

TEST(VoxelLayer, Projects3DPointsAndFiltersHeight)
{
  VoxelLayerConfig cfg;
  cfg.resolution = 0.1;
  cfg.min_height = 0.05;
  cfg.max_height = 1.0;
  cfg.decay_time = 0.0;
  VoxelLayer layer(cfg);
  layer.update({{1.0, 2.0, 0.5}, {1.02, 2.01, 0.52}, {1.0, 2.0, -0.5}}, 1.0);
  const auto pts = layer.occupancyPoints();
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_NEAR(pts[0].x, 1.0, 0.06);
  EXPECT_NEAR(pts[0].y, 2.0, 0.06);
}

TEST(VoxelLayer, DecayRemovesOldVoxels)
{
  VoxelLayerConfig cfg;
  cfg.resolution = 0.2;
  cfg.decay_time = 0.5;
  VoxelLayer layer(cfg);
  layer.update({{1.0, 1.0, 0.5}}, 1.0);
  EXPECT_EQ(layer.size(), 1u);
  layer.prune(1.6);
  EXPECT_EQ(layer.size(), 0u);
}
