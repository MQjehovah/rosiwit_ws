// ============================================================================
// test_distance_field.cpp — 精确欧氏距离变换 (替换曼哈顿 BFS)
//   与暴力解逐格比对, 保证"外接圆快速放行/膨胀等值线"用的是真欧氏距离。
// ============================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rosiwit_navigation/nav_core/distance_field.hpp"

using namespace rosiwit_navigation::core;

namespace
{

std::vector<float> bruteForce(const std::vector<uint8_t> & lethal, int w, int h)
{
  std::vector<float> out(static_cast<size_t>(w) * h, 1e6f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      // lethal 格自身距离为 0 (暴力解也要按此约定, 否则会和实现差 1e6)
      if (lethal[static_cast<size_t>(y) * w + x]) {
        out[static_cast<size_t>(y) * w + x] = 0.0f;
        continue;
      }
      float best = 1e6f;
      for (int yy = 0; yy < h; ++yy) {
        for (int xx = 0; xx < w; ++xx) {
          if (!lethal[static_cast<size_t>(yy) * w + xx]) { continue; }
          const float dx = static_cast<float>(x - xx);
          const float dy = static_cast<float>(y - yy);
          best = std::min(best, std::sqrt(dx * dx + dy * dy));
        }
      }
      out[static_cast<size_t>(y) * w + x] = best;
    }
  }
  return out;
}

}  // namespace

TEST(DistanceField, MatchesBruteForceSingleCell)
{
  const int w = 21, h = 17;
  std::vector<uint8_t> lethal(static_cast<size_t>(w) * h, 0);
  lethal[static_cast<size_t>(8) * w + 10] = 1;
  const std::vector<float> got = euclideanDistanceCells(lethal, w, h);
  const std::vector<float> want = bruteForce(lethal, w, h);
  ASSERT_EQ(got.size(), want.size());
  for (size_t i = 0; i < got.size(); ++i) {
    EXPECT_NEAR(got[i], want[i], 1e-4f) << "idx=" << i;
  }
}

TEST(DistanceField, MatchesBruteForceWallAndBlobs)
{
  const int w = 40, h = 30;
  std::vector<uint8_t> lethal(static_cast<size_t>(w) * h, 0);
  // 一堵竖墙 + 两个方块: 覆盖斜向/直角两种最近点分布
  for (int y = 0; y < h; ++y) { lethal[static_cast<size_t>(y) * w + 5] = 1; }
  for (int y = 18; y < 24; ++y) {
    for (int x = 25; x < 31; ++x) { lethal[static_cast<size_t>(y) * w + x] = 1; }
  }
  for (int y = 3; y < 6; ++y) {
    for (int x = 33; x < 37; ++x) { lethal[static_cast<size_t>(y) * w + x] = 1; }
  }
  const std::vector<float> got = euclideanDistanceCells(lethal, w, h);
  const std::vector<float> want = bruteForce(lethal, w, h);
  for (size_t i = 0; i < got.size(); ++i) {
    EXPECT_NEAR(got[i], want[i], 1e-4f) << "idx=" << i;
  }
}

// 45° 方向必须是欧氏: 距离 (3,3) 格 = 4.243, 而曼哈顿是 6 —— 这是之前
// "外接圆放行"把角部碰撞判成安全的根因。
TEST(DistanceField, DiagonalIsEuclideanNotManhattan)
{
  const int w = 9, h = 9;
  std::vector<uint8_t> lethal(static_cast<size_t>(w) * h, 0);
  lethal[0] = 1;   // (0,0)
  const std::vector<float> got = euclideanDistanceCells(lethal, w, h);
  EXPECT_NEAR(got[static_cast<size_t>(3) * w + 3], std::sqrt(18.0f), 1e-4f);
  EXPECT_NEAR(got[static_cast<size_t>(0) * w + 4], 4.0f, 1e-4f);
}

TEST(DistanceField, NoObstacleReturnsLargeFinite)
{
  const int w = 10, h = 10;
  std::vector<uint8_t> lethal(static_cast<size_t>(w) * h, 0);
  const std::vector<float> got = euclideanDistanceCells(lethal, w, h);
  for (float d : got) {
    EXPECT_GT(d, 1e5f);
    EXPECT_LT(d, 1e7f);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
