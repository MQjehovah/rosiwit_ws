#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rosiwit_navigation/nav_core/path_centering.hpp"

namespace
{
using rosiwit_navigation::core::centerPathAlongClearance;
using rosiwit_navigation::core::CenteringConfig;
using rosiwit_navigation::core::CostmapGrid;
using rosiwit_navigation::core::Footprint;
using rosiwit_navigation::core::ObstacleDistance;
using rosiwit_navigation::core::Path;
using rosiwit_navigation::core::PathPoint;
using rosiwit_navigation::core::Pose2D;

// 沿 +x 的走廊: 两侧墙在 y=±0.8, 走道宽 1.6 m
CostmapGrid makeCorridor()
{
  const double res = 0.05;
  CostmapGrid g;
  g.info.width = 100;      // 5.0 m
  g.info.height = 60;      // 3.0 m
  g.info.resolution = res;
  g.info.origin.position.x = -1.0;
  g.info.origin.position.y = -1.5;
  g.data.assign(static_cast<size_t>(g.info.width) * g.info.height, 0);
  for (unsigned r = 0; r < g.info.height; ++r) {
    const double wy = g.info.origin.position.y + (r + 0.5) * res;
    if (wy > 0.9 || wy < -0.9) {
      continue;
    }
  }
  // 墙: |y| in [0.5, 0.6]
  for (unsigned r = 0; r < g.info.height; ++r) {
    const double wy = g.info.origin.position.y + (r + 0.5) * res;
    if (wy >= 0.8 && wy <= 0.9) {
      for (unsigned c = 0; c < g.info.width; ++c) {
        g.data[static_cast<size_t>(r) * g.info.width + c] = 100;
      }
    }
    if (wy <= -0.8 && wy >= -0.9) {
      for (unsigned c = 0; c < g.info.width; ++c) {
        g.data[static_cast<size_t>(r) * g.info.width + c] = 100;
      }
    }
  }
  // 端墙 (让两端也有边界, 避免 outside window)
  for (unsigned r = 0; r < g.info.height; ++r) {
    for (unsigned c = 0; c < 2; ++c) {
      g.data[static_cast<size_t>(r) * g.info.width + c] = 100;
      g.data[static_cast<size_t>(r) * g.info.width + (g.info.width - 1 - c)] = 100;
    }
  }
  return g;
}

Path makeStraightPath(double y, double x0, double x1, double step)
{
  Path p;
  for (double x = x0; x <= x1 + 1e-9; x += step) {
    Pose2D q;
    q.x = x;
    q.y = y;
    q.theta = 0.0;
    PathPoint pp(q);
    pp.velocity_x = 1.0;
    p.points.push_back(pp);
  }
  return p;
}

Footprint makeFootprint() { return {{-0.25, -0.33}, {0.55, -0.33}, {0.55, 0.33}, {-0.25, 0.33}}; }

// 生产改动会让这些测试失败的情形:
//   * 间隙梯度符号写反 -> 路径被推到更贴墙的一侧 (OffCenterIsPushedToCenter)
//   * 没有 max_shift 限幅 -> MaxShiftIsRespected 失败 (会被推到对侧墙)
//   * 没固定起点终点 -> StartAndGoalFixed 失败
TEST(PathCentering, OffCenterIsPushedToCenter)
{
  CostmapGrid g = makeCorridor();
  ObstacleDistance esdf;
  esdf.update(g);

  Path path = makeStraightPath(0.30, 0.0, 3.0, 0.1);   // 偏 +y 0.30 (仍能容下车体)
  const double before = esdf.footprintClearance(Pose2D(1.5, 0.30, 0.0), {{0, 0}});
  CenteringConfig cc;
  cc.min_clearance = 0.25;   // 0.30 处间隙 0.17, 低于期望 -> 应被推向中心
  cc.max_shift = 0.40;       // 偏移 0.30, 需要允许足够位移才能回到中心
  const bool moved = centerPathAlongClearance(&path, esdf, makeFootprint(), cc);
  EXPECT_TRUE(moved);

  // 中间点应被推到接近中心线 (|y| < 0.08), 且间隙变大
  const Pose2D mid = path.points[path.points.size() / 2].pose;
  EXPECT_LT(std::fabs(mid.y), 0.10) << "y=" << mid.y;
  const double after = esdf.footprintClearance(mid, {{0, 0}});
  EXPECT_GT(after, before);
}

TEST(PathCentering, CenterlineStaysPut)
{
  CostmapGrid g = makeCorridor();
  ObstacleDistance esdf;
  esdf.update(g);
  Path path = makeStraightPath(0.0, 0.0, 3.0, 0.1);
  CenteringConfig cc;
  centerPathAlongClearance(&path, esdf, makeFootprint(), cc);
  for (size_t i = 1; i + 1 < path.points.size(); ++i) {
    EXPECT_LT(std::fabs(path.points[i].pose.y), 0.05) << "i=" << i;
  }
}

TEST(PathCentering, StartAndGoalAreFixed)
{
  CostmapGrid g = makeCorridor();
  ObstacleDistance esdf;
  esdf.update(g);
  Path path = makeStraightPath(0.25, 0.0, 3.0, 0.1);
  const Pose2D first = path.points.front().pose;
  const Pose2D last = path.points.back().pose;
  CenteringConfig cc;
  centerPathAlongClearance(&path, esdf, makeFootprint(), cc);
  EXPECT_NEAR(path.points.front().pose.x, first.x, 1e-9);
  EXPECT_NEAR(path.points.front().pose.y, first.y, 1e-9);
  EXPECT_NEAR(path.points.back().pose.x, last.x, 1e-9);
  EXPECT_NEAR(path.points.back().pose.y, last.y, 1e-9);
}

TEST(PathCentering, MaxShiftIsRespected)
{
  CostmapGrid g = makeCorridor();
  ObstacleDistance esdf;
  esdf.update(g);
  Path path = makeStraightPath(0.25, 0.0, 3.0, 0.1);
  CenteringConfig cc;
  cc.max_shift = 0.05;      // 故意只允许 5 cm
  centerPathAlongClearance(&path, esdf, makeFootprint(), cc);
  for (size_t i = 0; i < path.points.size(); ++i) {
    const double d = std::hypot(path.points[i].pose.x - (0.1 * static_cast<double>(i)),
                                path.points[i].pose.y - 0.25);
    EXPECT_LE(d, cc.max_shift + 1e-6) << "i=" << i;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
