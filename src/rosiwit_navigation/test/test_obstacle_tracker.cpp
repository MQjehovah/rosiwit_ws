#include <gtest/gtest.h>

#include "rosiwit_navigation/nav_core/obstacle_tracker.hpp"

using namespace rosiwit_navigation::core;

TEST(ObstacleTracker, TracksMovingClusterAndPredicts)
{
  ObstacleTrackerConfig cfg;
  cfg.confirm_hits = 2;
  cfg.cluster_distance = 0.25;
  cfg.association_distance = 1.0;
  ObstacleTracker tracker(cfg);

  std::vector<Point2D> frame{{1.0, 2.0}, {1.05, 2.02}, {1.02, 1.98}};
  tracker.update(frame, 1.0, 0.1);
  for (auto & p : frame) p.x += 0.3;
  tracker.update(frame, 1.1, 0.1);

  const auto tracked = tracker.tracked();
  ASSERT_EQ(tracked.size(), 1u);
  EXPECT_GT(tracked[0].vx, 1.0);
  EXPECT_NEAR(tracked[0].x, 1.3, 0.05);
  const auto predicted = tracker.predicted(0.5);
  ASSERT_EQ(predicted.size(), 1u);
  EXPECT_GT(predicted[0].x, tracked[0].x + 0.3);
}

TEST(ObstacleTracker, DropsMissedTracks)
{
  ObstacleTrackerConfig cfg;
  cfg.confirm_hits = 1;
  cfg.max_misses = 2;
  ObstacleTracker tracker(cfg);
  tracker.update({{1.0, 1.0}}, 1.0, 0.1);
  EXPECT_EQ(tracker.tracked().size(), 1u);
  tracker.update({}, 1.1, 0.1);
  tracker.update({}, 1.2, 0.1);
  tracker.update({}, 1.3, 0.1);
  EXPECT_TRUE(tracker.tracked(false).empty());
}

TEST(ObstacleTracker, DoesNotPredictStaticCluster)
{
  ObstacleTrackerConfig cfg;
  cfg.confirm_hits = 1;
  ObstacleTracker tracker(cfg);
  const std::vector<Point2D> static_cluster{{2.0, 1.0}, {2.05, 1.02}, {1.98, 0.99}};
  tracker.update(static_cluster, 1.0, 0.1);
  tracker.update(static_cluster, 1.1, 0.1);
  EXPECT_EQ(tracker.tracked().size(), 1u);
  // 静态货架/墙段仍可由静态地图表达, 但不能生成动态预测圆.
  EXPECT_TRUE(tracker.predicted(0.5).empty());
}
