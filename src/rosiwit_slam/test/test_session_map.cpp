// ============================================================================
// test_session_map.cpp — 突变场景滚动子图（B 阶段）
//
// 覆盖：未激活 / 激活后累积 / anchor 变换往返 / 半径丢弃 / 体素去重 / 关闭清空
// ============================================================================
#include <gtest/gtest.h>

#include <cmath>

#include "algorithms/localization/local_map/session_map.h"

using namespace rosiwit_slam;

namespace {

CloudType::Ptr makeCloud(const std::vector<Eigen::Vector3d>& pts) {
  CloudType::Ptr c(new CloudType());
  for (const auto& v : pts) {
    PointType p;
    p.x = static_cast<float>(v.x());
    p.y = static_cast<float>(v.y());
    p.z = static_cast<float>(v.z());
    p.intensity = 0.0f;
    c->push_back(p);
  }
  c->width = static_cast<uint32_t>(c->size());
  c->height = 1;
  return c;
}

PoseStamped makePose(double x, double y = 0.0, double yaw = 0.0) {
  PoseStamped p;
  p.time = 1.0;
  p.trans = Eigen::Vector3d(x, y, 0.0);
  p.rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return p;
}

}  // namespace

TEST(SessionMap, InactiveByDefault) {
  SessionMap sm;
  EXPECT_FALSE(sm.active());
  EXPECT_EQ(sm.size(), 0u);
  auto world = sm.pointsWorld();
  ASSERT_TRUE(world);
  EXPECT_TRUE(world->empty());
}

TEST(SessionMap, ResetActivatesAndAccumulates) {
  SessionMap sm;
  sm.configure(SessionMap::Config{25.0, 0.15, 1000});
  sm.reset(makePose(0.0), 10.0);
  EXPECT_TRUE(sm.active());

  auto cloud = makeCloud({{1.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, 1.0}});
  sm.update(*cloud, makePose(0.0));
  EXPECT_EQ(sm.size(), 3u);

  auto world = sm.pointsWorld();
  ASSERT_TRUE(world);
  ASSERT_EQ(world->size(), 3u);
}

TEST(SessionMap, AnchorTransformRoundTrip) {
  SessionMap sm;
  sm.configure(SessionMap::Config{50.0, 0.15, 1000});
  // 锚在 (10,0,0)，机器人也在 (10,0,0)
  sm.reset(makePose(10.0), 0.0);

  // 机体系点 (1,0,0) -> 世界 (11,0,0) -> anchor 系 (1,0,0)
  auto cloud = makeCloud({{1.0, 0.0, 0.0}});
  sm.update(*cloud, makePose(10.0));
  ASSERT_EQ(sm.size(), 1u);

  auto world = sm.pointsWorld();
  ASSERT_TRUE(world);
  ASSERT_EQ(world->size(), 1u);
  EXPECT_NEAR(world->points[0].x, 11.0f, 1e-3);
  EXPECT_NEAR(world->points[0].y, 0.0f, 1e-3);
}

TEST(SessionMap, RadiusDropsFarPoints) {
  SessionMap sm;
  sm.configure(SessionMap::Config{5.0, 0.15, 1000});
  sm.reset(makePose(0.0), 0.0);
  // 100m 外：直接丢
  auto far = makeCloud({{100.0, 0.0, 0.0}});
  sm.update(*far, makePose(0.0));
  EXPECT_EQ(sm.size(), 0u);
  // 半径内：保留
  auto near_cloud = makeCloud({{1.0, 0.0, 0.0}});
  sm.update(*near_cloud, makePose(0.0));
  EXPECT_EQ(sm.size(), 1u);
}

TEST(SessionMap, VoxelDedup) {
  SessionMap sm;
  sm.configure(SessionMap::Config{25.0, 0.15, 1000});
  sm.reset(makePose(0.0), 0.0);
  // 同一 0.15m 体素内的两个点 -> 只留一个
  auto cloud = makeCloud({{1.000, 0.0, 0.0}, {1.010, 0.0, 0.0}});
  sm.update(*cloud, makePose(0.0));
  EXPECT_EQ(sm.size(), 1u);
}

TEST(SessionMap, DeactivateClears) {
  SessionMap sm;
  sm.configure(SessionMap::Config{25.0, 0.15, 1000});
  sm.reset(makePose(0.0), 0.0);
  sm.update(*makeCloud({{1.0, 0.0, 0.0}}), makePose(0.0));
  ASSERT_EQ(sm.size(), 1u);
  sm.deactivate();
  EXPECT_FALSE(sm.active());
  EXPECT_EQ(sm.size(), 0u);
}

TEST(SessionMap, AgeGrows) {
  SessionMap sm;
  sm.reset(makePose(0.0), 100.0);
  EXPECT_NEAR(sm.ageS(100.0), 0.0, 1e-9);
  EXPECT_NEAR(sm.ageS(137.5), 37.5, 1e-9);
}

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main), 所以这里自带 main
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
