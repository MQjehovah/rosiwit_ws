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

// 动态点剔除 (2026-11-28): 只在 1 个关键帧里出现的体素 = 行人/门/反光等瞬态,
// 不是最新关键帧时从视图剔除; 静态点被多个关键帧看到 -> 保留。
TEST(SessionMap, DynamicTransientFilteredFromView) {
  SessionMap sm;
  SessionMap::Config cfg{25.0, 0.15, 1000};
  cfg.kf_min_dist = 1.0;
  cfg.min_kf_hits = 2;
  sm.configure(cfg);
  sm.reset(makePose(0.0), 0.0);

  // kf1 (机器人 x=1): 世界系 (10,0,0) 静态 + (2,0,0) 行人
  sm.update(*makeCloud({{9.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}), makePose(1.0));
  auto v1 = sm.pointsWorld();
  ASSERT_TRUE(v1);
  EXPECT_EQ(v1->size(), 2u) << "最新关键帧不过滤, 刚建帧时视图应完整";

  // kf2 (机器人 x=2): 只看到 (10,0,0)
  sm.update(*makeCloud({{8.0, 0.0, 0.0}}), makePose(2.0));
  auto v2 = sm.pointsWorld();
  ASSERT_TRUE(v2);
  ASSERT_EQ(v2->size(), 1u) << "只出现 1 个关键帧的行人点应被剔除";
  EXPECT_NEAR(v2->points[0].x, 10.0f, 1e-6);
}

// 同一剔除也要发生在"关键帧走出覆盖、送融合"之前: 否则瞬态点会被 mergeChunk
// 当成新结构写进 w 账本 (先污染再慢慢雕刻)。
TEST(SessionMap, DynamicTransientFilteredBeforeMerge) {
  SessionMap sm;
  SessionMap::Config cfg{1.0, 0.15, 1000};   // 小半径, 方便让关键帧退出
  cfg.kf_min_dist = 1.0;
  cfg.min_kf_hits = 2;
  sm.configure(cfg);
  sm.reset(makePose(0.0), 0.0);

  // kf1 (机器人 x=1): 静态 (1.5,0,0) + 行人 (2.2,0,0)
  sm.update(*makeCloud({{0.5, 0.0, 0.0}, {1.2, 0.0, 0.0}}), makePose(1.0));
  // kf2 (机器人 x=2): 静态 (1.5,0,0)
  sm.update(*makeCloud({{-0.5, 0.0, 0.0}}), makePose(2.0));

  auto leaving = sm.popLeavingKfs(Eigen::Vector3d(4.0, 0.0, 0.0));
  ASSERT_EQ(leaving.size(), 2u);
  size_t total = 0;
  for (const auto& kf : leaving) {
    ASSERT_TRUE(kf.world);
    total += kf.world->size();
    for (const auto& p : kf.world->points)
      EXPECT_GT(std::fabs(p.x - 2.2f), 0.05f) << "行人点不应进入融合";
  }
  EXPECT_EQ(total, 2u) << "两个关键帧各留 1 个静态点";
}

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main), 所以这里自带 main
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
