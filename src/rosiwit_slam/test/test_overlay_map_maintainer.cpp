// test/test_overlay_map_maintainer.cpp
// A 阶段(影子)体素可信度 w: 观测累加 / 雕刻衰减 / static_support 统计.
#include <gtest/gtest.h>

#include <initializer_list>

#include "algorithms/localization/maintainer/overlay_map_maintainer.h"

using namespace rosiwit_slam;

namespace {

PoseStamped mkPose(double x, double y = 0.0, double z = 0.0)
{
    PoseStamped p;
    p.trans = V3D(x, y, z);
    p.rot = M3D::Identity();
    return p;
}

CloudType::Ptr mkCloud(std::initializer_list<V3D> pts)
{
    CloudType::Ptr c(new CloudType());
    for (const V3D& p : pts) {
        PointType q;
        q.x = static_cast<float>(p.x());
        q.y = static_cast<float>(p.y());
        q.z = static_cast<float>(p.z());
        c->push_back(q);
    }
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}

// fitness < fitness_trigger -> 每帧都走"环境变化"分支, 保证体素会被写入
MaintainerConfig testCfg()
{
    MaintainerConfig c;
    c.enabled = true;
    c.voxel = 0.3;
    c.cell = 10.0;
    c.radius = 60.0;
    c.fitness_trigger = 0.99;
    c.min_confirm = 1;
    c.min_keyframe_dist = 0.0;
    c.persist_hits = 1;
    c.delta_min = 0.0;
    c.carve_stride = 1;
    c.decay_frames = 0;
    c.w_anchor = 128;
    c.shadow = true;
    return c;
}

MatchResult lowFitnessMatch()
{
    MatchResult mr;
    mr.converged = true;
    mr.fitness = 0.1;
    return mr;
}

}  // namespace

TEST(OverlayMaintainerWeights, NoOverlayReturnsNotAvailable)
{
    OverlayMapMaintainer m;
    m.configure(testCfg());
    int age = -1;
    size_t sub = 123;
    EXPECT_LT(m.staticSupport(mkPose(0.0), &age, &sub), 0.0);
    EXPECT_EQ(sub, 0u);
}

TEST(OverlayMaintainerWeights, HitsAccumulateToAnchor)
{
    OverlayMapMaintainer m;
    m.configure(testCfg());
    const PoseStamped pose = mkPose(0.0);
    auto scan = mkCloud({V3D(1.0, 0.0, 0.0)});
    const MatchResult mr = lowFitnessMatch();

    int age = -1;
    size_t sub = 0;
    // 首次写入发生在 observeHits 之后, 所以当帧还没有可统计的点 (n/a)
    m.onLocalized(pose, scan, mr, true);
    double s = m.staticSupport(pose, &age, &sub);
    ASSERT_EQ(sub, 0u);
    EXPECT_LT(s, 0.0);

    // 再观测 15 帧: w = 8 + 15*8 = 128 -> 恰好达标
    for (int i = 0; i < 15; ++i) m.onLocalized(pose, scan, mr, true);
    s = m.staticSupport(pose, &age, &sub);
    ASSERT_EQ(sub, 1u);
    EXPECT_DOUBLE_EQ(s, 1.0);
    EXPECT_GE(age, 0);

    // 少一帧就不够: 重新开始, 14 帧 -> w = 8 + 14*8 = 120 < 128
    OverlayMapMaintainer m2;
    m2.configure(testCfg());
    for (int i = 0; i < 15; ++i) m2.onLocalized(pose, scan, mr, true);
    s = m2.staticSupport(pose, &age, &sub);
    ASSERT_EQ(sub, 1u);
    EXPECT_DOUBLE_EQ(s, 0.0);
}

TEST(OverlayMaintainerWeights, UnhealthyFramesDoNotWrite)
{
    OverlayMapMaintainer m;
    m.configure(testCfg());
    const PoseStamped pose = mkPose(0.0);
    auto scan = mkCloud({V3D(1.0, 0.0, 0.0)});
    const MatchResult mr = lowFitnessMatch();

    // pose_confident = false (过桥/漂移期): 只读不写
    for (int i = 0; i < 20; ++i) m.onLocalized(pose, scan, mr, false);
    size_t sub = 123;
    EXPECT_LT(m.staticSupport(pose, nullptr, &sub), 0.0);
    EXPECT_EQ(sub, 0u);
}

TEST(OverlayMaintainerWeights, CarveDecaysAndDeletes)
{
    OverlayMapMaintainer m;
    m.configure(testCfg());
    const PoseStamped pose = mkPose(0.0);
    auto near = mkCloud({V3D(1.0, 0.0, 0.0)});
    const MatchResult mr = lowFitnessMatch();

    for (int i = 0; i < 16; ++i) m.onLocalized(pose, near, mr, true);   // w = 8 + 15*8 = 128
    ASSERT_DOUBLE_EQ(m.staticSupport(pose), 1.0);

    // 扫描到更远处: 射线穿过 1.0m 的体素 -> 每帧 -8, 16 帧后 w=0 删除.
    // 关掉写入(最小关键帧间距拉大), 保证只有"雕刻"在起作用.
    MaintainerConfig no_insert = testCfg();
    no_insert.min_keyframe_dist = 1e9;
    m.configure(no_insert);
    auto far = mkCloud({V3D(3.0, 0.0, 0.0)});
    for (int i = 0; i < 20; ++i) m.onLocalized(pose, far, mr, true);
    size_t sub = 123;
    const double s = m.staticSupport(pose, nullptr, &sub);
    // 锚点被雕刻删除, 也没有新体素写入 -> 没有变化结构可见 (n/a)
    EXPECT_EQ(sub, 0u);
    EXPECT_LT(s, 0.0);
}
