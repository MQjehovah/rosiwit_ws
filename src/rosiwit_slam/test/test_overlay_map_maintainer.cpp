// test/test_overlay_map_maintainer.cpp
// A 阶段(影子)体素可信度 w: 观测累加 / 雕刻衰减 / persistency_cov 统计.
#include <gtest/gtest.h>

#include <initializer_list>

#include "algorithms/localization/maintainer/overlay_map_maintainer.h"
#include "algorithms/localization/maintainer/insert_gate.h"

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
    c.novel_min_cluster = 1;   // 老用例: 单点也允许沉淀
    c.carve_stride = 1;
    c.decay_frames = 0;
    c.persistency_anchor = 128;
    c.shadow = true;
    // 老用例的语义 = 每帧计票, 每帧 -8, 递减到 0 删除
    c.persistency_unit = 0;
    c.persistency_hit_gain = 8;
    c.persistency_miss_confirm = 1;
    c.persistency_miss_action = 1;
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
    EXPECT_LT(m.persistencyCoverage(mkPose(0.0), &age, &sub), 0.0);
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
    double s = m.persistencyCoverage(pose, &age, &sub);
    ASSERT_EQ(sub, 0u);
    EXPECT_LT(s, 0.0);

    // 再观测 15 帧: w = 8 + 15*8 = 128 -> 恰好达标
    for (int i = 0; i < 15; ++i) m.onLocalized(pose, scan, mr, true);
    s = m.persistencyCoverage(pose, &age, &sub);
    ASSERT_EQ(sub, 1u);
    EXPECT_DOUBLE_EQ(s, 1.0);
    EXPECT_GE(age, 0);

    // 少一帧就不够: 重新开始, 14 帧 -> w = 8 + 14*8 = 120 < 128
    OverlayMapMaintainer m2;
    m2.configure(testCfg());
    for (int i = 0; i < 15; ++i) m2.onLocalized(pose, scan, mr, true);
    s = m2.persistencyCoverage(pose, &age, &sub);
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
    EXPECT_LT(m.persistencyCoverage(pose, nullptr, &sub), 0.0);
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
    ASSERT_DOUBLE_EQ(m.persistencyCoverage(pose), 1.0);

    // 扫描到更远处: 射线穿过 1.0m 的体素 -> 每帧 -8, 16 帧后 w=0 删除.
    // 关掉新结构沉淀(最小簇设到不可能满足), 保证只有"雕刻"在起作用.
    MaintainerConfig no_insert = testCfg();
    no_insert.novel_min_cluster = 1000;
    m.configure(no_insert);
    auto far = mkCloud({V3D(3.0, 0.0, 0.0)});
    for (int i = 0; i < 20; ++i) m.onLocalized(pose, far, mr, true);
    size_t sub = 123;
    const double s = m.persistencyCoverage(pose, nullptr, &sub);
    // 锚点被雕刻删除, 也没有新体素写入 -> 没有变化结构可见 (n/a)
    EXPECT_EQ(sub, 0u);
    EXPECT_LT(s, 0.0);
}

// ---------------- w 计票单位 ----------------

TEST(OverlayWUnits, FrameCountsEveryFrame)
{
    OverlayMapMaintainer m;
    MaintainerConfig c = testCfg();
    c.persistency_unit = 0;            // frame
    c.persistency_hit_gain = 1;
    m.configure(c);
    const PoseStamped pose = mkPose(0.0);
    auto scan = mkCloud({V3D(1.0, 0.0, 0.0)});
    const MatchResult mr = lowFitnessMatch();
    for (int i = 0; i < 5; ++i) m.onLocalized(pose, scan, mr, true);
    EXPECT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 5);
}

TEST(OverlayWUnits, SessionCountsOncePerSession)
{
    OverlayMapMaintainer m;
    MaintainerConfig c = testCfg();
    c.persistency_unit = 2;            // session
    c.persistency_hit_gain = 1;
    m.configure(c);
    const PoseStamped pose = mkPose(0.0);
    auto scan = mkCloud({V3D(1.0, 0.0, 0.0)});
    const MatchResult mr = lowFitnessMatch();

    for (int i = 0; i < 5; ++i) m.onLocalized(pose, scan, mr, true);
    EXPECT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 1);   // 同一会话只算一票

    m.beginPersistencySession();
    for (int i = 0; i < 5; ++i) m.onLocalized(pose, scan, mr, true);
    EXPECT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 2);   // 新会话 +1
}

TEST(OverlayWUnits, ViewCountsOnReentry)
{
    OverlayMapMaintainer m;
    MaintainerConfig c = testCfg();
    c.persistency_unit = 1;            // view
    c.persistency_hit_gain = 1;
    m.configure(c);
    const PoseStamped pose = mkPose(0.0);
    const MatchResult mr = lowFitnessMatch();

    auto scan_a = mkCloud({V3D(1.0, 0.0, 0.0)});
    m.onLocalized(pose, scan_a, mr, true);
    m.onLocalized(pose, scan_a, mr, true);
    EXPECT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 1);   // 连续看到 -> 一票

    // 中间一帧看点别处(射线不穿过 1m 处), 再回来 -> 新的一次"扫到"
    auto scan_b = mkCloud({V3D(-3.0, 0.0, 0.0)});
    m.onLocalized(pose, scan_b, mr, true);
    m.onLocalized(pose, scan_a, mr, true);
    EXPECT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 2);
}

TEST(OverlayWUnits, MissConfirmDeletesAfterNthFreeFrame)
{
    OverlayMapMaintainer m;
    MaintainerConfig c = testCfg();
    c.persistency_unit = 2;            // session
    c.persistency_hit_gain = 1;
    c.persistency_miss_confirm = 3;    // 连续 3 帧 free 才删
    c.persistency_miss_action = 0;     // 删除
    m.configure(c);
    const PoseStamped pose = mkPose(0.0);
    const MatchResult mr = lowFitnessMatch();

    auto near = mkCloud({V3D(1.0, 0.0, 0.0)});
    m.onLocalized(pose, near, mr, true);
    ASSERT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 1);

    // 射线穿过 1m 处但不在那里落点 -> free 证据
    auto far = mkCloud({V3D(5.0, 0.0, 0.0)});
    m.onLocalized(pose, far, mr, true);
    m.onLocalized(pose, far, mr, true);
    EXPECT_GE(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 1);   // 还没到 3 帧
    m.onLocalized(pose, far, mr, true);
    EXPECT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), -1);  // 已删除
}

TEST(OverlayWUnits, HitResetsMissCounter)
{
    OverlayMapMaintainer m;
    MaintainerConfig c = testCfg();
    c.persistency_unit = 2;
    c.persistency_hit_gain = 1;
    c.persistency_miss_confirm = 3;
    c.persistency_miss_action = 0;
    m.configure(c);
    const PoseStamped pose = mkPose(0.0);
    const MatchResult mr = lowFitnessMatch();

    auto near = mkCloud({V3D(1.0, 0.0, 0.0)});
    auto far = mkCloud({V3D(5.0, 0.0, 0.0)});
    m.onLocalized(pose, near, mr, true);
    m.onLocalized(pose, far, mr, true);    // miss 1
    m.onLocalized(pose, far, mr, true);    // miss 2
    m.onLocalized(pose, near, mr, true);   // 又看到 -> miss 清零
    m.onLocalized(pose, far, mr, true);    // miss 1
    m.onLocalized(pose, far, mr, true);    // miss 2
    EXPECT_GE(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 1);   // 没被删
}

// ---------------- 新结构沉淀 (与 fitness 解耦) ----------------

namespace {

// 底图: 4m 见方的地板 (z=0, 0.2m 间距), 2m 外没有结构
CloudType::Ptr mkFloorBase()
{
    CloudType::Ptr c(new CloudType());
    for (int i = -20; i <= 20; ++i)
        for (int j = -20; j <= 20; ++j) {
            PointType p;
            p.x = static_cast<float>(i * 0.2);
            p.y = static_cast<float>(j * 0.2);
            p.z = 0.0f;
            c->push_back(p);
        }
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}

// 扫描: 近处地板 + (可选) 2m 外一个 3x3x3 的新结构
CloudType::Ptr mkScanWithNovelBox(bool with_box = true)
{
    CloudType::Ptr c(new CloudType());
    for (int i = -5; i <= 5; ++i)
        for (int j = -5; j <= 5; ++j) {
            PointType p;
            p.x = static_cast<float>(i * 0.2);
            p.y = static_cast<float>(j * 0.2);
            p.z = 0.0f;
            c->push_back(p);
        }
    if (with_box) {
        for (int ix = 0; ix <= 2; ++ix)
            for (int iy = -1; iy <= 1; ++iy)
                for (int iz = 0; iz <= 2; ++iz) {
                    PointType p;
                    p.x = 2.0f + static_cast<float>(ix) * 0.3f;
                    p.y = static_cast<float>(iy) * 0.3f;
                    p.z = 0.6f + static_cast<float>(iz) * 0.3f;
                    c->push_back(p);
                }
    }
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}

MatchResult confidentMatch()
{
    MatchResult mr;
    mr.converged = true;
    mr.fitness = 0.95;   // 高于 fitness_trigger: 老逻辑不会沉淀
    return mr;
}

}  // namespace

TEST(OverlayNovelStructure, CommittedWhileFitnessHigh)
{
    OverlayMapMaintainer m;
    MaintainerConfig c = testCfg();
    c.fitness_trigger = 0.5;
    c.min_keyframe_dist = 1.0;   // 静止机器人: 距离门控永远不满足
    c.delta_min = 0.3;
    c.novel_min_cluster = 3;
    c.persistency_unit = 1;      // view
    c.persist_hits = 1;
    m.configure(c);
    m.setBaseMap("", mkFloorBase());

    m.onLocalized(mkPose(0.0), mkScanWithNovelBox(), confidentMatch(), true);

    EXPECT_GT(m.overlaySize(), 0u);
    EXPECT_GE(m.persistencyAt(V3D(2.0, 0.0, 0.6)), 1);
}

TEST(OverlayNovelStructure, VotesPerViewNotPerFrame)
{
    OverlayMapMaintainer m;
    MaintainerConfig c = testCfg();
    c.fitness_trigger = 0.5;
    c.min_keyframe_dist = 1.0;
    c.delta_min = 0.3;
    c.novel_min_cluster = 3;
    c.persistency_unit = 1;      // view
    c.persist_hits = 2;          // 需要两次"重新看到"
    m.configure(c);
    m.setBaseMap("", mkFloorBase());

    const PoseStamped pose = mkPose(0.0);
    auto box = mkScanWithNovelBox(true);
    auto plain = mkScanWithNovelBox(false);
    const MatchResult mr = confidentMatch();

    m.onLocalized(pose, box, mr, true);
    m.onLocalized(pose, box, mr, true);      // 连续看到: 仍只有 1 票
    EXPECT_EQ(m.overlaySize(), 0u);

    m.onLocalized(pose, plain, mr, true);    // 中间一帧看不到
    m.onLocalized(pose, box, mr, true);      // 重新进入视野: 第 2 票
    EXPECT_GT(m.overlaySize(), 0u);
}

TEST(OverlayNovelStructure, StructureAlreadyInBaseIsNotCommitted)
{
    OverlayMapMaintainer m;
    MaintainerConfig c = testCfg();
    c.fitness_trigger = 0.5;
    c.min_keyframe_dist = 1.0;
    c.delta_min = 0.3;
    c.novel_min_cluster = 3;
    c.persistency_unit = 1;
    c.persist_hits = 1;
    m.configure(c);

    // 底图里已经有同样的箱子 -> 属于"已知结构", 不能沉淀
    CloudType::Ptr base = mkFloorBase();
    auto with_box = mkScanWithNovelBox(true);
    for (const auto& p : with_box->points) {
        if (p.z > 0.1f) base->push_back(p);
    }
    base->width = static_cast<uint32_t>(base->size());
    base->height = 1;
    m.setBaseMap("", base);

    m.onLocalized(mkPose(0.0), mkScanWithNovelBox(), confidentMatch(), true);
    EXPECT_EQ(m.overlaySize(), 0u);
}

TEST(OverlayNovelStructure, IsolatedNovelPointRejectedByCluster)
{
    OverlayMapMaintainer m;
    MaintainerConfig c = testCfg();
    c.fitness_trigger = 0.5;
    c.min_keyframe_dist = 1.0;
    c.delta_min = 0.3;
    c.novel_min_cluster = 3;
    c.persistency_unit = 1;
    c.persist_hits = 1;
    m.configure(c);
    m.setBaseMap("", mkFloorBase());

    // 单个孤立点 (噪声) 不能进 overlay
    CloudType::Ptr scan = mkScanWithNovelBox(false);
    PointType p;
    p.x = 2.0f; p.y = 0.0f; p.z = 0.6f;
    scan->push_back(p);
    scan->width = static_cast<uint32_t>(scan->size());

    m.onLocalized(mkPose(0.0), scan, confidentMatch(), true);
    EXPECT_EQ(m.overlaySize(), 0u);
}

// ---------------- 锚质量: 相对可见证据 ----------------

TEST(AnchorQuality, NarrowAisleVisibleEvidencePasses)
{
    AnchorQuality q;
    q.converged = true;
    q.accepted = true;
    q.visible = 176;          // 窄过道: 整帧只覆盖 176 个 0.4m 体素
    q.correspondences = 126;  // dynamic_filter 之后
    q.inliers = 105;
    AnchorCriteria c;
    EXPECT_TRUE(anchorQualityOk(q, c));
}

TEST(AnchorQuality, WeakMatchInRichSceneRejected)
{
    AnchorQuality q;
    q.converged = true;
    q.accepted = true;
    q.visible = 5000;         // 开阔区: 可见上千点, 但只配上 20%
    q.correspondences = 4000;
    q.inliers = 1000;
    AnchorCriteria c;
    EXPECT_FALSE(anchorQualityOk(q, c));
}

TEST(AnchorQuality, DegenerateHessianRejected)
{
    AnchorQuality q;
    q.converged = true;
    q.accepted = true;
    q.visible = 1000;
    q.correspondences = 900;
    q.inliers = 900;
    q.has_hessian = true;
    q.smallest_eig_ratio = 1e-4;   // < 1e-3: 退化方向
    AnchorCriteria c;
    EXPECT_FALSE(anchorQualityOk(q, c));
}

TEST(AnchorQuality, BelowAbsoluteFloorRejected)
{
    AnchorQuality q;
    q.converged = true;
    q.accepted = true;
    q.visible = 20;
    q.correspondences = 20;
    q.inliers = 10;           // < 绝对下限
    AnchorCriteria c;
    EXPECT_FALSE(anchorQualityOk(q, c));
}

TEST(AnchorQuality, RejectedFrameIsNotAnAnchor)
{
    AnchorQuality q;
    q.converged = true;
    q.accepted = false;
    q.visible = 1000;
    q.correspondences = 900;
    q.inliers = 900;
    AnchorCriteria c;
    EXPECT_FALSE(anchorQualityOk(q, c));
}

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main), 所以这里自带 main
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
