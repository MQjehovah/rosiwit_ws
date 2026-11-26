// test/test_overlay_map_maintainer.cpp
// A 阶段(影子)体素可信度 w: 观测累加 / 雕刻衰减 / persistency_cov 统计.
#include <gtest/gtest.h>

#include <initializer_list>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>
#include <thread>

#include "algorithms/localization/maintainer/overlay_map_maintainer.h"
#include "algorithms/localization/maintainer/insert_gate.h"
#include "algorithms/localization/maintainer/annotate_slice.h"

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

PointType mkPt(float x, float y, float z, float inten = 0.0f)
{
    // 本机 PCL 1.8 的 PointXYZINormal 只有默认 / 拷贝构造, 必须逐字段赋值
    PointType q;
    q.x = x; q.y = y; q.z = z;
    q.normal_x = q.normal_y = q.normal_z = 0.0f;
    q.curvature = 0.0f;
    q.intensity = inten;
    return q;
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

TEST(OverlayNovelStructure, ClusterCountsWholeScanNotOnlyNovel)
{
    OverlayMapMaintainer m;
    MaintainerConfig c = testCfg();
    c.fitness_trigger = 0.5;
    c.min_keyframe_dist = 1.0;
    c.delta_min = 0.3;
    c.novel_min_cluster = 3;
    c.persistency_unit = 1;      // view
    c.persist_hits = 1;
    m.configure(c);
    m.setBaseMap("", mkFloorBase());

    // 第一帧: 一小块 3x1x3 的结构, 正常沉淀
    CloudType::Ptr patch(new CloudType());
    for (int ix = 0; ix <= 2; ++ix)
        for (int iz = 0; iz <= 2; ++iz) {
            PointType p;
            p.x = 2.0f + static_cast<float>(ix) * 0.3f;
            p.y = 0.0f;
            p.z = 0.6f + static_cast<float>(iz) * 0.3f;
            patch->push_back(p);
        }
    patch->width = static_cast<uint32_t>(patch->size());
    patch->height = 1;
    m.onLocalized(mkPose(0.0), patch, confidentMatch(), true);
    ASSERT_GE(m.persistencyAt(V3D(2.6, 0.0, 0.6)), 1);

    // 第二帧: 同一块 + 一个贴着它的新点. 新点的邻居全在"已沉淀体素"上,
    // 它们不该被当成噪声 —— 最小簇要按整帧观测算, 不是只看本帧新体素.
    PointType q;
    q.x = 2.9f; q.y = 0.0f; q.z = 0.6f;
    patch->push_back(q);
    patch->width = static_cast<uint32_t>(patch->size());
    m.onLocalized(mkPose(0.0), patch, confidentMatch(), true);

    EXPECT_GE(m.persistencyAt(V3D(2.9, 0.0, 0.6)), 1);
}

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main), 所以这里自带 main
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// ---------------------------------------------------------------------------
// 2026-11-26 (整点丢失根因修复的配套): annotateBaseWeights 拆成两个原语 ——
//   annotateBaseWeightsRange: 只写 intensity / 打删除标记, 可任意切片 (账本的写者线程
//                             在自己的帧预算内分多次调用);
//   compactAnnotated:          去掉带标记的点, 不需要账本 (后台线程做)。
// 整云一次调用的语义必须与原来的整云拷贝版完全一致:
//   intensity = min(255, w) (账本里没有的体素用 base_voxel_w); w<=0 的点丢弃;
//   非有限点丢弃; 结果是原地压缩后的同一云 (width/height 同步)。
TEST(OverlayMaintainerWeights, AnnotateBaseWeightsInPlaceKeepsSemantics)
{
    OverlayMapMaintainer m;
    MaintainerConfig cfg = testCfg();
    cfg.base_voxel_w = 100;
    m.configure(cfg);

    // 体素 (1.0, 0, 0) 在账本里拿到 w = 8 + 15*8 = 128 (见 HitsAccumulateToAnchor)
    const PoseStamped pose = mkPose(0.0);
    auto scan = mkCloud({V3D(1.0, 0.0, 0.0)});
    const MatchResult mr = lowFitnessMatch();
    for (int i = 0; i < 16; ++i) m.onLocalized(pose, scan, mr, true);

    CloudType cloud;
    const int kMany = 40000;                      // 远大于一个分片 (8192 点)
    for (int i = 0; i < kMany; ++i) {
        PointType p;
        p.x = 0.01f * static_cast<float>(i % 300);   // 0..2.99: 多点同体素
        p.y = 5.0f;
        p.z = 0.0f;
        p.intensity = 7.0f;                       // 旧 intensity 必须被覆盖
        cloud.push_back(p);
    }
    PointType hit;
    hit.x = 1.0f; hit.y = 0.0f; hit.z = 0.0f; hit.intensity = 7.0f;
    cloud.push_back(hit);
    PointType bad;                                // 非有限点: 丢弃
    bad.x = std::numeric_limits<float>::quiet_NaN(); bad.y = 0.0f; bad.z = 0.0f;
    cloud.push_back(bad);
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;

    m.annotateBaseWeights(cloud);

    ASSERT_EQ(cloud.size(), static_cast<size_t>(kMany + 1));
    EXPECT_EQ(cloud.width, static_cast<uint32_t>(cloud.size()));
    EXPECT_EQ(cloud.height, 1u);
    int n_hit = 0, n_default = 0;
    for (const PointType& p : cloud.points) {
        ASSERT_TRUE(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
        const bool is_hit = (p.x == 1.0f && p.y == 0.0f);
        EXPECT_FLOAT_EQ(p.intensity, is_hit ? 128.0f : 100.0f)
            << "点 (" << p.x << "," << p.y << "," << p.z << ")";
        is_hit ? ++n_hit : ++n_default;
    }
    EXPECT_EQ(n_hit, 1);
    EXPECT_EQ(n_default, kMany);
}

// 并发冒烟: 标注 (整云一次调用, 段内持锁) 的同时另一线程持续写账本 —— 不死锁 / 不崩溃 /
// 结果自洽。修复前 annotateBaseWeights 完全不持锁, 与 onLocalized 是数据竞争 (过去它
// 恰好只在跟踪线程调用才没出事)。生产路径上账本只由跟踪线程读 (见分片测试), 这里只
// 保证接口本身从任意线程调用都是安全的。
TEST(OverlayMaintainerWeights, AnnotateBaseWeightsConcurrentWithLedgerWrites)
{
    OverlayMapMaintainer m;
    MaintainerConfig cfg = testCfg();
    cfg.base_voxel_w = 100;
    m.configure(cfg);

    CloudType::Ptr big(new CloudType());
    for (int i = 0; i < 200000; ++i) {
        PointType p;
        p.x = 0.01f * static_cast<float>(i % 2000);
        p.y = 5.0f + 0.02f * static_cast<float>((i / 2000) % 100);   // 远离账本被写的体素
        p.z = 0.0f;
        big->push_back(p);
    }
    big->width = static_cast<uint32_t>(big->size());
    big->height = 1;

    const PoseStamped pose = mkPose(0.0);
    auto scan = mkCloud({V3D(1.0, 0.0, 0.0), V3D(1.3, 0.0, 0.0)});
    const MatchResult mr = lowFitnessMatch();
    std::atomic<bool> stop(false);
    std::thread writer([&] {
        int n = 0;
        while (!stop.load()) m.onLocalized(pose, scan, mr, (n++ % 3) != 0);
    });
    for (int r = 0; r < 3; ++r) {
        CloudType work = *big;
        m.annotateBaseWeights(work);
        ASSERT_EQ(work.size(), big->size());     // 这些体素账本里没有 -> 默认 w, 不丢点
        for (const PointType& p : work.points) EXPECT_FLOAT_EQ(p.intensity, 100.0f);
    }
    stop.store(true);
    writer.join();
}

// 任意切片 + 压缩 == 整云一次标注 (点序不变)。工作图落盘时跟踪线程每帧推进若干
// 不规则的小片 (按时间预算截断), 最终结果必须与一次做完完全相同。
TEST(OverlayMaintainerWeights, AnnotateRangeSlicesEqualOneShot)
{
    OverlayMapMaintainer m;
    MaintainerConfig cfg = testCfg();
    cfg.base_voxel_w = 100;
    m.configure(cfg);
    const PoseStamped pose = mkPose(0.0);
    auto scan = mkCloud({V3D(1.0, 0.0, 0.0)});
    const MatchResult mr = lowFitnessMatch();
    for (int i = 0; i < 16; ++i) m.onLocalized(pose, scan, mr, true);   // 体素 (1,0,0): w=128

    CloudType::Ptr src(new CloudType());
    for (int i = 0; i < 50000; ++i) {
        src->push_back(mkPt(0.01f * static_cast<float>(i % 300), 5.0f, 0.1f * static_cast<float>(i % 7), 7.0f));
    }
    src->push_back(mkPt(1.0f, 0.0f, 0.0f, 7.0f));                 // 命中体素: w=128
    src->push_back(mkPt(1.05f, 0.05f, 0.05f, 7.0f));
    src->push_back(mkPt(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 7.0f));   // 丢弃
    src->push_back(mkPt(1.0f, std::numeric_limits<float>::infinity(), 0.0f, 7.0f));    // 丢弃
    src->push_back(mkPt(2.0f, 9.0f, 0.0f, 7.0f));
    src->width = static_cast<uint32_t>(src->size());
    src->height = 1;

    CloudType one_shot = *src;
    m.annotateBaseWeights(one_shot);

    CloudType sliced = *src;
    size_t begin = 0;
    const size_t sizes[] = {1, 7, 1000, 8192, 40000, 3, 0 /* 余下全部 */};
    for (const size_t sz : sizes) {
        const size_t end = (sz == 0) ? sliced.size() : std::min(sliced.size(), begin + sz);
        m.annotateBaseWeightsRange(sliced, begin, end);
        begin = end;
    }
    IMapMaintainer::compactAnnotated(sliced);

    ASSERT_EQ(sliced.size(), one_shot.size());
    ASSERT_EQ(sliced.size(), src->size() - 2);
    EXPECT_EQ(sliced.width, static_cast<uint32_t>(sliced.size()));
    EXPECT_EQ(sliced.height, 1u);
    for (size_t i = 0; i < one_shot.size(); ++i) {
        ASSERT_FLOAT_EQ(sliced.points[i].x, one_shot.points[i].x) << "点序变了 @" << i;
        EXPECT_FLOAT_EQ(sliced.points[i].y, one_shot.points[i].y);
        EXPECT_FLOAT_EQ(sliced.points[i].z, one_shot.points[i].z);
        EXPECT_FLOAT_EQ(sliced.points[i].intensity, one_shot.points[i].intensity);
    }
    int n128 = 0;
    for (const PointType& p : one_shot.points) {
        EXPECT_TRUE(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
        n128 += (p.intensity == 128.0f) ? 1 : 0;
    }
    EXPECT_EQ(n128, 2);
}

// 单次分片的代价 (也是它的持锁范围) 与片长成正比: 260 万条账本上, 8192 点的一片必须
// 远小于其余全部 —— 这是"跟踪线程每帧只推进有界的一片"的根据。
TEST(OverlayMaintainerWeights, AnnotateRangeCostProportionalToRange)
{
    OverlayMapMaintainer m;
    MaintainerConfig cfg = testCfg();
    cfg.base_voxel_w = 128;
    m.configure(cfg);
    m.setBaseEmbeddedWeights(true);

    // 合成底图: 0.3m 格点, 每点独占一个体素 -> 账本条数 = 点数 (~260 万, 与实机工作图同量级)
    CloudType::Ptr base(new CloudType());
    const int nx = 200, ny = 200, nz = 65;
    base->reserve(static_cast<size_t>(nx) * ny * nz);
    for (int ix = 0; ix < nx; ++ix)
        for (int iy = 0; iy < ny; ++iy)
            for (int iz = 0; iz < nz; ++iz) {
                base->push_back(mkPt(0.3f * ix + 0.01f, 0.3f * iy + 0.01f, 0.3f * iz + 0.01f, 128.0f));
            }
    base->width = static_cast<uint32_t>(base->size());
    base->height = 1;
    ASSERT_EQ(base->size(), static_cast<size_t>(nx) * ny * nz);
    m.setBaseMap("synthetic", base);            // 账本播种 (与实机加载工作图同路径)

    using clk = std::chrono::steady_clock;
    const auto ms = [](clk::time_point a, clk::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    const size_t kSlice = 8192;                 // 生产里的子段大小 (localization_pipeline.cpp)
    const auto t0 = clk::now();
    m.annotateBaseWeightsRange(*base, 0, kSlice);
    const double small_ms = ms(t0, clk::now());
    const auto t1 = clk::now();
    m.annotateBaseWeightsRange(*base, kSlice, base->size());
    const double rest_ms = ms(t1, clk::now());
    std::cout << "[annotate] 账本 " << base->size() << " 条: Range(0," << kSlice << ") = " << small_ms
              << " ms, Range(" << kSlice << "," << base->size() << ") = " << rest_ms << " ms" << std::endl;
    EXPECT_EQ(base->size(), static_cast<size_t>(nx) * ny * nz);   // 只写 intensity, 不丢点
    EXPECT_GT(rest_ms, 20.0);                                     // 其余部分确实是"大活"
    EXPECT_LT(small_ms, 0.1 * rest_ms) << "单片代价没有随片长收缩";
}

// 预算内推进 (工作图落盘时跟踪线程每帧调用一次): 260 万条账本上, 每次调用的耗时必须有界
// (预算 2ms + 一片), 多次调用后做完, 结果与整云一次标注相同。
TEST(OverlayMaintainerWeights, AnnotateWithinBudgetAdvancesInBoundedSlices)
{
    OverlayMapMaintainer m;
    MaintainerConfig cfg = testCfg();
    cfg.base_voxel_w = 128;
    m.configure(cfg);
    m.setBaseEmbeddedWeights(true);

    CloudType::Ptr base(new CloudType());
    const int nx = 200, ny = 200, nz = 65;
    base->reserve(static_cast<size_t>(nx) * ny * nz);
    for (int ix = 0; ix < nx; ++ix)
        for (int iy = 0; iy < ny; ++iy)
            for (int iz = 0; iz < nz; ++iz)
                base->push_back(mkPt(0.3f * ix + 0.01f, 0.3f * iy + 0.01f, 0.3f * iz + 0.01f, 128.0f));
    base->width = static_cast<uint32_t>(base->size());
    base->height = 1;
    m.setBaseMap("synthetic", base);
    const size_t n = base->size();
    ASSERT_EQ(n, 2600000u);

    using clk = std::chrono::steady_clock;
    size_t next = 0;
    int calls = 0;
    double max_ms = 0.0, sum_ms = 0.0;
    while (next < n) {
        const auto t0 = clk::now();
        const size_t got = annotateWithinBudget(m, *base, next, 2.0);
        const double ms = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
        ASSERT_GT(got, next) << "没有推进";
        ASSERT_LE(got, next + 8192u * 64u) << "一次调用推进了 " << (got - next) << " 点 (预算失效)";
        next = got;
        max_ms = std::max(max_ms, ms);
        sum_ms += ms;
        ++calls;
        ASSERT_LT(calls, 100000);
    }
    std::cout << "[annotate] 预算推进 2ms: " << calls << " 次调用, 单次 max=" << max_ms
              << " ms, 合计=" << sum_ms << " ms" << std::endl;
    EXPECT_GE(calls, 10) << "一次调用就做完了, 预算没有生效";
    // 预算 2ms + 一片 (~0.5ms); 余量留给本机 (常态 load 8~10) 的调度抖动
    EXPECT_LT(max_ms, 25.0) << "单次调用耗时 " << max_ms << " ms, 超出预算一个数量级";
    EXPECT_EQ(base->size(), n);
    for (const PointType& p : base->points) EXPECT_FLOAT_EQ(p.intensity, 128.0f);
}
