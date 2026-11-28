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

// 2026-11-28 (判死不落盘修复的配套): 判死体素在死亡时已经从 m_persistency 里 erase,
// 只剩 key 在 m_dead_base。annotateBaseWeightsRange 必须查判死集合, 否则查不到 w
// 就回落到 base_voxel_w=128, 把已经删除的底图点当"没观测过"又写回工作图。
// 实机症状: 账本判死 25,592 个体素, 整点落盘 0 点被丢弃, 重启后全部复活。
TEST(OverlayMaintainerWeights, DeadBasePointIsDroppedByAnnotate)
{
    OverlayMapMaintainer m;
    MaintainerConfig cfg = testCfg();
    cfg.persistency_unit = 0;          // 每帧计票 (每帧 -8)
    cfg.persistency_hit_gain = 8;
    m.configure(cfg);

    // 底图两点: (1,0,0) 会被 +x 射线的自由证据判死; (-1,0,0) 在射线反方向, 保持存活
    m.setBaseMap("test", mkCloud({V3D(1.0, 0.0, 0.0), V3D(-1.0, 0.0, 0.0)}));
    ASSERT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 128);

    const PoseStamped pose = mkPose(0.0);
    const MatchResult mr = lowFitnessMatch();
    auto far = mkCloud({V3D(5.0, 0.0, 0.0)});
    for (int i = 0; i < 16 && m.persistencyAt(V3D(1.0, 0.0, 0.0)) >= 0; ++i) {
        m.onLocalized(pose, far, mr, true);
    }
    ASSERT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), -1) << "16 帧自由证据后底图点应判死";

    CloudType::Ptr work = mkCloud({V3D(1.0, 0.0, 0.0), V3D(-1.0, 0.0, 0.0)});
    m.annotateBaseWeights(*work);

    ASSERT_EQ(work->size(), 1u) << "判死的底图点必须从工作图里丢弃";
    EXPECT_NEAR(work->points[0].x, -1.0f, 1e-6);
    EXPECT_FLOAT_EQ(work->points[0].intensity, 128.0f);
}

// 同一处遗漏的另一半: 提交时 addPoint 会把点同时写进 overlay 分块 (m_cells) 和
// 工作图云 (m_base_raw, 与管道 m_map 同一指针)。判死时如果只记 overlay 的
// m_pending_remove, 工作图云里的那份拷贝就会在落盘时按 128 复活 —— 新结构只增不减。
TEST(OverlayMaintainerWeights, DeadCommittedPointIsDroppedByAnnotate)
{
    OverlayMapMaintainer m;
    MaintainerConfig cfg = testCfg();
    cfg.persistency_unit = 0;          // 每帧计票
    cfg.persistency_hit_gain = 8;
    cfg.persist_hits = 1;              // 单帧提交
    m.configure(cfg);

    // 工作图 = 两个底图点; 提交点会经 addPoint 追加进同一张云。
    // (5,0,0) 上放一个底图点, 让后续射线端点落在底图上 -> 不再触发新提交。
    CloudType::Ptr working = mkCloud({V3D(-1.0, 0.0, 0.0), V3D(5.0, 0.0, 0.0)});
    m.setBaseMap("test", working);

    const PoseStamped pose = mkPose(0.0);
    const MatchResult mr = lowFitnessMatch();

    // 1) (2,0,0) 新结构提交 -> 升格进工作图, 账本 w=8
    m.onLocalized(pose, mkCloud({V3D(2.0, 0.0, 0.0)}), mr, true);
    ASSERT_EQ(working->size(), 3u) << "提交点应升格进工作图云";
    ASSERT_EQ(m.persistencyAt(V3D(2.0, 0.0, 0.0)), 8);

    // 2) 射线穿过 (2,0,0), 端点落在 (5,0,0) 的底图上 -> w 8 -> 0, 判死
    m.onLocalized(pose, mkCloud({V3D(5.0, 0.0, 0.0)}), mr, true);
    ASSERT_EQ(m.persistencyAt(V3D(2.0, 0.0, 0.0)), -1) << "提交点应被判死";

    // 3) 落盘标注: 工作图云里那份已判死的提交点拷贝必须被丢弃, 两个底图点保留
    m.annotateBaseWeights(*working);
    ASSERT_EQ(working->size(), 2u) << "判死的提交点也必须从工作图里丢弃";
    for (const PointType& p : working->points)
        EXPECT_GT(std::fabs(p.x - 2.0f), 0.1f) << "判死提交点的拷贝不应留存";
}

// 2026-11-28 实车 SIGSEGV 的回归: addPoint 会把新结构直接追加到工作图云
// (与管道 m_map 同一指针)。落盘线程必须拿 snapshotBaseCloud() 的深拷贝,
// 否则 voxelDownsample 与 push_back 并发。这里钉住"快照不随后续追加变化"。
TEST(OverlayMaintainerWeights, BaseSnapshotStableAgainstCommits)
{
    OverlayMapMaintainer m;
    MaintainerConfig cfg = testCfg();
    cfg.persist_hits = 1;
    m.configure(cfg);

    CloudType::Ptr working = mkCloud({V3D(-1.0, 0.0, 0.0)});
    m.setBaseMap("test", working);
    CloudType::Ptr snap = m.snapshotBaseCloud();
    ASSERT_TRUE(snap);
    const size_t n0 = snap->size();

    // 新结构提交 -> addPoint 向工作图云尾部追加
    m.onLocalized(mkPose(0.0), mkCloud({V3D(2.0, 0.0, 0.0)}), lowFitnessMatch(), true);
    ASSERT_EQ(working->size(), n0 + 1u) << "提交应升格进工作图云";
    EXPECT_EQ(snap->size(), n0) << "快照必须不受之后的追加影响 (落盘线程读它)";
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

// ---------------------------------------------------------------------------
// w 证据模型 (2026-11-27, 移植 ELite 式 5/6): 回归测试。
//
// 实机 (2026-11-27 02:39 起): 沿走廊跑 3 分钟判死 1.06 万个底图点, 23 分钟 1.6 万个,
// fitness 越跑越低。旧模型"端点落在体素里 +8 / 采样点落在体素里 -8"对体素边界太敏感:
// 今天的扫描与地图差几厘米 (跨体素) 就拿不到命中, 而掠过墙面 30cm 内的射线照记满 free 票。
// 新模型: 证据强度随"观测到地图点的距离"指数衰减 (ELite 式 5), 对数几率域累加 (式 6)。
// 这组测试只用新旧两个版本都有的接口 —— 为了能在旧实现上做 A/B (应当是红的)。
// ---------------------------------------------------------------------------
namespace {

MaintainerConfig eliteCfg() {
    MaintainerConfig c = testCfg();
    c.persistency_unit = 1;          // view (生产配置)
    c.persistency_hit_gain = 8;
    c.carve_stride = 5;
    c.delta_min = 0.3;               // 只需要"别把偏移后的墙面当成新结构"
    c.novel_min_cluster = 3;
    c.persist_hits = 3;
    return c;
}

// 一面墙: x ≈ 0.15, 每 0.3m 体素一个点 (y 0~12m, z 0.3~2.4m)。每个点独占一个体素。
CloudType::Ptr mkWall() {
    CloudType::Ptr c(new CloudType());
    for (int iy = 0; iy < 40; ++iy)
        for (int iz = 0; iz < 8; ++iz)
            c->push_back(mkPt(0.152f, 0.3f * iy + 0.137f, 0.3f * iz + 0.118f));
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}

// 一帧扫描 (机体 = 世界系, 传感器在 (sx, yt, 1.2)): 端点落在"今天的墙面"上 —— 与地图差 dx。
// y 方向 ±15m (掠射角最小 ~5°), z 方向 24 排 (间距 0.1m)。
// ★点序: z 排为外层循环。射线雕刻按点序每 carve_stride 个取一个; 排号在内层时点序的
//   周期 (24) 会和步长混叠 (z_rows_inner=true 专门用来复现, 见 RaySubsampling 测试)。
CloudType::Ptr mkWallScan(double sx, double yt, double z0, double dx, bool z_rows_inner = false) {
    CloudType::Ptr c(new CloudType());
    const double y0 = std::max(0.0, yt - 15.0), y1 = std::min(12.0, yt + 15.0);
    const auto emit = [&](int iz, double y) {
        c->push_back(mkPt(static_cast<float>(0.152 + dx - sx), static_cast<float>(y - yt),
                          static_cast<float>(0.3 * (iz % 8) + 0.129 + 0.1 * (iz / 8) - z0)));
    };
    if (z_rows_inner) {
        for (double y = y0; y <= y1; y += 0.02)
            for (int iz = 0; iz < 24; ++iz) emit(iz, y);
    } else {
        for (int iz = 0; iz < 24; ++iz)
            for (double y = y0; y <= y1; y += 0.02) emit(iz, y);
    }
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}

// 沿墙行驶 nViews 个视角 (每 0.2m 一个), 返回 (判死的墙点数, 最低 w)。ghost: 墙前 0.5m 处的一个底图点。
std::pair<int, int> driveAlongWall(double dx, int nViews, bool with_ghost, int* ghost_w = nullptr,
                                   double* ms_per_frame = nullptr, bool z_rows_inner = false,
                                   int stride = 5, int* ghost_dead_view = nullptr) {
    OverlayMapMaintainer m;
    MaintainerConfig cfg = eliteCfg();
    cfg.carve_stride = stride;
    m.configure(cfg);
    m.setBaseEmbeddedWeights(false);
    CloudType::Ptr base = mkWall();
    const V3D ghost(0.652, 6.0, 1.21);
    if (with_ghost) base->push_back(mkPt(0.652f, 6.0f, 1.21f));
    m.setBaseMap("", base);
    const MatchResult mr = lowFitnessMatch();
    double acc_ms = 0.0;
    for (int k = 0; k < nViews; ++k) {
        const double yt = 0.2 * k;
        const PoseStamped pose = mkPose(1.40 + 0.0, yt, 1.2);   // 传感器离墙 1.25m
        CloudType::Ptr scan = mkWallScan(1.40, yt, 1.2, dx, z_rows_inner);
        const auto t0 = std::chrono::steady_clock::now();
        m.onLocalized(pose, scan, mr, true);
        acc_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        if (with_ghost && ghost_dead_view && *ghost_dead_view < 0 &&
            m.persistencyAt(V3D(0.652, 6.0, 1.21)) < 0)
            *ghost_dead_view = k + 1;
    }
    if (ms_per_frame) *ms_per_frame = acc_ms / nViews;
    // ★不能写 for (p : mkWall()->points): 临时 shared_ptr 在 range-for 的初始化表达式结束时就
    //   析构, 循环读的是已释放的内存 (第一版就这么写, "判死 1 个"的坐标是垃圾值)。
    const CloudType::Ptr wall = mkWall();
    int dead = 0, wmin = 999;
    for (const auto& p : wall->points) {
        const int w = m.persistencyAt(V3D(p.x, p.y, p.z));
        if (w < 0) {
            ++dead;
            std::cout << "[wall] 判死的点: (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
        } else {
            wmin = std::min(wmin, w);
        }
    }
    if (ghost_w) *ghost_w = m.persistencyAt(ghost);
    return {dead, wmin};
}

}  // namespace

// ELite 式 5: 命中证据随"端点到地图点的距离 x"衰减, 到 1.2σ (= inlier_dist) 归零。
TEST(OverlayWEvidence, HitStrengthDecaysWithDistance)
{
    const double sigma = 0.25;                      // = voxel * 5/6 (自动值), 1.2σ = 0.3 = inlier_dist
    // (端点到地图点的距离, 期望的一票增量) —— 增量 = 8 · logit(1 − f(x/σ)) / logit(0.9)
    const std::vector<std::pair<double, int>> cases = {
        {0.000, 8}, {0.050, 7}, {0.125, 5}, {0.250, 1}, {0.290, 0}, {0.300, 0}};
    for (const auto& c : cases) {
        OverlayMapMaintainer m;
        MaintainerConfig cfg = eliteCfg();
        cfg.persistency_unit = 0;                   // frame: 单帧就能观察
        EXPECT_LE(cfg.persistency_sigma_hit, 0.0);  // 用例不设 σ: 走自动值, 断言它与 inlier_dist 的关系
        m.configure(cfg);
        m.setBaseEmbeddedWeights(false);
        CloudType::Ptr one(new CloudType());
        one->push_back(mkPt(1.0f, 0.0f, 0.0f));     // 地图点 p
        m.setBaseMap("", one);
        CloudType::Ptr scan(new CloudType());
        scan->push_back(mkPt(1.0f, static_cast<float>(c.first), 0.0f));   // 端点: 距 p 恰好 x (同体素)
        m.onLocalized(mkPose(0, 0, 0), scan, lowFitnessMatch(), true);
        EXPECT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 128 + c.second)
            << "x=" << c.first << " m: 期望 +" << c.second;
    }
}

// 侵蚀的直接机理: 今天的端点落在地图点的**相邻体素**里 (扫描与地图差几厘米, 跨过体素边界)。
// 旧模型拿不到命中 (= +0); 新模型按距离记, 与同体素时完全一样。
TEST(OverlayWEvidence, HitCrossesVoxelBoundary)
{
    const std::vector<std::pair<double, int>> cases = {{0.050, 7}, {0.250, 1}, {0.290, 0}};
    for (const auto& c : cases) {
        OverlayMapMaintainer m;
        MaintainerConfig cfg = eliteCfg();
        cfg.persistency_unit = 0;
        m.configure(cfg);
        m.setBaseEmbeddedWeights(false);
        CloudType::Ptr one(new CloudType());
        one->push_back(mkPt(1.0f, 0.0f, 0.0f));     // 地图点 p: 体素 y ∈ [0, 0.3)
        m.setBaseMap("", one);
        CloudType::Ptr scan(new CloudType());
        scan->push_back(mkPt(1.0f, static_cast<float>(-c.first), 0.0f));   // 端点在体素 y ∈ [-0.3, 0)
        m.onLocalized(mkPose(0, 0, 0), scan, lowFitnessMatch(), true);
        EXPECT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 128 + c.second)
            << "跨体素, x=" << c.first << " m: 期望 +" << c.second;
    }
}

// ELite 式 5 (自由项): 射线必须**贴着地图点经过** (≤ 1.2σ_free = 6cm) 才算自由证据。
// 旧模型: 采样点落在体素里就扣满 8 —— 墙前 30cm 内掠过的射线都给真实墙面记一票。
TEST(OverlayWEvidence, FreeEvidenceNeedsToPassClose)
{
    // (射线到地图点的垂直距离, 期望的一票减量)
    const std::vector<std::pair<double, int>> cases = {
        {0.000, 8}, {0.040, 2}, {0.050, 1}, {0.060, 0}, {0.150, 0}};
    for (const auto& c : cases) {
        OverlayMapMaintainer m;
        MaintainerConfig cfg = eliteCfg();
        cfg.persistency_unit = 0;
        m.configure(cfg);
        m.setBaseEmbeddedWeights(false);
        CloudType::Ptr one(new CloudType());
        one->push_back(mkPt(1.0f, 0.0f, 0.0f));
        m.setBaseMap("", one);
        // 传感器在 (0, off, 0), 射线沿 +x: 采样点 (1.0, off, 0) 恰好在地图点旁 off 处
        CloudType::Ptr scan(new CloudType());
        scan->push_back(mkPt(3.0f, 0.0f, 0.0f));
        m.onLocalized(mkPose(0, c.first, 0), scan, lowFitnessMatch(), true);
        EXPECT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), 128 - c.second)
            << "射线距地图点 " << c.first << " m: 期望 -" << c.second;
    }
}

// 核心回归: 今天的墙面与地图差 10cm (两个方向), 沿墙行驶 12m (60 个视角), 含掠射射线。
// 一个底图点都不能被侵蚀掉。旧实现在这个场景里成片判死。
TEST(OverlayWEvidence, OffsetWallIsNotEroded)
{
    for (const double dx : {0.10, -0.10, -0.15}) {
        double ms = 0.0;
        const auto r = driveAlongWall(dx, 60, false, nullptr, &ms);
        std::cout << "[wall] 偏移 " << (dx > 0 ? "+" : "") << dx << " m: 判死 " << r.first
                  << " / 320, 最低 w " << r.second << ", " << ms << " ms/帧" << std::endl;
        EXPECT_EQ(r.first, 0) << "偏移 " << dx << " m: 底图被侵蚀";
        // 占据优先: 附近一直有端点的体素不应低于中性值 (128)
        EXPECT_GE(r.second, 128) << "偏移 " << dx << " m: 有端点支撑的体素仍被 free 票拖低";
        EXPECT_LT(ms, 60.0) << "账本每帧耗时 (" << ms << " ms) 超出跟踪线程预算";
    }
}

// 占据证据优先: 同一帧里既有端点落在地图点 inlier_dist 内、又有射线从它正上方穿过 ->
// 不叠加自由证据。(挪走的货物附近没有端点, 不受这条保护; 见 GhostInFrontOfWallIsRemoved)
TEST(OverlayWEvidence, OccupiedEvidenceDominatesFree)
{
    for (const bool with_endpoint : {true, false}) {
        OverlayMapMaintainer m;
        MaintainerConfig cfg = eliteCfg();
        cfg.persistency_unit = 0;                   // frame
        m.configure(cfg);
        m.setBaseEmbeddedWeights(false);
        CloudType::Ptr one(new CloudType());
        one->push_back(mkPt(1.0f, 0.0f, 0.0f));     // 地图点 p
        m.setBaseMap("", one);
        CloudType::Ptr scan(new CloudType());
        scan->push_back(mkPt(3.0f, 0.0f, 0.0f));    // 射线沿 +x: 采样点正好穿过 p -> 自由证据满强度 -8
        if (with_endpoint) scan->push_back(mkPt(1.0f, 0.2f, 0.0f));   // 端点距 p 0.2m (< inlier_dist): +2
        m.onLocalized(mkPose(0, 0, 0), scan, lowFitnessMatch(), true);
        // 128 + 2 - 8 = 122 (没有优先规则)  |  128 + 2 = 130 (占据优先)  |  128 - 8 = 120 (没有端点)
        EXPECT_EQ(m.persistencyAt(V3D(1.0, 0.0, 0.0)), with_endpoint ? 130 : 120);
    }
}

// 删除侧必须还工作: 墙前 0.5m 处的一个 (已不存在的) 底图点, 机器人经过后被删掉, 墙面不受影响。
TEST(OverlayWEvidence, GhostInFrontOfWallIsRemoved)
{
    int ghost_w = 999, ghost_at = -1;
    double ms = 0.0;
    const auto r = driveAlongWall(0.0, 90, true, &ghost_w, &ms, false, 5, &ghost_at);
    std::cout << "[ghost] 判死的墙点 " << r.first << ", 鬼影 w = " << ghost_w << " (第 30 个视角经过它, 判死于第 "
              << ghost_at << " 个视角), " << ms << " ms/帧" << std::endl;
    EXPECT_EQ(ghost_w, -1) << "墙前的鬼影没有被删掉";
    EXPECT_EQ(r.first, 0) << "删鬼影的同时墙面被侵蚀了";
}

// 射线抽样按点序每 stride 个取一个。点序有周期结构 (真实扫描 = 环 x 方位; 这里 = z 排) 且周期与
// stride 不互素时, 固定相位会永远跳过某些方向: 24 排 / stride 3 -> 只有 z 序号 ≡ 0 的排被采样,
// 经过鬼影的那一排从不采样 -> 鬼影一票自由证据都收不到 (实测 w 恒为 128)。
// 抽样相位逐帧轮转后, 任何点序 / 步长组合下都能覆盖到。
TEST(OverlayWEvidence, RaySubsamplingDoesNotAliasOnScanOrder)
{
    int ghost_w = 999, ghost_at = -1;
    const auto r = driveAlongWall(0.0, 40, true, &ghost_w, nullptr, /*z_rows_inner=*/true, /*stride=*/3, &ghost_at);
    std::cout << "[alias] z 排在内层 + stride 3: 鬼影 w = " << ghost_w << ", 墙判死 " << r.first << std::endl;
    EXPECT_LT(ghost_w, 100) << "抽样相位与点序混叠: 鬼影收不到自由证据";
    EXPECT_EQ(r.first, 0);
}

// 对抗场景: 狭窄走廊里从 25m 外接近 60m 长墙。扫描按真实角分辨率 (0.3°) 生成 —— 远处沿墙点间距
// 被入射角拉大到 ~1m, 很多墙面体素在该视角里附近没有端点 (不受占据优先保护), 而掠射射线正好
// 从它们旁边经过。这是自由证据最容易误伤墙面的几何; σ_free 取 voxel/6 (5cm) 就是为了它。
TEST(OverlayWEvidence, CorridorApproachDoesNotErodeFarWall)
{
    for (const double dx : {-0.10, 0.0}) {
        OverlayMapMaintainer m;
        MaintainerConfig cfg = eliteCfg();
        cfg.radius = 80.0;
        m.configure(cfg);
        m.setBaseEmbeddedWeights(false);
        const CloudType::Ptr wall(new CloudType());
        for (int iy = 0; iy < 200; ++iy)
            for (int iz = 0; iz < 8; ++iz)
                wall->push_back(mkPt(0.152f, 0.3f * iy + 0.137f, 0.3f * iz + 0.118f));   // y ∈ [0, 60]
        m.setBaseMap("", wall);
        const MatchResult mr = lowFitnessMatch();
        int dead = 0, wmin = 999;
        for (int k = 0; k < 300 && dead == 0; ++k) {        // 机器人 y: -15 -> 45, 0.2m/视角
            const double yt = -15.0 + 0.2 * k;
            const double D = 1.25;                           // 传感器到墙的横向距离
            CloudType::Ptr scan(new CloudType());
            const double zrows[8] = {0.129, 0.429, 0.729, 1.029, 1.329, 1.629, 1.929, 2.229};
            for (int ia = -200; ia <= 200; ++ia) {           // 方位角 0.3° 步长, 朝行进方向 ±60°
                const double th = ia * 0.3 * M_PI / 180.0;
                const double diry = std::cos(th), dirx = std::sin(th);      // x 分量: 朝墙 (-x)
                if (dirx <= 1e-6) continue;
                const double L = (D - dx) / dirx;                         // 到"今天的墙面"的距离
                if (L > 40.0) continue;                                    // 量程 40m
                const double ye = yt + L * diry;
                if (ye < 0.0 || ye > 60.0) continue;
                for (const double ze : zrows)
                    scan->push_back(mkPt(static_cast<float>(-(D - dx)), static_cast<float>(ye - yt),
                                         static_cast<float>(ze - 1.2)));
            }
            m.onLocalized(mkPose(0.152 + D, yt, 1.2), scan, mr, true);
            dead = 0; wmin = 999;
            for (const auto& p : wall->points) {
                const int w = m.persistencyAt(V3D(p.x, p.y, p.z));
                if (w < 0) ++dead;
                else wmin = std::min(wmin, w);
            }
        }
        std::cout << "[approach] 偏移 " << (dx > 0 ? "+" : "") << dx << " m: 判死 " << dead << " / 1600, 最低 w "
                  << (dead ? -1 : wmin) << std::endl;
        EXPECT_EQ(dead, 0) << "偏移 " << dx << " m: 远处掠射把还没开到的墙面侵蚀了";
        EXPECT_GE(wmin, 120) << "偏移 " << dx << " m: 墙面 w 被远处掠射明显拖低";
    }
}
