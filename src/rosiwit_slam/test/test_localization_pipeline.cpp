// test/test_localization_pipeline.cpp
// 定位各阶段契约测试: 对合成场景验证配准精度 / 局部地图 / 监测器 / 工厂.
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <pcl/common/transforms.h>
#include "algorithms/localization/localization_factory.h"
#include "algorithms/localization/localization_pipeline.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/constraints/planar_constraint.h"
#include "algorithms/localization/hypothesis/hypothesis_manager.h"
#include "algorithms/localization/motion_prior/imu_motion_prior.h"
#include "slam_core/pose_filter.h"
#include "algorithms/map_store/map_store_factory.h"
#include "io/xzmap_writer.h"
#include "io/xzmap_reader.h"

using namespace rosiwit_slam;

namespace {

// 合成一个 8m x 8m 房间: 地面 + 四面墙 + 两个柱子, 提供充分的几何约束.
CloudType::Ptr makeScene() {
    CloudType::Ptr c(new CloudType());
    const double S = 8.0;
    for (double x = 0; x <= S; x += 0.1)
        for (double y = 0; y <= S; y += 0.1) {
            c->push_back(PointType(static_cast<float>(x), static_cast<float>(y), 0.0f));
        }
    for (double t = 0; t <= S; t += 0.05)
        for (double z = 0.2; z <= 2.0; z += 0.1) {
            c->push_back(PointType(static_cast<float>(t), 0.0f, static_cast<float>(z)));
            c->push_back(PointType(static_cast<float>(t), static_cast<float>(S), static_cast<float>(z)));
            c->push_back(PointType(0.0f, static_cast<float>(t), static_cast<float>(z)));
            c->push_back(PointType(static_cast<float>(S), static_cast<float>(t), static_cast<float>(z)));
        }
    // 柱子 (竖直边缘, 供 line 特征/退化约束)
    for (double z = 0.0; z <= 2.0; z += 0.05) {
        for (double a = 0; a < 6.28; a += 0.2) {
            c->push_back(PointType(3.0f + 0.2f * static_cast<float>(std::cos(a)),
                                   3.0f + 0.2f * static_cast<float>(std::sin(a)),
                                   static_cast<float>(z)));
            c->push_back(PointType(6.0f + 0.2f * static_cast<float>(std::cos(a)),
                                   5.0f + 0.2f * static_cast<float>(std::sin(a)),
                                   static_cast<float>(z)));
        }
    }
    return c;
}

// 从世界地图生成"机体坐标系"扫描: scan = T_true^{-1} * map
CloudType::Ptr makeScanFromMap(const CloudType::Ptr& map, const PoseStamped& T_true) {
    Eigen::Matrix4f T = poseToMatrix(T_true).cast<float>().inverse();
    CloudType::Ptr scan(new CloudType());
    pcl::transformPointCloud(*map, *scan, T);
    return scan;
}

PoseStamped makePose(double x, double y, double z, double yaw_deg, double t = 0.0) {
    PoseStamped p;
    p.time = t;
    p.trans = V3D(x, y, z);
    p.rot = Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, V3D::UnitZ()).toRotationMatrix();
    return p;
}

} // namespace

// ---- 配准器契约: 各策略应能从扰动初值恢复真值位姿 ----
class MatcherContractTest : public ::testing::TestWithParam<std::string> {};

TEST_P(MatcherContractTest, RecoversPoseFromPerturbedGuess) {
    auto matcher = LocalizationFactory::createMatcher(GetParam());
    ASSERT_NE(matcher, nullptr);

    ScanMatcherConfig cfg;
    cfg.voxel_resolution = 0.2;
    cfg.max_iterations = 50;
    cfg.max_correspondence_dist = 1.0;
    cfg.corr_dist_init = 2.0;
    cfg.corr_dist_min = 0.5;
    cfg.min_correspondences = 10;
    matcher->configure(cfg);

    CloudType::Ptr map = makeScene();
    matcher->setTarget(map);

    const PoseStamped T_true = makePose(3.0, 3.0, 0.0, 8.0);
    CloudType::Ptr scan = makeScanFromMap(map, T_true);

    // 扰动初值: 平移 0.3m, 航向 +3°
    PoseStamped guess = makePose(3.2, 2.8, 0.0, 11.0);
    MatchResult res;
    ASSERT_TRUE(matcher->match(scan, guess, res));
    ASSERT_TRUE(res.converged);

    const double dt = (res.pose.trans - T_true.trans).norm();
    double dyaw = yawFromRot(T_true.rot.transpose() * res.pose.rot) * 180.0 / M_PI;
    EXPECT_LT(dt, 0.15) << "matcher=" << GetParam();
    EXPECT_LT(std::fabs(dyaw), 3.0) << "matcher=" << GetParam();
    EXPECT_GT(res.fitness, 0.5) << "matcher=" << GetParam();
}

INSTANTIATE_TEST_SUITE_P(AllMatchers, MatcherContractTest,
                         ::testing::ValuesIn(LocalizationFactory::listMatchers()));

// ---- 局部地图契约 ----
TEST(LocalMapTest, GlobalReturnsWholeMap) {
    auto p = LocalizationFactory::createLocalMapProvider("global");
    ASSERT_NE(p, nullptr);
    LocalMapConfig cfg; cfg.voxel_resolution = 0.2;
    p->configure(cfg);
    p->setMap("", makeScene());
    auto sub = p->getLocalMap(makePose(3, 3, 0, 0));
    EXPECT_GT(sub->size(), 0u);
}

TEST(LocalMapTest, RadiusSubmapLimitsSize) {
    auto p = LocalizationFactory::createLocalMapProvider("radius_submap");
    ASSERT_NE(p, nullptr);
    LocalMapConfig cfg; cfg.voxel_resolution = 0.2; cfg.radius = 2.0; cfg.cell_size = 4.0;
    p->configure(cfg);
    p->setMap("", makeScene());
    auto near_center = p->getLocalMap(makePose(3, 3, 0, 0));
    auto far_corner  = p->getLocalMap(makePose(0, 0, 0, 0));
    EXPECT_GT(near_center->size(), 0u);
    // 半径限制下局部地图远小于全图
    auto all = LocalizationFactory::createLocalMapProvider("global");
    LocalMapConfig gcfg; gcfg.voxel_resolution = 0.2;
    all->configure(gcfg); all->setMap("", makeScene());
    EXPECT_LT(near_center->size(), all->getLocalMap(makePose(0,0,0,0))->size());
    (void)far_corner;
}

TEST(LocalMapTest, KeyframeSubmapFromInjectedKeyframes) {
    auto p = LocalizationFactory::createLocalMapProvider("keyframe_submap");
    ASSERT_NE(p, nullptr);
    LocalMapConfig cfg;
    cfg.voxel_resolution = 0.0;  // 关闭降采样, 便于断言
    cfg.radius = 3.0;
    cfg.max_keyframes = 5;
    p->configure(cfg);
    p->setMap("", makeScene());

    // 注入 2x2 关键帧, 每帧一小团机体系点云
    std::vector<LocalKeyframe> kfs;
    for (double cx = 2.0; cx <= 6.0; cx += 4.0)
        for (double cy = 2.0; cy <= 6.0; cy += 4.0) {
            LocalKeyframe k;
            k.pose.trans = V3D(cx, cy, 0.0);
            k.pose.rot = M3D::Identity();
            k.cloud.reset(new CloudType());
            for (double dx = -1.0; dx <= 1.0; dx += 0.25)
                for (double dy = -1.0; dy <= 1.0; dy += 0.25)
                    k.cloud->push_back(PointType(static_cast<float>(dx),
                                                 static_cast<float>(dy), 0.5f));
            kfs.push_back(std::move(k));
        }
    p->setKeyframes(kfs);

    // (3,3) 半径 3 内只含 (2,2) 一帧
    auto local = p->getLocalMap(makePose(3, 3, 0, 0));
    EXPECT_GT(local->size(), 0u);
    EXPECT_LE(local->size(), kfs[0].cloud->size());
}

TEST(XzMapIoTest, WriteReadRoundTrip) {
    std::vector<io::XzMapKeyframe> kfs(3);
    for (int i = 0; i < 3; ++i) {
        kfs[i].pose.x = static_cast<double>(i);
        kfs[i].pose.y = 2.0;
        kfs[i].pose.yaw = 0.1 * i;
        kfs[i].points = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
    }
    const std::string path = std::string(::testing::TempDir()) + "rosiwit_roundtrip.xzmap";
    ASSERT_TRUE(io::writeXzMap(path, "mapname", kfs));

    std::vector<io::XzMapKeyframe> rd;
    std::string name;
    ASSERT_TRUE(io::readXzMap(path, rd, &name));
    EXPECT_EQ(name, "mapname");
    ASSERT_EQ(rd.size(), 3u);
    EXPECT_NEAR(rd[0].pose.x, 0.0, 1e-6);
    EXPECT_NEAR(rd[1].pose.x, 1.0, 1e-6);
    EXPECT_NEAR(rd[2].pose.yaw, 0.2, 1e-3);
    ASSERT_EQ(rd[0].points.size(), 2u);
    EXPECT_NEAR(rd[0].points[0].x(), 1.0, 1e-3);
    EXPECT_NEAR(rd[1].points[1].z(), 6.0, 1e-3);
}

// ---- 监测器契约 ----
TEST(MonitorTest, ThresholdDetectsLost) {
    auto m = LocalizationFactory::createMonitor("threshold");
    ASSERT_NE(m, nullptr);
    MonitorConfig cfg; cfg.credible_fitness = 0.75; cfg.lost_fitness = 0.5; cfg.lost_patience_low = 3;
    m->configure(cfg);
    m->reset();

    MonitorInput good; good.match.converged = true; good.match.fitness = 0.9;
    EXPECT_EQ(m->update(good), MonitorLevel::CREDIBLE);

    MonitorInput bad; bad.match.converged = true; bad.match.fitness = 0.1;
    EXPECT_NE(m->update(bad), MonitorLevel::LOST);
    EXPECT_NE(m->update(bad), MonitorLevel::LOST);
    EXPECT_EQ(m->update(bad), MonitorLevel::LOST);
}

// ---- 全局重定位契约 ----
TEST(GlobalLocalizerTest, BbsRecoversPoseWithoutInit) {
    auto loc = LocalizationFactory::createGlobalLocalizer("bbs");
    ASSERT_NE(loc, nullptr);
    GlobalLocalizeConfig cfg;
    cfg.voxel_resolution = 0.4;
    cfg.bbs_theta_bins = 60;
    cfg.bbs_max_levels = 4;
    cfg.bbs_z_min = -1.0;
    cfg.bbs_z_max = 1.0;
    cfg.min_score = 0.3;
    loc->configure(cfg);

    CloudType::Ptr map = makeScene();
    ASSERT_TRUE(loc->setMap("", map));

    const PoseStamped gt = makePose(3.0, 3.0, 0.0, 20.0);
    CloudType::Ptr scan = makeScanFromMap(map, gt);

    GlobalCandidate cand;
    ASSERT_TRUE(loc->localize(scan, cand));
    const double derr = (cand.pose.trans - gt.trans).norm();
    const double dyaw = std::fabs(yawFromRot(gt.rot.transpose() * cand.pose.rot)) * 180.0 / M_PI;
    EXPECT_LT(derr, 0.6);
    EXPECT_LT(dyaw, 12.0);
}

TEST(GlobalLocalizerTest, BbsUpdateEnvironmentSmoke) {
    auto loc = LocalizationFactory::createGlobalLocalizer("bbs");
    ASSERT_NE(loc, nullptr);
    GlobalLocalizeConfig cfg;
    cfg.voxel_resolution = 0.4;
    cfg.bbs_max_levels = 4;
    cfg.min_score = 0.3;
    loc->configure(cfg);
    CloudType::Ptr map = makeScene();
    ASSERT_TRUE(loc->setMap("", map));

    // lifelong: 叠加层(环境变化)刷新后仍可正常定位(不崩溃)
    CloudType::Ptr overlay(new CloudType());
    for (double x = 3.0; x <= 4.0; x += 0.1)
        for (double y = 3.0; y <= 4.0; y += 0.1)
            for (double z = 0.2; z <= 1.0; z += 0.1)
                overlay->push_back(PointType(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
    loc->updateEnvironment(overlay);

    GlobalCandidate c;
    loc->localize(makeScanFromMap(map, makePose(3.0, 3.0, 0.0, 20.0)), c);
    SUCCEED();
}

TEST(GlobalLocalizerTest, NoneReturnsFalse) {
    auto loc = LocalizationFactory::createGlobalLocalizer("none");
    ASSERT_NE(loc, nullptr);
    GlobalCandidate c;
    EXPECT_FALSE(loc->localize(makeScene(), c));
}

TEST(GlobalLocalizerTest, BbsTopKReturnsCandidates) {
    auto loc = LocalizationFactory::createGlobalLocalizer("bbs");
    ASSERT_NE(loc, nullptr);
    GlobalLocalizeConfig cfg;
    cfg.voxel_resolution = 0.4;
    cfg.bbs_theta_bins = 60;
    cfg.bbs_max_levels = 4;
    cfg.min_score = 0.3;
    cfg.top_k = 3;
    loc->configure(cfg);
    CloudType::Ptr map = makeScene();
    ASSERT_TRUE(loc->setMap("", map));
    CloudType::Ptr scan = makeScanFromMap(map, makePose(3.0, 3.0, 0.0, 20.0));

    std::vector<GlobalCandidate> cands;
    const int n = loc->localizeTopK(scan, 3, cands);
    EXPECT_GE(n, 1);
    if (n > 0) {
        const PoseStamped gt = makePose(3.0, 3.0, 0.0, 20.0);
        EXPECT_LT((cands.front().pose.trans - gt.trans).norm(), 0.6);
    }
}

// ---- FPFH-RANSAC 6DoF 重定位: 仅冒烟 (RANSAC 随机, 不强制成功) ----
TEST(GlobalLocalizerTest, FpfhRansacSmoke) {
    auto loc = LocalizationFactory::createGlobalLocalizer("fpfh_ransac");
    ASSERT_NE(loc, nullptr);
    GlobalLocalizeConfig cfg;
    cfg.voxel_resolution = 0.5;
    cfg.fpfh_normal_radius = 1.0;
    cfg.fpfh_feature_radius = 4.0;
    cfg.fpfh_max_iter = 3000;
    cfg.min_score = 0.2;
    loc->configure(cfg);
    CloudType::Ptr map = makeScene();
    ASSERT_TRUE(loc->setMap("", map));
    CloudType::Ptr scan = makeScanFromMap(map, makePose(3.0, 3.0, 0.0, 20.0));
    GlobalCandidate c;
    loc->localize(scan, c);  // 不崩溃即可
    SUCCEED();
}

// ---- 可切换鲁棒核: 各核均应稳定恢复 ----
TEST(MatcherRobustKernelTest, RecoversWithEachKernel) {
    const RobustKernel kernels[] = {RobustKernel::NONE, RobustKernel::HUBER,
                                    RobustKernel::CAUCHY, RobustKernel::TUKEY,
                                    RobustKernel::GNC};
    CloudType::Ptr map = makeScene();
    const PoseStamped T_true = makePose(3.0, 3.0, 0.0, 8.0);
    CloudType::Ptr scan = makeScanFromMap(map, T_true);
    for (RobustKernel k : kernels) {
        auto m = LocalizationFactory::createMatcher("pp_icp");
        ASSERT_NE(m, nullptr);
        ScanMatcherConfig cfg;
        cfg.voxel_resolution = 0.2;
        cfg.max_iterations = 50;
        cfg.corr_dist_init = 2.0;
        cfg.corr_dist_min = 0.5;
        cfg.robust_kernel = k;
        m->configure(cfg);
        m->setTarget(map);
        MatchResult res;
        ASSERT_TRUE(m->match(scan, makePose(3.2, 2.8, 0.0, 11.0), res));
        EXPECT_LT((res.pose.trans - T_true.trans).norm(), 0.2);
        EXPECT_TRUE(res.cov.allFinite());
    }
}

// ---- 地图维护: 非破坏性叠加 ----
TEST(MaintainerTest, OverlayAccumulatesOnChange) {
    auto mnt = LocalizationFactory::createMapMaintainer("overlay");
    ASSERT_NE(mnt, nullptr);
    MaintainerConfig cfg;
    cfg.fitness_trigger = 0.5;
    cfg.min_confirm = 2;
    cfg.min_keyframe_dist = 0.5;
    cfg.voxel = 0.2;
    cfg.radius = 5.0;
    mnt->configure(cfg);
    mnt->setBaseMap("", nullptr);

    PoseStamped pose = makePose(0, 0, 0, 0);
    CloudType::Ptr scan(new CloudType());
    for (double x = -1.0; x <= 1.0; x += 0.1)
        for (double y = -1.0; y <= 1.0; y += 0.1)
            scan->push_back(PointType(static_cast<float>(x), static_cast<float>(y), 0.5f));

    MatchResult low; low.converged = true; low.fitness = 0.1;
    mnt->onLocalized(pose, scan, low, true);   // 第1帧: 未达确认数
    EXPECT_EQ(mnt->getOverlay(pose)->size(), 0u);
    mnt->onLocalized(pose, scan, low, true);   // 第2帧: 并入叠加层
    EXPECT_GT(mnt->getOverlay(pose)->size(), 0u);

    // fitness 恢复 -> 不再累积 (用远处理想帧, 避免雕刻干扰)
    const size_t before = mnt->getOverlay(pose)->size();
    CloudType::Ptr scan1(new CloudType());
    scan1->push_back(PointType(0.0f, 0.0f, 0.0f));
    MatchResult good; good.converged = true; good.fitness = 0.95;
    for (int i = 0; i < 3; ++i) mnt->onLocalized(makePose(2, 0, 0, 0), scan1, good, true);
    EXPECT_EQ(mnt->getOverlay(pose)->size(), before);
}

// ---- 长期: 持久化过滤, 瞬态变化不入叠加层 ----
TEST(MaintainerTest, DurableChangeFilter) {
    auto mnt = LocalizationFactory::createMapMaintainer("overlay");
    ASSERT_NE(mnt, nullptr);
    MaintainerConfig cfg;
    cfg.fitness_trigger = 0.5;
    cfg.min_confirm = 1;
    cfg.min_keyframe_dist = 0.0;
    cfg.voxel = 0.2;
    cfg.radius = 5.0;
    cfg.persist_hits = 3;    // 需观测 3 次才并入
    mnt->configure(cfg);
    mnt->setBaseMap("", nullptr);

    PoseStamped pose = makePose(0, 0, 0, 0);
    CloudType::Ptr scan(new CloudType());
    scan->push_back(PointType(0.0f, 0.0f, 0.0f));
    MatchResult low; low.converged = true; low.fitness = 0.1;

    mnt->onLocalized(pose, scan, low, true);
    EXPECT_EQ(mnt->getOverlay(pose)->size(), 0u);   // 1 次: 未达阈值
    mnt->onLocalized(pose, scan, low, true);
    EXPECT_EQ(mnt->getOverlay(pose)->size(), 0u);   // 2 次: 未达阈值
    mnt->onLocalized(pose, scan, low, true);
    EXPECT_GT(mnt->getOverlay(pose)->size(), 0u);   // 3 次: 并入
}

// ---- 动态/变化物剔除: 未入图的点被忽略, 定位仍以静态结构为准 ----
TEST(MatcherDynamicFilterTest, IgnoresUnmappedPoints) {
    auto m = LocalizationFactory::createMatcher("pp_icp");
    ASSERT_NE(m, nullptr);
    ScanMatcherConfig cfg;
    cfg.voxel_resolution = 0.2;
    cfg.max_iterations = 50;
    cfg.corr_dist_init = 2.0;
    cfg.corr_dist_min = 0.5;
    cfg.dynamic_filter = true;
    cfg.dynamic_dist = 0.5;
    m->configure(cfg);

    CloudType::Ptr map = makeScene();
    m->setTarget(map);
    const PoseStamped T_true = makePose(3.0, 3.0, 0.0, 8.0);
    CloudType::Ptr scan = makeScanFromMap(map, T_true);
    // 注入"动态物"团簇(地图中没有): 模拟人/叉车/未入图的新堆货
    for (int i = 0; i < 800; ++i)
        scan->push_back(PointType(0.5f * ((i % 10) - 5), 0.5f * (((i / 10) % 10) - 5),
                                  1.0f + 0.02f * (i % 5)));

    MatchResult res;
    ASSERT_TRUE(m->match(scan, makePose(3.2, 2.8, 0.0, 11.0), res));
    EXPECT_LT((res.pose.trans - T_true.trans).norm(), 0.2);
}

// ---- 叠加层衰减: 长期未再观测的体素被移除 ----
TEST(MaintainerTest, DecayRemovesStaleVoxels) {
    auto mnt = LocalizationFactory::createMapMaintainer("overlay");
    ASSERT_NE(mnt, nullptr);
    MaintainerConfig cfg;
    cfg.fitness_trigger = 0.5;
    cfg.min_confirm = 1;
    cfg.min_keyframe_dist = 0.0;
    cfg.voxel = 0.2;
    cfg.radius = 5.0;
    cfg.persist_hits = 1;
    cfg.decay_frames = 5;
    mnt->configure(cfg);
    mnt->setBaseMap("", nullptr);

    PoseStamped pose = makePose(0, 0, 0, 0);
    CloudType::Ptr scan(new CloudType());
    scan->push_back(PointType(0.0f, 0.0f, 0.0f));
    MatchResult low; low.converged = true; low.fitness = 0.1;
    mnt->onLocalized(pose, scan, low, true);
    EXPECT_GT(mnt->getOverlay(pose)->size(), 0u);

    MatchResult good; good.converged = true; good.fitness = 0.95;
    for (int i = 0; i < 7; ++i) mnt->onLocalized(pose, scan, good, true);  // 帧推进, 未再观测
    EXPECT_EQ(mnt->getOverlay(pose)->size(), 0u);
}

// ---- 叠加层压制原图 (结构被移除/替换) ----
TEST(LocalizationUtilsTest, SuppressBaseByOverlay) {
    CloudType::Ptr base(new CloudType());
    for (double x = 0.0; x < 3.0; x += 0.1)
        base->push_back(PointType(static_cast<float>(x), 0.0f, 0.0f));
    CloudType::Ptr ov(new CloudType());
    ov->push_back(PointType(1.5f, 0.0f, 0.0f));
    const size_t before = base->size();
    suppressBaseByOverlay(base, ov, 0.2);
    EXPECT_LT(base->size(), before);
}

// ---- 置信门控: 位姿不可信(过桥)时不写入叠加层 ----
TEST(MaintainerTest, NoCommitWhenUnconfident) {
    auto mnt = LocalizationFactory::createMapMaintainer("overlay");
    ASSERT_NE(mnt, nullptr);
    MaintainerConfig cfg;
    cfg.fitness_trigger = 0.5;
    cfg.min_confirm = 1;
    cfg.min_keyframe_dist = 0.0;
    cfg.voxel = 0.2;
    cfg.radius = 20.0;
    cfg.persist_hits = 1;
    mnt->configure(cfg);
    mnt->setBaseMap("", nullptr);

    PoseStamped pose = makePose(0, 0, 0, 0);
    CloudType::Ptr scan(new CloudType());
    scan->push_back(PointType(5.0f, 0.0f, 0.0f));
    MatchResult low; low.converged = true; low.fitness = 0.1;
    for (int i = 0; i < 5; ++i) mnt->onLocalized(pose, scan, low, /*pose_confident=*/false);
    EXPECT_EQ(mnt->getOverlay(pose)->size(), 0u);   // 不可信 -> 不写
}

// ---- 自由空间雕刻: 货物被发走(看到更远) -> 清除鬼影 ----
TEST(MaintainerTest, RayCarveRemovesStaleGoods) {
    auto mnt = LocalizationFactory::createMapMaintainer("overlay");
    ASSERT_NE(mnt, nullptr);
    MaintainerConfig cfg;
    cfg.fitness_trigger = 0.5;
    cfg.min_confirm = 1;
    cfg.min_keyframe_dist = 0.0;
    cfg.voxel = 0.2;
    cfg.radius = 20.0;
    cfg.persist_hits = 1;
    cfg.carve_stride = 1;
    mnt->configure(cfg);
    mnt->setBaseMap("", nullptr);

    PoseStamped pose = makePose(0, 0, 0, 0);
    CloudType::Ptr scan(new CloudType());
    scan->push_back(PointType(5.0f, 0.0f, 0.0f));
    MatchResult low; low.converged = true; low.fitness = 0.1;
    mnt->onLocalized(pose, scan, low, true);          // 写入货墙 (5,0,0)
    ASSERT_GT(mnt->getOverlay(pose)->size(), 0u);

    // 货物被发走: 现在看到更远的 (10,0,0), 射线穿过 (5,0,0) -> 雕刻清除
    CloudType::Ptr scan2(new CloudType());
    scan2->push_back(PointType(10.0f, 0.0f, 0.0f));
    mnt->onLocalized(pose, scan2, low, true);

    auto ov = mnt->getOverlay(pose);
    int near5 = 0;
    for (const auto& p : ov->points) if (std::fabs(p.x - 5.0f) < 0.3f) ++near5;
    EXPECT_EQ(near5, 0);                              // (5,0,0) 处鬼影被清除
}

// ---- 地面/平面约束 ----
TEST(PlanarConstraintTest, GroundPlaneLocksZAndLevels) {
    CloudType::Ptr map(new CloudType());
    for (double x = -5.0; x <= 5.0; x += 0.1)
        for (double y = -5.0; y <= 5.0; y += 0.1)
            map->push_back(PointType(static_cast<float>(x), static_cast<float>(y), 0.0f));
    PlanarConstraint pc;
    PlanarConstraintConfig cfg;
    cfg.mode = "ground_plane";
    cfg.cell = 1.0;
    cfg.ground_band = 0.3;
    cfg.sensor_height = 0.3;
    cfg.max_tilt_deg = 35.0;
    pc.configure(cfg);
    pc.setMap(map);

    PoseStamped p = makePose(-1.0, -1.0, 5.0, 0.0);          // z 错误
    p.rot = Eigen::AngleAxisd(0.2, V3D::UnitX()).toRotationMatrix() * p.rot;  // 加 roll
    pc.apply(p);
    EXPECT_NEAR(p.trans.z(), 0.3, 1e-6);                      // 锁到地面+安装高
    const Eigen::Vector3d up = p.rot * V3D::UnitZ();
    EXPECT_NEAR(up.z(), 1.0, 0.05);                           // roll/pitch 锁平
}

TEST(PlanarConstraintTest, LockZrp) {
    PlanarConstraint pc;
    PlanarConstraintConfig cfg;
    cfg.mode = "lock_zrp";
    cfg.z = 1.0;
    pc.configure(cfg);
    PoseStamped p = makePose(2.0, 3.0, 9.0, 10.0);
    p.rot = Eigen::AngleAxisd(0.3, V3D::UnitX()).toRotationMatrix() * p.rot;
    pc.apply(p);
    EXPECT_NEAR(p.trans.z(), 1.0, 1e-9);
    const Eigen::Vector3d up = p.rot * V3D::UnitZ();
    EXPECT_NEAR(up.z(), 1.0, 1e-6);                           // roll/pitch=0
}

// ---- 多假设跟踪: 落后的假设被淘汰 ----
TEST(HypothesisManagerTest, SeedAndPrune) {
    HypothesisManager hm;
    hm.setMax(4);
    std::vector<PoseStamped> poses = {makePose(0, 0, 0, 0), makePose(3, 0, 0, 0), makePose(6, 0, 0, 0)};
    std::vector<double> scores = {0.9, 0.1, 0.8};
    EXPECT_EQ(hm.seed(poses, scores), 3);
    EXPECT_TRUE(hm.ambiguous());
    EXPECT_EQ(hm.best(), 0);

    // 第2个假设适应度持续落后 -> 淘汰
    MatchResult r; r.converged = true; r.fitness = 0.1; r.pose = poses[1];
    hm.update(1, r);
    for (int i = 0; i < 11; ++i) hm.prune(0.25, 10);
    EXPECT_EQ(hm.size(), 2);   // 假设1 被淘汰
}

// ---- P3: 配准器经统一存储底座匹配 (存储与算法解耦) ----
TEST(MatcherStoreTest, MatchersViaVoxelStore) {
    const char* names[] = {"pp_icp", "p2p_icp"};
    for (const char* nm : names) {
        auto matcher = LocalizationFactory::createMatcher(nm);
        ASSERT_NE(matcher, nullptr);
        ScanMatcherConfig cfg;
        cfg.voxel_resolution = 0.2;
        cfg.max_iterations = 50;
        cfg.corr_dist_init = 2.0;
        cfg.corr_dist_min = 0.5;
        matcher->configure(cfg);

        auto store = MapStoreFactory::create("voxel", 0.2);
        ASSERT_TRUE(store->buildFromCloud(makeScene()));
        matcher->setStore(store.get());
        ASSERT_TRUE(matcher->usesStore());

        const PoseStamped T_true = makePose(3.0, 3.0, 0.0, 8.0);
        CloudType::Ptr scan = makeScanFromMap(makeScene(), T_true);
        MatchResult res;
        ASSERT_TRUE(matcher->match(scan, makePose(3.2, 2.8, 0.0, 11.0), res)) << nm;
        EXPECT_LT((res.pose.trans - T_true.trans).norm(), 0.15) << nm;
    }
}

// ---- L11 IMU 零偏在线估计 ----
TEST(ImuBiasTest, StaticCalibEstimatesGyroBias) {
    ImuMotionPrior p;
    MotionPriorConfig cfg;
    cfg.imu_calib_samples = 50;
    cfg.use_gravity = false;
    cfg.static_gyro_std = 0.05;
    cfg.static_accel_std = 0.5;
    p.configure(cfg);
    p.reset(makePose(0, 0, 0, 0));

    const V3D tb(0.05, -0.03, 0.02);
    for (int i = 0; i < 60; ++i) {
        IMUSample s;
        s.time = i * 0.01;
        s.gyro = tb;
        s.acc = V3D(0, 0, 9.81);
        p.onImu(s);
    }
    ASSERT_TRUE(p.calibrated());
    EXPECT_NEAR((p.gyroBias() - tb).norm(), 0.0, 1e-6);
}

TEST(ImuBiasTest, NotStaticSkipsCalibration) {
    ImuMotionPrior p;
    MotionPriorConfig cfg;
    cfg.imu_calib_samples = 50;
    cfg.use_gravity = false;
    p.configure(cfg);
    p.reset(makePose(0, 0, 0, 0));

    for (int i = 0; i < 60; ++i) {
        IMUSample s;
        s.time = i * 0.01;
        s.gyro = V3D(0.5, 0.5, 0.5) * ((i % 2) ? 1.0 : -1.0);  // 大抖动
        s.acc = V3D(0, 0, 9.81);
        p.onImu(s);
    }
    EXPECT_FALSE(p.calibrated());   // 未静止 -> 跳过标定
}

// ---- L12 输出滤波: 常速外推命中解析值 ----
TEST(PoseOutputFilterTest, ExtrapolatesConstantVelocity) {
    PoseOutputFilter f;
    f.configure(1.0, 1.0, 0.5);
    PoseStamped p0 = makePose(0, 0, 0, 0); p0.time = 0.0;
    f.reset(p0);
    PoseStamped p1 = makePose(1, 0, 0, 0); p1.time = 1.0;
    f.update(1.0, p1);
    PoseStamped o;
    ASSERT_TRUE(f.poseAt(1.5, o));
    EXPECT_NEAR(o.trans.x(), 1.5, 0.05);
    EXPECT_NEAR(o.trans.y(), 0.0, 1e-6);
}

// ---- 工厂注册完整性 ----
TEST(LocalizationFactoryTest, RegistersAllStages) {
    for (const auto& n : LocalizationFactory::listMatchers())          EXPECT_NE(LocalizationFactory::createMatcher(n), nullptr);
    for (const auto& n : LocalizationFactory::listLocalMapProviders()) EXPECT_NE(LocalizationFactory::createLocalMapProvider(n), nullptr);
    for (const auto& n : LocalizationFactory::listMotionPriors())      EXPECT_NE(LocalizationFactory::createMotionPrior(n), nullptr);
    for (const auto& n : LocalizationFactory::listGlobalLocalizers())  EXPECT_NE(LocalizationFactory::createGlobalLocalizer(n), nullptr);
    for (const auto& n : LocalizationFactory::listMonitors())          EXPECT_NE(LocalizationFactory::createMonitor(n), nullptr);
    for (const auto& n : LocalizationFactory::listMapMaintainers())    EXPECT_NE(LocalizationFactory::createMapMaintainer(n), nullptr);
}
