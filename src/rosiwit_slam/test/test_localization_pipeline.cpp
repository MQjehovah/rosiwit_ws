// test/test_localization_pipeline.cpp
// 定位各阶段契约测试: 对合成场景验证配准精度 / 局部地图 / 监测器 / 工厂.
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <pcl/common/transforms.h>
#include "algorithms/localization/localization_factory.h"
#include "algorithms/localization/localization_pipeline.h"
#include "algorithms/localization/localization_utils.h"

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
    p->setMap(makeScene());
    auto sub = p->getLocalMap(makePose(3, 3, 0, 0));
    EXPECT_GT(sub->size(), 0u);
}

TEST(LocalMapTest, RadiusSubmapLimitsSize) {
    auto p = LocalizationFactory::createLocalMapProvider("radius_submap");
    ASSERT_NE(p, nullptr);
    LocalMapConfig cfg; cfg.voxel_resolution = 0.2; cfg.radius = 2.0; cfg.cell_size = 4.0;
    p->configure(cfg);
    p->setMap(makeScene());
    auto near_center = p->getLocalMap(makePose(3, 3, 0, 0));
    auto far_corner  = p->getLocalMap(makePose(0, 0, 0, 0));
    EXPECT_GT(near_center->size(), 0u);
    // 半径限制下局部地图远小于全图
    auto all = LocalizationFactory::createLocalMapProvider("global");
    LocalMapConfig gcfg; gcfg.voxel_resolution = 0.2;
    all->configure(gcfg); all->setMap(makeScene());
    EXPECT_LT(near_center->size(), all->getLocalMap(makePose(0,0,0,0))->size());
    (void)far_corner;
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

// ---- 工厂注册完整性 ----
TEST(LocalizationFactoryTest, RegistersAllStages) {
    for (const auto& n : LocalizationFactory::listMatchers())          EXPECT_NE(LocalizationFactory::createMatcher(n), nullptr);
    for (const auto& n : LocalizationFactory::listLocalMapProviders()) EXPECT_NE(LocalizationFactory::createLocalMapProvider(n), nullptr);
    for (const auto& n : LocalizationFactory::listMotionPriors())      EXPECT_NE(LocalizationFactory::createMotionPrior(n), nullptr);
    for (const auto& n : LocalizationFactory::listGlobalLocalizers())  EXPECT_NE(LocalizationFactory::createGlobalLocalizer(n), nullptr);
    for (const auto& n : LocalizationFactory::listMonitors())          EXPECT_NE(LocalizationFactory::createMonitor(n), nullptr);
}
