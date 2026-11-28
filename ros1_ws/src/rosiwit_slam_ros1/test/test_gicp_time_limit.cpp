// test/test_gicp_time_limit.cpp
// 求解器时间预算 (2026-11-28) 的契约 —— 对真实的 fast_gicp 引擎, 走生产封装 GicpMatcher::match():
//   1. 不设预算 (time_limit_ms = 0)  -> 原行为: time_limited 恒为 false, 解收敛;
//   2. 预算极小                      -> 只做 1 轮就停 (第一轮总是执行), time_limited = true,
//                                       且这一轮的解比猜测更接近真值 (至少做了一次步);
//   3. 预算极大                      -> 从不触发, 解与不设预算时逐位相同 (预算没触发就不改变任何结果);
//   4. 迭代上限                      -> 迭代次数不超过 max_iterations, 且这不叫"因预算停止".
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "algorithms/localization/localization_factory.h"
#include "slam_core/i_scan_matcher.h"
#include "slam_core/localization_stage_types.h"

using namespace rosiwit_slam;

namespace {

PointType mkPt(double x, double y, double z)
{
    PointType q;
    q.x = static_cast<float>(x);
    q.y = static_cast<float>(y);
    q.z = static_cast<float>(z);
    q.normal_x = q.normal_y = q.normal_z = 0.0f;
    q.curvature = 0.0f;
    q.intensity = 0.0f;
    return q;
}

// 世界系场景: 半径 6m 的圆柱墙 (5 层高度) + 室内两个不对称的方箱, 让平移/旋转都可观。
CloudType::Ptr makeWorld()
{
    CloudType::Ptr c(new CloudType());
    for (double a = 0.0; a < 360.0; a += 0.5) {
        const double r = a * M_PI / 180.0;
        for (double z = 0.3; z <= 1.8; z += 0.3)
            c->push_back(mkPt(6.0 * std::cos(r), 6.0 * std::sin(r), z));
    }
    auto box = [&](double cx, double cy, double s) {
        for (double t = -s; t <= s; t += 0.1)
            for (double z = 0.2; z <= 1.0; z += 0.2) {
                c->push_back(mkPt(cx + t, cy - s, z));
                c->push_back(mkPt(cx + t, cy + s, z));
                c->push_back(mkPt(cx - s, cy + t, z));
                c->push_back(mkPt(cx + s, cy + t, z));
            }
    };
    box(2.5, 1.0, 0.5);
    box(-1.5, -3.0, 0.7);
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}

// 传感器在 truth 处看到的那部分场景 (前向 ±100°, 1m 以外), 表达在传感器系。
CloudType::Ptr scanAt(const CloudType::Ptr& world, const PoseStamped& truth)
{
    CloudType::Ptr c(new CloudType());
    for (const auto& p : world->points) {
        const double dx = p.x - truth.trans.x(), dy = p.y - truth.trans.y(), dz = p.z - truth.trans.z();
        const double range = std::hypot(std::hypot(dx, dy), dz);
        if (range < 1.0 || range > 8.0) continue;
        const double bearing = std::atan2(dy, dx) - std::atan2(truth.rot(1, 0), truth.rot(0, 0));
        if (std::fabs(std::atan2(std::sin(bearing), std::cos(bearing))) > 100.0 * M_PI / 180.0) continue;
        const Eigen::Vector3d b = truth.rot.transpose() * Eigen::Vector3d(dx, dy, dz);
        c->push_back(mkPt(b.x(), b.y(), b.z()));
    }
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}

PoseStamped mkPose(double x, double y, double yaw, double z = 0.0)
{
    PoseStamped p;
    p.time = 1000.0;
    p.trans = V3D(x, y, z);
    p.rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return p;
}

// 猜测 = 真值 + 固定偏差 (真实跟踪里的新息量级再放大一些: 5cm / 3cm / 2°)
PoseStamped perturb(const PoseStamped& t, double dx, double dy, double dyaw)
{
    const Eigen::Isometry3d T = [&] {
        Eigen::Isometry3d e = Eigen::Isometry3d::Identity();
        e.linear() = t.rot;
        e.translation() = t.trans;
        return e;
    }();
    Eigen::Isometry3d d = Eigen::Isometry3d::Identity();
    d.linear() = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    d.translation() = Eigen::Vector3d(dx, dy, 0.0);
    const Eigen::Isometry3d g = d * T;
    PoseStamped p;
    p.time = t.time;
    p.rot = g.linear();
    p.trans = g.translation();
    return p;
}

std::unique_ptr<IScanMatcher> makeMatcher(const ScanMatcherConfig& cfg, const CloudType::Ptr& world)
{
    std::unique_ptr<IScanMatcher> m = LocalizationFactory::createMatcher("fast_gicp");
    if (!m) return m;
    m->configure(cfg);
    m->setTarget(world);   // 生产口径: 目标已是 0.3 体素
    return m;
}

struct Err {
    double t = 0.0;   // m
    double r = 0.0;   // rad
};
Err errorTo(const PoseStamped& a, const PoseStamped& b)
{
    Err e;
    e.t = (a.trans - b.trans).norm();
    e.r = std::fabs(Eigen::AngleAxisd(b.rot * a.rot.transpose()).angle());
    return e;
}

constexpr double kDx = 0.05, kDy = 0.03, kDyaw = 2.0 * M_PI / 180.0;

}  // namespace

TEST(GicpTimeLimit, NoLimitKeepsOriginalBehaviour)
{
    ScanMatcherConfig cfg;
    cfg.voxel_resolution = 0.3;
    cfg.max_iterations = 8;
    cfg.max_correspondence_dist = 1.2;
    cfg.transformation_epsilon = 5e-3;
    cfg.num_threads = 2;
    cfg.match_z_filter = false;
    cfg.dynamic_filter = false;
    cfg.min_match_points = 50;
    cfg.time_limit_ms = 0.0;

    const CloudType::Ptr world = makeWorld();
    const PoseStamped truth = mkPose(0.4, -0.2, 0.3);
    std::unique_ptr<IScanMatcher> m = makeMatcher(cfg, world);
    ASSERT_TRUE(m != nullptr);
    MatchResult r;
    ASSERT_TRUE(m->match(scanAt(world, truth), perturb(truth, kDx, kDy, kDyaw), r));
    EXPECT_FALSE(r.time_limited);
    EXPECT_LE(r.iterations, cfg.max_iterations);
    const Err e = errorTo(r.pose, truth);
    EXPECT_LT(e.t, 0.02) << "不设预算时解应收敛到真值 2cm 内 (t=" << e.t << ")";
    EXPECT_LT(e.r, 0.5 * M_PI / 180.0) << "不设预算时解应收敛到真值 0.5° 内 (r=" << e.r << ")";
    std::cout << "[gicp_time_limit] 不设预算: 迭代 " << r.iterations << " 轮, 误差 " << e.t * 1000.0 << " mm / "
              << e.r * 180.0 / M_PI << "°" << std::endl;
}

TEST(GicpTimeLimit, TinyLimitStopsAfterExactlyOneIteration)
{
    ScanMatcherConfig cfg;
    cfg.voxel_resolution = 0.3;
    cfg.max_iterations = 8;
    cfg.max_correspondence_dist = 1.2;
    cfg.transformation_epsilon = 5e-3;
    cfg.num_threads = 2;
    cfg.match_z_filter = false;
    cfg.dynamic_filter = false;
    cfg.min_match_points = 50;
    cfg.time_limit_ms = 1e-6;   // 1 ns: 第二轮开始前一定已超

    const CloudType::Ptr world = makeWorld();
    const PoseStamped truth = mkPose(0.4, -0.2, 0.3);
    const PoseStamped guess = perturb(truth, kDx, kDy, kDyaw);
    std::unique_ptr<IScanMatcher> m = makeMatcher(cfg, world);
    ASSERT_TRUE(m != nullptr);
    MatchResult r;
    ASSERT_TRUE(m->match(scanAt(world, truth), guess, r));
    EXPECT_TRUE(r.time_limited);
    EXPECT_EQ(r.iterations, 1) << "预算极小时只应做第一轮";
    // 至少做了一次步: 一次高斯-牛顿之后, 平移误差应比猜测小一半以上
    const Err e0 = errorTo(guess, truth);
    const Err e1 = errorTo(r.pose, truth);
    EXPECT_LT(e1.t, 0.5 * e0.t) << "一轮之后平移误差 " << e1.t * 1000.0 << " mm (猜测 " << e0.t * 1000.0 << " mm)";
    std::cout << "[gicp_time_limit] 预算 1ns: 迭代 " << r.iterations << " 轮, 猜测误差 " << e0.t * 1000.0 << " mm -> "
              << e1.t * 1000.0 << " mm / " << e1.r * 180.0 / M_PI << "°" << std::endl;
}

TEST(GicpTimeLimit, LargeLimitNeverTriggersAndChangesNothing)
{
    ScanMatcherConfig cfg;
    cfg.voxel_resolution = 0.3;
    cfg.max_iterations = 8;
    cfg.max_correspondence_dist = 1.2;
    cfg.transformation_epsilon = 5e-3;
    cfg.num_threads = 2;
    cfg.match_z_filter = false;
    cfg.dynamic_filter = false;
    cfg.min_match_points = 50;

    const CloudType::Ptr world = makeWorld();
    const PoseStamped truth = mkPose(0.4, -0.2, 0.3);
    const PoseStamped guess = perturb(truth, kDx, kDy, kDyaw);
    MatchResult off, on;
    {
        cfg.time_limit_ms = 0.0;
        std::unique_ptr<IScanMatcher> m = makeMatcher(cfg, world);
        ASSERT_TRUE(m != nullptr);
        ASSERT_TRUE(m->match(scanAt(world, truth), guess, off));
    }
    {
        cfg.time_limit_ms = 1e6;   // 1000 s
        std::unique_ptr<IScanMatcher> m = makeMatcher(cfg, world);
        ASSERT_TRUE(m != nullptr);
        ASSERT_TRUE(m->match(scanAt(world, truth), guess, on));
    }
    EXPECT_FALSE(on.time_limited);
    EXPECT_EQ(off.iterations, on.iterations);
    // 没触发的预算不改变任何结果: 解逐位相同
    EXPECT_TRUE(off.pose.rot == on.pose.rot);
    EXPECT_TRUE(off.pose.trans == on.pose.trans);
    EXPECT_DOUBLE_EQ(off.fitness, on.fitness);
}

TEST(GicpTimeLimit, IterationCapIsNotATimeLimit)
{
    ScanMatcherConfig cfg;
    cfg.voxel_resolution = 0.3;
    cfg.max_iterations = 1;
    cfg.max_correspondence_dist = 1.2;
    cfg.transformation_epsilon = 5e-3;
    cfg.num_threads = 2;
    cfg.match_z_filter = false;
    cfg.dynamic_filter = false;
    cfg.min_match_points = 50;
    cfg.time_limit_ms = 80.0;

    const CloudType::Ptr world = makeWorld();
    const PoseStamped truth = mkPose(0.4, -0.2, 0.3);
    std::unique_ptr<IScanMatcher> m = makeMatcher(cfg, world);
    ASSERT_TRUE(m != nullptr);
    MatchResult r;
    ASSERT_TRUE(m->match(scanAt(world, truth), perturb(truth, kDx, kDy, kDyaw), r));
    EXPECT_EQ(r.iterations, 1);
    EXPECT_FALSE(r.time_limited) << "因迭代上限停下不是因预算停下";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
