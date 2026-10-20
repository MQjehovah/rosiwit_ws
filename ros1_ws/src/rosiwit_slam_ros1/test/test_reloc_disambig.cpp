// 全局重定位消歧回归测试
//
// 背景 (2026-10-17 推行丢失事故): FPFH 重定位器未覆写 localizeTopK, 走基类默认
// 实现 = 单候选。地库重复结构下单候选 0.999 锁进相似车位, 界面仍显示定位成功。
// 本测试用"两个完全相同的房间隔 25m"的歧义地图复现该场景, 验证:
//   1. localizeTopK 能产出 >=2 个候选(歧义可见是消歧的前提)
//   2. 候选分别落在两个房间附近
// 另含: 位姿-里程计一致性监测(误锁检测) + IMU 先验 odom 平移滑移门控。
#include <gtest/gtest.h>
#include <cmath>
#include "algorithms/localization/global/fpfh_ransac_global_localizer.h"
#include "algorithms/localization/motion_prior/imu_motion_prior.h"
#include "lost_monitor.h"

using namespace rosiwit_slam;

namespace {
// 一面墙的点 (y 固定, x/z 铺点)
CloudType::Ptr makeWall(double step, double x0, double x1, double y, double z0, double z1) {
    CloudType::Ptr c(new CloudType());
    for (double x = x0; x <= x1 + 1e-9; x += step)
        for (double z = z0; z <= z1 + 1e-9; z += step) {
            PointType p;
            p.x = x; p.y = y; p.z = z;
            c->push_back(p);
        }
    return c;
}
// 一个"房间": 4 面墙 (10m x 10m, z 0.2~1.8), 加两根柱子增加 3D 特征
CloudType::Ptr makeRoom(double cx, double cy) {
    CloudType::Ptr c(new CloudType());
    const double step = 0.3;
    CloudType::Ptr w;
    w = makeWall(step, cx - 5, cx + 5, cy - 5, 0.2, 1.8); *c += *w;
    w = makeWall(step, cx - 5, cx + 5, cy + 5, 0.2, 1.8); *c += *w;
    // 左右墙: 沿 y 铺点
    for (double y = cy - 5; y <= cy + 5 + 1e-9; y += step)
        for (double z = 0.2; z <= 1.8 + 1e-9; z += step) {
            PointType p;
            p.x = cx - 5; p.y = y; p.z = z; c->push_back(p);
            p.x = cx + 5; p.y = y; p.z = z; c->push_back(p);
        }
    // 两根柱子 (0.3 x 0.3), 位置对称 -> 两个房间完全相同
    for (double z = 0.2; z <= 1.8 + 1e-9; z += step)
        for (double d = -0.15; d <= 0.15 + 1e-9; d += 0.15) {
            PointType p;
            p.x = cx - 2.5 + d; p.y = cy - 2.5 + d; p.z = z; c->push_back(p);
            p.x = cx + 2.5 + d; p.y = cy + 2.5 + d; p.z = z; c->push_back(p);
        }
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}
PoseStamped poseAt(double x, double y) {
    PoseStamped p;
    p.rot = Eigen::Matrix3d::Identity();
    p.trans = V3D(x, y, 0.0);
    return p;
}
}   // namespace

// ---- 1. 歧义地图多候选 (今晚事故的回归测试) ----
TEST(FpfhDisambig, TwoIdenticalRoomsYieldTwoCandidates) {
    CloudType::Ptr map(new CloudType());
    CloudType::Ptr r1 = makeRoom(0.0, 0.0);
    CloudType::Ptr r2 = makeRoom(25.0, 0.0);
    *map += *r1;
    *map += *r2;
    map->width = static_cast<uint32_t>(map->size());
    map->height = 1;

    // 扫描 = A 房间 (加 0.2m 平移扰动, 模拟真实初始误差)
    CloudType::Ptr scan = makeRoom(0.2, 0.1);

    FpfhRansacGlobalLocalizer loc;
    GlobalLocalizeConfig cfg;
    cfg.fpfh_normal_radius = 1.0;
    cfg.fpfh_feature_radius = 3.0;
    cfg.fpfh_max_iter = 8000;
    cfg.fpfh_inlier_fraction = 0.25;
    cfg.fpfh_similarity = 0.9;
    cfg.min_score = 0.4;
    cfg.voxel_resolution = 0.3;
    loc.configure(cfg);
    ASSERT_TRUE(loc.setMap("", map));

    std::vector<GlobalCandidate> out;
    const int nk = loc.localizeTopK(scan, 3, out);
    ASSERT_GE(nk, 1) << "至少应找到 1 个候选";
    std::cout << "[test] candidates=" << nk << std::endl;
    for (const auto& c : out)
        std::cout << "[test]   score=" << c.score << " trans=(" << c.pose.trans.x()
                  << ", " << c.pose.trans.y() << ", " << c.pose.trans.z() << ")" << std::endl;

    // 两个一模一样的房间: 候选应 >=2 (歧义必须可见), 且分别靠近两个房间
    ASSERT_GE(nk, 2) << "完全相同的两个房间, 单候选 = 歧义不可见 (2026-10-17 事故根因)";
    bool near_a = false, near_b = false;
    for (const auto& c : out) {
        const double da = std::hypot(c.pose.trans.x() - 0.0, c.pose.trans.y() - 0.0);
        const double db = std::hypot(c.pose.trans.x() - 25.0, c.pose.trans.y() - 0.0);
        if (da < 2.0) near_a = true;
        if (db < 2.0) near_b = true;
    }
    EXPECT_TRUE(near_a) << "应有候选落在 A 房间";
    EXPECT_TRUE(near_b) << "应有候选落在 B 房间 (歧义的另一解)";
}

// ---- 2. 位姿-里程计一致性监测: 位姿冻住但轮子在走 -> 报可疑 ----
TEST(OdomConsistency, FrozenPoseMovingOdomFlagsWrongness) {
    rosiwit_slam_ros1::OdomConsistencyMonitor mon;
    double t = 0;
    double odom = 0, sx = 0, sy = 0;
    mon.onPose(t, sx, sy, odom, 0);
    for (int w = 0; w < 3; ++w) {
        t += 10.0; odom += 2.0;   // 轮子每窗口走 2m
        sx += 0.05;               // 激光位姿几乎没动 (锁死后被"钉"住)
        mon.onPose(t, sx, sy, odom, 0);
    }
    EXPECT_TRUE(mon.wrongness()) << "位姿冻结 + 轮速 0.2m/s 持续 30s, 必须报可疑";
    EXPECT_GT(mon.deviation(), 1.5);
}

TEST(OdomConsistency, ConsistentMotionNoFlag) {
    rosiwit_slam_ros1::OdomConsistencyMonitor mon;
    double t = 0, odom = 0, x = 0;
    mon.onPose(t, x, 0, odom, 0);
    for (int w = 0; w < 4; ++w) {
        t += 10.0; odom += 2.0; x += 1.95;   // 位移基本一致
        mon.onPose(t, x, 0, odom, 0);
    }
    EXPECT_FALSE(mon.wrongness());
}

TEST(OdomConsistency, BrakedWheelsNoFalseFlag) {
    rosiwit_slam_ros1::OdomConsistencyMonitor mon;
    double t = 0, x = 0;
    mon.onPose(t, x, 0, 0, 0);
    for (int w = 0; w < 4; ++w) {
        t += 10.0; x += 2.0;   // 抱死推行: odom 不动, 位姿在走 -> 无法判定, 不报
        mon.onPose(t, x, 0, 0, 0);
    }
    EXPECT_FALSE(mon.wrongness()) << "轮子没转时无法判定, 不允许误报";
}

// ---- 3. IMU 先验 odom 平移滑移门控 ----
TEST(ImuOdomGate, SlipSampleDropped) {
    ImuMotionPrior prior;
    MotionPriorConfig cfg;
    cfg.use_odom = true;
    cfg.max_odom_speed = 3.0;
    prior.configure(cfg);

    PoseStamped m0 = poseAt(0, 0);
    m0.time = 0.0;
    prior.reset(m0);

    PoseStamped o0 = poseAt(0, 0);
    o0.time = 0.0;
    prior.onOdom(o0);                     // 锚定 odom 起点
    prior.onMatch(m0);                    // 锚定 map<-odom 偏移

    PoseStamped o1 = poseAt(2.0, 0);      // 2 m/s: 合法
    o1.time = 1.0;
    prior.onOdom(o1);
    PoseStamped p1 = prior.predict(1.0);
    EXPECT_NEAR(p1.trans.x(), 2.0, 0.05) << "合法速度的 odom 平移应被采用";

    PoseStamped o2 = poseAt(12.0, 0);     // 10 m/s: 物理不可能, 整包丢弃
    o2.time = 2.0;
    prior.onOdom(o2);
    PoseStamped p2 = prior.predict(2.0);
    EXPECT_LT(p2.trans.x(), 3.0) << "滑移样本被丢弃后不得外推到 12m";
}

// ---- 4. 监测器只做单帧健康分类；连续判丢由 LocalizationPipeline 的 lost_patience 负责 ----
#include "algorithms/localization/monitor/windowed_monitor.h"

TEST(MonitorHealth, MediocreIsDegradedAndCredibleIsHealthy) {
    WindowedMonitor mon;
    MonitorConfig cfg;
    cfg.credible_fitness = 0.75;
    cfg.lost_fitness = 0.5;
    mon.configure(cfg);
    mon.reset();

    MonitorInput in;
    in.match.converged = true;
    in.pose.rot = Eigen::Matrix3d::Identity();
    in.match.fitness = 0.6;   // 僵尸带 [0.5, 0.75) -> 单帧不健康
    EXPECT_EQ(mon.update(in), MonitorLevel::DEGRADED);

    in.match.fitness = 0.95;  // 健康帧
    EXPECT_EQ(mon.update(in), MonitorLevel::CREDIBLE);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
