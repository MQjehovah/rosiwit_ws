// odom+IMU EKF 运动先验单元测试
// 场景对照 2026-10-17 推行丢失事故: 旧 ImuMotionPrior 在推行时预测跟不上
// (常速外推速度≈0), EKF 要做到: 直推/侧推速度可测、打滑样本被卡方门控拒收、
// 匹配锚定不破坏速度状态、轮子抱死时优雅降级(与旧行为一致)。
#include <gtest/gtest.h>
#include <cmath>
#include "algorithms/localization/motion_prior/ekf_motion_prior.h"

using namespace rosiwit_slam;

namespace {
MotionPriorConfig ekfCfg() {
    MotionPriorConfig cfg;
    cfg.use_odom = true;
    cfg.max_odom_speed = 3.0;
    cfg.slip_yaw_thresh = 0.3;
    cfg.ekf_q_v = 1.0;
    cfg.ekf_r_odom = 0.15;
    cfg.ekf_r_match = 0.8;
    cfg.ekf_chi2 = 5.99;
    cfg.use_gravity = false;   // 单测不喂重力
    return cfg;
}
IMUSample imuAt(double t) {
    IMUSample s;
    s.time = t;
    s.gyro = V3D::Zero();
    s.acc = V3D(0, 0, 9.81);
    return s;
}
PoseStamped odomAt(double t, double x, double y) {
    PoseStamped p;
    p.time = t;
    p.trans = V3D(x, y, 0);
    p.rot = Eigen::Matrix3d::Identity();
    return p;
}
}   // namespace

// 直线推行: odom 以 1m/s 前进, EKF 速度应收敛到 ~1m/s, 预测跟上推行
TEST(EkfPrior, StraightPushVelocityConverges) {
    EkfMotionPrior prior;
    prior.configure(ekfCfg());
    PoseStamped m0;
    m0.time = 0.0;
    prior.reset(m0);

    const double dt = 0.005;               // 200Hz IMU
    double t = 0.0, x = 0.0;
    for (int i = 0; i < 400; ++i) {        // 2 秒
        t += dt;
        prior.onImu(imuAt(t));
        if (i % 20 == 19) {                // 10Hz odom
            x += 0.1;                      // 1 m/s
            prior.onOdom(odomAt(t, x, 0.0));
        }
    }
    EXPECT_NEAR(prior.velocity().x(), 1.0, 0.2) << "前向速度应收敛到 1m/s";
    EXPECT_LT(std::fabs(prior.velocity().y()), 0.1);
    // 预测位置应跟着走 (不再"留在原地")
    PoseStamped p = prior.predict(t);
    EXPECT_GT(p.trans.x(), 1.5) << "2 秒 1m/s 推行, 预测应前进 >1.5m";
}

// 侧向推行: 体系二维速度的优势 —— 旧单前向速度模型表达不了横向
TEST(EkfPrior, LateralPushMeasured) {
    EkfMotionPrior prior;
    prior.configure(ekfCfg());
    PoseStamped m0;
    m0.time = 0.0;
    prior.reset(m0);

    double t = 0.0, y = 0.0;
    for (int i = 0; i < 400; ++i) {
        t += 0.005;
        prior.onImu(imuAt(t));
        if (i % 20 == 19) {
            y += 0.1;                      // 沿 +y 侧推 1 m/s
            prior.onOdom(odomAt(t, 0.0, y));
        }
    }
    EXPECT_NEAR(prior.velocity().y(), 1.0, 0.2) << "侧推速度必须可测";
}

// 打滑样本: 10m/s 的 odom 跳变 —— 物理门直接拒收, 速度不受污染
TEST(EkfPrior, SlipSampleRejectedByPhysicsGate) {
    EkfMotionPrior prior;
    prior.configure(ekfCfg());
    PoseStamped m0;
    m0.time = 0.0;
    prior.reset(m0);

    prior.onImu(imuAt(0.1));
    prior.onOdom(odomAt(0.1, 0.0, 0.0));   // 锚定
    prior.onImu(imuAt(0.2));
    prior.onOdom(odomAt(0.2, 1.0, 0.0));   // 10 m/s: 物理上限 3.0, 拒收
    EXPECT_EQ(prior.odomRejected(), 1);
    EXPECT_LT(std::fabs(prior.velocity().x()), 0.5) << "打滑样本不得污染速度";
}

// 卡方门控: 速度突变 (0 -> 1.5m/s 一步到位, 无渐变) 且协方差已收敛时,
// 新息检验应拒绝 (真车加速是渐变的)
TEST(EkfPrior, ChiSquareGateRejectsAbruptJump) {
    EkfMotionPrior prior;
    MotionPriorConfig cfg = ekfCfg();
    cfg.max_odom_speed = 10.0;             // 关物理门, 只测卡方
    prior.configure(cfg);
    PoseStamped m0;
    m0.time = 0.0;
    prior.reset(m0);

    // 先喂 3 秒静止 odom 让协方差收敛 (P 缩小)
    double t = 0.0;
    for (int i = 0; i < 600; ++i) {
        t += 0.005;
        prior.onImu(imuAt(t));
        if (i % 20 == 19) prior.onOdom(odomAt(t, 0.0, 0.0));
    }
    const int rej0 = prior.odomRejected();
    // 突然声称 1.5m/s (新息 1.5, P 已小 -> 超卡方)
    prior.onImu(imuAt(t + 0.005));
    prior.onOdom(odomAt(t + 0.1, 0.15, 0.0));
    EXPECT_GT(prior.odomRejected(), rej0) << "收敛后突跳应被卡方门控拒收";
}

// 匹配锚定: onMatch 重置绝对位姿, 但速度状态保持 (不因锚定丢速度)
TEST(EkfPrior, MatchAnchorKeepsVelocity) {
    EkfMotionPrior prior;
    prior.configure(ekfCfg());
    PoseStamped m0;
    m0.time = 0.0;
    prior.reset(m0);

    double t = 0.0, x = 0.0;
    for (int i = 0; i < 200; ++i) {        // 1 秒建立 1m/s 速度
        t += 0.005;
        prior.onImu(imuAt(t));
        if (i % 20 == 19) {
            x += 0.1;
            prior.onOdom(odomAt(t, x, 0.0));
        }
    }
    const double v_before = prior.velocity().x();
    ASSERT_GT(v_before, 0.5);
    PoseStamped match;
    match.time = t;
    match.trans = V3D(0.95, 0.0, 0.0);     // 激光给出绝对位姿
    match.rot = Eigen::Matrix3d::Identity();
    prior.onMatch(match);
    PoseStamped p = prior.predict(t + 0.1);
    EXPECT_GT(p.trans.x(), 1.0) << "锚定后 0.1s, 预测应继续以 ~1m/s 前进";
}

// 轮子抱死被推行: odom=0, v 收敛到 0 (优雅降级, 不发散; 真位置靠匹配层每帧修正)
TEST(EkfPrior, BrakedWheelsDegradeGracefully) {
    EkfMotionPrior prior;
    prior.configure(ekfCfg());
    PoseStamped m0;
    m0.time = 0.0;
    prior.reset(m0);
    double t = 0.0;
    for (int i = 0; i < 400; ++i) {
        t += 0.005;
        prior.onImu(imuAt(t));
        if (i % 20 == 19) prior.onOdom(odomAt(t, 0.0, 0.0));   // 抱死: 恒 0
    }
    EXPECT_LT(prior.velocity().norm(), 0.1) << "odom 恒 0 -> v 应降级到 0";
    PoseStamped p = prior.predict(t);
    EXPECT_LT(p.trans.norm(), 0.1) << "预测冻结在原地 (与旧行为一致, 不发散)";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
