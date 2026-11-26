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

// ---------------------------------------------------------------------------
// 2026-11-26 根因回归 (静止自转 / 发布航向恒差 11°): 倾角新息不是陀螺零偏。
//
// 平面锁定设计里 (planarizePose), 匹配与输出的 roll/pitch 恒等于标称安装角, 不携带
// 任何陀螺信息; 先验内部姿态则由重力对平。地面坡度 / 标称角误差让两者差一个恒定
// 倾角 δ。旧实现把整只新息旋转向量喂进零偏积分器 (增益 1.0): 零偏在水平轴上一路
// 跑到重力校正顶住为止 (|ê|≈10δ, 时间常数 1/(0.5·(1-e⁻¹)/10)=31.6s —— 与实机
// bz 0 -> 0.103 rad/s 的一阶响应逐点吻合), 其 z 分量再经融合层泄漏成 yaw 幻影。
TEST(EkfPrior, TiltInnovationIsNotLearnedAsGyroBias) {
    MotionPriorConfig cfg = ekfCfg();
    cfg.use_gravity = true;          // 本用例必须开: "重力对平 vs 标称安装角"的差就是被测对象
    cfg.gravity_gain = 0.05;         // 实车值
    cfg.gravity = 9.81;
    cfg.imu_calib_samples = 50;
    cfg.gyro_bias_gain = 1.0;        // 实车值
    // 实车安装: IMU 随雷达俯装 39°
    const double tilt = 39.37 * M_PI / 180.0;
    const M3D R_mount = Eigen::AngleAxisd(tilt, V3D::UnitX()).toRotationMatrix();  // map <- sensor
    cfg.up_body = (R_mount.transpose() * V3D::UnitZ()).normalized();
    cfg.up_body_valid = true;

    EkfMotionPrior prior;
    prior.configure(cfg);
    PoseStamped p0;
    p0.time = 0.0;
    p0.rot = R_mount;                // 重力对平的真实姿态 (base yaw = 0)
    prior.reset(p0);

    // 平面锁定的输出姿态: roll/pitch = 标称安装角, 与真实姿态差 δ (绕世界 x 轴 = 1° 坡度)
    const double delta = 1.0 * M_PI / 180.0;
    const M3D R_nominal = Eigen::AngleAxisd(delta, V3D::UnitX()).toRotationMatrix() * R_mount;

    // 真实零偏 (静止时陀螺读数): 标定窗口直接给出, 之后不应再变
    const V3D bias_true(0.004, -0.007, 0.012);
    const double dt_imu = 0.005;
    double t = 0.0;
    for (int k = 0; k < 2000; ++k) {              // 200 s @10Hz
        for (int i = 0; i < 20; ++i) {            // 200Hz
            t += dt_imu;
            IMUSample s;
            s.time = t;
            s.gyro = bias_true;                   // 静止: 陀螺 = 零偏
            s.acc = cfg.up_body * 9.81;           // 静止: 比力 = g·up (传感器系)
            prior.onImu(s);
        }
        // 跟踪线程每帧 (与 localization_pipeline 同序): 先验预测 -> 输出姿态被平面
        // 锁定成 Rz(base yaw)·标称安装角 -> 先验锚定到这个输出。
        PoseStamped fused = prior.predict(t);
        const M3D R_base = fused.rot * R_nominal.transpose();
        const double yaw = std::atan2(R_base(1, 0), R_base(0, 0));
        fused.rot = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix() * R_nominal;
        prior.onMatch(fused);
    }
    // 判据是零偏本身, 不是航向: 跑飞的分量在水平轴上 (被重力校正顶住), 在这个几何下
    // 几乎不表现为航向漂移 —— A/B 实测旧实现 190s 航向只差 6e-7 rad, 零偏却错了 0.18。
    // (输出姿态 roll/pitch 被锁成标称安装角后, z 分量经融合层才变成 yaw 幻影, 见
    //  test_pose_fusion。)
    const V3D err = prior.gyroBias() - bias_true;
    EXPECT_LT(err.norm(), 1e-6)
        << "倾角差被当成零偏学习: b=(" << prior.gyroBias().transpose() << ")";
}

// 对照: 航向新息仍然要学 (只去掉倾角分量, 不去掉 yaw 分量的学习)。
// 真实零偏在标定后发生漂移 -> 先验以恒定角速度偏转 -> 每帧出现恒定的 yaw 新息
// -> 零偏估计应把它收敛掉。
TEST(EkfPrior, YawInnovationStillDrivesBiasLearning) {
    MotionPriorConfig cfg = ekfCfg();
    cfg.use_gravity = false;
    cfg.imu_calib_samples = 50;
    cfg.gyro_bias_gain = 1.0;
    const double tilt = 39.37 * M_PI / 180.0;
    const M3D R_mount = Eigen::AngleAxisd(tilt, V3D::UnitX()).toRotationMatrix();
    cfg.up_body = (R_mount.transpose() * V3D::UnitZ()).normalized();
    cfg.up_body_valid = true;

    EkfMotionPrior prior;
    prior.configure(cfg);
    PoseStamped p0;
    p0.time = 0.0;
    p0.rot = R_mount;
    prior.reset(p0);

    // 标定: 零偏 = 0 (实机开机时 bz=0.000); 标定完成后真实零偏漂到 bias_drift
    const V3D bias_drift(0.0, 0.0, 0.02);
    const double dt_imu = 0.005;
    double t = 0.0;
    double yaw_true = 0.0;            // 机器人真实静止: 航向不变
    // 100 s: yaw 分量学习的时间常数 ~20s, 5τ 后残差 <1% 初值 (断言阈值 1e-3 的 1/7)
    for (int k = 0; k < 1000; ++k) {
        for (int i = 0; i < 20; ++i) {
            t += dt_imu;
            IMUSample s;
            s.time = t;
            s.gyro = (k >= 2) ? bias_drift : V3D::Zero();   // 第 2 帧起漂移
            s.acc = cfg.up_body * 9.81;
            prior.onImu(s);
        }
        PoseStamped fused = prior.predict(t);
        // 输出航向 = 真值 (匹配解不动), 其余姿态照抄先验
        fused.rot = Eigen::AngleAxisd(yaw_true, V3D::UnitZ()).toRotationMatrix() * R_mount;
        prior.onMatch(fused);
    }
    // 零偏估计应跟上真实零偏在重力轴上的分量 (未被观测的水平轴分量不在断言内)
    const V3D b = prior.gyroBias();
    const double up_true = bias_drift.dot(cfg.up_body);
    EXPECT_NEAR(b.dot(cfg.up_body), up_true, 1e-3)
        << "yaw 新息未被学习: b·up=" << b.dot(cfg.up_body) << " 期望≈" << up_true;
}
