// pose_fusion 单元测试：相对位姿观测(LIO)与 direct 直通语义
//
// 覆盖两件事（后续任务会继续追加用例）：
//   1) 相对观测能把滤波器位姿朝 LIO 报告的增量方向拉，且受单帧限幅约束；
//   2) direct 实现必须忽略相对观测（回退语义）。
#include <gtest/gtest.h>
#include <Eigen/Eigen>

#include "algorithms/localization/fusion/direct_pose_fusion.h"
#include "algorithms/localization/fusion/ekf_pose_fusion.h"
#include "algorithms/localization/maintainer/insert_gate.h"

using namespace rosiwit_slam;

namespace {

// 观测协方差下限很小 -> 增益接近上限，便于观察"被拉向观测"的行为。
Eigen::Matrix<double, 6, 6> SmallCov() {
    return Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
}

PoseStamped MakePose(double t, double x = 0.0, double y = 0.0, double yaw = 0.0) {
    PoseStamped p;
    p.time = t;
    p.trans = Eigen::Vector3d(x, y, 0.0);
    p.rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return p;
}

}  // namespace

// LIO 报告前进 0.10 m，滤波内部预测前进 0 m -> 位姿应朝 0.10 m 方向移动，
// 但单帧修正受 max_correction_trans(0.15) 限制，不会一步到位。
TEST(PoseFusion, RelativeMeasurementPullsTowardsLio) {
    EkfPoseFusion f;
    PoseFusionConfig cfg;
    cfg.chi2_gate = 0.0;   // 本用例只验证方向与限幅
    f.configure(cfg);

    const PoseStamped p0 = MakePose(1.0);
    f.reset(p0);

    f.onRelative(p0, SmallCov());                       // 第一帧只做锚定
    f.onRelative(MakePose(1.1, 0.10), SmallCov());      // 第二帧才产生修正

    const PoseStamped out = f.predict(1.1);
    EXPECT_GT(out.trans.x(), 0.01);      // 确实被拉动了
    EXPECT_LE(out.trans.x(), 0.10);      // 但没有一步跨过去
    EXPECT_EQ(f.diag().relative_accepted, 1);
    EXPECT_GT(f.diag().last_rel_residual, 0.0);
}

// direct 模式：相对观测必须被忽略（保持"配准直通"的回退语义）。
// 该用例验证的是 IPoseFusion 默认实现被 direct 继承，因此写完后即刻通过。
TEST(PoseFusion, DirectIgnoresRelative) {
    DirectPoseFusion f;
    const PoseStamped p = MakePose(1.0, 1.0, 2.0);
    f.reset(p);

    PoseStamped lio = p;
    lio.trans.x() = 9.0;
    f.onRelative(lio, SmallCov());

    const PoseStamped out = f.predict(1.0);
    EXPECT_NEAR(out.trans.x(), 1.0, 1e-9);
    EXPECT_NEAR(out.trans.y(), 2.0, 1e-9);
}

// 打滑：位姿只前进 0.02 m，而 odom 报告 0.20 m -> odom 权重应降到下限
TEST(PoseFusion, OdomSlipDownweights) {
    EkfPoseFusion f;
    PoseFusionConfig cfg;              // 默认值 + 显式门限
    cfg.slip_ratio_lo = 0.6;
    cfg.slip_ratio_hi = 1.6;
    cfg.odom_weight_min = 0.0;
    f.configure(cfg);

    PoseStamped p = MakePose(0.0);
    f.reset(p);

    // 匹配把"真实"位移锚定在 0.02 m
    PoseStamped m = MakePose(0.1, 0.02);
    f.onMatch(m, SmallCov(), MatchResult{});

    // odom 先给一帧 0 m，再报 0.20 m
    f.onOdom(MakePose(0.1));
    f.onOdom(MakePose(0.2, 0.20));

    EXPECT_LT(f.diag().odom_weight, 0.2);
}

// 模式切出 LOCALIZATION / 重定位时，相对观测的锚点必须清空，
// 否则重新进入后第一次相对观测会把"跨模式位移"当成真实增量。
TEST(PoseFusion, RelativeStateClearedOnReset) {
    EkfPoseFusion f;
    PoseFusionConfig cfg;
    f.configure(cfg);

    const PoseStamped p = MakePose(1.0);
    f.reset(p);
    f.onRelative(p, SmallCov());   // 锚定
    f.reset(p);                    // 模拟模式切换/重定位

    f.onRelative(MakePose(1.1, 5.0), SmallCov());   // 复位后的首帧只记录锚点
    EXPECT_EQ(f.diag().relative_accepted, 0);
}

// LIO 复位或长时间中断后位姿会跳变: 必须重新锚定, 不能当成真实位移。
TEST(PoseFusion, RelativeJumpReanchors) {
    EkfPoseFusion f;
    PoseFusionConfig cfg;
    f.configure(cfg);

    const PoseStamped p = MakePose(1.0);
    f.reset(p);
    f.onRelative(p, SmallCov());                    // 锚定

    f.onRelative(MakePose(1.1, 5.0), SmallCov());   // 5 m 跳变 -> 重新锚定
    EXPECT_NEAR(f.predict(1.1).trans.x(), 0.0, 1e-9);
    EXPECT_EQ(f.diag().relative_accepted, 0);
    EXPECT_EQ(f.diag().relative_reanchored, 1);

    // 重新锚定之后的正常增量应继续生效
    f.onRelative(MakePose(1.2, 5.05), SmallCov());
    EXPECT_EQ(f.diag().relative_accepted, 1);
}

// 地图插入门控: 非 TRACKING / 本帧匹配未被接受 / LIO-S2M 分歧过大时都不得写入叠加层。
TEST(MapGate, RejectsWhenNotTrackingOrDiverged) {
    MaintainerConfig cfg;
    cfg.max_insert_divergence = 0.2;

    EXPECT_FALSE(allowMapInsert(false, true, 0.01, cfg));   // 非 TRACKING
    EXPECT_FALSE(allowMapInsert(true, false, 0.01, cfg));   // 匹配未被接受
    EXPECT_FALSE(allowMapInsert(true, true, 0.50, cfg));    // LIO/S2M 分歧过大
    EXPECT_TRUE (allowMapInsert(true, true, 0.05, cfg));    // 全部满足

    // 0 = 关闭分歧门控 (默认行为兼容)
    cfg.max_insert_divergence = 0.0;
    EXPECT_TRUE(allowMapInsert(true, true, 100.0, cfg));
}

int main(int argc, char ** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
