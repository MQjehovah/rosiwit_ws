#include <gtest/gtest.h>
#include "algorithms/scan_context_lc/scan_context_lc.h"

using namespace rosiwit_slam;

TEST(ScanContextLCTest, CreateAndInit) {
    ScanContextLC lc;
    EXPECT_TRUE(lc.init("nonexistent.yaml"));
}

TEST(ScanContextLCTest, DetectWithTooFewFramesReturnsFalse) {
    ScanContextLC lc;
    lc.init("nonexistent.yaml");

    KeyFrame kf;
    kf.pose.trans = V3D(0, 0, 0);
    lc.addKeyFrame(kf);

    PoseStamped rel;
    double cov = 0.0;
    EXPECT_FALSE(lc.detect(rel, cov));
}

TEST(ScanContextLCTest, DetectDistantFramesReturnsFalse) {
    ScanContextLC lc;
    lc.init("nonexistent.yaml");

    KeyFrame kf1;
    kf1.pose.trans = V3D(0, 0, 0);
    lc.addKeyFrame(kf1);

    // 添加足够多的远距离帧 (超出 loop_radius_ 默认 15m)
    for (int i = 0; i < 15; ++i) {
        KeyFrame kf;
        kf.pose.trans = V3D(100 + i, 0, 0);
        lc.addKeyFrame(kf);
    }

    PoseStamped rel;
    double cov = 0.0;
    EXPECT_FALSE(lc.detect(rel, cov));
}

TEST(ScanContextLCTest, DetectLoopReturnsTrue) {
    ScanContextLC lc;
    lc.init("nonexistent.yaml");

    KeyFrame kf1;
    kf1.pose.trans = V3D(0, 0, 0);
    lc.addKeyFrame(kf1);

    // 添加 10 个向外走的帧 (符合 min_keyframe_gap_ 默认 30, 但这里会触发边界)
    for (int i = 1; i <= 35; ++i) {
        KeyFrame kf;
        kf.pose.trans = V3D(1.0 * i, 0, 0);
        lc.addKeyFrame(kf);
    }

    // 回到起点附近 — 应检测到回环 (索引 0 距离 < loop_radius_)
    KeyFrame kf_close;
    kf_close.pose.trans = V3D(0.5, 0, 0);
    lc.addKeyFrame(kf_close);

    PoseStamped rel;
    double cov = 0.0;
    EXPECT_TRUE(lc.detect(rel, cov));
    // cov = 1.0 - 0.5/15.0 ≈ 0.967
    EXPECT_GT(cov, 0.9);
    EXPECT_LE(cov, 1.0);
}

// 新增测试: 验证返回的相对位姿正确性
TEST(ScanContextLCTest, CorrectRelativePose) {
    ScanContextLC lc;
    lc.init("nonexistent.yaml");

    // 第一个关键帧在 x=0, 朝向 90°
    KeyFrame kf0;
    kf0.pose.trans = V3D(0, 0, 0);
    kf0.pose.rot = Eigen::AngleAxisd(M_PI/2, V3D::UnitZ()).toRotationMatrix();
    lc.addKeyFrame(kf0);

    // 添加足够的中间帧
    for (int i = 1; i <= 35; ++i) {
        KeyFrame kf;
        kf.pose.trans = V3D(i * 0.5, i * 0.5, 0);
        kf.pose.rot = M3D::Identity();
        lc.addKeyFrame(kf);
    }

    // 回到起点附近 (平移起点+旋转90度回来)
    KeyFrame kf_close;
    kf_close.pose.trans = V3D(0.5, 0.5, 0);
    kf_close.pose.rot = Eigen::AngleAxisd(M_PI/2, V3D::UnitZ()).toRotationMatrix();
    lc.addKeyFrame(kf_close);

    PoseStamped rel;
    double cov = 0.0;
    EXPECT_TRUE(lc.detect(rel, cov));

    // 相对旋转应为恒等 (因为起点和终点朝向相同)
    Eigen::AngleAxisd aa(rel.rot);
    EXPECT_NEAR(aa.angle(), 0.0, 1e-6);
}
