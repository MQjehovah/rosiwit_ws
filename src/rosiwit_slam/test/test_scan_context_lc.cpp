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

    for (int i = 0; i < 10; ++i) {
        KeyFrame kf;
        kf.pose.trans = V3D(1.0 * i, 0, 0);
        lc.addKeyFrame(kf);
    }

    KeyFrame kf_close;
    kf_close.pose.trans = V3D(0.5, 0, 0);
    lc.addKeyFrame(kf_close);

    PoseStamped rel;
    double cov = 0.0;
    EXPECT_TRUE(lc.detect(rel, cov));
    EXPECT_GE(cov, 0.0);
    EXPECT_LE(cov, 1.0);
}
