// test/test_visual_constraint_source.cpp
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "algorithms/visual/visual_constraint_source.h"

using namespace rosiwit_slam;

namespace {
PoseStamped mk(double t, double x, double y, double yaw) {
    PoseStamped p;
    p.time = t;
    p.trans = V3D(x, y, 0.0);
    p.rot = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix();
    return p;
}
} // namespace

TEST(VisualConstraintSource, BetweenFactorsFromVO) {
    VisualConstraintSource v;
    VisualConstraintConfig cfg;
    cfg.assoc_dt = 0.05;
    v.configure(cfg);

    v.addVisualOdometry(0.0, mk(0.0, 0.0, 0.0, 0.0));
    v.addVisualOdometry(1.0, mk(1.0, 1.0, 0.0, 0.0));
    v.addVisualOdometry(2.0, mk(2.0, 2.0, 0.0, 0.0));

    v.addKeyframe("k0", 0.0);
    v.addKeyframe("k1", 1.0);
    v.addKeyframe("k2", 2.0);

    std::vector<Constraint> cs = v.popBetweenConstraints();
    ASSERT_EQ(cs.size(), 2u);
    EXPECT_EQ(cs[0].from_kf, "k0");
    EXPECT_EQ(cs[0].to_kf, "k1");
    EXPECT_NEAR(cs[0].relative_pose.trans.x(), 1.0, 1e-6);
    EXPECT_NEAR(cs[1].relative_pose.trans.x(), 1.0, 1e-6);
    EXPECT_TRUE(v.popBetweenConstraints().empty());
}

TEST(VisualConstraintSource, NoVOWithinToleranceSkips) {
    VisualConstraintSource v;
    VisualConstraintConfig cfg;
    cfg.assoc_dt = 0.01;   // 严格
    v.configure(cfg);
    v.addVisualOdometry(0.0, mk(0.0, 0.0, 0.0, 0.0));
    v.addKeyframe("k0", 0.0);
    v.addKeyframe("k1", 5.0);   // 无对应 VO
    EXPECT_TRUE(v.popBetweenConstraints().empty());
}

TEST(VisualConstraintSource, LoopConstraintPassthrough) {
    Constraint c = VisualConstraintSource::makeLoopConstraint("a", "b", mk(0, 1, 2, 0.3), 0.4);
    EXPECT_EQ(c.from_kf, "a");
    EXPECT_EQ(c.to_kf, "b");
    EXPECT_DOUBLE_EQ(c.cov, 0.4);
    EXPECT_NEAR(c.relative_pose.trans.x(), 1.0, 1e-9);
}
