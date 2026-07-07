#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <cmath>
#include "algorithms/ceres_backend/ceres_backend.h"

using namespace rosiwit_slam;

TEST(CeresBackendTest, EmptyInit) {
    CeresBackend backend;
    EXPECT_TRUE(backend.init("nonexistent.yaml"));

    std::vector<PoseStamped> poses;
    EXPECT_FALSE(backend.getUpdatedPoses(poses));

    EXPECT_FALSE(backend.optimize());
}

TEST(CeresBackendTest, SingleKeyframe) {
    CeresBackend backend;
    backend.init("nonexistent.yaml");

    KeyFrame kf;
    kf.id = "0";
    kf.pose.trans = Eigen::Vector3d(0, 0, 0);
    kf.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf);

    EXPECT_FALSE(backend.optimize());

    std::vector<PoseStamped> poses;
    EXPECT_TRUE(backend.getUpdatedPoses(poses));
    ASSERT_EQ(poses.size(), 1);
}

TEST(CeresBackendTest, ThreeKeyframesOdometryOnly) {
    CeresBackend backend;
    backend.init("nonexistent.yaml");

    KeyFrame kf0;
    kf0.id = "0";
    kf0.pose.trans = Eigen::Vector3d(0, 0, 0);
    kf0.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf0);

    KeyFrame kf1;
    kf1.id = "1";
    kf1.pose.trans = Eigen::Vector3d(1, 0, 0);
    kf1.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf1);

    KeyFrame kf2;
    kf2.id = "2";
    kf2.pose.trans = Eigen::Vector3d(2, 0, 0);
    kf2.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf2);

    ASSERT_TRUE(backend.optimize());

    std::vector<PoseStamped> poses;
    ASSERT_TRUE(backend.getUpdatedPoses(poses));
    ASSERT_EQ(poses.size(), 3);

    EXPECT_NEAR(poses[0].trans.x(), 0.0, 1e-6);
    EXPECT_NEAR(poses[0].trans.y(), 0.0, 1e-6);
    EXPECT_NEAR(poses[0].trans.z(), 0.0, 1e-6);

    EXPECT_NEAR(poses[1].trans.x(), 1.0, 1e-6);
    EXPECT_NEAR(poses[1].trans.y(), 0.0, 1e-6);
    EXPECT_NEAR(poses[1].trans.z(), 0.0, 1e-6);

    EXPECT_NEAR(poses[2].trans.x(), 2.0, 1e-6);
    EXPECT_NEAR(poses[2].trans.y(), 0.0, 1e-6);
    EXPECT_NEAR(poses[2].trans.z(), 0.0, 1e-6);
}

TEST(CeresBackendTest, LoopClosureConstraint) {
    CeresBackend backend;
    backend.init("nonexistent.yaml");

    KeyFrame kf0;
    kf0.id = "0";
    kf0.pose.trans = Eigen::Vector3d(0, 0, 0);
    kf0.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf0);

    KeyFrame kf1;
    kf1.id = "1";
    kf1.pose.trans = Eigen::Vector3d(1.0, 0.1, 0);
    kf1.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf1);

    KeyFrame kf2;
    kf2.id = "2";
    kf2.pose.trans = Eigen::Vector3d(1.9, 0.2, 0);
    kf2.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf2);

    Constraint loop;
    loop.from_kf = "0";
    loop.to_kf = "2";
    loop.relative_pose.trans = Eigen::Vector3d(2, 0, 0);
    loop.relative_pose.rot = Eigen::Matrix3d::Identity();
    loop.cov = 0.5;
    backend.addConstraints({loop});

    ASSERT_TRUE(backend.optimize());

    std::vector<PoseStamped> poses;
    ASSERT_TRUE(backend.getUpdatedPoses(poses));
    ASSERT_EQ(poses.size(), 3);

    EXPECT_NEAR(poses[2].trans.x(), 2.0, 0.1);
    EXPECT_NEAR(poses[2].trans.y(), 0.0, 0.05);
}

TEST(CeresBackendTest, AddedPoseConstraintBeforeKeyframe) {
    CeresBackend backend;
    backend.init("nonexistent.yaml");

    Constraint c;
    c.from_kf = "0";
    c.to_kf = "1";
    c.relative_pose.trans = Eigen::Vector3d(1, 0, 0);
    c.relative_pose.rot = Eigen::Matrix3d::Identity();
    backend.addConstraints({c});

    KeyFrame kf0;
    kf0.id = "0";
    kf0.pose.trans = Eigen::Vector3d(0, 0, 0);
    kf0.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf0);

    KeyFrame kf1;
    kf1.id = "1";
    kf1.pose.trans = Eigen::Vector3d(0.5, 0, 0);
    kf1.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf1);

    ASSERT_TRUE(backend.optimize());

    std::vector<PoseStamped> poses;
    ASSERT_TRUE(backend.getUpdatedPoses(poses));
    ASSERT_EQ(poses.size(), 2);

    EXPECT_NEAR(poses[1].trans.x(), 0.75, 0.05);
}

TEST(CeresBackendTest, UpdateExistingKeyframe) {
    CeresBackend backend;
    backend.init("nonexistent.yaml");

    KeyFrame kf0;
    kf0.id = "0";
    kf0.pose.trans = Eigen::Vector3d(0, 0, 0);
    kf0.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf0);

    KeyFrame kf0_updated;
    kf0_updated.id = "0";
    kf0_updated.pose.trans = Eigen::Vector3d(5, 0, 0);
    kf0_updated.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf0_updated);

    KeyFrame kf1;
    kf1.id = "1";
    kf1.pose.trans = Eigen::Vector3d(6, 0, 0);
    kf1.pose.rot = Eigen::Matrix3d::Identity();
    backend.addKeyFrame(kf1);

    ASSERT_TRUE(backend.optimize());

    std::vector<PoseStamped> poses;
    ASSERT_TRUE(backend.getUpdatedPoses(poses));
    ASSERT_EQ(poses.size(), 2);

    EXPECT_NEAR(poses[0].trans.x(), 5.0, 1e-6);
    EXPECT_NEAR(poses[1].trans.x(), 6.0, 1e-6);
}

// namespace
