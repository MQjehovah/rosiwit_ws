#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "ros_interface/frame_utils.h"

namespace rosiwit_slam_ros2 {
namespace {

RigidTransform makeBaseImu() {
    RigidTransform T;
    T.set(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
          Eigen::Vector3d(1.0, 0.0, 0.0));
    return T;   // T_base_link_imu
}

rosiwit_slam::PoseStamped makePose() {
    rosiwit_slam::PoseStamped p;
    p.time = 12.5;
    p.rot = Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    p.trans = Eigen::Vector3d(-2.0, 3.5, 0.4);
    p.vel = Eigen::Vector3d(0.5, -0.1, 0.0);
    return p;
}

TEST(FrameUtils, BaseImuRoundTrip) {
    const RigidTransform base_imu = makeBaseImu();
    const auto p_base = makePose();
    const auto p_imu = composeRight(p_base, base_imu);
    const auto back = composeRightInv(p_imu, base_imu);
    EXPECT_TRUE(back.rot.isApprox(p_base.rot, 1e-12));
    EXPECT_TRUE(back.trans.isApprox(p_base.trans, 1e-12));
    EXPECT_TRUE(back.vel.isApprox(p_base.vel));
}

TEST(FrameUtils, KnownYaw90Offset) {
    const RigidTransform base_imu = makeBaseImu();
    rosiwit_slam::PoseStamped p_base;
    p_base.trans = Eigen::Vector3d(1.0, 0.0, 0.0);
    p_base.rot = Eigen::Matrix3d::Identity();

    const auto p_imu = composeRight(p_base, base_imu);
    // T_world_imu = T_world_base * T_base_imu: t = (1,0,0) + (1,0,0)
    EXPECT_TRUE(p_imu.trans.isApprox(Eigen::Vector3d(2.0, 0.0, 0.0), 1e-12));
    EXPECT_TRUE(p_imu.rot.isApprox(
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(), 1e-12));
}

TEST(FrameUtils, CovarianceRotates) {
    RigidTransform T;
    T.set(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
          Eigen::Vector3d::Zero());
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
    cov(3, 3) = 4.0;
    const auto out = transformCovariance(cov, T.R);
    EXPECT_NEAR(out(4, 4), 4.0, 1e-12);
    EXPECT_NEAR(out(3, 3), 0.0, 1e-12);
}

}  // namespace
}  // namespace rosiwit_slam_ros2
