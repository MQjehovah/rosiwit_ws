#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include "algorithms/gicp_localization/gicp_localization.h"

using namespace rosiwit_slam;

TEST(GicpLocalizationTest, CreateAndInit) {
    GicpLocalization loc;
    EXPECT_TRUE(loc.init("nonexistent.yaml"));
    EXPECT_EQ(loc.getStatus(), ILocalization::INIT);
}

TEST(GicpLocalizationTest, InitWithoutMapReturnsLOST) {
    GicpLocalization loc;
    loc.init("nonexistent.yaml");

    PoseStamped init_pose;
    init_pose.trans = V3D(0, 0, 0);
    loc.setInitPose(init_pose);

    LidarFrame frame;
    frame.cloud.reset(new CloudType());
    frame.cloud->push_back(PointType());
    loc.onLidar(frame);

    EXPECT_EQ(loc.getStatus(), ILocalization::LOST);
}

TEST(GicpLocalizationTest, GetPoseSucceedsWhenInit) {
    GicpLocalization loc;
    loc.init("nonexistent.yaml");
    PoseStamped out;
    EXPECT_TRUE(loc.getPose(out));
}

TEST(GicpLocalizationTest, GetPoseFailsWhenLost) {
    GicpLocalization loc;
    loc.init("nonexistent.yaml");
    loc.setInitPose(PoseStamped());
    LidarFrame frame;
    frame.cloud.reset(new CloudType());
    loc.onLidar(frame);
    PoseStamped out;
    EXPECT_FALSE(loc.getPose(out));
}

TEST(GicpLocalizationTest, SetInitPoseWorks) {
    GicpLocalization loc;
    loc.init("nonexistent.yaml");

    PoseStamped init_pose;
    init_pose.trans = V3D(1.0, 2.0, 3.0);
    loc.setInitPose(init_pose);

    GicpLocalization loc2;
    loc2.init("nonexistent.yaml");
    PoseStamped init_pose2;
    init_pose2.trans = V3D(4.0, 5.0, 6.0);
    loc2.setInitPose(init_pose2);

    SUCCEED();
}

TEST(GicpLocalizationTest, OnImuNoCrash) {
    GicpLocalization loc;
    loc.init("nonexistent.yaml");
    IMUSample imu;
    imu.acc = V3D(0, 0, 9.81);
    imu.gyro = V3D::Zero();
    loc.onImu(imu);
    SUCCEED();
}

TEST(GicpLocalizationTest, GetStatusReturnsInitAfterConstruction) {
    GicpLocalization loc;
    EXPECT_EQ(loc.getStatus(), ILocalization::INIT);
}
