#include <gtest/gtest.h>

#include "slam_core/i_frontend.h"
#include "slam_core/slam_factory.h"
#include "slam_core/slam_types.h"

namespace {

struct DummyFrontend : rosiwit_slam::IFrontend {
    bool init(const std::string&) override { return true; }
    void onImu(const rosiwit_slam::IMUSample&) override {}
    void onLidar(const rosiwit_slam::LidarFrame&) override {}
    bool getOdometry(rosiwit_slam::PoseStamped&) override { return false; }
    bool getClouds(rosiwit_slam::CloudType::Ptr&, rosiwit_slam::CloudType::Ptr&) override { return false; }
    bool getKeyFrame(rosiwit_slam::KeyFrame&) override { return false; }
    bool getGlobalMap(rosiwit_slam::PointVec&) override { return false; }
};

}  // namespace

TEST(FastLivo2Types, Defaults) {
    rosiwit_slam::ImageFrame frame;
    EXPECT_LT(frame.time, 0.0);
    EXPECT_TRUE(frame.image.empty());
    EXPECT_TRUE(frame.frame_id.empty());

    rosiwit_slam::CameraIntrinsics cam;
    EXPECT_EQ(cam.model, "Pinhole");
    EXPECT_DOUBLE_EQ(cam.scale, 1.0);

    rosiwit_slam::SlamOutput out;
    EXPECT_EQ(out.rgb_world_cloud, nullptr);

    DummyFrontend frontend;
    frontend.onImage(frame);  // default no-op, must not crash
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb;
    EXPECT_FALSE(frontend.getRGBCloud(rgb));
}

TEST(FastLivo2Factory, CreatesFrontend) {
    auto frontend = rosiwit_slam::SlamFactory::createFrontend("fast_livo2_frontend");
    ASSERT_NE(frontend, nullptr);
}
