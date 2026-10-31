#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include "algorithms/fast_livo2/fast_livo2_frontend.h"
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

TEST(FastLivo2Frontend, LioSmokeWithoutImages) {
    const std::string path = "/tmp/fast_livo2_lio.yaml";
    {
        std::ofstream f(path);
        f << R"(
mapping:
  camera:
    model: Pinhole
    width: 640
    height: 480
    fx: 400.0
    fy: 400.0
    cx: 320.0
    cy: 240.0
  fast_livo2:
    img_en: false
    lidar_type: 8
    point_filter_num: 1
    blind: 0.1
    imu_int_frame: 30
)";
    }

    rosiwit_slam::FastLivo2Frontend frontend;
    ASSERT_TRUE(frontend.init(path));

    auto makeCloud = []() {
        auto cloud = std::make_shared<rosiwit_slam::CloudType>();
        for (int i = 0; i < 40; ++i) {
            for (int j = 0; j < 40; ++j) {
                rosiwit_slam::PointType p;
                p.x = -5.0f + 0.25f * i;
                p.y = -5.0f + 0.25f * j;
                p.z = 0.2f * std::sin(0.3f * i) + 0.1f * std::cos(0.2f * j);
                p.curvature = 0.1f * (i % 10);
                cloud->push_back(p);
            }
        }
        return cloud;
    };

    rosiwit_slam::LidarFrame frame;
    frame.cloud = makeCloud();
    frame.start_time = 1000.0;
    frame.end_time = 1000.1;
    frontend.onLidar(frame);

    // 1.6 s of IMU at 200 Hz with a 10 Hz lidar, like the real pipeline.
    for (int k = 0; k < 320; ++k) {
        const double t = 1000.0 + 0.005 * k;
        rosiwit_slam::IMUSample s;
        s.time = t;
        s.acc = rosiwit_slam::V3D(0.0, 0.0, 9.81);
        s.gyro = rosiwit_slam::V3D::Zero();
        frontend.onImu(s);

        if (k % 20 == 0) {
            frame.start_time = t;
            frame.end_time = t + 0.1;
            frontend.onLidar(frame);
        }
    }

    rosiwit_slam::PoseStamped pose;
    ASSERT_TRUE(frontend.getOdometry(pose));
    EXPECT_TRUE(pose.trans.allFinite());
    EXPECT_TRUE(pose.rot.allFinite());
    EXPECT_LT(pose.trans.norm(), 5.0);

    std::remove(path.c_str());
}
