#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "algorithms/fast_livo2/parse_config.h"

namespace {

std::string writeTempYaml(const std::string& name, const std::string& content) {
    auto path = std::string("/tmp/") + name;
    std::ofstream f(path);
    f << content;
    return path;
}

const char* kFullYaml = R"(
global:
  modules:
    frontend: fast_livo2_frontend
mapping:
  camera:
    model: Pinhole
    width: 752
    height: 480
    scale: 1.0
    fx: 425.0
    fy: 427.0
    cx: 386.0
    cy: 242.0
    d0: -0.28
    d1: 0.07
    d2: 0.001
    d3: -0.0002
  extrin_calib:
    lidar_imu_t: [1.0, 2.0, 3.0]
    lidar_imu_r: [1,0,0, 0,1,0, 0,0,1]
    camera_lidar_r: [0,1,0, 1,0,0, 0,0,1]
    camera_lidar_t: [0.1, 0.2, 0.3]
  fast_livo2:
    img_en: true
    lidar_type: 8
    point_filter_num: 3
    blind: 0.5
    time_offset:
      lidar: -0.1
      img: 0.05
      imu: 0.0
)";

const char* kRootOverrideYaml = R"(
r_il: [0,0,1, 0,1,0, -1,0,0]
t_il: [9.0, 8.0, 7.0]
mapping:
  camera:
    model: Pinhole
    width: 640
    height: 480
    fx: 400.0
    fy: 400.0
    cx: 320.0
    cy: 240.0
  extrin_calib:
    lidar_imu_t: [1.0, 2.0, 3.0]
    lidar_imu_r: [1,0,0, 0,1,0, 0,0,1]
)";

}  // namespace

TEST(FastLivo2Config, ParsesCameraExtrinsicsAndOffsets) {
    rosiwit_slam::FastLivo2Config cfg;
    std::string err;
    const std::string path = writeTempYaml("fast_livo2_full.yaml", kFullYaml);

    ASSERT_TRUE(rosiwit_slam::parseFastLivo2Config(path, cfg, err)) << err;

    EXPECT_EQ(cfg.camera.model, "Pinhole");
    EXPECT_EQ(cfg.camera.width, 752);
    EXPECT_DOUBLE_EQ(cfg.camera.fx, 425.0);
    EXPECT_DOUBLE_EQ(cfg.camera.d0, -0.28);

    EXPECT_DOUBLE_EQ(cfg.lidar_imu_trans.x(), 1.0);
    EXPECT_DOUBLE_EQ(cfg.lidar_imu_trans.y(), 2.0);
    EXPECT_DOUBLE_EQ(cfg.lidar_imu_trans.z(), 3.0);

    EXPECT_DOUBLE_EQ(cfg.cam_lidar_trans.x(), 0.1);
    EXPECT_DOUBLE_EQ(cfg.cam_lidar_trans.y(), 0.2);
    EXPECT_DOUBLE_EQ(cfg.cam_lidar_trans.z(), 0.3);
    EXPECT_DOUBLE_EQ(cfg.cam_lidar_rot(0, 1), 1.0);

    EXPECT_DOUBLE_EQ(cfg.lidar_time_offset, -0.1);
    EXPECT_DOUBLE_EQ(cfg.img_time_offset, 0.05);
    EXPECT_DOUBLE_EQ(cfg.imu_time_offset, 0.0);

    EXPECT_TRUE(cfg.img_en);
    EXPECT_EQ(cfg.lidar_type, 8);
    EXPECT_EQ(cfg.point_filter_num, 3);
    EXPECT_DOUBLE_EQ(cfg.blind, 0.5);

    std::remove(path.c_str());
}

TEST(FastLivo2Config, RootTfExtrinsicsOverrideYaml) {
    rosiwit_slam::FastLivo2Config cfg;
    std::string err;
    const std::string path = writeTempYaml("fast_livo2_root.yaml", kRootOverrideYaml);

    ASSERT_TRUE(rosiwit_slam::parseFastLivo2Config(path, cfg, err)) << err;

    // Root r_il/t_il (TF-derived) win over mapping.extrin_calib.
    EXPECT_DOUBLE_EQ(cfg.lidar_imu_trans.x(), 9.0);
    EXPECT_DOUBLE_EQ(cfg.lidar_imu_rot(0, 2), 1.0);
    EXPECT_DOUBLE_EQ(cfg.lidar_imu_rot(0, 0), 0.0);

    std::remove(path.c_str());
}

TEST(FastLivo2Config, MissingCameraFails) {
    rosiwit_slam::FastLivo2Config cfg;
    std::string err;
    const std::string path = writeTempYaml("fast_livo2_nocam.yaml", "mapping:\n  fast_livo2:\n    blind: 0.5\n");

    EXPECT_FALSE(rosiwit_slam::parseFastLivo2Config(path, cfg, err));
    EXPECT_FALSE(err.empty());

    std::remove(path.c_str());
}
