// ============================================================================
// legacy_convert 单测 (计划 T5 §5.1 的 6 条 + 3 条补充)
// ============================================================================
#include "legacy_convert.h"

#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace rosiwit_slam_ros1 {
namespace {

using rosiwit_slam::CloudType;
using rosiwit_slam::GridInfo;
using rosiwit_slam::PointType;

GridInfo makeGrid() {
    GridInfo gi;
    gi.width = 3;
    gi.height = 2;
    gi.resolution = 0.5;
    gi.origin_x = 1.0;
    gi.origin_y = -2.0;
    gi.data = {10, 11, 12, 13, 14, 15};   // 行 0 = {10,11,12}, 行 1 = {13,14,15}
    return gi;
}

PointType mkPoint(double x, double y, double z) {
    PointType p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
    return p;
}

// 1
TEST(LegacyConvert, LegacyGridKeepsRowOrder) {
    const GridInfo gi = makeGrid();
    const auto g = toLegacyGrid(gi, "map");

    EXPECT_EQ(g.data.size(), gi.data.size());
    EXPECT_TRUE(std::equal(g.data.begin(), g.data.end(), gi.data.begin()));
    // 明确钉住"不翻转": 第 0 个元素仍是 10
    EXPECT_EQ(static_cast<int>(g.data[0]), 10);
    EXPECT_EQ(static_cast<int>(g.data[5]), 15);
}

// 2
TEST(LegacyConvert, FlippedGridFlipsRows) {
    const GridInfo gi = makeGrid();
    const auto g = toFlippedGrid(gi, "map");

    // 首行(3 个) == 原最后一行
    EXPECT_EQ(static_cast<int>(g.data[0]), 13);
    EXPECT_EQ(static_cast<int>(g.data[1]), 14);
    EXPECT_EQ(static_cast<int>(g.data[2]), 15);
    // 次行 == 原首行
    EXPECT_EQ(static_cast<int>(g.data[3]), 10);
    EXPECT_EQ(static_cast<int>(g.data[4]), 11);
    EXPECT_EQ(static_cast<int>(g.data[5]), 12);
}

// 3
TEST(LegacyConvert, LegacyGridHeaderAndInfo) {
    const GridInfo gi = makeGrid();
    const auto g = toLegacyGrid(gi, "map");

    EXPECT_EQ(g.header.frame_id, "map");
    EXPECT_NEAR(g.info.resolution, 0.5, 1e-12);
    EXPECT_EQ(g.info.width, 3);
    EXPECT_EQ(g.info.height, 2);
    EXPECT_NEAR(g.info.origin.position.x, 1.0, 1e-12);
    EXPECT_NEAR(g.info.origin.position.y, -2.0, 1e-12);
    EXPECT_NEAR(g.info.origin.orientation.w, 1.0, 1e-12);
}

// 4
TEST(LegacyConvert, SlamInfoFieldMapping) {
    const auto si = buildSlamInfo(1, 2, "map_alpha", 1.5, -2.5, 0.7, 12.5f);

    EXPECT_EQ(static_cast<int>(si.slam_state), 1);
    EXPECT_EQ(static_cast<int>(si.location_status), 2);
    EXPECT_EQ(si.map_name, "map_alpha");
    EXPECT_NEAR(si.robot_pose.x, 1.5, 1e-12);
    EXPECT_NEAR(si.robot_pose.y, -2.5, 1e-12);
    EXPECT_NEAR(si.robot_pose.theta, 0.7, 1e-12);
    EXPECT_NEAR(si.map_area, 12.5f, 1e-6);
    EXPECT_EQ(static_cast<int>(si.update_request), 0);
    EXPECT_NEAR(si.matching_degree, 0.0f, 1e-9);   // D3
}

// 5
TEST(LegacyConvert, WarningJsonShape) {
    // 用 T1 抄进来的那份 warning_code.json 里的真值 (20003504 = pose lost)
    const std::string j = buildWarningJson("20003504", "FAULT", "pose lost",
                                           "slam localization lost post!");
    const YAML::Node n = YAML::Load(j);

    ASSERT_TRUE(n["faults"]);
    ASSERT_TRUE(n["faults"].IsSequence());
    ASSERT_EQ(n["faults"].size(), 1u);

    const YAML::Node f = n["faults"][0];
    EXPECT_EQ(f["code"].as<std::string>(), "20003504");
    EXPECT_EQ(f["module"].as<std::string>(), "xz_3d_slam");
    EXPECT_EQ(f["type"].as<std::string>(), "algorithm fault");
    EXPECT_EQ(f["level"].as<std::string>(), "FAULT");
    EXPECT_EQ(f["name"].as<std::string>(), "pose lost");
    EXPECT_EQ(f["content"].as<std::string>(), "slam localization lost post!");
}

// 5b —— 转义: 内容里带引号也不能把 JSON 打坏
TEST(LegacyConvert, WarningJsonEscapesQuotes) {
    const std::string j = buildWarningJson("20003001", "WARNING", "he said \"hi\"",
                                           "a\\b");
    const YAML::Node n = YAML::Load(j);
    EXPECT_EQ(n["faults"][0]["name"].as<std::string>(), "he said \"hi\"");
    EXPECT_EQ(n["faults"][0]["content"].as<std::string>(), "a\\b");
}

// 5c —— 多码合并 (旧实现 toJson(vector<int>) 一次成型)
TEST(LegacyConvert, WarningJsonMultiCode) {
    std::vector<WarningFault> fs;
    fs.push_back({"20003001", "WARNING", "lidar hz low", "slam subscripe lidar frequency is low!"});
    fs.push_back({"20003002", "WARNING", "imu hz low", "slam subscripe imu frequency is low!"});
    const YAML::Node n = YAML::Load(buildWarningJson(fs));
    ASSERT_EQ(n["faults"].size(), 2u);
    EXPECT_EQ(n["faults"][0]["code"].as<std::string>(), "20003001");
    EXPECT_EQ(n["faults"][1]["code"].as<std::string>(), "20003002");
    EXPECT_EQ(n["faults"][1]["level"].as<std::string>(), "WARNING");
    // 空列表 -> 空串 (旧实现 json_warning_code.cpp:57-59)
    EXPECT_EQ(buildWarningJson(std::vector<WarningFault>{}), "");
}

// 6 —— 钉住符号。对 R = Rot(Z,θ): R(0,1) = -sinθ, R(0,0) = cosθ,
//      所以 legacyYawFromRot(Rot(Z,+π/2)) == -π/2; 旧实现随后用 AngleAxis(-yaw, Z)
//      再取负, 合起来还原成 Rot(Z,θ) (legacy.cpp:1921-1924)。
TEST(LegacyConvert, LegacyYawFromIdentityIsZero) {
    EXPECT_NEAR(legacyYawFromRot(Eigen::Matrix3d::Identity()), 0.0, 1e-12);
}

TEST(LegacyConvert, LegacyYawFromPlus90degIsMinusHalfPi) {
    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    EXPECT_NEAR(R(0, 1), -1.0, 1e-12);   // 不是 +1
    EXPECT_NEAR(R(0, 0), 0.0, 1e-12);

    const double yaw = legacyYawFromRot(R);
    EXPECT_NEAR(yaw, -M_PI / 2.0, 1e-12);

    // 旧 TF 的完整套路: 再转 -yaw 就还原成 R 里的那个 +90°
    const Eigen::Matrix3d R2 =
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    EXPECT_TRUE(R2.isApprox(R, 1e-12));
}

// 6b —— 标准航向角 (slam_info.robot_pose.theta 用它) 与 legacyYawFromRot 反号
TEST(LegacyConvert, StdYawIsNegativeOfLegacyYaw) {
    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    EXPECT_NEAR(stdYawFromRot(R), M_PI / 3.0, 1e-12);
    EXPECT_NEAR(stdYawFromRot(R), -legacyYawFromRot(R), 1e-12);
    EXPECT_NEAR(stdYawFromRot(Eigen::Matrix3d::Identity()), 0.0, 1e-12);
}

// 7 —— 塑形: z 过滤 + 抽稀
TEST(LegacyConvert, ScanShapeFiltersByZ) {
    CloudType in;
    in.push_back(mkPoint(0, 0, -0.5));   // 低于 0    -> 丢
    in.push_back(mkPoint(0, 0, 0.0));    // 正好 0    -> 丢 (条件是 p.z > 0)
    in.push_back(mkPoint(0, 0, 0.5));    // 留下
    in.push_back(mkPoint(0, 0, 3.0));    // 高于 z_max -> 丢

    auto out = legacyScanShape(in, 2.0);
    ASSERT_EQ(out->size(), 1u);
    EXPECT_NEAR(out->points[0].z, 0.5, 1e-6);

    // 建图的 z 上界是 1.0
    auto out_map = legacyScanShape(in, 1.0);
    ASSERT_EQ(out_map->size(), 1u);
}

TEST(LegacyConvert, ScanShapeZMaxNonPositiveMeansNoFilter) {
    CloudType in;
    for (double z = -1.0; z <= 3.0; z += 0.5) in.push_back(mkPoint(0, 0, z));
    auto out = legacyScanShape(in, -1.0);   // 不过滤 (旧实现的"原始帧"就是这样)
    EXPECT_EQ(out->size(), in.size());
}

TEST(LegacyConvert, ScanShapeDownsamplesToAbout2000) {
    CloudType in;
    for (int i = 0; i < 10000; ++i) in.push_back(mkPoint(0, 0, 0.5));  // 全部在 z 带内
    auto out = legacyScanShape(in, 2.0);           // down_sample = round(10000/2000) = 5
    EXPECT_EQ(out->size(), 2000u);

    auto out2 = legacyScanShape(in, 2.0, 2);       // 再乘 2 -> 每 10 个取 1
    EXPECT_EQ(out2->size(), 1000u);

    // 少于 2000 点时 down_sample 固定为 1, 一个都不抽
    CloudType small;
    for (int i = 0; i < 100; ++i) small.push_back(mkPoint(0, 0, 0.5));
    EXPECT_EQ(legacyScanShape(small, 2.0)->size(), 100u);
}

// 8 —— 老类型转换确实产出的不是 PointCloud2
TEST(LegacyConvert, OldPointCloudHasPointsAndHeader) {
    CloudType in;
    in.push_back(mkPoint(1, 2, 3));
    in.push_back(mkPoint(4, 5, 6));

    const ros::Time stamp(123, 456);
    auto pc = toOldPointCloud(in, "map", stamp);
    EXPECT_EQ(pc.header.frame_id, "map");
    EXPECT_EQ(pc.header.stamp, stamp);
    ASSERT_EQ(pc.points.size(), 2u);
    EXPECT_NEAR(pc.points[0].x, 1.0, 1e-6);
    EXPECT_NEAR(pc.points[1].z, 6.0, 1e-6);

    auto pc2 = toPointCloud2(in, "map", stamp);
    EXPECT_EQ(pc2.header.frame_id, "map");
    EXPECT_EQ(pc2.header.stamp, stamp);
    EXPECT_EQ(pc2.width, 2u);
}

} // namespace
} // namespace rosiwit_slam_ros1

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main), 所以这里自带 main
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
