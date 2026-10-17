// ============================================================================
// LegacyConfig 单测 —— 覆盖旧配置的整条覆盖链与逐键默认值
// fixture 目录由 CMake 以 LEGACY_FIXTURE_DIR 宏传入 (不依赖测试进程的 CWD)
// ============================================================================
#include "legacy_config.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace rosiwit_slam_ros1 {
namespace {

std::string FIX(const std::string& name) { return std::string(LEGACY_FIXTURE_DIR) + "/" + name; }

const char* const kMissing = "/nonexistent/definitely-not-here.json";

/// 全部路径都不存在: 只给内置默认值
LegacyConfPaths nothingExists() {
    LegacyConfPaths p;
    p.proj_conf_json = kMissing;
    p.pkg_conf_json = kMissing;
    p.opt_sensor_json = kMissing;
    p.opt_slam_json = kMissing;
    return p;
}

/// 只有主配置 (proj) 的常见形态
LegacyConfPaths onlyProj(const std::string& proj) {
    LegacyConfPaths p = nothingExists();
    p.proj_conf_json = proj;
    return p;
}

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

// 1
TEST(LegacyConfig, DefaultsWhenNothingExists) {
    const LegacyConfig c = LegacyConfig::load(nothingExists(), "");

    EXPECT_DOUBLE_EQ(c.pitch_new, 0.0);
    EXPECT_DOUBLE_EQ(c.yaw_new, 90.0);
    EXPECT_DOUBLE_EQ(c.imu_in_car_frame_yaw_livox, 65.0);
    EXPECT_DOUBLE_EQ(c.imu_in_car_frame_yaw_wj, 60.0);
    EXPECT_EQ(c.laser_sub_name, "rslidar_points");
    EXPECT_EQ(c.imu_sub_name, "zyz_imu_data");
    EXPECT_TRUE(c.use_2d_undistort);
    EXPECT_EQ(c.lidar_pts_monitor_interval, 10);
    EXPECT_TRUE(c.lidar_pts_monitor);
    EXPECT_DOUBLE_EQ(c.yaw_calib, 0.0);
    EXPECT_EQ(c.slam_init_type, "localization");
    EXPECT_EQ(c.publish_hz, 20);
    EXPECT_EQ(c.rt_grid_map_publish_hz, 1);
    EXPECT_EQ(c.test_wait_time, 30);
    EXPECT_DOUBLE_EQ(c.delete_pt_in_car_min_z, -0.5); // 旧代码默认, 不是文件里的 -10
    EXPECT_DOUBLE_EQ(c.th_2d_range_min, 0.5);
    EXPECT_DOUBLE_EQ(c.th_2d_range_max, 3.0);
    EXPECT_DOUBLE_EQ(c.mapping_z_min, 0.0); // 缺省取 use_3d_z_min
    EXPECT_DOUBLE_EQ(c.mapping_z_max, 20.0);
    // model 为空且 sensor_config 缺失 -> config_type=Livox_Mid360, 但机型不匹配 -> laser_type 空
    EXPECT_EQ(c.laser_type, "");
}

// 2
TEST(LegacyConfig, ProjConfOverridesDefaults) {
    const LegacyConfig c = LegacyConfig::load(onlyProj(FIX("slam_conf_min.json")), "XZ-SW50");
    EXPECT_DOUBLE_EQ(c.pitch_new, 10.0);
    EXPECT_DOUBLE_EQ(c.yaw_new, 90.0); // 未被 fixture 覆盖
}

// 3
TEST(LegacyConfig, OptSlamConfDeepMergesOverProj) {
    LegacyConfPaths p = onlyProj(FIX("slam_conf_min.json"));
    p.opt_slam_json = FIX("slam_conf_override.json");
    const LegacyConfig c = LegacyConfig::load(p, "XZ-SWGT");

    EXPECT_DOUBLE_EQ(c.pitch_new, 33.0); // 覆盖生效
    EXPECT_DOUBLE_EQ(c.yaw_new, 90.0);   // 主配置里的其它键仍在
    // 机型 = GT 且 sensor_config 缺失 -> config_type=Livox_Mid360 -> laser_type=new_mid360
    EXPECT_EQ(c.laser_type, "new_mid360");
    EXPECT_DOUBLE_EQ(c.loc_z_max, 5.0); // laser_params.mid360 子树合并进来
}

// 4
TEST(LegacyConfig, LaserParamsSubtreeMergedForBaseType) {
    LegacyConfPaths p = onlyProj(FIX("slam_conf_override.json"));
    p.opt_sensor_json = FIX("sensor_vanjee_gt.json"); // WLR_722 + XZ-SWGT
    const LegacyConfig c = LegacyConfig::load(p, "XZ-SWGT");

    EXPECT_EQ(c.laser_type, "new_vanjee");
    // new_vanjee -> 合并 laser_params.vanjee 子树
    EXPECT_EQ(c.laser_sub_name, "vanjee_points722");
}

// 5
TEST(LegacyConfig, Mid360BaseTypeStripsNewPrefix) {
    LegacyConfPaths p = onlyProj(FIX("slam_conf_override.json"));
    p.opt_sensor_json = FIX("sensor_mid360.json");
    const LegacyConfig c = LegacyConfig::load(p, "XZ-SWGT");

    EXPECT_EQ(c.laser_type, "new_mid360");
    EXPECT_DOUBLE_EQ(c.loc_z_max, 5.0); // new_mid360 -> 合并 laser_params.mid360
}

// 6
TEST(LegacyConfig, RobosenseGtForcesTopicNames) {
    LegacyConfPaths p = nothingExists();
    p.opt_sensor_json = FIX("sensor_robosense.json");
    const LegacyConfig c = LegacyConfig::load(p, "XZ-SWGT");

    EXPECT_EQ(c.laser_type, "new_robosense");
    EXPECT_EQ(c.laser_sub_name, "rslidar_points");
    EXPECT_EQ(c.imu_sub_name, "rslidar_imu");
}

// 7
TEST(LegacyConfig, UnknownModelLeavesLaserTypeEmpty) {
    LegacyConfPaths p = nothingExists();
    p.opt_sensor_json = FIX("sensor_robosense.json");
    const LegacyConfig c = LegacyConfig::load(p, ""); // 机型未知

    EXPECT_EQ(c.laser_type, "");
    // 未命中 Robosense 分支, 话题名保持默认
    EXPECT_EQ(c.laser_sub_name, "rslidar_points");
    EXPECT_EQ(c.imu_sub_name, "zyz_imu_data");
}

// 8
TEST(LegacyConfig, UseDeletePtsGatedByModel) {
    const std::string conf = FIX("slam_conf_min.json"); // use_delete_pts 未配置 -> 默认 true
    EXPECT_FALSE(LegacyConfig::load(onlyProj(conf), "XZ-SWGT").use_delete_pts);
    EXPECT_TRUE(LegacyConfig::load(onlyProj(conf), "XZ-SW50").use_delete_pts);
}

// 9
TEST(LegacyConfig, YawCalibConvertedToRadiansWhenPresent) {
    const LegacyConfig c =
        LegacyConfig::load(onlyProj(FIX("slam_conf_yaw_calib.json")), "XZ-SW50");
    EXPECT_TRUE(near(c.yaw_calib, M_PI / 2.0));
}

// 10
TEST(LegacyConfig, PkgConfUsedAsFallback) {
    LegacyConfPaths p = nothingExists();
    p.pkg_conf_json = FIX("slam_conf_min.json");
    const LegacyConfig c = LegacyConfig::load(p, "XZ-SW50");
    EXPECT_DOUBLE_EQ(c.pitch_new, 10.0);
}

// 额外: proj 存在时优先于 pkg
TEST(LegacyConfig, ProjConfBeatsPkgConf) {
    LegacyConfPaths p = nothingExists();
    p.proj_conf_json = FIX("slam_conf_min.json");      // pitch_new=10
    p.pkg_conf_json = FIX("slam_conf_override.json");  // pitch_new=33
    const LegacyConfig c = LegacyConfig::load(p, "XZ-SW50");
    EXPECT_DOUBLE_EQ(c.pitch_new, 10.0);
}

// 额外: lidar_pts_monitor_interval < 1 时强制 10 (legacy.cpp:1697)
TEST(LegacyConfig, MonitorIntervalBelowOneForcedToTen) {
    const LegacyConfig c =
        LegacyConfig::load(onlyProj(FIX("slam_conf_tiny_interval.json")), "XZ-SW50");
    EXPECT_EQ(c.lidar_pts_monitor_interval, 10);
}

// 额外: 日志两键来自 conf; 缺失时用旧 node main 的回落值
TEST(LegacyConfig, LogKeysFromConfAndDefaults) {
    const LegacyConfig c = LegacyConfig::load(onlyProj(FIX("slam_conf_log.json")), "XZ-SW50");
    EXPECT_EQ(c.xz_log_path, "/opt/xzrobot/logs/xzrobot_3d_slam/xz_3d_slam.log");
    EXPECT_TRUE(c.print_xz_log_to_file);

    const LegacyConfig d = LegacyConfig::load(nothingExists(), "XZ-SW50");
    EXPECT_EQ(d.xz_log_path, "/home/xzrobot/slam_logs/xz_3d_slam.log");
    EXPECT_FALSE(d.print_xz_log_to_file);
}

// 11
// 实机实测: UI 传的 map_dir 其实是**文件路径** (".../map.yaml"),
// 旧实现靠 substr(0,32) 恰好切掉后缀; 我们泛化成剥后缀。
TEST(LegacyConfig, LegacyMapBaseAcceptsMapYamlFilePath) {
    EXPECT_EQ(legacyMapBase("/opt/xzrobot/maps/1789354050210/map.yaml"),
              "/opt/xzrobot/maps/1789354050210/map");
    EXPECT_EQ(legacyMapBase("/opt/xzrobot/maps/1789354050210/map.pgm"),
              "/opt/xzrobot/maps/1789354050210/map");
    EXPECT_EQ(legacyMapBase("/opt/xzrobot/maps/1789354050210/map.xzmap"),
              "/opt/xzrobot/maps/1789354050210/map");
    // 目录形式(带/不带尾斜杠)与已是基名 的形式都仍然对
    EXPECT_EQ(legacyMapBase("/opt/xzrobot/maps/1789354050210/"),
              "/opt/xzrobot/maps/1789354050210/map");
    EXPECT_EQ(legacyMapBase("/opt/xzrobot/maps/1789354050210"),
              "/opt/xzrobot/maps/1789354050210/map");
    EXPECT_EQ(legacyMapBase("/opt/xzrobot/maps/1789354050210/map"),
              "/opt/xzrobot/maps/1789354050210/map");
}

TEST(LegacyConfig, LegacyMapBaseStripsTrailingSlash) {
    EXPECT_EQ(legacyMapBase("/a/b/"), "/a/b/map");
}

// 12
TEST(LegacyConfig, LegacyMapBaseWithoutTrailingSlash) {
    EXPECT_EQ(legacyMapBase("/a/b"), "/a/b/map");
}

// 13
TEST(LegacyConfig, LegacyMapBaseLongPathNotTruncated) {
    // 旧实现用 substr(0,32) 会截断; 这是设计文档 D9 的有意修正
    const std::string dir = "/opt/xzrobot/maps/some_really_long_map_dir_name_1234567890";
    ASSERT_GT(dir.size(), 40u);
    EXPECT_EQ(legacyMapBase(dir), dir + "/map");
}

// 14
TEST(LegacyConfig, LegacyMapBaseRoot) {
    EXPECT_EQ(legacyMapBase("/"), "/map");
}

// 额外: 空 map_dir 也按根处理 (宁可拼出 "/map" 让写盘权限报错, 也不拼成相对路径)
TEST(LegacyConfig, LegacyMapBaseEmptyDir) {
    EXPECT_EQ(legacyMapBase(""), "/map");
}

// 额外: 多个尾斜杠全部去掉
TEST(LegacyConfig, LegacyMapBaseMultipleTrailingSlashes) {
    EXPECT_EQ(legacyMapBase("/a/b///"), "/a/b/map");
}

}  // namespace
}  // namespace rosiwit_slam_ros1

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main), 所以这里自带 main
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
