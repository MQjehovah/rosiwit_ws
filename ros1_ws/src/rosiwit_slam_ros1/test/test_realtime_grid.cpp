// ============================================================================
// RealtimeGrid / buildGridFromPoints / loadGridConfigFromYaml 单测
// config 目录由 CMake 以 ROSIWIT_ROS1_CONFIG_DIR 宏传入 (不依赖测试进程 CWD)
// ============================================================================
#include "realtime_grid.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "slam_core/slam_types.h"

namespace rosiwit_slam_ros1 {
namespace {

using rosiwit_slam::GridInfo;
using rosiwit_slam::GridMapConfig;
using rosiwit_slam::GridPostConfig;
using rosiwit_slam::PointType;
using rosiwit_slam::PointVec;

/// PCL 1.8 的 PointXYZINormal 没有 (x,y,z) 三参构造, 逐字段赋值
PointType makePoint(double x, double y, double z) {
    PointType p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
    return p;
}

/// 墙的点间距必须**远小于**栅格分辨率, 两个原因:
///  1. 每格都得有落点, 否则栅格上出现空洞, 会被默认后处理第一步 denoise 当碎片清掉
///     (GridPostConfig::min_component_cells=30);
///  2. 点间距**正好等于**分辨率时更糟: VoxelGrid 取体素质心后落点与格边界对齐, 会稳定留洞
///     —— 实测 0.05 间距的 6m 墙被拆成 35 个连通域, 87 格全被删。
constexpr double kWallStep = 0.01;

/// 沿 x 造一堵 6m 长的薄墙 (6m / 0.05 = 120 格 > min_component_cells=30, 保得住)
PointVec wallAlongX(double y, double z, double x0 = -3.0, double x1 = 3.0) {
    PointVec v;
    for (double x = x0; x <= x1 + 1e-9; x += kWallStep) v.push_back(makePoint(x, y, z));
    return v;
}

/// 基准配置: ground_z 钉成 0 -> height(p) 就是 p.z (见 ground_segmentation.cpp:35-39)
GridMapConfig baseCfg() {
    GridMapConfig c;
    c.resolution = 0.05;
    c.min_height = 0.1;
    c.max_height = 2.0;
    c.occupied_thresh = 1;
    c.ground_z = 0.0;
    return c;
}

/// 关掉膨胀, 让占据格数只反映墙本身 (默认 dilate_radius=-1 + point_spacing=0 -> 本就是 no-op)
GridPostConfig basePost() {
    GridPostConfig p;
    p.dilate_radius = 0;
    return p;
}

/// 数占据格
int occupiedCount(const GridInfo& g) {
    return static_cast<int>(
        std::count(g.data.begin(), g.data.end(), static_cast<int8_t>(100)));
}

// 1
TEST(RealtimeGrid, BuildsGridFromSyntheticWall) {
    const PointVec wall = wallAlongX(0.0, 1.0);
    GridInfo info;
    ASSERT_TRUE(buildGridFromPoints(wall, baseCfg(), basePost(), "height_projection", info));

    EXPECT_GT(info.width, 0);
    EXPECT_GT(info.height, 0);
    EXPECT_NEAR(info.resolution, 0.05, 1e-9);
    EXPECT_EQ(static_cast<int>(info.data.size()), info.width * info.height);
    EXPECT_GT(occupiedCount(info), 0);
}

// 2
TEST(RealtimeGrid, ReturnsFalseOnEmptyInput) {
    const PointVec empty;
    GridInfo info;
    EXPECT_FALSE(buildGridFromPoints(empty, baseCfg(), basePost(), "height_projection", info));
}

// 3 —— z=0 的点低于 grid_min_height=0.1, 应该是自由而不是占据
TEST(RealtimeGrid, FloorPointsBecomeFreeNotOccupied) {
    const PointVec floor = wallAlongX(0.0, 0.0);
    GridInfo info;
    ASSERT_TRUE(buildGridFromPoints(floor, baseCfg(), basePost(), "height_projection", info));
    EXPECT_EQ(occupiedCount(info), 0);
}

// 4 —— 钉住行序: height_projection 的 row 0 = 最小 y (height_projection_grid_map.cpp:94-95)
TEST(RealtimeGrid, RowZeroIsMinY) {
    const PointVec wall = wallAlongX(5.0, 1.0); // y = +5
    GridInfo info;
    ASSERT_TRUE(buildGridFromPoints(wall, baseCfg(), basePost(), "height_projection", info));

    int max_row = -1;
    for (int r = 0; r < info.height; ++r)
        for (int c = 0; c < info.width; ++c)
            if (info.data[r * info.width + c] == 100) max_row = std::max(max_row, r);

    ASSERT_GE(max_row, 0);
    EXPECT_GT(max_row, info.height / 2)
        << "row 0 必须是最小 y, 否则 y=+5 的墙会落在上半区 (height=" << info.height << ")";
}

// 5
TEST(RealtimeGrid, LatestIsNullBeforeStart) {
    RealtimeGrid grid;
    grid.setConfig(baseCfg(), basePost(), "height_projection");
    GridInfo info;
    EXPECT_FALSE(grid.latest(info));
}

TEST(RealtimeGrid, LatestHasDataAfterStart) {
    RealtimeGrid grid;
    grid.setConfig(baseCfg(), basePost(), "height_projection");
    const PointVec wall = wallAlongX(0.0, 1.0);
    grid.setSource([wall](PointVec& out) {
        out = wall;
        return true;
    });

    grid.start(20.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    GridInfo info;
    EXPECT_TRUE(grid.latest(info));
    EXPECT_GT(info.width, 0);
    EXPECT_GT(occupiedCount(info), 0);
    grid.stop(); // 幂等
    grid.stop();
}

// 额外: source 说没有数据时不应产出栅格
TEST(RealtimeGrid, SourceReturningFalseProducesNothing) {
    RealtimeGrid grid;
    grid.setConfig(baseCfg(), basePost(), "height_projection");
    grid.setSource([](PointVec&) { return false; });
    grid.start(50.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    GridInfo info;
    EXPECT_FALSE(grid.latest(info));
    grid.stop();
}

// 额外: hz <= 0 按 1.0 处理 (不能变成忙等)
TEST(RealtimeGrid, NonPositiveHzFallsBackTo1) {
    RealtimeGrid grid;
    grid.setConfig(baseCfg(), basePost(), "height_projection");
    const PointVec wall = wallAlongX(0.0, 1.0);
    grid.setSource([wall](PointVec& out) {
        out = wall;
        return true;
    });
    grid.start(0.0); // -> 1.0 Hz; 立即先算一张, 所以 150ms 内就该有
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    GridInfo info;
    EXPECT_TRUE(grid.latest(info));
    grid.stop();
}

// 额外: 未知生成器名回落 height_projection (与核心 pcd_map_manager.cpp:109-115 一致)
TEST(RealtimeGrid, UnknownGeneratorFallsBack) {
    const PointVec wall = wallAlongX(0.0, 1.0);
    GridInfo info;
    ASSERT_TRUE(buildGridFromPoints(wall, baseCfg(), basePost(), "no_such_generator", info));
    EXPECT_GT(occupiedCount(info), 0);
}

// 6
TEST(RealtimeGrid, LoadsGridConfigFromDefaultYaml) {
    const std::string yaml = std::string(ROSIWIT_ROS1_CONFIG_DIR) + "/default.yaml";

    GridMapConfig cfg;
    GridPostConfig post;
    std::string generator;
    loadGridConfigFromYaml(yaml, cfg, post, generator);

    EXPECT_EQ(generator, "height_projection");
    EXPECT_NEAR(cfg.min_height, 0.1, 1e-9);
    EXPECT_NEAR(cfg.max_height, 2.0, 1e-9);
    EXPECT_EQ(cfg.occupied_thresh, 1);
    EXPECT_EQ(cfg.dilate_radius, 1);
    EXPECT_EQ(cfg.ground_method, "ransac");
    EXPECT_NEAR(cfg.ransac_dist_thr, 0.10, 1e-9);
    EXPECT_EQ(cfg.min_hits, 2);
    EXPECT_EQ(cfg.min_misses, 2);
    EXPECT_EQ(post.pipeline, (std::vector<std::string>{"denoise", "close", "dilate"}));
    EXPECT_EQ(post.connectivity, 8);
    EXPECT_EQ(post.close_radius, 1);
    EXPECT_EQ(post.dilate_radius, 1); // grid_dilate_radius 同时喂给生成配置与后处理
    EXPECT_EQ(post.min_component_cells, 30);
    // 分辨率/点间距来自 map_resolution / map_save_voxel (核心在 generateGridMap() 里赋)
    EXPECT_NEAR(cfg.resolution, 0.3, 1e-9);
    EXPECT_NEAR(cfg.point_spacing, 0.3, 1e-9);
    EXPECT_NEAR(post.point_spacing, 0.3, 1e-9);
}

// 额外: yaml 不存在时用结构默认值, 且不抛异常
TEST(RealtimeGrid, MissingYamlUsesDefaults) {
    GridMapConfig cfg;
    GridPostConfig post;
    std::string generator;
    loadGridConfigFromYaml("/nonexistent/definitely-not-here.yaml", cfg, post, generator);

    EXPECT_EQ(generator, "height_projection");
    EXPECT_NEAR(cfg.min_height, 0.1, 1e-9);
    EXPECT_NEAR(cfg.resolution, 0.05, 1e-9);
    EXPECT_EQ(post.pipeline, (std::vector<std::string>{"denoise", "close", "dilate"}));
}

} // namespace
} // namespace rosiwit_slam_ros1

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main), 所以这里自带 main
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
