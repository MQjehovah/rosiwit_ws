// test/test_grid_map_generator.cpp
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <limits>
#include <string>
#include "algorithms/grid_map/grid_map_factory.h"
#include "algorithms/grid_map/grid_map_io.h"
#include "algorithms/grid_map/height_projection_grid_map.h"
#include "algorithms/grid_map/ray_casting_grid_map.h"

using namespace rosiwit_slam;

namespace {

// 构造一个 5m x 5m 房间: 地面 z=0 + 四面高 0.2~1.5m 的墙
CloudType::Ptr makeRoomCloud(double size = 5.0) {
    CloudType::Ptr cloud(new CloudType());
    // 地面 (z=0, 低于障碍高度带)
    for (double x = -0.5; x <= size + 0.5; x += 0.1) {
        for (double y = -0.5; y <= size + 0.5; y += 0.1) {
            cloud->push_back(PointType(static_cast<float>(x), static_cast<float>(y), 0.0f));
        }
    }
    // 四面墙 (z 在障碍高度带内)
    for (double t = 0.0; t <= size; t += 0.1) {
        for (double z = 0.2; z <= 1.5; z += 0.1) {
            cloud->push_back(PointType(static_cast<float>(0.0), static_cast<float>(t), static_cast<float>(z)));
            cloud->push_back(PointType(static_cast<float>(size), static_cast<float>(t), static_cast<float>(z)));
            cloud->push_back(PointType(static_cast<float>(t), static_cast<float>(0.0), static_cast<float>(z)));
            cloud->push_back(PointType(static_cast<float>(t), static_cast<float>(size), static_cast<float>(z)));
        }
    }
    return cloud;
}

bool cellOccupied(const GridInfo& g, int col, int row) {
    if (col < 0 || col >= g.width || row < 0 || row >= g.height) return false;
    return g.data[row * g.width + col] == 100;
}

int occupiedCount(const GridInfo& g) {
    int n = 0;
    for (auto v : g.data) if (v == 100) ++n;
    return n;
}

int cellValue(const GridInfo& g, double x, double y) {
    int col = static_cast<int>((x - g.origin_x) / g.resolution);
    int row = static_cast<int>((y - g.origin_y) / g.resolution);
    if (col < 0 || col >= g.width || row < 0 || row >= g.height) return -2;
    return g.data[row * g.width + col];
}

// 原点(0,0) + 前方地面 + x=3 处一堵墙: 用于验证 free/occupied/unknown 三态
CloudType::Ptr makeWallScene() {
    CloudType::Ptr cloud(new CloudType());
    for (double x = -0.5; x <= 2.5; x += 0.1)
        for (double y = -2.0; y <= 2.0; y += 0.1)
            cloud->push_back(PointType(static_cast<float>(x), static_cast<float>(y), 0.0f));
    for (double y = -2.0; y <= 2.0; y += 0.05)
        for (double z = 0.2; z <= 1.0; z += 0.1)
            cloud->push_back(PointType(3.0f, static_cast<float>(y), static_cast<float>(z)));
    return cloud;
}

} // namespace

// ---- 参数化契约测试: 所有策略对同一输入应满足的公共行为 ----
class GridMapContractTest : public ::testing::TestWithParam<std::string> {};

TEST_P(GridMapContractTest, EmptyCloudFails) {
    auto gen = GridMapFactory::create(GetParam());
    ASSERT_NE(gen, nullptr);
    GridMapConfig cfg;
    GridInfo out;
    CloudType::Ptr empty(new CloudType());
    EXPECT_FALSE(gen->build(empty, Eigen::Vector3d::Zero(), cfg, out));
}

TEST_P(GridMapContractTest, NonFiniteOnlyFails) {
    auto gen = GridMapFactory::create(GetParam());
    ASSERT_NE(gen, nullptr);
    GridMapConfig cfg;
    GridInfo out;
    CloudType::Ptr cloud(new CloudType());
    const float nan = std::numeric_limits<float>::quiet_NaN();
    cloud->push_back(PointType(nan, nan, nan));
    cloud->push_back(PointType(std::numeric_limits<float>::infinity(), 0.f, 0.f));
    EXPECT_FALSE(gen->build(cloud, Eigen::Vector3d::Zero(), cfg, out));
}

TEST_P(GridMapContractTest, BuildsValidGrid) {
    auto gen = GridMapFactory::create(GetParam());
    ASSERT_NE(gen, nullptr);
    GridMapConfig cfg;
    cfg.resolution = 0.05;
    cfg.point_spacing = 0.2;      // 模拟建图下采样点间距
    GridInfo out;
    ASSERT_TRUE(gen->build(makeRoomCloud(), Eigen::Vector3d::Zero(), cfg, out));

    EXPECT_GT(out.width, 0);
    EXPECT_GT(out.height, 0);
    EXPECT_EQ(out.data.size(), static_cast<size_t>(out.width) * out.height);
    EXPECT_NEAR(out.resolution, cfg.resolution, 1e-9);
    // 包围盒外扩: 原点应小于最小坐标
    EXPECT_LT(out.origin_x, 0.0);
    EXPECT_LT(out.origin_y, 0.0);
    EXPECT_GT(occupiedCount(out), 0);
}

TEST_P(GridMapContractTest, PgmRoundTrip) {
    auto gen = GridMapFactory::create(GetParam());
    ASSERT_NE(gen, nullptr);
    GridMapConfig cfg;
    cfg.resolution = 0.1;
    cfg.point_spacing = 0.2;
    GridInfo out;
    ASSERT_TRUE(gen->build(makeRoomCloud(), Eigen::Vector3d::Zero(), cfg, out));

    std::string pgm = std::string(::testing::TempDir()) + "gm_test_" + GetParam() + ".pgm";
    std::string yml = std::string(::testing::TempDir()) + "gm_test_" + GetParam() + ".yaml";
    ASSERT_TRUE(saveGridMapPGM(out, pgm, yml, "map"));

    GridInfo loaded;
    ASSERT_TRUE(loadGridMapPGM(pgm, yml, loaded));
    EXPECT_EQ(loaded.width, out.width);
    EXPECT_EQ(loaded.height, out.height);
    EXPECT_NEAR(loaded.resolution, out.resolution, 1e-9);
    EXPECT_EQ(occupiedCount(loaded), occupiedCount(out));
}

INSTANTIATE_TEST_SUITE_P(AllStrategies, GridMapContractTest,
                         ::testing::ValuesIn(GridMapFactory::listNames()));

// ---- HeightProjection 专项: 墙体应连续, 门洞不被封死 ----
TEST(HeightProjectionGridMapTest, WallsAreContinuous) {
    HeightProjectionGridMap gen;
    GridMapConfig cfg;
    cfg.resolution = 0.05;
    cfg.point_spacing = 0.2;
    GridInfo out;
    ASSERT_TRUE(gen.build(makeRoomCloud(5.0), Eigen::Vector3d::Zero(), cfg, out));

    // 沿 x=0 的墙, 在 y∈[0.5,4.5] 上应基本每个 y 附近都有占据格
    int hits = 0, samples = 0;
    for (double y = 0.5; y <= 4.5; y += 0.25) {
        int row = static_cast<int>((y - out.origin_y) / out.resolution);
        int col = static_cast<int>((0.0 - out.origin_x) / out.resolution);
        bool found = false;
        for (int dc = -2; dc <= 2 && !found; ++dc)
            for (int dr = -2; dr <= 2 && !found; ++dr)
                found = cellOccupied(out, col + dc, row + dr);
        hits += found ? 1 : 0;
        samples++;
    }
    EXPECT_GT(static_cast<double>(hits) / samples, 0.9);
}

// ---- RayCasting 专项: free / occupied / unknown 三态正确 ----
TEST(RayCastingGridMapTest, TriStateFreeOccupiedUnknown) {
    RayCastingGridMap gen;
    GridMapConfig cfg;
    cfg.resolution = 0.1;
    cfg.dilate_radius = 0;      // 关闭膨胀, 检验原始三态
    GridInfo out;
    ASSERT_TRUE(gen.build(makeWallScene(), Eigen::Vector3d::Zero(), cfg, out));

    EXPECT_EQ(cellValue(out, 1.5, 0.0), 0);    // 原点与墙之间: free
    EXPECT_EQ(cellValue(out, 3.0, 0.0), 100);  // 墙: occupied
    EXPECT_EQ(cellValue(out, 3.1, 0.0), -1);   // 墙体之后: unknown
}

TEST(RayCastingGridMapTest, NeedsObservations) {
    RayCastingGridMap gen;
    EXPECT_TRUE(gen.needsObservations());
    HeightProjectionGridMap hp;
    EXPECT_FALSE(hp.needsObservations());
}
