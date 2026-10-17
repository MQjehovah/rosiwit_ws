// 实时栅格的单元测试 (薄层)
//
// 历史: 本文件原先测的是 RealtimeGrid 自己算栅格的旧实现 (buildGridFromPoints)。
// 该实现已删除 —— 实时 /map 现在统一走核心 PcdMapManager::generateGridMap
// (与存 pgm 同一条路径, 有逐帧地面处理), 栅格算法本身由核心侧的
// src/rosiwit_slam/test/test_grid_map_generator.cpp 覆盖。
//
// 这里只保留薄层还剩下的职责: 取"已生成好的栅格"并发布给订阅方, 以及线程/清空语义。

#include <gtest/gtest.h>

#include <thread>

#include "realtime_grid.h"

namespace {

rosiwit_slam::GridInfo makeGrid(int w, int h, int8_t v) {
    rosiwit_slam::GridInfo g;
    g.width = w;
    g.height = h;
    g.resolution = 0.05;
    g.origin_x = -1.0;
    g.origin_y = -2.0;
    g.data.assign(static_cast<size_t>(w) * h, v);
    return g;
}

} // namespace

// 没有设置数据源时不应产出结果
TEST(RealtimeGrid, NoSourceNoResult) {
    rosiwit_slam_ros1::RealtimeGrid rg;
    rosiwit_slam::GridInfo out;
    EXPECT_FALSE(rg.latest(out));
}

// 设置 gridSource 后, start() 应把源给的那张栅格取过来
TEST(RealtimeGrid, GridSourceIsUsed) {
    rosiwit_slam_ros1::RealtimeGrid rg;
    const rosiwit_slam::GridInfo g = makeGrid(8, 4, 100);

    int calls = 0;
    rg.setGridSource([&](rosiwit_slam::GridInfo& out) {
        ++calls;
        out = g;
        return true;
    });
    rg.start(50.0);   // 50Hz: 很快就有结果

    rosiwit_slam::GridInfo got;
    bool ok = false;
    for (int i = 0; i < 100 && !ok; ++i) {
        ok = rg.latest(got);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rg.stop();

    EXPECT_TRUE(ok);
    EXPECT_GT(calls, 0);
    EXPECT_EQ(got.width, 8);
    EXPECT_EQ(got.height, 4);
    EXPECT_DOUBLE_EQ(got.resolution, 0.05);
    EXPECT_DOUBLE_EQ(got.origin_x, -1.0);
    EXPECT_DOUBLE_EQ(got.origin_y, -2.0);
}

// 源返回 false 时不应刷新 (保持"没有结果")
TEST(RealtimeGrid, SourceFailKeepsNoResult) {
    rosiwit_slam_ros1::RealtimeGrid rg;
    rg.setGridSource([](rosiwit_slam::GridInfo&) { return false; });
    rg.start(50.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rg.stop();

    rosiwit_slam::GridInfo out;
    EXPECT_FALSE(rg.latest(out));
}

// clear() 之后 latest() 应回到"没有结果" (进回建图/重建核心时用)
TEST(RealtimeGrid, ClearDropsCachedGrid) {
    rosiwit_slam_ros1::RealtimeGrid rg;
    const rosiwit_slam::GridInfo g = makeGrid(4, 4, 0);
    rg.setGridSource([&](rosiwit_slam::GridInfo& out) {
        out = g;
        return true;
    });
    rg.start(50.0);

    rosiwit_slam::GridInfo out;
    for (int i = 0; i < 100 && !rg.latest(out); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_TRUE(rg.latest(out));

    rg.stop();
    rg.clear();
    EXPECT_FALSE(rg.latest(out));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
