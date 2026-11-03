# test/test_grid_map_postprocess.cpp
#include <gtest/gtest.h>
#include <vector>

#include "algorithms/grid_map/grid_map_postprocess.h"

using namespace rosiwit_slam;

namespace {

GridInfo makeGrid(int w, int h, int8_t v) {
    GridInfo g;
    g.width = w;
    g.height = h;
    g.data.assign(static_cast<size_t>(w) * h, v);
    return g;
}

void setCell(GridInfo& g, int x, int y, int8_t v) {
    g.data[static_cast<size_t>(y) * g.width + x] = v;
}

int getCell(const GridInfo& g, int x, int y) {
    return g.data[static_cast<size_t>(y) * g.width + x];
}

GridPostConfig unknownFreeCfg() {
    GridPostConfig c;
    c.pipeline = {"unknown_free"};
    c.unknown_free_max_cells = 30;
    return c;
}

}  // namespace

// 自由地面里的小块 unknown -> free
TEST(GridPostProcessUnknownFree, ClearsFreeSurroundedSpeckle) {
    GridInfo g = makeGrid(20, 20, 0);
    setCell(g, 8, 8, -1);
    setCell(g, 9, 8, -1);
    setCell(g, 8, 9, -1);
    setCell(g, 9, 9, -1);
    applyGridPostProcess(g, unknownFreeCfg());
    EXPECT_EQ(getCell(g, 8, 8), 0);
    EXPECT_EQ(getCell(g, 9, 9), 0);
}

// 紧挨墙脚但大部分邻域是 free 的散点 -> 清掉 (地面噪点)
TEST(GridPostProcessUnknownFree, ClearsSpeckleNextToWall) {
    GridInfo g = makeGrid(20, 20, 0);
    setCell(g, 8, 8, -1);
    setCell(g, 9, 8, -1);
    setCell(g, 9, 9, 100);
    applyGridPostProcess(g, unknownFreeCfg());
    EXPECT_EQ(getCell(g, 8, 8), 0);
}

// 两道墙之间的细窄 unknown 条 (墙后阴影) -> 保留
TEST(GridPostProcessUnknownFree, KeepsThinUnknownStripBetweenWalls) {
    GridInfo g = makeGrid(20, 20, 0);
    for (int y = 5; y <= 9; ++y) {
        setCell(g, 7, y, 100);
        setCell(g, 9, y, 100);
        setCell(g, 8, y, -1);
    }
    applyGridPostProcess(g, unknownFreeCfg());
    EXPECT_EQ(getCell(g, 8, 7), -1);
}

TEST(GridPostProcessUnknownFree, KeepsBorderUnknown) {
    GridInfo g = makeGrid(20, 20, 0);
    setCell(g, 0, 5, -1);
    setCell(g, 1, 5, -1);
    applyGridPostProcess(g, unknownFreeCfg());
    EXPECT_EQ(getCell(g, 0, 5), -1);
}

TEST(GridPostProcessUnknownFree, KeepsLargeUnknown) {
    GridInfo g = makeGrid(20, 20, 0);
    for (int y = 5; y < 15; ++y)
        for (int x = 5; x < 15; ++x) setCell(g, x, y, -1);
    applyGridPostProcess(g, unknownFreeCfg());
    EXPECT_EQ(getCell(g, 10, 10), -1);
}

TEST(GridPostProcessUnknownFree, KeepsUnknownInsideUnknown) {
    GridInfo g = makeGrid(20, 20, -1);
    setCell(g, 9, 9, 0);
    applyGridPostProcess(g, unknownFreeCfg());
    EXPECT_EQ(getCell(g, 8, 8), -1);
}

TEST(GridPostProcessUnknownFree, DisabledKeepsUnknown) {
    GridInfo g = makeGrid(20, 20, 0);
    setCell(g, 8, 8, -1);
    GridPostConfig c = unknownFreeCfg();
    c.unknown_free_max_cells = 0;
    applyGridPostProcess(g, c);
    EXPECT_EQ(getCell(g, 8, 8), -1);
}
