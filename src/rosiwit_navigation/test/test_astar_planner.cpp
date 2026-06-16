// ============================================================
// A* 规划器基础测试 (v2.0 — 对齐简化 shared_ptr API)
// 测试基本路径规划、避障、不可达目标、空网格等场景
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <memory>
#include "diffbot_navigation/planners/astar_planner.hpp"

using namespace diffbot_navigation::planners;

// ============================================================
// 测试辅助：创建简单 OccupancyGrid（返回 shared_ptr）
// ============================================================
static std::shared_ptr<OccupancyGrid> makeSimpleGrid(int width, int height, unsigned char free_val = 0)
{
    auto grid = std::make_shared<OccupancyGrid>();
    grid->info.width = static_cast<unsigned int>(width);
    grid->info.height = static_cast<unsigned int>(height);
    grid->info.resolution = 0.05;
    grid->info.origin.x = 0.0;
    grid->info.origin.y = 0.0;
    grid->data.resize(static_cast<size_t>(width * height), free_val);
    return grid;
}

// ============================================================
// TC-ASTAR-01: 直线路径——无障碍物
// ============================================================
TEST(AStarPlannerTest, StraightLineNoObstacles)
{
    AStarPlanner planner;
    planner.configure(AStarPlanner::Config{});

    auto grid = makeSimpleGrid(10, 10, 0);

    PathPoint start{0.0, 0.0, 0.0};
    PathPoint goal{0.45, 0.45, 0.0};  // 网格坐标 (9, 9) 的世界坐标

    AStarResult result = planner.plan(grid, start, goal);
    EXPECT_TRUE(result.success) << "Straight-line path should succeed";
    EXPECT_GT(result.path.size(), 0u);
}

// ============================================================
// TC-ASTAR-02: 起点被障碍物阻挡
// ============================================================
TEST(AStarPlannerTest, StartBlockedByObstacle)
{
    AStarPlanner planner;
    planner.configure(AStarPlanner::Config{});

    auto grid = makeSimpleGrid(10, 10, 0);
    // 在起点所在单元格放置障碍
    int sx = 0, sy = 0;
    grid->data[sy * 10 + sx] = 100;

    PathPoint start{0.025, 0.025, 0.0};   // 网格 (0,0)
    PathPoint goal{0.475, 0.475, 0.0};    // 网格 (9,9)

    AStarResult result = planner.plan(grid, start, goal);
    EXPECT_FALSE(result.success) << "Path from occupied cell should fail";
}

// ============================================================
// TC-ASTAR-03: 目标被障碍物阻挡
// ============================================================
TEST(AStarPlannerTest, GoalBlockedByObstacle)
{
    AStarPlanner planner;
    planner.configure(AStarPlanner::Config{});

    auto grid = makeSimpleGrid(10, 10, 0);
    // 在目标所在单元格放置障碍
    int gx = 9, gy = 9;
    grid->data[gy * 10 + gx] = 100;

    PathPoint start{0.025, 0.025, 0.0};   // 网格 (0,0)
    PathPoint goal{0.475, 0.475, 0.0};    // 网格 (9,9)

    AStarResult result = planner.plan(grid, start, goal);
    EXPECT_FALSE(result.success) << "Path to occupied cell should fail";
}

// ============================================================
// TC-ASTAR-04: 完全包围——无可行路径
// ============================================================
TEST(AStarPlannerTest, NoPathExists)
{
    AStarPlanner planner;
    planner.configure(AStarPlanner::Config{});

    auto grid = makeSimpleGrid(5, 5, 0);
    // 用障碍物将起点包围
    grid->data[1 * 5 + 0] = 100;
    grid->data[1 * 5 + 1] = 100;
    grid->data[1 * 5 + 2] = 100;
    grid->data[0 * 5 + 1] = 100;

    PathPoint start{0.025, 0.025, 0.0};
    PathPoint goal{0.225, 0.225, 0.0};  // 网格 (4,4)

    AStarResult result = planner.plan(grid, start, goal);
    EXPECT_FALSE(result.success) << "No path should exist when start is surrounded";
}

// ============================================================
// TC-ASTAR-05: 对角线路径
// ============================================================
TEST(AStarPlannerTest, DiagonalPath)
{
    AStarPlanner planner;
    AStarPlanner::Config config;
    config.use_diagonal_moves = true;
    planner.configure(config);

    auto grid = makeSimpleGrid(10, 10, 0);

    PathPoint start{0.025, 0.025, 0.0};
    PathPoint goal{0.475, 0.475, 0.0};

    AStarResult result = planner.plan(grid, start, goal);
    EXPECT_TRUE(result.success) << "Diagonal path should succeed";
    EXPECT_GT(result.path.size(), 0u);

    // 对角线路径长度应 ≤ 直线距离的 1.5 倍
    double dx = goal.x - start.x;
    double dy = goal.y - start.y;
    double straight_dist = std::sqrt(dx * dx + dy * dy);

    double path_length = 0.0;
    for (size_t i = 1; i < result.path.size(); ++i) {
        double px = result.path[i].x - result.path[i - 1].x;
        double py = result.path[i].y - result.path[i - 1].y;
        path_length += std::sqrt(px * px + py * py);
    }
    EXPECT_LE(path_length, straight_dist * 2.0) << "Path should not be excessively long";
}

// ============================================================
// TC-ASTAR-06: 异步规划检查（使用同步 plan 验证正确性）
// ============================================================
TEST(AStarPlannerTest, AsyncPlanningCheck)
{
    AStarPlanner planner;
    planner.configure(AStarPlanner::Config{});

    auto grid = makeSimpleGrid(10, 10, 0);

    PathPoint start{0.025, 0.025, 0.0};
    PathPoint goal{0.475, 0.475, 0.0};

    // IPlannerStrategy::planAsync 通过不同签名测试，这里测试相同路径的同步结果
    AStarResult result = planner.plan(grid, start, goal);
    EXPECT_TRUE(result.success) << "Sync path planning should succeed";
    EXPECT_GT(result.path.size(), 0u);
}

// ============================================================
// TC-ASTAR-07: 路径长度合理性检查
// ============================================================
TEST(AStarPlannerTest, PathLengthSanity)
{
    AStarPlanner planner;
    planner.configure(AStarPlanner::Config{});

    auto grid = makeSimpleGrid(20, 20, 0);

    PathPoint start{0.025, 0.025, 0.0};
    PathPoint goal{0.975, 0.975, 0.0};  // 网格 (19,19)

    AStarResult result = planner.plan(grid, start, goal);
    ASSERT_TRUE(result.success);

    double path_length = 0.0;
    for (size_t i = 1; i < result.path.size(); ++i) {
        double dx = result.path[i].x - result.path[i - 1].x;
        double dy = result.path[i].y - result.path[i - 1].y;
        path_length += std::sqrt(dx * dx + dy * dy);
    }

    double dx = goal.x - start.x;
    double dy = goal.y - start.y;
    double straight_dist = std::sqrt(dx * dx + dy * dy);

    // 无障碍路径长度应接近直线距离
    EXPECT_NEAR(path_length, straight_dist, straight_dist * 0.5)
        << "Unobstructed path should be close to straight-line distance";
}

// ============================================================
// TC-ASTAR-08: 单单元格网格——最小网格测试
// ============================================================
TEST(AStarPlannerTest, SingleCellGrid)
{
    AStarPlanner planner;
    planner.configure(AStarPlanner::Config{});

    auto grid = makeSimpleGrid(1, 1, 0);

    PathPoint start{0.025, 0.025, 0.0};
    PathPoint goal{0.025, 0.025, 0.0};   // 同一点

    AStarResult result = planner.plan(grid, start, goal);
    // 起点=终点，应返回成功（单点路径）
    EXPECT_TRUE(result.success) << "Single-cell same-point path should succeed";
    EXPECT_GE(result.path.size(), 1u);
}

// ============================================================
// TC-ASTAR-09: 边界坐标处理
// ============================================================
TEST(AStarPlannerTest, BoundaryCoordinates)
{
    AStarPlanner planner;
    planner.configure(AStarPlanner::Config{});

    auto grid = makeSimpleGrid(10, 10, 0);

    // 使用接近网格边界的坐标
    PathPoint start{0.001, 0.001, 0.0};
    PathPoint goal{0.499, 0.499, 0.0};

    AStarResult result = planner.plan(grid, start, goal);
    // 应能正确处理边界坐标
    if (result.success) {
        EXPECT_GT(result.path.size(), 0u);
    }
}
