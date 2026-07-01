// ============================================================
// A* 规划器大网格性能与边界测试
// 对应需求: requirements.md § A*大网格性能优化
// 对应架构: architecture.md § 3.2 AStarPlanner性能增强
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <chrono>
#include <vector>
#include "diffbot_navigation/planners/astar_planner.hpp"
#include "diffbot_navigation/core/types.hpp"

using namespace diffbot_navigation::planners;

// ============================================================
// Fixture: AStar 规划器测试环境
// ============================================================
class AStarPlannerPerfTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    planner_ = std::make_unique<AStarPlanner>();
    AStarPlanner::Config config;
    config.grid_resolution = 0.05;         // 5cm 分辨率
    config.max_iterations = 50000;         // 降低到 5万 以满足性能要求
    config.timeout_seconds = 2.0;          // 2秒超时
    config.heuristic_weight = 1.2;         // 轻微权重A*
    config.use_diagonal_moves = true;
    config.obstacle_cost_threshold = 100;
    planner_->configure(config);
  }

  // 创建指定大小的测试网格（无遮挡）
  std::shared_ptr<OccupancyGrid> createEmptyGrid(int width, int height)
  {
    auto grid = std::make_shared<OccupancyGrid>();
    grid->info.width = width;
    grid->info.height = height;
    grid->info.resolution = 0.05;
    grid->info.origin.x = 0.0;
    grid->info.origin.y = 0.0;
    grid->data.resize(width * height, 0);  // 全部空闲
    return grid;
  }

  // 创建有绕行障碍的网格
  std::shared_ptr<OccupancyGrid> createObstacleGrid(int width, int height)
  {
    auto grid = std::make_shared<OccupancyGrid>();
    grid->info.width = width;
    grid->info.height = height;
    grid->info.resolution = 0.05;
    grid->info.origin.x = 0.0;
    grid->info.origin.y = 0.0;
    grid->data.resize(width * height, 0);

    // 在中间添加一个障碍墙（迫使绕行）
    for (int y = height/4; y < 3*height/4; ++y) {
      grid->data[y * width + width/2] = 100;  // 障碍
    }
    return grid;
  }

  std::unique_ptr<AStarPlanner> planner_;
};

// ============================================================
// TC-ASTAR-01: 小网格基准性能（100x100）
// ============================================================
TEST_F(AStarPlannerPerfTest, SmallGridBenchmark)
{
  auto grid = createEmptyGrid(100, 100);
  PathPoint start{0.0, 0.0, 0.0};
  PathPoint goal{4.5, 4.5, 0.0};

  auto t0 = std::chrono::steady_clock::now();
  auto result = planner_->plan(grid, start, goal);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();

  EXPECT_TRUE(result.success);
  EXPECT_GT(result.path.size(), 1u);
  EXPECT_LT(elapsed, 100) << "100x100 grid: " << elapsed << "ms, expected < 100ms";
}

// ============================================================
// TC-ASTAR-02: 中等网格性能（500x500）
// ============================================================
TEST_F(AStarPlannerPerfTest, MediumGridBenchmark)
{
  auto grid = createEmptyGrid(500, 500);
  PathPoint start{0.0, 0.0, 0.0};
  PathPoint goal{24.0, 24.0, 0.0};

  auto t0 = std::chrono::steady_clock::now();
  auto result = planner_->plan(grid, start, goal);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();

  EXPECT_TRUE(result.success);
  EXPECT_GT(result.path.size(), 1u);
  EXPECT_LT(elapsed, 1000) << "500x500 grid: " << elapsed << "ms, expected < 1000ms";
}

// ============================================================
// TC-ASTAR-03: 大网格性能（1000x1000）—— 核心性能测试
// 期望：2秒内完成
// ============================================================
TEST_F(AStarPlannerPerfTest, LargeGridPerformance1000x1000)
{
  auto grid = createEmptyGrid(1000, 1000);
  PathPoint start{0.0, 0.0, 0.0};
  PathPoint goal{49.0, 49.0, 0.0};  // 接近边界

  auto t0 = std::chrono::steady_clock::now();
  auto result = planner_->plan(grid, start, goal);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();

  EXPECT_TRUE(result.success) << "Large grid planning should succeed";
  EXPECT_GT(result.path.size(), 1u) << "Should find a path";

  // 关键性能指标：2秒内完成
  EXPECT_LT(elapsed, 2000)
    << "1000x1000 grid: " << elapsed << "ms — EXCEEDS 2s timeout!";

  // 验证迭代次数不超过限制
  EXPECT_LE(result.iterations, 50000)
    << "Iteration count " << result.iterations << " exceeds max_iterations 50000";
}

// ============================================================
// TC-ASTAR-04: 大网格绕障性能
// ============================================================
TEST_F(AStarPlannerPerfTest, LargeGridWithObstacles)
{
  auto grid = createObstacleGrid(1000, 1000);
  PathPoint start{0.0, 24.5, 0.0};
  PathPoint goal{49.0, 24.5, 0.0};  // 需绕过中间墙

  auto t0 = std::chrono::steady_clock::now();
  auto result = planner_->plan(grid, start, goal);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();

  EXPECT_TRUE(result.success) << "Should find path even with obstacles";
  EXPECT_LT(elapsed, 2000)
    << "1000x1000 grid with obstacles: " << elapsed << "ms — EXCEEDS 2s!";
}

// ============================================================
// TC-ASTAR-05: 超时保护验证
// 期望：不可达目标下，达到迭代上限或超时后应优雅退出
// ============================================================
TEST_F(AStarPlannerPerfTest, TimeoutProtectionOnUnreachableGoal)
{
  // 创建被障碍完全包围的目标
  auto grid = std::make_shared<OccupancyGrid>();
  grid->info.width = 200;
  grid->info.height = 200;
  grid->info.resolution = 0.05;
  grid->info.origin.x = 0.0;
  grid->info.origin.y = 0.0;
  grid->data.resize(200 * 200, 0);

  // 用障碍完全包围目标区域
  int cx = 190, cy = 190;
  for (int dx = -2; dx <= 2; ++dx) {
    for (int dy = -2; dy <= 2; ++dy) {
      int idx = (cy + dy) * 200 + (cx + dx);
      if (idx >= 0 && idx < static_cast<int>(grid->data.size())) {
        grid->data[idx] = 100;
      }
    }
  }

  PathPoint start{0.0, 0.0, 0.0};
  PathPoint goal{9.5, 9.5, 0.0};  // 在障碍包围中

  auto t0 = std::chrono::steady_clock::now();
  auto result = planner_->plan(grid, start, goal);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();

  // 不应无限循环
  EXPECT_LT(elapsed, 3000) << "Timeout protection must work, took " << elapsed << "ms";

  // 不可达时应返回失败
  EXPECT_FALSE(result.success)
    << "Planning to unreachable goal should fail gracefully";
}

// ============================================================
// TC-ASTAR-06: 起点==终点（零长度规划）
// ============================================================
TEST_F(AStarPlannerPerfTest, StartEqualsGoal)
{
  auto grid = createEmptyGrid(100, 100);
  PathPoint start{2.0, 3.0, 0.0};
  PathPoint goal{2.0, 3.0, 0.0};  // 同一点

  auto result = planner_->plan(grid, start, goal);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.path.size(), 1u)
    << "Plan from start to same point should return a single-point path";
  EXPECT_NEAR(result.path[0].x, 2.0, 0.001);
  EXPECT_NEAR(result.path[0].y, 3.0, 0.001);
}

// ============================================================
// TC-ASTAR-07: 超出地图边界的起点/终点
// ============================================================
TEST_F(AStarPlannerPerfTest, OutOfBoundsPoints)
{
  auto grid = createEmptyGrid(100, 100);
  PathPoint start{-10.0, -10.0, 0.0};  // 明显超出地图
  PathPoint goal{20.0, 20.0, 0.0};     // 超出地图（100x100x0.05 = 5m范围）

  auto result = planner_->plan(grid, start, goal);
  EXPECT_FALSE(result.success)
    << "Planning with out-of-bounds points should fail gracefully";
}

// ============================================================
// TC-ASTAR-08: 极小分辨率网格（高精度）
// ============================================================
TEST_F(AStarPlannerPerfTest, HighResolutionGrid)
{
  auto grid = createEmptyGrid(200, 200);
  grid->info.resolution = 0.01;  // 1cm 精度
  PathPoint start{0.0, 0.0, 0.0};
  PathPoint goal{1.5, 1.5, 0.0};

  auto t0 = std::chrono::steady_clock::now();
  auto result = planner_->plan(grid, start, goal);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();

  EXPECT_TRUE(result.success);
  EXPECT_LT(elapsed, 500)
    << "High-res planning: " << elapsed << "ms, expected < 500ms";
}

// ============================================================
// TC-ASTAR-09: 空网格 (>nullptr) 容错
// ============================================================
TEST_F(AStarPlannerPerfTest, NullGridHandling)
{
  PathPoint start{0.0, 0.0, 0.0};
  PathPoint goal{1.0, 0.0, 0.0};

  auto result = planner_->plan(nullptr, start, goal);
  EXPECT_FALSE(result.success)
    << "Planning with nullptr grid should fail gracefully";
}

// ============================================================
// TC-ASTAR-10: 连续规划：缓存/状态重置验证
// ============================================================
TEST_F(AStarPlannerPerfTest, ConsecutivePlanningNoDegradation)
{
  auto grid = createEmptyGrid(200, 200);
  PathPoint start{0.0, 0.0, 0.0};
  PathPoint goals[] = {
    {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 1.0, 0.0},
    {1.0, 3.0, 0.0}, {5.0, 5.0, 0.0}, {8.0, 2.0, 0.0},
  };

  std::vector<long> times;
  for (const auto& goal : goals) {
    auto t0 = std::chrono::steady_clock::now();
    auto result = planner_->plan(grid, start, goal);
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count();

    EXPECT_TRUE(result.success) << "Consecutive plan must succeed";
    times.push_back(elapsed);
  }

  // 连续规划不应性能退化（后续规划不应显著慢于首次）
  if (times.size() >= 2) {
    double avg = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    EXPECT_LT(times.back(), avg * 3.0)
      << "Last plan " << times.back() << "ms is significantly slower than avg "
      << avg << "ms — possible state leak";
  }
}
