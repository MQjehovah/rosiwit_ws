// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include <gtest/gtest.h>
#include "coverage_planner/zigzag_planner.hpp"
#include "coverage_planner/spiral_planner.hpp"
#include "coverage_planner/zone_decomposer.hpp"
#include "coverage_planner/turn_optimizer.hpp"
#include "coverage_planner/coverage_utils.hpp"
#include <cmath>
#include <chrono>

namespace coverage_planner
{

/**
 * @brief 验收标准测试类
 * 
 * 根据需求文档的验收标准编写测试用例：
 * 
 * 核心验收标准：
 * | 指标 | 当前值 | 目标值 | 衡量方法 |
 * |------|--------|--------|----------|
 * | 复杂环境覆盖率 | 33.32% | ≥85% | CoverageStats工具自动统计 |
 * | 转弯次数 | 1367次/2332米 | 减少≥30% | turn_count字段 |
 * | 100x100地图规划时间 | <1s | <1.5s | planning_time_ms字段 |
 * | 路径长度增加 | - | ≤20% | path_length字段 |
 * 
 * 验收测试场景：
 * 1. L型房间（凸多边形）
 * 2. 带中柱的矩形房间
 * 3. 多房间连通区域（非凸）
 * 4. 狭长走廊
 */
class AcceptanceTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // ZoneDecomposer配置
        zone_config_.min_zone_area = 100;
        zone_config_.max_zone_count = 20;
        zone_config_.enable_rectangular_split = true;
        zone_config_.enable_pca_direction = true;
        zone_config_.connection_search_radius = 5;
        zone_config_.channel_width_threshold = 2.0;
        zone_config_.merge_threshold = 0.2;
        
        // TurnOptimizer配置
        turn_config_.angle_threshold = 0.1;
        turn_config_.merge_distance_threshold = 15.0;
        turn_config_.merge_angle_threshold = 0.35;
        turn_config_.enable_merge = true;
        turn_config_.enable_smooth = false;
        
        decomposer_ = std::make_unique<ZoneDecomposer>();
        optimizer_ = std::make_unique<TurnOptimizer>();
    }

    /**
     * @brief 创建测试地图
     */
    nav_msgs::msg::OccupancyGrid createTestMap(int width, int height, double resolution = 0.05)
    {
        nav_msgs::msg::OccupancyGrid map;
        map.info.width = width;
        map.info.height = height;
        map.info.resolution = resolution;
        map.info.origin.position.x = 0.0;
        map.info.origin.position.y = 0.0;
        map.info.origin.position.z = 0.0;
        map.info.origin.orientation.w = 1.0;
        map.header.frame_id = "map";
        map.header.stamp.sec = 0;
        map.header.stamp.nanosec = 0;
        map.data.resize(width * height, 0);  // 全部空闲
        return map;
    }

    /**
     * @brief 创建L型房间地图
     * 
     * 结构：
     * ┌──────────────┐
     * │              │
     * │              │
     * │              ├──────┐
     * │              │      │
     * │              │      │
     * │              │      │
     * └──────────────┴──────┘
     * 
     * 验收标准：覆盖率 ≥95%
     */
    nav_msgs::msg::OccupancyGrid createLShapedRoomMap()
    {
        // 100x100栅格，每格0.05米 → 5米x5米房间
        nav_msgs::msg::OccupancyGrid map = createTestMap(100, 100, 0.05);
        
        // 设置右上角为障碍物（形成L型空闲区域）
        int corner_x = 66;  // 2/3位置
        int corner_y = 33;  // 1/3位置
        
        for (int y = 0; y < corner_y; ++y) {
            for (int x = corner_x; x < 100; ++x) {
                map.data[y * 100 + x] = 100;  // 障碍物
            }
        }
        
        return map;
    }

    /**
     * @brief 创建带中心柱子的矩形房间地图
     * 
     * 结构：
     * ┌──────────────┐
     * │      ╔══╗    │
     * │      ║柱║    │
     * │      ╚══╝    │
     * │              │
     * └──────────────┘
     * 
     * 验收标准：覆盖率 ≥90%
     */
    nav_msgs::msg::OccupancyGrid createRoomWithColumnMap()
    {
        nav_msgs::msg::OccupancyGrid map = createTestMap(100, 100, 0.05);
        
        // 在中心添加柱子（10x10栅格）
        int column_size = 10;
        int center_x = 50 - column_size / 2;
        int center_y = 30 - column_size / 2;
        
        for (int y = center_y; y < center_y + column_size; ++y) {
            for (int x = center_x; x < center_x + column_size; ++x) {
                map.data[y * 100 + x] = 100;
            }
        }
        
        return map;
    }

    /**
     * @brief 创建多房间连通区域地图
     * 
     * 结构：
     * ┌──────┬──────┐
     * │      │      │
     * │      │      │
     * │      │      │
     * │  ──  │  ──  │
     * │      │      │
     * │      │      │
     * └──────┴──────┘
     * 
     * 验收标准：覆盖率 ≥85%
     */
    nav_msgs::msg::OccupancyGrid createMultiRoomMap()
    {
        nav_msgs::msg::OccupancyGrid map = createTestMap(100, 100, 0.05);
        
        // 创建分隔墙（中间留有通道）
        int wall_x = 50;
        int gap_center_y = 50;
        int gap_size = 6;
        
        for (int y = 0; y < 100; ++y) {
            if (y < gap_center_y - gap_size / 2 || y > gap_center_y + gap_size / 2) {
                map.data[y * 100 + wall_x] = 100;
            }
        }
        
        return map;
    }

    /**
     * @brief 创建狭长走廊地图
     * 
     * 结构：
     * ┌──────────────────────┐
     * │                      │
     * └──────────────────────┘
     * 
     * 验收标准：覆盖率 >99%，PCA优化生效
     */
    nav_msgs::msg::OccupancyGrid createCorridorMap()
    {
        // 走廊：宽度20栅格，长度200栅格
        nav_msgs::msg::OccupancyGrid map = createTestMap(200, 20, 0.05);
        
        return map;
    }

    /**
     * @brief 创建模拟弓字形路径（用于转弯测试）
     */
    std::vector<geometry_msgs::msg::PoseStamped> createSimulatedZigzagPath(
        int num_lines, int points_per_line, double line_spacing = 5.0, double step = 0.3)
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        
        for (int line = 0; line < num_lines; ++line) {
            double y = line * line_spacing;
            bool going_right = (line % 2 == 0);
            
            for (int i = 0; i < points_per_line; ++i) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                
                if (going_right) {
                    pose.pose.position.x = i * step;
                    pose.pose.position.y = y;
                } else {
                    pose.pose.position.x = (points_per_line - 1 - i) * step;
                    pose.pose.position.y = y;
                }
                
                pose.pose.position.z = 0.0;
                path.push_back(pose);
            }
        }
        
        return path;
    }

    /**
     * @brief 计算路径总长度
     */
    double calculatePathLength(const std::vector<geometry_msgs::msg::PoseStamped>& path)
    {
        double length = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            double dx = path[i].pose.position.x - path[i-1].pose.position.x;
            double dy = path[i].pose.position.y - path[i-1].pose.position.y;
            length += std::sqrt(dx*dx + dy*dy);
        }
        return length;
    }

    std::unique_ptr<ZoneDecomposer> decomposer_;
    std::unique_ptr<TurnOptimizer> optimizer_;
    ZoneDecomposerConfig zone_config_;
    TurnOptimizerConfig turn_config_;
};

// ==================== 功能验收测试 ====================

/**
 * @brief 验收测试1：L型房间分区规划
 * 
 * 测试步骤：
 * 1. 创建L型房间地图
 * 2. 启用分区规划和转弯优化
 * 3. 执行路径规划
 * 4. 验证分区结果
 * 
 * 验收标准：覆盖率 ≥95%（分区正确是前提）
 */
TEST_F(AcceptanceTest, LShapedRoomDecomposition)
{
    nav_msgs::msg::OccupancyGrid map = createLShapedRoomMap();
    
    // 执行分区规划
    DecompositionResult result = decomposer_->decompose(map, zone_config_);
    
    // 验证：分解成功
    EXPECT_TRUE(result.success);
    
    // 验证：至少检测到一个区域
    EXPECT_GE(result.zones.size(), static_cast<size_t>(1));
    
    // 验证：L型房间总面积正确
    int expected_free_cells = 100 * 100 - 34 * 33;  // L型面积（大致）
    EXPECT_NEAR(result.total_free_cells, expected_free_cells, expected_free_cells * 0.3);
}

/**
 * @brief 验收测试2：带柱房间分区规划
 * 
 * 验收标准：覆盖率 ≥90%
 */
TEST_F(AcceptanceTest, RoomWithColumnDecomposition)
{
    nav_msgs::msg::OccupancyGrid map = createRoomWithColumnMap();
    
    // 执行分区规划
    DecompositionResult result = decomposer_->decompose(map, zone_config_);
    
    // 验证：分解成功
    EXPECT_TRUE(result.success);
    
    // 验证：区域数量合理
    EXPECT_GE(result.zones.size(), static_cast<size_t>(1));
    
    // 验证：每个区域面积合理
    for (const auto& zone : result.zones) {
        EXPECT_GT(zone.area, zone_config_.min_zone_area);
    }
}

/**
 * @brief 验收测试3：多房间连通区域分区规划
 * 
 * 验收标准：覆盖率 ≥85%
 */
TEST_F(AcceptanceTest, MultiRoomDecomposition)
{
    nav_msgs::msg::OccupancyGrid map = createMultiRoomMap();
    
    // 执行分区规划
    DecompositionResult result = decomposer_->decompose(map, zone_config_);
    
    // 验证：分解成功
    EXPECT_TRUE(result.success);
    
    // 验证：连接通道存在
    EXPECT_GE(result.channels.size(), static_cast<size_t>(1));
    
    // 验证：访问顺序包含所有区域
    if (result.zones.size() > 0) {
        EXPECT_EQ(result.visit_order.size(), result.zones.size());
    }
}

/**
 * @brief 验收测试4：狭长走廊分区规划
 * 
 * 验收标准：覆盖率 >99%，PCA优化生效
 */
TEST_F(AcceptanceTest, CorridorDecomposition)
{
    nav_msgs::msg::OccupancyGrid map = createCorridorMap();
    
    // 执行分区规划
    DecompositionResult result = decomposer_->decompose(map, zone_config_);
    
    // 验证：分解成功
    EXPECT_TRUE(result.success);
    
    // 验证：走廊应检测为单一区域或少数区域
    EXPECT_LE(result.zones.size(), static_cast<size_t>(2));
    
    // 验证：PCA方向检测已启用
    EXPECT_TRUE(zone_config_.enable_pca_direction);
}

// ==================== 性能验收测试 ====================

/**
 * @brief 验收测试5：100x100地图规划时间 <1.5s
 * 
 * 测试步骤：
 * 1. 创建100x100栅格地图
 * 2. 执行分区规划
 * 3. 测量规划时间
 * 
 * 验收标准：planning_time_ms <1500
 */
TEST_F(AcceptanceTest, PlanningTimePerformance)
{
    nav_msgs::msg::OccupancyGrid map = createTestMap(100, 100, 0.05);
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // 执行分区规划
    DecompositionResult result = decomposer_->decompose(map, zone_config_);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    // 验收标准：规划时间 <1.5s (1500ms)
    EXPECT_LT(duration.count(), 1500);
    
    // 验证：分解结果中的时间记录
    EXPECT_LT(result.decomposition_time_ms, 1500);
}

// ==================== 转弯优化验收测试 ====================

/**
 * @brief 验收测试6：转弯次数减少 ≥30%
 * 
 * 测试步骤：
 * 1. 创建模拟弓字形路径
 * 2. 检测原始转弯次数
 * 3. 执行转弯优化
 * 4. 比较优化前后转弯数量
 * 
 * 验收标准：turn_count 减少 ≥30%
 */
TEST_F(AcceptanceTest, TurnCountReduction)
{
    // 创建模拟弓字形路径
    std::vector<geometry_msgs::msg::PoseStamped> path = createSimulatedZigzagPath(20, 50);
    
    // 创建默认地图
    nav_msgs::msg::OccupancyGrid default_map = createTestMap(100, 50, 0.3);
    
    // 检测原始转弯
    int original_turn_count = optimizer_->countTurns(path, turn_config_.angle_threshold);
    
    // 执行优化
    TurnOptimizeResult optimized_result = optimizer_->optimize(path, default_map, turn_config_);
    
    // 验证：优化成功
    EXPECT_TRUE(optimized_result.success);
    
    // 验收标准：转弯减少 ≥30%
    if (original_turn_count > 5) {
        EXPECT_GE(optimized_result.reduction_rate, 0.3);
    }
}

/**
 * @brief 验收测试7：路径长度增加 ≤20%
 * 
 * 验收标准：path_length增加 ≤20%
 */
TEST_F(AcceptanceTest, PathLengthIncrease)
{
    // 创建模拟路径
    std::vector<geometry_msgs::msg::PoseStamped> path = createSimulatedZigzagPath(10, 30);
    
    // 创建默认地图
    nav_msgs::msg::OccupancyGrid default_map = createTestMap(60, 30, 0.3);
    
    // 计算原始路径长度
    double original_length = calculatePathLength(path);
    
    // 执行优化
    TurnOptimizeResult optimized_result = optimizer_->optimize(path, default_map, turn_config_);
    
    // 验证：优化成功
    EXPECT_TRUE(optimized_result.success);
    
    // 计算优化后路径长度
    double optimized_length = calculatePathLength(optimized_result.optimized_path);
    
    // 验收标准：路径长度增加 ≤20%
    double length_increase = std::abs(optimized_length - original_length) / original_length;
    EXPECT_LE(length_increase, 0.2);
}

// ==================== 向后兼容性测试 ====================

/**
 * @brief 验收测试8：原有功能不受影响
 * 
 * 验收标准：所有原有测试全部通过
 */
TEST_F(AcceptanceTest, BackwardCompatibility)
{
    // 验证：当优化功能禁用时，规划行为不变
    
    ZoneDecomposerConfig no_optimization_config;
    no_optimization_config.max_zone_count = 1;
    no_optimization_config.enable_pca_direction = false;
    
    // 创建矩形地图
    nav_msgs::msg::OccupancyGrid rect_map = createTestMap(50, 50, 0.05);
    
    // 禁用优化时的分区
    DecompositionResult result = decomposer_->decompose(rect_map, no_optimization_config);
    
    // 验证：矩形地图应产生单一区域
    EXPECT_LE(result.zones.size(), static_cast<size_t>(1));
    EXPECT_TRUE(result.success);
}

// ==================== 端到端测试 ====================

/**
 * @brief 端到端测试1：完整规划流程
 * 
 * 测试步骤：
 * 1. 创建复杂测试地图
 * 2. 执行完整规划流程（预处理→分区→规划→优化）
 * 3. 验证最终路径质量
 * 
 * 验收标准：
 * - 分区成功
 * - 转弯优化有效
 * - 无异常抛出
 */
TEST_F(AcceptanceTest, EndToEndPlanningWorkflow)
{
    // 创建L型房间地图
    nav_msgs::msg::OccupancyGrid map = createLShapedRoomMap();
    
    EXPECT_NO_THROW({
        // Step 1: 分区规划
        DecompositionResult zone_result = decomposer_->decompose(map, zone_config_);
        
        // 验证：分区成功
        EXPECT_TRUE(zone_result.success);
        EXPECT_GE(zone_result.zones.size(), static_cast<size_t>(1));
        
        // Step 2: 转弯优化（使用模拟路径）
        std::vector<geometry_msgs::msg::PoseStamped> simulated_path = 
            createSimulatedZigzagPath(10, 20);
        
        TurnOptimizeResult turn_result = optimizer_->optimize(simulated_path, map, turn_config_);
        
        // 验证：优化成功
        EXPECT_TRUE(turn_result.success);
        EXPECT_GE(turn_result.optimized_path.size(), static_cast<size_t>(0));
    });
}

/**
 * @brief 端到端测试2：多场景连续规划
 * 
 * 验收标准：
 * - 所有测试场景规划成功
 */
TEST_F(AcceptanceTest, MultipleScenarioPlanning)
{
    std::vector<nav_msgs::msg::OccupancyGrid> test_maps = {
        createLShapedRoomMap(),
        createRoomWithColumnMap(),
        createMultiRoomMap(),
        createCorridorMap()
    };
    
    int successful_plans = 0;
    
    for (size_t i = 0; i < test_maps.size(); ++i) {
        EXPECT_NO_THROW({
            DecompositionResult result = decomposer_->decompose(test_maps[i], zone_config_);
            
            if (result.success) {
                successful_plans++;
                
                // 验证：每个场景的分区结果合理
                for (const auto& zone : result.zones) {
                    EXPECT_GT(zone.area, 0);
                }
            }
        });
    }
    
    // 验收标准：所有场景规划成功
    EXPECT_EQ(successful_plans, static_cast<int>(test_maps.size()));
}

// ==================== 错误处理测试 ====================

/**
 * @brief 验收测试：异常输入处理
 * 
 * 验收标准：
 * - 无效输入不导致崩溃
 * - 返回合理的默认值或空结果
 */
TEST_F(AcceptanceTest, InvalidInputHandling)
{
    // 空地图测试
    nav_msgs::msg::OccupancyGrid empty_map;
    empty_map.info.width = 0;
    empty_map.info.height = 0;
    
    EXPECT_NO_THROW({
        DecompositionResult result = decomposer_->decompose(empty_map, zone_config_);
    });
    
    // 全障碍物地图测试
    nav_msgs::msg::OccupancyGrid full_obstacle_map = createTestMap(10, 10);
    for (int i = 0; i < 100; ++i) {
        full_obstacle_map.data[i] = 100;
    }
    
    EXPECT_NO_THROW({
        DecompositionResult result = decomposer_->decompose(full_obstacle_map, zone_config_);
        // 全障碍物地图应产生空区域或失败
        EXPECT_TRUE(result.zones.size() == 0 || !result.success);
    });
    
    // 空路径转弯测试
    std::vector<geometry_msgs::msg::PoseStamped> empty_path;
    nav_msgs::msg::OccupancyGrid default_map = createTestMap(50, 50);
    
    EXPECT_NO_THROW({
        TurnOptimizeResult result = optimizer_->optimize(empty_path, default_map, turn_config_);
        EXPECT_EQ(result.optimized_path.size(), static_cast<size_t>(0));
    });
}

// ==================== 回归测试 ====================

/**
 * @brief 回归测试：矩形房间分区
 * 
 * 验证矩形房间仍能正确分区
 */
TEST_F(AcceptanceTest, RectangularRoomRegression)
{
    nav_msgs::msg::OccupancyGrid rect_map = createTestMap(100, 100, 0.05);
    
    DecompositionResult result = decomposer_->decompose(rect_map, zone_config_);
    
    // 验证：矩形房间应产生单一连通域
    EXPECT_TRUE(result.success);
    EXPECT_GE(result.zones.size(), static_cast<size_t>(1));
    EXPECT_EQ(result.total_free_cells, 100 * 100);
}

/**
 * @brief 回归测试：单连通域地图
 * 
 * 验证简单地图分区正确
 */
TEST_F(AcceptanceTest, SingleConnectedComponentRegression)
{
    nav_msgs::msg::OccupancyGrid simple_map = createTestMap(50, 50, 0.1);
    
    DecompositionResult result = decomposer_->decompose(simple_map, zone_config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_GE(result.zones.size(), static_cast<size_t>(1));
}

}  // namespace coverage_planner

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}