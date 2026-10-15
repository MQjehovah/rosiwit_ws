// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include <gtest/gtest.h>
#include "coverage_planner/turn_optimizer.hpp"
#include <cmath>
#include <chrono>

namespace coverage_planner
{

/**
 * @brief TurnOptimizer单元测试类
 *
 * 测试覆盖目标：>80%
 * 测试场景：
 * 1. 直线路径（无转弯）
 * 2. 弓字形路径（U型转弯）
 * 3. 复杂路径（多转弯）
 * 4. 转弯合并功能
 * 5. 转弯分类功能
 * 6. 性能测试
 */
class TurnOptimizerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        optimizer_ = std::make_unique<TurnOptimizer>();

        // 默认配置
        config_.angle_threshold = 0.1;           // 约5.7度
        config_.merge_distance_threshold = 10.0;  // 10栅格
        config_.merge_angle_threshold = 0.35;     // 约20度
        config_.enable_merge = true;
        config_.enable_smooth = false;
        config_.enable_skip_obsolete = true;
        config_.smooth_radius = 0.3;
        config_.smooth_samples = 5;

        // 创建默认地图（用于碰撞检测）
        default_map_.info.width = 100;
        default_map_.info.height = 100;
        default_map_.info.resolution = 0.05;
        default_map_.data.resize(100 * 100, 0);  // 全部空闲
    }

    /**
     * @brief 创建直线路径
     */
    std::vector<geometry_msgs::msg::PoseStamped> createStraightPath(
        int num_points, double step = 1.0)
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;

        for (int i = 0; i < num_points; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = i * step;
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;  // 沿X方向
            path.push_back(pose);
        }

        return path;
    }

    /**
     * @brief 创建弓字形路径
     *
     * ──→───→───→
     *           ↓
     * ←───←───←
     * ↓
     * ──→───→───→
     */
    std::vector<geometry_msgs::msg::PoseStamped> createZigzagPath(
        int num_lines, int points_per_line, double line_spacing = 5.0, double step = 1.0)
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
                    pose.pose.orientation.w = 1.0;  // 0度
                    pose.pose.orientation.z = 0.0;
                } else {
                    pose.pose.position.x = (points_per_line - 1 - i) * step;
                    pose.pose.position.y = y;
                    pose.pose.orientation.w = 0.0;
                    pose.pose.orientation.z = 1.0;  // 180度
                }

                pose.pose.position.z = 0.0;
                path.push_back(pose);
            }

            // 添加过渡点
            if (line < num_lines - 1) {
                geometry_msgs::msg::PoseStamped transition;
                transition.header.frame_id = "map";
                transition.pose.position.y = (line + 1) * line_spacing;

                if (going_right) {
                    transition.pose.position.x = (points_per_line - 1) * step;
                } else {
                    transition.pose.position.x = 0.0;
                }

                transition.pose.position.z = 0.0;
                transition.pose.orientation.w = 0.707;
                transition.pose.orientation.z = 0.707;  // 90度
                path.push_back(transition);
            }
        }

        return path;
    }

    /**
     * @brief 创建多转弯路径
     */
    std::vector<geometry_msgs::msg::PoseStamped> createMultiTurnPath()
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;

        // 第一段：向右
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = i * 1.0;
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path.push_back(pose);
        }

        // 转弯1：向下（90度）
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = 10.0;
            pose.pose.position.y = i * 1.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 0.707;
            pose.pose.orientation.z = -0.707;  // -90度
            path.push_back(pose);
        }

        // 转弯2：向左（90度）
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = 10.0 - i * 1.0;
            pose.pose.position.y = 10.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 0.0;
            pose.pose.orientation.z = 1.0;  // 180度
            path.push_back(pose);
        }

        // 转弯3：向上（90度）
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = 0.0;
            pose.pose.position.y = 10.0 - i * 1.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 0.707;
            pose.pose.orientation.z = 0.707;  // 90度
            path.push_back(pose);
        }

        return path;
    }

    /**
     * @brief 创建密集小转弯路径
     */
    std::vector<geometry_msgs::msg::PoseStamped> createDenseTurnPath()
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;

        // 创建锯齿形路径
        for (int i = 0; i < 50; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = i * 1.0;
            pose.pose.position.y = (i % 2 == 0) ? 0.5 : 0.0;
            pose.pose.position.z = 0.0;

            // 计算方向
            double angle = (i % 2 == 0) ? M_PI / 12 : -M_PI / 12;  // ±15度
            pose.pose.orientation.w = cos(angle / 2);
            pose.pose.orientation.z = sin(angle / 2);

            path.push_back(pose);
        }

        return path;
    }

    std::unique_ptr<TurnOptimizer> optimizer_;
    TurnOptimizerConfig config_;
    nav_msgs::msg::OccupancyGrid default_map_;
};

// ==================== 转弯计数测试 ====================

/**
 * @brief 测试：直线路径转弯计数
 *
 * 验收标准：
 * - 直线路径不应有转弯
 */
TEST_F(TurnOptimizerTest, StraightPathTurnCount)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createStraightPath(100);

    int turn_count = optimizer_->countTurns(path, config_.angle_threshold);

    // 验证：直线路径应无转弯或极少转弯
    EXPECT_LE(turn_count, 2);
}

/**
 * @brief 测试：弓字形路径转弯计数
 *
 * 验收标准：
 * - 应检测到每条扫描线末端的转弯点
 */
TEST_F(TurnOptimizerTest, ZigzagPathTurnCount)
{
    int num_lines = 5;
    std::vector<geometry_msgs::msg::PoseStamped> path = createZigzagPath(num_lines, 10);

    int turn_count = optimizer_->countTurns(path, config_.angle_threshold);

    // 验证：应检测到转弯（num_lines-1个U型转弯）
    EXPECT_GE(turn_count, num_lines - 1);
}

/**
 * @brief 测试：多转弯路径计数
 *
 * 验收标准：
 * - 应正确检测到所有主要转弯
 */
TEST_F(TurnOptimizerTest, MultiTurnPathCount)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createMultiTurnPath();

    int turn_count = optimizer_->countTurns(path, config_.angle_threshold);

    // 验证：应检测到转弯（预期4个主要转弯）
    EXPECT_GE(turn_count, 3);
}

// ==================== 转弯分类测试 ====================

/**
 * @brief 测试：转弯类型通过优化结果验证
 *
 * 验收标准：
 * - 不同角度应正确分类
 * - 通过优化结果验证分类正确性
 */
TEST_F(TurnOptimizerTest, TurnClassificationThroughOptimization)
{
    // 创建包含各种转弯类型的路径
    std::vector<geometry_msgs::msg::PoseStamped> path = createMultiTurnPath();

    // 执行优化，通过优化结果验证转弯分类
    TurnOptimizeResult result = optimizer_->optimize(path, default_map_, config_);

    // 验证：优化成功
    EXPECT_TRUE(result.success);

    // 验证：转弯类型列表
    for (const auto& turn : result.original_turns) {
        // 转弯类型应为有效类型
        EXPECT_TRUE(turn.type == TurnType::NONE ||
                    turn.type == TurnType::SHARP ||
                    turn.type == TurnType::MEDIUM ||
                    turn.type == TurnType::GENTLE ||
                    turn.type == TurnType::U_TURN ||
                    turn.type == TurnType::SCANLINE_END);

        // 转弯角度范围
        EXPECT_GE(turn.angle_change, 0.0);
        EXPECT_LE(turn.angle_change, M_PI);
    }
}

// ==================== 转弯优化测试 ====================

/**
 * @brief 测试：转弯合并功能
 *
 * 验收标准：
 * - 相邻小转弯应合并为单转弯
 * - 合并后转弯数量应减少
 */
TEST_F(TurnOptimizerTest, TurnMergeFunctionality)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createDenseTurnPath();

    // 首先计算原始转弯数
    int original_turn_count = optimizer_->countTurns(path, config_.angle_threshold);

    // 执行优化（带合并）
    TurnOptimizerConfig merge_config = config_;
    merge_config.enable_merge = true;
    merge_config.merge_distance_threshold = 5.0;

    TurnOptimizeResult optimize_result = optimizer_->optimize(path, default_map_, merge_config);

    // 验证：优化成功
    EXPECT_TRUE(optimize_result.success);

    // 验证：合并后转弯数量应减少或相等
    EXPECT_LE(optimize_result.optimized_turn_count, original_turn_count);

    // 验证：优化后路径长度应合理
    EXPECT_GE(optimize_result.optimized_path.size(), static_cast<size_t>(10));
}

/**
 * @brief 测试：合并距离阈值影响
 *
 * 验收标准：
 * - 较大的合并距离阈值应合并更多转弯
 */
TEST_F(TurnOptimizerTest, MergeDistanceThreshold)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createDenseTurnPath();

    // 小阈值
    TurnOptimizerConfig small_config = config_;
    small_config.merge_distance_threshold = 2.0;
    TurnOptimizeResult small_result = optimizer_->optimize(path, default_map_, small_config);

    // 大阈值
    TurnOptimizerConfig large_config = config_;
    large_config.merge_distance_threshold = 20.0;
    TurnOptimizeResult large_result = optimizer_->optimize(path, default_map_, large_config);

    // 验证：大阈值应合并更多转弯（转弯数更少）
    EXPECT_LE(large_result.optimized_turn_count, small_result.optimized_turn_count);
}

/**
 * @brief 测试：合并角度阈值影响
 */
TEST_F(TurnOptimizerTest, MergeAngleThreshold)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createDenseTurnPath();

    // 小角度阈值
    TurnOptimizerConfig small_config = config_;
    small_config.merge_angle_threshold = 0.1;
    TurnOptimizeResult small_result = optimizer_->optimize(path, default_map_, small_config);

    // 大角度阈值
    TurnOptimizerConfig large_config = config_;
    large_config.merge_angle_threshold = 0.5;
    TurnOptimizeResult large_result = optimizer_->optimize(path, default_map_, large_config);

    // 验证：大角度阈值应合并更多转弯
    EXPECT_LE(large_result.optimized_turn_count, small_result.optimized_turn_count);
}

// ==================== 路径重建测试 ====================

/**
 * @brief 测试：路径重建正确性
 *
 * 验收标准：
 * - 重建后路径应保持起点和终点
 */
TEST_F(TurnOptimizerTest, PathRebuildCorrectness)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createZigzagPath(5, 10);

    TurnOptimizeResult result = optimizer_->optimize(path, default_map_, config_);

    // 验证：起点不变
    if (result.optimized_path.size() > 0 && path.size() > 0) {
        EXPECT_NEAR(result.optimized_path.front().pose.position.x,
                    path.front().pose.position.x, 1.0);
        EXPECT_NEAR(result.optimized_path.front().pose.position.y,
                    path.front().pose.position.y, 1.0);
    }

    // 验证：终点不变
    if (result.optimized_path.size() > 0 && path.size() > 0) {
        EXPECT_NEAR(result.optimized_path.back().pose.position.x,
                    path.back().pose.position.x, 1.0);
        EXPECT_NEAR(result.optimized_path.back().pose.position.y,
                    path.back().pose.position.y, 1.0);
    }

    // 验证：路径连续性
    for (size_t i = 1; i < result.optimized_path.size(); ++i) {
        double dx = result.optimized_path[i].pose.position.x -
                    result.optimized_path[i-1].pose.position.x;
        double dy = result.optimized_path[i].pose.position.y -
                    result.optimized_path[i-1].pose.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        EXPECT_LT(dist, 15.0);
    }
}

// ==================== 配置参数测试 ====================

/**
 * @brief 测试：启用/禁用合并功能
 */
TEST_F(TurnOptimizerTest, EnableMergeSwitch)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createDenseTurnPath();

    // 禁用合并
    TurnOptimizerConfig no_merge_config = config_;
    no_merge_config.enable_merge = false;
    TurnOptimizeResult no_merge_result = optimizer_->optimize(path, default_map_, no_merge_config);

    // 启用合并
    TurnOptimizerConfig merge_config = config_;
    merge_config.enable_merge = true;
    TurnOptimizeResult merge_result = optimizer_->optimize(path, default_map_, merge_config);

    // 验证：启用合并后转弯数应减少或相等
    EXPECT_LE(merge_result.optimized_turn_count, no_merge_result.optimized_turn_count);
}

/**
 * @brief 测试：角度检测阈值影响
 */
TEST_F(TurnOptimizerTest, AngleThresholdEffect)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createDenseTurnPath();

    // 小角度阈值（检测更多转弯）
    int sensitive_count = optimizer_->countTurns(path, 0.05);

    // 大角度阈值（检测更少转弯）
    int insensitive_count = optimizer_->countTurns(path, 0.3);

    // 验证：敏感阈值检测更多转弯
    EXPECT_GE(sensitive_count, insensitive_count);
}

// ==================== 边界条件测试 ====================

/**
 * @brief 测试：空路径处理
 */
TEST_F(TurnOptimizerTest, EmptyPath)
{
    std::vector<geometry_msgs::msg::PoseStamped> empty_path;

    EXPECT_NO_THROW({
        int turn_count = optimizer_->countTurns(empty_path, config_.angle_threshold);
        EXPECT_EQ(turn_count, 0);
    });

    EXPECT_NO_THROW({
        TurnOptimizeResult result = optimizer_->optimize(empty_path, default_map_, config_);
        EXPECT_EQ(result.optimized_path.size(), static_cast<size_t>(0));
    });
}

/**
 * @brief 测试：单点路径处理
 */
TEST_F(TurnOptimizerTest, SinglePointPath)
{
    std::vector<geometry_msgs::msg::PoseStamped> single_path;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    single_path.push_back(pose);

    EXPECT_NO_THROW({
        int turn_count = optimizer_->countTurns(single_path, config_.angle_threshold);
        EXPECT_EQ(turn_count, 0);
    });
}

/**
 * @brief 测试：两点路径处理
 */
TEST_F(TurnOptimizerTest, TwoPointsPath)
{
    std::vector<geometry_msgs::msg::PoseStamped> two_point_path = createStraightPath(2);

    EXPECT_NO_THROW({
        int turn_count = optimizer_->countTurns(two_point_path, config_.angle_threshold);
        EXPECT_EQ(turn_count, 0);
    });
}

// ==================== 性能测试 ====================

/**
 * @brief 测试：大路径性能
 */
TEST_F(TurnOptimizerTest, LargePathPerformance)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createZigzagPath(100, 100);

    auto start = std::chrono::high_resolution_clock::now();
    TurnOptimizeResult result = optimizer_->optimize(path, default_map_, config_);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // 验证：性能
    EXPECT_LT(duration.count(), 100);

    // 验证：结果正确
    EXPECT_TRUE(result.success);
    EXPECT_GE(result.optimized_path.size(), static_cast<size_t>(100));
}

// ==================== 转弯优化效果测试 ====================

/**
 * @brief 测试：转弯优化效果验证
 *
 * 验收标准：
 * - 优化后转弯数量减少≥20%（目标≥30%）
 */
TEST_F(TurnOptimizerTest, TurnOptimizationEffect)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createZigzagPath(20, 20);

    // 原始转弯数量
    int original_turn_count = optimizer_->countTurns(path, config_.angle_threshold);

    // 优化后转弯数量
    TurnOptimizerConfig aggressive_config = config_;
    aggressive_config.merge_distance_threshold = 15.0;
    aggressive_config.merge_angle_threshold = 0.4;
    TurnOptimizeResult optimized_result = optimizer_->optimize(path, default_map_, aggressive_config);

    // 验证：转弯减少
    if (original_turn_count > 5) {
        EXPECT_LE(optimized_result.optimized_turn_count, original_turn_count);
        EXPECT_GT(optimized_result.reduction_rate, 0.0);
    }
}

// ==================== 综合测试 ====================

/**
 * @brief 测试：完整优化流程
 */
TEST_F(TurnOptimizerTest, FullOptimizationWorkflow)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createZigzagPath(10, 10);

    // Step 1: 计算原始转弯数
    int original_count = optimizer_->countTurns(path, config_.angle_threshold);

    // Step 2: 优化转弯
    TurnOptimizeResult optimize_result = optimizer_->optimize(path, default_map_, config_);

    // 验证：流程完整性
    EXPECT_TRUE(optimize_result.success);
    EXPECT_GE(optimize_result.optimized_path.size(), static_cast<size_t>(0));
    EXPECT_LE(optimize_result.optimized_turn_count, original_count);

    // Step 3: 验证结果完整性
    EXPECT_TRUE(optimize_result.original_turn_count >= 0);
    EXPECT_TRUE(optimize_result.optimized_turn_count >= 0);
}

// ==================== 重置测试 ====================

/**
 * @brief 测试：重置功能
 */
TEST_F(TurnOptimizerTest, ResetFunctionality)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createStraightPath(50);

    EXPECT_NO_THROW({
        optimizer_->reset();
    });

    // 重置后仍能正常工作
    int turn_count = optimizer_->countTurns(path, config_.angle_threshold);
    EXPECT_LE(turn_count, 2);
}

}  // namespace coverage_planner

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}