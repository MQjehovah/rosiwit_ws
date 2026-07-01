// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include <gtest/gtest.h>
#include "coverage_planner/zone_decomposer.hpp"
#include <cmath>

namespace coverage_planner
{

/**
 * @brief ZoneDecomposer单元测试类
 * 
 * 测试覆盖目标：>80%
 * 测试场景：
 * 1. 空地图分区
 * 2. 障碍物地图分区
 * 3. L型房间分区
 * 4. 多连通域地图
 * 5. 连接通道检测
 * 6. 访问顺序优化
 */
class ZoneDecomposerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        decomposer_ = std::make_unique<ZoneDecomposer>();
        
        // 默认配置
        config_.min_zone_area = 100;
        config_.enable_rectangular_split = true;
        config_.enable_pca_direction = true;
        config_.max_zone_count = 20;
        config_.connection_search_radius = 5;
        config_.channel_width_threshold = 2.0;
        config_.merge_threshold = 0.2;
    }

    /**
     * @brief 创建空白地图
     */
    nav_msgs::msg::OccupancyGrid createEmptyMap(int width, int height, double resolution = 0.1)
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
     * @brief 创建带中心障碍物的地图
     */
    nav_msgs::msg::OccupancyGrid createMapWithCentralObstacle(
        int width, int height, int obstacle_size)
    {
        nav_msgs::msg::OccupancyGrid map = createEmptyMap(width, height);

        int center_x = width / 2;
        int center_y = height / 2;

        // 在中心添加障碍物
        for (int y = center_y - obstacle_size / 2; y <= center_y + obstacle_size / 2; ++y) {
            for (int x = center_x - obstacle_size / 2; x <= center_x + obstacle_size / 2; ++x) {
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    map.data[y * width + x] = 100;
                }
            }
        }

        return map;
    }

    /**
     * @brief 创建L型房间地图
     * 
     * L型房间结构：
     * ┌──────────┐
     * │          │
     * │          │
     * │          ├──────┐
     * │          │      │
     * │          │      │
     * └──────────┴──────┘
     */
    nav_msgs::msg::OccupancyGrid createLShapedMap(int width, int height)
    {
        nav_msgs::msg::OccupancyGrid map = createEmptyMap(width, height);

        // 设置右上角区域为障碍物（形成L型空闲区域）
        int l_corner_x = width * 2 / 3;
        int l_corner_y = height / 3;

        for (int y = 0; y < l_corner_y; ++y) {
            for (int x = l_corner_x; x < width; ++x) {
                map.data[y * width + x] = 100;
            }
        }

        return map;
    }

    /**
     * @brief 创建多连通域地图（两个独立房间）
     */
    nav_msgs::msg::OccupancyGrid createMultiRoomMap(int width, int height)
    {
        nav_msgs::msg::OccupancyGrid map = createEmptyMap(width, height);

        // 创建两个房间之间的分隔墙
        int wall_x = width / 2;
        int wall_gap_y = height / 2;  // 中间留有通道

        for (int y = 0; y < height; ++y) {
            if (y < wall_gap_y - 2 || y > wall_gap_y + 2) {
                map.data[y * width + wall_x] = 100;
            }
        }

        return map;
    }

    /**
     * @brief 创建带多个障碍物的复杂地图
     */
    nav_msgs::msg::OccupancyGrid createComplexMap(int width, int height)
    {
        nav_msgs::msg::OccupancyGrid map = createEmptyMap(width, height);

        // 添加多个小障碍物
        // 障碍物1
        for (int y = 10; y < 20; ++y) {
            for (int x = 10; x < 20; ++x) {
                map.data[y * width + x] = 100;
            }
        }

        // 障碍物2
        for (int y = 30; y < 40; ++y) {
            for (int x = 30; x < 40; ++x) {
                map.data[y * width + x] = 100;
            }
        }

        // 障碍物3
        for (int y = 50; y < 60; ++y) {
            for (int x = 50; x < 60; ++x) {
                map.data[y * width + x] = 100;
            }
        }

        return map;
    }

    std::unique_ptr<ZoneDecomposer> decomposer_;
    ZoneDecomposerConfig config_;
};

// ==================== 基础功能测试 ====================

/**
 * @brief 测试：空白地图分区
 * 
 * 验收标准：
 * - 应产生1个连通域
 * - 区域面积等于地图总面积
 * - 分区成功
 */
TEST_F(ZoneDecomposerTest, EmptyMapDecomposition)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(50, 50);
    
    DecompositionResult result = decomposer_->decompose(map, config_);
    
    // 验证：分解成功
    EXPECT_TRUE(result.success);
    
    // 验证：至少检测到一个区域
    EXPECT_GE(result.zones.size(), static_cast<size_t>(1));
    
    // 验证：总空闲栅格数
    EXPECT_EQ(result.total_free_cells, 50 * 50);
    
    // 验证：第一个区域面积
    if (result.zones.size() > 0) {
        EXPECT_GT(result.zones[0].area, config_.min_zone_area);
    }
}

/**
 * @brief 测试：带中心障碍物的地图分区
 * 
 * 验收标准：
 * - 障碍物周围应形成连通域
 * - 障碍物本身不应作为区域
 * - 区域数量合理
 */
TEST_F(ZoneDecomposerTest, MapWithCentralObstacle)
{
    nav_msgs::msg::OccupancyGrid map = createMapWithCentralObstacle(100, 100, 20);
    
    DecompositionResult result = decomposer_->decompose(map, config_);
    
    // 验证：分解成功
    EXPECT_TRUE(result.success);
    
    // 验证：区域数量不超过最大值
    EXPECT_LE(result.zones.size(), static_cast<size_t>(config_.max_zone_count));
    
    // 验证：每个区域面积
    for (const auto& zone : result.zones) {
        EXPECT_GT(zone.area, config_.min_zone_area);
    }
}

/**
 * @brief 测试：L型房间分区
 * 
 * 验收标准：
 * - L型房间应正确识别为连通域
 * - 或被分解为多个矩形区域
 * - 最优扫描方向应沿长边方向
 */
TEST_F(ZoneDecomposerTest, LShapedRoomDecomposition)
{
    nav_msgs::msg::OccupancyGrid map = createLShapedMap(100, 100);
    
    DecompositionResult result = decomposer_->decompose(map, config_);
    
    // 验证：分解成功
    EXPECT_TRUE(result.success);
    
    // 验证：区域数量
    EXPECT_GE(result.zones.size(), static_cast<size_t>(1));
    
    // 验证：L型房间总面积
    int expected_area = 100 * 100 - (100 - 67) * 33;  // 粗略计算
    EXPECT_NEAR(result.total_free_cells, expected_area, expected_area * 0.3);
}

/**
 * @brief 测试：多连通域地图
 * 
 * 验收标准：
 * - 应检测到多个独立连通域
 * - 连接通道应正确识别
 * - 区域ID应唯一
 */
TEST_F(ZoneDecomposerTest, MultiRoomDecomposition)
{
    nav_msgs::msg::OccupancyGrid map = createMultiRoomMap(100, 100);
    
    DecompositionResult result = decomposer_->decompose(map, config_);
    
    // 验证：分解成功
    EXPECT_TRUE(result.success);
    
    // 验证：连接通道
    EXPECT_GE(result.channels.size(), static_cast<size_t>(1));
    
    // 验证：访问顺序
    EXPECT_GE(result.visit_order.size(), static_cast<size_t>(1));
    
    // 验证：区域ID唯一性
    std::set<int> ids;
    for (const auto& zone : result.zones) {
        ids.insert(zone.id);
    }
    EXPECT_EQ(ids.size(), result.zones.size());
}

/**
 * @brief 测试：复杂障碍物地图
 * 
 * 验收标准：
 * - 多个障碍物正确处理
 * - 区域分解合理
 * - 无异常抛出
 */
TEST_F(ZoneDecomposerTest, ComplexObstacleMap)
{
    nav_msgs::msg::OccupancyGrid map = createComplexMap(100, 100);
    
    // 不应抛出异常
    EXPECT_NO_THROW({
        DecompositionResult result = decomposer_->decompose(map, config_);
        
        // 验证：区域数量
        EXPECT_GE(result.zones.size(), static_cast<size_t>(1));
        EXPECT_LE(result.zones.size(), static_cast<size_t>(config_.max_zone_count));
    });
}

// ==================== 连接通道测试 ====================

/**
 * @brief 测试：连接通道检测
 * 
 * 验收标准：
 * - 相邻区域间连接通道正确识别
 * - 连接点位置合理
 */
TEST_F(ZoneDecomposerTest, ConnectionChannelDetection)
{
    nav_msgs::msg::OccupancyGrid map = createMultiRoomMap(100, 100);
    
    DecompositionResult result = decomposer_->decompose(map, config_);
    
    // 验证：连接通道数量
    if (result.zones.size() >= 2) {
        EXPECT_GE(result.channels.size(), static_cast<size_t>(1));
        
        // 验证：连接通道有效性
        for (const auto& channel : result.channels) {
            EXPECT_NE(channel.zone_a_id, channel.zone_b_id);
            EXPECT_TRUE(channel.is_reachable);
        }
    }
}

// ==================== 扫描方向优化测试 ====================

/**
 * @brief 测试：扫描方向计算
 * 
 * 验收标准：
 * - PCA主方向角度应在有效范围内
 * - 扫描方向合理
 */
TEST_F(ZoneDecomposerTest, OptimalScanDirection)
{
    nav_msgs::msg::OccupancyGrid map = createLShapedMap(100, 100);
    
    DecompositionResult result = decomposer_->decompose(map, config_);
    
    for (const auto& zone : result.zones) {
        // 验证：PCA角度范围
        EXPECT_GE(zone.principal_angle, -M_PI);
        EXPECT_LE(zone.principal_angle, M_PI);
        
        // 验证：扫描方向有效性
        EXPECT_TRUE(zone.optimal_scan_direction == 0 || zone.optimal_scan_direction == 1);
    }
}

// ==================== 边界条件测试 ====================

/**
 * @brief 测试：极小地图
 * 
 * 验收标准：
 * - 不应崩溃
 * - 返回空列表或单个小区域
 */
TEST_F(ZoneDecomposerTest, VerySmallMap)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(5, 5);
    
    EXPECT_NO_THROW({
        DecompositionResult result = decomposer_->decompose(map, config_);
        EXPECT_LE(result.zones.size(), static_cast<size_t>(1));
    });
}

/**
 * @brief 测试：全障碍物地图
 * 
 * 验收标准：
 * - 不应崩溃
 * - 返回空区域列表
 */
TEST_F(ZoneDecomposerTest, FullObstacleMap)
{
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = 50;
    map.info.height = 50;
    map.info.resolution = 0.1;
    map.data.resize(50 * 50, 100);  // 全部障碍物
    
    EXPECT_NO_THROW({
        DecompositionResult result = decomposer_->decompose(map, config_);
        EXPECT_EQ(result.zones.size(), static_cast<size_t>(0));
    });
}

/**
 * @brief 测试：单行地图
 * 
 * 验收标准：
 * - 不应崩溃
 * - 正确处理边界情况
 */
TEST_F(ZoneDecomposerTest, SingleRowMap)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(100, 1);
    
    EXPECT_NO_THROW({
        DecompositionResult result = decomposer_->decompose(map, config_);
        EXPECT_LE(result.zones.size(), static_cast<size_t>(1));
    });
}

/**
 * @brief 测试：单列地图
 * 
 * 验收标准：
 * - 不应崩溃
 * - 正确处理边界情况
 */
TEST_F(ZoneDecomposerTest, SingleColumnMap)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(1, 100);
    
    EXPECT_NO_THROW({
        DecompositionResult result = decomposer_->decompose(map, config_);
        EXPECT_LE(result.zones.size(), static_cast<size_t>(1));
    });
}

// ==================== 配置参数测试 ====================

/**
 * @brief 测试：最小区域面积参数影响
 * 
 * 验收标准：
 * - 较大的min_zone_area参数应合并小区域
 */
TEST_F(ZoneDecomposerTest, MinZoneAreaParameter)
{
    nav_msgs::msg::OccupancyGrid map = createComplexMap(100, 100);
    
    // 小面积阈值
    ZoneDecomposerConfig small_config;
    small_config.min_zone_area = 50;
    DecompositionResult small_result = decomposer_->decompose(map, small_config);
    
    // 大面积阈值
    ZoneDecomposerConfig large_config;
    large_config.min_zone_area = 2000;
    DecompositionResult large_result = decomposer_->decompose(map, large_config);
    
    // 验证：大阈值应产生更少区域
    EXPECT_LE(large_result.zones.size(), small_result.zones.size());
}

/**
 * @brief 测试：最大区域数量参数
 * 
 * 验收标准：
 * - max_zone_count参数应限制区域数量
 */
TEST_F(ZoneDecomposerTest, MaxZonesParameter)
{
    nav_msgs::msg::OccupancyGrid map = createComplexMap(100, 100);
    
    ZoneDecomposerConfig limited_config;
    limited_config.max_zone_count = 2;
    limited_config.min_zone_area = 50;
    
    DecompositionResult result = decomposer_->decompose(map, limited_config);
    
    // 验证：区域数量不超过最大值
    EXPECT_LE(result.zones.size(), static_cast<size_t>(limited_config.max_zone_count));
}

// ==================== 性能测试 ====================

/**
 * @brief 测试：大地图性能
 * 
 * 验收标准：
 * - 500x500地图分区时间应<500ms
 * - 不应内存溢出
 */
TEST_F(ZoneDecomposerTest, LargeMapPerformance)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(500, 500);
    
    auto start = std::chrono::high_resolution_clock::now();
    DecompositionResult result = decomposer_->decompose(map, config_);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    // 验证：性能
    EXPECT_LT(duration.count(), 500);
    
    // 验证：分解成功
    EXPECT_TRUE(result.success);
}

// ==================== 访问顺序测试 ====================

/**
 * @brief 测试：访问顺序计算
 * 
 * 验收标准：
 * - 访问顺序包含所有区域
 * - 区域ID唯一
 */
TEST_F(ZoneDecomposerTest, VisitOrderCalculation)
{
    nav_msgs::msg::OccupancyGrid map = createMultiRoomMap(100, 100);
    
    DecompositionResult result = decomposer_->decompose(map, config_);
    
    if (result.zones.size() >= 2) {
        // 验证：访问顺序包含所有区域
        EXPECT_EQ(result.visit_order.size(), result.zones.size());
        
        // 验证：所有区域ID唯一
        std::set<int> visited_ids;
        for (int id : result.visit_order) {
            visited_ids.insert(id);
        }
        EXPECT_EQ(visited_ids.size(), result.zones.size());
    }
}

// ==================== 重置测试 ====================

/**
 * @brief 测试：重置功能
 * 
 * 验收标准：
 * - reset()不应抛出异常
 */
TEST_F(ZoneDecomposerTest, ResetFunctionality)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(50, 50);
    DecompositionResult result1 = decomposer_->decompose(map, config_);
    
    EXPECT_NO_THROW({
        decomposer_->reset();
    });
    
    // 第二次分解
    DecompositionResult result2 = decomposer_->decompose(map, config_);
    EXPECT_TRUE(result2.success);
}

}  // namespace coverage_planner

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}