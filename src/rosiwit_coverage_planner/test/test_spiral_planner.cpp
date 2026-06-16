// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include <gtest/gtest.h>
#include "coverage_planner/spiral_planner.hpp"
#include "coverage_planner/coverage_utils.hpp"

namespace coverage_planner
{

class SpiralPlannerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        planner_ = std::make_unique<SpiralPlanner>();
        
        // 创建默认配置
        config_.robot_radius = 0.3;
        config_.coverage_resolution = 0.1;
        config_.inflation_radius = 0.25;
        config_.enable_optimization = true;
        config_.direction_optimization = 2;
    }
    
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
    
    nav_msgs::msg::OccupancyGrid createRectangularRoomMap(int width, int height)
    {
        return createEmptyMap(width, height);
    }
    
    geometry_msgs::msg::Pose createStartPose(double x, double y)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;
        pose.orientation.w = 1.0;
        return pose;
    }
    
    std::unique_ptr<SpiralPlanner> planner_;
    PlannerConfig config_;
};

// 测试：空旷矩形地图的回字形规划
TEST_F(SpiralPlannerTest, TestEmptyMapPlanning)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(50, 50);
    geometry_msgs::msg::Pose start_pose = createStartPose(2.5, 2.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GT(result.coverage_rate, 0.9);  // 覆盖率应该很高
    EXPECT_GT(result.path_length, 0.0);
}

// 测试：带障碍物的地图规划
TEST_F(SpiralPlannerTest, TestMapWithObstacle)
{
    nav_msgs::msg::OccupancyGrid map = createMapWithCentralObstacle(50, 50, 10);
    geometry_msgs::msg::Pose start_pose = createStartPose(2.5, 2.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GT(result.coverage_rate, 0.85);  // 即使有障碍物，覆盖率也应该较高
}

// 测试：小尺寸地图
TEST_F(SpiralPlannerTest, TestSmallMap)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(10, 10);
    geometry_msgs::msg::Pose start_pose = createStartPose(0.5, 0.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

// 测试：大尺寸地图
TEST_F(SpiralPlannerTest, TestLargeMap)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(100, 100);
    geometry_msgs::msg::Pose start_pose = createStartPose(5.0, 5.0);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GT(result.path_length, 50.0);  // 大地图路径应该很长
}

// 测试：规划器名称
TEST_F(SpiralPlannerTest, TestPlannerName)
{
    EXPECT_EQ(planner_->getName(), "SpiralPlanner");
}

// 测试：规划器重置
TEST_F(SpiralPlannerTest, TestReset)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(20, 20);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.0, 1.0);
    
    PlannerResult result1 = planner_->plan(map, start_pose, config_);
    EXPECT_TRUE(result1.success);
    
    planner_->reset();
    
    PlannerResult result2 = planner_->plan(map, start_pose, config_);
    EXPECT_TRUE(result2.success);
}

// 测试：矩形房间（凸区域）
TEST_F(SpiralPlannerTest, TestRectangularRoom)
{
    nav_msgs::msg::OccupancyGrid map = createRectangularRoomMap(30, 40);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 2.0);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GT(result.coverage_rate, 0.95);  // 矩形房间应该能完全覆盖
}

// 测试：不同分辨率地图
TEST_F(SpiralPlannerTest, TestDifferentResolution)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(50, 50, 0.05);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.25, 1.25);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

// 测试：无效起始位置（在障碍物上）
TEST_F(SpiralPlannerTest, TestInvalidStartPosition)
{
    nav_msgs::msg::OccupancyGrid map = createMapWithCentralObstacle(50, 50, 20);
    
    // 将起始位置放在障碍物中心
    geometry_msgs::msg::Pose start_pose = createStartPose(2.5, 2.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    // 螺旋规划器会尝试降级到弓字形，所以可能成功
    // 但如果起始位置无效，应该有适当的处理
    if (!result.success) {
        EXPECT_FALSE(result.error_message.empty());
    }
}

// 测试：非凸区域降级
TEST_F(SpiralPlannerTest, TestNonConvexRegionFallback)
{
    // 创建非凸区域地图（有多个障碍物）
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(50, 50);
    
    // 添加多个障碍物，使区域不凸
    for (int y = 10; y < 20; ++y) {
        for (int x = 25; x < 35; ++x) {
            map.data[y * 50 + x] = 100;
        }
    }
    
    for (int y = 30; y < 40; ++y) {
        for (int x = 25; x < 35; ++x) {
            map.data[y * 50 + x] = 100;
        }
    }
    
    geometry_msgs::msg::Pose start_pose = createStartPose(1.25, 1.25);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    // 即使是非凸区域，通过降级策略也应该能规划
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

// 测试：转弯次数统计
TEST_F(SpiralPlannerTest, TestTurnCount)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 30);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 1.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_GE(result.turn_count, 0);
    
    // 回字形路径转弯次数应该比弓字形少
    EXPECT_LT(result.turn_count, 100);
}

// 测试：覆盖率统计
TEST_F(SpiralPlannerTest, TestCoverageRate)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(40, 40);
    geometry_msgs::msg::Pose start_pose = createStartPose(2.0, 2.0);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_GE(result.coverage_rate, 0.0);
    EXPECT_LE(result.coverage_rate, 1.0);
}

// 测试：路径优化关闭
TEST_F(SpiralPlannerTest, TestOptimizationDisabled)
{
    config_.enable_optimization = false;
    
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 30);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 1.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

// 测试：复杂障碍物布局
TEST_F(SpiralPlannerTest, TestComplexObstacles)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(50, 50);
    
    // 创建复杂的障碍物布局
    // 左上角障碍物
    for (int y = 0; y < 15; ++y) {
        for (int x = 0; x < 15; ++x) {
            map.data[y * 50 + x] = 100;
        }
    }
    
    // 右下角障碍物
    for (int y = 35; y < 50; ++y) {
        for (int x = 35; x < 50; ++x) {
            map.data[y * 50 + x] = 100;
        }
    }
    
    geometry_msgs::msg::Pose start_pose = createStartPose(1.25, 1.25);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    // 通过降级策略应该能规划
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GT(result.coverage_rate, 0.7);
}

// 测试：路径连续性
TEST_F(SpiralPlannerTest, TestPathContinuity)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 30);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 1.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    
    // 检查路径是否连续（每个相邻点距离合理）
    for (size_t i = 1; i < result.path.size(); ++i) {
        double distance = PathUtils::distanceBetween(
            result.path[i - 1], result.path[i]);
        
        // 相邻点距离不应该太大（除非是跳跃连接）
        EXPECT_LT(distance, 5.0);  // 允许一定的跳跃，但不应太大
    }
}

}  // namespace coverage_planner

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}