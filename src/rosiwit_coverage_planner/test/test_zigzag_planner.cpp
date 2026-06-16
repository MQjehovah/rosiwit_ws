// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include <gtest/gtest.h>
#include "coverage_planner/zigzag_planner.hpp"
#include "coverage_planner/coverage_utils.hpp"

namespace coverage_planner
{

class ZigzagPlannerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        planner_ = std::make_unique<ZigzagPlanner>();
        
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
    
    nav_msgs::msg::OccupancyGrid createLShapedMap(int width, int height)
    {
        nav_msgs::msg::OccupancyGrid map = createEmptyMap(width, height);
        
        // 创建L形障碍物（右上角和右下角）
        for (int y = height - height / 4; y < height; ++y) {
            for (int x = 0; x < width - width / 4; ++x) {
                map.data[y * width + x] = 100;
            }
        }
        
        for (int y = 0; y < height / 4; ++y) {
            for (int x = width - width / 4; x < width; ++x) {
                map.data[y * width + x] = 100;
            }
        }
        
        return map;
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
    
    std::unique_ptr<ZigzagPlanner> planner_;
    PlannerConfig config_;
};

// 测试：空旷地图上的弓字形规划
TEST_F(ZigzagPlannerTest, TestEmptyMapPlanning)
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
TEST_F(ZigzagPlannerTest, TestMapWithObstacle)
{
    nav_msgs::msg::OccupancyGrid map = createMapWithCentralObstacle(50, 50, 10);
    geometry_msgs::msg::Pose start_pose = createStartPose(2.5, 2.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GT(result.coverage_rate, 0.85);  // 即使有障碍物，覆盖率也应该较高
}

// 测试：小尺寸地图
TEST_F(ZigzagPlannerTest, TestSmallMap)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(10, 10);
    geometry_msgs::msg::Pose start_pose = createStartPose(0.5, 0.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

// 测试：大尺寸地图
TEST_F(ZigzagPlannerTest, TestLargeMap)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(100, 100);
    geometry_msgs::msg::Pose start_pose = createStartPose(5.0, 5.0);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GT(result.path_length, 50.0);  // 大地图路径应该很长
}

// 测试：规划器名称
TEST_F(ZigzagPlannerTest, TestPlannerName)
{
    EXPECT_EQ(planner_->getName(), "ZigzagPlanner");
}

// 测试：规划器重置
TEST_F(ZigzagPlannerTest, TestReset)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(20, 20);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.0, 1.0);
    
    PlannerResult result1 = planner_->plan(map, start_pose, config_);
    EXPECT_TRUE(result1.success);
    
    planner_->reset();
    
    PlannerResult result2 = planner_->plan(map, start_pose, config_);
    EXPECT_TRUE(result2.success);
}

// 测试：无效起始位置（在障碍物上）
TEST_F(ZigzagPlannerTest, TestInvalidStartPosition)
{
    nav_msgs::msg::OccupancyGrid map = createMapWithCentralObstacle(50, 50, 20);
    
    // 将起始位置放在障碍物中心
    geometry_msgs::msg::Pose start_pose = createStartPose(2.5, 2.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    // 应该失败或返回错误
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// 测试：不同分辨率地图
TEST_F(ZigzagPlannerTest, TestDifferentResolution)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(50, 50, 0.05);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.25, 1.25);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

// 测试：水平扫描方向
TEST_F(ZigzagPlannerTest, TestHorizontalScanDirection)
{
    config_.direction_optimization = 0;  // 强制水平
    
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 20);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 1.0);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

// 测试：垂直扫描方向
TEST_F(ZigzagPlannerTest, TestVerticalScanDirection)
{
    config_.direction_optimization = 1;  // 强制垂直
    
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 20);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 1.0);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

// 测试：自动方向选择
TEST_F(ZigzagPlannerTest, TestAutoDirectionSelection)
{
    config_.direction_optimization = 2;  // 自动
    
    // 宽地图（水平应该更优）
    nav_msgs::msg::OccupancyGrid wide_map = createEmptyMap(40, 20);
    geometry_msgs::msg::Pose start_pose1 = createStartPose(2.0, 1.0);
    
    PlannerResult result1 = planner_->plan(wide_map, start_pose1, config_);
    EXPECT_TRUE(result1.success);
    
    // 高地图（垂直应该更优）
    nav_msgs::msg::OccupancyGrid tall_map = createEmptyMap(20, 40);
    geometry_msgs::msg::Pose start_pose2 = createStartPose(1.0, 2.0);
    
    PlannerResult result2 = planner_->plan(tall_map, start_pose2, config_);
    EXPECT_TRUE(result2.success);
}

// 测试：路径优化关闭
TEST_F(ZigzagPlannerTest, TestOptimizationDisabled)
{
    config_.enable_optimization = false;
    
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 30);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 1.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

// 测试：转弯次数统计
TEST_F(ZigzagPlannerTest, TestTurnCount)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 30);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 1.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_GE(result.turn_count, 0);
    
    // 对于30x30的地图，转弯次数应该合理
    EXPECT_LT(result.turn_count, 100);
}

// 测试：复杂障碍物布局
TEST_F(ZigzagPlannerTest, TestComplexObstacles)
{
    nav_msgs::msg::OccupancyGrid map = createLShapedMap(50, 50);
    geometry_msgs::msg::Pose start_pose = createStartPose(2.5, 2.5);
    
    PlannerResult result = planner_->plan(map, start_pose, config_);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GT(result.coverage_rate, 0.7);  // 复杂地图覆盖率可能稍低
}

}  // namespace coverage_planner

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}