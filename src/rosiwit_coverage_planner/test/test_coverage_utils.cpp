// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include <gtest/gtest.h>
#include "coverage_planner/coverage_utils.hpp"
#include "coverage_planner/zigzag_planner.hpp"
#include <cmath>

namespace coverage_planner
{

// 测试MapUtils工具类
class MapUtilsTest : public ::testing::Test
{
protected:
    nav_msgs::msg::OccupancyGrid createTestMap(int width, int height, int fill_value = 0)
    {
        nav_msgs::msg::OccupancyGrid map;
        map.info.width = width;
        map.info.height = height;
        map.info.resolution = 0.1;
        map.info.origin.position.x = 0.0;
        map.info.origin.position.y = 0.0;
        map.info.origin.position.z = 0.0;
        map.header.frame_id = "map";
        map.data.resize(width * height, fill_value);
        
        return map;
    }
    
    nav_msgs::msg::OccupancyGrid createMapWithObstacle(int width, int height)
    {
        nav_msgs::msg::OccupancyGrid map = createTestMap(width, height);
        
        // 在中心添加障碍物
        for (int y = height / 3; y < 2 * height / 3; ++y) {
            for (int x = width / 3; x < 2 * width / 3; ++x) {
                map.data[y * width + x] = 100;
            }
        }
        
        return map;
    }
};

// 测试边界检查
TEST_F(MapUtilsTest, TestIsInBounds)
{
    nav_msgs::msg::OccupancyGrid map = createTestMap(10, 10);
    
    // 有效边界
    EXPECT_TRUE(MapUtils::isInBounds(map, 0, 0));
    EXPECT_TRUE(MapUtils::isInBounds(map, 5, 5));
    EXPECT_TRUE(MapUtils::isInBounds(map, 9, 9));
    
    // 边界外
    EXPECT_FALSE(MapUtils::isInBounds(map, -1, 0));
    EXPECT_FALSE(MapUtils::isInBounds(map, 0, -1));
    EXPECT_FALSE(MapUtils::isInBounds(map, 10, 0));
    EXPECT_FALSE(MapUtils::isInBounds(map, 0, 10));
}

// 测试障碍物检查
TEST_F(MapUtilsTest, TestIsObstacle)
{
    nav_msgs::msg::OccupancyGrid map = createMapWithObstacle(10, 10);
    
    // 障碍物区域
    EXPECT_TRUE(MapUtils::isObstacle(map, 4, 4));
    EXPECT_TRUE(MapUtils::isObstacle(map, 5, 5));
    
    // 空闲区域
    EXPECT_FALSE(MapUtils::isObstacle(map, 0, 0));
    EXPECT_FALSE(MapUtils::isObstacle(map, 9, 9));
    
    // 边界外视为障碍物
    EXPECT_TRUE(MapUtils::isObstacle(map, -1, 0));
    EXPECT_TRUE(MapUtils::isObstacle(map, 10, 10));
}

// 测试空闲区域检查
TEST_F(MapUtilsTest, TestIsFree)
{
    nav_msgs::msg::OccupancyGrid map = createMapWithObstacle(10, 10);
    
    // 空闲区域
    EXPECT_TRUE(MapUtils::isFree(map, 0, 0));
    EXPECT_TRUE(MapUtils::isFree(map, 9, 9));
    
    // 障碍物区域
    EXPECT_FALSE(MapUtils::isFree(map, 4, 4));
    EXPECT_FALSE(MapUtils::isFree(map, 5, 5));
    
    // 边界外
    EXPECT_FALSE(MapUtils::isFree(map, -1, 0));
    EXPECT_FALSE(MapUtils::isFree(map, 10, 10));
}

// 测试坐标转换
TEST_F(MapUtilsTest, TestWorldToGrid)
{
    nav_msgs::msg::OccupancyGrid map = createTestMap(100, 100);
    map.info.resolution = 0.1;
    
    Point2D grid = MapUtils::worldToGrid(map, 5.0, 5.0);
    EXPECT_EQ(grid.x, 50);
    EXPECT_EQ(grid.y, 50);
    
    grid = MapUtils::worldToGrid(map, 0.05, 0.05);
    EXPECT_EQ(grid.x, 0);
    EXPECT_EQ(grid.y, 0);
}

TEST_F(MapUtilsTest, TestGridToWorld)
{
    nav_msgs::msg::OccupancyGrid map = createTestMap(100, 100);
    map.info.resolution = 0.1;
    
    double world_x, world_y;
    MapUtils::gridToWorld(map, 50, 50, world_x, world_y);
    
    EXPECT_NEAR(world_x, 5.05, 0.001);
    EXPECT_NEAR(world_y, 5.05, 0.001);
}

// 测试可达性检查
TEST_F(MapUtilsTest, TestGetReachableCells)
{
    nav_msgs::msg::OccupancyGrid map = createTestMap(10, 10);
    
    std::vector<Point2D> reachable = MapUtils::getReachableCells(map, Point2D(0, 0));
    
    // 所有空闲区域应该可达
    EXPECT_EQ(reachable.size(), 100);
}

TEST_F(MapUtilsTest, TestGetReachableCellsWithObstacle)
{
    nav_msgs::msg::OccupancyGrid map = createMapWithObstacle(10, 10);
    
    // 从左上角开始，只能访问障碍物外的区域
    std::vector<Point2D> reachable = MapUtils::getReachableCells(map, Point2D(0, 0));
    
    // 应该小于100（因为有障碍物）
    EXPECT_LT(reachable.size(), 100);
    EXPECT_GT(reachable.size(), 0);
}

// 测试空闲区域获取
TEST_F(MapUtilsTest, TestGetFreeCells)
{
    nav_msgs::msg::OccupancyGrid map = createMapWithObstacle(10, 10);
    
    std::vector<Point2D> free_cells = MapUtils::getFreeCells(map);
    
    // 空闲区域 = 总区域 - 障碍物区域
    // 障碍物区域: (10/3 to 20/3) = 约3x3 = 9个格子
    EXPECT_GT(free_cells.size(), 80);
    EXPECT_LT(free_cells.size(), 100);
}

// 测试最优扫描方向选择
TEST_F(MapUtilsTest, TestGetOptimalScanDirection)
{
    nav_msgs::msg::OccupancyGrid map = createTestMap(20, 10);
    
    int direction = MapUtils::getOptimalScanDirection(map);
    
    // 对于宽地图（20x10），水平扫描应该更优
    EXPECT_EQ(direction, 0);
}

// 测试PathUtils工具类
class PathUtilsTest : public ::testing::Test
{
protected:
    std::vector<geometry_msgs::msg::PoseStamped> createTestPath()
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = static_cast<double>(i);
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
            path.push_back(pose);
        }
        
        return path;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> createPathWithTurns()
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        
        // 直线段
        for (int i = 0; i < 5; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = static_cast<double>(i);
            pose.pose.position.y = 0.0;
            path.push_back(pose);
        }
        
        // 转弯后向上
        for (int i = 0; i < 5; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = 5.0;
            pose.pose.position.y = static_cast<double>(i);
            path.push_back(pose);
        }
        
        // 再次转弯向左
        for (int i = 0; i < 5; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = 5.0 - static_cast<double>(i);
            pose.pose.position.y = 5.0;
            path.push_back(pose);
        }
        
        return path;
    }
};

// 测试路径长度计算
TEST_F(PathUtilsTest, TestCalculatePathLength)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createTestPath();
    
    double length = PathUtils::calculatePathLength(path);
    
    // 10个点，间距1米，总长度约9米
    EXPECT_NEAR(length, 9.0, 0.01);
}

// 测试空路径长度
TEST_F(PathUtilsTest, TestCalculatePathLengthEmpty)
{
    std::vector<geometry_msgs::msg::PoseStamped> path;
    
    double length = PathUtils::calculatePathLength(path);
    
    EXPECT_NEAR(length, 0.0, 0.01);
}

// 测试转弯次数计算
TEST_F(PathUtilsTest, TestCalculateTurnCount)
{
    std::vector<geometry_msgs::msg::PoseStamped> path = createPathWithTurns();
    
    int turn_count = PathUtils::calculateTurnCount(path);
    
    // 两条转弯路径
    EXPECT_GE(turn_count, 1);
}

// 测试距离计算
TEST_F(PathUtilsTest, TestDistanceBetween)
{
    geometry_msgs::msg::PoseStamped pose1, pose2;
    pose1.pose.position.x = 0.0;
    pose1.pose.position.y = 0.0;
    pose2.pose.position.x = 3.0;
    pose2.pose.position.y = 4.0;
    
    double distance = PathUtils::distanceBetween(pose1, pose2);
    
    EXPECT_NEAR(distance, 5.0, 0.01);
}

// 测试PoseStamped创建
TEST_F(PathUtilsTest, TestCreatePoseStamped)
{
    geometry_msgs::msg::PoseStamped pose = 
        PathUtils::createPoseStamped(5.0, 10.0, M_PI / 4, "map");
    
    EXPECT_NEAR(pose.pose.position.x, 5.0, 0.01);
    EXPECT_NEAR(pose.pose.position.y, 10.0, 0.01);
    EXPECT_EQ(pose.header.frame_id, "map");
}

}  // namespace coverage_planner

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}