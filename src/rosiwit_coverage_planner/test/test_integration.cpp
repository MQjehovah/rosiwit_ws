// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include <gtest/gtest.h>
#include "coverage_planner/planner_context.hpp"
#include "coverage_planner/coverage_utils.hpp"
#include <chrono>

namespace coverage_planner
{

// 集成测试类
class IntegrationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        context_ = std::make_unique<PlannerContext>();

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

    nav_msgs::msg::OccupancyGrid createMapWithObstacles(int width, int height)
    {
        nav_msgs::msg::OccupancyGrid map = createEmptyMap(width, height);

        // 添加多个障碍物
        for (int y = height/3; y < height/3 + height/6; ++y) {
            for (int x = width/3; x < width/3 + width/6; ++x) {
                map.data[y * width + x] = 100;
            }
        }

        for (int y = height*2/3; y < height*2/3 + height/6; ++y) {
            for (int x = width*2/3; x < width*2/3 + width/6; ++x) {
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

    std::unique_ptr<PlannerContext> context_;
    PlannerConfig config_;
};

// 测试1：弓字形算法在空旷地图覆盖率100%
TEST_F(IntegrationTest, ZigzagEmptyMapCoverage100Percent)
{
    auto planner = context_->selectPlanner("zigzag");
    ASSERT_NE(planner, nullptr);

    nav_msgs::msg::OccupancyGrid map = createEmptyMap(50, 50);
    geometry_msgs::msg::Pose start_pose = createStartPose(2.5, 2.5);

    PlannerResult result = planner->plan(map, start_pose, config_);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GE(result.coverage_rate, 0.99);  // 允许1%误差
}

// 测试2：弓字形算法在含障碍物地图覆盖率≥90%
TEST_F(IntegrationTest, ZigzagObstacleMapCoverage90Percent)
{
    auto planner = context_->selectPlanner("zigzag");
    ASSERT_NE(planner, nullptr);

    nav_msgs::msg::OccupancyGrid map = createMapWithObstacles(50, 50);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.0, 1.0);

    PlannerResult result = planner->plan(map, start_pose, config_);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GE(result.coverage_rate, 0.90);  // ≥90%
}

// 测试3：回字形算法在空旷地图覆盖率100%
TEST_F(IntegrationTest, SpiralEmptyMapCoverage100Percent)
{
    auto planner = context_->selectPlanner("spiral");
    ASSERT_NE(planner, nullptr);

    nav_msgs::msg::OccupancyGrid map = createEmptyMap(50, 50);
    geometry_msgs::msg::Pose start_pose = createStartPose(2.5, 2.5);

    PlannerResult result = planner->plan(map, start_pose, config_);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GE(result.coverage_rate, 0.99);  // 允许1%误差
}

// 测试4：回字形算法在含障碍物地图覆盖率≥95%
TEST_F(IntegrationTest, SpiralObstacleMapCoverage95Percent)
{
    auto planner = context_->selectPlanner("spiral");
    ASSERT_NE(planner, nullptr);

    nav_msgs::msg::OccupancyGrid map = createMapWithObstacles(50, 50);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.0, 1.0);

    PlannerResult result = planner->plan(map, start_pose, config_);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GE(result.coverage_rate, 0.95);  // ≥95%
}

// 测试5：算法切换功能
TEST_F(IntegrationTest, AlgorithmSwitch)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 30);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 1.5);

    // 切换到弓字形
    auto zigzag_planner = context_->selectPlanner("zigzag");
    ASSERT_NE(zigzag_planner, nullptr);

    PlannerResult result1 = zigzag_planner->plan(map, start_pose, config_);
    EXPECT_TRUE(result1.success);

    // 切换到回字形
    auto spiral_planner = context_->selectPlanner("spiral");
    ASSERT_NE(spiral_planner, nullptr);

    PlannerResult result2 = spiral_planner->plan(map, start_pose, config_);
    EXPECT_TRUE(result2.success);

    // 两种算法都应该成功规划
    EXPECT_FALSE(result1.path.empty());
    EXPECT_FALSE(result2.path.empty());
}

// 测试6：路径连续性检查
TEST_F(IntegrationTest, PathContinuity)
{
    auto planner = context_->selectPlanner("zigzag");

    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 30);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 1.5);

    PlannerResult result = planner->plan(map, start_pose, config_);

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 1);

    // 检查路径点之间的连续性
    double max_gap = 0.0;
    for (size_t i = 1; i < result.path.size(); ++i) {
        double dx = result.path[i].pose.position.x - result.path[i-1].pose.position.x;
        double dy = result.path[i].pose.position.y - result.path[i-1].pose.position.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        max_gap = std::max(max_gap, distance);
    }

    // 最大间距不应超过机器人半径的3倍
    EXPECT_LT(max_gap, config_.robot_radius * 3.0);
}

// 测试7：路径长度合理性
TEST_F(IntegrationTest, PathLengthReasonable)
{
    auto planner = context_->selectPlanner("zigzag");

    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 30);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 1.5);

    PlannerResult result = planner->plan(map, start_pose, config_);

    EXPECT_TRUE(result.success);

    double path_length = PathUtils::calculatePathLength(result.path);

    // 路径长度应该在合理范围内
    // 30x30地图，分辨率为0.1，覆盖约900个栅格，路径长度约90米
    EXPECT_GT(path_length, 50.0);  // 最小长度
    EXPECT_LT(path_length, 150.0);  // 最大长度
}

// 测试8：转弯次数合理性
TEST_F(IntegrationTest, TurnCountReasonable)
{
    auto planner = context_->selectPlanner("zigzag");

    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 30);
    geometry_msgs::msg::Pose start_pose = createStartPose(1.5, 1.5);

    PlannerResult result = planner->plan(map, start_pose, config_);

    EXPECT_TRUE(result.success);

    // 弓字形算法在30x30地图上转弯次数应该在合理范围
    EXPECT_GT(result.turn_count, 0);
    EXPECT_LT(result.turn_count, 100);
}

// 测试9：复杂地图处理
TEST_F(IntegrationTest, ComplexMapHandling)
{
    auto planner = context_->selectPlanner("spiral");

    // 创建复杂的L形地图
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(50, 50);

    // 添加L形障碍物
    for (int y = 0; y < 25; ++y) {
        for (int x = 30; x < 50; ++x) {
            map.data[y * 50 + x] = 100;
        }
    }
    for (int y = 30; y < 50; ++y) {
        for (int x = 0; x < 50; ++x) {
            map.data[y * 50 + x] = 100;
        }
    }

    geometry_msgs::msg::Pose start_pose = createStartPose(2.5, 2.5);

    PlannerResult result = planner->plan(map, start_pose, config_);

    // 即使复杂地图也应该能规划成功
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

// 测试10：不同分辨率地图
TEST_F(IntegrationTest, DifferentResolution)
{
    // 测试不同分辨率
    std::vector<double> resolutions = {0.05, 0.1, 0.2};

    for (double res : resolutions) {
        auto planner = context_->selectPlanner("zigzag");
        nav_msgs::msg::OccupancyGrid map = createEmptyMap(50, 50, res);
        geometry_msgs::msg::Pose start_pose = createStartPose(2.5, 2.5);

        PlannerResult result = planner->plan(map, start_pose, config_);

        EXPECT_TRUE(result.success) << "Failed for resolution: " << res;
        EXPECT_FALSE(result.path.empty()) << "Empty path for resolution: " << res;
    }
}

// 测试11：边界条件 - 起点在地图边缘
TEST_F(IntegrationTest, StartPoseAtBoundary)
{
    auto planner = context_->selectPlanner("zigzag");

    nav_msgs::msg::OccupancyGrid map = createEmptyMap(30, 30);

    // 起点在地图边缘（考虑机器人半径）
    geometry_msgs::msg::Pose start_pose = createStartPose(0.5, 0.5);

    PlannerResult result = planner->plan(map, start_pose, config_);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

// 测试12：边界条件 - 非常小的地图
TEST_F(IntegrationTest, VerySmallMap)
{
    auto planner = context_->selectPlanner("zigzag");

    nav_msgs::msg::OccupancyGrid map = createEmptyMap(10, 10);
    geometry_msgs::msg::Pose start_pose = createStartPose(0.5, 0.5);

    PlannerResult result = planner->plan(map, start_pose, config_);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

}  // namespace coverage_planner

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}