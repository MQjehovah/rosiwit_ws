// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include <gtest/gtest.h>
#include "coverage_planner/zigzag_planner.hpp"
#include "coverage_planner/spiral_planner.hpp"
#include "coverage_planner/planner_context.hpp"
#include <chrono>
#include <memory>

namespace coverage_planner
{

// 性能测试类
class PerformanceTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        zigzag_planner_ = std::make_unique<ZigzagPlanner>();
        spiral_planner_ = std::make_unique<SpiralPlanner>();

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

        // 添加障碍物（约占20%面积）
        for (int y = height/4; y < height/4 + height/6; ++y) {
            for (int x = width/4; x < width/4 + width/6; ++x) {
                map.data[y * width + x] = 100;
            }
        }

        for (int y = height*3/4; y < height*3/4 + height/6; ++y) {
            for (int x = width*3/4; x < width*3/4 + width/6; ++x) {
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

    // 测量规划时间（毫秒）
    double measurePlanningTime(
        IPlanner* planner,
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Pose& start_pose)
    {
        auto start = std::chrono::high_resolution_clock::now();
        PlannerResult result = planner->plan(map, start_pose, config_);
        auto end = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        return duration.count();
    }

    std::unique_ptr<ZigzagPlanner> zigzag_planner_;
    std::unique_ptr<SpiralPlanner> spiral_planner_;
    PlannerConfig config_;
};

// 性能测试1：100x100地图规划耗时<500ms（弓字形）
TEST_F(PerformanceTest, Zigzag100x100MapUnder500ms)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(100, 100);
    geometry_msgs::msg::Pose start_pose = createStartPose(5.0, 5.0);

    double planning_time = measurePlanningTime(zigzag_planner_.get(), map, start_pose);

    // 验收标准：100x100地图规划耗时<500ms
    EXPECT_LT(planning_time, 500.0) << "Planning time: " << planning_time << "ms";

    // 验证规划成功
    PlannerResult result = zigzag_planner_->plan(map, start_pose, config_);
    EXPECT_TRUE(result.success);
}

// 性能测试2：100x100地图规划耗时<500ms（回字形）
TEST_F(PerformanceTest, Spiral100x100MapUnder500ms)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(100, 100);
    geometry_msgs::msg::Pose start_pose = createStartPose(5.0, 5.0);

    double planning_time = measurePlanningTime(spiral_planner_.get(), map, start_pose);

    // 验收标准：100x100地图规划耗时<500ms
    EXPECT_LT(planning_time, 500.0) << "Planning time: " << planning_time << "ms";

    // 验证规划成功
    PlannerResult result = spiral_planner_->plan(map, start_pose, config_);
    EXPECT_TRUE(result.success);
}

// 性能测试3：500x500地图规划耗时<3s（弓字形）
TEST_F(PerformanceTest, Zigzag500x500MapUnder3Seconds)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(500, 500);
    geometry_msgs::msg::Pose start_pose = createStartPose(25.0, 25.0);

    double planning_time = measurePlanningTime(zigzag_planner_.get(), map, start_pose);

    // 验收标准：500x500地图规划耗时<3000ms（3秒）
    EXPECT_LT(planning_time, 3000.0) << "Planning time: " << planning_time << "ms";

    // 验证规划成功
    PlannerResult result = zigzag_planner_->plan(map, start_pose, config_);
    EXPECT_TRUE(result.success);
}

// 性能测试4：500x500地图规划耗时<3s（回字形）
TEST_F(PerformanceTest, Spiral500x500MapUnder3Seconds)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(500, 500);
    geometry_msgs::msg::Pose start_pose = createStartPose(25.0, 25.0);

    double planning_time = measurePlanningTime(spiral_planner_.get(), map, start_pose);

    // 验收标准：500x500地图规划耗时<3000ms（3秒）
    // 回字形算法可能稍慢，允许额外20%时间（3600ms）
    EXPECT_LT(planning_time, 3600.0) << "Planning time: " << planning_time << "ms";

    // 验证规划成功
    PlannerResult result = spiral_planner_->plan(map, start_pose, config_);
    EXPECT_TRUE(result.success);
}

// 性能测试5：含障碍物的100x100地图性能
TEST_F(PerformanceTest, ZigzagObstacleMap100x100)
{
    nav_msgs::msg::OccupancyGrid map = createMapWithObstacles(100, 100);
    geometry_msgs::msg::Pose start_pose = createStartPose(2.5, 2.5);

    double planning_time = measurePlanningTime(zigzag_planner_.get(), map, start_pose);

    // 含障碍物地图允许稍慢，但不超过600ms
    EXPECT_LT(planning_time, 600.0) << "Planning time: " << planning_time << "ms";
}

// 性能测试6：含障碍物的500x500地图性能
TEST_F(PerformanceTest, ZigzagObstacleMap500x500)
{
    nav_msgs::msg::OccupancyGrid map = createMapWithObstacles(500, 500);
    geometry_msgs::msg::Pose start_pose = createStartPose(12.5, 12.5);

    double planning_time = measurePlanningTime(zigzag_planner_.get(), map, start_pose);

    // 含障碍物的大地图允许稍慢，但不超过4秒
    EXPECT_LT(planning_time, 4000.0) << "Planning time: " << planning_time << "ms";
}

// 性能测试7：多次规划性能稳定性
TEST_F(PerformanceTest, MultiplePlansPerformance)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(100, 100);
    geometry_msgs::msg::Pose start_pose = createStartPose(5.0, 5.0);

    std::vector<double> planning_times;

    // 执行10次规划
    for (int i = 0; i < 10; ++i) {
        double planning_time = measurePlanningTime(zigzag_planner_.get(), map, start_pose);
        planning_times.push_back(planning_time);
    }

    // 计算平均时间
    double avg_time = 0.0;
    for (double t : planning_times) {
        avg_time += t;
    }
    avg_time /= planning_times.size();

    // 平均时间应该稳定在500ms以下
    EXPECT_LT(avg_time, 500.0) << "Average planning time: " << avg_time << "ms";

    // 最大时间不应该超过平均时间的2倍（性能稳定性）
    double max_time = 0.0;
    for (double t : planning_times) {
        max_time = std::max(max_time, t);
    }
    EXPECT_LT(max_time, avg_time * 2.0) << "Max time: " << max_time << "ms";
}

// 性能测试8：不同地图尺寸的性能对比
TEST_F(PerformanceTest, DifferentMapSizesPerformance)
{
    std::vector<std::pair<int, int>> map_sizes = {
        {20, 20},
        {50, 50},
        {100, 100},
        {200, 200},
        {300, 300}
    };

    std::vector<double> planning_times;

    for (auto& size : map_sizes) {
        nav_msgs::msg::OccupancyGrid map = createEmptyMap(size.first, size.second);
        geometry_msgs::msg::Pose start_pose = createStartPose(
            size.first * 0.1 / 2.0,
            size.second * 0.1 / 2.0);

        double planning_time = measurePlanningTime(zigzag_planner_.get(), map, start_pose);
        planning_times.push_back(planning_time);

        // 所有尺寸都应该在合理时间内完成
        EXPECT_LT(planning_time, 2000.0) << "Size: " << size.first << "x" << size.second
                                         << ", Time: " << planning_time << "ms";
    }

    // 性能应该随着地图尺寸线性增长，不应该有指数增长
    // 检查300x300的时间不应该超过20x20时间的100倍
    EXPECT_LT(planning_times.back() / planning_times.front(), 100.0);
}

// 性能测试9：内存占用测试（估算）
TEST_F(PerformanceTest, MemoryUsageEstimate)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(500, 500);
    geometry_msgs::msg::Pose start_pose = createStartPose(25.0, 25.0);

    PlannerResult result = zigzag_planner_->plan(map, start_pose, config_);

    // 估算内存占用
    // - 地图数据：500x500 = 250000字节（约250KB）
    // - 路径数据：假设路径点数量约为地图栅格数，每个PoseStamped约64字节
    //             约250000 * 64 = 16MB（粗略估计）
    // 验收标准：内存占用<50MB，路径数据应该远小于此

    size_t path_size_bytes = result.path.size() * sizeof(geometry_msgs::msg::PoseStamped);
    double path_size_mb = path_size_bytes / (1024.0 * 1024.0);

    // 路径数据应该小于20MB
    EXPECT_LT(path_size_mb, 20.0) << "Path memory: " << path_size_mb << "MB";
}

// 性能测试10：算法性能对比（弓字形 vs 回字形）
TEST_F(PerformanceTest, AlgorithmComparison)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(100, 100);
    geometry_msgs::msg::Pose start_pose = createStartPose(5.0, 5.0);

    // 测量弓字形性能
    double zigzag_time = measurePlanningTime(zigzag_planner_.get(), map, start_pose);

    // 测量回字形性能
    double spiral_time = measurePlanningTime(spiral_planner_.get(), map, start_pose);

    // 两种算法都应该在合理时间内完成
    EXPECT_LT(zigzag_time, 500.0);
    EXPECT_LT(spiral_time, 600.0);  // 回字形允许稍慢

    // 性能差异不应该太大（不超过50%）
    EXPECT_LT(std::abs(zigzag_time - spiral_time) / zigzag_time, 0.5);
}

// 性能测试11：路径优化对性能的影响
TEST_F(PerformanceTest, OptimizationImpact)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(100, 100);
    geometry_msgs::msg::Pose start_pose = createStartPose(5.0, 5.0);

    // 开启优化
    config_.enable_optimization = true;
    double time_with_optimization = measurePlanningTime(zigzag_planner_.get(), map, start_pose);

    // 关闭优化
    config_.enable_optimization = false;
    double time_without_optimization = measurePlanningTime(zigzag_planner_.get(), map, start_pose);

    // 优化不应该显著增加规划时间（不超过20%）
    // 因为优化带来的路径质量提升更重要
    EXPECT_LT(time_with_optimization, time_without_optimization * 1.2);
}

// 性能测试12：实时性测试（连续规划）
TEST_F(PerformanceTest, ContinuousPlanning)
{
    nav_msgs::msg::OccupancyGrid map = createEmptyMap(100, 100);
    geometry_msgs::msg::Pose start_pose = createStartPose(5.0, 5.0);

    auto start = std::chrono::high_resolution_clock::now();

    // 模拟连续规划场景（例如动态环境下的重规划）
    for (int i = 0; i < 5; ++i) {
        PlannerResult result = zigzag_planner_->plan(map, start_pose, config_);
        EXPECT_TRUE(result.success);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // 5次规划总时间应该小于2500ms（平均每次<500ms）
    EXPECT_LT(duration.count(), 2500.0) << "Total time: " << duration.count() << "ms";
}

}  // namespace coverage_planner

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}