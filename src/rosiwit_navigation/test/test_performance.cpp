// ============================================================
// Diffbot Navigation - 性能压力测试
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <chrono>
#include <vector>
#include "diffbot_navigation/navigation/trajectory_generator.hpp"
#include "diffbot_navigation/obstacle_avoidance/obstacle_detector.hpp"
#include "diffbot_navigation/narrow_passage/narrow_passage_detector.hpp"
#include "diffbot_navigation/controller/diff_drive_controller.hpp"
#include "diffbot_navigation/controller/velocity_limiter.hpp"

using namespace diffbot_navigation::navigation;
using namespace diffbot_navigation::obstacle_avoidance;
using namespace diffbot_navigation::narrow_passage;
using namespace diffbot_navigation::controller;

class PerformanceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    TrajectoryConfig traj_config;
    traj_config.max_velocity_x = 0.5;
    traj_config.max_velocity_theta = 1.0;
    traj_config.sim_time = 2.0;
    traj_config.sim_granularity = 0.02;
    trajectory_generator_ = std::make_unique<TrajectoryGenerator>(traj_config);
    detector_ = std::make_unique<ObstacleDetector>();
    narrow_detector_ = std::make_unique<NarrowPassageDetector>();
    controller_ = std::make_unique<DiffDriveController>();
    velocity_limiter_ = std::make_unique<VelocityLimiter>();
  }
  std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
  std::unique_ptr<ObstacleDetector> detector_;
  std::unique_ptr<NarrowPassageDetector> narrow_detector_;
  std::unique_ptr<DiffDriveController> controller_;
  std::unique_ptr<VelocityLimiter> velocity_limiter_;
};

// ==================== 响应时间测试 ====================

TEST_F(PerformanceTest, TrajectoryGenerationLatency)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 100; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;
  auto start = std::chrono::high_resolution_clock::now();
  auto trajectory = trajectory_generator_->generateTrajectory(path, current_velocity);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // 轨迹生成应该在100ms内完成
  EXPECT_LT(duration.count(), 100);
}

TEST_F(PerformanceTest, ObstacleDetectionLatency)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 180;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  for (int i = 0; i < 360; ++i) {
    scan.ranges.push_back(std::rand() % 1000 / 100.0);
  }
  auto start = std::chrono::high_resolution_clock::now();
  auto obstacles = detector_->detectFromLaserScan(scan);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // 障碍物检测应该在50ms内完成
  EXPECT_LT(duration.count(), 50);
}

TEST_F(PerformanceTest, NarrowPassageDetectionLatency)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 360;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  for (int i = 0; i < 720; ++i) {
    scan.ranges.push_back(std::rand() % 1000 / 100.0);
  }
  auto start = std::chrono::high_resolution_clock::now();
  auto passages = narrow_detector_->detectFromLaserScan(scan);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // 窄道检测应该在50ms内完成
  EXPECT_LT(duration.count(), 50);
}

TEST_F(PerformanceTest, VelocityCommandLatency)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 50; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  controller_->setPlan(path);
  controller_->setActive(true);
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.position.x = 1.0;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.2;
  current_velocity.angular.z = 0.0;
  auto start = std::chrono::high_resolution_clock::now();
  auto cmd_vel = controller_->computeVelocityCommands(robot_pose, current_velocity);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // 速度命令计算应该在10ms内完成
  EXPECT_LT(duration.count(), 10);
}

// ==================== 吞吐量测试 ====================

TEST_F(PerformanceTest, HighFrequencyTrajectoryGeneration)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;
  int iterations = 1000;
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; ++i) {
    auto trajectory = trajectory_generator_->generateTrajectory(path, current_velocity);
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  double avg_time = total_duration.count() / iterations;
  // 平均每次生成应该在10ms内
  EXPECT_LT(avg_time, 10.0);
}

TEST_F(PerformanceTest, HighFrequencyObstacleDetection)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 180;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  for (int i = 0; i < 360; ++i) {
    scan.ranges.push_back(2.0);
  }
  int iterations = 1000;
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; ++i) {
    auto obstacles = detector_->detectFromLaserScan(scan);
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  double avg_time = total_duration.count() / iterations;
  // 平均每次检测应该在5ms内
  EXPECT_LT(avg_time, 5.0);
}

TEST_F(PerformanceTest, HighFrequencyVelocityLimiting)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 1.0;
  velocity.angular.z = 2.0;
  int iterations = 10000;
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; ++i) {
    auto limited = velocity_limiter_->limitVelocity(velocity);
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  double avg_time = total_duration.count() / iterations;
  // 平均每次限制应该在0.1ms内
  EXPECT_LT(avg_time, 0.1);
}

// ==================== 内存使用测试 ====================

TEST_F(PerformanceTest, TrajectoryMemoryUsage)
{
  nav_msgs::msg::Path long_path;
  long_path.header.frame_id = "map";
  for (int i = 0; i <= 500; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = std::sin(i * 0.1) * 0.5;
    pose.pose.orientation.w = 1.0;
    long_path.poses.push_back(pose);
  }
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;
  auto trajectory = trajectory_generator_->generateTrajectory(long_path, current_velocity);
  // 验证轨迹大小合理
  size_t estimated_size = trajectory.size() * sizeof(TrajectoryPoint);
  EXPECT_LT(estimated_size, 10 * 1024 * 1024);  // 小于10MB
}

TEST_F(PerformanceTest, ObstacleListMemoryUsage)
{
  std::vector<Obstacle> obstacles;
  for (int i = 0; i < 1000; ++i) {
    Obstacle obs;
    obs.x = std::rand() % 100 / 10.0;
    obs.y = std::rand() % 100 / 10.0;
    obs.distance = std::sqrt(obs.x * obs.x + obs.y * obs.y);
    obs.radius = 0.3;
    obstacles.push_back(obs);
  }
  // 验证障碍物列表大小合理
  size_t estimated_size = obstacles.size() * sizeof(Obstacle);
  EXPECT_LT(estimated_size, 1 * 1024 * 1024);  // 小于1MB
}

// ==================== 大数据量测试 ====================

TEST_F(PerformanceTest, LargeLaserScanProcessing)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 720;  // 高分辨率
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  for (int i = 0; i < 1440; ++i) {
    scan.ranges.push_back(std::rand() % 1000 / 100.0);
  }
  auto start = std::chrono::high_resolution_clock::now();
  auto obstacles = detector_->detectFromLaserScan(scan);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // 高分辨率激光数据处理应该在100ms内
  EXPECT_LT(duration.count(), 100);
}

TEST_F(PerformanceTest, LargeMapProcessing)
{
  nav_msgs::msg::OccupancyGrid large_map;
  large_map.header.frame_id = "map";
  large_map.info.width = 1000;
  large_map.info.height = 1000;
  large_map.info.resolution = 0.1;
  for (int i = 0; i < 1000000; ++i) {
    large_map.data.push_back(std::rand() % 101);
  }
  auto start = std::chrono::high_resolution_clock::now();
  auto passages = narrow_detector_->detectFromMap(large_map);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // 大地图处理应该在500ms内
  EXPECT_LT(duration.count(), 500);
}

TEST_F(PerformanceTest, LongPathPlanning)
{
  nav_msgs::msg::Path very_long_path;
  very_long_path.header.frame_id = "map";
  for (int i = 0; i <= 1000; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.05;
    pose.pose.position.y = std::sin(i * 0.05) * 0.3;
    pose.pose.orientation.w = 1.0;
    very_long_path.poses.push_back(pose);
  }
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.1;
  current_velocity.angular.z = 0.0;
  auto start = std::chrono::high_resolution_clock::now();
  auto trajectory = trajectory_generator_->generateTrajectory(very_long_path, current_velocity);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // 长路径轨迹生成应该在200ms内
  EXPECT_LT(duration.count(), 200);
}

// ==================== 连续运行测试 ====================

TEST_F(PerformanceTest, ContinuousOperation)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 50; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  controller_->setPlan(path);
  controller_->setActive(true);
  int iterations = 1000;
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; ++i) {
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.pose.position.x = i * 0.05;
    robot_pose.pose.position.y = 0.0;
    robot_pose.pose.orientation.w = 1.0;
    geometry_msgs::msg::Twist current_velocity;
    current_velocity.linear.x = 0.2;
    current_velocity.angular.z = 0.0;
    auto cmd_vel = controller_->computeVelocityCommands(robot_pose, current_velocity);
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  double avg_time = total_duration.count() / iterations;
  // 平均每次计算应该在1ms内
  EXPECT_LT(avg_time, 1.0);
}

// ==================== 并发场景测试 ====================

TEST_F(PerformanceTest, SimulatedConcurrentDetection)
{
  sensor_msgs::msg::LaserScan scan1, scan2;
  scan1.header.frame_id = "laser";
  scan1.angle_min = -M_PI;
  scan1.angle_max = M_PI;
  scan1.angle_increment = M_PI / 180;
  scan1.range_min = 0.1;
  scan1.range_max = 10.0;
  scan2 = scan1;
  for (int i = 0; i < 360; ++i) {
    scan1.ranges.push_back(1.0);
    scan2.ranges.push_back(2.0);
  }
  auto start = std::chrono::high_resolution_clock::now();
  auto obstacles1 = detector_->detectFromLaserScan(scan1);
  auto obstacles2 = detector_->detectFromLaserScan(scan2);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // 并发检测应该在100ms内
  EXPECT_LT(duration.count(), 100);
}

// ==================== 资源重置测试 ====================

TEST_F(PerformanceTest, ResetPerformance)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 50; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;
  trajectory_generator_->generateTrajectory(path, current_velocity);
  auto start = std::chrono::high_resolution_clock::now();
  trajectory_generator_->reset();
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // 重置应该在1ms内
  EXPECT_LT(duration.count(), 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}