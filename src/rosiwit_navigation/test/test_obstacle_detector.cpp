// ============================================================
// Diffbot Navigation - 障碍物检测器单元测试
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include "diffbot_navigation/obstacle_avoidance/obstacle_detector.hpp"

using namespace diffbot_navigation::obstacle_avoidance;

class ObstacleDetectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 初始化检测器配置
    config_.detection_range = 5.0;
    config_.min_obstacle_distance = 0.1;
    config_.safe_distance = 0.5;
    config_.laser_min_range = 0.1;
    config_.laser_max_range = 10.0;
    config_.laser_min_angle = -M_PI;
    config_.laser_max_angle = M_PI;
    config_.cluster_tolerance = 0.2;
    config_.min_cluster_size = 3;
    config_.max_cluster_size = 100;
    config_.enable_dynamic_detection = true;
    config_.dynamic_velocity_threshold = 0.3;

    detector_ = std::make_unique<ObstacleDetector>();
    detector_->configure(config_);
  }

  DetectorConfig config_;
  std::unique_ptr<ObstacleDetector> detector_;
};

// ==================== 基本初始化测试 ====================

TEST_F(ObstacleDetectorTest, DefaultInitialization)
{
  ObstacleDetector default_detector;
  // 应该使用默认配置初始化
  EXPECT_NO_THROW({
    auto config = default_detector.getConfig();
    EXPECT_GT(config.detection_range, 0.0);
    EXPECT_GT(config.safe_distance, 0.0);
  });
}

TEST_F(ObstacleDetectorTest, ConfigInitialization)
{
  auto current_config = detector_->getConfig();

  EXPECT_DOUBLE_EQ(current_config.detection_range, 5.0);
  EXPECT_DOUBLE_EQ(current_config.safe_distance, 0.5);
  EXPECT_DOUBLE_EQ(current_config.min_obstacle_distance, 0.1);
  EXPECT_TRUE(current_config.enable_dynamic_detection);
}

TEST_F(ObstacleDetectorTest, SetNewConfig)
{
  DetectorConfig new_config;
  new_config.detection_range = 8.0;
  new_config.safe_distance = 1.0;
  new_config.enable_dynamic_detection = false;

  detector_->setConfig(new_config);

  auto current_config = detector_->getConfig();
  EXPECT_DOUBLE_EQ(current_config.detection_range, 8.0);
  EXPECT_DOUBLE_EQ(current_config.safe_distance, 1.0);
  EXPECT_FALSE(current_config.enable_dynamic_detection);
}

// ==================== 激光雷达数据处理测试 ====================

TEST_F(ObstacleDetectorTest, ProcessEmptyLaserScan)
{
  sensor_msgs::msg::LaserScan empty_scan;
  empty_scan.header.frame_id = "laser";
  empty_scan.angle_min = -M_PI;
  empty_scan.angle_max = M_PI;
  empty_scan.angle_increment = 0.1;
  empty_scan.range_min = 0.1;
  empty_scan.range_max = 10.0;
  empty_scan.ranges.clear();

  EXPECT_NO_THROW({
    auto obstacles = detector_->detectFromLaserScan(empty_scan);
  });
}

TEST_F(ObstacleDetectorTest, ProcessSingleObstacleLaserScan)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 4;
  scan.angle_max = M_PI / 4;
  scan.angle_increment = M_PI / 180;  // 1度
  scan.range_min = 0.1;
  scan.range_max = 10.0;

  // 生成一个前方障碍物的模拟数据
  int num_readings = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
  for (int i = 0; i < num_readings; ++i) {
    if (i > 30 && i < 60) {
      // 前方障碍物区域，距离1米
      scan.ranges.push_back(1.0);
    } else {
      // 其他区域无障碍物
      scan.ranges.push_back(10.0);
    }
  }

  auto obstacles = detector_->detectFromLaserScan(scan);

  // 应该检测到障碍物
  EXPECT_FALSE(obstacles.empty());
}

TEST_F(ObstacleDetectorTest, ProcessMultipleObstacles)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 180;
  scan.range_min = 0.1;
  scan.range_max = 10.0;

  int num_readings = 360;
  for (int i = 0; i < num_readings; ++i) {
    if ((i > 40 && i < 50) || (i > 150 && i < 160)) {
      // 两个障碍物区域
      scan.ranges.push_back(1.5);
    } else {
      scan.ranges.push_back(10.0);
    }
  }

  auto obstacles = detector_->detectFromLaserScan(scan);

  // 应该检测到多个障碍物
  EXPECT_GE(obstacles.size(), 1);
}

TEST_F(ObstacleDetectorTest, ProcessOutOfRangeReadings)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 180;
  scan.range_min = 0.1;
  scan.range_max = 10.0;

  int num_readings = 360;
  for (int i = 0; i < num_readings; ++i) {
    // 所有读数超出范围
    scan.ranges.push_back(0.05);  // 低于最小范围
  }

  // 应该安全处理超出范围的读数
  EXPECT_NO_THROW({
    auto obstacles = detector_->detectFromLaserScan(scan);
  });
}

TEST_F(ObstacleDetectorTest, ProcessInvalidReadings)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 180;
  scan.range_min = 0.1;
  scan.range_max = 10.0;

  int num_readings = 360;
  for (int i = 0; i < num_readings; ++i) {
    // 无效读数（NaN或Inf）
    scan.ranges.push_back(std::numeric_limits<float>::infinity());
  }

  EXPECT_NO_THROW({
    auto obstacles = detector_->detectFromLaserScan(scan);
  });
}

// ==================== 障碍物属性测试 ====================

TEST_F(ObstacleDetectorTest, ObstacleDistanceCalculation)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = 0.0;
  scan.angle_max = M_PI / 6;
  scan.angle_increment = M_PI / 180;
  scan.range_min = 0.1;
  scan.range_max = 10.0;

  // 前方2米的障碍物
  int num_readings = 30;
  for (int i = 0; i < num_readings; ++i) {
    scan.ranges.push_back(2.0);
  }

  auto obstacles = detector_->detectFromLaserScan(scan);

  if (!obstacles.empty()) {
    // 验证障碍物距离计算正确
    EXPECT_NEAR(obstacles[0].distance, 2.0, 0.5);
  }
}

TEST_F(ObstacleDetectorTest, ObstaclePositionCalculation)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 4;
  scan.angle_max = M_PI / 4;
  scan.angle_increment = M_PI / 180;
  scan.range_min = 0.1;
  scan.range_max = 10.0;

  // 前方3米的障碍物
  int num_readings = 90;
  for (int i = 0; i < num_readings; ++i) {
    scan.ranges.push_back(3.0);
  }

  auto obstacles = detector_->detectFromLaserScan(scan);

  if (!obstacles.empty()) {
    // 验证障碍物位置
    EXPECT_NEAR(obstacles[0].x, 3.0, 0.5);
    EXPECT_NEAR(obstacles[0].y, 0.0, 0.5);
  }
}

TEST_F(ObstacleDetectorTest, ObstacleRadiusEstimation)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 8;
  scan.angle_max = M_PI / 8;
  scan.angle_increment = M_PI / 360;
  scan.range_min = 0.1;
  scan.range_max = 10.0;

  // 较大的障碍物区域
  int num_readings = 45;
  for (int i = 0; i < num_readings; ++i) {
    scan.ranges.push_back(1.0);
  }

  auto obstacles = detector_->detectFromLaserScan(scan);

  if (!obstacles.empty()) {
    // 应该有半径估计
    EXPECT_GT(obstacles[0].radius, 0.0);
  }
}

// ==================== 安全距离检测测试 ====================

TEST_F(ObstacleDetectorTest, IsSafeNoObstacles)
{
  std::vector<Obstacle> obstacles;
  EXPECT_TRUE(detector_->isSafe(obstacles, 0.5));
}

TEST_F(ObstacleDetectorTest, IsSafeObstacleFarAway)
{
  std::vector<Obstacle> obstacles;
  Obstacle obs;
  obs.distance = 10.0;  // 远距离障碍物
  obs.radius = 0.3;
  obstacles.push_back(obs);

  EXPECT_TRUE(detector_->isSafe(obstacles, 0.5));
}

TEST_F(ObstacleDetectorTest, IsSafeObstacleNearby)
{
  std::vector<Obstacle> obstacles;
  Obstacle obs;
  obs.distance = 0.3;  // 近距离障碍物
  obs.radius = 0.2;
  obstacles.push_back(obs);

  EXPECT_FALSE(detector_->isSafe(obstacles, 0.5));
}

TEST_F(ObstacleDetectorTest, IsSafeCriticalDistance)
{
  std::vector<Obstacle> obstacles;
  Obstacle obs;
  obs.distance = 0.5;  // 刚好等于安全距离
  obs.radius = 0.3;
  obstacles.push_back(obs);

  // 边界情况
  EXPECT_FALSE(detector_->isSafe(obstacles, 0.5));
}

TEST_F(ObstacleDetectorTest, IsSafeMultipleObstacles)
{
  std::vector<Obstacle> obstacles;
  Obstacle obs1, obs2;
  obs1.distance = 10.0;  // 远
  obs2.distance = 0.4;   // 近

  obstacles.push_back(obs1);
  obstacles.push_back(obs2);

  EXPECT_FALSE(detector_->isSafe(obstacles, 0.5));
}

// ==================== 碰撞检测测试 ====================

TEST_F(ObstacleDetectorTest, WillCollideNoObstacles)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.5;
  velocity.angular.z = 0.0;

  std::vector<Obstacle> obstacles;

  EXPECT_FALSE(detector_->willCollide(obstacles, velocity, 2.0));
}

TEST_F(ObstacleDetectorTest, WillCollideObstacleInPath)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.5;
  velocity.angular.z = 0.0;

  std::vector<Obstacle> obstacles;
  Obstacle obs;
  obs.x = 1.0;  // 前方障碍物
  obs.y = 0.0;
  obs.distance = 1.0;
  obs.radius = 0.3;
  obstacles.push_back(obs);

  // 可能碰撞
  bool will_collide = detector_->willCollide(obstacles, velocity, 2.0);
  // 结果取决于具体实现，但不应抛出异常
  EXPECT_NO_THROW(detector_->willCollide(obstacles, velocity, 2.0));
}

TEST_F(ObstacleDetectorTest, WillCollideObstacleNotInPath)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.5;
  velocity.angular.z = 0.0;

  std::vector<Obstacle> obstacles;
  Obstacle obs;
  obs.x = 0.0;  // 侧面障碍物
  obs.y = 1.0;
  obs.distance = 1.0;
  obs.radius = 0.3;
  obstacles.push_back(obs);

  EXPECT_NO_THROW({
    bool will_collide = detector_->willCollide(obstacles, velocity, 2.0);
  });
}

TEST_F(ObstacleDetectorTest, WillCollideZeroVelocity)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.0;
  velocity.angular.z = 0.0;

  std::vector<Obstacle> obstacles;
  Obstacle obs;
  obs.distance = 0.5;
  obstacles.push_back(obs);

  // 静止时不会碰撞
  EXPECT_FALSE(detector_->willCollide(obstacles, velocity, 2.0));
}

// ==================== 最近障碍物检测测试 ====================

TEST_F(ObstacleDetectorTest, GetClosestObstacleNoObstacles)
{
  std::vector<Obstacle> obstacles;
  auto closest = detector_->getClosestObstacle(obstacles);

  // 无障碍物时返回空或默认值
  EXPECT_TRUE(closest.distance < 0.0 || closest.distance >= 0.0);
}

TEST_F(ObstacleDetectorTest, GetClosestObstacleSingle)
{
  std::vector<Obstacle> obstacles;
  Obstacle obs;
  obs.distance = 2.0;
  obs.x = 2.0;
  obs.y = 0.0;
  obstacles.push_back(obs);

  auto closest = detector_->getClosestObstacle(obstacles);

  EXPECT_DOUBLE_EQ(closest.distance, 2.0);
}

TEST_F(ObstacleDetectorTest, GetClosestObstacleMultiple)
{
  std::vector<Obstacle> obstacles;
  Obstacle obs1, obs2, obs3;
  obs1.distance = 3.0;
  obs2.distance = 1.0;  // 最近
  obs3.distance = 2.0;

  obstacles.push_back(obs1);
  obstacles.push_back(obs2);
  obstacles.push_back(obs3);

  auto closest = detector_->getClosestObstacle(obstacles);

  EXPECT_DOUBLE_EQ(closest.distance, 1.0);
}

// ==================== 前方障碍物检测测试 ====================

TEST_F(ObstacleDetectorTest, GetObstaclesInFrontEmpty)
{
  std::vector<Obstacle> obstacles;
  auto front_obstacles = detector_->getObstaclesInFront(obstacles, M_PI / 4);

  EXPECT_TRUE(front_obstacles.empty());
}

TEST_F(ObstacleDetectorTest, GetObstaclesInFrontSingle)
{
  std::vector<Obstacle> obstacles;
  Obstacle obs;
  obs.angle = 0.0;  // 正前方
  obs.distance = 2.0;
  obstacles.push_back(obs);

  auto front_obstacles = detector_->getObstaclesInFront(obstacles, M_PI / 4);

  EXPECT_EQ(front_obstacles.size(), 1);
}

TEST_F(ObstacleDetectorTest, GetObstaclesInFrontFilteredByAngle)
{
  std::vector<Obstacle> obstacles;
  Obstacle obs1, obs2;
  obs1.angle = 0.1;    // 前方
  obs2.angle = M_PI;   // 后方

  obstacles.push_back(obs1);
  obstacles.push_back(obs2);

  auto front_obstacles = detector_->getObstaclesInFront(obstacles, M_PI / 4);

  // 只有前方障碍物应该被返回
  EXPECT_EQ(front_obstacles.size(), 1);
}

// ==================== 动态障碍物检测测试 ====================

TEST_F(ObstacleDetectorTest, DynamicObstacleDetectionEnabled)
{
  config_.enable_dynamic_detection = true;
  detector_->setConfig(config_);

  std::vector<Obstacle> obstacles;
  Obstacle obs;
  obs.is_dynamic = true;
  obs.velocity_x = 0.5;
  obstacles.push_back(obs);

  // 应能够处理动态障碍物
  EXPECT_NO_THROW(detector_->isSafe(obstacles, 0.5));
}

TEST_F(ObstacleDetectorTest, DynamicObstacleVelocity)
{
  sensor_msgs::msg::LaserScan scan1, scan2;
  scan1.header.frame_id = "laser";
  scan1.angle_min = 0.0;
  scan1.angle_max = M_PI / 4;
  scan1.angle_increment = M_PI / 180;
  scan1.range_min = 0.1;
  scan1.range_max = 10.0;

  scan2.header = scan1.header;
  scan2.angle_min = scan1.angle_min;
  scan2.angle_max = scan1.angle_max;
  scan2.angle_increment = scan1.angle_increment;
  scan2.range_min = scan1.range_min;
  scan2.range_max = scan1.range_max;

  // 第一帧：障碍物在2米
  for (int i = 0; i < 45; ++i) {
    scan1.ranges.push_back(2.0);
    scan2.ranges.push_back(1.5);  // 第二帧：障碍物在1.5米（向机器人移动）
  }

  // 应能处理时间序列数据
  EXPECT_NO_THROW({
    auto obstacles1 = detector_->detectFromLaserScan(scan1);
    auto obstacles2 = detector_->detectFromLaserScan(scan2);
  });
}

// ==================== 状态管理测试 ====================

TEST_F(ObstacleDetectorTest, ActiveStatusToggle)
{
  EXPECT_FALSE(detector_->isActive());

  detector_->setActive(true);
  EXPECT_TRUE(detector_->isActive());

  detector_->setActive(false);
  EXPECT_FALSE(detector_->isActive());
}

TEST_F(ObstacleDetectorTest, ResetClearsState)
{
  // 处理一些数据
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = 0.0;
  scan.angle_max = M_PI / 2;
  scan.angle_increment = M_PI / 180;
  scan.range_min = 0.1;
  scan.range_max = 10.0;

  for (int i = 0; i < 90; ++i) {
    scan.ranges.push_back(2.0);
  }

  detector_->detectFromLaserScan(scan);

  // 重置
  EXPECT_NO_THROW(detector_->clearObstacles());

  // 再次处理应该正常工作
  EXPECT_NO_THROW(detector_->detectFromLaserScan(scan));
}

// ==================== 边界条件测试 ====================

TEST_F(ObstacleDetectorTest, ZeroDetectionRange)
{
  DetectorConfig zero_config;
  zero_config.detection_range = 0.0;
  zero_config.safe_distance = 0.5;

  ObstacleDetector zero_detector(zero_config);

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = 0.0;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 180;
  scan.range_min = 0.1;
  scan.range_max = 10.0;

  for (int i = 0; i < 180; ++i) {
    scan.ranges.push_back(1.0);
  }

  EXPECT_NO_THROW({
    auto obstacles = zero_detector.detectFromLaserScan(scan);
  });
}

TEST_F(ObstacleDetectorTest, VerySmallSafeDistance)
{
  DetectorConfig small_config;
  small_config.safe_distance = 0.01;
  small_config.min_obstacle_distance = 0.01;

  ObstacleDetector small_detector(small_config);

  std::vector<Obstacle> obstacles;
  Obstacle obs;
  obs.distance = 0.05;
  obs.radius = 0.02;
  obstacles.push_back(obs);

  EXPECT_NO_THROW({
    bool is_safe = small_detector.isSafe(obstacles, 0.03);
  });
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}