// ============================================================
// Diffbot Navigation - 集成测试：障碍物检测功能
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include "diffbot_navigation/obstacle_avoidance/obstacle_detector.hpp"
#include "diffbot_navigation/controller/diff_drive_controller.hpp"

using namespace diffbot_navigation::obstacle_avoidance;
using namespace diffbot_navigation::controller;

class ObstacleDetectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    DetectorConfig detector_config;
    detector_config.detection_range = 5.0;
    detector_config.safe_distance = 0.5;
    detector_config.min_obstacle_distance = 0.1;
    detector_ = std::make_unique<ObstacleDetector>(detector_config);
    controller_ = std::make_unique<DiffDriveController>();
  }
  void TearDown() override
  {
    detector_->clearObstacles();
  }
  std::unique_ptr<ObstacleDetector> detector_;
  std::unique_ptr<DiffDriveController> controller_;
};

// ==================== 障碍物检测场景测试 ====================

TEST_F(ObstacleDetectorTest, DetectStaticObstacle)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 2;
  scan.angle_max = M_PI / 2;
  scan.angle_increment = M_PI / 90;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  for (int i = 0; i < 90; ++i) {
    if (i >= 40 && i <= 50) {
      scan.ranges.push_back(1.0);
    } else {
      scan.ranges.push_back(10.0);
    }
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  detector_->updateLaserScan(scan_ptr);
  auto obstacles = detector_->getObstacles();
  EXPECT_FALSE(obstacles.empty());
}

TEST_F(ObstacleDetectorTest, DetectMultipleStaticObstacles)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 2;
  scan.angle_max = M_PI / 2;
  scan.angle_increment = M_PI / 180;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  for (int i = 0; i < 180; ++i) {
    if ((i >= 20 && i <= 30) || (i >= 80 && i <= 90) || (i >= 140 && i <= 150)) {
      scan.ranges.push_back(1.0);
    } else {
      scan.ranges.push_back(10.0);
    }
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  detector_->updateLaserScan(scan_ptr);
  auto obstacles = detector_->getObstacles();
  EXPECT_GE(obstacles.size(), 1u);
}

TEST_F(ObstacleDetectorTest, NoObstacle)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 2;
  scan.angle_max = M_PI / 2;
  scan.angle_increment = M_PI / 90;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  for (int i = 0; i < 90; ++i) {
    scan.ranges.push_back(10.0);
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  detector_->updateLaserScan(scan_ptr);
  EXPECT_FALSE(detector_->hasObstacles());
}

TEST_F(ObstacleDetectorTest, ClearObstacles)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 2;
  scan.angle_max = M_PI / 2;
  scan.angle_increment = M_PI / 90;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  for (int i = 0; i < 90; ++i) {
    if (i >= 40 && i <= 50) {
      scan.ranges.push_back(1.0);
    } else {
      scan.ranges.push_back(10.0);
    }
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  detector_->updateLaserScan(scan_ptr);
  EXPECT_TRUE(detector_->hasObstacles());
  detector_->clearObstacles();
  EXPECT_FALSE(detector_->hasObstacles());
}

TEST_F(ObstacleDetectorTest, IsPositionSafe)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 2;
  scan.angle_max = M_PI / 2;
  scan.angle_increment = M_PI / 90;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  for (int i = 0; i < 90; ++i) {
    if (i >= 40 && i <= 50) {
      scan.ranges.push_back(1.0);
    } else {
      scan.ranges.push_back(10.0);
    }
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  detector_->updateLaserScan(scan_ptr);
  // Far positions should be safe
  EXPECT_TRUE(detector_->isPositionSafe(5.0, 0.0));
}

TEST_F(ObstacleDetectorTest, ObstacleCount)
{
  EXPECT_EQ(detector_->getObstacleCount(), 0u);
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 2;
  scan.angle_max = M_PI / 2;
  scan.angle_increment = M_PI / 90;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  for (int i = 0; i < 90; ++i) {
    if (i >= 40 && i <= 50) {
      scan.ranges.push_back(1.0);
    } else {
      scan.ranges.push_back(10.0);
    }
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  detector_->updateLaserScan(scan_ptr);
  EXPECT_GT(detector_->getObstacleCount(), 0u);
}
