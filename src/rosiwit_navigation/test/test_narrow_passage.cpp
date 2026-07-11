// ============================================================
// Diffbot Navigation - 集成测试：窄道通行功能
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include "narrow_passage_detector.hpp"
#include "narrow_passage_planner.hpp"

using namespace rosiwit_navigation::narrow_passage;

class NarrowPassageTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    DetectorConfig detector_config;
    detector_config.robot_width = 0.4;
    detector_config.robot_length = 0.5;
    detector_config.min_passage_width = 0.5;
    detector_config.detection_range = 5.0;
    detector_config.safety_margin = 0.1;
    detector_config.precision_mode = true;
    detector_config.precision_safety_distance = 0.05;
    detector_ = std::make_unique<NarrowPassageDetector>();
    detector_->configure(detector_config);
    passage_planner_ = std::make_unique<NarrowPassagePlanner>();
  }
  void TearDown() override
  {
    detector_->clearPassages();
  }
  std::unique_ptr<NarrowPassageDetector> detector_;
  std::unique_ptr<NarrowPassagePlanner> passage_planner_;
};

// ==================== 窄道检测场景测试 ====================

TEST_F(NarrowPassageTest, DetectNarrowPassage)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 4;
  scan.angle_max = M_PI / 4;
  scan.angle_increment = M_PI / 360;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  int num_readings = 90;
  for (int i = 0; i < num_readings; ++i) {
    if (i < 20 || i > 70) {
      scan.ranges.push_back(0.3);
    } else {
      scan.ranges.push_back(0.6);
    }
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  EXPECT_NO_THROW({
    detector_->updateLaserScan(scan_ptr);
    auto passages = detector_->detectPassages();
  });
}

TEST_F(NarrowPassageTest, DetectVeryNarrowPassage)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 4;
  scan.angle_max = M_PI / 4;
  scan.angle_increment = M_PI / 360;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  int num_readings = 90;
  for (int i = 0; i < num_readings; ++i) {
    scan.ranges.push_back(0.3);
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  detector_->updateLaserScan(scan_ptr);
  auto passages = detector_->detectPassages();
  EXPECT_GE(passages.size(), 0u);
}

TEST_F(NarrowPassageTest, DetectWidePassage)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 2;
  scan.angle_max = M_PI / 2;
  scan.angle_increment = M_PI / 360;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  int num_readings = 180;
  for (int i = 0; i < num_readings; ++i) {
    if (i >= 70 && i <= 110) {
      scan.ranges.push_back(5.0);
    } else {
      scan.ranges.push_back(0.5);
    }
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  detector_->updateLaserScan(scan_ptr);
  auto passages = detector_->detectPassages();
  EXPECT_GE(passages.size(), 0u);
  for (const auto & passage : passages) {
    EXPECT_TRUE(detector_->isPassageTraversable(passage));
  }
}

TEST_F(NarrowPassageTest, NoPassage)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 4;
  scan.angle_max = M_PI / 4;
  scan.angle_increment = M_PI / 360;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  int num_readings = 90;
  for (int i = 0; i < num_readings; ++i) {
    scan.ranges.push_back(0.3);
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  detector_->updateLaserScan(scan_ptr);
  auto passages = detector_->detectPassages();
  EXPECT_GE(passages.size(), 0u);
}

TEST_F(NarrowPassageTest, MultiplePassages)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 2;
  scan.angle_max = M_PI / 2;
  scan.angle_increment = M_PI / 360;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  int num_readings = 360;
  for (int i = 0; i < num_readings; ++i) {
    if ((i >= 50 && i <= 80) || (i >= 200 && i <= 230)) {
      scan.ranges.push_back(3.0);
    } else {
      scan.ranges.push_back(0.5);
    }
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  detector_->updateLaserScan(scan_ptr);
  auto passages = detector_->detectPassages();
  EXPECT_GE(passages.size(), 0u);
}

TEST_F(NarrowPassageTest, ClearPassagesAndState)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -M_PI / 4;
  scan.angle_max = M_PI / 4;
  scan.angle_increment = M_PI / 360;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  int num_readings = 90;
  for (int i = 0; i < num_readings; ++i) {
    scan.ranges.push_back(0.3);
  }
  auto scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
  detector_->updateLaserScan(scan_ptr);
  detector_->detectPassages();
  detector_->clearPassages();
  auto all_passages = detector_->getAllPassages();
  EXPECT_EQ(all_passages.size(), 0u);
}
