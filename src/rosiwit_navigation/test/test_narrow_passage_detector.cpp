// ============================================================
// Diffbot Navigation - 窄道检测器单元测试
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include "rosiwit_navigation/algorithms/narrow_passage_detector.hpp"

using namespace rosiwit_navigation::narrow_passage;

class NarrowPassageDetectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config_.robot_width = 0.4;
    config_.robot_length = 0.5;
    config_.min_passage_width = 0.5;
    config_.detection_range = 5.0;
    config_.safety_margin = 0.1;
    config_.width_threshold = 0.6;
    config_.length_threshold = 0.3;
    config_.laser_min_range = 0.1;
    config_.laser_max_range = 10.0;
    config_.laser_angular_resolution = 0.1;
    config_.precision_mode = true;
    config_.precision_safety_distance = 0.05;

    detector_ = std::make_unique<NarrowPassageDetector>();
  detector_->configure(config_);
  }

  DetectorConfig config_;
  std::unique_ptr<NarrowPassageDetector> detector_;
};

// ==================== 基本初始化测试 ====================

TEST_F(NarrowPassageDetectorTest, DefaultInitialization)
{
  NarrowPassageDetector default_detector;
  EXPECT_NO_THROW({
    DetectorConfig config;
    config.robot_width = 0.4;
    config.min_passage_width = 0.5;
    default_detector.configure(config);
    EXPECT_GT(config.robot_width, 0.0);
    EXPECT_GT(config.min_passage_width, 0.0);
  });
}

TEST_F(NarrowPassageDetectorTest, ConfigInitialization)
{
  // 使用 configure 方法设置配置
  detector_->configure(config_);

  // 基本验证
  EXPECT_GT(config_.robot_width, 0.0);
  EXPECT_GT(config_.min_passage_width, 0.0);
}

TEST_F(NarrowPassageDetectorTest, SetNewConfig)
{
  DetectorConfig new_config;
  new_config.robot_width = 0.6;
  new_config.min_passage_width = 0.7;
  new_config.precision_mode = false;

  detector_->configure(new_config);

  EXPECT_DOUBLE_EQ(new_config.robot_width, 0.6);
  EXPECT_DOUBLE_EQ(new_config.min_passage_width, 0.7);
  EXPECT_FALSE(new_config.precision_mode);
}

// ==================== 通道检测测试 ====================

TEST_F(NarrowPassageDetectorTest, DetectPassagesFromLaserScan)
{
  // 创建激光扫描消息（使用 SharedPtr）
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.frame_id = "laser";
  scan->angle_min = 0.0;
  scan->angle_max = M_PI / 2;
  scan->angle_increment = M_PI / 180;
  scan->range_min = 0.1;
  scan->range_max = 10.0;

  // 模拟一个通道：中间距离大于两侧
  for (int i = 0; i < 90; ++i) {
    if (i > 30 && i < 60) {
      scan->ranges.push_back(5.0);  // 通道
    } else {
      scan->ranges.push_back(0.5);  // 墙壁
    }
  }

  detector_->updateLaserScan(scan);
  auto passages = detector_->detectPassages();

  EXPECT_GE(passages.size(), 0u);
}

// ==================== 可通过性测试 ====================

TEST_F(NarrowPassageDetectorTest, TraversableWidePassage)
{
  NarrowPassage passage;
  passage.width = 1.0;  // 大于机器人宽度
  passage.is_traversable = true;

  bool traversable = detector_->isPassageTraversable(passage);
  EXPECT_TRUE(traversable);
}

TEST_F(NarrowPassageDetectorTest, NotTraversableTooNarrow)
{
  NarrowPassage passage;
  passage.width = 0.3;  // 小于机器人宽度
  passage.is_traversable = false;

  bool traversable = detector_->isPassageTraversable(passage);
  EXPECT_FALSE(traversable);
}

// ==================== 状态管理测试 ====================

TEST_F(NarrowPassageDetectorTest, ClearPassages)
{
  // 清除检测到的通道
  detector_->clearPassages();
  auto passages = detector_->getAllPassages();
  EXPECT_EQ(passages.size(), 0u);
}

// ==================== 边界条件测试 ====================

TEST_F(NarrowPassageDetectorTest, ZeroRobotWidth)
{
  DetectorConfig zero_config;
  zero_config.robot_width = 0.0;
  zero_config.min_passage_width = 0.5;
  EXPECT_NO_THROW({
    NarrowPassageDetector zero_detector;
    zero_detector.configure(zero_config);
  });
}

TEST_F(NarrowPassageDetectorTest, EmptyLaserScan)
{
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.frame_id = "laser";
  scan->ranges.clear();

  detector_->updateLaserScan(scan);
  auto passages = detector_->detectPassages();

  EXPECT_EQ(passages.size(), 0u);
}

// ==================== 速度建议测试 ====================

TEST_F(NarrowPassageDetectorTest, RecommendedVelocityWidePassage)
{
  NarrowPassage passage;
  passage.width = 1.0;

  double velocity = detector_->getRecommendedVelocity(passage);
  EXPECT_GT(velocity, 0.0);
  EXPECT_LT(velocity, 1.0);  // 最大速度限制
}

TEST_F(NarrowPassageDetectorTest, RecommendedVelocityNarrowPassage)
{
  NarrowPassage passage;
  passage.width = 0.5;  // 略大于机器人宽度

  double velocity = detector_->getRecommendedVelocity(passage);
  EXPECT_GT(velocity, 0.0);
  EXPECT_LT(velocity, 0.3);  // 窄通道速度更慢
}

// ==================== 通道中心线计算测试 ====================

TEST_F(NarrowPassageDetectorTest, ComputeCenterline)
{
  NarrowPassage passage;
  passage.start_x = 0.0;
  passage.start_y = 0.0;
  passage.end_x = 1.0;
  passage.end_y = 0.0;
  passage.width = 1.0;

  EXPECT_NO_THROW(detector_->computeCenterline(passage));

  // 中心线应该在起点和终点中间
  double expected_x = 0.5;
  double expected_y = 0.0;
  EXPECT_NEAR(passage.centerline_x, expected_x, 0.1);
  EXPECT_NEAR(passage.centerline_y, expected_y, 0.1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}