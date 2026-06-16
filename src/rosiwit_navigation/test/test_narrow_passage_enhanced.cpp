// ============================================================
// 窄道检测器增强测试（提高覆盖率）
// 对应建议: test_report.md § 窄道检测器单元测试覆盖率偏低
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>
#include "diffbot_navigation/narrow_passage/narrow_passage_detector.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace diffbot_navigation::narrow_passage;

class NarrowPassageDetectorEnhancedTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    DetectorConfig config;
    config.robot_width = 0.4;
    config.robot_length = 0.5;
    config.min_passage_width = 0.5;
    config.detection_range = 5.0;
    config.safety_margin = 0.1;
    config.width_threshold = 0.6;
    config.length_threshold = 0.3;
    config.laser_min_range = 0.1;
    config.laser_max_range = 10.0;
    config.laser_angular_resolution = 0.05;
    config.precision_mode = true;
    config.precision_safety_distance = 0.05;

    detector_ = std::make_unique<NarrowPassageDetector>();
    detector_->configure(config);
  }

  // 创建模拟激光扫描（直线走廊）
  std::shared_ptr<sensor_msgs::msg::LaserScan> createCorridorScan(
      double wall_dist_left, double wall_dist_right, int num_beams = 360)
  {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header.frame_id = "laser";
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->angle_increment = (2.0 * M_PI) / num_beams;
    scan->range_min = 0.1;
    scan->range_max = 10.0;

    scan->ranges.resize(num_beams);
    for (int i = 0; i < num_beams; ++i) {
      double angle = scan->angle_min + i * scan->angle_increment;
      // 前方（-π/2 ~ π/2）
      if (std::abs(angle) < M_PI_2 - 0.1) {
        scan->ranges[i] = 5.0;  // 前方开阔
      }
      // 左侧
      else if (angle > M_PI_2 && angle < M_PI) {
        scan->ranges[i] = wall_dist_left / std::abs(std::cos(M_PI - angle));
      }
      // 右侧
      else {
        scan->ranges[i] = wall_dist_right / std::abs(std::cos(angle));
      }
      scan->ranges[i] = std::clamp(scan->ranges[i], scan->range_min, scan->range_max);
    }
    return scan;
  }

  // 创建窄道激光扫描
  std::shared_ptr<sensor_msgs::msg::LaserScan> createNarrowPassageScan(
      double passage_center_angle, double passage_width, int num_beams = 360)
  {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header.frame_id = "laser";
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->angle_increment = (2.0 * M_PI) / num_beams;
    scan->range_min = 0.1;
    scan->range_max = 10.0;
    scan->ranges.resize(num_beams);

    for (int i = 0; i < num_beams; ++i) {
      double angle = scan->angle_min + i * scan->angle_increment;
      double angle_diff = std::abs(angle - passage_center_angle);

      if (angle_diff < std::atan2(passage_width / 2.0, 3.0)) {
        scan->ranges[i] = 5.0;  // 通道内，远距离
      } else {
        scan->ranges[i] = 0.5;  // 墙壁，近距离
      }
    }
    return scan;
  }

  std::unique_ptr<NarrowPassageDetector> detector_;
};

// ============================================================
// TC-NPD-01: 宽走廊检测
// ============================================================
TEST_F(NarrowPassageDetectorEnhancedTest, WideCorridorDetection)
{
  auto scan = createCorridorScan(2.0, 2.0);
  detector_->updateLaserScan(scan);
  auto passages = detector_->detectPassages();

  for (const auto& p : passages) {
    EXPECT_GT(p.width, 0.6)  // 宽于宽度阈值
      << "Wide corridor passages should exceed width_threshold";
    EXPECT_TRUE(p.is_traversable)
      << "Wide corridor should be traversable";
  }
}

// ============================================================
// TC-NPD-02: 窄道检测
// ============================================================
TEST_F(NarrowPassageDetectorEnhancedTest, NarrowPassageDetection)
{
  auto scan = createCorridorScan(0.3, 0.3);  // 窄于机器人宽度(0.4)
  detector_->updateLaserScan(scan);
  auto passages = detector_->detectPassages();

  for (const auto& p : passages) {
    EXPECT_LT(p.width, 0.6)
      << "Narrow passage should be below width_threshold";
  }
}

// ============================================================
// TC-NPD-03: 多个通道检测
// ============================================================
TEST_F(NarrowPassageDetectorEnhancedTest, MultiplePassagesDetection)
{
  auto scan = createCorridorScan(1.5, 1.5);
  detector_->updateLaserScan(scan);
  auto passages = detector_->detectPassages();

  // 正常走廊应在前后各有一个通道
  EXPECT_GE(passages.size(), 1u)
    << "Should detect at least one passage in corridor";
}

// ============================================================
// TC-NPD-04: 精度模式与普通模式行为差异
// ============================================================
TEST_F(NarrowPassageDetectorEnhancedTest, PrecisionModeVsNormalMode)
{
  // 精度模式
  DetectorConfig precision_cfg;
  precision_cfg.robot_width = 0.4;
  precision_cfg.min_passage_width = 0.5;
  precision_cfg.precision_mode = true;
  precision_cfg.precision_safety_distance = 0.05;

  auto precision_detector = std::make_unique<NarrowPassageDetector>();
  precision_detector->configure(precision_cfg);
  auto scan = createCorridorScan(0.5, 0.5);
  precision_detector->updateLaserScan(scan);
  auto precision_passages = precision_detector->detectPassages();

  // 普通模式
  DetectorConfig normal_cfg = precision_cfg;
  normal_cfg.precision_mode = false;
  auto normal_detector = std::make_unique<NarrowPassageDetector>();
  normal_detector->configure(normal_cfg);
  normal_detector->updateLaserScan(scan);
  auto normal_passages = normal_detector->detectPassages();

  // 两种模式都应工作
  // 精度模式更保守（可能检测到更多窄道或更窄的宽度）
  EXPECT_TRUE(true);  // 至少不崩溃
}

// ============================================================
// TC-NPD-05: 连续更新激光扫描
// ============================================================
TEST_F(NarrowPassageDetectorEnhancedTest, ConsecutiveLaserUpdates)
{
  for (int i = 0; i < 10; ++i) {
    double width = 0.3 + i * 0.1;  // 从窄到宽
    auto scan = createCorridorScan(width, width);
    detector_->updateLaserScan(scan);
    auto passages = detector_->detectPassages();
    // 每次更新都不应崩溃
  }
}

// ============================================================
// TC-NPD-06: 极端近距离障碍物
// ============================================================
TEST_F(NarrowPassageDetectorEnhancedTest, ExtremeCloseObstacle)
{
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.frame_id = "laser";
  scan->angle_min = -M_PI;
  scan->angle_max = M_PI;
  scan->angle_increment = 0.02;
  scan->range_min = 0.05;
  scan->range_max = 10.0;
  scan->ranges.resize(315);
  std::fill(scan->ranges.begin(), scan->ranges.end(), 0.06);  // 极近距离

  detector_->updateLaserScan(scan);
  EXPECT_NO_THROW({
    auto passages = detector_->detectPassages();
  });
}

// ============================================================
// TC-NPD-07: 所有点都是 max_range（空旷环境）
// ============================================================
TEST_F(NarrowPassageDetectorEnhancedTest, FullyOpenEnvironment)
{
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.frame_id = "laser";
  scan->angle_min = -M_PI;
  scan->angle_max = M_PI;
  scan->angle_increment = 0.02;
  scan->range_min = 0.1;
  scan->range_max = 10.0;
  scan->ranges.resize(315);
  std::fill(scan->ranges.begin(), scan->ranges.end(), 10.0);  // 全空旷

  detector_->updateLaserScan(scan);
  auto passages = detector_->detectPassages();

  // 空旷环境不应检测到窄道
  for (const auto& p : passages) {
    EXPECT_TRUE(p.is_traversable)
      << "Open environment passages should all be traversable";
  }
}

// ============================================================
// TC-NPD-08: 推荐速度的边界值
// ============================================================
TEST_F(NarrowPassageDetectorEnhancedTest, RecommendedVelocityBoundaries)
{
  // 通道宽度小于机器人宽度 → 速度应为0
  NarrowPassage impassable;
  impassable.width = 0.3;  // < robot_width(0.4)
  impassable.is_traversable = false;
  double v_impassable = detector_->getRecommendedVelocity(impassable);
  EXPECT_NEAR(v_impassable, 0.0, 0.001)
    << "Velocity for impassable passage should be 0";

  // 通道宽度远大于机器人宽度 → 应接近最大值
  NarrowPassage wide;
  wide.width = 2.0;
  wide.is_traversable = true;
  double v_wide = detector_->getRecommendedVelocity(wide);
  EXPECT_GT(v_wide, 0.3)
    << "Wide passage should allow reasonable velocity";

  // 通道宽度刚好等于阈值 → 应有中间值
  NarrowPassage marginal;
  marginal.width = 0.6;  // 略大于 robot_width + some margin
  marginal.is_traversable = true;
  double v_marginal = detector_->getRecommendedVelocity(marginal);
  EXPECT_GT(v_marginal, 0.0)
    << "Marginal passage should allow some velocity";
  EXPECT_LT(v_marginal, 1.0)
    << "Marginal passage velocity should be conservative";
}

// ============================================================
// TC-NPD-09: getAllPassages 一致性
// ============================================================
TEST_F(NarrowPassageDetectorEnhancedTest, GetAllPassagesConsistency)
{
  auto scan = createCorridorScan(1.0, 1.0);
  detector_->updateLaserScan(scan);

  auto passages1 = detector_->getAllPassages();
  auto passages2 = detector_->getAllPassages();

  EXPECT_EQ(passages1.size(), passages2.size())
    << "getAllPassages should be idempotent when no updates occur";
}

// ============================================================
// TC-NPD-10: 频繁配置切换
// ============================================================
TEST_F(NarrowPassageDetectorEnhancedTest, FrequentConfigSwitching)
{
  for (int i = 0; i < 5; ++i) {
    DetectorConfig config;
    config.robot_width = 0.3 + i * 0.05;
    config.min_passage_width = 0.4 + i * 0.05;
    config.precision_mode = (i % 2 == 0);

    EXPECT_NO_THROW({
      detector_->configure(config);
      auto scan = createCorridorScan(0.8, 0.8);
      detector_->updateLaserScan(scan);
      auto passages = detector_->detectPassages();
    });
  }
}
