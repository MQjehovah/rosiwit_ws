#include <gtest/gtest.h>

#include <cmath>

#include "slam_core/online_calibrator.h"

using namespace rosiwit_slam;

TEST(OnlineTimeCalibrator, RecoversKnownOffset)
{
  OnlineTimeCalibrator calib;
  const double true_offset = -0.05;  // lio timestamp = true + 0.05
  for (int i = 0; i <= 1000; ++i) {
    const double t = i * 0.01;
    calib.addImu(t, std::cos(t));
  }
  for (int i = 0; i <= 100; ++i) {
    const double t = i * 0.1;
    calib.addLioPose(t + 0.05, std::sin(t));
  }
  const auto result = calib.estimate();
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.offset, true_offset, 0.012);
  EXPECT_GT(result.confidence, 0.2);
}

TEST(YawExtrinsicCalibrator, RecoversCircularMean)
{
  YawExtrinsicCalibrator calib;
  for (int i = 0; i < 20; ++i) calib.addSample(0.10, 0.10 + 0.30);
  const auto result = calib.estimate();
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.yaw_offset, 0.30, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
