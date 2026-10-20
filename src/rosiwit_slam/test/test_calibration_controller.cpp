#include <gtest/gtest.h>

#include "slam_core/calibration_controller.h"

using namespace rosiwit_slam;

namespace
{
TimeCalibrationResult goodTime(double off, double conf = 0.9)
{
  TimeCalibrationResult r; r.valid = true; r.offset = off; r.confidence = conf; r.samples = 100; return r;
}
YawCalibrationResult goodYaw(double off) { YawCalibrationResult r; r.valid = true; r.yaw_offset = off; r.samples = 100; return r; }
}  // namespace

TEST(CalibrationController, DisabledDoesNothing)
{
  CalibrationController c;
  const auto d = c.update(goodTime(0.03), goodYaw(0.05));
  EXPECT_FALSE(d.applied);
  EXPECT_EQ(d.state, "DISABLED");
}

TEST(CalibrationController, RequiresConfidenceAndStability)
{
  CalibrationApplyConfig cfg; cfg.enabled = true; cfg.stable_windows = 3; cfg.max_step = 0.01;
  CalibrationController c(cfg);
  EXPECT_FALSE(c.update(goodTime(0.02, 0.3), {}).applied);
  EXPECT_FALSE(c.update(goodTime(0.02), {}).applied);
  EXPECT_FALSE(c.update(goodTime(0.02), {}).applied);
  const auto d = c.update(goodTime(0.02), {});
  EXPECT_TRUE(d.applied);
  EXPECT_NEAR(d.time_offset, 0.01, 1e-9);  // step limited
}

TEST(CalibrationController, RejectsOutOfRangeAndRollsBack)
{
  CalibrationApplyConfig cfg; cfg.enabled = true; cfg.stable_windows = 1;
  cfg.max_time_offset = 0.05; cfg.rollback_windows = 2;
  CalibrationController c(cfg);
  const auto rejected = c.update(goodTime(0.2), {});
  EXPECT_FALSE(rejected.applied);
  EXPECT_EQ(rejected.state, "REJECTED");

  const auto applied = c.update(goodTime(0.02), {});
  ASSERT_TRUE(applied.applied);
  EXPECT_NEAR(applied.time_offset, 0.01, 1e-9);

  c.update({}, {});
  const auto rolled = c.update({}, {});
  EXPECT_TRUE(rolled.rolled_back);
  EXPECT_NEAR(rolled.time_offset, 0.0, 1e-9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
