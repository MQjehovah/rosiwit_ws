// 核心单测: 多类别 KF 跟踪 / 关联 / 生命周期 / 双速度坐标系
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "object_semantics/multi_object_tracker.h"

namespace rp = rosiwit_perception;

namespace {

rp::ObjectObservation makeObs(double x, double y, int class_id = 0,
                              const std::string& class_name = "person",
                              double max_speed_mps = 2.5) {
  rp::ObjectObservation ob;
  ob.depth_valid = true;
  ob.max_speed_mps = max_speed_mps;
  ob.position = Eigen::Vector2d(x, y);
  ob.ground = ob.position;
  ob.z = 1.2;
  ob.distance = ob.position.norm();
  ob.cov_xy = Eigen::Matrix2d::Identity() * 0.0025;  // σ=0.05 m
  ob.confidence = 0.9;
  ob.box.class_id = class_id;
  ob.box.class_name = class_name;
  ob.box.score = 0.9;
  return ob;
}

rp::RobotState staticRobot() {
  rp::RobotState r;
  r.valid = true;
  r.position_world = Eigen::Vector3d::Zero();
  r.R_world_base = Eigen::Matrix3d::Identity();
  r.velocity_world = Eigen::Vector3d::Zero();
  return r;
}

}  // namespace

TEST(MultiObjectTracker, ConfirmsAfterHitsAndReportsVelocity) {
  rp::MultiObjectTracker tracker;
  std::vector<rp::ObjectTrack> out;
  const auto robot = staticRobot();
  // 人沿 +x 以 1 m/s 走, 10 Hz
  for (int i = 0; i < 20; ++i) {
    const double t = i * 0.1;
    tracker.update(t, {makeObs(2.0 + 1.0 * t, 0.3)}, robot, out);
  }
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(out[0].confirmed);
  EXPECT_TRUE(out[0].velocity_valid);
  EXPECT_NEAR(out[0].velocity_base.x(), 1.0, 0.15);
  EXPECT_NEAR(out[0].position.x(), 2.0 + 1.0 * 1.9, 0.15);
  EXPECT_NEAR(out[0].age, 1.9, 0.05);
}

TEST(MultiObjectTracker, VelocityNotValidBeforeMinAge) {
  rp::MultiObjectTracker tracker;
  std::vector<rp::ObjectTrack> out;
  const auto robot = staticRobot();
  for (int i = 0; i < 3; ++i) {          // 0.2 s < velocity_min_age_s(0.3)
    tracker.update(i * 0.1, {makeObs(2.0, 0.0)}, robot, out);
  }
  ASSERT_FALSE(out.empty());
  EXPECT_FALSE(out[0].velocity_valid);
}

TEST(MultiObjectTracker, TentativeTrackDroppedWhenNeverConfirmed) {
  rp::MultiObjectTracker tracker;
  std::vector<rp::ObjectTrack> out;
  const auto robot = staticRobot();
  tracker.update(0.0, {makeObs(2.0, 0.0)}, robot, out);   // 1 hit
  for (int i = 1; i <= 3; ++i) {
    tracker.update(i * 0.1, {}, robot, out);             // 连续 miss
  }
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(tracker.trackCount(), 0u);
}

TEST(MultiObjectTracker, CoastingKeepsTrackForHalfSecond) {
  rp::MultiObjectTracker tracker;
  std::vector<rp::ObjectTrack> out;
  const auto robot = staticRobot();
  for (int i = 0; i < 5; ++i) {
    tracker.update(i * 0.1, {makeObs(2.0 + 0.05 * i, 0.0)}, robot, out);
  }
  ASSERT_EQ(out.size(), 1u);
  const uint32_t id = out[0].id;

  tracker.update(0.5, {}, robot, out);          // 0.1s 无观测
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(out[0].coasting);
  EXPECT_EQ(out[0].id, id);

  tracker.update(1.2, {}, robot, out);          // 超过 max_coast_s(0.5) 后删除
  EXPECT_TRUE(out.empty());
}

TEST(MultiObjectTracker, EgoMotionCompensation) {
  rp::MultiObjectTracker tracker;
  std::vector<rp::ObjectTrack> out;
  // 机器人沿 +x 以 1 m/s 前进, 人静止在世界系 x=3 m
  for (int i = 0; i < 20; ++i) {
    const double t = i * 0.1;
    rp::RobotState robot;
    robot.valid = true;
    robot.position_world = Eigen::Vector3d(1.0 * t, 0.0, 0.0);
    robot.R_world_base = Eigen::Matrix3d::Identity();
    robot.velocity_world = Eigen::Vector3d(1.0, 0.0, 0.0);
    const double x_base = 3.0 - robot.position_world.x();
    tracker.update(t, {makeObs(x_base, 0.0)}, robot, out);
  }
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].velocity_world.x(), 0.0, 0.2);   // 世界系静止
  EXPECT_NEAR(out[0].velocity_base.x(), -1.0, 0.2);   // 相对机器人后退 1 m/s
}

TEST(MultiObjectTracker, TwoSeparatedPeopleKeepDistinctIds) {
  rp::MultiObjectTracker tracker;
  std::vector<rp::ObjectTrack> out;
  const auto robot = staticRobot();
  for (int i = 0; i < 10; ++i) {
    const double t = i * 0.1;
    tracker.update(t, {makeObs(2.0, -1.0), makeObs(3.0, 1.5)}, robot, out);
  }
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NE(out[0].id, out[1].id);
  EXPECT_TRUE(out[0].confirmed && out[1].confirmed);
  const double d01 = (out[0].position - out[1].position).norm();
  EXPECT_NEAR(d01, std::hypot(1.0, 2.5), 0.3);
}

TEST(MultiObjectTracker, DifferentClassesDoNotShareTracks) {
  rp::MultiObjectTracker tracker;
  std::vector<rp::ObjectTrack> out;
  const auto robot = staticRobot();
  // 同一位置同时出现 person(0) 与 car(2): 类别门必须把它们分成两条轨迹
  for (int i = 0; i < 10; ++i) {
    const double t = i * 0.1;
    tracker.update(t, {makeObs(2.0, 0.0, 0, "person"), makeObs(2.0, 0.0, 2, "car")}, robot, out);
  }
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NE(out[0].id, out[1].id);
  EXPECT_NE(out[0].class_id, out[1].class_id);
}

TEST(MultiObjectTracker, VelocityLimitComesFromClassSpec) {
  // 同样是 1 m/s 的运动: person 上限 2.5 m/s 判有效; 上限 0.5 m/s 的类别判无效
  const auto run = [](double max_speed) {
    rp::MultiObjectTracker tracker;
    std::vector<rp::ObjectTrack> out;
    const auto robot = staticRobot();
    for (int i = 0; i < 20; ++i) {
      const double t = i * 0.1;
      tracker.update(t, {makeObs(2.0 + 1.0 * t, 0.0, 0, "person", max_speed)}, robot, out);
    }
    return out;
  };
  const auto fast_class = run(2.5);
  const auto slow_class = run(0.5);
  ASSERT_EQ(fast_class.size(), 1u);
  ASSERT_EQ(slow_class.size(), 1u);
  EXPECT_TRUE(fast_class[0].velocity_valid);
  EXPECT_FALSE(slow_class[0].velocity_valid);
}

TEST(MultiObjectTracker, ClassIdAndNamePropagateToOutput) {
  rp::MultiObjectTracker tracker;
  std::vector<rp::ObjectTrack> out;
  const auto robot = staticRobot();
  for (int i = 0; i < 5; ++i) {
    tracker.update(i * 0.1, {makeObs(3.0, 0.5, 2, "car")}, robot, out);
  }
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].class_id, 2);
  EXPECT_EQ(out[0].class_name, "car");
  EXPECT_EQ(out[0].box.class_id, 2);
}
