// ============================================================================
// test_movebase_protocol.cpp — robot_movebase action 协议解析的回归测试
//
// 这些用例钉住的是"报文语义", 不是实现细节:
//   * navi_cmd 1/2/3/6/7 的字段怎么解释, 4/5/8..13 为什么必须明确失败
//   * path_index 越界的回落 (旧实现会走到最后一个点, 不能崩)
//   * 路径文件 JSON 的真实形状 ({"path":[[{"x","y"}, ...]]})
// 不需要 ROS1 环境:
//   g++ -std=c++17 -I src/rosiwit_navigation/include
//       test/test_movebase_protocol.cpp src/movebase_protocol.cpp
//       -lgtest -lgtest_main -pthread -o /tmp/test_movebase_protocol
// ============================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "movebase_protocol.h"

using rosiwit_navigation::core::Pose2D;
using namespace rosiwit_navigation::movebase;

namespace
{

Goal makeGoal(int cmd)
{
  Goal goal;
  goal.navi_cmd = cmd;
  return goal;
}

Pose2D makePose(double x, double y, double theta = 0.0)
{
  return Pose2D(x, y, theta);
}

}  // namespace

TEST(MovebaseProtocol, CancelIsSupported)
{
  const Decoded decoded = decodeGoal(makeGoal(0));
  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.supported);
  EXPECT_EQ(decoded.command, Command::CANCEL);
}

TEST(MovebaseProtocol, SinglePointUsesFirstPose)
{
  Goal goal = makeGoal(1);
  goal.tracking_path = {makePose(1.0, 2.0, 0.5)};
  const Decoded decoded = decodeGoal(goal);
  EXPECT_TRUE(decoded.supported);
  EXPECT_EQ(decoded.command, Command::SINGLE_POINT);
  EXPECT_DOUBLE_EQ(decoded.target.x, 1.0);
  EXPECT_DOUBLE_EQ(decoded.target.y, 2.0);
  EXPECT_DOUBLE_EQ(decoded.target.theta, 0.5);
}

TEST(MovebaseProtocol, SinglePointHonoursPathIndexAndClamps)
{
  Goal goal = makeGoal(1);
  goal.tracking_path = {makePose(1.0, 0.0), makePose(2.0, 0.0), makePose(3.0, 0.0)};

  goal.path_index = 2;
  EXPECT_DOUBLE_EQ(decodeGoal(goal).target.x, 3.0);

  // 旧实现索引越界会崩; 协议层回落到最后一个点.
  goal.path_index = 99;
  EXPECT_DOUBLE_EQ(decodeGoal(goal).target.x, 3.0);

  goal.path_index = -5;
  EXPECT_DOUBLE_EQ(decodeGoal(goal).target.x, 1.0);
}

TEST(MovebaseProtocol, SinglePointWithoutPoseRejectsWithPoseError)
{
  const Decoded decoded = decodeGoal(makeGoal(1));
  EXPECT_FALSE(decoded.valid);
  EXPECT_FALSE(decoded.supported);
  EXPECT_EQ(decoded.reject_result, RESULT_POSE_ERROR);
}

TEST(MovebaseProtocol, TwoPointsKeepsExplicitStart)
{
  Goal goal = makeGoal(2);
  goal.tracking_path = {makePose(1.0, 1.0, 0.1), makePose(4.0, 5.0, -0.2)};
  const Decoded decoded = decodeGoal(goal);
  EXPECT_TRUE(decoded.supported);
  EXPECT_TRUE(decoded.has_start);
  EXPECT_DOUBLE_EQ(decoded.start.x, 1.0);
  EXPECT_DOUBLE_EQ(decoded.target.x, 4.0);
  EXPECT_DOUBLE_EQ(decoded.target.theta, -0.2);
}

TEST(MovebaseProtocol, TwoPointsNeedsTwoPoses)
{
  Goal goal = makeGoal(2);
  goal.tracking_path = {makePose(1.0, 1.0)};
  const Decoded decoded = decodeGoal(goal);
  EXPECT_FALSE(decoded.supported);
  EXPECT_EQ(decoded.reject_result, RESULT_POSE_ERROR);
}

TEST(MovebaseProtocol, FollowPathSlicesFromPathIndex)
{
  Goal goal = makeGoal(3);
  goal.tracking_path = {makePose(0.0, 0.0), makePose(1.0, 0.0),
                        makePose(2.0, 0.0), makePose(3.0, 0.0)};
  goal.path_index = 2;
  const Decoded decoded = decodeGoal(goal);
  EXPECT_TRUE(decoded.supported);
  ASSERT_EQ(decoded.path.size(), 2u);
  EXPECT_DOUBLE_EQ(decoded.path.front().x, 2.0);
  EXPECT_DOUBLE_EQ(decoded.path.back().x, 3.0);
}

TEST(MovebaseProtocol, FollowPathWithFileDefersToLoader)
{
  Goal goal = makeGoal(3);
  goal.file_path = "/tmp/path.json";
  goal.path_segment_index = 3;
  const Decoded decoded = decodeGoal(goal);
  EXPECT_TRUE(decoded.supported);
  EXPECT_TRUE(decoded.use_file);
  EXPECT_EQ(decoded.path_segment_index, 3);
}

TEST(MovebaseProtocol, RotateAndBackOffAreSupported)
{
  EXPECT_EQ(decodeGoal(makeGoal(6)).command, Command::ROTATE_180);
  EXPECT_TRUE(decodeGoal(makeGoal(6)).supported);

  const Decoded back = decodeGoal(makeGoal(7));
  EXPECT_EQ(back.command, Command::BACK_OFF);
  EXPECT_DOUBLE_EQ(back.back_distance, 0.3);  // 旧实现的默认值

  Goal explicit_back = makeGoal(7);
  explicit_back.backing_dist = 1.5;
  EXPECT_DOUBLE_EQ(decodeGoal(explicit_back).back_distance, 1.5);
}

TEST(MovebaseProtocol, ElevatorAndCleaningCommandsFailExplicitly)
{
  for (int cmd : {4, 5, 8, 9, 10, 11, 12, 13, 42}) {
    const Decoded decoded = decodeGoal(makeGoal(cmd));
    EXPECT_FALSE(decoded.supported) << "navi_cmd=" << cmd << " 不应被静默接受";
    EXPECT_EQ(decoded.reject_result, RESULT_PLAN_FAILED);
  }
}

TEST(MovebaseProtocol, PathJsonParsesSegmentsAndFillsYaw)
{
  const std::string text =
      "{\n"
      "  \"path\": [\n"
      "    [{\"x\": 1, \"y\": 2}, {\"x\": 3, \"y\": 2}, {\"x\": 3, \"y\": 4}],\n"
      "    [{\"x\": -1.5, \"y\": 0.25}, {\"x\": -2.5e0, \"y\": 0.25}]\n"
      "  ],\n"
      "  \"map_area\": [{\"x\": 0, \"y\": 0}],\n"
      "  \"path_percentage\": [0.5, 0.5]\n"
      "}\n";

  std::vector<std::vector<Pose2D>> segments;
  std::string error;
  ASSERT_TRUE(parsePathJson(text, segments, &error)) << error;
  ASSERT_EQ(segments.size(), 2u);
  ASSERT_EQ(segments[0].size(), 3u);
  EXPECT_DOUBLE_EQ(segments[0][1].x, 3.0);
  EXPECT_DOUBLE_EQ(segments[0][1].y, 2.0);
  EXPECT_NEAR(segments[0][0].theta, 0.0, 1e-9);          // 向右
  EXPECT_NEAR(segments[0][2].theta, M_PI / 2.0, 1e-9);   // 最后一点沿用前一段方向
  EXPECT_DOUBLE_EQ(segments[1][0].x, -1.5);
  EXPECT_DOUBLE_EQ(segments[1][1].x, -2.5);
}

TEST(MovebaseProtocol, PathJsonAcceptsFlatPointArray)
{
  const std::string text = "{\"path\": [{\"x\": 0, \"y\": 0}, {\"x\": 0, \"y\": 1}]}";
  std::vector<std::vector<Pose2D>> segments;
  ASSERT_TRUE(parsePathJson(text, segments));
  ASSERT_EQ(segments.size(), 1u);
  ASSERT_EQ(segments[0].size(), 2u);
  EXPECT_NEAR(segments[0][0].theta, M_PI / 2.0, 1e-9);
}

TEST(MovebaseProtocol, PathJsonRejectsBrokenOrEmptyInput)
{
  std::vector<std::vector<Pose2D>> segments;

  EXPECT_FALSE(parsePathJson("", segments));
  EXPECT_FALSE(parsePathJson("{}", segments));
  EXPECT_FALSE(parsePathJson("{\"path\": []}", segments));
  EXPECT_FALSE(parsePathJson("{\"path\": [[{\"x\": 1}]]}", segments));   // 缺 y
  EXPECT_FALSE(parsePathJson("{\"path\": [[{\"x\": 1, \"y\": 2}", segments));
}

TEST(MovebaseProtocol, LoadPathFileReportsMissingFile)
{
  std::vector<std::vector<Pose2D>> segments;
  std::string error;
  EXPECT_FALSE(loadPathFile("/tmp/definitely_not_a_movebase_path.json", segments, &error));
  EXPECT_FALSE(error.empty());
}

TEST(MovebaseProtocol, PathHelpers)
{
  const std::vector<Pose2D> path = {makePose(0, 0), makePose(1, 0), makePose(1, 1)};
  EXPECT_NEAR(pathLength(path), 2.0, 1e-9);
  const std::vector<Pose2D> tail = slicePath(path, 1);
  ASSERT_EQ(tail.size(), 2u);
  EXPECT_DOUBLE_EQ(tail.front().x, 1.0);
  EXPECT_EQ(slicePath(path, 99).size(), 1u);

  EXPECT_DOUBLE_EQ(clampPercent(1.7), 1.0);
  EXPECT_DOUBLE_EQ(clampPercent(-0.2), 0.0);
  EXPECT_DOUBLE_EQ(clampPercent(std::nan("")), 0.0);
}
