// ============================================================================
// test_action_task.cpp — 任务状态机的行为回归 (ROS 无关)
//   这些用例对应 robot_movebase 的几条真实语义:
//     * 单点导航"位置到位 + 末端航向对齐"才结束
//     * navi_cmd=6 原地旋转 180°
//     * navi_cmd=7 倒车固定距离
//     * 长时间无进展 -> 卡死失败 (旧实现 -16)
// ============================================================================
#include <gtest/gtest.h>

#include <cmath>

#include "rosiwit_navigation/nav_core/action_task.hpp"

using namespace rosiwit_navigation::core;

namespace
{

ActionTaskConfig testConfig()
{
  ActionTaskConfig config;
  config.goal_yaw_tolerance = 0.10;
  config.rotate_tolerance = 0.05;
  config.stuck_timeout = 10.0;
  config.back_speed = 0.1;
  config.rotate_speed = 0.6;
  return config;
}

Pose2D pose(double x, double y, double yaw)
{
  return Pose2D(x, y, yaw);
}

}  // namespace

TEST(ActionTask, IdleStartIsRejected)
{
  ActionTask task;
  task.configure(testConfig());
  EXPECT_FALSE(task.start(ActionTaskCommand(), pose(0, 0, 0), 0.0));
  EXPECT_EQ(task.status(), ActionTaskStatus::IDLE);
}

TEST(ActionTask, NavigateSucceedsWhenXyReachedWithoutFinalYaw)
{
  ActionTask task;
  task.configure(testConfig());
  ActionTaskCommand command;
  command.kind = ActionKind::NAVIGATE;
  command.final_rotate = false;

  ASSERT_TRUE(task.start(command, pose(0, 0, 0.2), 0.0));
  EXPECT_EQ(task.update(pose(0.5, 0, 0.2), 1.0, false, 0.4), ActionTaskStatus::RUNNING);
  EXPECT_NEAR(task.progress(), 0.4, 1e-9);

  EXPECT_EQ(task.update(pose(1.0, 0, 0.2), 2.0, true, 1.0), ActionTaskStatus::SUCCEEDED);
  EXPECT_DOUBLE_EQ(task.progress(), 1.0);
}

TEST(ActionTask, NavigateAlignsFinalYawAfterReachingPosition)
{
  ActionTask task;
  task.configure(testConfig());
  ActionTaskCommand command;
  command.kind = ActionKind::NAVIGATE;
  command.final_rotate = true;
  command.final_yaw = M_PI / 2.0;

  ASSERT_TRUE(task.start(command, pose(0, 0, 0.0), 0.0));

  // 位置到位但航向差 90°: 进入末端旋转, 不算成功.
  EXPECT_EQ(task.update(pose(1.0, 1.0, 0.0), 1.0, true, 1.0), ActionTaskStatus::RUNNING);
  EXPECT_TRUE(task.rotating());
  EXPECT_NEAR(task.rotationTarget(), M_PI / 2.0, 1e-9);
  EXPECT_GT(task.rotationCommand(pose(1.0, 1.0, 0.0)), 0.0);

  // 转到位才成功.
  EXPECT_EQ(task.update(pose(1.0, 1.0, M_PI / 2.0), 2.0, true, 1.0),
            ActionTaskStatus::SUCCEEDED);
  EXPECT_DOUBLE_EQ(task.progress(), 1.0);
}

TEST(ActionTask, Rotate180UsesShortestDirection)
{
  ActionTask task;
  task.configure(testConfig());
  ActionTaskCommand command;
  command.kind = ActionKind::ROTATE;

  ASSERT_TRUE(task.start(command, pose(0, 0, 0.0), 0.0));
  EXPECT_TRUE(task.rotating());
  EXPECT_NEAR(task.rotationTarget(), M_PI, 1e-9);

  EXPECT_EQ(task.update(pose(0, 0, 1.0), 1.0, false, 0.0), ActionTaskStatus::RUNNING);
  EXPECT_GT(task.progress(), 0.0);
  EXPECT_LT(task.progress(), 1.0);
  EXPECT_GT(task.rotationCommand(pose(0, 0, 1.0)), 0.0);

  EXPECT_EQ(task.update(pose(0, 0, M_PI - 0.01), 2.0, false, 0.0),
            ActionTaskStatus::SUCCEEDED);
  EXPECT_DOUBLE_EQ(task.progress(), 1.0);
}

TEST(ActionTask, RotateWrapsAcrossPi)
{
  ActionTask task;
  task.configure(testConfig());
  ActionTaskCommand command;
  command.kind = ActionKind::ROTATE;

  // 从 -170° 转到 +10° 附近 (跨越 ±π): 应按最短方向转 +20°.
  ASSERT_TRUE(task.start(command, pose(0, 0, -170.0 * M_PI / 180.0), 0.0));
  const double target = task.rotationTarget();
  EXPECT_NEAR(std::remainder(target - (-170.0 * M_PI / 180.0), 2.0 * M_PI), M_PI, 1e-9);
}

TEST(ActionTask, BackOffCompletesAtRequestedDistance)
{
  ActionTask task;
  task.configure(testConfig());
  ActionTaskCommand command;
  command.kind = ActionKind::BACK_OFF;
  command.back_distance = 0.3;

  ASSERT_TRUE(task.start(command, pose(0, 0, 0.0), 0.0));
  EXPECT_TRUE(task.backing());
  EXPECT_LT(task.backOffCommand(), 0.0);
  EXPECT_NEAR(task.backRemaining(), 0.3, 1e-9);

  EXPECT_EQ(task.update(pose(-0.1, 0, 0.0), 1.0, false, 0.0), ActionTaskStatus::RUNNING);
  EXPECT_NEAR(task.progress(), 1.0 / 3.0, 1e-6);

  EXPECT_EQ(task.update(pose(-0.3, 0, 0.0), 2.0, false, 0.0), ActionTaskStatus::SUCCEEDED);
  EXPECT_DOUBLE_EQ(task.progress(), 1.0);
  EXPECT_NEAR(task.backRemaining(), 0.0, 1e-9);
}

TEST(ActionTask, StuckWatchdogFailsAfterTimeout)
{
  ActionTask task;
  ActionTaskConfig config = testConfig();
  config.stuck_timeout = 2.0;
  task.configure(config);

  ActionTaskCommand command;
  command.kind = ActionKind::NAVIGATE;
  ASSERT_TRUE(task.start(command, pose(0, 0, 0), 0.0));

  EXPECT_EQ(task.update(pose(0, 0, 0), 1.0, false, 0.0), ActionTaskStatus::RUNNING);
  EXPECT_EQ(task.update(pose(0, 0, 0), 2.5, false, 0.0), ActionTaskStatus::FAILED_STUCK);
}

TEST(ActionTask, StuckWatchdogResetsOnProgress)
{
  ActionTask task;
  ActionTaskConfig config = testConfig();
  config.stuck_timeout = 2.0;
  task.configure(config);

  ActionTaskCommand command;
  command.kind = ActionKind::FOLLOW_PATH;
  ASSERT_TRUE(task.start(command, pose(0, 0, 0), 0.0));

  EXPECT_EQ(task.update(pose(0.5, 0, 0), 1.5, false, 0.2), ActionTaskStatus::RUNNING);
  // 位移超过 0.25m, 计时重新开始: 到 3.0s (距上次进展 1.5s) 仍应 RUNNING.
  EXPECT_EQ(task.update(pose(0.6, 0, 0), 3.0, false, 0.3), ActionTaskStatus::RUNNING);
  // 再静止 2.5s -> 失败.
  EXPECT_EQ(task.update(pose(0.6, 0, 0), 5.5, false, 0.3), ActionTaskStatus::FAILED_STUCK);
}

TEST(ActionTask, RotationCountsAsProgress)
{
  ActionTask task;
  ActionTaskConfig config = testConfig();
  config.stuck_timeout = 1.5;
  task.configure(config);

  ActionTaskCommand command;
  command.kind = ActionKind::ROTATE;
  ASSERT_TRUE(task.start(command, pose(0, 0, 0), 0.0));

  // 原地旋转 0.2rad (> progress_yaw) 应续命.
  EXPECT_EQ(task.update(pose(0, 0, 0.2), 1.0, false, 0.0), ActionTaskStatus::RUNNING);
  EXPECT_EQ(task.update(pose(0, 0, 0.5), 2.0, false, 0.0), ActionTaskStatus::RUNNING);
}

// 恢复动作 (不在 update() 里) 必须能把卡死计时清零, 并把已锁存的 FAILED_STUCK
// 重新置回 RUNNING —— 否则 update() 永远返回终态, 恢复->判卡死->恢复 死循环。
TEST(ActionTask, NoteMotionReArmsStuckWatchdog)
{
  ActionTask task;
  ActionTaskConfig config = testConfig();
  config.stuck_timeout = 2.0;
  task.configure(config);

  ActionTaskCommand command;
  command.kind = ActionKind::NAVIGATE;
  ASSERT_TRUE(task.start(command, pose(0, 0, 0), 0.0));
  EXPECT_EQ(task.update(pose(0, 0, 0), 2.5, false, 0.0), ActionTaskStatus::FAILED_STUCK);

  // 恢复动作 (倒车/原地转) 期间上层每拍调用 noteMotion
  const Pose2D moved = pose(0.0, -0.4, 0.0);
  task.noteMotion(moved, 3.0);
  EXPECT_EQ(task.status(), ActionTaskStatus::RUNNING);

  // 重新计时: 3.0s 之后 1.5s 内仍然 RUNNING (旧实现这里会立刻再返回 FAILED_STUCK)
  EXPECT_EQ(task.update(moved, 4.5, false, 0.0), ActionTaskStatus::RUNNING);
  // 继续静止到超时 -> 再次失败
  EXPECT_EQ(task.update(moved, 5.5, false, 0.0), ActionTaskStatus::FAILED_STUCK);
}

TEST(ActionTask, ResetClearsState)
{
  ActionTask task;
  task.configure(testConfig());
  ActionTaskCommand command;
  command.kind = ActionKind::ROTATE;
  ASSERT_TRUE(task.start(command, pose(0, 0, 0), 0.0));
  EXPECT_TRUE(task.rotating());

  task.reset();
  EXPECT_FALSE(task.active());
  EXPECT_FALSE(task.rotating());
  EXPECT_EQ(task.status(), ActionTaskStatus::IDLE);
  EXPECT_EQ(task.kind(), ActionKind::IDLE);
  EXPECT_DOUBLE_EQ(task.progress(), 0.0);
}

// catkin_add_gtest 只链 GTEST_LIBRARIES (不含 gtest_main)
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
