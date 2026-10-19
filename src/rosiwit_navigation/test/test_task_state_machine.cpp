#include <gtest/gtest.h>

#include "rosiwit_navigation/nav_core/task_state_machine.hpp"

using namespace rosiwit_navigation::core;

TEST(TaskStateMachine, HappyPath)
{
  TaskStateMachine sm;
  EXPECT_EQ(sm.state(), TaskState::IDLE);
  EXPECT_TRUE(sm.handle(TaskEvent::GOAL_RECEIVED));
  EXPECT_EQ(sm.state(), TaskState::PLANNING);
  EXPECT_TRUE(sm.handle(TaskEvent::PLAN_OK));
  EXPECT_EQ(sm.state(), TaskState::FOLLOWING);
  EXPECT_TRUE(sm.handle(TaskEvent::GOAL_REACHED));
  EXPECT_EQ(sm.state(), TaskState::SUCCEEDED);
}

TEST(TaskStateMachine, RecoveryPath)
{
  TaskStateMachine sm;
  sm.handle(TaskEvent::GOAL_RECEIVED);
  sm.handle(TaskEvent::PLAN_OK);
  EXPECT_TRUE(sm.handle(TaskEvent::PATH_BLOCKED));
  EXPECT_EQ(sm.state(), TaskState::RECOVERING);
  EXPECT_TRUE(sm.handle(TaskEvent::RECOVERY_DONE));
  EXPECT_EQ(sm.state(), TaskState::PLANNING);
  EXPECT_TRUE(sm.handle(TaskEvent::PLAN_OK));
  EXPECT_EQ(sm.state(), TaskState::FOLLOWING);
}

TEST(TaskStateMachine, StopResetsActiveTask)
{
  TaskStateMachine sm;
  sm.handle(TaskEvent::GOAL_RECEIVED);
  sm.handle(TaskEvent::PLAN_OK);
  EXPECT_TRUE(sm.handle(TaskEvent::STOP));
  EXPECT_EQ(sm.state(), TaskState::STOPPED);
  EXPECT_FALSE(sm.isActive());
  sm.handle(TaskEvent::RESET);
  EXPECT_EQ(sm.state(), TaskState::IDLE);
}

TEST(TaskStateMachine, InvalidTransitionKeepsState)
{
  TaskStateMachine sm;
  EXPECT_FALSE(sm.handle(TaskEvent::PLAN_OK));
  EXPECT_EQ(sm.state(), TaskState::IDLE);
}
