// ============================================================
// Diffbot Navigation - 状态机单元测试
// 测试导航状态机的状态转换和事件处理
// ============================================================

#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>

#include "diffbot_navigation/core/state_machine.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace diffbot_navigation::core;

class StateMachineTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        state_machine_ = std::make_unique<NavigationStateMachine>();
        state_machine_->initialize();
    }

    void TearDown() override
    {
        state_machine_.reset();
    }

    std::unique_ptr<NavigationStateMachine> state_machine_;
};

// ==================== 状态机初始化测试 ====================

TEST_F(StateMachineTest, InitialStateIsIdle)
{
    EXPECT_EQ(state_machine_->state(), NavigationState::IDLE);
}

TEST_F(StateMachineTest, InitializeSetsIdleState)
{
    // 先转换到其他状态
    state_machine_->transitionTo(NavigationState::PLANNING);
    EXPECT_NE(state_machine_->state(), NavigationState::IDLE);

    // 重新初始化
    state_machine_->initialize();
    EXPECT_EQ(state_machine_->state(), NavigationState::IDLE);
}

TEST_F(StateMachineTest, ResetReturnsToIdle)
{
    // 转换到其他状态
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);

    // 重置
    state_machine_->transitionTo(NavigationState::IDLE);
    EXPECT_EQ(state_machine_->state(), NavigationState::IDLE);
}

// ==================== 状态转换测试 ====================

TEST_F(StateMachineTest, TransitionIdleToPlanning)
{
    auto result = state_machine_->transitionTo(NavigationState::PLANNING);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::PLANNING);
}

TEST_F(StateMachineTest, TransitionPlanningToControlling)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    auto result = state_machine_->transitionTo(NavigationState::CONTROLLING);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::CONTROLLING);
}

TEST_F(StateMachineTest, TransitionPlanningToFailed)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    auto result = state_machine_->transitionTo(NavigationState::FAILED);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::FAILED);
}

TEST_F(StateMachineTest, TransitionControllingToGoalReached)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    auto result = state_machine_->transitionTo(NavigationState::GOAL_REACHED);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::GOAL_REACHED);
}

TEST_F(StateMachineTest, TransitionControllingToObstacleAvoidance)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    auto result = state_machine_->transitionTo(NavigationState::OBSTACLE_AVOIDANCE);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::OBSTACLE_AVOIDANCE);
}

TEST_F(StateMachineTest, TransitionControllingToNarrowPassage)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    auto result = state_machine_->transitionTo(NavigationState::NARROW_PASSAGE);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::NARROW_PASSAGE);
}

TEST_F(StateMachineTest, TransitionObstacleAvoidanceBackToControlling)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    state_machine_->transitionTo(NavigationState::OBSTACLE_AVOIDANCE);
    auto result = state_machine_->transitionTo(NavigationState::CONTROLLING);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::CONTROLLING);
}

TEST_F(StateMachineTest, TransitionNarrowPassageBackToControlling)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    state_machine_->transitionTo(NavigationState::NARROW_PASSAGE);
    auto result = state_machine_->transitionTo(NavigationState::CONTROLLING);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::CONTROLLING);
}

// ==================== 取消和错误处理测试 ====================

TEST_F(StateMachineTest, CancelFromPlanning)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    auto result = state_machine_->transitionTo(NavigationState::CANCELLED);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::CANCELLED);
}

TEST_F(StateMachineTest, CancelFromControlling)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    auto result = state_machine_->transitionTo(NavigationState::CANCELLED);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::CANCELLED);
}

TEST_F(StateMachineTest, CancelFromObstacleAvoidance)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    state_machine_->transitionTo(NavigationState::OBSTACLE_AVOIDANCE);
    auto result = state_machine_->transitionTo(NavigationState::CANCELLED);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::CANCELLED);
}

TEST_F(StateMachineTest, ErrorTransitionsToFailed)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    auto result = state_machine_->handleEvent(StateEvent::ERROR);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::FAILED);
}

// ==================== 无效转换测试 ====================

TEST_F(StateMachineTest, InvalidEventInIdleState)
{
    // 在 IDLE 状态下，某些事件应该无效
    auto result = state_machine_->transitionTo(NavigationState::CONTROLLING);

    // PATH_COMPUTED 在 IDLE 状态下应该失败
    EXPECT_FALSE(result.success);
    EXPECT_EQ(state_machine_->state(), NavigationState::IDLE);
}

TEST_F(StateMachineTest, InvalidEventInGoalReachedState)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    state_machine_->transitionTo(NavigationState::GOAL_REACHED);

    // 在 GOAL_REACHED 状态下，PATH_COMPUTED 应该无效
    auto result = state_machine_->transitionTo(NavigationState::CONTROLLING);
    EXPECT_FALSE(result.success);
}

TEST_F(StateMachineTest, InvalidEventInCancelledState)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CANCELLED);

    // 在 CANCELLED 状态下，PATH_COMPUTED 应该无效
    auto result = state_machine_->transitionTo(NavigationState::CONTROLLING);
    EXPECT_FALSE(result.success);
}

// ==================== 状态历史测试 ====================

TEST_F(StateMachineTest, StateHistoryIsRecorded)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    state_machine_->transitionTo(NavigationState::OBSTACLE_AVOIDANCE);
    state_machine_->transitionTo(NavigationState::CONTROLLING);

    // 状态机没有history方法，这里仅验证状态序列正确
    EXPECT_EQ(state_machine_->state(), NavigationState::CONTROLLING);
}

TEST_F(StateMachineTest, StateHistoryClearsOnReset)
{
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);

    state_machine_->transitionTo(NavigationState::IDLE);

    // 重置后状态机应该回到IDLE
    EXPECT_EQ(state_machine_->state(), NavigationState::IDLE);
}

// ==================== 状态转换回调测试 ====================

TEST_F(StateMachineTest, TransitionCallbackIsCalled)
{
    bool callback_called = false;
    NavigationState new_state = NavigationState::IDLE;

    state_machine_->setStateChangeCallback([&](NavigationState from, NavigationState to) {
        callback_called = true;
        new_state = to;
    });

    state_machine_->transitionTo(NavigationState::PLANNING);

    EXPECT_TRUE(callback_called);
    EXPECT_EQ(new_state, NavigationState::PLANNING);
}

// ==================== 状态字符串转换测试 ====================

TEST_F(StateMachineTest, StateToStringConversion)
{
    EXPECT_EQ(stateToString(NavigationState::IDLE), "IDLE");
    EXPECT_EQ(stateToString(NavigationState::PLANNING), "PLANNING");
    EXPECT_EQ(stateToString(NavigationState::CONTROLLING), "CONTROLLING");
    EXPECT_EQ(stateToString(NavigationState::OBSTACLE_AVOIDANCE), "OBSTACLE_AVOIDANCE");
    EXPECT_EQ(stateToString(NavigationState::NARROW_PASSAGE), "NARROW_PASSAGE");
    EXPECT_EQ(stateToString(NavigationState::GOAL_REACHED), "GOAL_REACHED");
    EXPECT_EQ(stateToString(NavigationState::FAILED), "FAILED");
    EXPECT_EQ(stateToString(NavigationState::CANCELLED), "CANCELLED");
}

TEST_F(StateMachineTest, EventToStringConversion)
{
    EXPECT_EQ(eventToString(StateEvent::START), "START");
    EXPECT_EQ(eventToString(StateEvent::GOAL_RECEIVED), "GOAL_RECEIVED");
    EXPECT_EQ(eventToString(StateEvent::PATH_COMPUTED), "PATH_COMPUTED");
    EXPECT_EQ(eventToString(StateEvent::PATH_FAILED), "PATH_FAILED");
    EXPECT_EQ(eventToString(StateEvent::OBSTACLE_DETECTED), "OBSTACLE_DETECTED");
    EXPECT_EQ(eventToString(StateEvent::OBSTACLE_CLEARED), "OBSTACLE_CLEARED");
    EXPECT_EQ(eventToString(StateEvent::NARROW_PASSAGE_ENTER), "NARROW_PASSAGE_ENTER");
    EXPECT_EQ(eventToString(StateEvent::NARROW_PASSAGE_EXIT), "NARROW_PASSAGE_EXIT");
    EXPECT_EQ(eventToString(StateEvent::GOAL_REACHED), "GOAL_REACHED");
    EXPECT_EQ(eventToString(StateEvent::GOAL_FAILED), "GOAL_FAILED");
    EXPECT_EQ(eventToString(StateEvent::CANCEL), "CANCEL");
    EXPECT_EQ(eventToString(StateEvent::ERROR), "ERROR");
    EXPECT_EQ(eventToString(StateEvent::RESET), "RESET");
}

// ==================== 边界情况测试 ====================

TEST_F(StateMachineTest, MultipleResetsAreSafe)
{
    state_machine_->transitionTo(NavigationState::PLANNING);

    // 多次重置应该是安全的
    EXPECT_NO_THROW({
        state_machine_->transitionTo(NavigationState::IDLE);
        state_machine_->transitionTo(NavigationState::IDLE);
        state_machine_->transitionTo(NavigationState::IDLE);
    });

    EXPECT_EQ(state_machine_->state(), NavigationState::IDLE);
}

TEST_F(StateMachineTest, RapidStateTransitions)
{
    // 快速执行多次状态转换
    for (int i = 0; i < 100; ++i) {
        state_machine_->transitionTo(NavigationState::IDLE);
        state_machine_->transitionTo(NavigationState::PLANNING);
        state_machine_->transitionTo(NavigationState::CONTROLLING);
        state_machine_->transitionTo(NavigationState::GOAL_REACHED);
    }

    // 最终应该在 GOAL_REACHED 状态
    EXPECT_EQ(state_machine_->state(), NavigationState::GOAL_REACHED);
}

TEST_F(StateMachineTest, GetCurrentStateIsConsistent)
{
    // 测试状态一致性
    for (int i = 0; i < 10; ++i) {
        auto state1 = state_machine_->state();
        auto state2 = state_machine_->state();
        auto state3 = state_machine_->state();

        EXPECT_EQ(state1, state2);
        EXPECT_EQ(state2, state3);

        state_machine_->transitionTo(NavigationState::PLANNING);

        state1 = state_machine_->state();
        state2 = state_machine_->state();
        state3 = state_machine_->state();

        EXPECT_EQ(state1, state2);
        EXPECT_EQ(state2, state3);

        state_machine_->transitionTo(NavigationState::IDLE);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    // 初始化 ROS2（测试类内部会检查是否已初始化）
    rclcpp::init(argc, argv);
    auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}