// ============================================================
// Diffbot Navigation - 集成测试
// 测试模块间交互和系统整体行为
// ============================================================

#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>

#include "diffbot_navigation/core/state_machine.hpp"
#include "diffbot_navigation/core/event_bus.hpp"
#include "diffbot_navigation/core/error_manager.hpp"
#include "diffbot_navigation/core/parameter_manager.hpp"
#include "diffbot_navigation/core/exceptions.hpp"
#include "diffbot_navigation/core/types.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace diffbot_navigation::core;

class IntegrationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        // 创建所有核心组件
        state_machine_ = std::make_unique<NavigationStateMachine>();
        event_bus_ = std::make_unique<EventBus>();
        error_manager_ = std::make_unique<ErrorManager>();

        rclcpp::NodeOptions options;
        options.allow_undeclared_parameters(true);
        node_ = std::make_shared<rclcpp::Node>("integration_test_node", options);
        param_manager_ = std::make_unique<ParameterManager>(node_);

        // 初始化状态机
        state_machine_->initialize();
    }

    void TearDown() override
    {
        param_manager_.reset();
        node_.reset();
        error_manager_.reset();
        event_bus_.reset();
        state_machine_.reset();

        rclcpp::shutdown();
    }

    std::unique_ptr<NavigationStateMachine> state_machine_;
    std::unique_ptr<EventBus> event_bus_;
    std::unique_ptr<ErrorManager> error_manager_;
    std::unique_ptr<ParameterManager> param_manager_;
    rclcpp::Node::SharedPtr node_;
};

// ==================== 状态机与事件总线集成测试 ====================

TEST_F(IntegrationTest, StateMachinePublishesStateChangeEvents)
{
    std::string old_state;
    std::string new_state;
    bool event_received = false;

    // 订阅状态变化事件
    event_bus_->subscribe<StateChangeEvent>("state_change",
        [&](const StateChangeEvent& event) {
            old_state = event.old_state;
            new_state = event.new_state;
            event_received = true;
        });

    // 设置状态机回调发布事件
    state_machine_->setStateChangeCallback([&](NavigationState from, NavigationState to) {
        StateChangeEvent event(stateToString(from), stateToString(to));
        event_bus_->publish(event);
    });

    // 触发状态转换
    state_machine_->transitionTo(NavigationState::PLANNING);

    EXPECT_TRUE(event_received);
    EXPECT_EQ(old_state, "IDLE");
    EXPECT_EQ(new_state, "PLANNING");
}

TEST_F(IntegrationTest, EventBusTriggersStateMachineEvents)
{
    // 通过事件总线触发状态机事件
    event_bus_->subscribe<NavigationEvent>("trigger_planning",
        [&](const NavigationEvent&) {
            state_machine_->transitionTo(NavigationState::PLANNING);
        });

    NavigationEvent event("trigger_planning");
    event_bus_->publish(event);

    EXPECT_EQ(state_machine_->state(), NavigationState::PLANNING);
}

// ==================== 状态机与错误管理器集成测试 ====================

TEST_F(IntegrationTest, StateMachineErrorsReportedToErrorManager)
{
    bool error_reported = false;
    ErrorCode reported_code = ErrorCode::SUCCESS;

    error_manager_->registerErrorCallback([&](const ErrorInfo& error) {
        error_reported = true;
        reported_code = error.code;
    });

    // 触发无效转换
    auto result = state_machine_->transitionTo(NavigationState::CONTROLLING);  // 在 IDLE 状态无效

    if (!result.success) {
        ErrorInfo error;
        error.code = ErrorCode::PLANNING_FAILED;
        error.message = result.message;
        error.module = "StateMachine";
        error.severity = ErrorSeverity::ERROR;
        error_manager_->reportError(error);
    }

    EXPECT_TRUE(error_reported);
    EXPECT_EQ(reported_code, ErrorCode::PLANNING_FAILED);
}

// ==================== 完整导航流程集成测试 ====================

TEST_F(IntegrationTest, FullNavigationFlowSuccess)
{
    std::vector<std::string> state_sequence;

    state_machine_->setStateChangeCallback([&](NavigationState from, NavigationState to) {
        state_sequence.push_back(stateToString(from) + "->" + stateToString(to));
        StateChangeEvent event(stateToString(from), stateToString(to));
        event_bus_->publish(event);
    });

    // 完整导航流程
    // 1. IDLE -> PLANNING
    state_machine_->transitionTo(NavigationState::PLANNING);

    // 2. PLANNING -> CONTROLLING
    state_machine_->transitionTo(NavigationState::CONTROLLING);

    // 3. CONTROLLING -> OBSTACLE_AVOIDANCE -> CONTROLLING
    state_machine_->transitionTo(NavigationState::OBSTACLE_AVOIDANCE);
    state_machine_->transitionTo(NavigationState::CONTROLLING);

    // 4. CONTROLLING -> NARROW_PASSAGE -> CONTROLLING
    state_machine_->transitionTo(NavigationState::NARROW_PASSAGE);
    state_machine_->transitionTo(NavigationState::CONTROLLING);

    // 5. CONTROLLING -> GOAL_REACHED
    state_machine_->transitionTo(NavigationState::GOAL_REACHED);

    EXPECT_EQ(state_machine_->state(), NavigationState::GOAL_REACHED);

    // 验证状态序列
    EXPECT_EQ(state_sequence.size(), 7u);
    EXPECT_EQ(state_sequence[0], "IDLE->PLANNING");
    EXPECT_EQ(state_sequence[1], "PLANNING->CONTROLLING");
    EXPECT_EQ(state_sequence[2], "CONTROLLING->OBSTACLE_AVOIDANCE");
    EXPECT_EQ(state_sequence[3], "OBSTACLE_AVOIDANCE->CONTROLLING");
    EXPECT_EQ(state_sequence[4], "CONTROLLING->NARROW_PASSAGE");
    EXPECT_EQ(state_sequence[5], "NARROW_PASSAGE->CONTROLLING");
    EXPECT_EQ(state_sequence[6], "CONTROLLING->GOAL_REACHED");
}

TEST_F(IntegrationTest, FullNavigationFlowWithPlanningFailure)
{
    // 触发规划失败
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::FAILED);

    EXPECT_EQ(state_machine_->state(), NavigationState::FAILED);
}

TEST_F(IntegrationTest, FullNavigationFlowWithCancellation)
{
    // 在控制过程中取消
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    state_machine_->transitionTo(NavigationState::CANCELLED);

    EXPECT_EQ(state_machine_->state(), NavigationState::CANCELLED);
}

// ==================== 多模块并发集成测试 ====================

TEST_F(IntegrationTest, ConcurrentModuleInteractions)
{
    std::atomic<bool> running{true};
    std::atomic<int> state_changes{0};
    std::atomic<int> events_published{0};
    std::atomic<int> errors_reported{0};

    // 设置回调
    state_machine_->setStateChangeCallback([&](NavigationState, NavigationState) {
        state_changes++;
    });

    event_bus_->subscribe<NavigationEvent>("test_event",
        [&](const NavigationEvent&) {
            events_published++;
        });

    error_manager_->registerErrorCallback([&](const ErrorInfo&) {
        errors_reported++;
    });

    // 状态机线程
    std::thread state_thread([&]() {
        while (running.load()) {
            state_machine_->transitionTo(NavigationState::PLANNING);
            state_machine_->transitionTo(NavigationState::CONTROLLING);
            state_machine_->transitionTo(NavigationState::GOAL_REACHED);
            state_machine_->transitionTo(NavigationState::IDLE);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // 事件总线线程
    std::thread event_thread([&]() {
        while (running.load()) {
            NavigationEvent event("test_event");
            event_bus_->publish(event);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });

    // 错误管理器线程
    std::thread error_thread([&]() {
        while (running.load()) {
            ErrorInfo error;
            error.code = ErrorCode::OBSTACLE_DETECTED;
            error_manager_->reportError(error);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });

    // 运行一段时间
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    running = false;

    state_thread.join();
    event_thread.join();
    error_thread.join();

    // 验证所有操作都被执行
    EXPECT_GT(state_changes.load(), 0);
    EXPECT_GT(events_published.load(), 0);
    EXPECT_GT(errors_reported.load(), 0);
}

// ==================== 异常处理集成测试 ====================

TEST_F(IntegrationTest, ExceptionFlowThroughModules)
{
    bool error_logged = false;

    // 设置错误回调
    error_manager_->registerErrorCallback([&](const ErrorInfo&) {
        error_logged = true;
    });

    ExceptionContext context;
    context.function_name = "testFunction";
    context.file_name = "test_file.cpp";
    context.line_number = 100;
    context.module_name = "TestModule";

    NavigationException ex(ErrorCode::INVALID_CONFIGURATION, "Invalid parameter", context);
    // NavigationException 没有 setRecoverable 方法，恢复性由错误码决定
    // ex.isRecoverable() 会根据 ErrorCode 返回结果

    error_manager_->reportException(ex, "TestModule");

    EXPECT_TRUE(error_logged);

    auto history = error_manager_->getErrorHistory();
    EXPECT_GT(history.size(), 0u);
}

// ==================== 类型转换集成测试 ====================

TEST_F(IntegrationTest, Pose2DIntegration)
{
    Pose2D pose(1.0, 2.0, 0.5);

    // 转换为 PoseStamped
    auto pose_stamped = pose.toPoseStamped("map");

    EXPECT_DOUBLE_EQ(pose_stamped.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(pose_stamped.pose.position.y, 2.0);
    EXPECT_EQ(pose_stamped.header.frame_id, "map");

    // 再转换回来
    Pose2D recovered = Pose2D::fromPoseStamped(pose_stamped);

    EXPECT_NEAR(recovered.x, 1.0, 0.001);
    EXPECT_NEAR(recovered.y, 2.0, 0.001);
    EXPECT_NEAR(recovered.theta, 0.5, 0.001);
}

// ==================== 状态恢复集成测试 ====================

TEST_F(IntegrationTest, StateMachineRecoveryAfterError)
{
    // 进入失败状态
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::FAILED);

    EXPECT_EQ(state_machine_->state(), NavigationState::FAILED);

    // 重置并重新开始
    state_machine_->transitionTo(NavigationState::IDLE);

    EXPECT_EQ(state_machine_->state(), NavigationState::IDLE);

    // 再次尝试导航
    state_machine_->transitionTo(NavigationState::PLANNING);
    state_machine_->transitionTo(NavigationState::CONTROLLING);
    state_machine_->transitionTo(NavigationState::GOAL_REACHED);

    EXPECT_EQ(state_machine_->state(), NavigationState::GOAL_REACHED);
}

// ==================== 事件链集成测试 ====================

TEST_F(IntegrationTest, EventChainPropagation)
{
    std::vector<std::string> event_chain;

    // 第一层：接收导航事件
    event_bus_->subscribe<NavigationEvent>("start_navigation",
        [&](const NavigationEvent& event) {
            event_chain.push_back("received: " + event.goal_id);
            // 触发状态机
            state_machine_->transitionTo(NavigationState::PLANNING);
        });

    // 第二层：监听状态变化
    state_machine_->setStateChangeCallback([&](NavigationState from, NavigationState to) {
        event_chain.push_back("state: " + stateToString(from) + "->" + stateToString(to));
        // 发布状态变化事件
        StateChangeEvent state_event(stateToString(from), stateToString(to));
        event_bus_->publish(state_event);
    });

    // 第三层：监听状态变化事件
    event_bus_->subscribe<StateChangeEvent>("state_change",
        [&](const StateChangeEvent& event) {
            event_chain.push_back("state_change: " + event.old_state + "->" + event.new_state);
        });

    // 启动事件链
    NavigationEvent nav_event("start_navigation", "goal_001");
    event_bus_->publish(nav_event);

    // 验证事件链
    EXPECT_GE(event_chain.size(), 3u);
    EXPECT_TRUE(event_chain[0].find("received: goal_001") != std::string::npos);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    // rclcpp::init 在 SetUp 中调用，这里不需要重复调用
    auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}