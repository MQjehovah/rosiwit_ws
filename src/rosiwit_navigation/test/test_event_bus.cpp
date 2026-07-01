// ============================================================
// Diffbot Navigation - 事件总线单元测试
// 测试事件发布订阅机制和线程安全性
// ============================================================

#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>

#include "diffbot_navigation/core/event_bus.hpp"

using namespace diffbot_navigation::core;

class EventBusTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        event_bus_ = std::make_unique<EventBus>();
    }

    void TearDown() override
    {
        event_bus_.reset();
    }

    std::unique_ptr<EventBus> event_bus_;
};

// ==================== 基本发布订阅测试 ====================

TEST_F(EventBusTest, SubscribeAndPublish)
{
    int call_count = 0;
    std::string received_type;
    
    auto id = event_bus_->subscribe<NavigationEvent>("navigation", 
        [&](const NavigationEvent& event) {
            call_count++;
            received_type = event.type;
        });
    
    EXPECT_GT(id, 0u);
    
    NavigationEvent event("navigation", "goal_123");
    event_bus_->publish(event);
    
    EXPECT_EQ(call_count, 1);
    EXPECT_EQ(received_type, "navigation");
}

TEST_F(EventBusTest, MultipleSubscribers)
{
    std::atomic<int> call_count{0};
    
    auto id1 = event_bus_->subscribe<NavigationEvent>("navigation", 
        [&](const NavigationEvent&) { call_count++; });
    
    auto id2 = event_bus_->subscribe<NavigationEvent>("navigation", 
        [&](const NavigationEvent&) { call_count++; });
    
    auto id3 = event_bus_->subscribe<NavigationEvent>("navigation", 
        [&](const NavigationEvent&) { call_count++; });
    
    NavigationEvent event("navigation");
    event_bus_->publish(event);
    
    EXPECT_EQ(call_count.load(), 3);
    
    event_bus_->unsubscribe(id1);
    event_bus_->unsubscribe(id2);
    event_bus_->unsubscribe(id3);
}

TEST_F(EventBusTest, Unsubscribe)
{
    int call_count = 0;
    
    auto id = event_bus_->subscribe<NavigationEvent>("navigation", 
        [&](const NavigationEvent&) { call_count++; });
    
    NavigationEvent event("navigation");
    event_bus_->publish(event);
    EXPECT_EQ(call_count, 1);
    
    // 取消订阅
    event_bus_->unsubscribe(id);
    
    event_bus_->publish(event);
    EXPECT_EQ(call_count, 1);  // 不应该再增加
}

TEST_F(EventBusTest, UnsubscribeInvalidIdDoesNotCrash)
{
    // 取消一个不存在的ID应该安全
    EXPECT_NO_THROW(event_bus_->unsubscribe(99999));
}

TEST_F(EventBusTest, ClearAllSubscriptions)
{
    std::atomic<int> call_count{0};
    
    event_bus_->subscribe<NavigationEvent>("navigation", 
        [&](const NavigationEvent&) { call_count++; });
    
    event_bus_->subscribe<NavigationEvent>("navigation", 
        [&](const NavigationEvent&) { call_count++; });
    
    event_bus_->subscribe<StateChangeEvent>("state_change", 
        [&](const StateChangeEvent&) { call_count++; });
    
    event_bus_->clearAllSubscriptions();
    
    NavigationEvent nav_event("navigation");
    event_bus_->publish(nav_event);
    
    StateChangeEvent state_event("old", "new");
    event_bus_->publish(state_event);
    
    EXPECT_EQ(call_count.load(), 0);
}

// ==================== 不同事件类型测试 ====================

TEST_F(EventBusTest, NavigationEvent)
{
    std::string received_goal_id;
    std::string received_frame_id;
    
    event_bus_->subscribe<NavigationEvent>("navigation",
        [&](const NavigationEvent& event) {
            received_goal_id = event.goal_id;
            received_frame_id = event.frame_id;
        });
    
    NavigationEvent event("navigation", "goal_456", "odom");
    event_bus_->publish(event);
    
    EXPECT_EQ(received_goal_id, "goal_456");
    EXPECT_EQ(received_frame_id, "odom");
}

TEST_F(EventBusTest, StateChangeEvent)
{
    std::string old_state;
    std::string new_state;
    
    event_bus_->subscribe<StateChangeEvent>("state_change",
        [&](const StateChangeEvent& event) {
            old_state = event.old_state;
            new_state = event.new_state;
        });
    
    StateChangeEvent event("IDLE", "PLANNING");
    event_bus_->publish(event);
    
    EXPECT_EQ(old_state, "IDLE");
    EXPECT_EQ(new_state, "PLANNING");
}

TEST_F(EventBusTest, ObstacleEvent)
{
    double received_distance = 0.0;
    std::string received_type;
    bool received_dangerous = false;
    
    event_bus_->subscribe<ObstacleEvent>("obstacle",
        [&](const ObstacleEvent& event) {
            received_distance = event.distance;
            received_type = event.obstacle_type;
            received_dangerous = event.is_dangerous;
        });
    
    ObstacleEvent event(1.5, "dynamic", true);
    event_bus_->publish(event);
    
    EXPECT_DOUBLE_EQ(received_distance, 1.5);
    EXPECT_EQ(received_type, "dynamic");
    EXPECT_TRUE(received_dangerous);
}

TEST_F(EventBusTest, VelocityCommandEvent)
{
    double linear_x = 0.0;
    double angular_z = 0.0;
    
    event_bus_->subscribe<VelocityCommandEvent>("velocity_command",
        [&](const VelocityCommandEvent& event) {
            linear_x = event.linear_x;
            angular_z = event.angular_z;
        });
    
    VelocityCommandEvent event(0.5, 0.3);
    event_bus_->publish(event);
    
    EXPECT_DOUBLE_EQ(linear_x, 0.5);
    EXPECT_DOUBLE_EQ(angular_z, 0.3);
}

// ==================== 线程安全测试 ====================

TEST_F(EventBusTest, ThreadSafePublish)
{
    const int num_threads = 10;
    const int events_per_thread = 100;
    std::atomic<int> total_received{0};
    
    event_bus_->subscribe<NavigationEvent>("navigation",
        [&](const NavigationEvent&) {
            total_received++;
        });
    
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&, i]() {
            for (int j = 0; j < events_per_thread; ++j) {
                NavigationEvent event("navigation", "goal_" + std::to_string(i) + "_" + std::to_string(j));
                event_bus_->publish(event);
            }
        });
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    EXPECT_EQ(total_received.load(), num_threads * events_per_thread);
}

TEST_F(EventBusTest, ThreadSafeSubscribe)
{
    const int num_threads = 10;
    const int subs_per_thread = 10;
    std::atomic<int> total_calls{0};
    std::vector<uint64_t> subscription_ids;
    std::mutex ids_mutex;
    
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&, i]() {
            for (int j = 0; j < subs_per_thread; ++j) {
                auto id = event_bus_->subscribe<NavigationEvent>("navigation",
                    [&](const NavigationEvent&) {
                        total_calls++;
                    });
                std::lock_guard<std::mutex> lock(ids_mutex);
                subscription_ids.push_back(id);
            }
        });
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    // 发布一个事件，应该被所有订阅者接收
    NavigationEvent event("navigation");
    event_bus_->publish(event);
    
    EXPECT_EQ(total_calls.load(), num_threads * subs_per_thread);
}

TEST_F(EventBusTest, ThreadSafeUnsubscribe)
{
    std::atomic<int> call_count{0};
    std::vector<uint64_t> subscription_ids;
    std::mutex ids_mutex;
    
    // 先创建一些订阅
    for (int i = 0; i < 100; ++i) {
        auto id = event_bus_->subscribe<NavigationEvent>("navigation",
            [&](const NavigationEvent&) {
                call_count++;
            });
        std::lock_guard<std::mutex> lock(ids_mutex);
        subscription_ids.push_back(id);
    }
    
    // 多线程取消订阅
    std::vector<std::thread> threads;
    for (size_t i = 0; i < subscription_ids.size(); ++i) {
        threads.emplace_back([&, i]() {
            event_bus_->unsubscribe(subscription_ids[i]);
        });
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    // 发布事件，不应该有任何回调被调用
    NavigationEvent event("navigation");
    event_bus_->publish(event);
    
    EXPECT_EQ(call_count.load(), 0);
}

// ==================== 性能测试 ====================

TEST_F(EventBusTest, PerformanceSingleSubscriber)
{
    const int num_events = 10000;
    std::atomic<int> count{0};
    
    event_bus_->subscribe<NavigationEvent>("navigation",
        [&](const NavigationEvent&) {
            count++;
        });
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_events; ++i) {
        NavigationEvent event("navigation");
        event_bus_->publish(event);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    EXPECT_EQ(count.load(), num_events);
    
    // 性能要求：10000个事件应该在100ms内处理完成
    EXPECT_LT(duration.count(), 100000);
    
    std::cout << "[Performance] Single subscriber: " 
              << num_events << " events in " 
              << duration.count() << " us ("
              << (double)num_events * 1000000 / duration.count() 
              << " events/sec)" << std::endl;
}

TEST_F(EventBusTest, PerformanceMultipleSubscribers)
{
    const int num_events = 1000;
    const int num_subscribers = 10;
    std::atomic<int> count{0};
    
    for (int i = 0; i < num_subscribers; ++i) {
        event_bus_->subscribe<NavigationEvent>("navigation",
            [&](const NavigationEvent&) {
                count++;
            });
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_events; ++i) {
        NavigationEvent event("navigation");
        event_bus_->publish(event);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    EXPECT_EQ(count.load(), num_events * num_subscribers);
    
    // 性能要求：10000个回调（1000事件*10订阅）应该在200ms内完成
    EXPECT_LT(duration.count(), 200000);
    
    std::cout << "[Performance] Multiple subscribers: " 
              << num_events << " events to " << num_subscribers << " subscribers in "
              << duration.count() << " us" << std::endl;
}

// ==================== 边界情况测试 ====================

TEST_F(EventBusTest, PublishWithNoSubscribers)
{
    NavigationEvent event("navigation");
    
    // 没有订阅者时发布应该安全
    EXPECT_NO_THROW(event_bus_->publish(event));
}

TEST_F(EventBusTest, SubscribeToNonExistentEventType)
{
    // 订阅没有发布者的事件类型应该可以
    int call_count = 0;
    
    event_bus_->subscribe<NavigationEvent>("nonexistent",
        [&](const NavigationEvent&) {
            call_count++;
        });
    
    // 不应该被调用
    EXPECT_EQ(call_count, 0);
}

TEST_F(EventBusTest, GetSubscriptionCount)
{
    EXPECT_EQ(event_bus_->getSubscriptionCount("navigation"), 0u);
    
    auto id1 = event_bus_->subscribe<NavigationEvent>("navigation", 
        [&](const NavigationEvent&) {});
    EXPECT_EQ(event_bus_->getSubscriptionCount("navigation"), 1u);
    
    auto id2 = event_bus_->subscribe<NavigationEvent>("navigation", 
        [&](const NavigationEvent&) {});
    EXPECT_EQ(event_bus_->getSubscriptionCount("navigation"), 2u);
    
    event_bus_->unsubscribe(id1);
    EXPECT_EQ(event_bus_->getSubscriptionCount("navigation"), 1u);
    
    event_bus_->unsubscribe(id2);
    EXPECT_EQ(event_bus_->getSubscriptionCount("navigation"), 0u);
}

TEST_F(EventBusTest, MultipleEventTypes)
{
    std::map<std::string, int> call_counts;
    
    event_bus_->subscribe<NavigationEvent>("navigation",
        [&](const NavigationEvent&) { call_counts["navigation"]++; });
    
    event_bus_->subscribe<StateChangeEvent>("state_change",
        [&](const StateChangeEvent&) { call_counts["state_change"]++; });
    
    event_bus_->subscribe<ObstacleEvent>("obstacle",
        [&](const ObstacleEvent&) { call_counts["obstacle"]++; });
    
    // 发布不同类型的事件
    NavigationEvent nav_event("navigation");
    event_bus_->publish(nav_event);
    
    StateChangeEvent state_event("old", "new");
    event_bus_->publish(state_event);
    
    ObstacleEvent obs_event(1.0, "test", false);
    event_bus_->publish(obs_event);
    
    // 再次发布
    event_bus_->publish(nav_event);
    
    EXPECT_EQ(call_counts["navigation"], 2);
    EXPECT_EQ(call_counts["state_change"], 1);
    EXPECT_EQ(call_counts["obstacle"], 1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}