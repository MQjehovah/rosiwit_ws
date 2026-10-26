#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "rosiwit_common/channel.hpp"

using namespace rosiwit_common;

TEST(Channel, DropOldestKeepsFresh)
{
  Channel<int> ch(2, OverflowPolicy::kDropOldest);
  EXPECT_TRUE(ch.push(1));
  EXPECT_TRUE(ch.push(2));
  EXPECT_TRUE(ch.push(3));   // 丢 1
  EXPECT_TRUE(ch.push(4));   // 丢 2
  EXPECT_EQ(ch.size(), 2u);
  EXPECT_EQ(ch.pop().value(), 3);
  EXPECT_EQ(ch.pop().value(), 4);
  const auto st = ch.stats();
  EXPECT_EQ(st.dropped_oldest, 2u);
  EXPECT_EQ(st.pushed, 4u);
}

TEST(Channel, DropNewestRejects)
{
  Channel<int> ch(1, OverflowPolicy::kDropNewest);
  EXPECT_TRUE(ch.push(1));
  EXPECT_FALSE(ch.push(2));
  EXPECT_EQ(ch.stats().dropped_newest, 1u);
  EXPECT_EQ(ch.pop().value(), 1);
}

TEST(Channel, CloseWakesBlockedPop)
{
  Channel<int> ch(4);
  std::atomic<bool> got_nullopt{false};
  std::thread t([&] {
    auto v = ch.pop();
    got_nullopt = !v.has_value();
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ch.close();
  t.join();
  EXPECT_TRUE(got_nullopt.load());
  EXPECT_FALSE(ch.push(1));
  EXPECT_EQ(ch.stats().rejected_closed, 1u);
}

TEST(Channel, PopForTimesOut)
{
  Channel<int> ch(2);
  const auto t0 = std::chrono::steady_clock::now();
  const auto v = ch.pop_for(std::chrono::milliseconds(40));
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  EXPECT_FALSE(v.has_value());
  EXPECT_GE(ms, 30);
}

TEST(Channel, BlockPolicyWaitsForSpace)
{
  Channel<int> ch(1, OverflowPolicy::kBlock);
  EXPECT_TRUE(ch.push(1));
  std::atomic<bool> pushed{false};
  std::thread producer([&] {
    pushed = ch.push(2);      // 必须等消费者
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_FALSE(pushed.load());
  EXPECT_EQ(ch.pop().value(), 1);
  producer.join();
  EXPECT_TRUE(pushed.load());
  EXPECT_EQ(ch.pop().value(), 2);
}

TEST(Channel, ClearDropsQueuedItems)
{
  Channel<int> ch(4);
  EXPECT_TRUE(ch.push(1));
  EXPECT_TRUE(ch.push(2));
  EXPECT_EQ(ch.clear(), 2u);
  EXPECT_EQ(ch.size(), 0u);
  EXPECT_FALSE(ch.pop_for(std::chrono::milliseconds(5)).has_value());
  // 清空后还能继续用
  EXPECT_TRUE(ch.push(3));
  EXPECT_EQ(ch.pop().value(), 3);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
