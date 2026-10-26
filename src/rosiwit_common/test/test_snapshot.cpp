#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rosiwit_common/snapshot.hpp"

using namespace rosiwit_common;

TEST(SnapshotSlot, PublishAndLoad)
{
  SnapshotSlot<std::string> slot;
  EXPECT_FALSE(slot.has_value());
  EXPECT_EQ(slot.version(), 0u);

  slot.publish(std::make_shared<const std::string>("a"));
  EXPECT_TRUE(slot.has_value());
  EXPECT_EQ(*slot.load(), "a");
  EXPECT_EQ(slot.version(), 1u);

  slot.publish(std::make_shared<const std::string>("b"));
  EXPECT_EQ(*slot.load(), "b");
  EXPECT_EQ(slot.version(), 2u);
}

TEST(SnapshotSlot, ReaderNeverTears)
{
  SnapshotSlot<std::string> slot;
  slot.publish(std::make_shared<const std::string>(std::string(4096, 'a')));
  std::atomic<bool> stop{false};
  std::atomic<int> reads{0};
  std::thread reader([&] {
    while (!stop.load()) {
      auto s = slot.load();
      if (!s) { continue; }
      // 每份快照必须内部一致: 全是同一个字符
      const char c = (*s)[0];
      for (char ch : *s) {
        if (ch != c) { ADD_FAILURE() << "torn snapshot"; stop = true; break; }
      }
      reads.fetch_add(1);
    }
  });
  for (int i = 0; i < 200; ++i) {
    slot.publish(std::make_shared<const std::string>(
      std::string(4096, static_cast<char>('a' + (i % 26)))));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  stop = true;
  reader.join();
  EXPECT_GT(reads.load(), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
