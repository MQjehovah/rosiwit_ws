#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "rosiwit_common/periodic.hpp"
#include "rosiwit_common/thread.hpp"

using namespace rosiwit_common;

TEST(Thread, DestructorJoinsWithoutDetach)
{
  std::atomic<int> steps{0};
  {
    Thread t("worker", [&](CancelToken token) {
      while (!token.cancelled()) {
        steps.fetch_add(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }   // 析构必须 requestStop + join 后才返回
  EXPECT_GT(steps.load(), 0);
}

TEST(Thread, RequestStopBreaksLoop)
{
  std::atomic<bool> exited{false};
  Thread t("loop", [&](CancelToken token) {
    while (!token.cancelled()) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    exited = true;
  });
  t.requestStop();
  t.join();
  EXPECT_TRUE(exited.load());
}

TEST(Thread, ExceptionDoesNotKillProcess)
{
  std::atomic<bool> after{false};
  Thread t("thrower", [&](CancelToken) {
    throw std::runtime_error("boom");
  });
  t.join();
  after = true;
  EXPECT_TRUE(after.load());
}

TEST(PeriodicThread, TicksRoughlyAtRate)
{
  std::atomic<int> ticks{0};
  PeriodicThread p("tick", std::chrono::milliseconds(10), [&] { ticks.fetch_add(1); });
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  p.requestStop();
  p.join();
  EXPECT_GE(ticks.load(), 8);      // 150ms / 10ms = 15, 留余量
  EXPECT_LE(ticks.load(), 20);
}

TEST(PeriodicThread, StopIsPrompt)
{
  std::atomic<int> ticks{0};
  PeriodicThread p("tick", std::chrono::milliseconds(50), [&] { ticks.fetch_add(1); });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const auto t0 = std::chrono::steady_clock::now();
  p.requestStop();
  p.join();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  EXPECT_LT(ms, 100);              // 不能等到下一个周期才退
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
