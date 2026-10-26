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

TEST(Thread, AbandonDoesNotBlockDestructor)
{
  // 线程体只碰 shared_ptr 保活的状态 (没有 owner 的 this), 因此可以安全 abandon
  struct State { std::atomic<bool> ran{false}; std::atomic<bool> exited{false}; };
  auto st = std::make_shared<State>();
  const auto t0 = std::chrono::steady_clock::now();
  {
    Thread t("abandon", [st](CancelToken token) {
      st->ran = true;
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (!token.cancelled() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      st->exited = true;
    });
    while (!st->ran.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    t.abandon();          // 不 join
  }                        // 析构不能等线程跑满 5s
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();
  EXPECT_LT(ms, 1000);
  // 取消位已置: 线程应该在毫秒级内自己退出
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!st->exited.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_TRUE(st->exited.load());
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
