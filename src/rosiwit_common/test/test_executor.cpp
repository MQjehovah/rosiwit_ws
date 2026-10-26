#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "rosiwit_common/thread_pool.hpp"

using namespace rosiwit_common;

namespace
{
std::vector<LaneSpec> TwoLanes(int serial_threads = 1, size_t cap = 64)
{
  return {
    {"serial", serial_threads, cap, LaneOverflow::kReject},
    {"cpu", 4, cap, LaneOverflow::kReject},
  };
}

void bumpPeak(std::atomic<int> & peak, int value)
{
  int prev = peak.load();
  while (value > prev && !peak.compare_exchange_weak(prev, value)) {}
}
}  // namespace

TEST(ThreadPool, SerialLaneNeverRunsTwoJobsAtOnce)
{
  ThreadPool pool(TwoLanes(1));
  std::atomic<int> in_flight{0};
  std::atomic<int> peak{0};
  std::atomic<int> done{0};
  for (int i = 0; i < 40; ++i) {
    pool.submit("serial", {"job", 0}, [&](CancelToken) {
      const int now = in_flight.fetch_add(1) + 1;
      bumpPeak(peak, now);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      in_flight.fetch_sub(1);
      done.fetch_add(1);
    });
  }
  pool.shutdown(true);
  EXPECT_EQ(done.load(), 40);
  EXPECT_EQ(peak.load(), 1);      // 串行 lane 的核心保证
}

TEST(ThreadPool, MultiThreadLaneRunsInParallel)
{
  ThreadPool pool({{"cpu", 4, 64, LaneOverflow::kReject}});
  std::atomic<int> in_flight{0};
  std::atomic<int> peak{0};
  for (int i = 0; i < 20; ++i) {
    pool.submit("cpu", {"job", 0}, [&](CancelToken) {
      const int now = in_flight.fetch_add(1) + 1;
      bumpPeak(peak, now);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      in_flight.fetch_sub(1);
    });
  }
  pool.shutdown(true);
  EXPECT_GT(peak.load(), 1);
}

TEST(ThreadPool, RejectsWhenFull)
{
  ThreadPool pool({{"serial", 1, 1, LaneOverflow::kReject}});
  std::atomic<bool> release{false};
  const JobId blocker = pool.submit("serial", {"blocker", 0}, [&](CancelToken) {
    while (!release.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
  });
  ASSERT_NE(blocker, kInvalidJobId);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const JobId queued = pool.submit("serial", {"queued", 0}, [](CancelToken) {});
  EXPECT_NE(queued, kInvalidJobId);
  const JobId rejected = pool.submit("serial", {"rejected", 0}, [](CancelToken) {});
  EXPECT_EQ(rejected, kInvalidJobId);
  EXPECT_GE(pool.stats().rejected, 1u);
  release = true;
  pool.shutdown(true);
}

TEST(ThreadPool, CancelQueuedJobCallsDoneFalse)
{
  ThreadPool pool({{"serial", 1, 8, LaneOverflow::kReject}});
  std::atomic<bool> release{false};
  std::atomic<int> ran{0};
  pool.submit("serial", {"blocker", 0}, [&](CancelToken) {
    while (!release.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
  });
  std::atomic<int> done_ok{-1};
  const JobId id = pool.submit("serial", {"victim", 0},
    [&](CancelToken) { ran.fetch_add(1); },
    [&](JobId, bool ok) { done_ok = ok ? 1 : 0; });
  ASSERT_NE(id, kInvalidJobId);
  EXPECT_TRUE(pool.cancel(id));
  EXPECT_EQ(done_ok.load(), 0);      // done(false)
  release = true;
  pool.shutdown(true);
  EXPECT_EQ(ran.load(), 0);          // 从没执行
}

TEST(ThreadPool, CancelRunningJobSeenByToken)
{
  ThreadPool pool({{"serial", 1, 8, LaneOverflow::kReject}});
  std::atomic<bool> started{false};
  std::atomic<bool> saw_cancel{false};
  const JobId id = pool.submit("serial", {"long", 0}, [&](CancelToken token) {
    started = true;
    for (int i = 0; i < 2000 && !token.cancelled(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    saw_cancel = token.cancelled();
  });
  while (!started.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
  EXPECT_TRUE(pool.cancel(id));
  pool.shutdown(true);
  EXPECT_TRUE(saw_cancel.load());
}

TEST(ThreadPool, ShutdownDrainFinishesPending)
{
  ThreadPool pool({{"serial", 1, 16, LaneOverflow::kReject}});
  std::atomic<int> done{0};
  for (int i = 0; i < 10; ++i) {
    pool.submit("serial", {"j", 0}, [&](CancelToken) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      done.fetch_add(1);
    });
  }
  pool.shutdown(true);
  EXPECT_EQ(done.load(), 10);
}

TEST(ThreadPool, ShutdownDropCancelsPending)
{
  ThreadPool pool({{"serial", 1, 16, LaneOverflow::kReject}});
  std::atomic<bool> release{false};
  std::atomic<int> ran{0};
  std::atomic<int> cancelled_done{0};
  pool.submit("serial", {"blocker", 0}, [&](CancelToken) {
    while (!release.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
  });
  for (int i = 0; i < 5; ++i) {
    pool.submit("serial", {"pending", 0},
      [&](CancelToken) { ran.fetch_add(1); },
      [&](JobId, bool ok) { if (!ok) { cancelled_done.fetch_add(1); } });
  }
  release = true;
  pool.shutdown(false);
  EXPECT_EQ(ran.load(), 0);
  EXPECT_EQ(cancelled_done.load(), 5);
}

TEST(ThreadPool, PriorityRunsHighFirst)
{
  ThreadPool pool({{"serial", 1, 16, LaneOverflow::kReject}});
  std::atomic<bool> release{false};
  std::vector<int> order;
  std::mutex order_mu;
  pool.submit("serial", {"blocker", 100}, [&](CancelToken) {
    while (!release.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
  });
  pool.submit("serial", {"low", 1}, [&](CancelToken) {
    std::lock_guard<std::mutex> lk(order_mu);
    order.push_back(1);
  });
  pool.submit("serial", {"high", 9}, [&](CancelToken) {
    std::lock_guard<std::mutex> lk(order_mu);
    order.push_back(9);
  });
  release = true;
  pool.shutdown(true);
  ASSERT_EQ(order.size(), 2u);
  EXPECT_EQ(order[0], 9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
