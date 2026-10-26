// ============================================================================
// thread_pool.hpp — 按 lane 组织的线程池 (IExecutor 的默认实现)
//
// lane = "一组同名、共享一条队列的执行槽":
//   threads=1  -> 串行执行器 (写盘/载图/地图修改必须如此)
//   threads=N  -> 可并行 (CPU 突发任务)
//
// 语义:
//   * submit 默认不阻塞调用方 (只有显式选 kBlock 的 lane 会阻塞)
//   * 队列满: kBlock 等待 / kDropOldest 丢最旧 / kReject 返回 kInvalidJobId
//   * cancel: 排队中 -> 出队 + done(false); 执行中 -> 只置取消位
//   * shutdown(drain): 排空后 join; shutdown(false): 取消排队任务后 join
// ============================================================================
#ifndef ROSIWIT_COMMON__THREAD_POOL_HPP_
#define ROSIWIT_COMMON__THREAD_POOL_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rosiwit_common/executor.hpp"

namespace rosiwit_common
{

class ThreadPool : public IExecutor
{
public:
  explicit ThreadPool(std::vector<LaneSpec> lanes);
  ~ThreadPool() override;

  ThreadPool(const ThreadPool &) = delete;
  ThreadPool & operator=(const ThreadPool &) = delete;

  JobId submit(
    const std::string & lane, JobOptions options,
    JobFn job, DoneFn done = DoneFn()) override;

  bool cancel(JobId id) override;

  ExecutorStats stats() const override;

  void shutdown(bool drain = true) override;

private:
  struct Job
  {
    JobId id = kInvalidJobId;
    JobOptions options;
    JobFn fn;
    DoneFn done;
    CancellationSource source;
  };

  struct Lane
  {
    std::string name;
    size_t capacity = 16;
    LaneOverflow overflow = LaneOverflow::kReject;
    std::mutex mu;
    std::condition_variable cv_job;     // 有活干/要退出
    std::condition_variable cv_space;   // 有空间 (kBlock 用)
    std::deque<Job> queue;
    bool stopping = false;
    std::vector<std::thread> threads;
  };

  void workerLoop(std::shared_ptr<Lane> lane);
  /// 队尾里挑优先级最高的 (同优先级取最早); 空则 false。
  static bool takeBest(Lane & lane, Job & out);
  void finishJob(const Job & job, bool ok);

  std::vector<std::shared_ptr<Lane>> lanes_;
  std::unordered_map<std::string, std::shared_ptr<Lane>> lane_by_name_;

  mutable std::mutex sources_mu_;
  std::unordered_map<JobId, CancellationSource> sources_;

  std::atomic<JobId> next_id_{1};
  std::atomic<bool> stopping_{false};

  std::atomic<uint64_t> submitted_{0};
  std::atomic<uint64_t> started_{0};
  std::atomic<uint64_t> completed_{0};
  std::atomic<uint64_t> failed_{0};
  std::atomic<uint64_t> cancelled_{0};
  std::atomic<uint64_t> rejected_{0};
};

}  // namespace rosiwit_common

#endif  // ROSIWIT_COMMON__THREAD_POOL_HPP_
