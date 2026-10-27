#include "rosiwit_common/thread_pool.hpp"

#include <algorithm>
#include <cstdio>
#include <exception>

namespace rosiwit_common
{

ThreadPool::ThreadPool(std::vector<LaneSpec> lanes)
{
  lanes_.reserve(lanes.size());
  for (const auto & spec : lanes) {
    auto lane = std::make_shared<Lane>();
    lane->name = spec.name;
    lane->capacity = std::max<size_t>(1, spec.capacity);
    lane->overflow = spec.overflow;
    lane_by_name_[spec.name] = lane;
    lanes_.push_back(lane);
    const int n = std::max(1, spec.threads);
    lane->threads.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      lane->threads.emplace_back([this, lane]() { workerLoop(lane); });
    }
  }
}

ThreadPool::~ThreadPool()
{
  shutdown(false);
}

bool ThreadPool::takeBest(Lane & lane, Job & out)
{
  if (lane.queue.empty()) { return false; }
  auto best = lane.queue.begin();
  for (auto it = lane.queue.begin(); it != lane.queue.end(); ++it) {
    if (it->options.priority > best->options.priority) { best = it; }
  }
  out = std::move(*best);
  lane.queue.erase(best);
  return true;
}

void ThreadPool::finishJob(const Job & job, bool ok)
{
  {
    std::lock_guard<std::mutex> lk(sources_mu_);
    sources_.erase(job.id);
  }
  if (job.done) {
    // done 回调抛异常会把整个进程带走 (worker 线程里未捕获 -> std::terminate)
    try { job.done(job.id, ok); }
    catch (const std::exception & e) {
      std::fprintf(stderr, "[thread_pool] done() threw: %s\n", e.what());
    } catch (...) {
      std::fprintf(stderr, "[thread_pool] done() threw (unknown)\n");
    }
  }
}

void ThreadPool::workerLoop(std::shared_ptr<Lane> lane)
{
  for (;;) {
    Job job;
    {
      std::unique_lock<std::mutex> lk(lane->mu);
      lane->cv_job.wait(lk, [&] { return lane->stopping || !lane->queue.empty(); });
      if (!takeBest(*lane, job)) {
        if (lane->stopping) { return; }
        continue;
      }
    }
    lane->cv_space.notify_one();

    ++started_;
    bool ok = true;
    try {
      if (job.fn) { job.fn(job.source.token()); }
      if (job.source.cancelled()) { ok = false; }
    } catch (const CancelledException &) {
      ok = false;
    } catch (...) {
      ok = false;
      ++failed_;
    }
    if (ok) { ++completed_; } else if (job.source.cancelled()) { ++cancelled_; }
    finishJob(job, ok);
  }
}

JobId ThreadPool::submit(
  const std::string & lane_name, JobOptions options,
  JobFn job, DoneFn done)
{
  const auto it = lane_by_name_.find(lane_name);
  if (it == lane_by_name_.end() || stopping_.load(std::memory_order_acquire)) {
    ++rejected_;
    return kInvalidJobId;
  }
  auto lane = it->second;

  Job j;
  j.id = next_id_.fetch_add(1, std::memory_order_relaxed);
  j.options = std::move(options);
  j.fn = std::move(job);
  j.done = std::move(done);
  j.source = CancellationSource();
  {
    std::lock_guard<std::mutex> lk(sources_mu_);
    sources_[j.id] = j.source;
  }

  std::unique_lock<std::mutex> lk(lane->mu);
  if (lane->stopping) {
    ++rejected_;
    lk.unlock();
    std::lock_guard<std::mutex> lk2(sources_mu_);
    sources_.erase(j.id);
    return kInvalidJobId;
  }
  if (lane->queue.size() >= lane->capacity) {
    switch (lane->overflow) {
      case LaneOverflow::kBlock:
        lane->cv_space.wait(lk, [&] {
          return lane->stopping || lane->queue.size() < lane->capacity;
        });
        if (lane->stopping) {
          // 关停唤醒: 任务从未入队 -> 清取消源 + 回调 done(false)。
          // 否则 runOnLaneAndWait 等不到结果 (只能等超时), 还会留一条 ghost sources_ 记录。
          ++rejected_;
          lk.unlock();
          {
            std::lock_guard<std::mutex> src_lk(sources_mu_);
            sources_.erase(j.id);
          }
          finishJob(j, false);
          return kInvalidJobId;
        }
        break;
      case LaneOverflow::kDropOldest: {
        // 丢掉最旧的排队任务, 并回调 done(false)
        Job dropped = std::move(lane->queue.front());
        lane->queue.pop_front();
        lk.unlock();
        dropped.source.cancel();
        ++cancelled_;
        finishJob(dropped, false);
        lk.lock();
        break;
      }
      case LaneOverflow::kReject:
      default:
        ++rejected_;
        lk.unlock();
        std::lock_guard<std::mutex> lk2(sources_mu_);
        sources_.erase(j.id);
        return kInvalidJobId;
    }
  }
  lane->queue.push_back(std::move(j));
  ++submitted_;
  const JobId id = lane->queue.back().id;
  lk.unlock();
  lane->cv_job.notify_one();
  return id;
}

bool ThreadPool::cancel(JobId id)
{
  CancellationSource source;
  {
    std::lock_guard<std::mutex> lk(sources_mu_);
    const auto it = sources_.find(id);
    if (it == sources_.end()) { return false; }
    source = it->second;
  }
  source.cancel();   // 执行中的任务自己检查; 排队中的任务下面摘掉

  for (const auto & lane : lanes_) {
    Job removed;
    bool found = false;
    {
      std::lock_guard<std::mutex> lk(lane->mu);
      for (auto it = lane->queue.begin(); it != lane->queue.end(); ++it) {
        if (it->id == id) {
          removed = std::move(*it);
          lane->queue.erase(it);
          found = true;
          break;
        }
      }
    }
    if (found) {
      lane->cv_space.notify_all();
      ++cancelled_;
      finishJob(removed, false);
      return true;
    }
  }
  // 已经开始执行 (或刚好完成): 取消位已置, 由任务体自己决定何时退出
  return true;
}

ExecutorStats ThreadPool::stats() const
{
  ExecutorStats s;
  s.submitted = submitted_.load(std::memory_order_relaxed);
  s.started = started_.load(std::memory_order_relaxed);
  s.completed = completed_.load(std::memory_order_relaxed);
  s.failed = failed_.load(std::memory_order_relaxed);
  s.cancelled = cancelled_.load(std::memory_order_relaxed);
  s.rejected = rejected_.load(std::memory_order_relaxed);
  size_t queued = 0;
  for (const auto & lane : lanes_) {
    std::lock_guard<std::mutex> lk(lane->mu);
    queued += lane->queue.size();
  }
  s.queued = queued;
  return s;
}

void ThreadPool::shutdown(bool drain)
{
  {
    // 串行化多次 shutdown: 并发的 join 是 UB (joinable() 与 join() 之间有竞态)
    static std::mutex shutdown_mu;
    std::lock_guard<std::mutex> guard(shutdown_mu);
    if (shutdown_done_) { return; }
    shutdown_done_ = true;
  }
  stopping_.store(true, std::memory_order_release);
  for (const auto & lane : lanes_) {
    std::deque<Job> dropped;
    {
      std::lock_guard<std::mutex> lk(lane->mu);
      lane->stopping = true;
      if (!drain) {
        dropped.swap(lane->queue);
      }
    }
    lane->cv_job.notify_all();
    lane->cv_space.notify_all();
    for (auto & job : dropped) {
      job.source.cancel();
      ++cancelled_;
      finishJob(job, false);
    }
  }
  for (const auto & lane : lanes_) {
    for (auto & t : lane->threads) {
      if (t.joinable()) { t.join(); }
    }
  }
  std::lock_guard<std::mutex> lk(sources_mu_);
  sources_.clear();
}

}  // namespace rosiwit_common
