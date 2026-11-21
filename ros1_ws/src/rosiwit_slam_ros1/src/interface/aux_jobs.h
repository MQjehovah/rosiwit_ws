// ============================================================================
// aux_jobs.h — 池任务的"生命周期守卫"
//
// 池任务的任务体捕获了接口对象的 this, 所以接口析构**必须**等在途任务结束,
// 否则是 use-after-free。这里只做两件事:
//   * submit(): 计数 +1 后投递, DoneFn (一定被调用一次) 里 -1
//   * shutdown(): 关闸 + 等在途归零 (有上限, 不拖死退出)
// 不持有任何业务状态, 所以两个接口类可以各持一个。
// ============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>

#include <ros/console.h>

#include "slam_algorithm.h"

namespace rosiwit_slam_ros1 {

class AuxJobGuard {
public:
    /// body 在池线程执行 (被取消则不执行); on_finish 在任务结束时**一定**执行一次
    /// (执行完 / 被取消 / 被丢队列 三种情况), 用于清 "最新优先" 的 busy 标志。
    bool submit(SlamAlgorithm& engine, const std::string& lane, const std::string& name,
                std::function<void()> body, std::function<void()> on_finish = nullptr) {
        if (closed_.load()) return false;
        inflight_.fetch_add(1, std::memory_order_acq_rel);
        const bool ok = engine.runOnLaneWithDone(
            lane, name,
            [body](rosiwit_common::CancelToken token) {
                if (!token.cancelled() && body) body();
            },
            [this, on_finish](bool /*ran*/) {
                if (on_finish) on_finish();
                if (inflight_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                    std::lock_guard<std::mutex> lk(mu_);
                    cv_.notify_all();
                }
            });
        if (!ok) {
            // runOnLaneWithDone 在拒绝时已同步调用过 DoneFn -> 计数已归位, 这里只记日志
            ROS_WARN("[ros1_slam] aux job '%s' rejected (lane=%s)", name.c_str(), lane.c_str());
        }
        return ok;
    }

    /// 关闸并等在途任务收尾 (上限 2 s; 超时只告警, 因为继续等会卡住退出)
    void shutdown(const char* who) {
        closed_.store(true);
        std::unique_lock<std::mutex> lk(mu_);
        if (inflight_.load() == 0) return;
        if (cv_.wait_for(lk, std::chrono::seconds(2),
                         [this] { return inflight_.load() == 0; })) {
            return;
        }
        ROS_WARN("[ros1_slam] %s: %d aux job(s) still running at shutdown",
                 who, inflight_.load());
    }

private:
    std::atomic<bool> closed_{false};
    std::atomic<int> inflight_{0};
    std::mutex mu_;
    std::condition_variable cv_;
};

}  // namespace rosiwit_slam_ros1
