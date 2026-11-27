#pragma once
// 帧耗时分解 (2026-11-27)。只测墙钟只能看到"一帧多久", 分不清是**算得慢** (在 CPU 上跑)、
// **被抢占** (在就绪队列里等 CPU) 还是**在等别人** (锁 / 别的线程 / IO)。这里在跟踪线程上每帧记:
//   wall            本帧墙钟 (onLidar 入口 -> 出口)
//   cpu             本线程的 CPU 时间 (CLOCK_THREAD_CPUTIME_ID, user + sys)
//   wall - cpu      本帧内线程"没在运行"的时间 = 被抢占 + 阻塞。这个划分本线程自己看不到
//                   (本机内核没编 SCHEDSTATS / SCHED_DEBUG), 由进程外的状态采样给:
//                   tools/thread_sampler.py 高频读 /proc/<tid>/stat 的 R / S 状态。
//   preempted       被抢占次数 (ru_nivcsw: 时间片用完 / 被更高优先级挤下)
//   blocked         主动阻塞次数 (ru_nvcsw: 等锁 / 等工作线程 / 等 IO)
//   阶段 (墙钟):    lock = 等 m_state_mu;  pre = 预测 / 去畸变 / 目标切换 ...;
//                   solver / eval = 匹配器内部 (对齐 / 覆盖率评估);  post = 其余 (融合 / 账本 / 子图 / 发布)
// 每 period 帧汇总一行 [FramePerf] (p50 / p90 / max)。开销: 每帧 2 次 vDSO 时钟 + 2 次 getrusage。
// 全部在跟踪线程上调用, 不需要加锁。
#include <sys/resource.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

struct FramePerf {
    int period = 100;

    void begin() {
        t_begin_ = Clock::now();
        cpu0_ = cpuMs();
        rusage(vc0_, iv0_);
        t_lock_ = t_m0_ = t_m1_ = t_begin_;
        solver_ms_ = eval_ms_ = 0.0;
        iters_ = src_pts_ = 0;
        active_ = true;
    }
    void locked() { t_lock_ = Clock::now(); }
    void matchBegin() { t_m0_ = Clock::now(); }
    void matchEnd(const MatchResult& r) {
        t_m1_ = Clock::now();
        solver_ms_ = r.ms_solver;
        eval_ms_ = r.ms_eval;
        iters_ = r.iterations;
        src_pts_ = r.visible;
    }
    void end() {
        if (!active_) return;
        active_ = false;
        const Clock::time_point te = Clock::now();
        long vc1 = 0, iv1 = 0;
        rusage(vc1, iv1);
        const double wall = ms(te - t_begin_);
        wall_.push_back(wall);
        cpu_.push_back(cpuMs() - cpu0_);
        lock_.push_back(ms(t_lock_ - t_begin_));
        pre_.push_back(ms(t_m0_ - t_lock_));
        post_.push_back(ms(te - t_m1_));
        solver_.push_back(solver_ms_);
        eval_.push_back(eval_ms_);
        iv_.push_back(iv1 - iv0_);
        vc_.push_back(vc1 - vc0_);
        it_.push_back(iters_);
        src_.push_back(src_pts_);
        if (static_cast<int>(wall_.size()) < period) return;
        report();
        clear();
    }

private:
    using Clock = std::chrono::steady_clock;
    static double ms(Clock::duration d) { return std::chrono::duration<double, std::milli>(d).count(); }
    static double cpuMs() {
        timespec ts;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
        return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
    }
    static void rusage(long& vc, long& iv) {
        struct rusage ru;
        getrusage(RUSAGE_THREAD, &ru);
        vc = ru.ru_nvcsw;
        iv = ru.ru_nivcsw;
    }
    template <typename T>
    static T pct(std::vector<T> v, double p) {
        if (v.empty()) return T();
        const size_t k = std::min(v.size() - 1, static_cast<size_t>(p * (v.size() - 1)));
        std::nth_element(v.begin(), v.begin() + k, v.end());
        return v[k];
    }
    static double sum(const std::vector<long>& v) {
        double s = 0.0;
        for (long x : v) s += static_cast<double>(x);
        return s;
    }
    void clear() {
        wall_.clear(); cpu_.clear(); lock_.clear(); pre_.clear(); post_.clear(); solver_.clear(); eval_.clear();
        iv_.clear(); vc_.clear(); it_.clear(); src_.clear();
    }
    void report() {
        std::vector<double> notrun;
        notrun.reserve(wall_.size());
        for (size_t i = 0; i < wall_.size(); ++i) notrun.push_back(wall_[i] - cpu_[i]);
        const double n = static_cast<double>(wall_.size());
        std::printf("[FramePerf] n=%d ms p50/p90/max: wall=%.0f/%.0f/%.0f cpu=%.0f/%.0f/%.0f notrun=%.0f/%.0f/%.0f"
                    " | preempted/frame p50=%ld sum=%.0f  blocked/frame p50=%ld sum=%.0f\n",
                    static_cast<int>(n),
                    pct(wall_, 0.5), pct(wall_, 0.9), pct(wall_, 1.0),
                    pct(cpu_, 0.5), pct(cpu_, 0.9), pct(cpu_, 1.0),
                    pct(notrun, 0.5), pct(notrun, 0.9), pct(notrun, 1.0),
                    pct(iv_, 0.5), sum(iv_) / n, pct(vc_, 0.5), sum(vc_) / n);
        std::printf("[FramePerf]   stages p50/p90/max: lock=%.1f/%.1f/%.1f pre=%.0f/%.0f/%.0f"
                    " solver=%.0f/%.0f/%.0f eval=%.0f/%.0f/%.0f post=%.0f/%.0f/%.0f"
                    " | iter p50/max=%d/%d src_pts p50=%d\n",
                    pct(lock_, 0.5), pct(lock_, 0.9), pct(lock_, 1.0),
                    pct(pre_, 0.5), pct(pre_, 0.9), pct(pre_, 1.0),
                    pct(solver_, 0.5), pct(solver_, 0.9), pct(solver_, 1.0),
                    pct(eval_, 0.5), pct(eval_, 0.9), pct(eval_, 1.0),
                    pct(post_, 0.5), pct(post_, 0.9), pct(post_, 1.0),
                    pct(it_, 0.5), pct(it_, 1.0), pct(src_, 0.5));
        std::fflush(stdout);
    }

    Clock::time_point t_begin_, t_lock_, t_m0_, t_m1_;
    double cpu0_ = 0.0, solver_ms_ = 0.0, eval_ms_ = 0.0;
    long vc0_ = 0, iv0_ = 0;
    int iters_ = 0, src_pts_ = 0;
    bool active_ = false;
    std::vector<double> wall_, cpu_, lock_, pre_, post_, solver_, eval_;
    std::vector<long> iv_, vc_;
    std::vector<int> it_, src_;
};

}  // namespace rosiwit_slam
