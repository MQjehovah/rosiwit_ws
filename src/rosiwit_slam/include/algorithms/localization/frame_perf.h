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
        n_mark_ = 0;
        flag_ = false;
        active_ = true;
    }
    void locked() { t_lock_ = Clock::now(); }
    // 帧属性: 这帧开始时后台是否正在建目标 (2026-11-27)。汇总时按它分两组 —— 用来回答
    // "后台建目标到底让跟踪线程慢了多少", 比按 100 帧窗口关联精确 (窗口里只有一部分帧和建目标重叠)。
    void setFlag(bool f) { flag_ = f; }
    void matchBegin() { t_m0_ = Clock::now(); }
    // post ("其余") 之内的子阶段 (2026-11-27): 之前 post 是黑盒, 只知道偶发 1.8s 的单帧卡顿在 post 里。
    // mark(k) 记一个时间点 (k = 0..kSubMarks-1, 按执行顺序); 子阶段 k 的耗时 = 从上一个点 (k=0 时是 matchEnd)
    // 到下一个点 (最后一段到 end)。帧中途 return 时后面的 mark 没被打到, 最后一段算到 return 处。
    static constexpr int kSubMarks = 11;
    static constexpr int kSubStages = kSubMarks + 1;
    static constexpr const char* kSubNames[kSubStages] = {
        "accept", "fuse", "outlim", "misc", "prep", "ledger", "state", "covbase", "session", "merge", "save", "tail"};
    void mark(int k) {
        if (k < 0 || k >= kSubMarks) return;
        const auto now = Clock::now();
        for (int j = n_mark_; j <= k; ++j) t_mark_[j] = now;   // 帧里被跳过的 mark 记成 0 时长
        n_mark_ = std::max(n_mark_, k + 1);
    }
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
        for (int k = 0; k < kSubStages; ++k) {
            if (k > n_mark_) break;                 // 这一段还没开始 (帧提前 return)
            const Clock::time_point t0 = (k == 0) ? t_m1_ : t_mark_[k - 1];
            const Clock::time_point t1 = (k < n_mark_) ? t_mark_[k] : te;
            sub_[k].push_back(ms(t1 - t0));
        }
        solver_.push_back(solver_ms_);
        eval_.push_back(eval_ms_);
        iv_.push_back(iv1 - iv0_);
        vc_.push_back(vc1 - vc0_);
        it_.push_back(iters_);
        src_.push_back(src_pts_);
        flags_.push_back(flag_);
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
        iv_.clear(); vc_.clear(); it_.clear(); src_.clear(); flags_.clear();
        for (auto& v : sub_) v.clear();
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
        {
            // 按属性分组: 墙钟 / 求解 / 每轮求解 / 线程 CPU 的 p50 与均值
            std::vector<double> w[2], s[2], spi[2], c[2], e[2];
            for (size_t i = 0; i < wall_.size(); ++i) {
                const int g = flags_[i] ? 1 : 0;
                w[g].push_back(wall_[i]); s[g].push_back(solver_[i]); c[g].push_back(cpu_[i]); e[g].push_back(eval_[i]);
                if (it_[i] > 0) spi[g].push_back(solver_[i] / it_[i]);
            }
            auto mean = [](const std::vector<double>& v) { double a = 0; for (double x : v) a += x; return v.empty() ? 0.0 : a / v.size(); };
            std::printf("[FramePerf]   按后台建目标分组 (帧数 墙钟p50 求解p50 每轮p50 评估p50 CPU均值): 不建 %zu %.0f %.0f %.1f %.0f %.0f | 建目标中 %zu %.0f %.0f %.1f %.0f %.0f\n",
                        w[0].size(), pct(w[0], 0.5), pct(s[0], 0.5), pct(spi[0], 0.5), pct(e[0], 0.5), mean(c[0]),
                        w[1].size(), pct(w[1], 0.5), pct(s[1], 0.5), pct(spi[1], 0.5), pct(e[1], 0.5), mean(c[1]));
        }
        if (!sub_[0].empty()) {
            std::printf("[FramePerf]   post 内 p50/p90/max:");
            for (int k = 0; k < kSubStages; ++k) {
                if (sub_[k].empty()) break;
                std::printf(" %s=%.1f/%.1f/%.1f", kSubNames[k], pct(sub_[k], 0.5), pct(sub_[k], 0.9), pct(sub_[k], 1.0));
            }
            std::printf("\n");
        }
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
    bool flag_ = false;
    std::vector<char> flags_;
    Clock::time_point t_mark_[kSubMarks];
    int n_mark_ = 0;
    std::vector<double> sub_[kSubStages];
};

}  // namespace rosiwit_slam
