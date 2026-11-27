// src/algorithms/localization/maintainer/map_update_worker.cpp
#include "algorithms/localization/maintainer/map_update_worker.h"

#include <pthread.h>
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <exception>
#include <utility>

namespace rosiwit_slam {
namespace {
constexpr auto kIdleTick = std::chrono::milliseconds(50);     // 空闲时维护回调的节拍
constexpr auto kFlushTimeout = std::chrono::seconds(5);       // flush 的保险丝 (正常是毫秒级)
}  // namespace

MapUpdateWorker::MapUpdateWorker(std::function<void(Job&)> handler, std::function<void()> idle_hook,
                                 size_t capacity, std::string name)
    : m_handler(std::move(handler)), m_idle_hook(std::move(idle_hook)),
      m_capacity(capacity > 0 ? capacity : 1), m_name(std::move(name)) {
    m_thread = std::thread([this] { run(); });
}

MapUpdateWorker::~MapUpdateWorker() {
    flush();                       // 先让当前任务与维护回调做完 (它们会访问调用方的成员)
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_closed = true;
    }
    m_cv.notify_all();
    if (m_thread.joinable()) m_thread.join();
}

void MapUpdateWorker::submit(Job job) {
    {
        std::lock_guard<std::mutex> lk(m_mu);
        if (m_closed) return;
        if (job.kind == Kind::Localized) {
            // 观测任务有界: 满了从队头起丢最旧的一个观测任务 (结算任务不动)
            while (m_obs >= m_capacity) {
                const auto it = std::find_if(m_q.begin(), m_q.end(),
                                             [](const Job& j) { return j.kind == Kind::Localized; });
                if (it == m_q.end()) break;
                m_q.erase(it);
                --m_obs;
                ++m_stats.dropped;
            }
            ++m_obs;
        }
        m_q.push_back(std::move(job));
    }
    m_cv.notify_one();
}

void MapUpdateWorker::flush() {
    std::unique_lock<std::mutex> lk(m_mu);
    m_stats.dropped += m_obs;
    m_q.clear();
    m_obs = 0;
    // 等本线程回到"等任务": 手里的任务 / 维护回调做完, 之后队列保持为空
    // (调用方保证换图期间不再 submit —— 例如持 m_state_mu, 与帧处理互斥)。
    if (!m_cv.wait_for(lk, kFlushTimeout, [this] { return m_idle; })) {
        // 不无限等: 回调卡死时不能把换图/析构也挂死。数据结构仍然安全 —— 账本入口都持主锁。
        std::fprintf(stderr, "[map_update] flush: 地图更新线程 %lus 内没有回到空闲 (仍在一次任务/维护回调里)\n",
                     static_cast<unsigned long>(kFlushTimeout.count()));
    }
}

MapUpdateWorker::Stats MapUpdateWorker::stats() const {
    std::lock_guard<std::mutex> lk(m_mu);
    Stats s = m_stats;
    s.queued = m_q.size();
    return s;
}

void MapUpdateWorker::run() {
    pthread_setname_np(pthread_self(), m_name.c_str());
    // 地图更新不是实时路径: 显式降回普通调度 + nice 10 (创建线程时若继承了实时优先级, 在这里收回;
    // 普通调度下这次调用是空操作; 无特权/被沙箱拦截时失败, 忽略)。
    {
        sched_param sp{};
        sp.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
    }
    setpriority(PRIO_PROCESS, 0, 10);

    using clk = std::chrono::steady_clock;
    auto t_hook = clk::now() - kIdleTick;     // 上次维护回调的时刻 (开局即可跑)
    for (;;) {
        Job job;
        {
            std::unique_lock<std::mutex> lk(m_mu);
            while (!m_closed && m_q.empty()) {
                m_idle = true;
                m_cv.notify_all();            // flush() 在等这一刻
                m_cv.wait_for(lk, kIdleTick, [this] { return m_closed || !m_q.empty(); });
                if (m_closed || !m_q.empty()) break;
                // 等满一拍仍没有任务: 做一次维护 (期间不算空闲)
                m_idle = false;
                lk.unlock();
                if (m_idle_hook) m_idle_hook();
                t_hook = clk::now();
                lk.lock();
            }
            if (m_closed) {
                m_idle = true;
                m_cv.notify_all();
                return;
            }
            job = std::move(m_q.front());
            m_q.pop_front();
            if (job.kind == Kind::Localized) --m_obs;
            m_idle = false;
        }
        const auto t0 = clk::now();
        bool ok = true;
        try {
            if (m_handler) m_handler(job);
        } catch (const std::exception& e) {
            ok = false;
            std::fprintf(stderr, "[map_update] 任务 (%d) 抛异常, 已跳过: %s\n",
                         static_cast<int>(job.kind), e.what());
        } catch (...) {
            ok = false;
            std::fprintf(stderr, "[map_update] 任务 (%d) 抛未知异常, 已跳过\n", static_cast<int>(job.kind));
        }
        const double ms = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
        std::unique_lock<std::mutex> lk(m_mu);
        ++m_stats.done;
        if (!ok) ++m_stats.failed;
        m_stats.last_ms = ms;
        m_stats.ema_ms = m_stats.ema_ms <= 0.0 ? ms : 0.98 * m_stats.ema_ms + 0.02 * ms;
        m_stats.max_ms = std::max(m_stats.max_ms, ms);
        // 任务做完, 到了维护节拍也做一片 —— 即使队列里还有任务: 一片有预算 (几 ms), 换来的是
        // 维护工作不会被持续的观测流饿死 (只靠"等满一拍"的话, 每 <50ms 来一个任务就永远等不到那一拍)。
        // 回调期间不持 m_mu (它里面会去拿别的锁), 也不算空闲。
        if (!m_closed && clk::now() - t_hook >= kIdleTick) {
            m_idle = false;
            lk.unlock();
            if (m_idle_hook) m_idle_hook();
            t_hook = clk::now();
        }
    }
}

}  // namespace rosiwit_slam
