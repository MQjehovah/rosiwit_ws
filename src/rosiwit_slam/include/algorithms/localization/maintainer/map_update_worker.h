// include/algorithms/localization/maintainer/map_update_worker.h
#pragma once
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

/// 地图更新线程 (2026-11-27)。
///
/// 职责划分: **定位**只负责产出每一帧的位姿与可信度; **地图更新**(账本/地图维护器的全部读写)
/// 在本线程上按提交顺序串行执行。两者之间只隔一个队列:
///   - 观测任务 (Kind::Localized): 有界, 满了丢最旧。账本证据是累积的, 丢几帧只是少几票;
///     反过来让定位等账本才是错的 (起因: 实车行驶中账本写入每帧 32ms / p90 40ms / 尖峰 400~550ms
///     全记在跟踪线程上, 跟踪线程平均每帧 126ms > 预算 100ms, 队列常满, 21% 的帧被丢)。
///   - 结算任务 (Merge / Lifelong): 不丢。它们是整段证据的一次性结算 (走出子图的关键帧 /
///     周期刷新), 丢了就没了; 生产方本身是限速的 (每 25 帧 / 每 refresh_frames 帧一次)。
///   - 维护回调 (idle_hook): 每拍 (~50ms) 至多一次。没有任务时等满一拍就跑; 有任务时在任务做完后
///     到了节拍也跑 —— 所以它既不会被持续的任务流饿死, 每次也只占调用方自己定的一片预算
///     (工作图落盘的分片标注, 几 ms)。
/// 定位线程读账本只走不加锁的快照/缓存 (revision / lastNovel* / filterDeadBase /
/// cachedPersistencyCoverage), 不碰维护器的主锁 —— 否则账本耗时会原封不动变成锁等待回到定位线程。
///
/// 线程: 入口显式把自己降回普通调度 + nice 10 (即使将来从实时线程创建也不继承实时优先级)。
/// 本类不依赖维护器: 任务怎么执行由调用方的 handler 决定。
class MapUpdateWorker {
public:
    enum class Kind { Localized, Merge, Lifelong };
    struct Job {
        Kind kind = Kind::Localized;
        PoseStamped pose;              // Localized: 输出位姿; Merge: 关键帧位姿; Lifelong: 不用
        CloudType::Ptr cloud;          // Localized: 世界系去畸变扫描; Merge: 关键帧世界系点云
        MatchResult match;             // 仅 Localized
    };
    struct Stats {
        size_t done = 0;               // 完成的任务数 (全部类型)
        size_t dropped = 0;            // 丢掉的观测任务数 (队列满丢最旧 / flush 清空; 只有 Localized 会被丢)
        size_t failed = 0;             // 执行中抛异常的任务数
        size_t queued = 0;             // 当前排队
        double last_ms = 0.0;          // 最近一次任务的墙钟耗时
        double ema_ms = 0.0;           // 任务耗时的指数平均 (α=0.02)
        double max_ms = 0.0;           // 任务耗时的最大值
    };
    /// @param handler 在本线程上执行一个任务 (可能抛异常, 本线程捕获并计数)
    /// @param idle_hook 维护回调: 每拍 (~50ms) 至多一次 (空闲时等满一拍跑, 有任务时任务做完后跑)
    /// @param capacity 排队中的观测任务 (Localized) 上限
    explicit MapUpdateWorker(std::function<void(Job&)> handler, std::function<void()> idle_hook,
                             size_t capacity = 3, std::string name = "slam_mapupd");
    ~MapUpdateWorker();
    MapUpdateWorker(const MapUpdateWorker&) = delete;
    MapUpdateWorker& operator=(const MapUpdateWorker&) = delete;

    /// 定位线程调用: 入队 (永不阻塞)。观测任务在队列已满时挤掉最旧的观测任务。
    void submit(Job job);
    /// 丢掉全部排队任务, 并等当前任务 / 维护回调结束 (换图 / 析构前调用; 之后队列是空的)。
    void flush();
    Stats stats() const;

private:
    void run();
    std::function<void(Job&)> m_handler;
    std::function<void()> m_idle_hook;
    const size_t m_capacity;
    const std::string m_name;
    mutable std::mutex m_mu;              // 保护下面全部状态
    std::condition_variable m_cv;
    std::deque<Job> m_q;
    size_t m_obs = 0;                     // m_q 里的观测任务数
    bool m_closed = false;
    bool m_idle = true;                   // 本线程正在等任务 (不在任务 / 维护回调里, 且队列空)
    Stats m_stats;
    std::thread m_thread;
};

}  // namespace rosiwit_slam
