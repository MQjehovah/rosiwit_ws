// include/slam_core/diagnostics.h
#pragma once
#include <string>
#include <deque>
#include <mutex>
#include <cstdio>
#include <cstdint>

namespace rosiwit_slam {

/// 统一诊断出口: 带时间戳日志 + 飞行记录仪 (2026-10-18 日志重构).
///
/// 背景: 核心日志原本全是裸 std::cout, 落盘后没有时间戳 —— 2026-10-17 推行丢失
/// 取证时只能靠 PubProbe tick 数(10s 粒度)倒推事件时刻, "31m 瞬移发生在哪两帧之间"
/// 花了一小时。所有核心输出改走这里, 格式:
///   [2026-10-18 00:12:34.567][I][LocPipeline] 消息
///
/// 飞行记录仪: 每帧一条 FrameRecord 入环形缓冲(默认 3600 帧 = 10Hz 下 6 分钟),
/// 关键事件(event)同步入事件表; LOST 翻转时自动落盘 + 服务手动落盘 —— 现场
/// "刚才丢了"之后一个 CSV 回放全程, 不再依赖打印是否恰好被采样到。
namespace diag {

/// 带时间戳日志 (线程安全; 级别 I/W/E)
void log(char level, const char* tag, const std::string& msg);

/// 单帧记录 (定位管线每帧喂一次; 字段与 DropProbe 一致)
struct FrameRecord {
    double t = 0.0;            // 帧时间(雷达时基, s)
    int    status = 0;         // 0=INIT 1=LOCALIZED 2=LOST
    double fitness = 0.0;      // 覆盖率
    double lag = 0.0;          // |配准位姿 - 先验猜测|(m), 推行时增大=预测跟不上
    int    corr = 0;           // 有效对应点数(降采样后源点规模)
    int    tgt = 0;            // 目标局部地图点数
    int    accepted = 0;       // 本帧匹配是否被采纳
    int    coast = 0;          // 连续未采纳帧数
    int    hyp = 1;            // 当前假设数
};

/// 关键事件 (判丢/重定位/恢复/后台构建/假设切换...)
struct EventRecord {
    double t = 0.0;
    std::string tag;
    std::string msg;
};

class FlightRecorder {
public:
    static FlightRecorder& instance();

    void configure(size_t max_frames, const std::string& auto_dump_dir);
    void record(const FrameRecord& r);           // 每帧一次 (管线调用)
    void event(const std::string& tag, std::string msg);
    void setStatus(int s) { std::lock_guard<std::mutex> lk(m_mu); m_status = s; }

    /// LOST 翻转时管线自动调用 (true=刚进入 LOST)
    void onStatusChange(bool entered_lost);
    /// 手动落盘 (服务/信号触发); 返回落盘文件路径, 失败返回空
    std::string dump(const std::string& reason);

    // 测试
    size_t frameCount() const;
    size_t eventCount() const;

private:
    FlightRecorder() = default;
    mutable std::mutex m_mu;
    std::deque<FrameRecord> m_frames;
    std::deque<EventRecord> m_events;
    size_t m_max_frames = 3600;
    int m_status = 0;
    bool m_was_lost = false;
    std::string m_auto_dump_dir;    // 空=LOST 不自动落盘
    uint64_t m_dump_seq = 0;
};

}   // namespace diag
}   // namespace rosiwit_slam
