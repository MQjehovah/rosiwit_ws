// include/slam_core/slam_base.h
#pragma once
#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <functional>
#include "slam_core/i_slam_algorithm.h"

namespace rosiwit_slam {

// SLAM 层的同步包:基于 IMUSample(算法无关)
struct SyncPackage {
    std::vector<IMUSample> imus;
    LidarFrame frame;                 // cloud + start/end_time
};

class SlamBase : public ISlamAlgorithm {
public:
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;   // 内部尝试同步并触发回调
    void setOutputCallback(OutputCallback cb) override { m_cb = std::move(cb); }
    SlamState state() const override { return m_state; }

    // 测试/轮询入口:尝试同步一帧,成功则调用 onSyncedPackage 并 emitOutput
    bool tryPopAndProcess();

    /// 丢弃已缓存的 IMU/LiDAR/待同步包。切模式重建前端时必须调用: 否则新一轮建图的
    /// 前端会拿**上一轮**最多 2 万条旧 IMU 去估重力方向/陀螺零偏 (2019-…实机复现为
    /// "进建图先看到残留、地图整体倾斜")。
    void clearBuffers();

    /// 同步诊断: 收到的原始条数 / 已生成的同步包数 / 当前缓冲区深度.
    /// "有话题没位姿" 时靠这组数就能分清是没收包、收包后同步不出包, 还是前端不产位姿。
    struct SyncStats {
        uint64_t imu_rx = 0;
        uint64_t lidar_rx = 0;
        uint64_t pkg_rx = 0;
        size_t   imu_buf = 0;
        size_t   lidar_buf = 0;
        bool     lidar_pushed = false;
        double   last_imu_time = -1.0;      // 最新收到 IMU 的时刻
        double   last_lidar_end = -1.0;     // 最新收到雷达帧的结束时刻
        double   pending_end = -1.0;        // 正在等 IMU 的那一帧的结束时刻
    };
    SyncStats syncStats() const;

    /// IMU 覆盖不足时是否丢弃该雷达帧 (默认 true: 保护 LIO 不用陈旧 IMU 强行传播).
    /// LIVO 前端自己缓存 IMU 并做预积分, 丢帧反而让它失去连续性 -> 这类前端应关闭.
    void setDropOnImuGap(bool v) { m_drop_on_imu_gap = v; }

protected:
    // 子类实现:处理一帧已同步数据,产出输出。返回 false 表示尚未产出
    virtual bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) = 0;
    void emitOutput(const SlamOutput& out) { if (m_cb) m_cb(out); }
    void setState(SlamState s) { m_state = s; }

private:
    bool popSyncedPackage(SyncPackage& out);

    std::deque<IMUSample>  m_imu_buf;
    std::deque<LidarFrame> m_lidar_buf;
    mutable std::mutex     m_buf_mutex;
    bool   m_lidar_pushed = false;
    bool   m_drop_on_imu_gap = true;
    double m_last_imu_time = -1.0;
    double m_last_lidar_end = -1.0;
    SyncPackage m_pending;
    OutputCallback m_cb;
    SlamState m_state = SlamState::INITIALIZING;
    std::atomic<uint64_t> m_imu_rx{0};
    std::atomic<uint64_t> m_lidar_rx{0};
    std::atomic<uint64_t> m_pkg_rx{0};
};

} // namespace rosiwit_slam
