// include/slam_core/slam_base.h
#pragma once
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
    SyncPackage m_pending;
    OutputCallback m_cb;
    SlamState m_state = SlamState::INITIALIZING;
};

} // namespace rosiwit_slam
