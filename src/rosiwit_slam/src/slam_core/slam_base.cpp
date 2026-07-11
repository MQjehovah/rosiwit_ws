// src/slam_core/slam_base.cpp
#include "slam_core/slam_base.h"
#include <algorithm>

namespace rosiwit_slam {

void SlamBase::onImu(const IMUSample& s) {
    std::lock_guard<std::mutex> lock(m_buf_mutex);
    if (s.time >= 0 && !m_imu_buf.empty() && s.time < m_imu_buf.back().time) {
        m_imu_buf.clear();   // 乱序,重置
    }
    m_imu_buf.push_back(s);
    // fix(b): 不在此触发同步,由 Node timer 周期调用 tryPopAndProcess()
}

void SlamBase::onLidar(const LidarFrame& f) {
    std::lock_guard<std::mutex> lock(m_buf_mutex);
    if (!m_lidar_buf.empty() && f.start_time < m_lidar_buf.back().start_time) {
        m_lidar_buf.clear();
    }
    m_lidar_buf.push_back(f);
    // fix(b): 不在此触发同步,避免持锁递归 lock 同一 m_buf_mutex 导致死锁
}

bool SlamBase::popSyncedPackage(SyncPackage& out) {
    // 调用方持锁
    if (m_imu_buf.empty() || m_lidar_buf.empty()) return false;
    if (!m_lidar_pushed) {
        m_pending.frame = m_lidar_buf.front();
        // 按点时间排序(沿用现状对 curvature 的排序语义由上层 Node 完成)
        m_lidar_pushed = true;
    }
    if (m_imu_buf.back().time < m_pending.frame.end_time) return false;

    m_pending.imus.clear();
    while (!m_imu_buf.empty() && m_imu_buf.front().time < m_pending.frame.end_time) {
        m_pending.imus.push_back(std::move(m_imu_buf.front()));
        m_imu_buf.pop_front();
    }
    out = std::move(m_pending);
    m_lidar_buf.pop_front();
    m_lidar_pushed = false;
    return true;
}

bool SlamBase::tryPopAndProcess() {
    SyncPackage pkg;
    {
        std::lock_guard<std::mutex> lock(m_buf_mutex);
        if (!popSyncedPackage(pkg)) return false;
    }
    SlamOutput out;
    if (onSyncedPackage(pkg, out)) emitOutput(out);
    return true;
}

} // namespace rosiwit_slam
