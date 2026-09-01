// src/slam_core/slam_base.cpp
#include "slam_core/slam_base.h"
#include <algorithm>

namespace rosiwit_slam {

void SlamBase::onImu(const IMUSample& s) {
    std::lock_guard<std::mutex> lock(m_buf_mutex);
    if (s.time >= 0 && !m_imu_buf.empty() && s.time < m_imu_buf.back().time) {
        m_imu_buf.clear();   // 乱序,重置
    }
    // IMU 丢帧插值: 相邻样本间隔 >30ms(录包丢帧)时, 线性插值填满到 ~5ms,
    // 让 EKF 传播连续。长空洞插值不精确但保持姿态连续, 避免位姿跳变。
    if (!m_imu_buf.empty()) {
        const IMUSample& last = m_imu_buf.back();
        double gap = s.time - last.time;
        if (gap > 0.03 && gap < 0.8) {
            int n = std::max(1, static_cast<int>(gap / 0.005));
            for (int k = 1; k < n; ++k) {
                double a = static_cast<double>(k) / n;
                IMUSample mid;
                mid.time = last.time + a * gap;
                mid.acc  = last.acc  * (1.0 - a) + s.acc  * a;
                mid.gyro = last.gyro * (1.0 - a) + s.gyro * a;
                m_imu_buf.push_back(mid);
            }
        }
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

    // 退化处理: IMU 空洞(丢帧)时, 帧窗口内没有足够的 IMU 覆盖。
    // 短空洞已被 onImu 插值补全; 这里兜底处理插值未覆盖的大空洞帧:
    // 跳过 IMU 覆盖不足的 lidar 帧, 避免用陈旧 IMU 强行传播污染状态。
    // 注意: 判定时【不消费 IMU】, 丢弃帧时 IMU 保留在 buffer 中, 避免 IMU/lidar 失同步。
    int dropped = 0;
    while (true) {
        const double frame_begin = m_pending.frame.start_time;
        const double frame_end   = m_pending.frame.end_time;
        const double window = frame_end - frame_begin;
        bool bad = false;
        if (window > 0.05) {
            double first_in = -1.0, last_in = -1.0, prev_time = -1.0;
            int n_in = 0;
            for (const auto& imu : m_imu_buf) {
                if (imu.time < frame_begin) { prev_time = imu.time; continue; }
                if (imu.time >= frame_end) break;
                if (first_in < 0) first_in = imu.time;
                last_in = imu.time;
                n_in++;
            }
            double coverage = (n_in > 0 && last_in > first_in) ? (last_in - first_in) : 0.0;
            if (n_in < 2 || coverage < 0.7 * window) bad = true;
            // 帧起始前有超大空洞: 传播基准不可靠
            if (!bad && prev_time > 0 && (frame_begin - prev_time) > 0.2) bad = true;
        }
        if (!bad) break;

        dropped++;
        m_lidar_buf.pop_front();
        m_lidar_pushed = false;
        if (m_lidar_buf.empty()) return false;
        m_pending.frame = m_lidar_buf.front();
        m_lidar_pushed = true;
    }
    if (dropped > 0)
        std::cerr << "[sync] dropped " << dropped << " lidar frame(s) due to IMU gap" << std::endl;

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
