// src/algorithms/localization/monitor/windowed_monitor.cpp
#include "algorithms/localization/monitor/windowed_monitor.h"
#include "algorithms/localization/localization_utils.h"
#include <cmath>
#include <numeric>

namespace rosiwit_slam {

void WindowedMonitor::reset() {
    m_fitness_hist.clear();
    m_nees_hist.clear();
    m_low_count = 0;
    m_mediocre_count = 0;
    m_have_prev = false;
    m_level = MonitorLevel::INIT;
}

MonitorLevel WindowedMonitor::update(const MonitorInput& in) {
    // 1) 未收敛 / 低分: 计数判丢
    if (!in.match.converged) {
        ++m_low_count;
        if (m_low_count >= m_cfg.lost_patience_low) m_level = MonitorLevel::LOST;
        else m_level = MonitorLevel::DEGRADED;
        return m_level;
    }

    // 2) 位姿跳变
    bool jumped = false;
    if (m_have_prev) {
        V3D dt; double da = 0.0;
        poseDelta(in.pose, m_prev, dt, da);
        if (dt.norm() > m_cfg.max_trans_step || da * 57.2957795 > m_cfg.max_yaw_step_deg)
            jumped = true;
    }
    m_prev = in.pose;
    m_have_prev = true;

    if (in.match.fitness < m_cfg.lost_fitness || jumped) {
        if (in.match.fitness < m_cfg.lost_fitness) ++m_low_count;
        m_level = (m_low_count >= m_cfg.lost_patience_low) ? MonitorLevel::LOST
                                                            : MonitorLevel::DEGRADED;
        return m_level;
    }
    m_low_count = 0;

    m_fitness_hist.push_back(in.match.fitness);
    while (static_cast<int>(m_fitness_hist.size()) > m_cfg.window_length)
        m_fitness_hist.pop_front();

    if (in.match.fitness < m_cfg.credible_fitness) {
        // 持续平庸升级判丢: 单帧 0.5~0.75 可能是环境合法降级(注释见 MonitorConfig),
        // 但健康跟踪一旦建立, 连续 mediocre_patience 帧上不了 credible 线 = 大概率
        // 锁错位姿。升级 LOST 触发重定位(要么确认要么纠正, 代价一次几秒尝试)。
        if (m_cfg.mediocre_patience > 0 &&
            ++m_mediocre_count >= m_cfg.mediocre_patience) {
            m_level = MonitorLevel::LOST;
            return m_level;
        }
        m_level = MonitorLevel::DEGRADED;
        return m_level;
    }
    m_mediocre_count = 0;

    if (static_cast<int>(m_fitness_hist.size()) >= std::max(3, m_cfg.window_length / 2)) {
        const double mean = std::accumulate(m_fitness_hist.begin(), m_fitness_hist.end(), 0.0)
                            / m_fitness_hist.size();
        double var = 0.0;
        for (double f : m_fitness_hist) var += (f - mean) * (f - mean);
        var /= m_fitness_hist.size();
        const double stddev = std::sqrt(var);
        if (std::fabs(in.match.fitness - mean) > m_cfg.window_std_k * stddev + 1e-6) {
            m_level = MonitorLevel::DEGRADED;
            return m_level;
        }
    }
    // NEES 一致性: 滑窗均值超阈值 -> 估计与协方差不一致, 降级
    if (in.nees >= 0.0) {
        m_nees_hist.push_back(in.nees);
        while (static_cast<int>(m_nees_hist.size()) > m_cfg.window_length) m_nees_hist.pop_front();
        if (static_cast<int>(m_nees_hist.size()) >= std::max(3, m_cfg.window_length / 2)) {
            const double nm = std::accumulate(m_nees_hist.begin(), m_nees_hist.end(), 0.0)
                              / m_nees_hist.size();
            if (nm > m_cfg.nees_threshold) { m_level = MonitorLevel::DEGRADED; return m_level; }
        }
    }
    m_level = MonitorLevel::CREDIBLE;
    return m_level;
}

} // namespace rosiwit_slam
