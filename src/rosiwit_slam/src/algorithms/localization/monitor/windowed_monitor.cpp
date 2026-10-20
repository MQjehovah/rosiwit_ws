// src/algorithms/localization/monitor/windowed_monitor.cpp
#include "algorithms/localization/monitor/windowed_monitor.h"
#include "algorithms/localization/localization_utils.h"
#include <cmath>
#include <numeric>

namespace rosiwit_slam {

void WindowedMonitor::reset() {
    m_fitness_hist.clear();
    m_nees_hist.clear();
    m_have_prev = false;
    m_level = MonitorLevel::INIT;
}

MonitorLevel WindowedMonitor::update(const MonitorInput& in) {
    // 这里只做"当前帧健康与否"的分类；连续帧判丢统一由 LocalizationPipeline
    // 的 lost_patience 负责，避免 monitor 和 pipeline 各维护一套计数器。
    if (!in.match.converged) {
        m_level = MonitorLevel::DEGRADED;
        return m_level;
    }

    // 位姿跳变
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
        m_level = MonitorLevel::DEGRADED;
        return m_level;
    }

    m_fitness_hist.push_back(in.match.fitness);
    while (static_cast<int>(m_fitness_hist.size()) > m_cfg.window_length)
        m_fitness_hist.pop_front();

    if (in.match.fitness < m_cfg.credible_fitness) {
        m_level = MonitorLevel::DEGRADED;
        return m_level;
    }

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
