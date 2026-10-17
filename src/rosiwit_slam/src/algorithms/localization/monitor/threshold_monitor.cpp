// src/algorithms/localization/monitor/threshold_monitor.cpp
#include "algorithms/localization/monitor/threshold_monitor.h"

namespace rosiwit_slam {

void ThresholdMonitor::reset() {
    m_low_count = 0;
    m_mediocre_count = 0;
    m_level = MonitorLevel::INIT;
}

MonitorLevel ThresholdMonitor::update(const MonitorInput& in) {
    if (!in.match.converged || in.match.fitness < m_cfg.lost_fitness) {
        ++m_low_count;
        if (m_low_count >= m_cfg.lost_patience_low) m_level = MonitorLevel::LOST;
        else m_level = MonitorLevel::DEGRADED;
        return m_level;
    }
    m_low_count = 0;
    if (in.match.fitness >= m_cfg.credible_fitness) {
        m_mediocre_count = 0;
        m_level = MonitorLevel::CREDIBLE;
    } else {
        // 持续平庸升级判丢 (僵尸带防御, 见 MonitorConfig 注释)
        if (m_cfg.mediocre_patience > 0 &&
            ++m_mediocre_count >= m_cfg.mediocre_patience) {
            m_level = MonitorLevel::LOST;
            return m_level;
        }
        m_level = MonitorLevel::DEGRADED;
    }
    return m_level;
}

} // namespace rosiwit_slam
