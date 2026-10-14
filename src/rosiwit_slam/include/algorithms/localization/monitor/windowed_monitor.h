// include/algorithms/localization/monitor/windowed_monitor.h
#pragma once
#include <deque>
#include "slam_core/i_localization_monitor.h"

namespace rosiwit_slam {

/// 滑窗监测: 在阈值基础上叠加两项稳定性判据
///   1) 位姿跳变检测 (单帧位移/航向超限判降级)
///   2) fitness 滑窗均值±kσ 一致性 (突降/突升视为异常)
class WindowedMonitor : public ILocalizationMonitor {
public:
    std::string name() const override { return "windowed"; }
    void configure(const MonitorConfig& cfg) override { m_cfg = cfg; }
    void reset() override;
    MonitorLevel update(const MonitorInput& in) override;

private:
    MonitorConfig m_cfg;
    std::deque<double> m_fitness_hist;
    std::deque<double> m_nees_hist;
    int m_low_count = 0;
    bool m_have_prev = false;
    PoseStamped m_prev;
    MonitorLevel m_level = MonitorLevel::INIT;
};

} // namespace rosiwit_slam
