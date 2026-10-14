// include/slam_core/i_localization_monitor.h
#pragma once
#include <string>
#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

/// 定位健康监测: 依据配准质量 + 滑窗统计给出可信度等级, 供状态机使用.
class ILocalizationMonitor {
public:
    virtual ~ILocalizationMonitor() = default;

    virtual std::string name() const = 0;

    virtual bool init(const std::string& /*config_path*/) { return true; }

    virtual void reset() = 0;

    /// 喂入一帧结果, 返回当前等级
    virtual MonitorLevel update(const MonitorInput& in) = 0;

    virtual void configure(const MonitorConfig& /*cfg*/) {}
};

} // namespace rosiwit_slam
