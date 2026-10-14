// include/slam_core/i_global_localizer.h
#pragma once
#include <string>
#include <vector>
#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

/// 全局重定位候选
struct GlobalCandidate {
    PoseStamped pose;      // 全局位姿
    double score = 0.0;    // 打分(越大越好)
    int    id = -1;        // 匹配的关键帧 id
};

/// 全局重定位器: 无初值时在全图找回位姿(冷启动/丢失恢复).
class IGlobalLocalizer {
public:
    virtual ~IGlobalLocalizer() = default;

    virtual std::string name() const = 0;

    virtual bool init(const std::string& /*config_path*/) { return true; }

    /// @param map_path 全局地图路径(用于加载描述子库)
    /// @param global_map 全局点云
    virtual bool setMap(const std::string& map_path, const CloudType::Ptr& global_map) = 0;

    /// 尝试定位单帧; 命中返回 true 并填充 out
    virtual bool localize(const CloudType::Ptr& scan, GlobalCandidate& out) = 0;

    /// 多候选定位: 返回 Top-K 候选(按分数降序). 默认实现退化为单候选.
    virtual int localizeTopK(const CloudType::Ptr& scan, int /*max_k*/,
                             std::vector<GlobalCandidate>& out) {
        GlobalCandidate c;
        if (localize(scan, c)) { out.assign(1, c); return 1; }
        out.clear();
        return 0;
    }

    virtual void configure(const GlobalLocalizeConfig& /*cfg*/) {}
};

} // namespace rosiwit_slam
