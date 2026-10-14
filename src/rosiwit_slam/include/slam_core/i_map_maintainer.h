// include/slam_core/i_map_maintainer.h
#pragma once
#include <string>
#include <vector>
#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

/// 地图维护阶段 (第6阶段): 长期运行时吸收环境变化.
/// 采用"非破坏性叠加": 原始地图保持冻结, 检测到的变化以叠加层(overlay)形式累积;
/// 配准目标 = 原局部地图 + overlay, 从而原图文件与索引永不被改写.
class IMapMaintainer {
public:
    virtual ~IMapMaintainer() = default;

    virtual std::string name() const = 0;

    virtual bool init(const std::string& /*config_path*/) { return true; }
    virtual void configure(const MaintainerConfig& /*cfg*/) {}

    /// 载入原始地图(冻结) 与可选关键帧
    virtual void setBaseMap(const std::string& /*map_path*/, const CloudType::Ptr& /*base*/) {}
    virtual void setKeyframes(const std::vector<LocalKeyframe>& /*keyframes*/) {}

    /// 每帧定位完成后调用; 依据 fitness 判据决定是否把该扫描并入 overlay
    virtual void onLocalized(const PoseStamped& /*pose*/, const CloudType::Ptr& /*scan_body*/,
                             const MatchResult& /*match*/) {}

    /// 返回位姿附近(半径内)的 overlay 点云(世界系), 用于叠加到配准目标
    virtual CloudType::Ptr getOverlay(const PoseStamped& /*pose*/) { return nullptr; }

    /// 持久化/载入 overlay (原图不变)
    virtual bool save(const std::string& /*path*/) { return false; }
    virtual bool load(const std::string& /*path*/) { return false; }
};

} // namespace rosiwit_slam
