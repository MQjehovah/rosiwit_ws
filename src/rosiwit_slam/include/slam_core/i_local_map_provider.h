// include/slam_core/i_local_map_provider.h
#pragma once
#include <string>
#include <vector>
#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

/// 局部地图提供者: 依据当前位姿给出用于配准的目标点云.
/// 全图(global) / 半径子图(radius_submap) / 关键帧子图(keyframe_submap) 等.
class ILocalMapProvider {
public:
    virtual ~ILocalMapProvider() = default;

    virtual std::string name() const = 0;

    virtual bool init(const std::string& /*config_path*/) { return true; }

    /// 载入全局地图 (构建索引/缓存); map_path 供需要读取伴生文件(如 .xzmap)的实现使用
    virtual void setMap(const std::string& map_path, const CloudType::Ptr& global_map) = 0;

    /// 注入关键帧 (body->world 位姿 + 机体系点云); 默认无操作
    virtual void setKeyframes(const std::vector<LocalKeyframe>& /*keyframes*/) {}

    /// 返回位姿附近的局部地图
    virtual CloudType::Ptr getLocalMap(const PoseStamped& pose) = 0;

    virtual void configure(const LocalMapConfig& /*cfg*/) {}
};

} // namespace rosiwit_slam

