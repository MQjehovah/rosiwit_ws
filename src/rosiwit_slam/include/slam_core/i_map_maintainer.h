// include/slam_core/i_map_maintainer.h
#pragma once
#include <string>
#include <vector>
#include <cstdint>
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

    /// 每帧定位完成后调用.
    /// @param pose_confident 本帧位姿是否可信(由配准采纳判据决定); false(过桥外推)时
    ///        不写入/不雕刻, 避免把变化写到漂移位姿上而污染地图.
    virtual void onLocalized(const PoseStamped& /*pose*/, const CloudType::Ptr& /*scan_body*/,
                             const MatchResult& /*match*/, bool /*pose_confident*/) {}

    /// 返回位姿附近(半径内)的 overlay 点云(世界系), 用于叠加到配准目标
    virtual CloudType::Ptr getOverlay(const PoseStamped& /*pose*/) { return nullptr; }

    /// 返回全部 overlay 点云(世界系), 供刷新全局重定位器 (lifelong)
    virtual CloudType::Ptr snapshotOverlay() { return nullptr; }

    /// 叠加层内容版本号: 任何点写入/删除/载入时自增。
    /// 配准目标缓存用它判断失效 —— 静态环境下 overlay 不增长, 目标(降采样+KD树)
    /// 可跨帧复用, 不必每帧重建(实测每帧重建把位姿输出拖到 10Hz, 雷达是 15.8Hz)。
    virtual uint64_t revision() const { return 0; }

    /// 取出并清空"自上次调用以来新增"的 overlay 点 (供存储底座增量插入)
    virtual void popOverlayDelta(CloudType::Ptr& out) { out.reset(); }

    /// 持久化/载入 overlay (原图不变)
    virtual bool save(const std::string& /*path*/) { return false; }
    virtual bool load(const std::string& /*path*/) { return false; }
};

} // namespace rosiwit_slam
