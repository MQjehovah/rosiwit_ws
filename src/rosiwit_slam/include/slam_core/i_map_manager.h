#pragma once
#include "slam_core/slam_types.h"
#include "slam_core/pipeline_types.h"
#include <vector>

namespace rosiwit_slam {

class IMapManager {
public:
    virtual ~IMapManager() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual bool saveMap(const std::string& name) = 0;
    virtual bool loadMap(const std::string& name) = 0;
    virtual bool getGlobalMap(PointVec& out) = 0;
    virtual bool addSubMap(const KeyFrame& kf) = 0;

    // 后端位姿图优化完成回调: corrected 为与 addSubMap 交付顺序一致的关键帧修正位姿
    // (只覆盖已交付后端的关键帧). 地图管理器据此用修正位姿重建全局地图, 使回环优化
    // 真正作用到地图上("叠图被拉回"). 默认空实现.
    virtual void onBackendOptimized(const std::vector<PoseStamped>& /*corrected*/) {}

    // 最近一次优化得到的"前沿锚点" A = G_last * P_last^{-1}: 新的原始位姿 R 应用
    // A∘R 即为其在修正坐标系下的位姿(与最新修正关键帧连续). 默认返回 false.
    virtual bool getCorrectionAnchor(PoseStamped& /*out*/) { return false; }

    // 栅格地图操作 (默认空实现, 由 PcdMapManager 覆写)
    virtual bool saveGridMap(const std::string& /*pgm_path*/, const std::string& /*yaml_path*/, double /*resolution*/) { return false; }
    virtual bool generateGridMap(double /*resolution*/) { return false; }
    virtual GridInfo getGridInfo() const { return {}; }
};

} // namespace rosiwit_slam
