// include/algorithms/grid_map/occupancy_logodds.h
#pragma once
#include "slam_core/i_grid_map_generator.h"

namespace rosiwit_slam {

/// 概率占据栅格 (log-odds 累积):
/// 逐帧从传感器原点做 Bresenham 射线, 途经格累加 "未命中" 负值, 端点(障碍高度带内)
/// 累加 "命中" 正值, 并裁剪; 阈值化输出 occupied/free/unknown.
/// 同一面墙被多帧命中 → 致密连续(解决漏线); 偶发噪点证据不足 → 判 unknown(抑制散射).
class LogOddsGridMap : public IGridMapGenerator {
public:
    std::string name() const override { return "occupancy_logodds"; }
    bool needsObservations() const override { return true; }

    bool build(const CloudType::Ptr& cloud,
               const Eigen::Vector3d& origin,
               const GridMapConfig& cfg,
               GridInfo& out) override;

    bool buildFromObservations(const std::vector<Observation>& obs,
                               const Eigen::Vector3d& origin,
                               const GridMapConfig& cfg,
                               GridInfo& out) override;
};

} // namespace rosiwit_slam
