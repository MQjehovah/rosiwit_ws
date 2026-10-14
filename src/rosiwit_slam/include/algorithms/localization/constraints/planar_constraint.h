// include/algorithms/localization/constraints/planar_constraint.h
#pragma once
#include <unordered_map>
#include <cstdint>
#include <Eigen/Eigen>
#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

/// 地面/平面约束: 用底图构建"地面模型"(每格地面高度+法向), 把位姿约束到地形.
///   - lock_zrp   : 硬锁 roll=pitch=0, z=常数 (仅保证全平区域)
///   - ground_plane: z 锁到地面高度+安装高; roll/pitch 对齐地面法向 (平地锁平/坡道跟随);
///                   地面不可用或法向倾角超限(异常/坡顶过渡)时本帧跳过.
class PlanarConstraint {
public:
    void configure(const PlanarConstraintConfig& cfg) { m_cfg = cfg; }
    void setMap(const CloudType::Ptr& base_map);   // 从底图构建地面模型
    void apply(PoseStamped& pose);                 // 约束位姿(原地); 首帧自动标定安装高

private:
    struct Cell {
        double z = 0.0;
        Eigen::Vector3d n = Eigen::Vector3d::UnitZ();
        bool ok = false;
    };
    static int64_t key(int cx, int cy);
    const Cell* query(double x, double y) const;

    PlanarConstraintConfig m_cfg;
    std::unordered_map<int64_t, Cell> m_cells;
    double m_cell = 1.0;
    bool m_have_sensor_h = false;
    double m_sensor_h = 0.0;
};

} // namespace rosiwit_slam
