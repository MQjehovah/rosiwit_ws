// include/algorithms/localization/constraints/planar_constraint.h
#pragma once
#include <unordered_map>
#include <cstdint>
#include <Eigen/Eigen>
#include "slam_core/localization_stage_types.h"
#include "slam_core/i_map_store.h"

namespace rosiwit_slam {

/// 地面/平面约束: 用底图构建"地面模型"(每格地面高度+法向), 把位姿约束到地形.
///   - lock_zrp   : 硬锁 roll=pitch=0, z=常数 (仅保证全平区域)
///   - ground_plane: z 锁到地面高度+安装高; roll/pitch 对齐地面法向 (平地锁平/坡道跟随);
///                   地面不可用或法向倾角超限(异常/坡顶过渡)时本帧跳过.
/// 若提供 IMapStore, 则地面查询走 store(GroundAt), 使地面信息落到统一底座。
class PlanarConstraint {
public:
    void configure(const PlanarConstraintConfig& cfg) { m_cfg = cfg; }
    void setStore(IMapStore* store) { m_store = store; }
    void setMap(const CloudType::Ptr& base_map);   // 从底图构建地面模型
    void apply(PoseStamped& pose, const CloudType::Ptr& scan = nullptr);  // 约束位姿(原地)

    /// 用"当前扫描的水平面" × "地图地面" 求位姿的 z 偏置 (初值用).
    /// 只依赖扫描里的地面/天花板带 + 地图地面高度, 不看竖墙(墙对 z 不敏感),
    /// 所以能在墙主导的小房间里把 z 一次贴到地面。返回 false=测不到(视野里没有水平面)。
    bool estimateGroundDz(const PoseStamped& pose, const CloudType& scan,
                          double& dz, double& z_floor_body) const;

private:
    /// scan_plane: 从扫描里提"地板/天花板"主平面, 在 map 系把它的法向转回竖直.
    /// 只修 roll/pitch, 与雷达安装角无关(不在雷达系里做).
    void applyScanPlane(PoseStamped& pose, const CloudType::Ptr& scan);

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
    IMapStore* m_store = nullptr;   // 可选: 地面查询走统一底座
};

} // namespace rosiwit_slam
