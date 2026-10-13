// include/slam_core/i_grid_map_generator.h
#pragma once
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

/// 单帧观测: 世界坐标系下的传感器原点 + 该帧的扫描终点(已转到世界系).
/// 射线投射类算法需要逐帧原点来区分 free/occupied/unknown.
struct Observation {
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    CloudType::Ptr cloud;
};

/// 2D 占据栅格生成配置 (由策略实现解释, 不同策略可只使用其中部分字段)
struct GridMapConfig {
    double resolution = 0.05;     // 目标栅格分辨率(m)
    double min_height = 0.1;      // 障碍物最低高度(地面以上, m)
    double max_height = 2.0;      // 障碍物最高高度(地面以上, m)
    int    occupied_thresh = 1;   // 单格判为占据所需最少点数
    int    dilate_radius = -1;    // 障碍膨胀半径(格); <0 时按 point_spacing 自动
    double point_spacing = 0.0;   // 建图点云平均点间距(m), 用于自动膨胀
    double max_range = 0.0;       // 射线有效距离(m); <=0 表示不限
};

/// 栅格地图生成策略接口.
/// 输入为已配准到世界坐标系的整体点云, 输出统一的 GridInfo 数据结构.
/// 具体算法(高度投影/射线投射/OctoMap 等)由实现类提供, 通过 GridMapFactory 按名创建.
class IGridMapGenerator {
public:
    virtual ~IGridMapGenerator() = default;

    /// 策略名称 (与工厂注册名一致)
    virtual std::string name() const = 0;

    /// 可选初始化 (读取各策略私有参数), 默认无操作
    virtual bool init(const std::string& /*config_path*/) { return true; }

    /// 是否偏好逐帧观测(含传感器原点)输入; 返回 true 时调用方应优先使用
    /// buildFromObservations() 以保留射线起点信息. 默认 false.
    virtual bool needsObservations() const { return false; }

    /// 生成栅格地图. 成功返回 true 并填充 out.
    /// @param cloud  世界坐标系下的点云 (允许为空/含 NaN, 由实现自行过滤)
    /// @param origin 地图原点(通常为传感器起点), 用于扩展包围盒/射线起点
    /// @param cfg    生成配置
    /// @param out    输出栅格 (width/height/resolution/origin_x/origin_y/data)
    virtual bool build(const CloudType::Ptr& cloud,
                       const Eigen::Vector3d& origin,
                       const GridMapConfig& cfg,
                       GridInfo& out) = 0;

    /// 多帧观测输入. 默认实现: 合并所有帧点云后调用 build() (等价于单帧).
    /// 射线投射等算法应覆写此方法以逐帧投射.
    virtual bool buildFromObservations(const std::vector<Observation>& obs,
                                       const Eigen::Vector3d& origin,
                                       const GridMapConfig& cfg,
                                       GridInfo& out) {
        CloudType::Ptr merged(new CloudType());
        for (const auto& o : obs) {
            if (o.cloud) *merged += *o.cloud;
        }
        return build(merged, origin, cfg, out);
    }
};

} // namespace rosiwit_slam
