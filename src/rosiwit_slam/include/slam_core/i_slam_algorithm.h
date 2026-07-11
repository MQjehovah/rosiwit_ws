// include/slam_core/i_slam_algorithm.h
#pragma once
#include <string>
#include <functional>
#include <vector>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

class ISlamAlgorithm {
public:
    virtual ~ISlamAlgorithm() = default;

    virtual bool        init(const std::string& config_path) = 0;
    virtual void        onImu(const IMUSample& s) = 0;
    virtual void        onLidar(const LidarFrame& f) = 0;
    virtual SlamState   state() const = 0;
    virtual std::string name() const = 0;

    using OutputCallback = std::function<void(const SlamOutput&)>;
    virtual void        setOutputCallback(OutputCallback cb) = 0;

    // 可选:周期性提取全局地图点(供 Node 发布 cloud_map)
    virtual bool getGlobalMap(PointVec& out_points) { (void)out_points; return false; }

    // 管线控制接口 (默认空实现, 仅 SlamPipeline 覆写)
    /// 保存地图
    virtual bool saveMap(const std::string& /*path*/) { return false; }
    /// 加载地图用于定位
    virtual bool loadMap(const std::string& /*path*/) { return false; }
    /// 保存栅格地图 (PGM + YAML)
    virtual bool saveGridMap(const std::string& /*pgm_path*/, const std::string& /*yaml_path*/, double /*resolution*/) { return false; }
    /// 设置管线模式: "idle" / "mapping" / "localization"
    virtual bool setPipelineMode(const std::string& /*mode*/) { return false; }
    /// 获取当前管线模式
    virtual std::string getPipelineMode() const { return "mapping"; }
    /// 设置初始位姿 (定位模式使用)
    virtual bool setInitPose(const PoseStamped& /*pose*/) { return false; }
    /// 获取栅格地图信息 (用于定位模式发布 grid_map)
    virtual GridInfo getGridInfo() const { return {}; }
};

} // namespace rosiwit_slam
