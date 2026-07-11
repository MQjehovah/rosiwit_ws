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

    // 栅格地图操作 (默认空实现, 由 PcdMapManager 覆写)
    virtual bool saveGridMap(const std::string& /*pgm_path*/, const std::string& /*yaml_path*/, double /*resolution*/) { return false; }
    virtual bool generateGridMap(double /*resolution*/) { return false; }
    virtual GridInfo getGridInfo() const { return {}; }
};

} // namespace rosiwit_slam
