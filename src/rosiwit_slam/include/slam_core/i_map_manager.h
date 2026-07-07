#pragma once
#include "slam_core/slam_types.h"
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class IMapManager {
public:
    virtual ~IMapManager() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual bool saveMap(const std::string& name) = 0;
    virtual bool loadMap(const std::string& name) = 0;
    virtual bool getGlobalMap(PointVec& out) = 0;
    virtual bool addSubMap(const KeyFrame& kf) = 0;
};

} // namespace rosiwit_slam
