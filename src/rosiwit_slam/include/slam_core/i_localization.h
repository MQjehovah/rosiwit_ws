#pragma once
#include "slam_core/slam_types.h"
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class ILocalization {
public:
    enum Status { INIT, LOCALIZED, LOST };
    virtual ~ILocalization() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual void setMap(const std::string& map_name) = 0;
    virtual void setInitPose(const PoseStamped& pose) = 0;
    virtual void onImu(const IMUSample& s) = 0;
    virtual void onLidar(const LidarFrame& f) = 0;
    virtual Status getStatus() = 0;
    virtual bool getPose(PoseStamped& out) = 0;
};

} // namespace rosiwit_slam
