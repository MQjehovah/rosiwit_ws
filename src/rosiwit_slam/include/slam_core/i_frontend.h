#pragma once
#include <memory>
#include "slam_core/slam_types.h"
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class IFrontend {
public:
    virtual ~IFrontend() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual void onImu(const IMUSample& s) = 0;
    virtual void onLidar(const LidarFrame& f) = 0;
    virtual bool getOdometry(PoseStamped& out) = 0;
    virtual bool getClouds(CloudType::Ptr& body, CloudType::Ptr& world) = 0;
    virtual bool getKeyFrame(KeyFrame& out) = 0;  // 是否为关键帧
    virtual bool getGlobalMap(PointVec& out) = 0;
};

} // namespace rosiwit_slam
