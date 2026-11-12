#pragma once
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class IBackend {
public:
    virtual ~IBackend() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual void addKeyFrame(const KeyFrame& kf) = 0;
    virtual void addConstraints(const std::vector<Constraint>& constraints) = 0;
    virtual bool optimize() = 0;
    virtual bool getUpdatedPoses(std::vector<PoseStamped>& poses) = 0;
    /// 轮速因子开启时需要周期性优化 (默认 false: 由回环触发即可)
    virtual bool wantsPeriodicOptimize() const { return false; }
};

} // namespace rosiwit_slam
