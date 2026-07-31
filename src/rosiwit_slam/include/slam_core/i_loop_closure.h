#pragma once
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class ILoopClosure {
public:
    virtual ~ILoopClosure() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual void addKeyFrame(const KeyFrame& kf) = 0;
    virtual bool detect(PoseStamped& relative_pose, double& cov,
                        std::string& matched_kf_id) = 0;
    virtual bool saveDatabase(const std::string& path) { (void)path; return false; }
};

} // namespace rosiwit_slam
