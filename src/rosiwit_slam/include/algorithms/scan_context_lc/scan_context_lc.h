#pragma once
#include <string>
#include <vector>
#include "slam_core/i_loop_closure.h"

namespace rosiwit_slam {

class ScanContextLC : public ILoopClosure {
public:
    ScanContextLC();
    ~ScanContextLC() override = default;

    bool init(const std::string& config_path) override;
    void addKeyFrame(const KeyFrame& kf) override;
    bool detect(PoseStamped& relative_pose, double& cov) override;

private:
    std::vector<KeyFrame> keyframes_;
    double loop_radius_ = 5.0;
    int min_keyframe_gap_ = 10;
};

} // namespace rosiwit_slam
