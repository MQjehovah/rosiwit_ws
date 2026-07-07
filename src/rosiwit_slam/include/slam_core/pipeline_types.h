#pragma once
#include <string>
#include <vector>
#include <memory>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

struct KeyFrame {
    PoseStamped pose;
    CloudType::Ptr cloud;       // 降采样后的关键帧点云
    std::string id;
    int64_t timestamp = 0;
};

struct Constraint {
    std::string from_kf;
    std::string to_kf;
    PoseStamped relative_pose;
    double cov = 1.0;
};

} // namespace rosiwit_slam
