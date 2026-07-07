#include "algorithms/scan_context_lc/scan_context_lc.h"
#include <iostream>
#include <fstream>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

ScanContextLC::ScanContextLC() {}

bool ScanContextLC::init(const std::string& config_path) {
    std::ifstream fin(config_path);
    if (!fin.good()) {
        std::cerr << "[ScanContextLC] Config not found: " << config_path
                  << ", using defaults" << std::endl;
        return true;
    }

#ifndef YAML_CPP_DISABLED
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["loop_radius"])
            loop_radius_ = config["loop_radius"].as<double>();
        if (config["min_keyframe_gap"])
            min_keyframe_gap_ = config["min_keyframe_gap"].as<int>();
    } catch (const std::exception& e) {
        std::cerr << "[ScanContextLC] Config parse error: " << e.what() << std::endl;
        return false;
    }
#else
    (void)config_path;
#endif

    return true;
}

void ScanContextLC::addKeyFrame(const KeyFrame& kf) {
    keyframes_.push_back(kf);
}

bool ScanContextLC::detect(PoseStamped& relative_pose, double& cov) {
    if (keyframes_.size() < 2) return false;

    const KeyFrame& latest = keyframes_.back();
    size_t n = keyframes_.size();

    for (size_t i = 0; i + (size_t)min_keyframe_gap_ < n; ++i) {
        const KeyFrame& candidate = keyframes_[i];
        Eigen::Vector3d diff = latest.pose.trans - candidate.pose.trans;
        double dist = diff.norm();

        if (dist < loop_radius_) {
            relative_pose.rot = candidate.pose.rot.transpose() * latest.pose.rot;
            relative_pose.trans = candidate.pose.rot.transpose() *
                                  (latest.pose.trans - candidate.pose.trans);
            cov = dist / loop_radius_;
            return true;
        }
    }

    return false;
}

} // namespace rosiwit_slam
