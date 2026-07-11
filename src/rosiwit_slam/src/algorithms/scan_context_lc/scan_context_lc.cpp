#include "algorithms/scan_context_lc/scan_context_lc.h"
#include <iostream>
#include <fstream>
#include <cmath>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

ScanContextLC::ScanContextLC() {}

bool ScanContextLC::init(const std::string& config_path) {
    std::ifstream fin(config_path);
    if (!fin.good()) {
        std::cerr << "[ScanContextLC] Config not found: " << config_path
                  << ", using defaults (loop_radius=" << loop_radius_
                  << ", min_keyframe_gap=" << min_keyframe_gap_ << ")"
                  << std::endl;
        return true;
    }

#ifndef YAML_CPP_DISABLED
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["loop_closure"]) {
            auto lc = config["loop_closure"];
            loop_radius_      = lc["radius"].as<double>(loop_radius_);
            min_keyframe_gap_ = lc["min_gap"].as<int>(min_keyframe_gap_);
        } else {
            // 兼容旧配置扁平格式
            loop_radius_      = config["loop_radius"].as<double>(loop_radius_);
            min_keyframe_gap_ = config["min_keyframe_gap"].as<int>(min_keyframe_gap_);
        }
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
    const size_t n = keyframes_.size();

    // 从最早的关键帧开始遍历, 跳过最近的 min_keyframe_gap_ 个帧
    const size_t search_end = n - 1 - static_cast<size_t>(min_keyframe_gap_);

    for (size_t i = 0; i < search_end; ++i) {
        const KeyFrame& candidate = keyframes_[i];
        const Eigen::Vector3d diff = latest.pose.trans - candidate.pose.trans;
        const double dist = diff.norm();

        if (dist < loop_radius_) {
            // 计算相对位姿: 从 candidate 坐标系到 latest 坐标系的变换
            relative_pose.rot = candidate.pose.rot.transpose() * latest.pose.rot;
            relative_pose.trans = candidate.pose.rot.transpose() *
                                  (latest.pose.trans - candidate.pose.trans);
            // cov = 归一化置信度 (0~1 之间, 距离越近置信度越高)
            cov = std::max(0.01, 1.0 - dist / loop_radius_);
            return true;
        }
    }

    return false;
}

} // namespace rosiwit_slam
