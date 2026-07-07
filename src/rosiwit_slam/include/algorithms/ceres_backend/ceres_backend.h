#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "slam_core/i_backend.h"

namespace rosiwit_slam {

struct KeyFrameNode {
    PoseStamped pose;
    std::string id;
    int64_t timestamp = 0;
};

struct RelPoseConstraint {
    std::string from_kf;
    std::string to_kf;
    PoseStamped relative_pose;
    double cov = 1.0;
};

class CeresBackend : public IBackend {
public:
    CeresBackend();
    ~CeresBackend() override = default;

    bool init(const std::string& config_path) override;
    void addKeyFrame(const KeyFrame& kf) override;
    void addConstraints(const std::vector<Constraint>& constraints) override;
    bool optimize() override;
    bool getUpdatedPoses(std::vector<PoseStamped>& poses) override;

private:
    void addOdometryConstraints(ceres::Problem& problem, std::vector<double*>& param_blocks);
    void addLoopConstraints(ceres::Problem& problem, std::vector<double*>& param_blocks);
    void updatePoses(const std::vector<double*>& param_blocks);

    std::vector<KeyFrameNode> keyframes_;
    std::vector<RelPoseConstraint> constraints_;
    std::unordered_map<std::string, size_t> kf_index_;

    int optimize_period_ = 10;
    int num_optimizations_ = 0;
    double huber_loss_ = 1.0;
    int max_iterations_ = 50;
};

} // namespace rosiwit_slam
