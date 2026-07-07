#include "algorithms/ceres_backend/ceres_backend.h"
#include <iostream>
#include <fstream>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

struct RelativePoseError {
    RelativePoseError(const Eigen::Vector3d& t_rel, const Eigen::Quaterniond& q_rel)
        : t_rel_(t_rel), q_rel_(q_rel) {}

    template<typename T>
    bool operator()(const T* const from_t, const T* const from_r,
                    const T* const to_t, const T* const to_r,
                    T* residuals) const {
        T diff_t[3] = { to_t[0] - from_t[0], to_t[1] - from_t[1], to_t[2] - from_t[2] };
        T rotated_t[3];
        ceres::UnitQuaternionRotatePoint(from_r, diff_t, rotated_t);

        T from_r_inv[4] = { from_r[0], -from_r[1], -from_r[2], -from_r[3] };
        T rel_R[4];
        ceres::QuaternionProduct(to_r, from_r_inv, rel_R);

        T measured_R[4] = {T(q_rel_.w()), T(q_rel_.x()), T(q_rel_.y()), T(q_rel_.z())};
        T error_R[4];
        ceres::QuaternionProduct(rel_R, measured_R, error_R);

        T q_delta[3];
        ceres::QuaternionToAngleAxis(error_R, q_delta);

        residuals[0] = rotated_t[0] - T(t_rel_[0]);
        residuals[1] = rotated_t[1] - T(t_rel_[1]);
        residuals[2] = rotated_t[2] - T(t_rel_[2]);
        residuals[3] = q_delta[0];
        residuals[4] = q_delta[1];
        residuals[5] = q_delta[2];

        return true;
    }

    Eigen::Vector3d t_rel_;
    Eigen::Quaterniond q_rel_;
};

CeresBackend::CeresBackend() {}

bool CeresBackend::init(const std::string& config_path) {
    std::ifstream fin(config_path);
    if (!fin.good()) {
        std::cerr << "[CeresBackend] Config file not found: " << config_path
                  << ", using defaults" << std::endl;
        return true;
    }

#ifndef YAML_CPP_DISABLED
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["optimize_period"])
            optimize_period_ = config["optimize_period"].as<int>();
        if (config["huber_loss"])
            huber_loss_ = config["huber_loss"].as<double>();
        if (config["max_iterations"])
            max_iterations_ = config["max_iterations"].as<int>();
    } catch (const std::exception& e) {
        std::cerr << "[CeresBackend] Failed to parse config: " << e.what() << std::endl;
        return false;
    }
#else
    (void)config_path;
#endif

    return true;
}

void CeresBackend::addKeyFrame(const KeyFrame& kf) {
    if (kf_index_.count(kf.id)) {
        KeyFrameNode& node = keyframes_[kf_index_[kf.id]];
        node.pose = kf.pose;
        return;
    }

    KeyFrameNode node;
    node.id = kf.id;
    node.pose = kf.pose;
    node.timestamp = kf.timestamp;
    kf_index_[kf.id] = keyframes_.size();
    keyframes_.push_back(node);
}

void CeresBackend::addConstraints(const std::vector<Constraint>& constraints) {
    for (const auto& c : constraints) {
        RelPoseConstraint rel;
        rel.from_kf = c.from_kf;
        rel.to_kf = c.to_kf;
        rel.relative_pose = c.relative_pose;
        rel.cov = c.cov;
        constraints_.push_back(rel);
    }
}

bool CeresBackend::optimize() {
    if (keyframes_.size() < 2) {
        std::cerr << "[CeresBackend] Not enough keyframes to optimize (< 2)" << std::endl;
        return false;
    }

    ceres::Problem problem;

    size_t N = keyframes_.size();
    std::vector<double*> param_blocks;
    param_blocks.reserve(N);

    for (auto& kf : keyframes_) {
        double* t_block = new double[3];
        t_block[0] = kf.pose.trans.x();
        t_block[1] = kf.pose.trans.y();
        t_block[2] = kf.pose.trans.z();
        problem.AddParameterBlock(t_block, 3);

        Eigen::Quaterniond q(kf.pose.rot);
        double* r_block = new double[4];
        r_block[0] = q.w();
        r_block[1] = q.x();
        r_block[2] = q.y();
        r_block[3] = q.z();
        if (r_block[0] < 0.0) { r_block[0] = -r_block[0]; r_block[1] = -r_block[1]; r_block[2] = -r_block[2]; r_block[3] = -r_block[3]; }
        problem.AddParameterBlock(r_block, 4, new ceres::QuaternionParameterization());

        param_blocks.push_back(t_block);
        param_blocks.push_back(r_block);
    }

    if (param_blocks.size() > 2) {
        problem.SetParameterBlockConstant(param_blocks[0]);
        problem.SetParameterBlockConstant(param_blocks[1]);
    }

    addOdometryConstraints(problem, param_blocks);
    addLoopConstraints(problem, param_blocks);

    ceres::Solver::Options options;
    options.max_num_iterations = max_iterations_;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) {
        std::cerr << "[CeresBackend] Optimization failed: " << summary.message << std::endl;
        for (auto* block : param_blocks) delete[] block;
        return false;
    }

    updatePoses(param_blocks);

    for (auto* block : param_blocks) delete[] block;

    num_optimizations_++;
    return true;
}

bool CeresBackend::getUpdatedPoses(std::vector<PoseStamped>& poses) {
    poses.clear();
    for (const auto& kf : keyframes_) {
        poses.push_back(kf.pose);
    }
    return !poses.empty();
}

void CeresBackend::addOdometryConstraints(ceres::Problem& problem, std::vector<double*>& param_blocks) {
    for (size_t i = 1; i < keyframes_.size(); ++i) {
        const auto& from_kf = keyframes_[i - 1];
        const auto& to_kf = keyframes_[i];

        Eigen::Matrix3d from_R = from_kf.pose.rot;
        Eigen::Vector3d from_t = from_kf.pose.trans;

        Eigen::Matrix3d to_R = to_kf.pose.rot;
        Eigen::Vector3d to_t = to_kf.pose.trans;

        Eigen::Matrix3d rel_R = from_R.transpose() * to_R;
        Eigen::Vector3d rel_t = from_R.transpose() * (to_t - from_t);

        Eigen::Quaterniond rel_q(rel_R);

        size_t from_i = (i - 1) * 2;
        size_t to_i = i * 2;
        auto* cost_fn = new ceres::AutoDiffCostFunction<RelativePoseError, 6, 3, 4, 3, 4>(
            new RelativePoseError(rel_t, rel_q));
        problem.AddResidualBlock(cost_fn, new ceres::HuberLoss(huber_loss_),
                                  param_blocks[from_i], param_blocks[from_i + 1],
                                  param_blocks[to_i], param_blocks[to_i + 1]);
    }
}

void CeresBackend::addLoopConstraints(ceres::Problem& problem, std::vector<double*>& param_blocks) {
    for (const auto& c : constraints_) {
        auto from_it = kf_index_.find(c.from_kf);
        auto to_it = kf_index_.find(c.to_kf);
        if (from_it == kf_index_.end() || to_it == kf_index_.end()) continue;

        size_t from_idx = from_it->second;
        size_t to_idx = to_it->second;

        if (from_idx == to_idx) continue;

        Eigen::Vector3d t_rel(c.relative_pose.trans.x(),
                              c.relative_pose.trans.y(),
                              c.relative_pose.trans.z());
        Eigen::Quaterniond q_rel(c.relative_pose.rot);

        size_t from_i = from_idx * 2;
        size_t to_i = to_idx * 2;
        auto* cost_fn = new ceres::AutoDiffCostFunction<RelativePoseError, 6, 3, 4, 3, 4>(
            new RelativePoseError(t_rel, q_rel));

        double weight = 1.0 / c.cov;
        auto* loss = new ceres::ScaledLoss(
            new ceres::HuberLoss(huber_loss_), weight, ceres::TAKE_OWNERSHIP);

        problem.AddResidualBlock(cost_fn, loss,
                                  param_blocks[from_i], param_blocks[from_i + 1],
                                  param_blocks[to_i], param_blocks[to_i + 1]);
    }
}

void CeresBackend::updatePoses(const std::vector<double*>& param_blocks) {
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        keyframes_[i].pose.trans.x() = param_blocks[i * 2][0];
        keyframes_[i].pose.trans.y() = param_blocks[i * 2][1];
        keyframes_[i].pose.trans.z() = param_blocks[i * 2][2];

        Eigen::Quaterniond q(param_blocks[i * 2 + 1][0], param_blocks[i * 2 + 1][1],
                             param_blocks[i * 2 + 1][2], param_blocks[i * 2 + 1][3]);
        q.normalize();
        keyframes_[i].pose.rot = q.toRotationMatrix();
    }
}

} // namespace rosiwit_slam
