#include "algorithms/ceres_backend/ceres_backend.h"
#include <iostream>
#include <fstream>
#include <array>

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
        // 约定: relative_pose = T_from_to, 即"to 在 from 系里的位姿"
        //   t_rel = R_from^{-1} (t_to - t_from)
        //   q_rel = R_from^{-1} * R_to
        // (与 VisualConstraintSource / ScanContextLoopClosure 的构造保持一致)
        //
        // ⚠ 历史 bug: 这里原本写成 R_from*(t_to-t_from) 与 R_to*R_from^{-1},
        //   即平移用了正向旋转、旋转用了世界系顺序 —— 与测量值约定不一致, 残差永远消不掉,
        //   优化器只能给出几十米/上百度级的解。实机表现: 后端每次"修正"都是
        //   d_trans=40m d_rot=150deg (被薄层的兜底逻辑全部拒绝), 地图实际拿不到任何回环修正。
        T diff_t[3] = { to_t[0] - from_t[0], to_t[1] - from_t[1], to_t[2] - from_t[2] };
        T from_r_inv[4] = { from_r[0], -from_r[1], -from_r[2], -from_r[3] };
        T rotated_t[3];
        ceres::UnitQuaternionRotatePoint(from_r_inv, diff_t, rotated_t);

        T rel_R[4];
        ceres::QuaternionProduct(from_r_inv, to_r, rel_R);

        T measured_R[4] = {T(q_rel_.w()), T(q_rel_.x()), T(q_rel_.y()), T(q_rel_.z())};
        T measured_inv[4] = {measured_R[0], -measured_R[1], -measured_R[2], -measured_R[3]};
        T error_R[4];
        ceres::QuaternionProduct(rel_R, measured_inv, error_R);

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
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (kf_index_.count(kf.id)) {
        KeyFrameNode& node = keyframes_[kf_index_[kf.id]];
        node.pose = kf.pose;
        return;
    }

    KeyFrameNode node;
    node.id = kf.id;
    node.pose = kf.pose;
    node.timestamp = kf.timestamp;
    if (!keyframes_.empty()) {
        const PoseStamped& prev = keyframes_.back().pose;
        node.odom_relative.time = kf.pose.time;
        node.odom_relative.rot = prev.rot.transpose() * kf.pose.rot;
        node.odom_relative.trans = prev.rot.transpose() * (kf.pose.trans - prev.trans);
        node.has_odom_relative = true;
    }
    kf_index_[kf.id] = keyframes_.size();
    keyframes_.push_back(node);
}

void CeresBackend::addConstraints(const std::vector<Constraint>& constraints) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
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
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (keyframes_.size() < 2) {
        std::cerr << "[CeresBackend] Not enough keyframes to optimize (< 2)" << std::endl;
        return false;
    }

    ceres::Problem problem;

    size_t N = keyframes_.size();
    // RAII: 使用 stack 分配的数组, 自动析构, 无需手动 delete[]
    std::vector<std::array<double, 3>> trans_blocks(N);
    std::vector<std::array<double, 4>> rot_blocks(N);
    std::vector<double*> param_blocks;
    param_blocks.reserve(N * 2);

    for (size_t i = 0; i < N; ++i) {
        auto& kf = keyframes_[i];

        trans_blocks[i][0] = kf.pose.trans.x();
        trans_blocks[i][1] = kf.pose.trans.y();
        trans_blocks[i][2] = kf.pose.trans.z();
        problem.AddParameterBlock(trans_blocks[i].data(), 3);

        Eigen::Quaterniond q(kf.pose.rot);
        rot_blocks[i][0] = q.w();
        rot_blocks[i][1] = q.x();
        rot_blocks[i][2] = q.y();
        rot_blocks[i][3] = q.z();
        // 确保四元数 w 为正, 避免不必要的参数化奇异
        if (rot_blocks[i][0] < 0.0) {
            for (int j = 0; j < 4; ++j) rot_blocks[i][j] = -rot_blocks[i][j];
        }
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
        problem.AddParameterBlock(rot_blocks[i].data(), 4, new ceres::QuaternionManifold());
#else
        problem.AddParameterBlock(rot_blocks[i].data(), 4, new ceres::QuaternionParameterization());
#endif

        param_blocks.push_back(trans_blocks[i].data());
        param_blocks.push_back(rot_blocks[i].data());
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
    std::vector<PoseStamped> before;
    before.reserve(N);
    for (const auto& kf : keyframes_) before.push_back(kf.pose);
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) {
        std::cerr << "[CeresBackend] Optimization failed: " << summary.message << std::endl;
        return false;
    }

    updatePoses(param_blocks);

    // 诊断: 证明 optimize 是否真的改变了位姿, 以及回环约束是否进入问题.
    double max_dt = 0.0, max_dr = 0.0;
    for (size_t i = 0; i < N; ++i) {
        max_dt = std::max(max_dt, (before[i].trans - keyframes_[i].pose.trans).norm());
        const Eigen::AngleAxisd aa(before[i].rot.transpose() * keyframes_[i].pose.rot);
        max_dr = std::max(max_dr, std::abs(aa.angle()) * 180.0 / M_PI);
    }
    std::cout << "[CeresBackend] optimize N=" << N
              << " loop=" << constraints_.size()
              << " cost=" << summary.initial_cost << "->" << summary.final_cost
              << " iter=" << summary.iterations.size()
              << " max_dt=" << max_dt << "m"
              << " max_dr=" << max_dr << "deg" << std::endl;

    num_optimizations_++;
    return true;
}

bool CeresBackend::getUpdatedPoses(std::vector<PoseStamped>& poses) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    poses.clear();
    for (const auto& kf : keyframes_) {
        poses.push_back(kf.pose);
    }
    return !poses.empty();
}

void CeresBackend::addOdometryConstraints(ceres::Problem& problem, std::vector<double*>& param_blocks) {
    for (size_t i = 1; i < keyframes_.size(); ++i) {
        const auto& to_kf = keyframes_[i];
        if (!to_kf.has_odom_relative) continue;

        Eigen::Matrix3d rel_R = to_kf.odom_relative.rot;
        Eigen::Vector3d rel_t = to_kf.odom_relative.trans;
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
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
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
