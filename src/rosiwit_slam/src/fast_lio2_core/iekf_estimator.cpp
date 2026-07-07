/**
 * @file iekf_estimator.cpp
 * @brief FAST-LIO2 SLAM - IEKF 状态估计器实现
 */

#include "fast_lio2_slam/fast_lio2_core/iekf_estimator.h"
#include "fast_lio2_slam/common/utils.h"

namespace fast_lio2_slam {

IekfEstimator::IekfEstimator()
    : initialized_(false), update_count_(0) {
    P_ = Eigen::Matrix<double, 24, 24>::Identity() * 1e-6;
    Q_ = Eigen::Matrix<double, 12, 12>::Identity();
}

IekfEstimator::IekfEstimator(const IekfConfig& config)
    : config_(config), initialized_(true), update_count_(0) {
    P_ = Eigen::Matrix<double, 24, 24>::Identity() * 1e-6;
    Q_ = Eigen::Matrix<double, 12, 12>::Identity();
}

void IekfEstimator::initialize(const IekfConfig& config) {
    config_ = config;
    initialized_ = true;
    reset();
}

void IekfEstimator::setInitialState(const State& state) {
    state_ = state;

    // FAST-LIO2 convention: P = Identity, small values for velocity/biases
    P_ = Eigen::Matrix<double, 24, 24>::Identity();
    P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.0001;   // velocity
    P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.0001;   // acc_bias
    P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.0001; // gyro_bias
    P_.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() * 0.0001; // gravity

    initialized_ = true;
}

void IekfEstimator::predict(const ImuData& imu_data, double dt) {
    if (!initialized_ || dt <= 0) return;

    Vector3d acc_unbiased = imu_data.acc - state_.acc_bias;
    Vector3d gyro_unbiased = imu_data.gyro - state_.gyro_bias;
    Matrix3d R = state_.rotation.toRotationMatrix();

    // Mid-point integration: use average of previous and current measurement
    // For the first call, prev is same as current
    if (!last_imu_set_) {
        last_acc_unbiased_ = acc_unbiased;
        last_gyro_unbiased_ = gyro_unbiased;
        last_imu_set_ = true;
    }

    Vector3d gyro_mid = 0.5 * (last_gyro_unbiased_ + gyro_unbiased);
    Vector3d acc_mid = 0.5 * (last_acc_unbiased_ + acc_unbiased);
    last_acc_unbiased_ = acc_unbiased;
    last_gyro_unbiased_ = gyro_unbiased;

    // State propagation
    // 1. Rotation update
    Vector3d delta_angle = gyro_mid * dt;
    if (delta_angle.norm() > 1e-12) {
        Quaterniond delta_q(Eigen::AngleAxisd(delta_angle.norm(),
                                               delta_angle.normalized()));
        state_.rotation = state_.rotation * delta_q;
        state_.rotation.normalize();
    }

    // 2. Position and velocity update (use NEW rotation)
    Matrix3d R_new = state_.rotation.toRotationMatrix();
    Vector3d acc_world = R_new * acc_mid + state_.gravity;
    state_.velocity += acc_world * dt;
    state_.position += state_.velocity * dt + 0.5 * acc_world * dt * dt;

    state_.timestamp += dt;

    // Prediction Jacobian F (24x24)
    Eigen::Matrix<double, 24, 24> F = Eigen::Matrix<double, 24, 24>::Identity();
    F.block<3, 3>(0, 6) = Matrix3d::Identity() * dt;              // pos <- vel
    F.block<3, 3>(3, 12) = -Matrix3d::Identity() * dt;            // rot <- gyro_bias
    F.block<3, 3>(6, 3) = -R * skewSymmetric(acc_unbiased) * dt;  // vel <- rot
    F.block<3, 3>(6, 9) = -R * dt;                                // vel <- acc_bias
    F.block<3, 3>(6, 15) = Matrix3d::Identity() * dt;             // vel <- gravity

    // Noise Jacobian G (24x12)
    Eigen::Matrix<double, 24, 12> G = Eigen::Matrix<double, 24, 12>::Zero();
    G.block<3, 3>(0, 0) = 0.5 * R * dt * dt;                     // pos <- acc noise
    G.block<3, 3>(3, 3) = R * dt;                                 // rot <- gyro noise
    G.block<3, 3>(6, 0) = R * dt;                                 // vel <- acc noise
    G.block<3, 3>(9, 6) = Matrix3d::Identity() * dt;              // acc_bias random walk
    G.block<3, 3>(12, 9) = Matrix3d::Identity() * dt;             // gyro_bias random walk

    // Covariance propagation
    Eigen::Matrix<double, 12, 12> Q = buildProcessNoiseCovariance(dt);
    P_ = F * P_ * F.transpose() + G * Q * G.transpose();
    P_ = (P_ + P_.transpose()) / 2.0;
}

void IekfEstimator::predictBatch(const std::vector<ImuData>& imu_data) {
    if (imu_data.size() < 2) return;

    // Seed previous sample for mid-point integration using current state bias
    last_acc_unbiased_ = imu_data[0].acc - state_.acc_bias;
    last_gyro_unbiased_ = imu_data[0].gyro - state_.gyro_bias;
    last_imu_set_ = true;

    // Integrate consecutive pairs: each predict() uses last_* (prev) + current
    for (size_t i = 1; i < imu_data.size(); ++i) {
        double dt = imu_data[i].timestamp - imu_data[i - 1].timestamp;
        if (dt > 0) {
            predict(imu_data[i], dt);
        }
    }
}

bool IekfEstimator::update(const std::vector<Vector3d>& source_points,
                           const std::vector<Vector3d>& target_points,
                           const std::vector<std::pair<int, int>>& correspondences) {
    if (correspondences.empty() || source_points.empty()) return false;

    int n = correspondences.size();
    if (n < config_.min_valid_points) return false;

    Eigen::VectorXd residual(3 * n);
    Eigen::MatrixXd H(3 * n, 24);
    H.setZero();

    Matrix3d R = state_.rotation.toRotationMatrix();
    Vector3d t = state_.position;

    for (int i = 0; i < n; ++i) {
        int src_idx = correspondences[i].first;
        int tgt_idx = correspondences[i].second;

        if (src_idx >= static_cast<int>(source_points.size()) ||
            tgt_idx >= static_cast<int>(target_points.size())) {
            continue;
        }

        Vector3d p_src = source_points[src_idx];
        Vector3d p_tgt = target_points[tgt_idx];

        Vector3d p_transformed = R * p_src + t;
        residual.block<3, 1>(3 * i, 0) = p_transformed - p_tgt;

        H.block<3, 3>(3 * i, 0) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(3 * i, 3) = -R * skewSymmetric(p_src);
    }

    Eigen::MatrixXd R_mat = Eigen::MatrixXd::Identity(3 * n, 3 * n) *
                             config_.point_noise * config_.point_noise;

    Eigen::MatrixXd S = H * P_ * H.transpose() + R_mat;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    Eigen::Matrix<double, 24, 1> dx = K * residual;

    state_.position += dx.block<3, 1>(0, 0);

    Vector3d delta_rot = dx.block<3, 1>(3, 0);
    if (delta_rot.norm() > 1e-10) {
        Quaterniond delta_q(Eigen::AngleAxisd(delta_rot.norm(),
                                              delta_rot.normalized()));
        state_.rotation = state_.rotation * delta_q;
        state_.rotation.normalize();
    }
    state_.rotation.normalize();

    state_.velocity += dx.block<3, 1>(6, 0);
    state_.acc_bias += dx.block<3, 1>(9, 0);
    state_.gyro_bias += dx.block<3, 1>(12, 0);

    Eigen::Matrix<double, 24, 24> I_KH =
        Eigen::Matrix<double, 24, 24>::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_mat * K.transpose();
    P_ = (P_ + P_.transpose()) / 2.0;

    update_count_++;
    return true;
}

bool IekfEstimator::updateWithNormals(
    const std::vector<Vector3d>& source_points,
    const std::vector<Vector3d>& plane_normals,
    const std::vector<double>& plane_dists,
    const std::vector<int>& /* valid_indices */) {

    int n = source_points.size();
    if (n < config_.min_valid_points) return false;

    Matrix3d R = state_.rotation.toRotationMatrix();
    Vector3d t = state_.position;

    // FAST-LIO2 信息滤波形式: 累积 H^T*σ^{-2}*H (24×24) 和 H^T*σ^{-2}*r (24×1)
    // 避免 N×N 矩阵求逆，加入 P^{-1} 先验项防止发散
    Eigen::Matrix<double, 24, 24> H_accum = Eigen::Matrix<double, 24, 24>::Zero();
    Eigen::Matrix<double, 24, 1> b_accum = Eigen::Matrix<double, 24, 1>::Zero();

    double sigma_inv2 = config_.point_noise > 0 ?
        1.0 / (config_.point_noise * config_.point_noise) : 1000.0;

    for (int i = 0; i < n; ++i) {
        const Vector3d& p_src = source_points[i];
        const Vector3d& normal = plane_normals[i];
        double dist = plane_dists[i];

        Vector3d p_transformed = R * p_src + t;
        // pd2 已包含 normal·p_world + d, 直接作为残差
        double residual = dist;

        // 雅可比 (1×24, 只有前6列非零)
        Eigen::Matrix<double, 1, 24> J = Eigen::Matrix<double, 1, 24>::Zero();
        J.block<1, 3>(0, 0) = normal.transpose();
        J.block<1, 3>(0, 3) = -normal.transpose() * R * skewSymmetric(p_src);

        H_accum += J.transpose() * J * sigma_inv2;
        b_accum += J.transpose() * residual * sigma_inv2;
    }

    // MAP 求解: δx = -(P^{-1} + H^T R^{-1} H)^{-1} * (H^T R^{-1} r)
    Eigen::Matrix<double, 24, 24> P_inv = P_.inverse();
    Eigen::Matrix<double, 24, 24> A = P_inv + H_accum;
    Eigen::Matrix<double, 24, 1> dx = -A.householderQr().solve(b_accum);

    // 状态更新
    state_.position += dx.block<3, 1>(0, 0);

    Vector3d rot_dx = dx.block<3, 1>(3, 0);
    if (rot_dx.norm() > 1e-10) {
        Quaterniond delta_q(Eigen::AngleAxisd(rot_dx.norm(), rot_dx.normalized()));
        state_.rotation = state_.rotation * delta_q;
        state_.rotation.normalize();
    }

    state_.velocity += dx.block<3, 1>(6, 0);
    state_.acc_bias += dx.block<3, 1>(9, 0);
    state_.gyro_bias += dx.block<3, 1>(12, 0);

    // 协方差更新: P = A^{-1}
    P_ = A.inverse();
    P_ = (P_ + P_.transpose()) / 2.0;

    update_count_++;
    return true;
}

void IekfEstimator::reset() {
    state_ = State();
    P_ = Eigen::Matrix<double, 24, 24>::Identity() * 1e-6;
    update_count_ = 0;
}

Eigen::Matrix<double, 12, 12> IekfEstimator::buildProcessNoiseCovariance(double dt) {
    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();
    Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() *
                          config_.acc_noise * config_.acc_noise * dt;
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() *
                          config_.gyro_noise * config_.gyro_noise * dt;
    Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() *
                          config_.acc_bias_noise * config_.acc_bias_noise * dt;
    Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() *
                           config_.gyro_bias_noise * config_.gyro_bias_noise * dt;
    return Q;
}

Eigen::Matrix<double, 3, 3> IekfEstimator::buildMeasurementNoiseCovariance() {
    return Eigen::Matrix3d::Identity() * config_.point_noise * config_.point_noise;
}

double IekfEstimator::computePointToPlaneResidual(
    const Vector3d& source_point,
    const Vector3d& target_point,
    const Vector3d& target_normal) {
    Vector3d p_transformed = state_.rotation.toRotationMatrix() * source_point + state_.position;
    return target_normal.dot(p_transformed - target_point);
}

} // namespace fast_lio2_slam
