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

    P_ = Eigen::Matrix<double, 24, 24>::Identity() * 0.001;
    P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.001;
    P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.001;
    P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.01;
    P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.0001;
    P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.0001;
    P_.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() * 0.0001;

    initialized_ = true;
}

void IekfEstimator::predict(const ImuData& imu_data, double dt) {
    if (!initialized_ || dt <= 0) return;

    Vector3d acc = imu_data.acc - state_.acc_bias;
    Vector3d gyro = imu_data.gyro - state_.gyro_bias;

    Matrix3d R = state_.rotation.toRotationMatrix();

    Eigen::Matrix<double, 24, 24> F = Eigen::Matrix<double, 24, 24>::Identity();
    F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(3, 12) = -R * dt;
    F.block<3, 3>(6, 3) = -skewSymmetric(R * acc) * dt;
    F.block<3, 3>(6, 9) = -R * dt;
    F.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity() * dt;

    // 姿态更新
    Vector3d delta_angle = gyro * dt;
    if (delta_angle.norm() > 1e-12) {
        Quaterniond delta_q(Eigen::AngleAxisd(delta_angle.norm(),
                                               delta_angle.normalized()));
        state_.rotation = state_.rotation * delta_q;
        state_.rotation.normalize();
    }
    state_.rotation.normalize();

    if (config_.use_acc_integration) {
        Vector3d acc_world = R * acc + state_.gravity;
        state_.velocity += acc_world * dt;
        state_.position += state_.velocity * dt + 0.5 * acc_world * dt * dt;
    }

    state_.timestamp += dt;

    Eigen::Matrix<double, 12, 12> Q = buildProcessNoiseCovariance(dt);

    Eigen::Matrix<double, 24, 12> G = Eigen::Matrix<double, 24, 12>::Zero();
    G.block<3, 3>(0, 0) = 0.5 * R * dt * dt;
    G.block<3, 3>(3, 3) = R * dt;
    G.block<3, 3>(6, 0) = R * dt;

    P_ = F * P_ * F.transpose() + G * Q * G.transpose();
    P_ = (P_ + P_.transpose()) / 2.0;
}

void IekfEstimator::predictBatch(const std::vector<ImuData>& imu_data) {
    if (imu_data.empty()) return;
    for (size_t i = 0; i < imu_data.size() - 1; ++i) {
        double dt = imu_data[i + 1].timestamp - imu_data[i].timestamp;
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

    double g_norm = state_.gravity.norm();
    if (g_norm > 1e-6) {
        state_.gravity = state_.gravity / g_norm * config_.gravity_magnitude;
    }

    Eigen::Matrix<double, 24, 24> I_KH =
        Eigen::Matrix<double, 24, 24>::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_mat * K.transpose();
    P_ = (P_ + P_.transpose()) / 2.0;

    update_count_++;
    return true;
}

bool IekfEstimator::updateWithNormals(
    const std::vector<Vector3d>& source_points,
    const std::vector<Vector3d>& target_points,
    const std::vector<Vector3d>& target_normals,
    const std::vector<int>& /* valid_indices */) {

    int n = source_points.size();
    if (n < config_.min_valid_points) return false;

    const int max_obs_points = 500;
    std::vector<Vector3d> src_sub, tgt_sub, nrm_sub;
    if (n > max_obs_points) {
        double step = static_cast<double>(n) / max_obs_points;
        for (int j = 0; j < max_obs_points; ++j) {
            int idx = static_cast<int>(j * step);
            src_sub.push_back(source_points[idx]);
            tgt_sub.push_back(target_points[idx]);
            nrm_sub.push_back(target_normals[idx]);
        }
        n = max_obs_points;
    } else {
        src_sub = source_points;
        tgt_sub = target_points;
        nrm_sub = target_normals;
    }

    Eigen::VectorXd residual(n);
    Eigen::MatrixXd H(n, 24);
    H.setZero();

    Matrix3d R = state_.rotation.toRotationMatrix();
    Vector3d t = state_.position;

    for (int i = 0; i < n; ++i) {
        Vector3d p_src = src_sub[i];
        Vector3d p_tgt = tgt_sub[i];
        Vector3d normal = nrm_sub[i];

        Vector3d p_transformed = R * p_src + t;
        residual(i) = normal.dot(p_transformed - p_tgt);

        H.block<1, 3>(i, 0) = normal.transpose();
        H.block<1, 3>(i, 3) = -normal.transpose() * R * skewSymmetric(p_src);
    }

    double scaled_noise = config_.point_noise * std::sqrt(static_cast<double>(n));
    Eigen::MatrixXd R_mat = Eigen::MatrixXd::Identity(n, n) *
                            scaled_noise * scaled_noise;

    Eigen::MatrixXd S = H * P_ * H.transpose() + R_mat;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    Eigen::Matrix<double, 24, 1> dx = K * residual;

    Vector3d pos_dx = dx.block<3, 1>(0, 0);
    if (pos_dx.norm() > 0.5) pos_dx = pos_dx.normalized() * 0.5;
    Vector3d rot_dx = dx.block<3, 1>(3, 0);
    if (rot_dx.norm() > 0.1) rot_dx = rot_dx.normalized() * 0.1;
    Vector3d vel_dx = dx.block<3, 1>(6, 0);
    if (vel_dx.norm() > 1.0) vel_dx = vel_dx.normalized() * 1.0;

    state_.position += pos_dx;

    if (rot_dx.norm() > 1e-10) {
        Quaterniond delta_q(Eigen::AngleAxisd(rot_dx.norm(),
                                              rot_dx.normalized()));
        state_.rotation = state_.rotation * delta_q;
        state_.rotation.normalize();
    }
    state_.rotation.normalize();

    state_.velocity += vel_dx;
    state_.acc_bias += dx.block<3, 1>(9, 0);
    state_.gyro_bias += dx.block<3, 1>(12, 0);

    Eigen::Matrix<double, 24, 24> I_KH =
        Eigen::Matrix<double, 24, 24>::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_mat * K.transpose();
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
