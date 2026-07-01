/**
 * @file iekf_estimator.h
 * @brief FAST-LIO2 SLAM - 迭代扩展卡尔曼滤波(IEKF)状态估计器
 * @author AI Development Team
 * @date 2026-04-24
 *
 * 实现FAST-LIO2的核心IEKF算法，包含:
 * 1. IMU预测步 (状态传播)
 * 2. LiDAR更新步 (点云配准)
 * 3. 状态估计与协方差更新
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "fast_lio2_slam/common/sophus_se3.hpp"
#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/common/config.h"
#include <vector>
#include <rclcpp/logging.hpp>

namespace fast_lio2_slam {

using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using MatrixXd = Eigen::MatrixXd;
using SE3d = Sophus::SE3<double>;
using SO3d = Sophus::SO3<double>;

// 反对称矩阵声明 (定义在utils.h中)
inline Matrix3d skewSymmetric(const Vector3d& v);

/**
 * @brief IEKF估计器配置
 */
struct IekfConfig {
    // 迭代参数
    int max_iterations = 5;           // 最大迭代次数
    double converge_threshold = 1e-4; // 收敛阈值

    // 测量噪声
    double point_noise = 0.02;        // 点云测量噪声 (m)
    double position_noise = 0.01;     // 位置噪声
    double rotation_noise = 0.005;    // 旋转噪声 (rad)

    // IMU噪声参数
    double acc_noise = 0.1;           // 加速度计噪声
    double gyro_noise = 0.01;         // 陀螺仪噪声
    double acc_bias_noise = 0.0001;   // 加速度计偏置噪声
    double gyro_bias_noise = 0.00001; // 陀螺仪偏置噪声
    double gravity_magnitude = 9.81;  // 重力大小 (用于在线重力方向估计时归一化)
    bool use_acc_integration = true;  // 是否积分加速度 (false=激光主导模式)

    // 地图匹配参数
    double max_correspondence_dist = 1.0; // 最大对应点距离
    int min_valid_points = 100;       // 最小有效点数

    // 外参 (LiDAR到IMU)
    Vector3d ext_translation = Vector3d::Zero();
    Vector3d ext_rotation_euler = Vector3d::Zero();
};

/**
 * @brief 迭代扩展卡尔曼滤波(IEKF)状态估计器
 *
 * 实现FAST-LIO2的IEKF算法:
 * - 状态向量: 位置(3) + 姿态(3) + 速度(3) + acc_bias(3) + gyro_bias(3) +
 *             gravity(3) + ext_R(3) + ext_T(3) = 24维
 * - 预测步: IMU积分传播状态和协方差
 * - 更新步: 点云配准残差更新状态
 */
class IekfEstimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IekfEstimator();
    explicit IekfEstimator(const IekfConfig& config);
    ~IekfEstimator() = default;

    /**
     * @brief 初始化估计器
     */
    void initialize(const IekfConfig& config);

    /**
     * @brief 设置初始状态
     */
    void setInitialState(const State& state);

    /**
     * @brief 设置配置（不改变initialized_状态）
     */
    void setConfig(const IekfConfig& config) { config_ = config; }

    /**
     * @brief 获取当前状态
     */
    State getState() const { return state_; }

    /**
     * @brief 设置状态
     */
    void setState(const State& state) { state_ = state; }

    /**
     * @brief 获取协方差矩阵
     */
    Eigen::Matrix<double, 24, 24> getCovariance() const { return P_; }

    /**
     * @brief IMU预测步
     *
     * 使用IMU数据进行状态传播
     */
    void predict(const ImuData& imu_data, double dt);

    /**
     * @brief 批量IMU预测
     */
    void predictBatch(const std::vector<ImuData>& imu_data);

    /**
     * @brief LiDAR更新步
     *
     * 使用点云配准残差更新状态
     * @param source_points 当前帧点云
     * @param target_points 目标点云(地图中的对应点)
     * @param correspondences 对应点索引对
     * @return 是否更新成功
     */
    bool update(const std::vector<Vector3d>& source_points,
                const std::vector<Vector3d>& target_points,
                const std::vector<std::pair<int, int>>& correspondences);

    /**
     * @brief 使用法向量进行更新 (点到平面)
     */
    bool updateWithNormals(const std::vector<Vector3d>& source_points,
                           const std::vector<Vector3d>& target_points,
                           const std::vector<Vector3d>& target_normals,
                           const std::vector<int>& valid_indices);

    /**
     * @brief 状态重置
     */
    void reset();

    /**
     * @brief 检查是否已初始化
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief 计算残差和雅可比
     */
    void computeResidualAndJacobian(const std::vector<Vector3d>& source_points,
                                     const std::vector<Vector3d>& target_points,
                                     const std::vector<Vector3d>& target_normals,
                                     const std::vector<int>& valid_indices,
                                     Eigen::VectorXd& residual,
                                     Eigen::MatrixXd& H);

private:
    /**
     * @brief 构建过程噪声协方差矩阵
     */
    Eigen::Matrix<double, 12, 12> buildProcessNoiseCovariance(double dt);

    /**
     * @brief 构建测量噪声协方差矩阵
     */
    Eigen::Matrix<double, 3, 3> buildMeasurementNoiseCovariance();

    /**
     * @brief 更新协方差矩阵
     */
    void updateCovariance(const Eigen::MatrixXd& H,
                           const Eigen::MatrixXd& R,
                           const Eigen::VectorXd& residual);

    /**
     * @brief 计算点到平面残差
     */
    double computePointToPlaneResidual(const Vector3d& source_point,
                                        const Vector3d& target_point,
                                        const Vector3d& target_normal);

private:
    IekfConfig config_;
    State state_;                                    // 当前状态
    Eigen::Matrix<double, 24, 24> P_;               // 状态协方差
    Eigen::Matrix<double, 12, 12> Q_;               // 过程噪声协方差

    bool initialized_;
    int update_count_;
};

// ==================== 实现部分 ====================

inline IekfEstimator::IekfEstimator()
    : initialized_(false), update_count_(0) {
    P_ = Eigen::Matrix<double, 24, 24>::Identity() * 1e-6;
    Q_ = Eigen::Matrix<double, 12, 12>::Identity();
}

inline IekfEstimator::IekfEstimator(const IekfConfig& config)
    : config_(config), initialized_(true), update_count_(0) {
    P_ = Eigen::Matrix<double, 24, 24>::Identity() * 1e-6;
    Q_ = Eigen::Matrix<double, 12, 12>::Identity();
}

inline void IekfEstimator::initialize(const IekfConfig& config) {
    config_ = config;
    initialized_ = true;
    reset();
}

inline void IekfEstimator::setInitialState(const State& state) {
    state_ = state;

    // 初始化协方差矩阵 - 增大初始不确定性
    P_ = Eigen::Matrix<double, 24, 24>::Identity() * 0.01;

    // 位置协方差 - 初始位置不确定性较大（1米标准差）
    P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1.0;
    // 姿态协方差 - 初始姿态不确定性
    P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.1;
    // 速度协方差
    P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1.0;
    // 偏置协方差
    P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.01;
    P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.001;
    // 重力协方差 - 允许在线估计重力方向
    P_.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() * 1.0;

    initialized_ = true;
}

inline void IekfEstimator::predict(const ImuData& imu_data, double dt) {
    if (!initialized_ || dt <= 0) return;

    // 去除偏置
    Vector3d acc = imu_data.acc - state_.acc_bias;
    Vector3d gyro = imu_data.gyro - state_.gyro_bias;

    // 旋转矩阵
    Matrix3d R = state_.rotation.toRotationMatrix();
    Matrix3d R_inv = R.transpose();

    // 构建状态转移矩阵 F (24x24)
    Eigen::Matrix<double, 24, 24> F = Eigen::Matrix<double, 24, 24>::Identity();

    // 状态传播方程 (离散时间):
    // p = p + v*dt + 0.5*a*dt^2
    // R = R * exp(omega*dt)
    // v = v + a*dt

    // 位置对速度的雅可比
    F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;

    // 姿态对角速度的雅可比
    F.block<3, 3>(3, 12) = -R * dt;  // 对gyro_bias

    // 速度对姿态的雅可比
    F.block<3, 3>(6, 3) = -skewSymmetric(R * acc) * dt;

    // 速度对加速度偏置的雅可比
    F.block<3, 3>(6, 9) = -R * dt;

    // 速度对重力的雅可比
    F.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity() * dt;

    // ===== 状态更新 =====

    // 1. 姿态更新 (始终用陀螺仪积分)
    Vector3d delta_angle = gyro * dt;
    Quaterniond delta_q(Eigen::AngleAxisd(delta_angle.norm(),
                                           delta_angle.normalized()));
    state_.rotation = state_.rotation * delta_q;
    state_.rotation.normalize();

    // 2. 速度/位置更新 (激光主导模式下跳过，平移由激光匹配修正)
    if (config_.use_acc_integration) {
        Vector3d acc_world = R * acc + state_.gravity;
        state_.velocity += acc_world * dt;
        state_.position += state_.velocity * dt + 0.5 * acc_world * dt * dt;
    }

    // 3. 更新时间戳
    state_.timestamp += dt;

    // ===== 协方差更新 =====
    Eigen::Matrix<double, 12, 12> Q = buildProcessNoiseCovariance(dt);

    // 过程噪声输入矩阵 G (24x12)
    Eigen::Matrix<double, 24, 12> G = Eigen::Matrix<double, 24, 12>::Zero();

    // 位置对加速度噪声
    G.block<3, 3>(0, 0) = 0.5 * R * dt * dt;
    // 姿态对角速度噪声
    G.block<3, 3>(3, 3) = R * dt;
    // 速度对加速度噪声
    G.block<3, 3>(6, 0) = R * dt;

    // 协方差传播
    P_ = F * P_ * F.transpose() + G * Q * G.transpose();

    // 确保协方差矩阵对称
    P_ = (P_ + P_.transpose()) / 2.0;
}

inline void IekfEstimator::predictBatch(const std::vector<ImuData>& imu_data) {
    if (imu_data.empty()) return;

    for (size_t i = 0; i < imu_data.size() - 1; ++i) {
        double dt = imu_data[i + 1].timestamp - imu_data[i].timestamp;
        if (dt > 0) {
            predict(imu_data[i], dt);
        }
    }
}

inline bool IekfEstimator::update(const std::vector<Vector3d>& source_points,
                                   const std::vector<Vector3d>& target_points,
                                   const std::vector<std::pair<int, int>>& correspondences) {
    if (correspondences.empty() || source_points.empty()) {
        return false;
    }

    int n = correspondences.size();
    if (n < config_.min_valid_points) {
        return false;
    }

    // 构建测量残差和雅可比
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

        // 变换后的源点
        Vector3d p_transformed = R * p_src + t;

        // 残差 = 变换后的点 - 目标点
        residual.block<3, 1>(3 * i, 0) = p_transformed - p_tgt;

        // 雅可比矩阵 (对位置和姿态)
        // dr/dp = I
        H.block<3, 3>(3 * i, 0) = Eigen::Matrix3d::Identity();

        // dr/dR = -R * [p_src]^
        H.block<3, 3>(3 * i, 3) = -R * skewSymmetric(p_src);
    }

    // 测量噪声协方差
    Eigen::MatrixXd R_mat = Eigen::MatrixXd::Identity(3 * n, 3 * n) *
                             config_.point_noise * config_.point_noise;

    // 卡尔曼增益
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_mat;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // 状态更新
    Eigen::Matrix<double, 24, 1> dx = K * residual;

    // 更新位置
    state_.position += dx.block<3, 1>(0, 0);

    // 更新姿态
    Vector3d delta_rot = dx.block<3, 1>(3, 0);
    Quaterniond delta_q(Eigen::AngleAxisd(delta_rot.norm(),
                                          delta_rot.normalized()));
    state_.rotation = state_.rotation * delta_q;
    state_.rotation.normalize();

    // 更新速度、偏置、重力
    state_.velocity += dx.block<3, 1>(6, 0);
    state_.acc_bias += dx.block<3, 1>(9, 0);
    state_.gyro_bias += dx.block<3, 1>(12, 0);
    state_.gravity += dx.block<3, 1>(15, 0);
    double g_norm = state_.gravity.norm();
    if (g_norm > 1e-6) {
        state_.gravity = state_.gravity / g_norm * config_.gravity_magnitude;
    }

    // 更新协方差
    Eigen::Matrix<double, 24, 24> I_KH =
        Eigen::Matrix<double, 24, 24>::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_mat * K.transpose();

    // 确保对称
    P_ = (P_ + P_.transpose()) / 2.0;

    update_count_++;
    return true;
}

inline bool IekfEstimator::updateWithNormals(
    const std::vector<Vector3d>& source_points,
    const std::vector<Vector3d>& target_points,
    const std::vector<Vector3d>& target_normals,
    const std::vector<int>& /* valid_indices */) {

    int n = source_points.size();
    if (n < config_.min_valid_points) {
        return false;
    }

    // 子采样：限制观测点数量，防止协方差过度缩减
    const int max_obs_points = 200;
    std::vector<Vector3d> src_sub, tgt_sub, nrm_sub;
    if (n > max_obs_points) {
        // 均匀子采样
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

    // 构建点到平面残差
    Eigen::VectorXd residual(n);
    Eigen::MatrixXd H(n, 24);
    H.setZero();

    Matrix3d R = state_.rotation.toRotationMatrix();
    Vector3d t = state_.position;

    for (int i = 0; i < n; ++i) {
        Vector3d p_src = src_sub[i];
        Vector3d p_tgt = tgt_sub[i];
        Vector3d normal = nrm_sub[i];

        // 变换后的源点
        Vector3d p_transformed = R * p_src + t;

        // 点到平面残差
        residual(i) = normal.dot(p_transformed - p_tgt);

        // 雅可比矩阵
        // dr/dp = n^T
        H.block<1, 3>(i, 0) = normal.transpose();

        // dr/dR = n^T * (-R * [p_src]^)
        H.block<1, 3>(i, 3) = -normal.transpose() * R * skewSymmetric(p_src);
    }

    // 测量噪声
    // 使用信息矩阵缩放：将所有观测等价为单个"虚拟观测"
    // 每个观测的 noise^2 乘以观测数量 n，使得总体信息量合理
    double scaled_noise = config_.point_noise * std::sqrt(static_cast<double>(n));
    Eigen::MatrixXd R_mat = Eigen::MatrixXd::Identity(n, n) *
                            scaled_noise * scaled_noise;

    // 卡尔曼增益
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_mat;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // 状态更新
    Eigen::Matrix<double, 24, 1> dx = K * residual;

    // 更新位置
    state_.position += dx.block<3, 1>(0, 0);

    // 更新姿态
    Vector3d delta_rot = dx.block<3, 1>(3, 0);
    if (delta_rot.norm() > 1e-10) {
        Quaterniond delta_q(Eigen::AngleAxisd(delta_rot.norm(),
                                              delta_rot.normalized()));
        state_.rotation = state_.rotation * delta_q;
        state_.rotation.normalize();
    }

    // 更新速度等状态 (如果估计)
    state_.velocity += dx.block<3, 1>(6, 0);

    // 更新偏置
    state_.acc_bias += dx.block<3, 1>(9, 0);
    state_.gyro_bias += dx.block<3, 1>(12, 0);

    // 更新重力方向并归一化 (保持重力大小不变，仅估计方向)
    state_.gravity += dx.block<3, 1>(15, 0);
    double g_norm = state_.gravity.norm();
    if (g_norm > 1e-6) {
        state_.gravity = state_.gravity / g_norm * config_.gravity_magnitude;
    }

    // 更新协方差
    Eigen::Matrix<double, 24, 24> I_KH =
        Eigen::Matrix<double, 24, 24>::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_mat * K.transpose();

    P_ = (P_ + P_.transpose()) / 2.0;

    update_count_++;
    return true;
}

inline void IekfEstimator::reset() {
    state_ = State();
    P_ = Eigen::Matrix<double, 24, 24>::Identity() * 1e-6;
    update_count_ = 0;
}

inline Eigen::Matrix<double, 12, 12> IekfEstimator::buildProcessNoiseCovariance(double dt) {
    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();

    // 加速度噪声 - 增大以确保足够的协方差增长
    Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() *
                          config_.acc_noise * config_.acc_noise * dt;
    // 角速度噪声
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() *
                          config_.gyro_noise * config_.gyro_noise * dt;
    // 加速度偏置噪声
    Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() *
                          config_.acc_bias_noise * config_.acc_bias_noise * dt;
    // 角速度偏置噪声
    Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() *
                           config_.gyro_bias_noise * config_.gyro_bias_noise * dt;

    return Q;
}

inline Eigen::Matrix<double, 3, 3> IekfEstimator::buildMeasurementNoiseCovariance() {
    return Eigen::Matrix3d::Identity() * config_.point_noise * config_.point_noise;
}

inline double IekfEstimator::computePointToPlaneResidual(
    const Vector3d& source_point,
    const Vector3d& target_point,
    const Vector3d& target_normal) {

    Vector3d p_transformed = state_.rotation.toRotationMatrix() * source_point + state_.position;
    return target_normal.dot(p_transformed - target_point);
}

} // namespace fast_lio2_slam