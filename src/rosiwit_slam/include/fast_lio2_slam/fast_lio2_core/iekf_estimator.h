/**
 * @file iekf_estimator.h
 * @brief FAST-LIO2 SLAM - 迭代扩展卡尔曼滤波(IEKF)状态估计器
 * @author AI Development Team
 * @date 2026-04-24
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "fast_lio2_slam/common/sophus_se3.hpp"
#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/common/config.h"
#include <vector>

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
 * @brief 将旋转投影到纯 yaw (锁定 roll/pitch)
 */
inline Quaterniond clampToPlanarYaw(const Quaterniond& q) {
    double sin_yaw = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cos_yaw = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    double yaw = std::atan2(sin_yaw, cos_yaw);
    return Quaterniond(Eigen::AngleAxisd(yaw, Vector3d::UnitZ()));
}

/**
 * @brief IEKF估计器配置
 */
struct IekfConfig {
    int max_iterations = 5;
    double converge_threshold = 1e-4;

    double point_noise = 0.02;
    double position_noise = 0.01;
    double rotation_noise = 0.005;

    double acc_noise = 0.1;
    double gyro_noise = 0.01;
    double acc_bias_noise = 0.0001;
    double gyro_bias_noise = 0.00001;
    double gravity_magnitude = 9.81;
    bool use_acc_integration = true;

    double max_correspondence_dist = 1.0;
    int min_valid_points = 100;

    Vector3d ext_translation = Vector3d::Zero();
    Vector3d ext_rotation_euler = Vector3d::Zero();
};

/**
 * @brief 迭代扩展卡尔曼滤波(IEKF)状态估计器
 */
class IekfEstimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IekfEstimator();
    explicit IekfEstimator(const IekfConfig& config);
    ~IekfEstimator() = default;

    void initialize(const IekfConfig& config);
    void setInitialState(const State& state);
    void setConfig(const IekfConfig& config) { config_ = config; }

    State getState() const { return state_; }
    void setState(const State& state) { state_ = state; }

    Eigen::Matrix<double, 24, 24> getCovariance() const { return P_; }

    void predict(const ImuData& imu_data, double dt);
    void predictBatch(const std::vector<ImuData>& imu_data);

    bool update(const std::vector<Vector3d>& source_points,
                const std::vector<Vector3d>& target_points,
                const std::vector<std::pair<int, int>>& correspondences);

    bool updateWithNormals(const std::vector<Vector3d>& source_points,
                           const std::vector<Vector3d>& target_points,
                           const std::vector<Vector3d>& target_normals,
                           const std::vector<int>& valid_indices);

    void reset();

    bool isInitialized() const { return initialized_; }

    void computeResidualAndJacobian(const std::vector<Vector3d>& source_points,
                                     const std::vector<Vector3d>& target_points,
                                     const std::vector<Vector3d>& target_normals,
                                     const std::vector<int>& valid_indices,
                                     Eigen::VectorXd& residual,
                                     Eigen::MatrixXd& H);

private:
    Eigen::Matrix<double, 12, 12> buildProcessNoiseCovariance(double dt);
    Eigen::Matrix<double, 3, 3> buildMeasurementNoiseCovariance();
    void updateCovariance(const Eigen::MatrixXd& H,
                           const Eigen::MatrixXd& R,
                           const Eigen::VectorXd& residual);
    double computePointToPlaneResidual(const Vector3d& source_point,
                                        const Vector3d& target_point,
                                        const Vector3d& target_normal);

private:
    IekfConfig config_;
    State state_;
    Eigen::Matrix<double, 24, 24> P_;
    Eigen::Matrix<double, 12, 12> Q_;

    bool initialized_;
    int update_count_;
};

} // namespace fast_lio2_slam
