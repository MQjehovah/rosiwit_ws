/**
 * @file imu_processor.h
 * @brief FAST-LIO2 SLAM - IMU数据处理
 * @author AI Development Team
 * @date 2026-04-24
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "fast_lio2_slam/common/sophus_se3.hpp"
#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/common/config.h"
#include <deque>
#include <mutex>

namespace fast_lio2_slam {

/**
 * @brief IMU处理器配置
 */
struct ImuProcessorConfig {
    double acc_noise = 0.1;           // 加速度计噪声 (m/s^2)
    double gyro_noise = 0.01;          // 陀螺仪噪声 (rad/s)
    double acc_bias_noise = 0.0001;   // 加速度计偏置噪声
    double gyro_bias_noise = 0.00001; // 陀螺仪偏置噪声
    double gravity = 9.81;            // 重力加速度

    // 初始偏置估计参数
    int static_init_count = 200;      // 静止初始化帧数
    bool estimate_initial_bias = true; // 是否估计初始偏置
};

/**
 * @brief IMU预积分结果
 */
struct ImuPreintegrationResult {
    double t_start;
    double t_end;
    SE3d delta_pose;              // 相对位姿变化
    Vector3d delta_velocity;       // 速度变化
    Vector3d delta_acc_bias;       // 加速度计偏置变化
    Vector3d delta_gyro_bias;      // 陀螺仪偏置变化

    // 协方差
    Eigen::Matrix<double, 9, 9> covariance;
};

/**
 * @brief IMU数据处理与预积分类
 *
 * 实现IMU数据的:
 * 1. 缓存管理
 * 2. 状态预测 (IEKF预测步)
 * 3. 运动畸变校正
 */
class ImuProcessor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcessor();
    explicit ImuProcessor(const ImuProcessorConfig& config);
    ~ImuProcessor() = default;

    /**
     * @brief 初始化处理器
     */
    void initialize(const ImuProcessorConfig& config);

    /**
     * @brief 设置初始状态
     */
    void setInitialState(const State& state);

    /**
     * @brief 设置偏置
     */
    void setBias(const Vector3d& acc_bias, const Vector3d& gyro_bias);

    /**
     * @brief 添加IMU数据
     */
    void addImuData(const ImuData& imu);

    /**
     * @brief 获取IMU缓冲区
     */
    std::vector<ImuData> getImuBuffer() const;

    /**
     * @brief 清空缓冲区
     */
    void clearBuffer();

    /**
     * @brief 获取缓冲区大小
     */
    size_t bufferSize() const;

    /**
     * @brief IMU状态预测 (IEKF预测步)
     */
    void predict(State& state, double t_end);

    /**
     * @brief 使用IMU数据传播状态
     */
    State propagate(const State& state, const ImuData& imu, double dt);

    /**
     * @brief 获取指定时间范围内的IMU数据
     */
    std::vector<ImuData> getImuInRange(double t_start, double t_end);

    /**
     * @brief 静止初始化估计偏置
     */
    bool staticInitializeBias(int required_count = -1);

    /**
     * @brief 检查是否有足够数据进行初始化
     */
    bool hasEnoughDataForInit() const;

    /**
     * @brief 获取最新IMU数据
     */
    bool getLatestImu(ImuData& imu) const;

    /**
     * @brief 时间戳插值获取IMU数据
     */
    bool getImuAtTime(double timestamp, ImuData& imu);

    // 静止初始化结果
    Quaterniond getInitRotation() const { return init_rotation_; }
    Vector3d getGyroBiasEst() const { return gyro_bias_est_; }

private:
    ImuProcessorConfig config_;
    std::deque<ImuData> imu_buffer_;
    mutable std::mutex buffer_mutex_;
    bool initialized_;

    // 静止初始化相关
    Vector3d acc_sum_;
    Vector3d gyro_sum_;
    int init_count_;
    bool bias_initialized_;

    // 静止初始化结果
    Vector3d gyro_bias_est_ = Vector3d::Zero();
    Quaterniond init_rotation_ = Quaterniond::Identity();
};

} // namespace fast_lio2_slam
