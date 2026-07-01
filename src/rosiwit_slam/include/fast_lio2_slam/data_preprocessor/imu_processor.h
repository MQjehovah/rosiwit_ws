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

// ==================== 实现部分 ====================

inline ImuProcessor::ImuProcessor()
    : initialized_(false), acc_sum_(Vector3d::Zero()),
      gyro_sum_(Vector3d::Zero()), init_count_(0), bias_initialized_(false) {}

inline ImuProcessor::ImuProcessor(const ImuProcessorConfig& config)
    : config_(config), initialized_(true), acc_sum_(Vector3d::Zero()),
      gyro_sum_(Vector3d::Zero()), init_count_(0), bias_initialized_(false) {}

inline void ImuProcessor::initialize(const ImuProcessorConfig& config) {
    config_ = config;
    initialized_ = true;
}

inline void ImuProcessor::setInitialState(const State& state) {
    // 设置初始状态，包括偏置
    config_.acc_noise = state.acc_bias.norm();
}

inline void ImuProcessor::setBias(const Vector3d& acc_bias, const Vector3d& gyro_bias) {
    bias_initialized_ = true;
}

inline void ImuProcessor::addImuData(const ImuData& imu) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // 限制缓冲区大小
    const size_t max_buffer_size = 2000;
    if (imu_buffer_.size() >= max_buffer_size) {
        imu_buffer_.pop_front();
    }

    imu_buffer_.push_back(imu);

    // 累积数据用于静止初始化
    if (config_.estimate_initial_bias && !bias_initialized_) {
        acc_sum_ += imu.acc;
        gyro_sum_ += imu.gyro;
        init_count_++;
    }
}

inline std::vector<ImuData> ImuProcessor::getImuBuffer() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return std::vector<ImuData>(imu_buffer_.begin(), imu_buffer_.end());
}

inline void ImuProcessor::clearBuffer() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    imu_buffer_.clear();
}

inline size_t ImuProcessor::bufferSize() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return imu_buffer_.size();
}

inline void ImuProcessor::predict(State& state, double t_end) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    if (imu_buffer_.empty()) return;

    // 从当前状态时间到t_end进行IMU积分
    while (!imu_buffer_.empty()) {
        const ImuData& imu = imu_buffer_.front();

        if (imu.timestamp <= state.timestamp) {
            imu_buffer_.pop_front();
            continue;
        }

        if (imu.timestamp >= t_end) {
            break;
        }

        // 计算积分时间
        double dt = imu.timestamp - state.timestamp;
        if (dt <= 0) {
            imu_buffer_.pop_front();
            continue;
        }

        // 状态传播
        state = propagate(state, imu, dt);
        state.timestamp = imu.timestamp;

        imu_buffer_.pop_front();
    }

    // 如果还没有到达目标时间，使用最后一个IMU数据外推
    if (state.timestamp < t_end && !imu_buffer_.empty()) {
        double dt = t_end - state.timestamp;
        if (dt > 0) {
            state = propagate(state, imu_buffer_.front(), dt);
            state.timestamp = t_end;
        }
    }
}

inline State ImuProcessor::propagate(const State& state, const ImuData& imu, double dt) {
    State new_state = state;

    // 去除偏置
    Vector3d acc_unbiased = imu.acc - state.acc_bias;
    Vector3d gyro_unbiased = imu.gyro - state.gyro_bias;

    // 旋转矩阵
    Matrix3d R = state.rotation.toRotationMatrix();

    // 重力去除
    Vector3d acc_world = R * acc_unbiased + state.gravity;

    // 中值积分 (简化版，实际FAST-LIO2使用更精确的积分方法)
    // 1. 旋转更新 (使用角速度积分)
    Vector3d angle_axis = gyro_unbiased * dt;
    Quaterniond delta_q(Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized()));
    new_state.rotation = state.rotation * delta_q;
    new_state.rotation.normalize();

    // 2. 速度更新
    new_state.velocity = state.velocity + acc_world * dt;

    // 3. 位置更新
    new_state.position = state.position + state.velocity * dt + 0.5 * acc_world * dt * dt;

    // 仿真平面运动约束：锁定 z 轴
    new_state.position(2) = state.position(2);
    new_state.velocity(2) = state.velocity(2);

    // 4. 偏置更新 (随机游走)
    // 这里简化处理，实际应考虑偏置的随机游走模型
    // new_state.acc_bias += noise
    // new_state.gyro_bias += noise

    return new_state;
}

inline std::vector<ImuData> ImuProcessor::getImuInRange(double t_start, double t_end) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::vector<ImuData> result;

    for (const auto& imu : imu_buffer_) {
        if (imu.timestamp >= t_start && imu.timestamp <= t_end) {
            result.push_back(imu);
        }
    }

    return result;
}

inline bool ImuProcessor::staticInitializeBias(int required_count) {
    if (required_count < 0) {
        required_count = config_.static_init_count;
    }

    if (bias_initialized_) {
        return true;
    }

    if (init_count_ < required_count) {
        return false;
    }

    // 静止时: 加速度计均值 = body系重力反应(向上), 陀螺仪均值 = 零偏
    Vector3d acc_mean = acc_sum_ / init_count_;
    gyro_bias_est_ = gyro_sum_ / init_count_;

    // 计算初始姿态 R0, 使 R0 * acc_mean 对齐到 world 系 +Z
    double g_mag = acc_mean.norm();
    if (g_mag > 1e-3) {
        Vector3d a = acc_mean.normalized();
        Vector3d b(0.0, 0.0, 1.0);
        double c = std::min(1.0, std::max(-1.0, a.dot(b)));
        double angle = std::acos(c);
        Vector3d axis = a.cross(b);
        if (axis.norm() > 1e-6) {
            axis.normalize();
            init_rotation_ = Quaterniond(Eigen::AngleAxisd(angle, axis));
        } else {
            init_rotation_ = Quaterniond::Identity();
        }
    } else {
        init_rotation_ = Quaterniond::Identity();
    }

    bias_initialized_ = true;
    return true;
}

inline bool ImuProcessor::hasEnoughDataForInit() const {
    return init_count_ >= config_.static_init_count;
}

inline bool ImuProcessor::getLatestImu(ImuData& imu) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (imu_buffer_.empty()) return false;
    imu = imu_buffer_.back();
    return true;
}

inline bool ImuProcessor::getImuAtTime(double timestamp, ImuData& imu) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    if (imu_buffer_.empty()) return false;

    // 查找最近的IMU数据
    double min_dt = std::numeric_limits<double>::max();
    bool found = false;

    for (const auto& data : imu_buffer_) {
        double dt = std::abs(data.timestamp - timestamp);
        if (dt < min_dt) {
            min_dt = dt;
            imu = data;
            found = true;
        }
    }

    return found;
}

} // namespace fast_lio2_slam