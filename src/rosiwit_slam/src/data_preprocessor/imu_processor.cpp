/**
 * @file imu_processor.cpp
 * @brief FAST-LIO2 SLAM - IMU数据处理实现
 */

#include "fast_lio2_slam/data_preprocessor/imu_processor.h"

#include <cmath>
#include <limits>

namespace fast_lio2_slam {

ImuProcessor::ImuProcessor()
    : initialized_(false), acc_sum_(Vector3d::Zero()),
      gyro_sum_(Vector3d::Zero()), init_count_(0), bias_initialized_(false) {}

ImuProcessor::ImuProcessor(const ImuProcessorConfig& config)
    : config_(config), initialized_(true), acc_sum_(Vector3d::Zero()),
      gyro_sum_(Vector3d::Zero()), init_count_(0), bias_initialized_(false) {}

void ImuProcessor::initialize(const ImuProcessorConfig& config) {
    config_ = config;
    initialized_ = true;
}

void ImuProcessor::setInitialState(const State& state) {
    // 设置初始状态，包括偏置
    config_.acc_noise = state.acc_bias.norm();
}

void ImuProcessor::setBias(const Vector3d& acc_bias, const Vector3d& gyro_bias) {
    bias_initialized_ = true;
}

void ImuProcessor::addImuData(const ImuData& imu) {
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

std::vector<ImuData> ImuProcessor::getImuBuffer() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return std::vector<ImuData>(imu_buffer_.begin(), imu_buffer_.end());
}

void ImuProcessor::clearBuffer() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    imu_buffer_.clear();
}

size_t ImuProcessor::bufferSize() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return imu_buffer_.size();
}

void ImuProcessor::predict(State& state, double t_end) {
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

State ImuProcessor::propagate(const State& state, const ImuData& imu, double dt) {
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

std::vector<ImuData> ImuProcessor::getImuInRange(double t_start, double t_end) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::vector<ImuData> result;

    for (const auto& imu : imu_buffer_) {
        if (imu.timestamp >= t_start && imu.timestamp <= t_end) {
            result.push_back(imu);
        }
    }

    return result;
}

bool ImuProcessor::staticInitializeBias(int required_count) {
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

bool ImuProcessor::hasEnoughDataForInit() const {
    return init_count_ >= config_.static_init_count;
}

bool ImuProcessor::getLatestImu(ImuData& imu) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (imu_buffer_.empty()) return false;
    imu = imu_buffer_.back();
    return true;
}

bool ImuProcessor::getImuAtTime(double timestamp, ImuData& imu) {
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
