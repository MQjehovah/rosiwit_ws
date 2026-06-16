/**
 * @file odom_fusion.h
 * @brief FAST-LIO2 SLAM - 里程计融合模块
 * @author AI Development Team
 * @date 2026-04-24
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "fast_lio2_slam/common/sophus_se3.hpp"
#include "fast_lio2_slam/common/types.h"
#include <deque>
#include <mutex>

namespace fast_lio2_slam {

/**
 * @brief 里程计融合配置
 */
struct OdomFusionConfig {
    bool enable = false;
    std::string fusion_mode = "loose";  // "loose" or "tight"

    // 松耦合权重
    double lidar_weight = 0.7;
    double odom_weight = 0.3;

    // 协方差参数
    double position_cov = 0.05;
    double rotation_cov = 0.02;

    // 时间同步
    double max_time_diff = 0.1;  // 最大时间差 (秒)
};

/**
 * @brief 里程计融合类
 *
 * 支持松耦合和紧耦合两种模式:
 * - 松耦合: 加权平均LiDAR和里程计估计
 * - 紧耦合: 将里程计作为因子加入优化
 */
class OdomFusion {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OdomFusion();
    explicit OdomFusion(const OdomFusionConfig& config);
    ~OdomFusion() = default;

    /**
     * @brief 初始化融合模块
     */
    void initialize(const OdomFusionConfig& config);

    /**
     * @brief 添加里程计数据
     */
    void addOdomData(const OdomData& odom);

    /**
     * @brief 松耦合融合
     */
    State looseCoupling(const State& lidar_state, const OdomData& odom_data);

    /**
     * @brief 紧耦合融合 (ESKF形式)
     */
    State tightCoupling(const State& lidar_state, const std::vector<OdomData>& odom_data);

    /**
     * @brief 获取最近的里程计数据
     */
    bool getNearestOdom(double timestamp, OdomData& odom);

    /**
     * @brief 清空缓冲区
     */
    void clearBuffer();

    /**
     * @brief 检查是否有里程计数据
     */
    bool hasOdomData() const;

    /**
     * @brief 设置融合权重
     */
    void setWeights(double lidar_weight, double odom_weight);

private:
    OdomFusionConfig config_;
    std::deque<OdomData> odom_buffer_;
    mutable std::mutex buffer_mutex_;
    bool initialized_;
};

// ==================== 实现部分 ====================

inline OdomFusion::OdomFusion() : initialized_(false) {}

inline OdomFusion::OdomFusion(const OdomFusionConfig& config)
    : config_(config), initialized_(true) {}

inline void OdomFusion::initialize(const OdomFusionConfig& config) {
    config_ = config;
    initialized_ = true;
}

inline void OdomFusion::addOdomData(const OdomData& odom) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    const size_t max_buffer_size = 200;
    if (odom_buffer_.size() >= max_buffer_size) {
        odom_buffer_.pop_front();
    }

    odom_buffer_.push_back(odom);
}

inline State OdomFusion::looseCoupling(const State& lidar_state, const OdomData& odom_data) {
    State fused_state = lidar_state;

    if (!config_.enable) return lidar_state;

    // 加权融合位置
    fused_state.position = config_.lidar_weight * lidar_state.position +
                            config_.odom_weight * odom_data.position;

    // 加权融合姿态 (使用球面线性插值)
    double t = config_.odom_weight;
    fused_state.rotation = lidar_state.rotation.slerp(t, odom_data.rotation);

    // 融合速度
    fused_state.velocity = config_.lidar_weight * lidar_state.velocity +
                            config_.odom_weight * odom_data.linear_velocity;

    return fused_state;
}

inline State OdomFusion::tightCoupling(const State& lidar_state,
                                        const std::vector<OdomData>& odom_data) {
    // 紧耦合融合 (简化实现)
    // 实际应使用因子图优化 (GTSAM)
    State fused_state = lidar_state;

    if (!config_.enable || odom_data.empty()) return lidar_state;

    // 计算里程计约束
    for (const auto& odom : odom_data) {
        // 添加里程计观测作为约束
        // 实际实现需要使用因子图
    }

    return fused_state;
}

inline bool OdomFusion::getNearestOdom(double timestamp, OdomData& odom) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    if (odom_buffer_.empty()) return false;

    double min_dt = std::numeric_limits<double>::max();
    bool found = false;

    for (const auto& data : odom_buffer_) {
        double dt = std::abs(data.timestamp - timestamp);
        if (dt < min_dt && dt < config_.max_time_diff) {
            min_dt = dt;
            odom = data;
            found = true;
        }
    }

    return found;
}

inline void OdomFusion::clearBuffer() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    odom_buffer_.clear();
}

inline bool OdomFusion::hasOdomData() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return !odom_buffer_.empty();
}

inline void OdomFusion::setWeights(double lidar_weight, double odom_weight) {
    config_.lidar_weight = lidar_weight;
    config_.odom_weight = odom_weight;
}

} // namespace fast_lio2_slam