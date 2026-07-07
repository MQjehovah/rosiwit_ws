/**
 * @file odom_fusion.cpp
 * @brief FAST-LIO2 SLAM - 里程计融合模块实现
 */

#include "fast_lio2_slam/odom_fusion/odom_fusion.h"

#include <cmath>
#include <limits>

namespace fast_lio2_slam {

OdomFusion::OdomFusion() : initialized_(false) {}

OdomFusion::OdomFusion(const OdomFusionConfig& config)
    : config_(config), initialized_(true) {}

void OdomFusion::initialize(const OdomFusionConfig& config) {
    config_ = config;
    initialized_ = true;
}

void OdomFusion::addOdomData(const OdomData& odom) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    const size_t max_buffer_size = 200;
    if (odom_buffer_.size() >= max_buffer_size) {
        odom_buffer_.pop_front();
    }

    odom_buffer_.push_back(odom);
}

State OdomFusion::looseCoupling(const State& lidar_state, const OdomData& odom_data) {
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

State OdomFusion::tightCoupling(const State& lidar_state,
                                const std::vector<OdomData>& odom_data) {
    // 紧耦合融合 (简化实现)
    // 实际应使用因子图优化 (GTSAM)
    State fused_state = lidar_state;

    if (!config_.enable || odom_data.empty()) return lidar_state;

    // 计算里程计约束
    for (const auto& odom : odom_data) {
        // 添加里程计观测作为约束
        // 实际实现需要使用因子图
        (void)odom;
    }

    return fused_state;
}

bool OdomFusion::getNearestOdom(double timestamp, OdomData& odom) {
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

void OdomFusion::clearBuffer() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    odom_buffer_.clear();
}

bool OdomFusion::hasOdomData() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return !odom_buffer_.empty();
}

void OdomFusion::setWeights(double lidar_weight, double odom_weight) {
    config_.lidar_weight = lidar_weight;
    config_.odom_weight = odom_weight;
}

} // namespace fast_lio2_slam
