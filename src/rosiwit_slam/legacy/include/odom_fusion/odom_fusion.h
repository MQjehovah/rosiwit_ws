/**
 * @file odom_fusion.h
 * @brief FAST-LIO2 SLAM - 里程计融合模块
 * @author AI Development Team
 * @date 2026-04-24
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
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

} // namespace fast_lio2_slam
