/**
 * @file gtsam_backend.h
 * @brief FAST-LIO2 SLAM - GTSAM后端优化
 * @author AI Development Team
 * @date 2026-04-24
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "fast_lio2_slam/common/sophus_se3.hpp"
#include "fast_lio2_slam/common/types.h"
#include <vector>

// GTSAM条件编译支持
#ifdef USE_GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#endif

namespace fast_lio2_slam {

/**
 * @brief GTSAM后端配置
 */
struct GtsamBackendConfig {
    int max_iterations = 20;           // 最大迭代次数
    double relative_pose_noise = 0.1;  // 相对位姿噪声
    double prior_pose_noise = 0.01;    // 先验位姿噪声

    // ISAM2参数
    double wildfire_threshold = 0.001;
    int relinearize_skip = 1;
};

/**
 * @brief 位姿图节点 (简化版，用于无GTSAM时)
 */
struct PoseGraphNode {
    int id;
    SE3d pose;
    bool fixed;

    PoseGraphNode() : id(-1), fixed(false) {}
};

/**
 * @brief 位姿图边
 */
struct PoseGraphEdge {
    int from;
    int to;
    SE3d relative_pose;
    Eigen::Matrix<double, 6, 6> information;

    PoseGraphEdge() : from(-1), to(-1) {
        information = Eigen::Matrix<double, 6, 6>::Identity();
    }
};

/**
 * @brief GTSAM后端优化类
 *
 * 支持两种实现:
 * 1. 使用GTSAM库的ISAM2增量优化 (USE_GTSAM宏启用)
 * 2. 简化的位姿图优化 (当GTSAM不可用时)
 */
class GtsamBackend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GtsamBackend();
    explicit GtsamBackend(const GtsamBackendConfig& config);
    ~GtsamBackend() = default;

    /**
     * @brief 初始化后端优化器
     * @param config 配置参数
     */
    void initialize(const GtsamBackendConfig& config);

    /**
     * @brief 重置优化器状态
     */
    void reset();

    /**
     * @brief 添加先验节点 (固定首节点)
     * @param id 节点ID
     * @param pose 先验位姿
     */
    void addPriorFactor(int id, const SE3d& pose);

    /**
     * @brief 添加里程计因子 (相邻帧)
     * @param from_id 起始节点ID
     * @param to_id 目标节点ID
     * @param relative_pose 相对位姿变换
     */
    void addOdomFactor(int from_id, int to_id, const SE3d& relative_pose);

    /**
     * @brief 添加闭环因子
     * @param from_id 闭环起始节点ID
     * @param to_id 闭环目标节点ID
     * @param relative_pose 相对位姿测量
     */
    void addLoopClosureFactor(int from_id, int to_id, const SE3d& relative_pose);

    /**
     * @brief 添加闭环因子 (别名)
     */
    void addLoopFactor(int from_id, int to_id, const SE3d& relative_pose);

    /**
     * @brief 执行优化
     * @return 是否优化成功
     */
    bool optimize();

    /**
     * @brief 获取优化后的位姿
     * @param id 节点ID
     * @return 优化后的SE3位姿
     */
    SE3d getOptimizedPose(int id);

    /**
     * @brief 获取所有优化后的位姿
     * @return 位姿向量
     */
    std::vector<SE3d> getAllOptimizedPoses();

    /**
     * @brief 获取当前节点数量
     */
    size_t getNodeCount() const { return nodes_.size(); }

    /**
     * @brief 获取因子数量
     */
    size_t getFactorCount() const { return edges_.size(); }

    /**
     * @brief 获取节点数 (别名)
     */
    size_t nodeCount() const;

    /**
     * @brief 检查是否已初始化
     */
    bool isInitialized() const { return initialized_; }

#ifdef USE_GTSAM
    /**
     * @brief 获取ISAM2优化结果 (仅GTSAM模式)
     */
    gtsam::Values getISAM2Result();
#endif

    /**
     * @brief 更新前端位姿 (闭环修正后)
     */
    void updatePoses(std::vector<PoseNode>& poses);

    /**
     * @brief 清空位姿图
     */
    void clear();

private:
    /**
     * @brief 简化的位姿图优化 (不使用GTSAM)
     */
    void simplePoseGraphOptimization();

#ifdef USE_GTSAM
    /**
     * @brief GTSAM ISAM2增量优化
     */
    bool isam2Optimization();
#endif

    /**
     * @brief 构建信息矩阵
     */
    Eigen::Matrix<double, 6, 6> buildInformationMatrix(double translation_noise,
                                                        double rotation_noise);

private:
    GtsamBackendConfig config_;
    std::vector<PoseGraphNode> nodes_;
    std::vector<PoseGraphEdge> edges_;
    bool initialized_;

    // 优化结果
    std::vector<SE3d> optimized_poses_;

#ifdef USE_GTSAM
    // GTSAM相关成员
    gtsam::ISAM2 isam_;                  // ISAM2优化器
    gtsam::NonlinearFactorGraph graph_;  // 因子图
    gtsam::Values initial_estimate_;     // 初始估计值
    gtsam::Values current_estimate_;     // 当前优化结果

    // 累积噪声模型
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr loop_noise_;

    // 最新节点ID
    int latest_node_id_ = -1;
#endif
};

} // namespace fast_lio2_slam
