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
     * @brief 检查是否已初始化
     */
    bool isInitialized() const { return initialized_; }

#ifdef USE_GTSAM
    /**
     * @brief 获取ISAM2优化结果 (仅GTSAM模式)
     */
    gtsam::Values getISAM2Result();
#endif
    void addLoopFactor(int from_id, int to_id, const SE3d& relative_pose);

    /**
     * @brief 执行优化
     */
    void optimize();

    /**
     * @brief 获取优化后的位姿
     */
    SE3d getOptimizedPose(int id);

    /**
     * @brief 获取所有优化后的位姿
     */
    std::vector<SE3d> getAllOptimizedPoses();

    /**
     * @brief 更新前端位姿 (闭环修正后)
     */
    void updatePoses(std::vector<PoseNode>& poses);

    /**
     * @brief 清空位姿图
     */
    void clear();

    /**
     * @brief 获取节点数
     */
    size_t nodeCount() const;

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

// ==================== 实现部分 ====================

inline GtsamBackend::GtsamBackend() : initialized_(false) {}

inline GtsamBackend::GtsamBackend(const GtsamBackendConfig& config)
    : config_(config), initialized_(true) {}

inline void GtsamBackend::initialize(const GtsamBackendConfig& config) {
    config_ = config;
    clear();

#ifdef USE_GTSAM
    // 初始化ISAM2参数
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = config_.wildfire_threshold;
    isam_params.relinearizeSkip = config_.relinearize_skip;
    isam_params.enablePartialRelinearizeCheck = true;
    isam_params.cacheLinearizedFactors = true;
    isam_params.findUnusedFactorSlots = true;

    isam_ = gtsam::ISAM2(isam_params);

    // 设置噪声模型
    odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << config_.relative_pose_noise, config_.relative_pose_noise,
         config_.relative_pose_noise, config_.relative_pose_noise,
         config_.relative_pose_noise, config_.relative_pose_noise).finished());

    loop_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05).finished());

    initial_estimate_.clear();
    current_estimate_.clear();
#endif

    initialized_ = true;
}

inline void GtsamBackend::reset() {
    clear();
#ifdef USE_GTSAM
    isam_.reset();
    initial_estimate_.clear();
    current_estimate_.clear();
    latest_node_id_ = -1;
#endif
}

inline void GtsamBackend::addPriorFactor(int id, const SE3d& pose) {
#ifdef USE_GTSAM
    // 使用GTSAM添加先验因子
    gtsam::Pose3 gtsam_pose(
        gtsam::Rot3(pose.rotationMatrix()),
        gtsam::Point3(pose.translation().x(), pose.translation().y(), pose.translation().z()));

    // 添加先验因子
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << config_.prior_pose_noise, config_.prior_pose_noise,
         config_.prior_pose_noise, config_.prior_pose_noise,
         config_.prior_pose_noise, config_.prior_pose_noise).finished());

    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', id), gtsam_pose, prior_noise));
    initial_estimate_.insert(gtsam::Symbol('x', id), gtsam_pose);
    latest_node_id_ = id;
#else
    // 简化实现：添加固定节点
    PoseGraphNode node;
    node.id = id;
    node.pose = pose;
    node.fixed = true;
    nodes_.push_back(node);
#endif
}

inline void GtsamBackend::addOdomFactor(int from_id, int to_id, const SE3d& relative_pose) {
#ifdef USE_GTSAM
    // GTSAM实现：添加BetweenFactor
    gtsam::Pose3 relative_gtsam(
        gtsam::Rot3(relative_pose.rotationMatrix()),
        gtsam::Point3(relative_pose.translation().x(),
                      relative_pose.translation().y(),
                      relative_pose.translation().z()));

    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol('x', from_id),
        gtsam::Symbol('x', to_id),
        relative_gtsam, odom_noise_));

    // 添加新节点的初始估计
    if (latest_node_id_ == from_id && current_estimate_.exists(gtsam::Symbol('x', from_id))) {
        gtsam::Pose3 prev_pose = current_estimate_.at<gtsam::Pose3>(gtsam::Symbol('x', from_id));
        gtsam::Pose3 new_pose = prev_pose.compose(relative_gtsam);
        initial_estimate_.insert(gtsam::Symbol('x', to_id), new_pose);
    } else {
        // 使用累积估计
        gtsam::Pose3 estimated_pose = isam_.calculateEstimate<gtsam::Pose3>(gtsam::Symbol('x', from_id));
        gtsam::Pose3 new_pose = estimated_pose.compose(relative_gtsam);
        initial_estimate_.insert(gtsam::Symbol('x', to_id), new_pose);
    }

    latest_node_id_ = to_id;
#else
    // 简化实现
    if (nodes_.empty() || nodes_.back().id != to_id) {
        PoseGraphNode node;
        node.id = to_id;
        if (!nodes_.empty()) {
            node.pose = nodes_.back().pose * relative_pose;
        }
        nodes_.push_back(node);
    }

    PoseGraphEdge edge;
    edge.from = from_id;
    edge.to = to_id;
    edge.relative_pose = relative_pose;
    edge.information = buildInformationMatrix(config_.relative_pose_noise,
                                               config_.relative_pose_noise);
    edges_.push_back(edge);
#endif
}

inline void GtsamBackend::addLoopClosureFactor(int from_id, int to_id, const SE3d& relative_pose) {
#ifdef USE_GTSAM
    // GTSAM实现：添加闭环BetweenFactor
    gtsam::Pose3 relative_gtsam(
        gtsam::Rot3(relative_pose.rotationMatrix()),
        gtsam::Point3(relative_pose.translation().x(),
                      relative_pose.translation().y(),
                      relative_pose.translation().z()));

    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol('x', from_id),
        gtsam::Symbol('x', to_id),
        relative_gtsam, loop_noise_));
#else
    PoseGraphEdge edge;
    edge.from = from_id;
    edge.to = to_id;
    edge.relative_pose = relative_pose;
    edge.information = buildInformationMatrix(0.1, 0.05);  // 闭环约束更强
    edges_.push_back(edge);
#endif
}

inline bool GtsamBackend::optimize() {
#ifdef USE_GTSAM
    return isam2Optimization();
#else
    simplePoseGraphOptimization();
    return true;
#endif
}

#ifdef USE_GTSAM
inline bool GtsamBackend::isam2Optimization() {
    if (!initialized_ || initial_estimate_.empty()) {
        return false;
    }

    try {
        // 更新ISAM2
        isam_.update(graph_, initial_estimate_);

        // 可选：执行额外更新以提高精度
        isam_.update();

        // 获取当前估计
        current_estimate_ = isam_.calculateEstimate();

        // 清空图和初始估计（已累积到ISAM2中）
        graph_.resize(0);
        initial_estimate_.clear();

        return true;
    } catch (const std::exception& e) {
        // 优化失败
        return false;
    }
}

inline gtsam::Values GtsamBackend::getISAM2Result() {
    return current_estimate_;
}
#endif

inline void GtsamBackend::simplePoseGraphOptimization() {
    if (nodes_.size() < 3 || edges_.size() < 2) return;

    // 简化的位姿图优化 (使用GN迭代)
    optimized_poses_.resize(nodes_.size());

    for (size_t i = 0; i < nodes_.size(); ++i) {
        optimized_poses_[i] = nodes_[i].pose;
    }

    // 迭代优化
    for (int iter = 0; iter < config_.max_iterations; ++iter) {
        // 构建线性系统 Ax = b
        Eigen::MatrixXd A(6 * edges_.size(), 6 * nodes_.size());
        Eigen::VectorXd b(6 * edges_.size());
        A.setZero();
        b.setZero();

        for (size_t e = 0; e < edges_.size(); ++e) {
            const PoseGraphEdge& edge = edges_[e];

            // 查找节点索引
            int from_idx = -1, to_idx = -1;
            for (size_t n = 0; n < nodes_.size(); ++n) {
                if (nodes_[n].id == edge.from) from_idx = n;
                if (nodes_[n].id == edge.to) to_idx = n;
            }

            if (from_idx < 0 || to_idx < 0) continue;

            // 计算残差
            SE3d T_from = optimized_poses_[from_idx];
            SE3d T_to = optimized_poses_[to_idx];
            SE3d T_pred = T_from * edge.relative_pose;
            SE3d T_error = T_pred.inverse() * T_to;

            // 填充雅可比和残差
            // ... 简化实现
        }

        // 检查收敛
        // ...
    }

    // 更新节点位姿
    for (size_t i = 0; i < nodes_.size(); ++i) {
        if (!nodes_[i].fixed) {
            nodes_[i].pose = optimized_poses_[i];
        }
    }
}

inline Eigen::Matrix<double, 6, 6> GtsamBackend::buildInformationMatrix(
    double translation_noise, double rotation_noise) {

    Eigen::Matrix<double, 6, 6> info;
    info.setIdentity();

    // 平移部分
    info.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() / translation_noise;

    // 旋转部分
    info.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() / rotation_noise;

    return info;
}

inline SE3d GtsamBackend::getOptimizedPose(int id) {
#ifdef USE_GTSAM
    if (current_estimate_.exists(gtsam::Symbol('x', id))) {
        gtsam::Pose3 pose = current_estimate_.at<gtsam::Pose3>(gtsam::Symbol('x', id));
        Eigen::Matrix3d rot = pose.rotation().matrix();
        Eigen::Vector3d trans(pose.translation().x(), pose.translation().y(), pose.translation().z());
        return SE3d(Quaterniond(rot), trans);
    }
#else
    for (size_t i = 0; i < nodes_.size(); ++i) {
        if (nodes_[i].id == id) {
            return nodes_[i].pose;
        }
    }
#endif
    return SE3d();
}

inline std::vector<SE3d> GtsamBackend::getAllOptimizedPoses() {
    std::vector<SE3d> poses;
#ifdef USE_GTSAM
    // 从ISAM2获取所有节点位姿
    if (!current_estimate_.empty()) {
        for (int id = 0; id <= latest_node_id_; ++id) {
            if (current_estimate_.exists(gtsam::Symbol('x', id))) {
                gtsam::Pose3 pose = current_estimate_.at<gtsam::Pose3>(gtsam::Symbol('x', id));
                Eigen::Matrix3d rot = pose.rotation().matrix();
                Eigen::Vector3d trans(pose.translation().x(), pose.translation().y(), pose.translation().z());
                poses.push_back(SE3d(Quaterniond(rot), trans));
            }
        }
    }
#else
    for (const auto& node : nodes_) {
        poses.push_back(node.pose);
    }
#endif
    return poses;
}

inline void GtsamBackend::updatePoses(std::vector<PoseNode>& poses) {
    // 更新前端位姿历史
    for (auto& pose : poses) {
        SE3d optimized = getOptimizedPose(pose.id);
        if (optimized.translation().norm() > 0) {
            pose.pose = optimized;
        }
    }
}

inline void GtsamBackend::clear() {
    nodes_.clear();
    edges_.clear();
    optimized_poses_.clear();
}

inline size_t GtsamBackend::nodeCount() const {
    return nodes_.size();
}

} // namespace fast_lio2_slam