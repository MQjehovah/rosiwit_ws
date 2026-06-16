/**
 * @file scan_context.h
 * @brief FAST-LIO2 SLAM - Scan Context闭环检测
 * @author AI Development Team
 * @date 2026-04-24
 * 
 * 实现Scan Context算法进行闭环检测:
 * 1. 从点云生成极坐标编码
 * 2. 计算相似度进行匹配
 * 3. 估计相对位姿
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <memory>
#include "fast_lio2_slam/common/types.h"

namespace fast_lio2_slam {

/**
 * @brief Scan Context配置
 */
struct ScanContextConfig {
    int ring_num = 20;           // 环数
    int sector_num = 60;         // 扇区数
    double max_range = 80.0;     // 最大距离
    double ring_height = 2.0;    // 环高度
    
    // 匯配参数
    double threshold = 0.3;      // 匯配阈值
    int min_match_count = 3;     // 最小匹配数
    
    // 搜索参数
    int exclude_near_scan = 50;  // 排除近邻帧
};

/**
 * @brief Scan Context描述子
 */
struct ScanContextDescriptor {
    Eigen::MatrixXd context;      // 描述矩阵 (ring_num x sector_num)
    Eigen::VectorXd ring_key;     // 环键值
    double timestamp;
    int scan_id;
    SE3d pose;
    
    ScanContextDescriptor() : timestamp(0), scan_id(-1) {}
};

/**
 * @brief Scan Context闭环检测类
 */
class ScanContext {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ScanContext();
    explicit ScanContext(const ScanContextConfig& config);
    ~ScanContext() = default;

    /**
     * @brief 初始化
     */
    void initialize(const ScanContextConfig& config);

    /**
     * @brief 从点云生成描述子
     */
    ScanContextDescriptor makeDescriptor(const PointCloudPtr& cloud, 
                                          double timestamp,
                                          int scan_id,
                                          const SE3d& pose);

    /**
     * @brief 添加关键帧
     */
    void addKeyframe(const ScanContextDescriptor& desc);

    /**
     * @brief 检测闭环
     */
    bool detectLoop(const ScanContextDescriptor& query,
                    LoopConstraint& constraint);

    /**
     * @brief 计算两个描述子之间的相似度
     */
    double computeSimilarity(const Eigen::MatrixXd& desc1,
                              const Eigen::MatrixXd& desc2);

    /**
     * @brief 计算旋转角度偏移
     */
    int computeYawOffset(const Eigen::MatrixXd& desc1,
                          const Eigen::MatrixXd& desc2);

    /**
     * @brief 获取所有关键帧
     */
    std::vector<ScanContextDescriptor> getKeyframes() const;

    /**
     * @brief 清空关键帧
     */
    void clearKeyframes();

    /**
     * @brief 获取关键帧数量
     */
    size_t keyframeCount() const;

private:
    /**
     * @brief 将点投影到极坐标网格
     */
    void projectToPolarGrid(const PointCloudPtr& cloud, Eigen::MatrixXd& context);

    /**
     * @brief 计算环键值
     */
    Eigen::VectorXd computeRingKey(const Eigen::MatrixXd& context);

    /**
     * @brief 旋转描述子
     */
    Eigen::MatrixXd rotateContext(const Eigen::MatrixXd& context, int shift);

private:
    ScanContextConfig config_;
    std::vector<ScanContextDescriptor> keyframes_;
    bool initialized_;
};

// ==================== 实现部分 ====================

inline ScanContext::ScanContext() : initialized_(false) {}

inline ScanContext::ScanContext(const ScanContextConfig& config)
    : config_(config), initialized_(true) {}

inline void ScanContext::initialize(const ScanContextConfig& config) {
    config_ = config;
    initialized_ = true;
}

inline ScanContextDescriptor ScanContext::makeDescriptor(const PointCloudPtr& cloud,
                                                          double timestamp,
                                                          int scan_id,
                                                          const SE3d& pose) {
    ScanContextDescriptor desc;
    
    desc.context.resize(config_.ring_num, config_.sector_num);
    desc.context.setZero();
    
    // 投影到极坐标网格
    projectToPolarGrid(cloud, desc.context);
    
    // 计算环键值
    desc.ring_key = computeRingKey(desc.context);
    
    desc.timestamp = timestamp;
    desc.scan_id = scan_id;
    desc.pose = pose;
    
    return desc;
}

inline void ScanContext::projectToPolarGrid(const PointCloudPtr& cloud,
                                             Eigen::MatrixXd& context) {
    double ring_step = config_.max_range / config_.ring_num;
    double sector_step = 360.0 / config_.sector_num;
    
    for (const auto& point : cloud->points) {
        double x = point.x;
        double y = point.y;
        double z = point.z;
        
        // 计算极坐标
        double range = std::sqrt(x * x + y * y);
        double angle = std::atan2(y, x) * 180.0 / M_PI;
        if (angle < 0) angle += 360.0;
        
        // 计算索引
        int ring_idx = static_cast<int>(range / ring_step);
        int sector_idx = static_cast<int>(angle / sector_step);
        
        // 范围限制
        if (ring_idx < 0) ring_idx = 0;
        if (ring_idx >= config_.ring_num) ring_idx = config_.ring_num - 1;
        if (sector_idx < 0) sector_idx = 0;
        if (sector_idx >= config_.sector_num) sector_idx = config_.sector_num - 1;
        
        // 记录最大高度值
        double current_height = context(ring_idx, sector_idx);
        if (std::abs(z) > std::abs(current_height)) {
            context(ring_idx, sector_idx) = z;
        }
    }
}

inline Eigen::VectorXd ScanContext::computeRingKey(const Eigen::MatrixXd& context) {
    Eigen::VectorXd ring_key(config_.ring_num);
    
    for (int i = 0; i < config_.ring_num; ++i) {
        // 使用该环的均值作为键值
        ring_key(i) = context.row(i).mean();
    }
    
    return ring_key;
}

inline void ScanContext::addKeyframe(const ScanContextDescriptor& desc) {
    keyframes_.push_back(desc);
}

inline bool ScanContext::detectLoop(const ScanContextDescriptor& query,
                                     LoopConstraint& constraint) {
    if (keyframes_.size() < config_.min_match_count) {
        return false;
    }
    
    // 排除最近的帧
    int start_idx = 0;
    if (keyframes_.size() > config_.exclude_near_scan) {
        start_idx = keyframes_.size() - config_.exclude_near_scan;
    }
    
    double max_similarity = 0;
    int best_match_idx = -1;
    int best_yaw_offset = 0;
    
    // 遍历历史关键帧
    for (int i = start_idx; i < keyframes_.size(); ++i) {
        const auto& candidate = keyframes_[i];
        
        // 计算相似度
        double similarity = computeSimilarity(query.context, candidate.context);
        
        if (similarity > max_similarity) {
            max_similarity = similarity;
            best_match_idx = i;
            best_yaw_offset = computeYawOffset(query.context, candidate.context);
        }
    }
    
    // 检查是否满足阈值
    if (max_similarity > config_.threshold && best_match_idx >= 0) {
        constraint.from_id = query.scan_id;
        constraint.to_id = keyframes_[best_match_idx].scan_id;
        constraint.score = max_similarity;
        constraint.yaw_diff = best_yaw_offset * (360.0 / config_.sector_num) * M_PI / 180.0;
        
        // 计算相对位姿
        SE3d pose_from = query.pose;
        SE3d pose_to = keyframes_[best_match_idx].pose;
        constraint.relative_pose = pose_to.inverse() * pose_from;
        
        return true;
    }
    
    return false;
}

inline double ScanContext::computeSimilarity(const Eigen::MatrixXd& desc1,
                                              const Eigen::MatrixXd& desc2) {
    // 计算余弦相似度
    int cols = desc1.cols();
    double similarity = 0;
    
    for (int col = 0; col < cols; ++col) {
        Eigen::VectorXd v1 = desc1.col(col);
        Eigen::VectorXd v2 = desc2.col(col);
        
        double dot = v1.dot(v2);
        double norm1 = v1.norm();
        double norm2 = v2.norm();
        
        if (norm1 > 0 && norm2 > 0) {
            similarity += dot / (norm1 * norm2);
        }
    }
    
    return similarity / cols;
}

inline int ScanContext::computeYawOffset(const Eigen::MatrixXd& desc1,
                                          const Eigen::MatrixXd& desc2) {
    double max_sim = 0;
    int best_offset = 0;
    
    // 尝试所有可能的旋转偏移
    for (int shift = 0; shift < config_.sector_num / 2; ++shift) {
        Eigen::MatrixXd rotated = rotateContext(desc2, shift);
        double sim = computeSimilarity(desc1, rotated);
        
        if (sim > max_sim) {
            max_sim = sim;
            best_offset = shift;
        }
    }
    
    return best_offset;
}

inline Eigen::MatrixXd ScanContext::rotateContext(const Eigen::MatrixXd& context,
                                                   int shift) {
    Eigen::MatrixXd rotated(context.rows(), context.cols());
    
    for (int i = 0; i < context.cols(); ++i) {
        int new_idx = (i + shift) % context.cols();
        rotated.col(new_idx) = context.col(i);
    }
    
    return rotated;
}

inline std::vector<ScanContextDescriptor> ScanContext::getKeyframes() const {
    return keyframes_;
}

inline void ScanContext::clearKeyframes() {
    keyframes_.clear();
}

inline size_t ScanContext::keyframeCount() const {
    return keyframes_.size();
}

} // namespace fast_lio2_slam