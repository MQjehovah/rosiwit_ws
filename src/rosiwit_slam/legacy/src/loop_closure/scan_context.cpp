/**
 * @file scan_context.cpp
 * @brief FAST-LIO2 SLAM - Scan Context闭环检测实现
 */

#include "fast_lio2_slam/loop_closure/scan_context.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace fast_lio2_slam {

ScanContext::ScanContext() : initialized_(false) {}

ScanContext::ScanContext(const ScanContextConfig& config)
    : config_(config), initialized_(true) {}

void ScanContext::initialize(const ScanContextConfig& config) {
    config_ = config;
    initialized_ = true;
}

ScanContextDescriptor ScanContext::makeDescriptor(const PointCloudPtr& cloud,
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

void ScanContext::projectToPolarGrid(const PointCloudPtr& cloud,
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

Eigen::VectorXd ScanContext::computeRingKey(const Eigen::MatrixXd& context) {
    Eigen::VectorXd ring_key(config_.ring_num);

    for (int i = 0; i < config_.ring_num; ++i) {
        // 使用该环的均值作为键值
        ring_key(i) = context.row(i).mean();
    }

    return ring_key;
}

void ScanContext::addKeyframe(const ScanContextDescriptor& desc) {
    keyframes_.push_back(desc);
}

bool ScanContext::detectLoop(const ScanContextDescriptor& query,
                             LoopConstraint& constraint) {
    if (keyframes_.size() < static_cast<size_t>(config_.min_match_count)) {
        return false;
    }

    // 排除最近的帧
    int start_idx = 0;
    if (static_cast<int>(keyframes_.size()) > config_.exclude_near_scan) {
        start_idx = static_cast<int>(keyframes_.size()) - config_.exclude_near_scan;
    }

    double max_similarity = 0;
    int best_match_idx = -1;
    int best_yaw_offset = 0;

    // 遍历历史关键帧
    for (int i = start_idx; i < static_cast<int>(keyframes_.size()); ++i) {
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

double ScanContext::computeSimilarity(const Eigen::MatrixXd& desc1,
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

int ScanContext::computeYawOffset(const Eigen::MatrixXd& desc1,
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

Eigen::MatrixXd ScanContext::rotateContext(const Eigen::MatrixXd& context,
                                           int shift) {
    Eigen::MatrixXd rotated(context.rows(), context.cols());

    for (int i = 0; i < context.cols(); ++i) {
        int new_idx = (i + shift) % context.cols();
        rotated.col(new_idx) = context.col(i);
    }

    return rotated;
}

std::vector<ScanContextDescriptor> ScanContext::getKeyframes() const {
    return keyframes_;
}

void ScanContext::clearKeyframes() {
    keyframes_.clear();
}

size_t ScanContext::keyframeCount() const {
    return keyframes_.size();
}

} // namespace fast_lio2_slam
