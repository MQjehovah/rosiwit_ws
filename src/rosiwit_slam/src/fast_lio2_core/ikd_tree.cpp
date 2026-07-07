/**
 * @file ikd_tree.cpp
 * @brief FAST-LIO2 SLAM - KdTree封装实现 (基于PCL KdTreeFLANN)
 */

#include "fast_lio2_slam/fast_lio2_core/ikd_tree.h"

#include <cmath>

namespace fast_lio2_slam {

IKdTree::IKdTree() {}

IKdTree::IKdTree(const IKdTreeConfig& config) : config_(config) {}

void IKdTree::clear() {
    cloud_.reset(new pcl::PointCloud<PointType>());
    built_ = false;
}

void IKdTree::build(const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) return;
    cloud_ = cloud;
    kdtree_.setInputCloud(cloud_);
    built_ = true;
}

void IKdTree::insertPointCloud(const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) return;
    if (!cloud_) {
        cloud_.reset(new pcl::PointCloud<PointType>());
    }
    // 降采样: 仅添加没有近邻的点, 防止地图在已观测区域重复膨胀
    const double ds_sq = config_.downsample_size * config_.downsample_size;
    for (const auto& p : cloud->points) {
        if (built_) {
            std::vector<int> idx(1);
            std::vector<float> dist(1);
            if (kdtree_.nearestKSearch(p, 1, idx, dist) > 0) {
                if (dist[0] < ds_sq) {
                    continue;  // 距离过近, 跳过
                }
            }
        }
        cloud_->push_back(p);
    }
    // 用更新后的点云重建 kdtree
    kdtree_.setInputCloud(cloud_);
    built_ = true;
}

void IKdTree::insert(const PointType& point) {
    if (!cloud_) cloud_.reset(new pcl::PointCloud<PointType>());
    cloud_->push_back(point);
    kdtree_.setInputCloud(cloud_);
    built_ = true;
}

bool IKdTree::nearestSearch(const PointType& query, PointType& result, double& dist) {
    if (!built_) return false;
    std::vector<int> idx(1);
    std::vector<float> sq_dist(1);
    if (kdtree_.nearestKSearch(query, 1, idx, sq_dist) > 0) {
        result = cloud_->points[idx[0]];
        dist = std::sqrt(sq_dist[0]);
        return dist < config_.max_distance;
    }
    return false;
}

bool IKdTree::kNearestSearch(const PointType& query, int k,
                              std::vector<PointType>& results,
                              std::vector<double>& distances) {
    if (!built_ || k <= 0) return false;
    std::vector<int> idx(k);
    std::vector<float> sq_dist(k);
    int found = kdtree_.nearestKSearch(query, k, idx, sq_dist);
    if (found == 0) return false;
    results.clear();
    distances.clear();
    results.reserve(found);
    distances.reserve(found);
    for (int i = 0; i < found; ++i) {
        results.push_back(cloud_->points[idx[i]]);
        distances.push_back(sq_dist[i]);  // PCL返回平方距离
    }
    return found >= 1;
}

PointCloudPtr IKdTree::getAllPoints() {
    if (!cloud_) {
        PointCloudPtr empty(new pcl::PointCloud<PointType>());
        return empty;
    }
    return cloud_;
}

} // namespace fast_lio2_slam
