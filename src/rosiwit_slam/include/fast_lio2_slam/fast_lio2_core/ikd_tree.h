/**
 * @file ikd_tree.h
 * @brief FAST-LIO2 SLAM - 增量式KD树 (iKD-Tree) 地图管理
 * @author AI Development Team
 * @date 2026-04-24
 *
 * 实现FAST-LIO2的增量式KD树:
 * 1. 高效的点云插入和删除
 * 2. 快速的最近邻搜索
 * 3. 平衡维护
 */

#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <memory>
#include <mutex>
#include <queue>
#include <algorithm>
#include <limits>
#include "fast_lio2_slam/common/types.h"

namespace fast_lio2_slam {

/**
 * @brief KD树节点
 */
struct KDTreeNode {
    PointType point;
    int axis;              // 分割轴
    int left = -1;         // 左子节点索引
    int right = -1;        // 右子节点索引
    int parent = -1;       // 父节点索引
    bool deleted = false;  // 是否已删除
    bool is_leaf = false;  // 是否为叶节点

    KDTreeNode() : axis(0) {}
};

/**
 * @brief iKD-Tree配置
 */
struct IKdTreeConfig {
    int max_leaf_size = 10;          // 叶节点最大点数
    double balance_factor = 0.5;     // 平衡因子
    double max_distance = 5.0;       // 最大搜索距离
    int max_points = 500000;         // 最大点数
    bool enable_downsample = true;   // 启用降采样
    double downsample_size = 0.2;    // 降采样大小
};

/**
 * @brief 增量式KD树 (iKD-Tree)
 *
 * 特点:
 * - 支持增量插入和删除
 * - 自动平衡维护
 * - 高效的最近邻搜索
 */
class IKdTree {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IKdTree();
    explicit IKdTree(const IKdTreeConfig& config);
    ~IKdTree() = default;

    /**
     * @brief 初始化树
     */
    void initialize(const IKdTreeConfig& config);

    /**
     * @brief 设置配置
     */
    void setConfig(const IKdTreeConfig& config) { config_ = config; }

    /**
     * @brief 批量构建树
     */
    void build(const PointCloudPtr& cloud);

    /**
     * @brief 清空树
     */
    void clear();

    /**
     * @brief 获取点数
     */
    size_t size() const { return points_.size() - deleted_count_; }

    /**
     * @brief 检查是否为空
     */
    bool empty() const { return size() == 0; }

    /**
     * @brief 插入点
     */
    void insert(const PointType& point);

    /**
     * @brief 批量插入点云
     */
    void insertPointCloud(const PointCloudPtr& cloud);

    /**
     * @brief 删除点 (标记删除)
     */
    void remove(const PointType& point, double threshold = 0.01);

    /**
     * @brief 删除区域内的点
     */
    void removePointsInRadius(const Vector3d& center, double radius);

    /**
     * @brief 最近邻搜索 (返回最近点)
     */
    bool nearestSearch(const PointType& query, PointType& result, double& dist);

    /**
     * @brief K近邻搜索
     */
    bool kNearestSearch(const PointType& query, int k,
                        std::vector<PointType>& results,
                        std::vector<double>& distances);

    /**
     * @brief 半径搜索
     */
    int radiusSearch(const PointType& query, double radius,
                     std::vector<PointType>& results);

    /**
     * @brief 获取所有点
     */
    PointCloudPtr getAllPoints();

    /**
     * @brief 平衡树
     */
    void rebalance();

    /**
     * @brief 获取删除标记的点数
     */
    int getDeletedCount() const { return deleted_count_; }

    /**
     * @brief 构建失败处理
     */
    void rebuildTree();

private:
    /**
     * @brief 递归构建KD树
     */
    int buildRecursive(const std::vector<int>& indices, int depth);

    /**
     * @brief 最近邻搜索递归
     */
    void nearestSearchRecursive(const PointType& query, int node_idx,
                                 PointType& result, double& dist);

    /**
     * @brief K近邻搜索递归
     */
    void kNearestSearchRecursive(const PointType& query, int node_idx,
                                  std::vector<std::pair<double, PointType>>& heap,
                                  int k);

    /**
     * @brief 选择分割轴
     */
    int selectAxis(const std::vector<int>& indices);

    /**
     * @brief 计算节点距离
     */
    double pointDistance(const PointType& p1, const PointType& p2) const;

    /**
     * @brief 平面距离
     */
    double planeDistance(const PointType& point, int axis, double value) const;

    /**
     * @brief 检查是否需要平衡
     */
    bool needRebalance() const;

private:
    IKdTreeConfig config_;
    std::vector<KDTreeNode> nodes_;    // 树节点
    std::vector<PointType> points_;    // 原始点
    int root_idx_;                      // 根节点索引
    int deleted_count_;                 // 删除计数
    mutable std::mutex mutex_;          // 互斥锁
};

// ==================== 实现部分 ====================

inline IKdTree::IKdTree()
    : root_idx_(-1), deleted_count_(0) {}

inline IKdTree::IKdTree(const IKdTreeConfig& config)
    : config_(config), root_idx_(-1), deleted_count_(0) {}

inline void IKdTree::initialize(const IKdTreeConfig& config) {
    config_ = config;
    clear();
}

inline void IKdTree::build(const PointCloudPtr& cloud) {
    if (cloud->empty()) return;

    std::lock_guard<std::mutex> lock(mutex_);
    // Inline clear operations (do NOT call clear() which also locks mutex_)
    points_.clear();
    nodes_.clear();
    root_idx_ = -1;
    deleted_count_ = 0;

    // 复制点
    points_.reserve(cloud->size());
    for (const auto& point : cloud->points) {
        points_.push_back(point);
    }

    // 构建索引
    std::vector<int> indices;
    for (size_t i = 0; i < points_.size(); ++i) {
        indices.push_back(i);
    }

    // 递归构建
    nodes_.reserve(points_.size() * 2);
    root_idx_ = buildRecursive(indices, 0);
}

inline void IKdTree::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.clear();
    nodes_.clear();
    root_idx_ = -1;
    deleted_count_ = 0;
}

inline int IKdTree::buildRecursive(const std::vector<int>& indices, int depth) {
    if (indices.empty()) return -1;

    int node_idx = nodes_.size();
    nodes_.push_back(KDTreeNode());

    if (indices.size() <= config_.max_leaf_size) {
        // 叶节点
        nodes_[node_idx].is_leaf = true;
        nodes_[node_idx].point = points_[indices[0]];
        nodes_[node_idx].axis = -1;
        nodes_[node_idx].left = -1;
        nodes_[node_idx].right = -1;

        // 叶节点可以存储多个点
        for (size_t i = 1; i < indices.size(); ++i) {
            // 可以扩展为存储多个点
        }

        return node_idx;
    }

    // 选择分割轴
    int axis = depth % 3;  // 循环使用 x, y, z 轴
    nodes_[node_idx].axis = axis;

    // 找到中值作为分割点
    std::vector<int> sorted_indices = indices;
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [this, axis](int a, int b) {
                  switch (axis) {
                      case 0: return points_[a].x < points_[b].x;
                      case 1: return points_[a].y < points_[b].y;
                      case 2: return points_[a].z < points_[b].z;
                  }
                  return false;
              });

    int median_idx = sorted_indices.size() / 2;
    nodes_[node_idx].point = points_[sorted_indices[median_idx]];

    // 分割索引
    std::vector<int> left_indices(sorted_indices.begin(),
                                   sorted_indices.begin() + median_idx);
    std::vector<int> right_indices(sorted_indices.begin() + median_idx + 1,
                                    sorted_indices.end());

    // 递归构建子树
    nodes_[node_idx].left = buildRecursive(left_indices, depth + 1);
    nodes_[node_idx].right = buildRecursive(right_indices, depth + 1);

    if (nodes_[node_idx].left >= 0) {
        nodes_[nodes_[node_idx].left].parent = node_idx;
    }
    if (nodes_[node_idx].right >= 0) {
        nodes_[nodes_[node_idx].right].parent = node_idx;
    }

    return node_idx;
}

inline void IKdTree::insert(const PointType& point) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (root_idx_ < 0) {
        // 树为空，创建根节点
        points_.push_back(point);
        nodes_.push_back(KDTreeNode());
        root_idx_ = 0;
        nodes_[root_idx_].point = point;
        nodes_[root_idx_].is_leaf = true;
        return;
    }

    // 降采样: 已存在近距点则跳过, 防止地图在已观测区域重复膨胀/涂抹
    // (静止机器人的地图不应从 3000 点涨到几十万点)
    if (config_.enable_downsample) {
        PointType nearest;
        double dist = std::numeric_limits<double>::max();
        nearestSearchRecursive(point, root_idx_, nearest, dist);
        if (dist < config_.downsample_size) {
            return;
        }
    }

    // 添加点
    points_.push_back(point);

    // 找到插入位置
    int node_idx = root_idx_;
    int depth = 0;

    while (!nodes_[node_idx].is_leaf) {
        int axis = nodes_[node_idx].axis;
        double node_value, point_value;

        switch (axis) {
            case 0:
                node_value = nodes_[node_idx].point.x;
                point_value = point.x;
                break;
            case 1:
                node_value = nodes_[node_idx].point.y;
                point_value = point.y;
                break;
            case 2:
                node_value = nodes_[node_idx].point.z;
                point_value = point.z;
                break;
        }

        if (point_value < node_value) {
            if (nodes_[node_idx].left < 0) {
                // 创建左子节点
                nodes_.push_back(KDTreeNode());
                nodes_[node_idx].left = nodes_.size() - 1;
                nodes_[nodes_[node_idx].left].point = point;
                nodes_[nodes_[node_idx].left].is_leaf = true;
                nodes_[nodes_[node_idx].left].parent = node_idx;
                return;
            }
            node_idx = nodes_[node_idx].left;
        } else {
            if (nodes_[node_idx].right < 0) {
                // 创建右子节点
                nodes_.push_back(KDTreeNode());
                nodes_[node_idx].right = nodes_.size() - 1;
                nodes_[nodes_[node_idx].right].point = point;
                nodes_[nodes_[node_idx].right].is_leaf = true;
                nodes_[nodes_[node_idx].right].parent = node_idx;
                return;
            }
            node_idx = nodes_[node_idx].right;
        }

        depth++;
    }

    // 到达已存在的叶节点: 需要将该叶节点分裂为内部节点 + 两个子叶节点
    // 否则新点只进了 points_, 永远无法被 nearestSearch / getAllPoints 找到
    {
        PointType existing_point = nodes_[node_idx].point;
        int axis = depth % 3;
        nodes_[node_idx].is_leaf = false;
        nodes_[node_idx].axis = axis;

        double existing_value, new_value;
        switch (axis) {
            case 0: existing_value = existing_point.x; new_value = point.x; break;
            case 1: existing_value = existing_point.y; new_value = point.y; break;
            default: existing_value = existing_point.z; new_value = point.z; break;
        }

        nodes_.push_back(KDTreeNode());
        int left_idx = nodes_.size() - 1;
        nodes_[left_idx].is_leaf = true;
        nodes_[left_idx].axis = -1;
        nodes_[left_idx].parent = node_idx;

        nodes_.push_back(KDTreeNode());
        int right_idx = nodes_.size() - 1;
        nodes_[right_idx].is_leaf = true;
        nodes_[right_idx].axis = -1;
        nodes_[right_idx].parent = node_idx;

        if (new_value < existing_value) {
            nodes_[left_idx].point = point;
            nodes_[right_idx].point = existing_point;
        } else {
            nodes_[left_idx].point = existing_point;
            nodes_[right_idx].point = point;
        }

        nodes_[node_idx].left = left_idx;
        nodes_[node_idx].right = right_idx;
        nodes_[node_idx].point = new_value < existing_value ? point : existing_point;
    }

    // 检查是否需要平衡
    if (needRebalance()) {
        rebalance();
    }
}

inline void IKdTree::insertPointCloud(const PointCloudPtr& cloud) {
    for (const auto& point : cloud->points) {
        insert(point);
    }
}

inline bool IKdTree::nearestSearch(const PointType& query, PointType& result, double& dist) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (root_idx_ < 0) return false;

    dist = std::numeric_limits<double>::max();
    nearestSearchRecursive(query, root_idx_, result, dist);

    return dist < config_.max_distance;
}

inline void IKdTree::nearestSearchRecursive(const PointType& query, int node_idx,
                                             PointType& result, double& dist) {
    if (node_idx < 0 || nodes_[node_idx].deleted) return;

    const KDTreeNode& node = nodes_[node_idx];

    // 计算当前节点距离
    double d = pointDistance(query, node.point);
    if (d < dist) {
        dist = d;
        result = node.point;
    }

    if (node.is_leaf) return;

    int axis = node.axis;
    double query_value, node_value;
    switch (axis) {
        case 0:
            query_value = query.x;
            node_value = node.point.x;
            break;
        case 1:
            query_value = query.y;
            node_value = node.point.y;
            break;
        case 2:
            query_value = query.z;
            node_value = node.point.z;
            break;
    }

    // 先搜索较近的分支
    int first_branch, second_branch;
    if (query_value < node_value) {
        first_branch = node.left;
        second_branch = node.right;
    } else {
        first_branch = node.right;
        second_branch = node.left;
    }

    nearestSearchRecursive(query, first_branch, result, dist);

    // 检查是否需要搜索另一分支
    if (planeDistance(query, axis, node_value) < dist) {
        nearestSearchRecursive(query, second_branch, result, dist);
    }
}

inline bool IKdTree::kNearestSearch(const PointType& query, int k,
                                     std::vector<PointType>& results,
                                     std::vector<double>& distances) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (root_idx_ < 0 || k <= 0) return false;

    // 使用最大堆存储最近邻
    std::vector<std::pair<double, PointType>> heap;
    heap.reserve(k + 1);

    kNearestSearchRecursive(query, root_idx_, heap, k);

    // 排序并返回
    std::sort(heap.begin(), heap.end());

    results.clear();
    distances.clear();

    for (const auto& pair : heap) {
        results.push_back(pair.second);
        distances.push_back(pair.first);
    }

    return !results.empty();
}

inline void IKdTree::kNearestSearchRecursive(const PointType& query, int node_idx,
                                              std::vector<std::pair<double, PointType>>& heap,
                                              int k) {
    if (node_idx < 0 || nodes_[node_idx].deleted) return;

    const KDTreeNode& node = nodes_[node_idx];

    double d = pointDistance(query, node.point);

    // 插入堆
    if (heap.size() < k) {
        heap.push_back({d, node.point});
        std::push_heap(heap.begin(), heap.end());
    } else if (d < heap.front().first) {
        std::pop_heap(heap.begin(), heap.end());
        heap.back() = {d, node.point};
        std::push_heap(heap.begin(), heap.end());
    }

    if (node.is_leaf) return;

    int axis = node.axis;
    double query_value, node_value;
    switch (axis) {
        case 0:
            query_value = query.x;
            node_value = node.point.x;
            break;
        case 1:
            query_value = query.y;
            node_value = node.point.y;
            break;
        case 2:
            query_value = query.z;
            node_value = node.point.z;
            break;
    }

    int first_branch, second_branch;
    if (query_value < node_value) {
        first_branch = node.left;
        second_branch = node.right;
    } else {
        first_branch = node.right;
        second_branch = node.left;
    }

    kNearestSearchRecursive(query, first_branch, heap, k);

    double max_dist = heap.size() >= k ? heap.front().first : std::numeric_limits<double>::max();
    if (planeDistance(query, axis, node_value) < max_dist) {
        kNearestSearchRecursive(query, second_branch, heap, k);
    }
}

inline int IKdTree::radiusSearch(const PointType& query, double radius,
                                  std::vector<PointType>& results) {
    std::lock_guard<std::mutex> lock(mutex_);

    results.clear();
    if (root_idx_ < 0) return 0;

    // 简化实现: 使用K近邻搜索后过滤
    std::vector<double> distances;
    kNearestSearch(query, 1000, results, distances);  // 搜索足够多的点

    // 过滤距离
    int count = 0;
    for (size_t i = 0; i < results.size(); ++i) {
        if (distances[i] <= radius) {
            count++;
        } else {
            results.resize(i);
            break;
        }
    }

    return count;
}

inline PointCloudPtr IKdTree::getAllPoints() {
    std::lock_guard<std::mutex> lock(mutex_);

    PointCloudPtr cloud(new pcl::PointCloud<PointType>());
    for (const auto& node : nodes_) {
        if (!node.deleted) {
            cloud->points.push_back(node.point);
        }
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    return cloud;
}

inline void IKdTree::remove(const PointType& point, double threshold) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 找到最近的点并标记删除
    PointType nearest;
    double dist;
    if (nearestSearch(point, nearest, dist) && dist < threshold) {
        // 找到节点并标记删除
        for (auto& node : nodes_) {
            if (!node.deleted && pointDistance(node.point, nearest) < 0.001) {
                node.deleted = true;
                deleted_count_++;
                break;
            }
        }
    }
}

inline void IKdTree::removePointsInRadius(const Vector3d& center, double radius) {
    std::lock_guard<std::mutex> lock(mutex_);

    PointType query;
    query.x = center(0);
    query.y = center(1);
    query.z = center(2);

    std::vector<PointType> points_in_radius;
    radiusSearch(query, radius, points_in_radius);

    for (const auto& point : points_in_radius) {
        for (auto& node : nodes_) {
            if (!node.deleted && pointDistance(node.point, point) < 0.001) {
                node.deleted = true;
                deleted_count_++;
            }
        }
    }
}

inline bool IKdTree::needRebalance() const {
    if (points_.empty()) return false;

    // 检查删除比例
    double delete_ratio = deleted_count_ / points_.size();
    return delete_ratio > config_.balance_factor;
}

inline void IKdTree::rebalance() {
    if (!needRebalance()) return;

    // 重建树
    PointCloudPtr active_points = getAllPoints();
    build(active_points);
}

inline void IKdTree::rebuildTree() {
    rebalance();
}

inline double IKdTree::pointDistance(const PointType& p1, const PointType& p2) const {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                     (p1.y - p2.y) * (p1.y - p2.y) +
                     (p1.z - p2.z) * (p1.z - p2.z));
}

inline double IKdTree::planeDistance(const PointType& point, int axis, double value) const {
    switch (axis) {
        case 0: return std::abs(point.x - value);
        case 1: return std::abs(point.y - value);
        case 2: return std::abs(point.z - value);
    }
    return 0;
}

} // namespace fast_lio2_slam