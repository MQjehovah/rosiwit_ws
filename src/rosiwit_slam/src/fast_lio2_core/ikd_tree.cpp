/**
 * @file ikd_tree.cpp
 * @brief FAST-LIO2 SLAM - 增量式KD树实现
 */

#include "fast_lio2_slam/fast_lio2_core/ikd_tree.h"

#include <algorithm>
#include <limits>

namespace fast_lio2_slam {

IKdTree::IKdTree()
    : root_idx_(-1), deleted_count_(0) {}

IKdTree::IKdTree(const IKdTreeConfig& config)
    : config_(config), root_idx_(-1), deleted_count_(0) {}

void IKdTree::initialize(const IKdTreeConfig& config) {
    config_ = config;
    clear();
}

void IKdTree::build(const PointCloudPtr& cloud) {
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

void IKdTree::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.clear();
    nodes_.clear();
    root_idx_ = -1;
    deleted_count_ = 0;
}

int IKdTree::buildRecursive(const std::vector<int>& indices, int depth) {
    if (indices.empty()) return -1;

    int node_idx = nodes_.size();
    nodes_.push_back(KDTreeNode());

    if (indices.size() <= static_cast<size_t>(config_.max_leaf_size)) {
        // 叶节点
        nodes_[node_idx].is_leaf = true;
        nodes_[node_idx].point = points_[indices[0]];
        nodes_[node_idx].axis = -1;
        nodes_[node_idx].left = -1;
        nodes_[node_idx].right = -1;
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

void IKdTree::insert(const PointType& point) {
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
        double node_value = 0.0, point_value = 0.0;

        switch (axis) {
            case 0: node_value = nodes_[node_idx].point.x; point_value = point.x; break;
            case 1: node_value = nodes_[node_idx].point.y; point_value = point.y; break;
            case 2: node_value = nodes_[node_idx].point.z; point_value = point.z; break;
        }

        if (point_value < node_value) {
            if (nodes_[node_idx].left < 0) {
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

    // 到达已存在的叶节点: 把该叶节点升级为内部节点 (保留原 point 作为分割锚点),
    // 然后根据轴比较结果, 把新点挂到对应一侧的新子叶节点。
    {
        int axis = depth % 3;
        nodes_[node_idx].is_leaf = false;
        nodes_[node_idx].axis = axis;

        double node_value = 0.0, point_value = 0.0;
        switch (axis) {
            case 0: node_value = nodes_[node_idx].point.x; point_value = point.x; break;
            case 1: node_value = nodes_[node_idx].point.y; point_value = point.y; break;
            default: node_value = nodes_[node_idx].point.z; point_value = point.z; break;
        }

        nodes_.push_back(KDTreeNode());
        int child_idx = nodes_.size() - 1;
        nodes_[child_idx].point = point;
        nodes_[child_idx].is_leaf = true;
        nodes_[child_idx].axis = -1;
        nodes_[child_idx].parent = node_idx;

        if (point_value < node_value) {
            nodes_[node_idx].left = child_idx;
        } else {
            nodes_[node_idx].right = child_idx;
        }
    }

    if (needRebalance()) {
        rebalance();
    }
}

void IKdTree::insertPointCloud(const PointCloudPtr& cloud) {
    for (const auto& point : cloud->points) {
        insert(point);
    }
}

bool IKdTree::nearestSearch(const PointType& query, PointType& result, double& dist) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (root_idx_ < 0) return false;

    dist = std::numeric_limits<double>::max();
    nearestSearchRecursive(query, root_idx_, result, dist);

    return dist < config_.max_distance;
}

void IKdTree::nearestSearchRecursive(const PointType& query, int node_idx,
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
    double query_value = 0.0, node_value = 0.0;
    switch (axis) {
        case 0: query_value = query.x; node_value = node.point.x; break;
        case 1: query_value = query.y; node_value = node.point.y; break;
        case 2: query_value = query.z; node_value = node.point.z; break;
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

bool IKdTree::kNearestSearch(const PointType& query, int k,
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

void IKdTree::kNearestSearchRecursive(const PointType& query, int node_idx,
                                      std::vector<std::pair<double, PointType>>& heap,
                                      int k) {
    if (node_idx < 0 || nodes_[node_idx].deleted) return;

    const KDTreeNode& node = nodes_[node_idx];

    double d = pointDistance(query, node.point);

    // 插入堆
    if (heap.size() < static_cast<size_t>(k)) {
        heap.push_back({d, node.point});
        std::push_heap(heap.begin(), heap.end());
    } else if (d < heap.front().first) {
        std::pop_heap(heap.begin(), heap.end());
        heap.back() = {d, node.point};
        std::push_heap(heap.begin(), heap.end());
    }

    if (node.is_leaf) return;

    int axis = node.axis;
    double query_value = 0.0, node_value = 0.0;
    switch (axis) {
        case 0: query_value = query.x; node_value = node.point.x; break;
        case 1: query_value = query.y; node_value = node.point.y; break;
        case 2: query_value = query.z; node_value = node.point.z; break;
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

    double max_dist = heap.size() >= static_cast<size_t>(k)
                          ? heap.front().first
                          : std::numeric_limits<double>::max();
    if (planeDistance(query, axis, node_value) < max_dist) {
        kNearestSearchRecursive(query, second_branch, heap, k);
    }
}

int IKdTree::radiusSearch(const PointType& query, double radius,
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

PointCloudPtr IKdTree::getAllPoints() {
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

void IKdTree::remove(const PointType& point, double threshold) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 找到最近的点并标记删除
    PointType nearest;
    double dist;
    if (nearestSearch(point, nearest, dist) && dist < threshold) {
        for (auto& node : nodes_) {
            if (!node.deleted && pointDistance(node.point, nearest) < 0.001) {
                node.deleted = true;
                deleted_count_++;
                break;
            }
        }
    }
}

void IKdTree::removePointsInRadius(const Vector3d& center, double radius) {
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

bool IKdTree::needRebalance() const {
    if (points_.empty()) return false;
    double delete_ratio = deleted_count_ / points_.size();
    return delete_ratio > config_.balance_factor;
}

void IKdTree::rebalance() {
    if (!needRebalance()) return;
    PointCloudPtr active_points = getAllPoints();
    build(active_points);
}

void IKdTree::rebuildTree() {
    rebalance();
}

double IKdTree::pointDistance(const PointType& p1, const PointType& p2) const {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                     (p1.y - p2.y) * (p1.y - p2.y) +
                     (p1.z - p2.z) * (p1.z - p2.z));
}

double IKdTree::planeDistance(const PointType& point, int axis, double value) const {
    switch (axis) {
        case 0: return std::abs(point.x - value);
        case 1: return std::abs(point.y - value);
        case 2: return std::abs(point.z - value);
    }
    return 0;
}

} // namespace fast_lio2_slam
