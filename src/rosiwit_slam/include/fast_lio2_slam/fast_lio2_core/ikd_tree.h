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

#include <pcl/point_cloud.h>
#include <vector>
#include <mutex>
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

    void initialize(const IKdTreeConfig& config);

    void setConfig(const IKdTreeConfig& config) { config_ = config; }

    void build(const PointCloudPtr& cloud);

    void clear();

    size_t size() const { return points_.size() - deleted_count_; }
    bool empty() const { return size() == 0; }

    void insert(const PointType& point);
    void insertPointCloud(const PointCloudPtr& cloud);

    void remove(const PointType& point, double threshold = 0.01);
    void removePointsInRadius(const Vector3d& center, double radius);

    bool nearestSearch(const PointType& query, PointType& result, double& dist);
    bool kNearestSearch(const PointType& query, int k,
                        std::vector<PointType>& results,
                        std::vector<double>& distances);
    int radiusSearch(const PointType& query, double radius,
                     std::vector<PointType>& results);

    PointCloudPtr getAllPoints();

    void rebalance();
    void rebuildTree();

    int getDeletedCount() const { return deleted_count_; }

private:
    int buildRecursive(const std::vector<int>& indices, int depth);
    void nearestSearchRecursive(const PointType& query, int node_idx,
                                 PointType& result, double& dist);
    void kNearestSearchRecursive(const PointType& query, int node_idx,
                                  std::vector<std::pair<double, PointType>>& heap,
                                  int k);
    int selectAxis(const std::vector<int>& indices);
    double pointDistance(const PointType& p1, const PointType& p2) const;
    double planeDistance(const PointType& point, int axis, double value) const;
    bool needRebalance() const;

private:
    IKdTreeConfig config_;
    std::vector<KDTreeNode> nodes_;    // 树节点
    std::vector<PointType> points_;    // 原始点
    int root_idx_;                      // 根节点索引
    int deleted_count_;                 // 删除计数
    mutable std::mutex mutex_;          // 互斥锁
};

} // namespace fast_lio2_slam
