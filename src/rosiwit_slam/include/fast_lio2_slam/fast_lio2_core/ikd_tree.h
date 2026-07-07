/**
 * @file ikd_tree.h
 * @brief FAST-LIO2 SLAM - KdTree封装 (基于PCL KdTreeFLANN)
 * @author AI Development Team
 * @date 2026-04-24
 *
 * 使用 PCL::KdTreeFLANN 提供高效的最近邻搜索,
 * 替代原有的自定义增量式KD树实现。
 */

#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <memory>
#include "fast_lio2_slam/common/types.h"

namespace fast_lio2_slam {

/**
 * @brief KdTree配置
 */
struct IKdTreeConfig {
    double downsample_size = 0.2;   // 增量插入降采样大小
    double max_distance = 5.0;      // 最近邻搜索最大有效距离
};

/**
 * @brief KdTree封装 (基于 PCL KdTreeFLANN)
 *
 * 特点:
 * - 增量插入时对近距点降采样, 避免地图重复膨胀
 * - 高效的最近邻/K近邻搜索 (PCL FLANN 后端)
 */
class IKdTree {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IKdTree();
    explicit IKdTree(const IKdTreeConfig& config);
    ~IKdTree() = default;

    void initialize(const IKdTreeConfig& config) { config_ = config; }
    void setConfig(const IKdTreeConfig& config) { config_ = config; }

    void build(const PointCloudPtr& cloud);
    void clear();

    size_t size() const { return cloud_ ? cloud_->size() : 0; }
    bool empty() const { return size() == 0; }

    void insertPointCloud(const PointCloudPtr& cloud);
    void insert(const PointType& point);

    bool nearestSearch(const PointType& query, PointType& result, double& dist);
    bool kNearestSearch(const PointType& query, int k,
                        std::vector<PointType>& results,
                        std::vector<double>& distances);

    PointCloudPtr getAllPoints();
    int getDeletedCount() const { return 0; }
    void rebuildTree() { if (cloud_ && !cloud_->empty()) build(cloud_); }

private:
    IKdTreeConfig config_;
    PointCloudPtr cloud_;
    pcl::KdTreeFLANN<PointType> kdtree_;
    bool built_ = false;
};

} // namespace fast_lio2_slam
