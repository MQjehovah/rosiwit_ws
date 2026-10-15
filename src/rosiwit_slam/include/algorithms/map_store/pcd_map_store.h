// include/algorithms/map_store/pcd_map_store.h
#pragma once
#include <unordered_set>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_core/i_map_store.h"

namespace rosiwit_slam {

/// 点云 + KD 树 存储 (等价现状). 精确近邻/半径查询.
class PcdMapStore : public IMapStore {
public:
    explicit PcdMapStore(double leaf = 0.0) : m_leaf(leaf) {}

    std::string name() const override { return "pcd"; }
    bool buildFromCloud(const CloudType::Ptr& cloud) override;
    CloudType::Ptr queryRadius(const Eigen::Vector3d& center, double radius) const override;
    int nearestK(const PointType& query, int k,
                 std::vector<PointType>& points, std::vector<float>& sqdists) const override;
    double nearestDist2(const PointType& query) const override;
    VoxelStats statsAt(const Eigen::Vector3d& p, double radius) const override;
    void insert(const CloudType::Ptr& world_points) override;
    void erase(const CloudType::Ptr& world_points) override;
    uint64_t revision() const override { return m_rev; }
    size_t size() const override { return m_cloud ? m_cloud->size() : 0; }
    bool save(const std::string& path) const override;
    bool load(const std::string& path) override;

private:
    void ensureTree() const;
    int64_t voxelKey(const PointType& p) const;

    double m_leaf = 0.0;   // >0: erase 时按体素匹配
    CloudType::Ptr m_cloud;
    mutable pcl::KdTreeFLANN<PointType> m_kdt;
    mutable bool m_dirty = true;
    uint64_t m_rev = 0;
};

} // namespace rosiwit_slam
