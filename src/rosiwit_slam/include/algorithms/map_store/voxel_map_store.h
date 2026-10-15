// include/algorithms/map_store/voxel_map_store.h
#pragma once
#include <unordered_map>
#include <cstdint>
#include "slam_core/i_map_store.h"

namespace rosiwit_slam {

/// 体素哈希 + 格内存点 存储 (iVox 思路): O(1) 插入/删除/雕刻, 天然增量维护;
/// nearestK 用格扩展做精确近邻 (与 KD 树等价), 满足"换存储不改算法".
class VoxelMapStore : public IMapStore {
public:
    explicit VoxelMapStore(double leaf = 0.3) : m_leaf(leaf > 0 ? leaf : 0.3) {}

    std::string name() const override { return "voxel"; }
    bool buildFromCloud(const CloudType::Ptr& cloud) override;
    CloudType::Ptr queryRadius(const Eigen::Vector3d& center, double radius) const override;
    int nearestK(const PointType& query, int k,
                 std::vector<PointType>& points, std::vector<float>& sqdists) const override;
    double nearestDist2(const PointType& query) const override;
    VoxelStats statsAt(const Eigen::Vector3d& p, double radius) const override;
    bool groundAt(double x, double y, double& z, Eigen::Vector3d& normal) const override;
    void insert(const CloudType::Ptr& world_points, double time = -1.0) override;
    void erase(const CloudType::Ptr& world_points) override;
    void carve(const Eigen::Vector3d& origin, const CloudType::Ptr& endpoints) override;
    void decay(double now) override;
    void setDecayWindow(double w) { m_decay_window = w; }
    uint64_t revision() const override { return m_rev; }
    size_t size() const override { return m_size; }
    bool save(const std::string& path) const override;
    bool load(const std::string& path) override;

private:
    struct Cell {
        CloudType::Ptr pts;
        int cx = 0, cy = 0, cz = 0;
        double t = -1.0;
    };
    static int64_t key(int cx, int cy, int cz);
    int64_t keyOf(double x, double y, double z) const;

    double m_leaf = 0.3;
    std::unordered_map<int64_t, Cell> m_cells;
    double m_decay_window = 30.0;
    uint64_t m_rev = 0;
    size_t m_size = 0;
};

} // namespace rosiwit_slam
