// include/algorithms/localization/maintainer/overlay_map_maintainer.h
#pragma once
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_core/i_map_maintainer.h"

namespace rosiwit_slam {

/// 非破坏性叠加地图维护:
///   - 原图冻结, 不改写;
///   - fitness 低于阈值(连续 min_confirm 帧) 视为环境变化 -> 当前扫描(按定位位姿转到
///     世界系)并入 overlay; 以体素去重, 以粗分块支持半径查询;
///   - getOverlay() 返回位姿附近 overlay, 由管道叠加到局部地图作为配准目标.
class OverlayMapMaintainer : public IMapMaintainer {
public:
    std::string name() const override { return "overlay"; }
    void configure(const MaintainerConfig& cfg) override { m_cfg = cfg; }
    void setBaseMap(const std::string& map_path, const CloudType::Ptr& base) override;
    void onLocalized(const PoseStamped& pose, const CloudType::Ptr& scan_body,
                     const MatchResult& match, bool pose_confident) override;
    CloudType::Ptr getOverlay(const PoseStamped& pose) override;
    CloudType::Ptr snapshotOverlay() override;
    bool save(const std::string& path) override;
    bool load(const std::string& path) override;

    size_t overlaySize() const { return m_overlay_count; }

private:
    static int64_t voxelKey(int cx, int cy, int cz);
    void addPoint(const Eigen::Vector3d& p);
    void rayCarve(const PoseStamped& pose, const CloudType::Ptr& scan_body);
    void rebuildCells();

    MaintainerConfig m_cfg;
    std::string m_map_path;
    CloudType::Ptr m_base;                       // 冻结原图(降采样), 用于增量/雕刻
    pcl::KdTreeFLANN<PointType> m_base_kdtree;
    bool m_has_base = false;
    double m_voxel = 0.3;
    double m_cell = 10.0;
    std::unordered_map<int64_t, long> m_voxel_seen;   // 体素 -> 最近观测帧(去重+衰减)
    std::unordered_map<int64_t, int> m_hits;          // 候选体素命中计数(持久化过滤)
    std::unordered_map<int64_t, CloudType::Ptr> m_cells;  // 粗分块索引
    Eigen::Vector3d m_last_add = Eigen::Vector3d::Zero();
    bool m_have_last = false;
    int m_low_count = 0;
    long m_frame = 0;
    size_t m_overlay_count = 0;
};

} // namespace rosiwit_slam
