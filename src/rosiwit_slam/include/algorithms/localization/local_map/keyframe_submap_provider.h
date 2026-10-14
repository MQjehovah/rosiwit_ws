// include/algorithms/localization/local_map/keyframe_submap_provider.h
#pragma once
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_core/i_local_map_provider.h"

namespace rosiwit_slam {

/// 关键帧子图提供者: 以关键帧位姿建立 KD-tree, 按当前位姿取最近 K 帧(受半径约束)
/// 的点云并集作为配准目标. 相比体素半径切片, 每帧是"该视角的完整扫描", 结构更
/// 完整. 关键帧来源: setKeyframes() 注入 (如由 map.xzmap 载入); 若未注入, 则退化为
/// 用全局地图按 cell_size 分块合成"虚拟关键帧", 保证开箱可用.
class KeyframeSubmapProvider : public ILocalMapProvider {
public:
    std::string name() const override { return "keyframe_submap"; }
    void configure(const LocalMapConfig& cfg) override { m_cfg = cfg; }
    void setMap(const std::string& map_path, const CloudType::Ptr& global_map) override;
    void setKeyframes(const std::vector<LocalKeyframe>& keyframes) override;
    CloudType::Ptr getLocalMap(const PoseStamped& pose) override;

private:
    struct Kf {
        PoseStamped pose;      // world
        CloudType::Ptr world;  // 世界系点云
    };
    void buildFromMapFallback();
    void buildPoseTree();

    LocalMapConfig m_cfg;
    CloudType::Ptr m_map;
    std::vector<Kf> m_kfs;
    CloudType::Ptr m_pose_cloud;
    pcl::KdTreeFLANN<PointType> m_pose_kdtree;
    bool m_pose_tree_ready = false;
    bool m_have_injected = false;
};

} // namespace rosiwit_slam
