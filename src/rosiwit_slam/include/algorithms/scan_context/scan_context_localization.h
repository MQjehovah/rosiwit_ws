#pragma once
#include <string>
#include <vector>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include "slam_core/i_localization.h"
#include "algorithms/scan_context/scan_context.h"

namespace rosiwit_slam {

/// ScanContext 全局重定位:
/// 1. 建图阶段关键帧描述子通过 saveScanContextDatabase 落盘 (map.pcd + ".scdb")
/// 2. setMap() 加载该数据库
/// 3. onLidar() 用当前点云描述子与数据库匹配 (RingKey 预筛 + 精确匹配)
/// 4. 命中后以匹配关键帧位姿为初值, GICP 对全局地图精配准, 输出全局位姿
class ScanContextLocalization : public ILocalization {
public:
    ScanContextLocalization() = default;
    ~ScanContextLocalization() override = default;

    bool init(const std::string& config_path) override;
    void setMap(const std::string& map_name) override;
    void setInitPose(const PoseStamped& pose) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    Status getStatus() override;
    bool getPose(PoseStamped& out) override;

private:
    CloudType::Ptr downsampleCloud(const CloudType::Ptr& cloud, float resolution);

    ScanContext m_sc;
    std::vector<ScanContextDatabaseEntry> m_db;
    CloudType::Ptr m_map_cloud;
    CloudType::Ptr m_downsampled_map;
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> m_gicp;
    pcl::VoxelGrid<PointType> m_map_voxel;
    pcl::VoxelGrid<PointType> m_scan_voxel;

    PoseStamped m_pose;
    Status m_status = INIT;
    bool m_has_db = false;

    // ScanContext 配置
    int m_num_rings = 20;
    int m_num_sectors = 60;
    double m_max_radius = 80.0;
    double m_min_radius = 2.0;

    // 匹配阈值
    double m_dist_thresh = 0.4;
    double m_candidate_ratio = 0.3;

    // GICP 精配准
    float m_voxel_res = 0.5f;
    float m_icp_max_corr_dist = 2.0f;
    float m_icp_fitness_thresh = 1.0f;
    int m_icp_max_iter = 32;
};

} // namespace rosiwit_slam
