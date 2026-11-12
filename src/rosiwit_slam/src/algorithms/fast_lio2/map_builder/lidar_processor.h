#pragma once
#include "commons.h"
#include "ieskf.h"
#include "ikd_Tree.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <unordered_map>
#include <cmath>

struct LocalMap
{
    bool initialed = false;
    BoxPointType local_map_corner;
    Vec<BoxPointType> cub_to_rm;
};

class LidarProcessor
{
public:
    LidarProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void setOdomPrior(bool valid, double speed, double yaw_rate) {
        m_odom_prior = valid; m_odom_speed = speed; m_odom_yaw_rate = yaw_rate;
    }
    void setOdomFuse(bool fuse) { m_odom_fuse = fuse; }
    /// 地面跟随: n_world 是**世界系**地面法向 (朝上). 用它把 v.z 软约束到
    /// v_h 在坡面上的垂直分量, 从而 z 真实跟随坡面而不积分漂移.
    void setGroundPlane(bool valid, const V3D &n_world, bool enabled = true,
                        double z_target = 0.0) {
        m_ground_follow = enabled;
        m_ground_z_target = z_target;
        m_ground_valid = valid;
        if (valid) m_ground_normal = n_world;
    }

    void trimCloudMap();

    void incrCloudMap();

    void carveFreeSpace(const State &state);

    void initCloudMap(PointVec &point_vec);

    void process(SyncPackage &package);

    void updateLossFunc(State &state, SharedState &share_data);

    double robustWeight(double residual) const;

    static long voxelKey(const PointType &p, double voxel_size);

    static CloudType::Ptr transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t);
    M3D r_wl() { return m_kf->x().r_wi * m_kf->x().r_il; }
    V3D t_wl() { return m_kf->x().t_wi + m_kf->x().r_wi * m_kf->x().t_il; }

    void collectGlobalMap(std::vector<PointType, Eigen::aligned_allocator<PointType>> &points);

private:
    Config m_config;
    LocalMap m_local_map;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<KD_TREE<PointType>> m_ikdtree;
    CloudType::Ptr m_cloud_lidar;
    CloudType::Ptr m_cloud_down_lidar;
    CloudType::Ptr m_cloud_down_world;
    std::vector<bool> m_point_selected_flag;
    CloudType::Ptr m_norm_vec;
    CloudType::Ptr m_effect_cloud_lidar;
    CloudType::Ptr m_effect_norm_vec;
    std::vector<PointVec> m_nearest_points;
    std::vector<double> m_robust_weight;
    std::vector<double> m_effect_weight;
    std::vector<double> m_point_residual;
    pcl::VoxelGrid<PointType> m_scan_filter;
    std::unordered_map<long, int> m_voxel_count;
    std::unordered_map<long, long> m_voxel_last_frame;
    std::unordered_map<long, int> m_voxel_free;   // L3a: 体素"被穿过"计数
    long m_frame_idx = 0;
    double m_last_cloud_end = -1.0;   // 上一帧已处理点云结束时间, 用于识别空洞后首帧
    bool m_odom_prior = false;
    double m_odom_speed = 0.0;
    double m_odom_yaw_rate = 0.0;
    bool m_odom_fuse = false;
    // 坡地 z 安全先验: 2D SLAM 的 base z 是平地基准, 只作为"防跑飞"参考,
    // 用 50m 死区, 正常坡道 (实测 450s 内 z 约 -6..13m) 完全不受影响。
    bool m_z_ref_valid = false;
    double m_z_ref_offset = 0.0;
    // 地面跟随: 世界系地面法向, 用来把 v.z 软约束到坡面分量 (不锁 z/roll/pitch)
    bool m_ground_follow = false;
    bool m_ground_valid = false;
    V3D m_ground_normal = V3D::UnitZ();
    V3D m_ground_normal_prev = V3D::UnitZ();
    bool m_ground_z_init = false;
    double m_ground_z_ref = 0.0;   // 兼容旧字段
    double m_ground_z_target = 0.0; // 传感器正下方地面高度 + h0
};
