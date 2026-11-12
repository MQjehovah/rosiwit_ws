#pragma once
#include <memory>
#include <deque>
#include <mutex>
#include "slam_core/i_frontend.h"
#include "commons.h"
#include "map_builder.h"
#include "ieskf.h"
#include "slope_detector.h"

namespace rosiwit_slam {

class FastLio2Frontend : public IFrontend {
public:
    bool init(const std::string& config_path) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    void onWheelOdom(const PoseStamped& odom) override;
    bool getOdometry(PoseStamped& out) override;
    bool getClouds(CloudType::Ptr& body, CloudType::Ptr& world) override;
    bool getKeyFrame(KeyFrame& out) override;
    bool getGlobalMap(PointVec& out) override;
    bool isMapping() const;

private:
    std::shared_ptr<IESKF>      m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    Config                      m_config;
    bool m_inited = false;
    bool m_first_after_mapping = false;

    std::vector<IMUSample, Eigen::aligned_allocator<IMUSample>> m_imu_buffer;
    std::mutex m_odom_mu;
    std::vector<PoseStamped, Eigen::aligned_allocator<PoseStamped>> m_odom_buffer;   // 轮式 odom (建图门限用)
    bool m_z_lock_valid = false;
    double m_z_lock = 0.0;   // pose 模式下固定的 IMU body 高度
    // 平地 z 保持 + 坡度解锁
    bool m_z_hold_init = false;
    bool m_was_slope = false;
    bool m_flat_ground_valid = false;
    V3D m_flat_z_axis = V3D::UnitZ();
    V3D m_flat_ground_normal = V3D::UnitZ();
    // 地面跟随: 世界系地面法向 (交给 LidarProcessor 做 v.z 软先验)
    bool m_ground_valid = false;
    V3D m_ground_normal_w = V3D::UnitZ();
    double m_ground_last_ok = -1.0;
    double m_ground_h0 = 0.0;      // 传感器离地高度 (起始标定)
    bool m_ground_h0_valid = false;
    double m_ground_z_target = 0.0;
    double m_ground_z_anchor = 0.0;
    bool m_ground_z_anchor_valid = false;
    bool estimateGroundNormal(const CloudType::Ptr& cloud, V3D& normal, double& plane_d);
    CloudType::Ptr m_current_cloud;
    bool m_skip_output = false;   // 本帧因 IMU 空洞被跳过, 不产出 odom/keyframe
    double m_last_lidar_time = 0.0;
    PoseStamped m_last_kf_pose;
    bool m_has_last_kf = false;
    size_t m_frame_counter = 0;
    double m_keyframe_trans_thresh = 1.0;
    double m_keyframe_rot_thresh = 0.17;
};

} // namespace rosiwit_slam
