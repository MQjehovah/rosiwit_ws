#pragma once
#include <string>
#include <deque>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigen>
#include "slam_core/i_localization.h"

namespace rosiwit_slam {

/// GICP 配准定位, 支持 IMU 航迹推算作为初始猜测。
///
/// 工作流:
///   1. setMap() 加载全局点云地图
///   2. setInitPose() 设置初始位姿
///   3. onImu() → 内部 IMU 航迹推算传播位姿
///   4. onLidar() → 以推算位姿为初始猜测, GICP 配准修正
class GicpLocalization : public ILocalization {
public:
    GicpLocalization();
    ~GicpLocalization() override = default;

    bool init(const std::string& config_path) override;
    void setMap(const std::string& map_name) override;
    void setInitPose(const PoseStamped& pose) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    Status getStatus() override;
    bool getPose(PoseStamped& out) override;

private:
    CloudType::Ptr downsampleCloud(const CloudType::Ptr& cloud, float resolution);

    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp_;
    CloudType::Ptr map_cloud_;
    CloudType::Ptr downsampled_map_;
    pcl::VoxelGrid<PointType> map_voxel_;
    pcl::VoxelGrid<PointType> scan_voxel_;

    PoseStamped current_pose_;
    Status status_ = INIT;

    // ============ IMU 航迹推算 ============
    std::deque<IMUSample> m_imu_buffer;     ///< LiDAR 帧间的 IMU 缓存
    double m_last_imu_time = -1.0;           ///< 上一个 IMU 时间戳
    bool m_has_imu_propagation = false;      ///< 是否有 IMU 数据可用
    V3D m_last_vel = V3D::Zero();            ///< 机体速度 (body frame)
    V3D m_acc_bias = V3D::Zero();            ///< 加速度计零偏 (粗略估计)
    V3D m_gyro_bias = V3D::Zero();           ///< 陀螺仪零偏
    bool m_first_imu_received = false;       ///< 是否收到过首个 IMU

    // ============ 配置参数 ============
    float voxel_resolution_ = 0.5f;
    float fitness_threshold_ = 10.0f;
    int max_iterations_ = 64;
    double transformation_epsilon_ = 1e-8;
    bool has_map_ = false;
    bool first_scan_ = true;
};

} // namespace rosiwit_slam
