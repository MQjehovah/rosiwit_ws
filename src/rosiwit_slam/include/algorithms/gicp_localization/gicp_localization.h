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
    // 雷达 -> IMU 外参 (ROS1 override YAML 顶层 r_il/t_il). 定位路径不经过建图前端,
    // 需要自己把扫描转到 IMU 系; 实机雷达前倾 43 度时不转就永远配不上。
    bool m_lidar_to_imu_valid = false;
    Eigen::Matrix4f m_lidar_to_imu = Eigen::Matrix4f::Identity();
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
    float voxel_resolution_ = 0.3f;      // 地图/扫描降采样体素 (yaml: voxel_resolution)
    float fitness_threshold_ = 10.0f;
    int max_iterations_ = 64;
    double transformation_epsilon_ = 1e-8;
    float max_corr_dist_ = 2.0f;          // GICP 最大对应点距离 (yaml: gicp_max_corr_dist)
    double euclid_eps_ = 0.01;            // GICP 欧氏收敛阈值 (yaml: gicp_euclid_eps)
    bool has_map_ = false;
    bool first_scan_ = true;

    // ============ 输出位姿滤波 (可选, yaml: use_pose_filter) ============
    // 开启后: 预测(IMU角速度积分+常速) → GICP量测 → 按门控的修正量平滑融合,
    // 替代"每帧整体覆盖", 抑制帧间抖动且几乎不滞后; 关闭则保持原行为。
    bool use_pose_filter_ = true;
    double pose_filter_alpha_ = 0.6;       // 修正量融合系数 (0~1)
    double pose_filter_max_xy_m_ = 0.5;    // 位置残差门限 m (超限不采信)
    double pose_filter_max_yaw_deg_ = 20.0;// 航向残差门限 deg (超限不采信)
    V3D m_vel_world_ = V3D::Zero();        // 世界系滤波速度估计
    bool m_filter_inited_ = false;         // 滤波状态是否已初始化
    double m_last_filter_time_ = -1.0;     // 上一次滤波更新时间
    V3D m_last_filtered_trans_ = V3D::Zero(); // 上一次滤波后的世界系位移
};

} // namespace rosiwit_slam
