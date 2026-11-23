// include/slam_core/slam_types.h
#pragma once
#include <Eigen/Eigen>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace rosiwit_slam {

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec  = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;

/// PCL 的宽高契约: width * height == points.size().
/// 点云用 push_back 拼出来时 width/height 常常还是 0, 而 pcl::transformPointCloud
/// 内部按 height = size / width 计算 —— width 为 0 就是整数除零 (SIGFPE)。
/// 任何要把点云交给 PCL 运算/发布的地方, 先过一遍这个函数。
inline void ensureCloudDims(CloudType & cloud)
{
  if (cloud.width != cloud.size() || cloud.height != 1) {
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = false;
  }
}

inline void ensureCloudDims(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
  if (cloud.width != cloud.size() || cloud.height != 1) {
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = false;
  }
}

struct IMUSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc  = V3D::Zero();
    V3D gyro = V3D::Zero();
    double time = -1.0;
};

struct LidarFrame {
    CloudType::Ptr cloud;
    double start_time = -1.0;
    double end_time   = -1.0;
    // 本帧 IMU 覆盖窗内存在大空洞 (>100~120ms). 前端应跳过该帧,
    // 不能拿空洞前后的陈旧 IMU 速度/零偏强行传播 (长航时发散根因).
    bool imu_gap = false;
};

/// 相机图像帧（核心不依赖 ROS；接口层负责把 sensor_msgs 转成 cv::Mat）。
/// image 为 BGR8 或 MONO8，time 与 IMU/LiDAR 使用同一时间基准（秒）。
struct ImageFrame {
    cv::Mat image;
    double time = -1.0;
    std::string frame_id;
};

/// 相机内参（与 config 的 camera 段一一对应；模型解释交给前端）。
struct CameraIntrinsics {
    std::string model = "Pinhole";  // Pinhole | EquidistantCamera | ...
    int width = 0;
    int height = 0;
    double scale = 1.0;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    double d0 = 0.0, d1 = 0.0, d2 = 0.0, d3 = 0.0;
};

enum class SlamState { INITIALIZING, READY, RUNNING, LOST };

struct PoseStamped {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time  = -1.0;
    M3D rot      = M3D::Identity();   // r_wi: world <- imu body
    V3D trans    = V3D::Zero();       // t_wi
    V3D vel      = V3D::Zero();       // world-frame velocity
};

struct SlamOutput {
    SlamState state = SlamState::INITIALIZING;
    PoseStamped pose;
    // 建图前端原始 LIO 位姿 (odom->body) 与回环锚点 (map->odom),
    // 供 handheld/lio 模式发布标准 map->odom->base TF.
    PoseStamped odom_pose;
    PoseStamped map_T_odom;
    bool has_odom_pose = false;
    CloudType::Ptr body_cloud;
    CloudType::Ptr world_cloud;
    /// 当前帧世界系彩色点云（LIVO 前端可提供；可为空）。累计彩色地图走 PCD 导出。
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_world_cloud;
    bool has_new_pose = false;
};

/// 栅格地图信息 (在 ISlamAlgorithm / IMapManager 间共享)
struct GridInfo {
    int width = 0, height = 0;
    double resolution = 0.05;
    double origin_x = 0, origin_y = 0;
    std::vector<int8_t> data;
};

// ============================================================================
// IMU 轴向换算 (唯一入口)
//
// 2026-11-23 实机事故: IMU 随雷达俯装 ~38°, 某处直接拿 gyro.z 当偏航角速度,
// 每转一圈少算 ~19% (实测转 349°, gyro.z 积分 284°, 投影 ω·up_body 362°)。
// 转弯时先验跟不上 -> 匹配锁错 yaw -> "一转弯就丢定位"。
//
// 约定: 需要偏航角速度的地方一律 imuYawRate(), 不允许直接写 gyro.z();
//       up_body 由安装外参 (URDF/TF/参数) 推导, 或由加速度自动估计.
// ============================================================================
inline double imuYawRate(const V3D& gyro, const V3D& up_body) {
    return gyro.dot(up_body);
}

/// up_body = R^T·(0,0,1): R 为 IMU->世界 旋转 (列向量是 IMU 各轴在世界系的表示)
inline V3D upBodyFromMount(const M3D& r_imu_to_world) {
    return r_imu_to_world.row(2).transpose().normalized();
}

/// 杆臂 ρ = R_base_lidarᵀ·t_base_lidar (传感器系常值): 雷达原点相对 base 原点
/// 的位置矢量在传感器系的表达. 原地绕 base 自转时雷达位置绕 base 划弧:
///   ṗ_lidar = R·(R_blᵀ·v_base + ω_s × ρ)
/// 缺了 ω_s×ρ 项, 转弯位置误差 = 2|ρ_xy|·sin(θ/2) (本机 |ρ_xy|≈0.41m, 180° 达 0.83m),
/// 静止时无影响 —— 2026-11-23 "一转弯就丢定位"根因.
inline V3D leverArmSensorFrame(const M3D& r_base_lidar, const V3D& t_base_lidar) {
    return r_base_lidar.transpose() * t_base_lidar;
}

/// 用加速度计估计 up_body (静止/匀速有效). false = 当前样本不可用.
inline bool accelUpBody(const V3D& acc, double gravity, double tol, V3D& up) {
    const double n = acc.norm();
    if (!(n > 1e-3) || std::fabs(n - gravity) > tol) return false;
    up = acc / n;
    return true;
}

} // namespace rosiwit_slam