// include/slam_core/slam_types.h
#pragma once
#include <Eigen/Eigen>
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

} // namespace rosiwit_slam
