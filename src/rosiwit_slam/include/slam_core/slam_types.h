// include/slam_core/slam_types.h
#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <memory>
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
    CloudType::Ptr body_cloud;
    CloudType::Ptr world_cloud;
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
