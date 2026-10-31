// include/algorithms/fast_livo2/fast_livo2_config.h
#pragma once

#include <Eigen/Eigen>
#include <string>

#include "slam_core/slam_types.h"

namespace rosiwit_slam {

/// FAST-LIVO2 前端配置（ROS 无关）。
/// yaml 来源：mapping.camera / mapping.extrin_calib / mapping.fast_livo2，
/// 以及接口层写入的根级 r_il / t_il（TF 派生，优先级更高）。
struct FastLivo2Config {
    // ---- 相机 ----
    CameraIntrinsics camera;

    // ---- 外参 ----
    Eigen::Matrix3d lidar_imu_rot = Eigen::Matrix3d::Identity();  // R_il
    Eigen::Vector3d lidar_imu_trans = Eigen::Vector3d::Zero();    // t_il
    Eigen::Matrix3d cam_lidar_rot = Eigen::Matrix3d::Identity();  // R_cl
    Eigen::Vector3d cam_lidar_trans = Eigen::Vector3d::Zero();    // P_cl

    // ---- 时间偏移（秒） ----
    double lidar_time_offset = 0.0;
    double img_time_offset = 0.0;
    double imu_time_offset = 0.0;

    // ---- 输入/预处理 ----
    bool img_en = true;
    bool lidar_en = true;
    bool imu_en = true;
    int lidar_type = 8;  // 8 = GENERIC (已带逐点 curvature)
    int point_filter_num = 3;
    int scan_line = 32;
    double blind = 0.5;
    double filter_size_surf = 0.1;

    // ---- IMU ----
    int imu_int_frame = 30;
    double acc_cov = 0.5;
    double gyr_cov = 0.3;
    double b_acc_cov = 0.0001;
    double b_gyr_cov = 0.0001;

    // ---- LIO ----
    int lio_max_iterations = 5;
    double dept_err = 0.02;
    double beam_err = 0.05;
    double min_eigen_value = 0.0025;
    double voxel_size = 0.5;
    int max_layer = 2;
    int max_points_num = 50;

    // ---- VIO ----
    int vio_max_iterations = 5;
    double outlier_threshold = 1000.0;
    double img_point_cov = 100.0;
    int patch_size = 8;
    int patch_pyrimid_level = 3;
    bool normal_en = true;
    bool raycast_en = false;
    bool inverse_composition_en = false;
    bool exposure_estimate_en = true;
    double inv_expo_cov = 0.1;

    // ---- 发布 / 地图 ----
    bool dense_map_en = true;
    int pub_scan_num = 1;
    double blind_rgb_points = 0.0;
    bool pub_effect_point_en = false;
    bool pub_plane_en = false;
    bool map_sliding_en = false;
    double half_map_size = 100.0;
    double sliding_thresh = 8.0;

    // ---- 关键帧（对齐 FastLio2Frontend 语义） ----
    double keyframe_trans_thresh = 1.0;
    double keyframe_rot_thresh = 0.17;
};

}  // namespace rosiwit_slam
