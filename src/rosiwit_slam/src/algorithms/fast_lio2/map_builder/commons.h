#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M2D = Eigen::Matrix2d;
using V2D = Eigen::Vector2d;
using M2F = Eigen::Matrix2f;
using V2F = Eigen::Vector2f;
using M4D = Eigen::Matrix4d;
using V4D = Eigen::Vector4d;


template <typename T>
using Vec = std::vector<T>;


bool esti_plane(PointVec &points, const double &thresh, V4D &out);

float sq_dist(const PointType &p1, const PointType &p2);

struct Config
{
    int lidar_filter_num = 3;
    double lidar_min_range = 0.5;
    double lidar_max_range = 20.0;
    double scan_resolution = 0.15;
    double map_resolution = 0.3;

    double cube_len = 300;
    double det_range = 60;
    double move_thresh = 1.5;

    double na = 0.01;
    double ng = 0.01;
    double nba = 0.0001;
    double nbg = 0.0001;
    double acc_scale = 1.0; // 加速度单位转换: 传感器若以 g 为单位(如 Livox 内置 IMU), 设 9.81
    int imu_init_num = 20;
    int near_search_num = 5;
    int ieskf_max_iter = 5;
    // 有效平面点下限: 少于此数不做 EKF 更新(退化为纯预测)。
    // legacy(mapping_interface.cpp:2292) 是 50 —— 几何退化场景(窄区/视野受限)下
    // 只有几个有效点, 用它们做更新会让近秩亏的 H 把状态拖向不可观测方向(实测窄区漂移)。
    int min_effective_points = 50;
    // 退化检测: 有效平面点过少 / 占比过低 / 信息矩阵条件数过差时, 本帧跳过 LiDAR 更新
    // (只走 IMU 预测), 避免在开阔/走廊场景把位姿拉偏 (实测急转段厚墙)。
    int degenerate_min_effective_points = 300;
    double degenerate_ratio = 0.10;      // eff / 输入点数 下限
    double degenerate_eigen_ratio = 1e-4; // 平移块最小/最大特征值 下限
    bool gravity_align = true;
    double init_z = 0.0;  // IMU body 世界 z 初值; 设成 IMU 离地高度使 footprint z=0
    bool undistort_enable = true;
    bool esti_il = false;
    // 建图 odom 使用方式: none | gate | fuse (来自 mapping.odom_mode)
    bool odom_fuse = false;
    bool odom_gate = false;
    // 地面跟随: 不锁 z/roll/pitch, 只用世界系地面法向把 v.z 软约束到坡面分量.
    bool ground_follow_z = true;
    double ground_vz_sigma = 0.30;   // v.z 软约束标准差 (m/s)
    // IEKF 收敛早停: 不要用 0.2m/0.5deg 这种过松阈值, 否则一次更新就停.
    bool convergence_check = true;
    double pos_std_threshold = 0.01;   // m
    double rot_std_threshold = 0.05;   // deg
    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();

    double lidar_cov_inv = 1000.0;
    std::string robust_kernel = "tukey";
    double robust_scale = 0.5;
    double robust_floor = 0.1;
    bool dynamic_removal = true;
    int persist_thresh = 2;
    int persist_window = 20;
    double dynamic_residual_thresh = 1.0;
    // 在线自由空间雕刻 (L3a): 射线穿过已占据体素达阈值 -> 判该处物体已移除, 删除地图点
    bool free_space_carving = false;
    int carve_free_thresh = 3;    // 同一体素被"看到空"的次数阈值
    int carve_stride = 10;        // 射线行进时的扫描点抽样步长
};

/// 从 YAML 配置文件中解析 FastLIO2 公共配置字段。
/// 返回 true 表示成功; false 表示解析失败。
/// @note 此函数消除 FastLio2Algorithm 和 FastLio2Frontend 之间的配置解析重复。
bool parseFastLio2Config(const std::string& config_path, Config& out_cfg);

struct IMUData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    double time;
    IMUData() = default;
    IMUData(const V3D &a, const V3D &g, double &t) : acc(a), gyro(g), time(t) {}
};

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double offset;
    V3D acc;
    V3D gyro;
    V3D vel;
    V3D trans;
    M3D rot;
    Pose() = default;
    Pose(double t, const V3D &a, const V3D &g, const V3D &v, const V3D &p, const M3D &r) : offset(t), acc(a), gyro(g), vel(v), trans(p), rot(r) {}
};

struct SyncPackage
{
    Vec<IMUData> imus;
    CloudType::Ptr cloud;
    double cloud_start_time = 0.0;
    double cloud_end_time = 0.0;
};