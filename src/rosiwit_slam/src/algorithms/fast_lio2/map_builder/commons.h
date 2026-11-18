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
    // 陀螺尺度因子: 校准 IMU 角速度标度误差 (实测本车万集 IMU 偏航角速度低估 3.5%,
    // 即实测/真值≈0.965 -> gyro_scale=1.036)。大角度累计旋转(掉头/绕圈)时航向误差
    // 随累计转角逐帧放大, 是窄区"转过就发散"的关键因素。1.0 = 不校准。
    double gyro_scale = 1.0;
    // 状态速度物理上限(m/s): 坏配准把速度打爆后的最后一道刹车(IEKF 内部限幅)。
    // 清洁机器人实际 ~1m/s, 默认 3.0 留 3 倍余量。<=0 关闭。
    double max_state_velocity = 3.0;
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
    double degenerate_eigen_ratio = 0.02; // 信息矩阵 λmin/λmax 低于此值的方向按退化处理
    /// 弱方向自适应降权增益 (0 = 关闭, 只检测):
    ///   gain_k = clamp((r_k/thr)^0.25, degenerate_gain, 1), r_k=λ_k/λmax, thr=degenerate_eigen_ratio
    ///   = 0 时不做任何处理; (0,1] 时按下限降权, 该方向的信息量按 gain_k 保留.
    /// 说明: 没有"删除/置零"路径, 该方向永远保留至少 degenerate_gain 的信息,
    ///       避免沿走廊轴完全失去约束而自由漂移.
    double degenerate_gain = 0.0;
    bool gravity_align = true;
    double init_z = 0.0;  // IMU body 世界 z 初值; 设成 IMU 离地高度使 footprint z=0
    bool undistort_enable = true;
    bool esti_il = false;
    // 建图 odom 使用方式: none | gate | fuse (来自 mapping.odom_mode)
    bool odom_fuse = false;
    bool odom_gate = false;

    // ---- 轮速 odom 量测 (odom_mode=fuse): 每帧更新, 和 IMU 一样 ----
    // 用"上一帧到本帧"的位姿增量: Δp_ekf ≡ Δp_odom, Δyaw_ekf ≡ Δyaw_odom.
    // 不用 twist (位姿微分噪声), 不用窗口 (窗口会把 10Hz 的约束降成 1Hz).
    double odom_sigma_pos = 0.02;        // m, 每帧位移噪声
    double odom_sigma_pos_p_floor = 0.05;  // m, 位置协方差下限 (否则增益只有 1e-4)
    double odom_sigma_yaw = 0.002;       // rad, 每帧转角噪声 (~0.11deg)
    double odom_sigma_yaw_p_floor = 0.01;  // rad, 姿态协方差下限

    // ---- 轮速标定 (尺度, 默认 1.0 = 不改动) ----
    double odom_calib_yaw_scale = 1.0;      // 陀螺净转角 / 轮速净转角
    double odom_calib_dist_scale = 1.0;     // 已知长度 / 该段轮速位移

    // 地面跟随: 不锁 z/roll/pitch, 只用世界系地面法向把 v.z 软约束到坡面分量.
    bool ground_follow_z = true;
    // 轮速一致性门控 (LIO 位移远超轮速时暂停 LiDAR). 实测对窄道错层无效,
    // 默认关闭; 保留给'轮速可信但 LIO 会滑'的场合.
    bool odom_consistency_gate = false;
    // 轮速位置硬钳位: 0.5s 窗口内 LIO 位移 > 1.5x轮速+0.3m 时把位置钳回,
    // 并同步压速度. 实测窄道 7s 内滑 22m, 暂停 LiDAR 无效, 必须钳位置.
    // 注: 旧的 odom_position_clamp / odom_scale_xy 已删除 —— 硬钳位实测会把正常段
    // 也钳弯 (300-435m 偏差 16.9->22.2), 现在的 odom_win_pos_* 窗口位移约束是它的替代.
    double ground_vz_sigma = 0.30;   // v.z 软约束标准差 (m/s)
    // [新增] 平地假设的 v.z 软约束 (m/s), 0=关. 每帧把 v.z 拉向 0, 与地面是否
    // 有效无关; 用于长航时 z 通道无绝对观测时的漂移抑制 (现场基本平地).
    double flat_vz_sigma = 0.0;
    // z 的绝对软保持 (m, 0=关): 把 z 轻拉向初始值 0. 现场大多平地, 用它防
    // 长时间漂移; sigma 给大(如 1.0)时坡道几乎不受影响 (vz 约束权重高两个量级).
    double ground_z_hold_sigma = 0.0;
    // 地面 z 约束改成"锚定平面"(历史平面参数): 平面在锚点时刻冻结, 本帧 z 目标由
    // 旧平面 + 传感器离地高度推出. 若用本帧现拟合的平面, 点云与平面随位姿一起平移,
    // 残差恒等于标定误差 (实测全程 -0.29..-0.59m), 约束退化成一个只往下推的棘轮.
    double ground_anchor_dt = 1.0;    // 重锚间隔 (s), <=0 表示只在首次/门控时锚定
    double ground_drift_max = 0.15;   // 平面一致性门控 (m): 偏离超过即判为坡度/地形变化
    // 加计零偏限幅 (m/s^2). MEMS 实测零偏在 0.02~0.1 量级; 放到 1.0 会让它变成
    // 误差吸收器 (实测 450-550s |ba| 顶到 1.0, 同时速度虚报 1.5~1.9 m/s).
    // 默认 1.0 = 保持原有行为, 收紧需显式配置.
    double accel_bias_max = 1.0;
    // 沿车头速度标量约束 (m/s, 0=关): v·ĥ = 轮速里程/Δt.
    // 只用 odom 的里程, 不用它的 yaw. 针对长走廊沿轴无观测的滑行.
    double odom_speed_sigma = 0.0;
    // 沿"行进方向"的速度标量约束 (m/s, 0=关). 行进方向由 LIO 自身位移的低通给出,
    // 不用车体轴(安装外参未标定), 也不用轮速 yaw; 只取轮速的标量速度(里程可信).
    double odom_travel_sigma = 0.0;
    // 入图距离限制 (m, 0=不限制). 自转/低速时只把近处点入图, 避免远端点被角度
    // 误差扫成扇形 (实测原地自转 |w|~60deg/s 时 60m 外扫出大片拖影).
    double map_insert_max_range = 0.0;        // 正常行驶时的入图距离上限 (0=不限)
    double map_insert_spin_range = 0.0;       // 自转/低速时的入图距离上限 (0=同 max)
    double map_insert_spin_speed = 0.20;      // m/s, 低于此视为"低速"
    double map_insert_spin_yaw_rate = 15.0;   // deg/s, 高于此视为"自转"
    // 静止检测 (ZUPT): 轮速速度低于阈值连续 zupt_frames 帧后, 约束水平速度=0.
    double zupt_speed_thresh = 0.0;   // m/s, 0=关
    int    zupt_frames = 5;
    double zupt_sigma = 0.03;         // m/s
    // 重力方向倾斜约束 (rad, 0=关). 用加速度计低通后的方向约束 roll/pitch;
    // 地面法向对姿态是自指的 (点云随位姿一起转), 无法提供绝对参考.
    double gravity_tilt_sigma = 0.0;
    double gravity_tilt_tau = 1.0;      // s, 加速度低通时间常数
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