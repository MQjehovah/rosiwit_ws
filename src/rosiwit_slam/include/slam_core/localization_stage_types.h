// include/slam_core/localization_stage_types.h
// 定位流水线各阶段共享的数据结构 / 配置 / 小工具.
// 设计目标与建图侧一致: 每个阶段(运动先验/局部地图/配准/全局重定位/健康监测)
// 都是可插拔策略, 由配置选择, 便于对同一阶段的不同算法做横向性能验证.
#pragma once
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <cstdint>
#include <Eigen/Eigen>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

// ==================== 鲁棒核 ====================
enum class RobustKernel { NONE, HUBER, CAUCHY, TUKEY, GNC };
inline double robustWeight(RobustKernel k, double residual, double delta) {
    const double ar = std::fabs(residual);
    if (delta <= 1e-12) return 1.0;
    switch (k) {
        case RobustKernel::HUBER:  return ar <= delta ? 1.0 : delta / ar;
        case RobustKernel::CAUCHY: { const double t = ar / delta; return 1.0 / (1.0 + t * t); }
        case RobustKernel::TUKEY:  { const double t = ar / delta; return t <= 1.0 ? (1.0 - t * t) * (1.0 - t * t) : 0.0; }
        case RobustKernel::NONE:
        default: return 1.0;
    }
}
// GNC (渐进非凸, Geman-McClure 对偶): mu 由大到小, 早期近似 L2(凸)、后期逼近非凸.
// 高外点比(人/叉车多)下比固定鲁棒核更接近全局最优.
inline double gncWeight(double residual, double delta, double mu) {
    if (delta <= 1e-12) return 1.0;
    const double t = (residual / delta) * (residual / delta);
    const double s = mu / (mu + t);
    return s * s;
}

// ==================== SE(3) 位姿小工具 ====================
// poseMul(a, b) = 先 b 后 a (a ∘ b)
inline PoseStamped poseMul(const PoseStamped& a, const PoseStamped& b) {
    PoseStamped c;
    c.time = b.time;
    c.rot = a.rot * b.rot;
    c.trans = a.rot * b.trans + a.trans;
    c.vel = a.rot * b.vel;
    return c;
}
inline PoseStamped poseInv(const PoseStamped& p) {
    PoseStamped q;
    q.time = p.time;
    q.rot = p.rot.transpose();
    q.trans = -q.rot * p.trans;
    q.vel = q.rot * p.vel;
    return q;
}
inline Eigen::Matrix4d poseToMatrix(const PoseStamped& p) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = p.rot;
    T.block<3,1>(0,3) = p.trans;
    return T;
}
inline PoseStamped matrixToPose(const Eigen::Matrix4d& T, double time = -1.0) {
    PoseStamped p;
    p.time = time;
    p.rot = T.block<3,3>(0,0);
    p.trans = T.block<3,1>(0,3);
    return p;
}

// ==================== 配准结果 ====================
struct MatchResult {
    bool   converged = false;      // 求解器收敛
    PoseStamped pose;              // 配准得到的全局位姿 (rot/trans 为 world<-body)
    double fitness = 0.0;          // 覆盖率/内点率 [0,1] (跨配准器统一口径)
    double residual = -1.0;        // 平均点到面残差(m), <0=未计算
    double rmse = -1.0;            // 平均最近邻距离(m)
    int    correspondences = 0;    // 有效对应数
    int    inliers = 0;            // 内点数
    int    visible = 0;            // 匹配前可见点数(降采样后, 未做动态滤除); 0=配准器未填
    // 诚实覆盖率: 分母用**未做动态剔除**的全部降采样点, 阈值同 inlier_dist。
    // 动态剔除会让"位姿自己偏了的结构"在评分前消失(2026-11-22 实机: 删后 152 点
    // fitness 0.92, 不删 168 点只有 0.47) —— 判丢/锚点应该看这个数。<0=未计算。
    double fitness_all = -1.0;
    int    iterations = 0;         // 实际迭代次数
    bool   degenerate = false;     // 是否检测到退化方向
    int    degenerate_dirs = 0;    // 被门控的退化方向数
    double smallest_eig_ratio = 1.0; // 最小/最大特征值比 (可观测性)
    // JtJ 6x6 Hessian 的特征分解 (order: [rot(3), trans(3)]): 供上层做
    // 退化方向的部分更新 (弱观测方向保留先验). 小配准器可能不填.
    bool   has_hessian = false;
    Eigen::Matrix<double, 6, 1> hess_eigvals = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> hess_eigvecs = Eigen::Matrix<double, 6, 6>::Identity();
    // 位姿协方差 (order: [rot(3), trans(3)]); 默认大不确定度
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e3;
};

// ==================== 配准器配置 ====================
struct ScanMatcherConfig {
    double voxel_resolution = 0.3;          // 配准前降采样体素
    int    max_iterations = 32;             // 最大迭代
    double max_correspondence_dist = 2.0;   // 初始最近邻门限(m)
    double transformation_epsilon = 1e-6;   // 收敛阈值
    // 质量评估
    double inlier_dist = 0.3;               // 内点最近邻阈值(m)

    // 退化输入守卫: 降采样/动态滤除后点数低于该值直接判配准失败。
    // PCL 1.8 GICP 的 computeCovariances 要求点数 >= k_correspondences(20),
    // 实测 16 点输入会**直接段错误崩掉整个节点**(2026-10-17 20:06 实机死亡,
    // 单测 test_gicp_matcher_guard 复现 exit 139)。
    int    min_match_points = 50;

    // ---- Point-to-Plane (pp_icp) 专用 ----
    bool   use_corr_decay = true;           // 迭代内对应点半径衰减
    double corr_dist_init = 4.0;            // 初始关联半径(m)
    double corr_dist_min = 1.0;             // 末次关联半径下限(m)
    bool   use_huber = true;                // 兼容旧字段 (若配置 robust_kernel 则以其为准)
    double huber_delta = 0.25;              // 鲁棒核尺度(m)
    RobustKernel robust_kernel = RobustKernel::HUBER;  // NONE|HUBER|CAUCHY|TUKEY|GNC
    double gnc_mu0 = 1e4;      // GNC 初始 mu (大)
    double gnc_decay = 0.6;    // GNC 每迭代衰减 (mu *= decay)
    bool   use_edge_feature = false;        // 点到线边缘特征
    double edge_line_ratio = 0.12;          // 邻域协方差 λ1/λ2 线判据
    double edge_valid_thr = 0.10;           // 点到拟合直线最大垂距(m)
    double edge_min_extent = 0.20;          // 直线延伸长度下限(m)
    double edge_vertical_cos = 0.866;       // 仅保留竖直边缘(cos30°)

    // 退化门控
    bool   degeneracy_gate = true;          // JtJ 特征值退化方向门控
    double degeneracy_eig_ratio = 1e-3;     // 退化判据: eig < ratio*max
    int    min_correspondences = 20;        // 低于该有效对应数判失败

    // 动态/变化物剔除: 仅保留"贴近静态地图"的点 (人来人往/叉车/每天变的堆货
    // 未入图即被剔除, 定位以静态结构为准); 由 maintainer 沉淀的持久变化会进入地图
    bool   dynamic_filter = false;
    double dynamic_dist = 1.0;              // 与地图最近距离阈值(m), 超过视为动态

    // 参与匹配的点高度带裁剪 (map 系, 相对地面).
    // 地图没有地面点 -> 扫描地面点永远配不上, 只会撑大 coverage 分母 (实测 ~13%).
    // 默认只裁地面: [match_z_min, match_z_max) = [0.15, 10000), 天花板不裁.
    // 2026-11-23 现场要求: 地面在地图系 z≈0, 下界 0.15m 足够去掉地面/贴地点.
    bool   match_z_filter = true;
    double match_z_min = 0.15;              // 下界(m, 相对地面)
    double match_z_max = 10000.0;           // 上界(m), 默认不裁天花板
    // 裁剪坐标系: 由安装外参 (URDF/TF/参数) 注入 lidar->base_footprint 的**旋转+平移**.
    // 地面在 base_footprint 系 z=0, 因此裁剪**与位姿 z 无关** —— 重定位/位姿 z 漂了也不会裁错.
    // valid=false 时才退回用 guess 投到 map 系 (不推荐, 依赖位姿 z).
    Eigen::Matrix3d crop_r = Eigen::Matrix3d::Identity();
    bool   crop_r_valid = false;
    // 雷达原点在 base_footprint 系的位置. **平移项不能省**: 点的离地高度 = (R·p + t).z,
    // 只转不平移时裁剪面落在"雷达安装高度 + 阈值" —— 本机 0.844+0.15 = 0.99m, 把墙裙/
    // 货架腿等近场约束点也裁掉了 (2026-11-23 回放对账: 实际裁掉 16~19% 而非预期的 ~13%).
    Eigen::Vector3d crop_t = Eigen::Vector3d::Zero();
    // 退化扫描打折: 裁剪后参与配准的点数低于该值时, fitness 按点数比例线性下调.
    // 被近处物体包围/遮挡时 (实车: 雷达四面 0.6~3.7m 全是遮挡, 降采样后仅 240 点)
    // "覆盖率"没有统计意义 —— 任何位姿都能拿 1.000, 节点据此判"已定位"并把位姿冻住.
    // 正常扫描实测 4400~5100 点. 0 = 关闭.
    int    min_coverage_points = 800;
    // 退化扫描打折的豁免线: 绝对内点数够多时, 点数少也不打折。
    // 小房间(雷达四面近墙)降采样后可能只有 379 点, 但 374 个内点、覆盖率 0.99 是可信的;
    // 而 2026-11-23 那次"240 点拿 1.000"的内点只有一百多, 仍应打折。
    int    min_inliers_full_score = 150;

    // 2.5D 平面 NDT (matcher: ndt2d): 在 (x,y,yaw) 求解, z/roll/pitch 取自初值
    double ndt2d_cell = 0.5;                // 2D 高斯格尺寸(m), 通常比 3D 体素粗
    double ndt2d_z_min = -1.0;              // 参与 2D 投影的 z 下界(m)
    double ndt2d_z_max = 1.0;               // z 上界(m)
};

// ==================== 局部地图配置 ====================
struct LocalMapConfig {
    double voxel_resolution = 0.3;   // 局部地图降采样
    double radius = 60.0;            // radius_submap: 邻域半径(m)
    double cell_size = 10.0;         // radius_submap: 体素哈希单元边长(m)
    int    max_keyframes = 15;       // keyframe_submap: 最近关键帧数上限
    double cache_move = 0.5;         // 位姿移动小于该值时复用上一帧局部地图(m)
    // ---- 子图窗口锚 (2026-11-25 实验后默认关闭) ----
    // 想法: 只有"已验证帧"才把窗口中心吸附到匹配位姿, 弱帧保持旧锚点, 防止弱匹配
    // 把子图窗口带偏. 同包对照(见 docs/LOCALIZATION_ANCHOR_2026-11-25.md):
    //   关闭 98.5~99.8%; 打开 lag=1m 99.4%; 打开 lag=8m+可信门限 85.6%(单次).
    // 没有证据显示更好, 而窗口滞后本身有"把匹配往后拽"的风险 -> 默认关, 仅留实验.
    bool   anchor_hold = false;          // 总开关; false = 按当前估计建图(旧行为)
    double anchor_max_lag = 1.0;         // 窗口中心允许的最大滞后(m)
    double anchor_min_eig_ratio = 1e-3;  // 视为"可观测"的最小 λmin/λmax
    double anchor_min_fitness = 0.0;     // 视为"可信帧"的最低 fitness (0=只看创新量)
};

// ==================== 关键帧 (keyframe_submap 输入) ====================
struct LocalKeyframe {
    PoseStamped pose;          // body -> world
    CloudType::Ptr cloud;      // body 系点云
};

// ==================== 运动先验配置 ====================
struct MotionPriorConfig {
    double max_dt = 0.2;             // IMU 积分最大间隔(s)
    double max_vel = 10.0;           // 常速外推速度上限(m/s)
    bool   use_odom = false;         // 融合轮式里程计(默认关: 洗地机打滑不可信)
    // 机体系 up 方向 (IMU 系): 由安装姿态参数给出 (TF/URDF/启动参数解析);
    // false 时由加速度静态标定自动估计. 偏航角速度统一 = ω·up_body (slam_types.h: imuYawRate).
    V3D    up_body = V3D::UnitZ();
    bool   up_body_valid = false;
    // 杆臂 (传感器系, slam_types.h: leverArmSensorFrame): 原地绕 base 自转时
    // 雷达绕 base 划弧, 位置积分需要 ω×ρ 轨道项; false = 未注入, 保持旧行为.
    V3D    lever_arm = V3D::Zero();
    bool   lever_valid = false;
    M3D    r_base_lidar = M3D::Identity();  // 体系速度(base系) -> 传感器系
    double slip_yaw_thresh = 0.3;    // 打滑判据: |odom_yaw_rate - imu_yaw_rate| 与它比较, 决定 odom 平移的权重
    double max_odom_speed = 3.0;     // odom 平移速度物理上限(m/s), 超出 = 打滑/毛刺样本, 降权
    // ---- imu_ekf 先验 (误差状态 EKF, 状态=[vx,vy] 体系速度) ----
    double ekf_q_v = 1.0;            // 过程噪声: 速度随机游走 (m/s)²/s
    double ekf_r_odom = 0.15;        // odom 速度观测噪声 (m/s)
    double ekf_r_match = 0.8;        // 激光位移速度观测噪声 (m/s, 低频弱观测)
    double ekf_chi2 = 5.99;          // 卡方门控阈值 (2 自由度 95%); 0=关
    // ---- IMU 零偏在线估计 (L11) ----
    int    imu_calib_samples = 200;  // 静止标定窗口样本数
    double static_gyro_std = 0.02;   // 静止判据: gyro 标准差(rad/s)
    double static_accel_std = 0.15;  // 静止判据: ‖acc‖ 与 g 的偏差 / acc 标准差(m/s^2)
    double gyro_bias_gain = 1.0;     // 陀螺零偏在线增益
    double gyro_bias_max = 0.5;      // 陀螺零偏上限(rad/s)
    double acc_bias_gain = 0.5;      // 加速度零偏在线增益
    double acc_bias_max = 5.0;       // 加速度零偏上限(m/s^2)
    bool   use_gravity = true;       // 用重力方向做 roll/pitch 校正
    double gravity = 9.81;
    double gravity_gain = 0.05;      // 重力校正融合增益
};

// ==================== 全局重定位配置 ====================
struct GlobalLocalizeConfig {
    double min_score = 0.5;          // 命中最低分
    double reloc_accept_score = 0.85; // 冷启动接受线: 低于此值宁可保持 LOST 也不给假位姿
                                       // (2026-10-17: 曾用 min_score*0.9=0.45, 0.68 的弱
                                       //  重定位被反复接受 -> 僵尸循环: 接受->10s判丢->再接受)
    int    top_k = 5;                // 全局重定位多候选数
    int    retry_interval = 30;      // 丢失后重定位尝试间隔(帧, 节流)
    double voxel_resolution = 0.3;   // 精配准降采样
    double icp_max_corr_dist = 2.0;
    double icp_fitness_thresh = 1.0;
    int    icp_max_iter = 32;
    // 精配准局部子图半径(m): 全局重定位只需要候选附近的结构.
    // 全图上给每个候选跑 GICP 是重定位 20~45 s 的主要来源; 0 = 不裁剪(旧行为).
    double reloc_submap_radius = 30.0;
    // seed 帧防漂门限: 种子帧的 ICP 结果相对种子位姿的允许漂移.
    // 超过就判 seed 失败, 保持 INIT, 而不是"滑过去当成定位成功".
    // (2026-10-25: 之前硬编码 1.0m/15°, 实测拉偏 1m 后 ICP 能滑 0.84m
    //  并以 fitness 0.96 "锁定"在错位姿上 -> 假定位.)
    double seed_max_drift = 1.0;       // m
    double seed_max_yaw_deg = 15.0;    // deg
    // 种子"邻域最优性"校验: 在 ±trans / ±yaw 的候选里做同一帧匹配, 若有明显更优解,
    // 说明种子落在局部最优(实车: 偏 0.5m 也能拿 0.9+ 分), 宁可拒绝等人工也不锁死。
    bool   seed_neighbor_check = true;
    double seed_check_trans_m = 0.5;
    double seed_check_yaw_deg = 5.0;
    double seed_check_margin = 0.03;
    // ScanContext
    int    sc_num_rings = 20;
    int    sc_num_sectors = 60;
    double sc_max_radius = 80.0;
    double sc_min_radius = 2.0;
    double sc_dist_thresh = 0.4;
    double sc_candidate_ratio = 0.3;
    // BBS (branch-and-bound) 分支定界 2D 全局搜索
    // 安装角 (base<-lidar 的 rpy, 度)。BBS 的 2D 搜索要求"水平安装":
    // 先把扫描与地图都转到 base(车体)系再投影, 最后把结果换回雷达系。0 = 不转换。
    // **首选**: base<-lidar 旋转矩阵(行优先9元素), 由 ROS 层从 TF 直接注入,
    // 彻底避免欧拉角顺序歧义(URDF rpy / Eigen eulerAngles / 打印顺序 三套约定)。
    std::array<double, 9> bbs_mount_r{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
    double bbs_mount_roll_deg = 0.0;
    double bbs_mount_pitch_deg = 0.0;
    double bbs_mount_yaw_deg = 0.0;
    double bbs_z_min = -1.0;          // 参与 2D 投影的 z 下界(m)
    double bbs_z_max = 1.0;           // z 上界(m)
    int    bbs_theta_bins = 60;       // 航向离散数
    int    bbs_max_levels = 6;        // 金字塔最大层数
    double bbs_min_points_per_cell = 5.0;  // 单格命中密度归一化上限
    // FPFH-RANSAC 6DoF 全局重定位
    // 注意: FPFH 自己用 fpfh_voxel_resolution, 不要复用 BBS 的 voxel_resolution(0.3);
    // 8m 特征半径在 6M 点地图上会把邻域放大到几万点, aarch64 实测数小时跑不完.
    double fpfh_normal_radius = 1.0;  // 法向估计半径(m)
    double fpfh_feature_radius = 3.0; // FPFH 特征半径(m)
    double fpfh_voxel_resolution = 1.0; // FPFH 专用降采样(m); <=0 时回退 voxel_resolution
    int    fpfh_threads = 2;          // Normal/FPFH OMP 线程数 (0 = 全部核心)
    int    fpfh_max_iter = 5000;      // RANSAC 最大迭代 (50000 在 25 万点地图上单场景 >10 min;
                                       // 有 .scdb 时优先用 scan_context, FPFH 只做无库兜底)
    double fpfh_inlier_fraction = 0.25;
    // 全局重定位单次预算(ms): 超了就带着已有候选返回, 不能无限跑
    int    reloc_budget_ms = 5000;
    double fpfh_similarity = 0.9;     // 预拒绝相似度阈值
    // RANSAC 随机子集种子: 0 = 用时钟(生产默认, 每帧采样不同 -> 多候选);
    // 非 0 = 固定种子, 用于可复现的回归测试/A-B (采样相同 -> 结果确定).
    uint32_t random_seed = 0;
};

// ==================== 位姿融合 (pose_fusion) ====================
// 把"配准结果(观测)"变成"位姿(状态)"的那一步. direct=直接采用(旧行为),
// eskf=IMU/odom 推进 + 带协方差观测做增益修正 (单帧修正有界, 防止 1m 级跳变).
struct PoseFusionConfig {
    double gain_trans = 0.6;              // 位置增益上限 (0~1)
    double gain_rot = 0.8;                // 旋转增益上限
    double max_correction_trans = 0.15;   // 单帧最大平移修正 (m)
    double max_correction_rot_deg = 3.0;  // 单帧最大旋转修正 (deg)
    double gravity = 9.81;                // 重力加速度 (m/s^2), 用于 up_body 自动估计
    double gravity_tol = 0.5;             // 加速度模长与 g 的容差 (m/s^2)
    // 机体系 up 方向 (IMU 系): 由安装外参参数给出 (URDF/TF/启动参数注入);
    // false 时用加速度自动估计. 偏航角速度统一 = ω·up_body (slam_types.h: imuYawRate).
    V3D    up_body = V3D::UnitZ();
    bool   up_body_valid = false;
    // 杆臂 (传感器系, slam_types.h: leverArmSensorFrame): 位置积分的 ω×ρ 轨道项,
    // 与 EkfMotionPrior 同源; false = 未注入, 保持旧行为.
    V3D    lever_arm = V3D::Zero();
    bool   lever_valid = false;
    M3D    r_base_lidar = M3D::Identity();  // 体系速度(base系) -> 传感器系
    double trans_cov_floor = 0.02;        // 匹配平移协方差下限 (m^2)
    double rot_cov_floor = 0.001;         // 匹配旋转协方差下限 (rad^2)
    double process_trans = 0.05;          // 每帧位置过程噪声 (m^2)
    double process_rot = 0.01;            // 每帧旋转过程噪声 (rad^2)
    double chi2_gate = 20.0;              // 新息卡方门控 (6DoF, 99.9%≈22.5); 0=关
    double max_odom_speed = 3.0;          // odom 速度物理上限 (m/s)
    bool   reject_in_place_trans = true;  // 原地旋转时丢弃匹配的平移分量
    double in_place_yaw_step_deg = 2.0;   // 判定"原地旋转"的单帧航向变化阈值 (deg)
    // ---- odom 打滑门控 (打滑是常态: odom 不删, 只连续降权) ----
    double slip_ratio_lo = 0.5;           // |d_pose|/|d_odom| 低于此开始降权
    double slip_ratio_hi = 2.0;           // 高于此开始降权
    double slip_yaw_thresh_deg = 20.0;    // odom 横摆率与陀螺横摆率差的阈值 (deg/s)
    double odom_weight_min = 0.05;        // 权重下限 (保底, 不归零)
    // ---- 相对观测的连续性保护 (LIO 复位/模式切换/丢帧) ----
    double rel_max_gap = 0.6;             // 两帧相对观测最大时间间隔 (s)
    double rel_max_step = 3.0;            // 单步相对位移上限 (m), 超过视为跳变
    double rel_chi2_gate = 16.0;          // 相对观测卡方门限 (3DoF, 99.9%≈16.3); 0=关
    // 激光位移速度观测噪声 (m/s): 相邻两次被接受匹配的位移 -> 体系速度.
    // odom_mode=none 时这是融合层**唯一**的速度来源 —— 没有它, 位置只能靠每帧限幅
    // 修正挪动 (0.15 m/帧 ≈ 0.75 m/s), 1~2 m/s 的推行必然落后 (2026-11-23 实车).
    // 有 odom 时 (50Hz, r=0.15) 它权重自然较低, 只做补充. 0 = 关闭.
    double match_vel_r = 0.3;
};

// ==================== 健康监测 ====================
enum class MonitorLevel { INIT = 0, CREDIBLE = 1, DEGRADED = 2, LOST = 3 };

struct MonitorInput {
    MatchResult match;
    PoseStamped pose;
    double time = -1.0;
    double nees = -1.0;   // 归一化估计误差平方(Mahalanobis), <0=无效
};

struct MonitorConfig {
    bool   enabled = true;
    double credible_fitness = 0.75;  // >= 判 CREDIBLE
    double lost_fitness = 0.5;       // < 判 DEGRADED, 连续 patience 次判 LOST
    // 状态机唯一使用的两个连续帧计数器：
    //   INIT -> LOCALIZED 需要连续 localized_patience 帧健康；
    //   INIT/LOCALIZED -> LOST 需要连续 lost_patience 帧不健康。
    // 原 mediocre_patience / lost_patience_low / coast_patience 已合并到 lost_patience。
    int    localized_patience = 3;   // 连续健康帧数 -> LOCALIZED
    // 种子观察窗: 刚进入 LOCALIZED 的前 N 帧, 仍要求 fitness >= reloc_accept_score
    // (而不是只要求 >= lost_fitness)。撑不住就退回 INIT —— 专治
    // "0.80~0.84 的局部自洽假锁": 假锁撑不过观察窗, 真锁不会掉。0=关闭。
    int    scrutiny_frames = 0;      // 默认关闭: 本站稳态 fitness 0.80~0.87, 用 0.85 复核会抖动
                                     // (实机 2026-11-21); 设备上如需可设 ~10 (~1s @10Hz)
    int    lost_patience = 100;      // 连续不健康帧数 -> LOST
    int    window_length = 20;       // windowed: 滑窗长度
    double window_std_k = 2.0;       // windowed: 均值±kσ 门限
    double max_trans_step = 1.0;     // 单帧位移跳变门限(m)
    double max_yaw_step_deg = 20.0;  // 单帧航向跳变门限(deg)
    double nees_threshold = 50.0;    // NEES 滑窗均值超此判不一致(DEGRADED)
};

// ==================== 配准接受判据 (s2m accept) ====================
struct AcceptConfig {
    // 0=仅覆盖率; 1=残差+多帧一致性
    int    mode = 1;
    double min_coverage = 0.5;       // 覆盖率门限
    double residual_margin = 0.10;   // 残差相对改善阈值
    int    consistent_frames = 2;    // 修正方向连续一致帧数
    bool   mahalanobis_gate = true;  // 用匹配协方差做 Mahalanobis 门控(抑制离群)
    double mahalanobis_chi2 = 16.81; // 6DoF 99% 卡方阈值
    // 协方差下限：GICP 的 cov 来自残差平方，容易把“正常但稍大的修正”
    // 误判成离群；这里给平移/旋转一个物理下限。
    double mahalanobis_trans_floor = 0.20;  // m
    double mahalanobis_rot_floor = 0.05;    // rad
    // 协方差 floor 的参考对应点数: 少于该值时按比例放宽门限
    int    mahalanobis_ref_points = 500;
    // 连续拒绝 (coast) 后马氏门限按 ×(1 + k·coast) 放宽. 预测一旦偏出门限, 即使匹配
    // 一直正确也永远被拒 -> 死锁 (2026-11-23 实车: 1.9m 创新量 d²=17.5 对门限 16.81,
    // 卡死到人工重锁). k=0.5: coast=1 过 1.5 倍, coast=4 过 3 倍. 0 = 关闭.
    double mahalanobis_coast_widen = 0.5;
    // 单帧位置创新量 (匹配位姿 - 预测位姿, m) 的绝对上限, 同样按 √(1 + k·coast) 放宽.
    // 协方差门限在点云充分时允许 ~1.9m, 挡不住两个相似结构之间 0.5~0.9m 的"中间解"
    // (实车推车: 把输出位姿拽歪, 来回跳). 0 = 关闭.
    double max_trans_innov = 0.6;
    // 退化方向部分更新: 用 JtJ 特征分解把修正量按可观测度缩放.
    // 弱观测方向(eig < ratio_soft*eig_max)增益趋 0 => 只信先验, 不再让
    // "沿长墙滑动"这类退化解整体生效 (旋转时对应点集中在单面墙的典型失效).
    bool   degeneracy_partial_update = false;
    double degeneracy_ratio_soft = 0.05;    // 软门限: 特征值比低于该值逐步忽略
    // ---- 冻结逃逸 (2026-11-25) ----
    // 现场/回放复现: 位姿偏出 max_trans_innov 之后, 匹配每帧都能稳定找到同一个正确
    // 位姿, 但单帧修正超标被永久拒收 -> 速度先验衰减 + odom 被判 slip -> 位姿冻在
    // 原地, 只剩重定位试探(帧率从 10Hz 掉到 1.4Hz), relocalizer=none 时永不恢复.
    // 这里只在"连续 N 帧匹配结果彼此一致"时放行一次, 把位姿放回去.
    // 不新增 fitness 门限(沿用 min_coverage), 更不看点数.
    bool   escape_enabled = true;
    int    escape_frames = 6;         // 需要连续一致的被拒帧数
    int    escape_min_coast = 8;      // 先连续拒绝这么多帧才允许逃逸(不和正常门控抢)
    double escape_consist_trans = 0.20;     // 相邻两帧匹配位姿允许差(m)
    double escape_consist_rot_deg = 5.0;    // 相邻两帧匹配姿态允许差(deg)
    double escape_max_spread = 0.45;        // 候选相对首个候选的最大离散(m)
    double escape_max_jump = 8.0;           // 单次逃逸的最大位移(m), 防跨图乱跳
    double escape_cooldown_s = 3.0;         // 两次逃逸最小间隔(s), 防两个解之间来回跳
};

// ==================== 地面/平面约束 ====================
// 用"地图地面模型"把位姿约束到地形: 平地锁 z/roll/pitch, 坡道自然跟随。
// 仅用实时扫描是恒等式(无约束), 必须相对地图地面推理。
struct PlanarConstraintConfig {
    std::string mode = "none";   // none | lock_zrp | ground_plane
    double z = 0.0;              // lock_zrp: 目标高度(m)
    double sensor_height = 0.0;  // 传感器离地高(m); <=0 用首帧自动标定
    double cell = 1.0;           // 地面模型网格单元(m)
    double ground_band = 0.3;    // 地面点带厚度(m, 取每格最低点附近为地面)
    double max_tilt_deg = 35.0;  // 地面法向与竖直夹角上限, 超过则本帧不约束(异常/跳变)
    double max_correction_deg = 15.0;  // scan_plane: 单帧允许修正的最大倾角(超过视为跳变, 跳过)
};

// ==================== 绝对约束融合 (L19) ====================
// GNSS/UWB/反光标/二维码等绝对观测, 以"门控互补融合"接入定位(防跳变).
struct AbsoluteConfig {
    bool   enabled = false;
    double window = 0.5;         // 观测有效窗口(s, 超过则忽略)
    double alpha = 0.5;          // 融合权重 (0~1; 有平滑器时由平滑器统一)
    double max_trans = 2.0;      // 创新门限: 平移超此丢弃(m)
    double max_yaw_deg = 30.0;   // 创新门限: 航向超此丢弃(deg)
};

// ==================== 地图维护 (map_maintainer) ====================
// 非破坏性: 冻结原始地图, 将"变化"以叠加层(overlay)形式记录, 配准目标=原图+叠加层.
struct MaintainerConfig {
    bool   enabled = true;
    double fitness_trigger = 0.5;    // fitness 低于此判为环境变化
    double min_keyframe_dist = 1.0;  // 叠加关键帧最小间距(m)
    int    min_confirm = 2;          // 连续确认帧数(抑制误检)
    double radius = 60.0;            // 叠加层按位姿取用半径(m)
    double voxel = 0.3;              // 叠加层体素(去重)
    double cell = 10.0;              // 叠加层空间分块边长(m)
    int    persist_hits = 2;         // 体素需被观测到的最少次数才并入(2026-11-25: 一律 view+2)
    int    refresh_frames = 0;       // 每 N 帧用叠加层刷新全局重定位器 (0=关闭)
    int    decay_frames = 0;         // 叠加层体素超过该帧数未再观测则移除 (0=不衰减)
    // ---- A 阶段: 体素可信度 w (方案 v3 §3) ----
    // w 是每个 overlay 体素的"高可信几何"权重: 被观测为占据 +8, 被自由空间雕刻 -8,
    // 0 时删除该体素。**参与决策**: 匹配目标按 target_w_min 过滤 (T2), 低 w 的
    // 动态/未确认结构不参与匹配。
    int    persistency_anchor = 128;
    // ---- w 的计票单位 (同一个体素在每个单位内最多 +1) ----
    // 0=frame(每帧) 1=view(每次重新进入视野) 2=session(每个任务会话) 3=day(每个自然日)
    // frame 与扫描频率耦合, 只用于 A/B; 2026-11-25 起一律 view: w ≈ "重新进入视野几次确认过".
    int    persistency_unit = 1;
    int    persistency_hit_gain = 1;        // 每个单位加多少
    int    persistency_miss_confirm = 3;    // 连续 free 证据帧数 -> 判定消失(去抖)
    int    persistency_miss_action = 0;     // 0=删除  1=按 persistency_hit_gain 递减
    int    persistency_session_hours = 12;  // 会话超时自动换新(防长期不重启)
    bool   shadow = true;            // true: w 只用于 persistency_cov 观测, 不影响定位/插入
    // 写图要求匹配残差小 (rmse = 平均最近邻距离, m): 位姿不确定时不落图。
    // 0 = 关闭该条。
    double max_rmse = 0.06;
    // 参与匹配的最低可信度: w < 此值的 overlay 点只当"虚影", 不进匹配目标。
    // 默认 = persistency_anchor(128): 只有被反复确认(≈16 个 view)的结构才参与定位。
    int    match_w_min = 128;
    bool   override_base = false;    // 叠加层压制同位置的原图点(处理"结构被移除/被替换")
    double delta_min = 0.3;          // 只沉淀"离原图 > 该距离(m)"的新结构(变化增量)
    int    novel_min_cluster = 3;    // 新结构最小簇(26 邻域内本帧新颖体素数, 含自身)
    int    carve_stride = 20;        // 自由空间雕刻时扫描点抽样步长
    int    base_voxel_w = 128;       // 统一工作图 Phase A: 底图体素初始 w (射线穿过可递减)
    int    target_w_min = 16;        // 匹配目标里 overlay 点的最低 w(0=不过滤):
                                     // 只让确认结构(固定物体)参与/压制, 动态与未确认忽略
                                     // (ELite 式4 的离散化; w 单位=hit_gain, 16=2 个计票单位)
    // ---- L13 位姿图维护 (可选) ----
    bool   pose_graph = false;       // true: 变化关键帧缓冲后用 GTSAM 位姿图平滑再写入
    int    pg_min_keyframes = 3;     // 触发一次位姿图优化的最少关键帧数
    // 叠加层容量上限 (体素). 超过后整体重置并告警, 防止长期运行无界增长.
    int    max_voxels = 400000;
    // ---- 插入门控 (规则 1~3) ----
    // LIO 相对观测与 S2M 的残差超过此值时不写入 (0=关闭该条).
    double max_insert_divergence = 0.2;
};

} // namespace rosiwit_slam
