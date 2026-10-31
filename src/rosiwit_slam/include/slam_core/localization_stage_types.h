// include/slam_core/localization_stage_types.h
// 定位流水线各阶段共享的数据结构 / 配置 / 小工具.
// 设计目标与建图侧一致: 每个阶段(运动先验/局部地图/配准/全局重定位/健康监测)
// 都是可插拔策略, 由配置选择, 便于对同一阶段的不同算法做横向性能验证.
#pragma once
#include <string>
#include <vector>
#include <memory>
#include <cmath>
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
    // seed 帧防漂门限: 种子帧的 ICP 结果相对种子位姿的允许漂移.
    // 超过就判 seed 失败, 保持 INIT, 而不是"滑过去当成定位成功".
    // (2026-10-25: 之前硬编码 1.0m/15°, 实测拉偏 1m 后 ICP 能滑 0.84m
    //  并以 fitness 0.96 "锁定"在错位姿上 -> 假定位.)
    double seed_max_drift = 1.0;       // m
    double seed_max_yaw_deg = 15.0;    // deg
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
    double fpfh_normal_radius = 2.0;  // 法向估计半径(m)
    double fpfh_feature_radius = 8.0; // FPFH 特征半径(m)
    int    fpfh_max_iter = 50000;     // RANSAC 最大迭代
    double fpfh_inlier_fraction = 0.25;
    double fpfh_similarity = 0.9;     // 预拒绝相似度阈值
};

// ==================== 位姿融合 (pose_fusion) ====================
// 把"配准结果(观测)"变成"位姿(状态)"的那一步. direct=直接采用(旧行为),
// eskf=IMU/odom 推进 + 带协方差观测做增益修正 (单帧修正有界, 防止 1m 级跳变).
struct PoseFusionConfig {
    double gain_trans = 0.6;              // 位置增益上限 (0~1)
    double gain_rot = 0.8;                // 旋转增益上限
    double max_correction_trans = 0.15;   // 单帧最大平移修正 (m)
    double max_correction_rot_deg = 3.0;  // 单帧最大旋转修正 (deg)
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
    // 退化方向部分更新: 用 JtJ 特征分解把修正量按可观测度缩放.
    // 弱观测方向(eig < ratio_soft*eig_max)增益趋 0 => 只信先验, 不再让
    // "沿长墙滑动"这类退化解整体生效 (旋转时对应点集中在单面墙的典型失效).
    bool   degeneracy_partial_update = false;
    double degeneracy_ratio_soft = 0.05;    // 软门限: 特征值比低于该值逐步忽略
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
    int    persist_hits = 1;         // 体素需被观测到的最少次数才并入(>1 过滤瞬态, 长期用)
    int    autosave_frames = 0;      // 每 N 帧自动落盘叠加层 (0=仅析构时)
    int    refresh_frames = 0;       // 每 N 帧用叠加层刷新全局重定位器 (0=关闭)
    int    decay_frames = 0;         // 叠加层体素超过该帧数未再观测则移除 (0=不衰减)
    bool   override_base = false;    // 叠加层压制同位置的原图点(处理"结构被移除/被替换")
    double delta_min = 0.3;          // 只沉淀"离原图 > 该距离(m)"的新结构(变化增量)
    int    carve_stride = 20;        // 自由空间雕刻时扫描点抽样步长
    // ---- L13 位姿图维护 (可选) ----
    bool   pose_graph = false;       // true: 变化关键帧缓冲后用 GTSAM 位姿图平滑再写入
    int    pg_min_keyframes = 3;     // 触发一次位姿图优化的最少关键帧数
    // 叠加层容量上限 (体素). 超过后整体重置并告警, 防止长期运行无界增长.
    int    max_voxels = 400000;
};

} // namespace rosiwit_slam
