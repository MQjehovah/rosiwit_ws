// include/slam_core/localization_stage_types.h
// 定位流水线各阶段共享的数据结构 / 配置 / 小工具.
// 设计目标与建图侧一致: 每个阶段(运动先验/局部地图/配准/全局重定位/健康监测)
// 都是可插拔策略, 由配置选择, 便于对同一阶段的不同算法做横向性能验证.
#pragma once
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

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
};

// ==================== 配准器配置 ====================
struct ScanMatcherConfig {
    double voxel_resolution = 0.3;          // 配准前降采样体素
    int    max_iterations = 32;             // 最大迭代
    double max_correspondence_dist = 2.0;   // 初始最近邻门限(m)
    double transformation_epsilon = 1e-6;   // 收敛阈值
    // 质量评估
    double inlier_dist = 0.3;               // 内点最近邻阈值(m)

    // ---- Point-to-Plane (pp_icp) 专用 ----
    bool   use_corr_decay = true;           // 迭代内对应点半径衰减
    double corr_dist_init = 4.0;            // 初始关联半径(m)
    double corr_dist_min = 1.0;             // 末次关联半径下限(m)
    bool   use_huber = true;                // Huber 鲁棒核
    double huber_delta = 0.25;              // 法向残差阈值(m)
    bool   use_edge_feature = false;        // 点到线边缘特征
    double edge_line_ratio = 0.12;          // 邻域协方差 λ1/λ2 线判据
    double edge_valid_thr = 0.10;           // 点到拟合直线最大垂距(m)
    double edge_min_extent = 0.20;          // 直线延伸长度下限(m)
    double edge_vertical_cos = 0.866;       // 仅保留竖直边缘(cos30°)

    // 退化门控
    bool   degeneracy_gate = true;          // JtJ 特征值退化方向门控
    double degeneracy_eig_ratio = 1e-3;     // 退化判据: eig < ratio*max
    int    min_correspondences = 20;        // 低于该有效对应数判失败
};

// ==================== 局部地图配置 ====================
struct LocalMapConfig {
    double voxel_resolution = 0.3;   // 局部地图降采样
    double radius = 60.0;            // radius_submap: 邻域半径(m)
    double cell_size = 10.0;         // radius_submap: 体素哈希单元边长(m)
};

// ==================== 运动先验配置 ====================
struct MotionPriorConfig {
    double max_dt = 0.2;             // IMU 积分最大间隔(s)
    double max_vel = 10.0;           // 常速外推速度上限(m/s)
};

// ==================== 全局重定位配置 ====================
struct GlobalLocalizeConfig {
    double min_score = 0.5;          // 命中最低分
    double voxel_resolution = 0.3;   // 精配准降采样
    double icp_max_corr_dist = 2.0;
    double icp_fitness_thresh = 1.0;
    int    icp_max_iter = 32;
    // ScanContext
    int    sc_num_rings = 20;
    int    sc_num_sectors = 60;
    double sc_max_radius = 80.0;
    double sc_min_radius = 2.0;
    double sc_dist_thresh = 0.4;
    double sc_candidate_ratio = 0.3;
    // BBS (branch-and-bound) 分支定界 2D 全局搜索
    double bbs_z_min = -1.0;          // 参与 2D 投影的 z 下界(m)
    double bbs_z_max = 1.0;           // z 上界(m)
    int    bbs_theta_bins = 60;       // 航向离散数
    int    bbs_max_levels = 6;        // 金字塔最大层数
    double bbs_min_points_per_cell = 5.0;  // 单格命中密度归一化上限
};

// ==================== 健康监测 ====================
enum class MonitorLevel { INIT = 0, CREDIBLE = 1, DEGRADED = 2, LOST = 3 };

struct MonitorInput {
    MatchResult match;
    PoseStamped pose;
    double time = -1.0;
};

struct MonitorConfig {
    bool   enabled = true;
    double credible_fitness = 0.75;  // >= 判 CREDIBLE
    double lost_fitness = 0.5;       // < 判 DEGRADED, 连续 patience 次判 LOST
    int    lost_patience = 3;        // 连续低 fitness 次数
    int    lost_patience_low = 8;    // fitness<lost_fitness 的连续次数
    int    window_length = 20;       // windowed: 滑窗长度
    double window_std_k = 2.0;       // windowed: 均值±kσ 门限
    double max_trans_step = 1.0;     // 单帧位移跳变门限(m)
    double max_yaw_step_deg = 20.0;  // 单帧航向跳变门限(deg)
};

// ==================== 配准接受判据 (s2m accept) ====================
struct AcceptConfig {
    // 0=仅覆盖率; 1=残差+多帧一致性
    int    mode = 1;
    double min_coverage = 0.5;       // 覆盖率门限
    double residual_margin = 0.10;   // 残差相对改善阈值
    int    consistent_frames = 2;    // 修正方向连续一致帧数
};

} // namespace rosiwit_slam
