#pragma once
#include <string>
#include <array>
#include <vector>
#include <mutex>
#include <unordered_map>
#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "slam_core/i_backend.h"

namespace rosiwit_slam {

struct KeyFrameNode {
    PoseStamped pose;
    std::string id;
    int64_t timestamp = 0;
    // 入图时冻结的相邻里程计测量, 不能每次 optimize 从当前(可能已修正)
    // 位姿重新推导, 否则上一次修正会变成新的观测.
    PoseStamped odom_relative;
    bool has_odom_relative = false;
};

struct RelPoseConstraint {
    std::string from_kf;
    std::string to_kf;
    PoseStamped relative_pose;
    double cov = 1.0;
};

class CeresBackend : public IBackend {
public:
    CeresBackend();
    ~CeresBackend() override = default;

    bool init(const std::string& config_path) override;
    void addKeyFrame(const KeyFrame& kf) override;
    void addConstraints(const std::vector<Constraint>& constraints) override;
    bool optimize() override;
    bool getUpdatedPoses(std::vector<PoseStamped>& poses) override;

private:
    void addOdometryConstraints(ceres::Problem& problem, std::vector<double*>& param_blocks);
    void addLoopConstraints(ceres::Problem& problem, std::vector<double*>& param_blocks);
    void updatePoses(const std::vector<double*>& param_blocks);

    std::vector<KeyFrameNode> keyframes_;
    std::vector<RelPoseConstraint> constraints_;
    std::unordered_map<std::string, size_t> kf_index_;
    std::recursive_mutex m_mutex;

    int optimize_period_ = 10;
    int num_optimizations_ = 0;
    double huber_loss_ = 1.0;
    double loop_loss_scale_ = 5.0;          // 回环因子 Cauchy 核尺度(m); 假回环会被强降权
    double max_optimization_trans_m_ = 20.0; // 单次含新约束优化的最大平移修正
    double max_optimization_rot_deg_ = 60.0; // 单次含新约束优化的最大旋转修正
    int max_iterations_ = 50;
    // 上一次成功优化时的约束数; 失败时截断到该值, 防止假回环永久留在图里.
    size_t last_success_constraints_ = 0;
    // 回环约束"自洽性"守卫: 优化后残差还这么大的回环, 说明它和图里其它约束矛盾
    // (典型: 假回环 / ICP 锁到相似结构)。这种约束不能留在图里 —— 它会通过
    // 里程计边把整条轨迹匀着拉歪, 地图就"被回环改得更差"。直接剔除并重解。
    double loop_residual_trans_m_ = 0.40;   // 回环残差(平移)上限 m
    double loop_residual_rot_deg_ = 5.0;    // 回环残差(旋转)上限 deg
    /// 剔除残差超限的回环约束, 返回剔除数量
    size_t dropInconsistentLoops(const std::vector<std::array<double, 3>> & trans,
                                 const std::vector<std::array<double, 4>> & rot);
};

} // namespace rosiwit_slam
