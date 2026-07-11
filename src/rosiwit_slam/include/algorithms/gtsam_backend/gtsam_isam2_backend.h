// include/algorithms/gtsam_backend/gtsam_isam2_backend.h
// GTSAM iSAM2 增量式因子图后端 — 替代 Ceres 批量优化
//
// 对比 CeresBackend:
//   CeresBackend: 每 50 帧从零构建整个问题 → O(N) 随帧数增长
//   GtsamIsam2Backend: 增量更新, 每帧仅添加新因子 → O(log N) 摊销
//
// 使用方法 (config/default.yaml):
//   modules:
//     backend: "gtsam_isam2"
//
// 依赖: sudo apt install ros-humble-gtsam

#pragma once
#include "slam_core/i_backend.h"
#include "slam_core/slam_types.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#ifdef USE_GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#endif

namespace rosiwit_slam {

#ifdef USE_GTSAM
class GtsamIsam2Backend : public IBackend {
public:
    GtsamIsam2Backend() = default;
    ~GtsamIsam2Backend() override = default;

    bool init(const std::string& config_path) override;
    void addKeyFrame(const KeyFrame& kf) override;
    void addConstraints(const std::vector<Constraint>& constraints) override;
    bool optimize() override;
    bool getUpdatedPoses(std::vector<PoseStamped>& poses) override;

private:
    struct KeyFrameNode {
        std::string id;
        PoseStamped pose;
        int64_t timestamp = 0;
        bool is_optimized = false;
    };

    gtsam::Pose3 toGtsamPose(const PoseStamped& p) const;
    PoseStamped fromGtsamPose(const gtsam::Pose3& gp, double time) const;

    std::vector<KeyFrameNode> m_keyframes;
    std::vector<Constraint> m_pending_constraints;
    std::unordered_map<std::string, size_t> m_kf_index;

    std::unique_ptr<gtsam::NonlinearFactorGraph> m_graph;
    std::unique_ptr<gtsam::Values> m_initial_estimates;
    std::unique_ptr<gtsam::ISAM2> m_isam2;

    bool m_inited = false;
    size_t m_last_opt_size = 0;

    // 配置
    double m_odom_trans_noise = 0.1;
    double m_odom_rot_noise = 0.05;
    double m_lc_trans_noise = 0.5;
    double m_lc_rot_noise = 0.3;
    int m_isam2_relin_thresh = 10;
    bool m_enable_partial_relin = true;
};
#else
// GTSAM 未安装时提供桩实现, 编译通过但工厂不会注册
class GtsamIsam2Backend : public IBackend {
public:
    bool init(const std::string&) override { return false; }
    void addKeyFrame(const KeyFrame&) override {}
    void addConstraints(const std::vector<Constraint>&) override {}
    bool optimize() override { return false; }
    bool getUpdatedPoses(std::vector<PoseStamped>&) override { return false; }
};
#endif

} // namespace rosiwit_slam
