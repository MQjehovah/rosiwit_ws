// include/algorithms/scan_context/scan_context_lc.h
#pragma once
#include "slam_core/i_loop_closure.h"
#include "slam_core/pipeline_types.h"
#include "algorithms/scan_context/scan_context.h"
#include <deque>
#include <unordered_map>
#include <mutex>
#include <pcl/registration/gicp.h>

namespace rosiwit_slam {

class ScanContextLoopClosure : public ILoopClosure {
public:
    ScanContextLoopClosure() = default;
    ~ScanContextLoopClosure() override = default;

    bool init(const std::string& config_path) override;
    void addKeyFrame(const KeyFrame& kf) override;
    bool detect(PoseStamped& relative_pose, double& cov,
                std::string& matched_kf_id) override;
    bool saveDatabase(const std::string& path) override;
    bool getKeyframes(std::vector<KeyFrame>& out) override;

private:
    struct KeyFrameRecord {
        std::string id;
        PoseStamped pose;
        CloudType::Ptr cloud;
        ScanContext::Descriptor desc;
        bool has_descriptor = false;
    };

    bool icpVerify(const CloudType::Ptr& src,
                   const CloudType::Ptr& tgt,
                   const Eigen::Matrix4f& init_guess,
                   Eigen::Matrix4f& out_delta,
                   float& out_fitness,
                   float& out_overlap);
    /// 已经降采样过的版本: 同一份 latest 子图/候选子图会被多个初值反复用,
    /// 之前每次调用都重新 VoxelGrid, 检测一次要跑几十遍滤波 (aarch64 上很贵)。
    bool icpVerifyDownsampled(const CloudType::Ptr& src_down,
                              const CloudType::Ptr& tgt_down,
                              const Eigen::Matrix4f& init_guess,
                              Eigen::Matrix4f& out_delta,
                              float& out_fitness,
                              float& out_overlap) const;
    static CloudType::Ptr downsampleCloud(const CloudType::Ptr& in, float leaf);
    /// 以 center 关键帧为参考，把邻近关键帧拼成局部子图；单帧 ICP 太容易受视角/稀疏影响.
    CloudType::Ptr buildSubmap(size_t center_idx, int half_window) const;

    std::deque<KeyFrameRecord> m_keyframes;
    std::unordered_map<std::string, size_t> m_kf_index;
    mutable std::mutex m_mutex;

    ScanContext m_sc;

    int m_min_keyframe_gap = 30;
    double m_dist_thresh = 0.4;
    double m_candidate_ratio = 0.3;
    int m_cfg_num_rings = 20;
    int m_cfg_num_sectors = 60;
    double m_cfg_max_radius = 80.0;

    // ICP 验证参数
    float m_icp_max_corr_dist = 2.0f;
    float m_icp_fitness_thresh = 1.0f;
    int m_icp_max_iter = 32;
    float m_icp_voxel_res = 0.3f;
    int m_submap_half_window = 3;       // 局部回环子图: 左右各取几个关键帧
    float m_min_overlap = 0.35f;        // ICP 后最近邻命中率下限
    // 回环约束进入后端前的相对位姿门控: 拒绝几十米/上百度的假回环.
    double m_max_relative_trans_m = 8.0;
    double m_max_relative_rot_deg = 30.0;
    // 回环相对位姿必须与前端里程计先验相差不大, 防止几何上自洽但实际错配的假回环.
    double m_max_consistency_trans_m = 8.0;
    double m_max_consistency_rot_deg = 30.0;
    /// 里程计门控: 前端里程计说两帧相距超过这个距离时, 不可能是回环。
    /// 以前没有这道门, 每个候选都要跑 GICP (还带 2~3 个初值), 既慢又给了假回环机会。
    double m_odom_gate_m = 10.0;

    std::vector<std::pair<size_t, size_t>> m_detected_loops;
};

} // namespace rosiwit_slam
