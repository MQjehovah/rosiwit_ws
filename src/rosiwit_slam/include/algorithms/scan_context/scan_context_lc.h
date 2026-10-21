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
                   float& out_fitness);

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
    // 回环约束进入后端前的相对位姿门控: 拒绝几十米/上百度的假回环.
    double m_max_relative_trans_m = 8.0;
    double m_max_relative_rot_deg = 30.0;
    // 回环相对位姿必须与前端里程计先验相差不大, 防止几何上自洽但实际错配的假回环.
    double m_max_consistency_trans_m = 8.0;
    double m_max_consistency_rot_deg = 30.0;

    std::vector<std::pair<size_t, size_t>> m_detected_loops;
};

} // namespace rosiwit_slam
