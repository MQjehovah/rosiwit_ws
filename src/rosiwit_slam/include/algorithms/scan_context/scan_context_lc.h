// include/algorithms/scan_context/scan_context_lc.h
// 基于真正 ScanContext 描述符的回环检测 — 替代简单的距离阈值方案
//
// 对比 ScanContextLC (距离阈值方案):
//   ScanContextLC:        每来一帧检查位置距离 < 15m → 大场景失效
//   ScanContextLoopClosure: 计算描述符 → RingKey 快速检索 → 精确匹配
//
// 使用方法 (config/default.yaml):
//   modules:
//     loop_closure: "scan_context_v2"
//
// 配置:
//   loop_closure:
//     sc_num_rings: 20         # 环数
//     sc_num_sectors: 60       # 扇区数
//     sc_max_radius: 80.0      # 最大半径
//     sc_min_keyframe_gap: 30  # 最小关键帧间隔 (帧数)
//     sc_dist_thresh: 0.4      # 描述符距离阈值
//     sc_candidate_ratio: 0.3  # RingKey 筛选比例

#pragma once
#include "slam_core/i_loop_closure.h"
#include "slam_core/pipeline_types.h"
#include "algorithms/scan_context/scan_context.h"
#include <deque>
#include <unordered_map>

namespace rosiwit_slam {

class ScanContextLoopClosure : public ILoopClosure {
public:
    ScanContextLoopClosure() = default;
    ~ScanContextLoopClosure() override = default;

    bool init(const std::string& config_path) override;
    void addKeyFrame(const KeyFrame& kf) override;
    bool detect(PoseStamped& relative_pose, double& cov) override;

private:
    struct KeyFrameRecord {
        std::string id;
        PoseStamped pose;
        ScanContext::Descriptor desc;
        bool has_descriptor = false;
    };

    // 计算两点云之间的相对位姿 (简易 ICP, 用于验证回环)
    bool computeRelativePose(const CloudType::Ptr& src_cloud,
                             const CloudType::Ptr& tgt_cloud,
                             const PoseStamped& src_pose,
                             const PoseStamped& tgt_pose,
                             PoseStamped& rel_pose);

    std::deque<KeyFrameRecord> m_keyframes;
    std::unordered_map<std::string, size_t> m_kf_index;

    // ScanContext
    ScanContext m_sc;

    // 配置
    int m_min_keyframe_gap = 30;
    double m_dist_thresh = 0.4;
    double m_candidate_ratio = 0.3;
    int m_cfg_num_rings = 20;
    int m_cfg_num_sectors = 60;
    double m_cfg_max_radius = 80.0;

    // 已检测到的回环 (避免重复)
    std::vector<std::pair<size_t, size_t>> m_detected_loops;
};

} // namespace rosiwit_slam
