// src/algorithms/scan_context/scan_context_lc.cpp
#include "algorithms/scan_context/scan_context_lc.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <pcl/common/transforms.h>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

bool ScanContextLoopClosure::init(const std::string& config_path) {
    std::ifstream fin(config_path);
    if (!fin.good()) {
        std::cerr << "[ScanContextLC_v2] Config not found, using defaults" << std::endl;
    } else {
#ifndef YAML_CPP_DISABLED
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            auto lc = config["loop_closure"];
            if (lc) {
                m_min_keyframe_gap  = lc["sc_min_keyframe_gap"].as<int>(m_min_keyframe_gap);
                m_dist_thresh       = lc["sc_dist_thresh"].as<double>(m_dist_thresh);
                m_candidate_ratio   = lc["sc_candidate_ratio"].as<double>(m_candidate_ratio);

                ScanContext::Config sc_cfg;
                sc_cfg.num_rings   = lc["sc_num_rings"].as<int>(sc_cfg.num_rings);
                sc_cfg.num_sectors = lc["sc_num_sectors"].as<int>(sc_cfg.num_sectors);
                sc_cfg.max_radius  = lc["sc_max_radius"].as<double>(sc_cfg.max_radius);
                m_sc = ScanContext(sc_cfg);
                m_cfg_num_rings = sc_cfg.num_rings;
                m_cfg_num_sectors = sc_cfg.num_sectors;
                m_cfg_max_radius = sc_cfg.max_radius;
            }
        } catch (const std::exception& e) {
            std::cerr << "[ScanContextLC_v2] Config parse error: " << e.what() << std::endl;
            return false;
        }
#endif
    }

    std::cout << "[ScanContextLC_v2] Initialized (rings=" << m_cfg_num_rings
              << ", sectors=" << m_cfg_num_sectors
              << ", max_r=" << m_cfg_max_radius
              << ", dist_thresh=" << m_dist_thresh << ")" << std::endl;
    return true;
}

void ScanContextLoopClosure::addKeyFrame(const KeyFrame& kf) {
    if (!kf.cloud || kf.cloud->empty()) return;

    // 构造描述符
    KeyFrameRecord rec;
    rec.id = kf.id;
    rec.pose = kf.pose;
    rec.desc = m_sc.makeDescriptor(kf.cloud);
    rec.has_descriptor = true;

    m_kf_index[kf.id] = m_keyframes.size();
    m_keyframes.push_back(std::move(rec));
}

bool ScanContextLoopClosure::detect(PoseStamped& relative_pose, double& cov) {
    if (m_keyframes.size() < static_cast<size_t>(m_min_keyframe_gap + 1)) {
        return false;
    }

    const auto& latest = m_keyframes.back();
    if (!latest.has_descriptor) return false;

    size_t latest_idx = m_keyframes.size() - 1;
    size_t search_start = 0;  // 从头开始搜索

    // === 1. RingKey 快速筛选候选 ===
    struct Candidate {
        size_t idx;
        double rk_dist;
    };
    std::vector<Candidate> candidates;

    for (size_t i = search_start; i < latest_idx - m_min_keyframe_gap; ++i) {
        if (!m_keyframes[i].has_descriptor) continue;

        // 检查是否已检测过
        bool already_detected = false;
        for (const auto& loop : m_detected_loops) {
            if (loop.first == i && loop.second == latest_idx) {
                already_detected = true;
                break;
            }
        }
        if (already_detected) continue;

        double rk_dist = ScanContext::ringKeyDistance(
            m_keyframes[i].desc.ring_key, latest.desc.ring_key);
        candidates.push_back({i, rk_dist});
    }

    if (candidates.empty()) return false;

    // 按 RingKey 距离排序, 取 top k
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) {
                  return a.rk_dist < b.rk_dist;
              });

    size_t num_candidates = std::max(1, static_cast<int>(
        candidates.size() * m_candidate_ratio));
    num_candidates = std::min(num_candidates, candidates.size());

    // === 2. 精确描述符匹配 ===
    double best_dist = m_dist_thresh;  // 只有比阈值更好的才接受
    int best_candidate_idx = -1;
    int best_col_shift = 0;

    for (size_t ci = 0; ci < num_candidates; ++ci) {
        const auto& cand = candidates[ci];
        const auto& cand_rec = m_keyframes[cand.idx];

        int col_shift = 0;
        double d = ScanContext::distance(cand_rec.desc, latest.desc, &col_shift);

        if (d < best_dist) {
            best_dist = d;
            best_candidate_idx = static_cast<int>(cand.idx);
            best_col_shift = col_shift;
        }
    }

    if (best_candidate_idx < 0) return false;

    // === 3. 计算相对位姿 ===
    const auto& match = m_keyframes[best_candidate_idx];

    // 柱偏移 → 旋转角度
    double yaw_diff = 2.0 * M_PI * best_col_shift / m_cfg_num_sectors;

    // 从两个关键帧的估计位姿计算相对变换
    // T_candidate_latest = T_candidate^{-1} * T_latest
    Eigen::Matrix3d R_c_inv = match.pose.rot.transpose();
    Eigen::Vector3d t_c_inv = -R_c_inv * match.pose.trans;

    Eigen::Matrix3d R_rel = R_c_inv * latest.pose.rot;
    Eigen::Vector3d t_rel = R_c_inv * (latest.pose.trans - match.pose.trans);

    relative_pose.time = latest.pose.time;
    relative_pose.rot = R_rel;
    relative_pose.trans = t_rel;

    // 置信度: 距离越小置信度越高
    cov = std::min(1.0, best_dist / m_dist_thresh);

    // 记录检测到的回环
    m_detected_loops.emplace_back(best_candidate_idx, latest_idx);

    std::cout << "[ScanContextLC_v2] Loop detected: kf_" << best_candidate_idx
              << " <-> kf_" << latest_idx
              << " (desc_dist=" << best_dist << ", yaw_shift=" << yaw_diff * 180.0 / M_PI << "deg)"
              << std::endl;

    return true;
}

} // namespace rosiwit_slam
