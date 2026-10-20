// src/algorithms/scan_context/scan_context_lc.cpp
#include "algorithms/scan_context/scan_context_lc.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

bool ScanContextLoopClosure::init(const std::string& config_path) {
    std::ifstream fin(config_path);
    if (!fin.good()) {
        std::cerr << "[ScanContextLC] Config not found, using defaults" << std::endl;
    } else {
#ifndef YAML_CPP_DISABLED
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            // 标准层级 mapping.loop_closure；旧配置回退根节点 loop_closure。
            auto lc = (config["mapping"] && config["mapping"]["loop_closure"])
                          ? config["mapping"]["loop_closure"]
                          : config["loop_closure"];
            const YAML::Node common = config["scan_context"];
            if (lc) {
                m_min_keyframe_gap = lc["min_keyframe_gap"]
                                         ? lc["min_keyframe_gap"].as<int>(m_min_keyframe_gap)
                                         : lc["sc_min_keyframe_gap"].as<int>(m_min_keyframe_gap);
                m_dist_thresh     = common && common["dist_thresh"]
                                        ? common["dist_thresh"].as<double>(m_dist_thresh)
                                        : lc["sc_dist_thresh"].as<double>(m_dist_thresh);
                m_candidate_ratio = common && common["candidate_ratio"]
                                        ? common["candidate_ratio"].as<double>(m_candidate_ratio)
                                        : lc["sc_candidate_ratio"].as<double>(m_candidate_ratio);
                m_icp_fitness_thresh = lc["icp_fitness_thresh"].as<float>(m_icp_fitness_thresh);
                m_icp_max_corr_dist  = lc["icp_max_corr_dist"].as<float>(m_icp_max_corr_dist);
                m_icp_voxel_res      = lc["icp_voxel_res"].as<float>(m_icp_voxel_res);

                ScanContext::Config sc_cfg;
                sc_cfg.num_rings   = common && common["num_rings"]
                                         ? common["num_rings"].as<int>(sc_cfg.num_rings)
                                         : lc["sc_num_rings"].as<int>(sc_cfg.num_rings);
                sc_cfg.num_sectors = common && common["num_sectors"]
                                         ? common["num_sectors"].as<int>(sc_cfg.num_sectors)
                                         : lc["sc_num_sectors"].as<int>(sc_cfg.num_sectors);
                sc_cfg.max_radius  = common && common["max_radius"]
                                         ? common["max_radius"].as<double>(sc_cfg.max_radius)
                                         : lc["sc_max_radius"].as<double>(sc_cfg.max_radius);
                m_sc = ScanContext(sc_cfg);
                m_cfg_num_rings = sc_cfg.num_rings;
                m_cfg_num_sectors = sc_cfg.num_sectors;
                m_cfg_max_radius = sc_cfg.max_radius;
            }
        } catch (const std::exception& e) {
            std::cerr << "[ScanContextLC] Config parse error: " << e.what() << std::endl;
            return false;
        }
#endif
    }

    std::cout << "[ScanContextLC] Initialized (rings=" << m_cfg_num_rings
              << ", sectors=" << m_cfg_num_sectors
              << ", max_r=" << m_cfg_max_radius
              << ", dist_thresh=" << m_dist_thresh
              << ", icp_fitness<" << m_icp_fitness_thresh << ")" << std::endl;
    return true;
}

void ScanContextLoopClosure::addKeyFrame(const KeyFrame& kf) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!kf.cloud || kf.cloud->empty()) return;

    KeyFrameRecord rec;
    rec.id = kf.id;
    rec.pose = kf.pose;
    rec.cloud = kf.cloud;
    rec.desc = m_sc.makeDescriptor(kf.cloud);
    rec.has_descriptor = true;

    m_kf_index[kf.id] = m_keyframes.size();
    m_keyframes.push_back(std::move(rec));
}

bool ScanContextLoopClosure::icpVerify(
    const CloudType::Ptr& src, const CloudType::Ptr& tgt,
    const Eigen::Matrix4f& init_guess,
    Eigen::Matrix4f& out_delta, float& out_fitness)
{
    // 降采样
    auto downsample = [&](const CloudType::Ptr& in) -> CloudType::Ptr {
        CloudType::Ptr out(new CloudType);
        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud(in);
        vg.setLeafSize(m_icp_voxel_res, m_icp_voxel_res, m_icp_voxel_res);
        vg.filter(*out);
        return out;
    };

    auto src_down = downsample(src);
    auto tgt_down = downsample(tgt);

    if (src_down->size() < 10 || tgt_down->size() < 10) return false;

    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setInputSource(src_down);
    gicp.setInputTarget(tgt_down);
    gicp.setMaxCorrespondenceDistance(m_icp_max_corr_dist);
    gicp.setMaximumIterations(m_icp_max_iter);
    gicp.setTransformationEpsilon(1e-8);
    gicp.setEuclideanFitnessEpsilon(1e-6);

    CloudType aligned;
    gicp.align(aligned, init_guess);

    if (!gicp.hasConverged()) return false;

    out_fitness = gicp.getFitnessScore();
    out_delta = gicp.getFinalTransformation();

    return out_fitness < m_icp_fitness_thresh;
}

bool ScanContextLoopClosure::saveDatabase(const std::string& path) {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<ScanContextDatabaseEntry> db;
    db.reserve(m_keyframes.size());
    for (const auto& rec : m_keyframes) {
        if (!rec.has_descriptor) continue;
        ScanContextDatabaseEntry e;
        e.id = rec.id;
        e.rot = rec.pose.rot;
        e.trans = rec.pose.trans;
        e.desc = rec.desc;
        db.push_back(std::move(e));
    }
    bool ok = saveScanContextDatabase(path, db);
    if (ok) {
        std::cout << "[ScanContextLC] Saved database: " << db.size()
                  << " keyframes -> " << path << std::endl;
    }
    return ok;
}

bool ScanContextLoopClosure::getKeyframes(std::vector<KeyFrame>& out) {
    std::lock_guard<std::mutex> lock(m_mutex);
    out.clear();
    out.reserve(m_keyframes.size());
    for (const auto& rec : m_keyframes) {
        KeyFrame kf;
        kf.id = rec.id;
        kf.pose = rec.pose;
        kf.cloud = rec.cloud;
        kf.timestamp = rec.pose.time > 0
            ? static_cast<int64_t>(rec.pose.time * 1e6) : 0;
        out.push_back(std::move(kf));
    }
    return !out.empty();
}

bool ScanContextLoopClosure::detect(PoseStamped& relative_pose, double& cov,
                                     std::string& matched_kf_id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_keyframes.size() < static_cast<size_t>(m_min_keyframe_gap + 1)) {
        return false;
    }

    const auto& latest = m_keyframes.back();
    if (!latest.has_descriptor || !latest.cloud) return false;

    size_t latest_idx = m_keyframes.size() - 1;
    size_t search_start = 0;

    // === 1. RingKey 快速筛选 ===
    struct Candidate { size_t idx; double rk_dist; };
    std::vector<Candidate> candidates;

    for (size_t i = search_start; i < latest_idx - m_min_keyframe_gap; ++i) {
        if (!m_keyframes[i].has_descriptor) continue;

        bool already = false;
        for (const auto& loop : m_detected_loops) {
            if (loop.first == i && loop.second == latest_idx) { already = true; break; }
        }
        if (already) continue;

        double rk_dist = ScanContext::ringKeyDistance(
            m_keyframes[i].desc.ring_key, latest.desc.ring_key);
        candidates.push_back({i, rk_dist});
    }
    if (candidates.empty()) {
        std::cout << "[ScanContextLC] No candidates from RingKey" << std::endl;
        return false;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.rk_dist < b.rk_dist; });

    size_t num_candidates = std::max(1, static_cast<int>(
        candidates.size() * m_candidate_ratio));
    num_candidates = std::min(num_candidates, candidates.size());

    // === 2. 精确描述符匹配 + ICP 验证 ===
    std::cout << "[ScanContextLC] " << candidates.size() << " candidates, top "
              << num_candidates << " to check" << std::endl;
    int best_idx = -1;
    int best_col_shift = 0;
    double best_desc_dist = m_dist_thresh;
    Eigen::Matrix4f best_delta = Eigen::Matrix4f::Identity();

    for (size_t ci = 0; ci < num_candidates; ++ci) {
        const auto& cand = candidates[ci];
        const auto& cand_rec = m_keyframes[cand.idx];

        int col_shift = 0;
        double desc_d = ScanContext::distance(cand_rec.desc, latest.desc, &col_shift);
        if (desc_d >= best_desc_dist) continue;

        // ICP 验证
        double yaw_diff = 2.0 * M_PI * col_shift / m_cfg_num_sectors;
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        init_guess.block<3,3>(0,0) = Eigen::AngleAxisf(yaw_diff, Eigen::Vector3f::UnitZ()).matrix();

        Eigen::Matrix4f delta;
        float fitness;
        if (icpVerify(latest.cloud, cand_rec.cloud, init_guess, delta, fitness)) {
            std::cout << "[ScanContextLC] ICP passed: desc=" << desc_d
                      << ", fitness=" << fitness << std::endl;
            best_desc_dist = desc_d;
            best_idx = static_cast<int>(cand.idx);
            best_col_shift = col_shift;
            best_delta = delta;
        } else {
            std::cout << "[ScanContextLC] ICP failed: desc=" << desc_d
                      << ", fitness=" << fitness << std::endl;
        }
    }

    if (best_idx < 0) return false;

    const auto& match = m_keyframes[best_idx];

    // === 3. 相对位姿 ===
    // GICP 的 delta 是 candidate<-latest 的实测变换; 回环因子必须用它.
    // 之前用 match.pose^{-1}*latest.pose (前端里程计先验) 重建相对位姿,
    // 该因子与里程计完全一致, 回环不会提供任何修正量.
    matched_kf_id = match.id;
    relative_pose.time = latest.pose.time;
    relative_pose.rot = best_delta.block<3,3>(0,0).cast<double>();
    relative_pose.trans = best_delta.block<3,1>(0,3).cast<double>();

    cov = std::min(1.0, best_desc_dist / m_dist_thresh);

    m_detected_loops.emplace_back(best_idx, latest_idx);

    std::cout << "[ScanContextLC] Loop detected: kf_" << best_idx
              << " <-> kf_" << latest_idx
              << " (desc_dist=" << best_desc_dist << ")" << std::endl;

    return true;
}

} // namespace rosiwit_slam
