// src/algorithms/scan_context/scan_context_lc.cpp
#include "algorithms/scan_context/scan_context_lc.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
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
            const YAML::Node common =
                (config["global"] && config["global"]["scan_context"])
                    ? config["global"]["scan_context"] : config["scan_context"];
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
                m_max_relative_trans_m = lc["max_relative_trans_m"].as<double>(m_max_relative_trans_m);
                m_max_relative_rot_deg = lc["max_relative_rot_deg"].as<double>(m_max_relative_rot_deg);
                m_max_consistency_trans_m = lc["max_consistency_trans_m"].as<double>(m_max_consistency_trans_m);
                m_max_consistency_rot_deg = lc["max_consistency_rot_deg"].as<double>(m_max_consistency_rot_deg);

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

    // 起点关键帧必须参与回环验证: 采图 bag 常常从起点出发、回到起点结束,
    // 但 RingKey 的 top-N 筛选可能把起点挤掉. 这里强制保留前几帧作为候选.
    for (size_t i = 0; i < 20 && i + static_cast<size_t>(m_min_keyframe_gap) < latest_idx; ++i) {
        if (!m_keyframes[i].has_descriptor) continue;
        bool present = false;
        for (const auto& c : candidates) {
            if (c.idx == i) { present = true; break; }
        }
        if (!present) candidates.push_back({i, 0.0});
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
    double best_desc_dist = 0.0;
    double best_fitness = std::numeric_limits<double>::infinity();
    Eigen::Matrix4f best_delta = Eigen::Matrix4f::Identity();

    for (size_t ci = 0; ci < num_candidates; ++ci) {
        const auto& cand = candidates[ci];
        const auto& cand_rec = m_keyframes[cand.idx];

        int col_shift = 0;
        double desc_d = ScanContext::distance(cand_rec.desc, latest.desc, &col_shift);
        if (cand.idx < 20) {
            std::cout << "[ScanContextLC] start candidate kf_" << cand.idx
                      << " <-> kf_" << latest_idx << " desc=" << desc_d << std::endl;
        }
        if (desc_d >= m_dist_thresh) continue;

        // ICP 验证. ScanContext 的列偏移方向依赖于描述子的约定,
        // 这里同时尝试 +yaw 和 -yaw, 避免正确的起点/终点回环因为初值符号错误被拒.
        double yaw_diff = 2.0 * M_PI * col_shift / m_cfg_num_sectors;
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        init_guess.block<3,3>(0,0) = Eigen::AngleAxisf(yaw_diff, Eigen::Vector3f::UnitZ()).matrix();

        Eigen::Matrix4f delta;
        float fitness = std::numeric_limits<float>::infinity();
        bool icp_ok = icpVerify(latest.cloud, cand_rec.cloud, init_guess, delta, fitness);
        if (!icp_ok) {
            Eigen::Matrix4f init_guess_flip = Eigen::Matrix4f::Identity();
            init_guess_flip.block<3,3>(0,0) =
                Eigen::AngleAxisf(-yaw_diff, Eigen::Vector3f::UnitZ()).matrix();
            Eigen::Matrix4f delta_flip;
            float fitness_flip = std::numeric_limits<float>::infinity();
            if (icpVerify(latest.cloud, cand_rec.cloud, init_guess_flip, delta_flip, fitness_flip)) {
                icp_ok = true;
                delta = delta_flip;
                fitness = fitness_flip;
            }
        }
        // 起终点通常姿态接近; ScanContext 列偏移可能给错初值, 对起点额外尝试单位初值.
        if (!icp_ok && cand.idx < 20) {
            Eigen::Matrix4f delta_id;
            float fitness_id = std::numeric_limits<float>::infinity();
            if (icpVerify(latest.cloud, cand_rec.cloud, Eigen::Matrix4f::Identity(), delta_id, fitness_id)) {
                icp_ok = true;
                delta = delta_id;
                fitness = fitness_id;
            }
        }
        if (!icp_ok) {
            std::cout << "[ScanContextLC] ICP failed: desc=" << desc_d
                      << ", fitness=" << fitness << std::endl;
            continue;
        }

        // 先在候选层做几何门控, 再按 ICP fitness 选择最优候选.
        // 不能按 desc_dist 贪心: 假回环的 desc 往往比真回环更小.
        PoseStamped cand_rel;
        cand_rel.rot = delta.block<3,3>(0,0).cast<double>();
        cand_rel.trans = delta.block<3,1>(0,3).cast<double>();
        const double rel_trans = cand_rel.trans.norm();
        const Eigen::AngleAxisd rel_aa(cand_rel.rot);
        const double rel_rot_deg = std::abs(rel_aa.angle()) * 180.0 / M_PI;
        if (rel_trans > m_max_relative_trans_m || rel_rot_deg > m_max_relative_rot_deg) {
            std::cout << "[ScanContextLC] reject relative loop: kf_" << cand.idx
                      << " <-> kf_" << latest_idx
                      << " d_trans=" << rel_trans << "m"
                      << " d_rot=" << rel_rot_deg << "deg" << std::endl;
            continue;
        }

        PoseStamped odom_rel;
        odom_rel.rot = cand_rec.pose.rot.transpose() * latest.pose.rot;
        odom_rel.trans = cand_rec.pose.rot.transpose() *
                         (latest.pose.trans - cand_rec.pose.trans);
        const double consist_t = (cand_rel.trans - odom_rel.trans).norm();
        const Eigen::AngleAxisd consist_aa(cand_rel.rot.transpose() * odom_rel.rot);
        const double consist_rot_deg = std::abs(consist_aa.angle()) * 180.0 / M_PI;
        if (consist_t > m_max_consistency_trans_m ||
            consist_rot_deg > m_max_consistency_rot_deg) {
            std::cout << "[ScanContextLC] reject inconsistent loop: kf_" << cand.idx
                      << " <-> kf_" << latest_idx
                      << " odom_delta=" << consist_t << "m/"
                      << consist_rot_deg << "deg" << std::endl;
            continue;
        }

        std::cout << "[ScanContextLC] ICP candidate passed: kf_" << cand.idx
                  << " <-> kf_" << latest_idx
                  << " desc=" << desc_d << " fitness=" << fitness
                  << " odom_delta=" << consist_t << "m/"
                  << consist_rot_deg << "deg" << std::endl;
        if (fitness < best_fitness) {
            best_fitness = fitness;
            best_desc_dist = desc_d;
            best_idx = static_cast<int>(cand.idx);
            best_delta = delta;
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

    // 假回环保护: ICP 通过不等于几何合理. 几十米/上百度的相对位姿
    // 会把 Ceres 位姿图整体拉坏, 必须在进入后端前拒绝.
    const double rel_trans = relative_pose.trans.norm();
    const Eigen::AngleAxisd rel_aa(relative_pose.rot);
    const double rel_rot_deg = std::abs(rel_aa.angle()) * 180.0 / M_PI;
    if (rel_trans > m_max_relative_trans_m || rel_rot_deg > m_max_relative_rot_deg) {
        std::cout << "[ScanContextLC] reject relative loop: kf_" << best_idx
                  << " <-> kf_" << latest_idx
                  << " d_trans=" << rel_trans << "m"
                  << " d_rot=" << rel_rot_deg << "deg"
                  << " (limits " << m_max_relative_trans_m << "m/"
                  << m_max_relative_rot_deg << "deg)" << std::endl;
        return false;
    }

    // 与前端里程计先验做一致性检查: 回环测量不能和当前轨迹相差过大,
    // 否则 Ceres 会把一串关键帧整体拉走 (实测 30~500m / 170° 的假回环).
    PoseStamped odom_rel;
    odom_rel.time = latest.pose.time;
    odom_rel.rot = match.pose.rot.transpose() * latest.pose.rot;
    odom_rel.trans = match.pose.rot.transpose() * (latest.pose.trans - match.pose.trans);
    const double consist_t = (relative_pose.trans - odom_rel.trans).norm();
    const Eigen::AngleAxisd consist_aa(relative_pose.rot.transpose() * odom_rel.rot);
    const double consist_rot_deg = std::abs(consist_aa.angle()) * 180.0 / M_PI;
    if (consist_t > m_max_consistency_trans_m || consist_rot_deg > m_max_consistency_rot_deg) {
        std::cout << "[ScanContextLC] reject inconsistent loop: kf_" << best_idx
                  << " <-> kf_" << latest_idx
                  << " odom_delta=" << consist_t << "m/"
                  << consist_rot_deg << "deg"
                  << " (limits " << m_max_consistency_trans_m << "m/"
                  << m_max_consistency_rot_deg << "deg)" << std::endl;
        return false;
    }

    cov = std::min(1.0, best_desc_dist / m_dist_thresh);

    m_detected_loops.emplace_back(best_idx, latest_idx);

    std::cout << "[ScanContextLC] Loop detected: kf_" << best_idx
              << " <-> kf_" << latest_idx
              << " (desc_dist=" << best_desc_dist << ")" << std::endl;

    return true;
}

} // namespace rosiwit_slam
