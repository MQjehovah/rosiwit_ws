// src/algorithms/localization/global/scan_context_global_localizer.cpp
#include "algorithms/localization/global/scan_context_global_localizer.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/registration_metrics.h"
#include <algorithm>
#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>

namespace rosiwit_slam {

void ScanContextGlobalLocalizer::configure(const GlobalLocalizeConfig& cfg) {
    m_cfg = cfg;
    ScanContext::Config sc_cfg;
    sc_cfg.num_rings = cfg.sc_num_rings;
    sc_cfg.num_sectors = cfg.sc_num_sectors;
    sc_cfg.max_radius = cfg.sc_max_radius;
    sc_cfg.min_radius = cfg.sc_min_radius;
    m_sc = std::make_shared<ScanContext>(sc_cfg);

    m_gicp.setMaximumIterations(cfg.icp_max_iter);
    m_gicp.setMaxCorrespondenceDistance(cfg.icp_max_corr_dist);
    m_gicp.setTransformationEpsilon(1e-8);
    m_gicp.setEuclideanFitnessEpsilon(1e-6);
    // 粗配准: 少迭代 + 松对应距离, 只用来淘汰错误候选
    m_gicp_coarse.setMaximumIterations(12);
    m_gicp_coarse.setMaxCorrespondenceDistance(cfg.icp_max_corr_dist);
    m_gicp_coarse.setTransformationEpsilon(1e-6);
    m_gicp_coarse.setEuclideanFitnessEpsilon(1e-5);
}

bool ScanContextGlobalLocalizer::setMap(const std::string& map_path,
                                        const CloudType::Ptr& global_map) {
    m_ready = false;
    m_db.clear();

    const std::string db_path = map_path + ".scdb";
    if (!loadScanContextDatabase(db_path, m_db) || m_db.empty()) {
        std::cerr << "[SCGlobal] no ScanContext db: " << db_path << std::endl;
        return false;
    }
    m_map = downsampleCloud(filterFinite(global_map), m_cfg.voxel_resolution);
    if (!m_map || m_map->empty()) return false;
    m_kdt.setInputCloud(m_map);
    // 粗层: 六级精配的候选淘汰放在这上面跑 (点少 => 单次配准 10~20x 便宜)
    m_map_coarse = downsampleCloud(m_map, 0.6);
    if (!m_map_coarse || m_map_coarse->size() < 100) m_map_coarse = m_map;
    m_kdt_coarse.setInputCloud(m_map_coarse);

    m_ready = true;
    std::cout << "[SCGlobal] loaded db: " << m_db.size() << " kfs" << std::endl;
    return true;
}

int ScanContextGlobalLocalizer::localizeTopK(const CloudType::Ptr& scan, int max_k,
                                             std::vector<GlobalCandidate>& out) {
    out.clear();
    if (!m_ready || !m_sc || !scan || scan->empty()) return 0;

    const ScanContext::Descriptor cur = m_sc->makeDescriptor(scan);

    struct Candidate { size_t idx; double rk; double d; int shift; };
    std::vector<Candidate> cands;
    cands.reserve(m_db.size());
    for (size_t i = 0; i < m_db.size(); ++i)
        cands.push_back({i, ScanContext::ringKeyDistance(m_db[i].desc.ring_key, cur.ring_key), 0.0, 0});
    std::sort(cands.begin(), cands.end(),
              [](const Candidate& a, const Candidate& b) { return a.rk < b.rk; });
    const size_t num = std::max<size_t>(1, static_cast<size_t>(cands.size() * m_cfg.sc_candidate_ratio));

    std::vector<Candidate> hits;
    for (size_t ci = 0; ci < num; ++ci) {
        int shift = 0;
        const double d = ScanContext::distance(m_db[cands[ci].idx].desc, cur, &shift);
        if (d < m_cfg.sc_dist_thresh) {
            Candidate c = cands[ci]; c.d = d; c.shift = shift;
            hits.push_back(c);
        }
    }
    std::sort(hits.begin(), hits.end(), [](const Candidate& a, const Candidate& b) { return a.d < b.d; });

    CloudType::Ptr scan_down = downsampleCloud(scan, m_cfg.voxel_resolution);
    if (!scan_down || scan_down->size() < 10) return 0;
    // 精配用的源点云: 比评分用的更稀 (GICP 代价 ~ 源点数 x 目标点数 x 迭代),
    // 位姿精度由最终 coverage 与管线二级精配保证.
    CloudType::Ptr scan_icp = downsampleCloud(
        scan_down, std::max(m_cfg.voxel_resolution, 0.4));
    if (!scan_icp || scan_icp->size() < 10) scan_icp = scan_down;

    // 两级精配 + 提前结束:
    //   阶段1 粗层 (0.6 m) 淘汰绝大多数错误候选; 阶段2 只在粗层通过的候选上做全精度.
    //   以前每个候选都在 0.3 m 全图上跑完整 GICP, 15 个候选 = 10~40 s.
    const int try_limit = std::max(1, max_k * 3);
    const double coarse_gate = std::max(0.30, 0.55 * m_cfg.min_score);
    int refined = 0;      // 成功产出候选数
    int fine_tried = 0;   // 实际精配次数 (无论成功), 决定最坏耗时
    // 只精配最靠前的几个候选: 覆盖率打分已经排序, 排在 4 名之后的基本是重复结构.
    // 管线只需要 best/second 做歧义门控, 3 个足够; 5 个会把最坏耗时顶到 5 s 以上.
    const int fine_budget = std::max(1, std::min(3, max_k));
    for (int ci = 0; ci < static_cast<int>(hits.size()) && ci < try_limit; ++ci) {
        const auto& match = m_db[hits[ci].idx];
        const double yaw = 2.0 * M_PI * hits[ci].shift / m_cfg.sc_num_sectors;
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
        guess.block<3,3>(0,0) = (match.rot * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                 .toRotationMatrix()).cast<float>();
        guess.block<3,1>(0,3) = match.trans.cast<float>();

        // ---- 阶段1: 用候选位姿直接打覆盖分 ----
        // 不在这里跑粗 GICP: GICP 每次 setInputTarget 都要重算目标协方差,
        // 十几个候选 = 几秒~几十秒 (实测最坏 25 s). 覆盖打分只有 2000 次 1-NN,
        // 单次 ~10 ms, 用 0.8 m 容差足够把 "差 1~2 m" 的错误候选淘汰掉.
        Eigen::Matrix4f guess2 = guess;
        {
            const Eigen::Matrix4d T0 = guess.cast<double>();
            const CoverageStats cs0 = evaluateCoverage(m_kdt, *scan_down, T0,
                                                      /*inlier_dist=*/0.8, /*max_points=*/2000);
            if (cs0.coverage < coarse_gate) continue;
        }

        // ---- 阶段2: 全精度 ----
        if (fine_tried >= fine_budget) break;   // 最坏耗时的上界 (自相似场景候选很多)
        ++fine_tried;
        // 精配目标点数上限 1.2 万: GICP 代价 ~ 源点数 x 目标点数, 点太多会把
        // 最坏情况拖到几秒 (实测 6 万点 x 15 候选 = 35 s).
        CloudType::Ptr target = m_cfg.reloc_submap_radius > 0.0
            ? cropRadius(m_kdt, m_map, V3D(match.trans), m_cfg.reloc_submap_radius,
                         /*max_points=*/10000)
            : m_map;
        if (!target || target->size() < 100) target = m_map;
        m_gicp.setInputSource(scan_icp);
        m_gicp.setInputTarget(target);
        CloudType aligned;
        m_gicp.align(aligned, guess2);
        if (!m_gicp.hasConverged()) continue;

        const Eigen::Matrix4d T = m_gicp.getFinalTransformation().cast<double>();
        const CoverageStats cs = evaluateCoverage(m_kdt, *scan_down, T, m_cfg.voxel_resolution);
        if (cs.coverage < m_cfg.min_score) continue;

        GlobalCandidate out_c;
        out_c.pose = matrixToPose(T);
        out_c.score = cs.coverage;
        out_c.id = static_cast<int>(hits[ci].idx);
        out.push_back(std::move(out_c));
        ++refined;
        // 高置信命中不必再试其它候选; 但至少给 2 个, 否则管线拿不到"次优分数",
        // 自相似场景里的歧义门控 (`best - second < margin`) 就失效了.
        if (cs.coverage >= 0.88 && refined >= std::min(2, max_k)) break;

    }
    std::sort(out.begin(), out.end(),
              [](const GlobalCandidate& a, const GlobalCandidate& b) { return a.score > b.score; });
    if (static_cast<int>(out.size()) > max_k) out.resize(max_k);
    return static_cast<int>(out.size());
}

bool ScanContextGlobalLocalizer::localize(const CloudType::Ptr& scan, GlobalCandidate& out) {
    std::vector<GlobalCandidate> cands;
    if (localizeTopK(scan, 1, cands) < 1) return false;
    out = cands.front();
    return true;
}

} // namespace rosiwit_slam
