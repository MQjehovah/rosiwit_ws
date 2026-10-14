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

    m_ready = true;
    std::cout << "[SCGlobal] loaded db: " << m_db.size() << " kfs" << std::endl;
    return true;
}

bool ScanContextGlobalLocalizer::localize(const CloudType::Ptr& scan, GlobalCandidate& out) {
    if (!m_ready || !m_sc || !scan || scan->empty()) return false;

    const ScanContext::Descriptor cur = m_sc->makeDescriptor(scan);

    struct Candidate { size_t idx; double rk; };
    std::vector<Candidate> cands;
    cands.reserve(m_db.size());
    for (size_t i = 0; i < m_db.size(); ++i) {
        cands.push_back({i, ScanContext::ringKeyDistance(m_db[i].desc.ring_key, cur.ring_key)});
    }
    std::sort(cands.begin(), cands.end(),
              [](const Candidate& a, const Candidate& b) { return a.rk < b.rk; });
    const size_t num = std::max<size_t>(1, static_cast<size_t>(cands.size() * m_cfg.sc_candidate_ratio));

    int best_idx = -1, best_shift = 0;
    double best_dist = m_cfg.sc_dist_thresh;
    for (size_t ci = 0; ci < num; ++ci) {
        int shift = 0;
        double d = ScanContext::distance(m_db[cands[ci].idx].desc, cur, &shift);
        if (d < best_dist) { best_dist = d; best_idx = static_cast<int>(cands[ci].idx); best_shift = shift; }
    }
    if (best_idx < 0) return false;

    const auto& match = m_db[best_idx];
    const double yaw = 2.0 * M_PI * best_shift / m_cfg.sc_num_sectors;
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    guess.block<3,3>(0,0) = (match.rot * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                             .toRotationMatrix()).cast<float>();
    guess.block<3,1>(0,3) = match.trans.cast<float>();

    CloudType::Ptr scan_down = downsampleCloud(scan, m_cfg.voxel_resolution);
    if (!scan_down || scan_down->size() < 10) return false;

    m_gicp.setInputSource(scan_down);
    m_gicp.setInputTarget(m_map);
    CloudType aligned;
    m_gicp.align(aligned, guess);
    if (!m_gicp.hasConverged()) return false;

    const Eigen::Matrix4d T = m_gicp.getFinalTransformation().cast<double>();
    pcl::KdTreeFLANN<PointType> kdt;
    kdt.setInputCloud(m_map);
    const CoverageStats cs = evaluateCoverage(kdt, *scan_down, T, m_cfg.voxel_resolution);
    if (cs.coverage < m_cfg.min_score) return false;

    out.pose = matrixToPose(T);
    out.score = cs.coverage;
    out.id = best_idx;
    return true;
}

} // namespace rosiwit_slam
