// src/algorithms/localization/global/fpfh_ransac_global_localizer.cpp
#include "algorithms/localization/global/fpfh_ransac_global_localizer.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/registration_metrics.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>
#include <random>
#include <numeric>
#include <chrono>
#include <iostream>

namespace rosiwit_slam {

namespace {
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz, double radius) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(xyz);
    ne.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(
        new pcl::search::KdTree<pcl::PointXYZ>()));
    ne.setRadiusSearch(radius);
    ne.compute(*normals);
    return normals;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFpfh(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz,
        const pcl::PointCloud<pcl::Normal>::Ptr& normals, double radius) {
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fe;
    fe.setInputCloud(xyz);
    fe.setInputNormals(normals);
    fe.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(
        new pcl::search::KdTree<pcl::PointXYZ>()));
    fe.setRadiusSearch(radius);
    fe.compute(*fpfh);
    return fpfh;
}
} // namespace

void FpfhRansacGlobalLocalizer::configure(const GlobalLocalizeConfig& cfg) {
    m_cfg = cfg;
    m_gicp.setMaximumIterations(cfg.icp_max_iter);
    m_gicp.setMaxCorrespondenceDistance(cfg.icp_max_corr_dist);
    m_gicp.setTransformationEpsilon(1e-8);
    m_gicp.setEuclideanFitnessEpsilon(1e-6);
}

bool FpfhRansacGlobalLocalizer::setMap(const std::string& map_path,
                                       const CloudType::Ptr& global_map) {
    (void)map_path;
    m_ready = false;
    if (!global_map || global_map->empty()) return false;

    m_map_down = downsampleCloud(filterFinite(global_map),
                                 m_cfg.voxel_resolution > 0 ? m_cfg.voxel_resolution : 0.5);
    if (!m_map_down || m_map_down->size() < 64) return false;
    m_kdtree.setInputCloud(m_map_down);

    m_map_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*m_map_down, *m_map_xyz);
    const pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(m_map_xyz, m_cfg.fpfh_normal_radius);
    m_map_fpfh = computeFpfh(m_map_xyz, normals, m_cfg.fpfh_feature_radius);

    m_ready = true;
    std::cout << "[FPFH-RANSAC] map features: " << m_map_fpfh->size()
              << " pts=" << m_map_xyz->size() << std::endl;
    return true;
}

bool FpfhRansacGlobalLocalizer::localize(const CloudType::Ptr& scan, GlobalCandidate& out) {
    if (!m_ready || !scan || scan->empty()) return false;

    CloudType::Ptr scan_down = downsampleCloud(filterFinite(scan),
                                               m_cfg.voxel_resolution > 0 ? m_cfg.voxel_resolution : 0.5);
    if (!scan_down || scan_down->size() < 64) return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*scan_down, *scan_xyz);
    const pcl::PointCloud<pcl::Normal>::Ptr scan_normals = estimateNormals(scan_xyz, m_cfg.fpfh_normal_radius);
    const pcl::PointCloud<pcl::FPFHSignature33>::Ptr scan_fpfh = computeFpfh(scan_xyz, scan_normals, m_cfg.fpfh_feature_radius);

    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac;
    sac.setInputSource(scan_xyz);
    sac.setSourceFeatures(scan_fpfh);
    sac.setInputTarget(m_map_xyz);
    sac.setTargetFeatures(m_map_fpfh);
    sac.setMaximumIterations(m_cfg.fpfh_max_iter);
    sac.setNumberOfSamples(3);
    sac.setCorrespondenceRandomness(5);
    sac.setSimilarityThreshold(m_cfg.fpfh_similarity);
    sac.setMaxCorrespondenceDistance(1.5);
    sac.setInlierFraction(m_cfg.fpfh_inlier_fraction);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    sac.align(aligned);
    if (!sac.hasConverged()) return false;

    Eigen::Matrix4f guess = sac.getFinalTransformation();
    m_gicp.setInputSource(scan_down);
    m_gicp.setInputTarget(m_map_down);
    CloudType refined;
    m_gicp.align(refined, guess);

    Eigen::Matrix4d T = guess.cast<double>();
    if (m_gicp.hasConverged()) T = m_gicp.getFinalTransformation().cast<double>();

    const CoverageStats cs = evaluateCoverage(m_kdtree, *scan_down, T, m_cfg.voxel_resolution);
    if (cs.coverage < m_cfg.min_score) return false;

    out.pose = matrixToPose(T);
    out.score = cs.coverage;
    out.id = -1;
    return true;
}

int FpfhRansacGlobalLocalizer::localizeTopK(const CloudType::Ptr& scan, int max_k,
                                            std::vector<GlobalCandidate>& out) {
    out.clear();
    if (!m_ready || !scan || scan->empty() || max_k < 1) return 0;

    // 扫描特征只算一次 (原来 localize() 每次调用都重算 FPFH)
    CloudType::Ptr scan_down = downsampleCloud(filterFinite(scan),
                                               m_cfg.voxel_resolution > 0 ? m_cfg.voxel_resolution : 0.5);
    if (!scan_down || scan_down->size() < 64) return 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*scan_down, *scan_xyz);
    const pcl::PointCloud<pcl::Normal>::Ptr scan_normals =
        estimateNormals(scan_xyz, m_cfg.fpfh_normal_radius);
    const pcl::PointCloud<pcl::FPFHSignature33>::Ptr scan_fpfh =
        computeFpfh(scan_xyz, scan_normals, m_cfg.fpfh_feature_radius);

    const int n = static_cast<int>(scan_xyz->size());
    std::vector<int> idx(n);
    for (int i = 0; i < n; ++i) idx[i] = i;
    std::mt19937 rng(static_cast<uint32_t>(
        std::chrono::steady_clock::now().time_since_epoch().count() & 0xffffffff));

    // 每轮留 85% 随机子集: 不同的采样让 RANSAC 在重复结构下落到不同的"相似位置",
    // 这是产生多候选的来源 (同一份数据同一采样 = 同一个解, 跑再多轮也没用)。
    const int keep = std::max(64, static_cast<int>(0.85 * n));
    const int attempts = std::max(6, 3 * max_k);
    const auto t0 = std::chrono::steady_clock::now();
    auto budget_left = [&]() {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - t0).count() < 6000.0;
    };

    std::vector<GlobalCandidate> cands;
    for (int a = 0; a < attempts && budget_left(); ++a) {
        std::shuffle(idx.begin(), idx.end(), rng);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sub(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr sub_f(new pcl::PointCloud<pcl::FPFHSignature33>());
        sub->reserve(keep); sub_f->reserve(keep);
        for (int i = 0; i < keep; ++i) {
            sub->push_back((*scan_xyz)[idx[i]]);
            sub_f->push_back((*scan_fpfh)[idx[i]]);
        }

        pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac;
        sac.setInputSource(sub);
        sac.setSourceFeatures(sub_f);
        sac.setInputTarget(m_map_xyz);
        sac.setTargetFeatures(m_map_fpfh);
        sac.setMaximumIterations(m_cfg.fpfh_max_iter);
        sac.setNumberOfSamples(3);
        sac.setCorrespondenceRandomness(5);
        sac.setSimilarityThreshold(m_cfg.fpfh_similarity);
        sac.setMaxCorrespondenceDistance(1.5);
        sac.setInlierFraction(m_cfg.fpfh_inlier_fraction);

        pcl::PointCloud<pcl::PointXYZ> aligned;
        sac.align(aligned);
        if (!sac.hasConverged()) continue;

        Eigen::Matrix4f guess = sac.getFinalTransformation();
        m_gicp.setInputSource(scan_down);   // 精配用全量(非子集), 与 localize() 一致
        m_gicp.setInputTarget(m_map_down);
        CloudType refined;
        m_gicp.align(refined, guess);
        Eigen::Matrix4d T = guess.cast<double>();
        if (m_gicp.hasConverged()) T = m_gicp.getFinalTransformation().cast<double>();

        const CoverageStats cs = evaluateCoverage(m_kdtree, *scan_down, T, m_cfg.voxel_resolution);
        if (cs.coverage < m_cfg.min_score) continue;

        GlobalCandidate cand;
        cand.pose = matrixToPose(T);
        cand.score = cs.coverage;
        cand.id = -1;

        // 3m 聚类: 同一位置的多次命中只留分最高的; 不同位置各自成簇
        bool merged = false;
        for (auto& c : cands) {
            if ((c.pose.trans - cand.pose.trans).norm() < 3.0) {
                merged = true;
                if (cand.score > c.score) c = cand;
                break;
            }
        }
        if (!merged) cands.push_back(cand);
    }

    std::sort(cands.begin(), cands.end(), [](const GlobalCandidate& x, const GlobalCandidate& y) {
        return x.score > y.score;
    });
    if (static_cast<int>(cands.size()) > max_k) cands.resize(max_k);
    out = cands;
    std::cout << "[FPFH-RANSAC] topk: clusters=" << out.size()
              << " best=" << (out.empty() ? 0.0 : out[0].score)
              << (out.size() > 1 ? (" second=" + std::to_string(out[1].score)) : "")
              << std::endl;
    return static_cast<int>(out.size());
}

} // namespace rosiwit_slam
