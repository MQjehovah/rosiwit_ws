// src/algorithms/localization/global/fpfh_ransac_global_localizer.cpp
#include "algorithms/localization/global/fpfh_ransac_global_localizer.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/registration_metrics.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>
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

} // namespace rosiwit_slam
