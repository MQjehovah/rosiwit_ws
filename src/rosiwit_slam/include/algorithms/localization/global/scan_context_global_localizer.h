// include/algorithms/localization/global/scan_context_global_localizer.h
#pragma once
#include <vector>
#include <pcl/registration/gicp.h>
#include "slam_core/i_global_localizer.h"
#include "algorithms/scan_context/scan_context.h"

namespace rosiwit_slam {

/// 关闭全局重定位 (仅依赖外部初值)
class NoGlobalLocalizer : public IGlobalLocalizer {
public:
    std::string name() const override { return "none"; }
    bool setMap(const std::string&, const CloudType::Ptr&) override { return true; }
    bool localize(const CloudType::Ptr&, GlobalCandidate&) override { return false; }
};

/// ScanContext 全局重定位: 描述子库召回 + GICP 精配准.
/// 复用建图侧落盘的 map.pcd + ".scdb".
class ScanContextGlobalLocalizer : public IGlobalLocalizer {
public:
    std::string name() const override { return "scan_context"; }
    void configure(const GlobalLocalizeConfig& cfg) override;
    bool setMap(const std::string& map_path, const CloudType::Ptr& global_map) override;
    bool localize(const CloudType::Ptr& scan, GlobalCandidate& out) override;
    int localizeTopK(const CloudType::Ptr& scan, int max_k,
                     std::vector<GlobalCandidate>& out) override;

private:
    GlobalLocalizeConfig m_cfg;
    std::shared_ptr<ScanContext> m_sc;
    std::vector<ScanContextDatabaseEntry> m_db;
    CloudType::Ptr m_map;
    /// 粗层地图 (0.6 m): 两级精配的第一级只在这里跑, 大多数错误候选在这一步被淘汰
    CloudType::Ptr m_map_coarse;
    pcl::KdTreeFLANN<PointType> m_kdt;         // m_map 上建一次, 供裁子图/打分
    pcl::KdTreeFLANN<PointType> m_kdt_coarse;  // m_map_coarse
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> m_gicp;
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> m_gicp_coarse;
    bool m_ready = false;
};

} // namespace rosiwit_slam
