// include/algorithms/localization/global/fpfh_ransac_global_localizer.h
#pragma once
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include "slam_core/i_global_localizer.h"

namespace rosiwit_slam {

/// FPFH 特征 + RANSAC 6DoF 全局重定位 (PCL SampleConsensusPrerejective).
/// 相比 BBS/ScanContext 只能恢复 (x,y,yaw), 本策略可恢复完整 6DoF (含 roll/pitch/z),
/// 适合 3D 场景冷启动; 命中后 GICP 精配准到全图. 计算量大, 建议仅在初始化时调用.
class FpfhRansacGlobalLocalizer : public IGlobalLocalizer {
public:
    std::string name() const override { return "fpfh_ransac"; }
    void configure(const GlobalLocalizeConfig& cfg) override;
    bool setMap(const std::string& map_path, const CloudType::Ptr& global_map) override;
    bool localize(const CloudType::Ptr& scan, GlobalCandidate& out) override;

    /// 多候选: 随机子集多轮 RANSAC, 按 3m 聚类去重, 每簇取分最高者。
    /// 2026-10-17 事故教训: 基类默认实现只有单候选, 地库重复结构下单候选 0.999
    /// 锁进相似车位(推行丢失事故), 上层连"存在歧义"都不知道。多候选是消歧前提。
    int localizeTopK(const CloudType::Ptr& scan, int max_k,
                     std::vector<GlobalCandidate>& out) override;

private:
    GlobalLocalizeConfig m_cfg;
    CloudType::Ptr m_map_down;                              // 供 GICP 精配准
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_map_xyz;          // 供 FPFH
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr m_map_fpfh;
    pcl::KdTreeFLANN<PointType> m_kdtree;                   // 覆盖率评估
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> m_gicp;
    bool m_ready = false;
};

} // namespace rosiwit_slam
