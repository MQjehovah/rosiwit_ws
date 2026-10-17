// include/algorithms/localization/global/bbs_global_localizer.h
#pragma once
#include <vector>
#include <queue>
#include <Eigen/Eigen>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include "slam_core/i_global_localizer.h"

namespace rosiwit_slam {

/// BBS (Branch-and-Bound Search) 全局重定位 (x,y,yaw).
/// 参考 Cartographer 实时回环检测: 将地图 z 带 [-1,1] 投影为 2D 命中密度栅格,
/// 逐层 2x2 最大池化构建金字塔(粗层分数是细层上界→可剪枝), 在 (x,y) 上分支定界,
/// 航向在叶层离散; 命中后以 2D 结果为初值 GICP 精配准到全图.
class BbsGlobalLocalizer : public IGlobalLocalizer {
public:
    std::string name() const override { return "bbs"; }
    void configure(const GlobalLocalizeConfig& cfg) override;
    bool setMap(const std::string& map_path, const CloudType::Ptr& global_map) override;
    bool localize(const CloudType::Ptr& scan, GlobalCandidate& out) override;
    int localizeTopK(const CloudType::Ptr& scan, int max_k,
                     std::vector<GlobalCandidate>& out) override;
    void updateEnvironment(const CloudType::Ptr& overlay) override;

private:
    struct Level {
        std::vector<float> grid;
        int w = 0, h = 0;
        double res = 0.0;
        float at(int col, int row) const {
            if (col < 0 || row < 0 || col >= w || row >= h) return 0.0f;
            return grid[static_cast<size_t>(row) * w + col];
        }
    };
    struct Node { double score; int level; int x; int y; int theta; };
    struct NodeCmp { bool operator()(const Node& a, const Node& b) const { return a.score < b.score; } };

    double scoreNode(const Level& lv, int x, int y, int theta) const;
    bool projectScan(const CloudType::Ptr& scan);
    bool rebuild(const CloudType::Ptr& src);
    void searchLeaves(int max_k, std::vector<Node>& out);
    /// 车体系投影支持: base<-lidar 安装旋转
    CloudType::Ptr toCarFrame(const CloudType::Ptr& in) const;
    PoseStamped toLidarPose(const PoseStamped& in) const;
    Eigen::Matrix3d m_r_bl = Eigen::Matrix3d::Identity();
    bool m_have_mount = false;

    bool refineCandidate(const Node& n, const CloudType::Ptr& scan_down, GlobalCandidate& out);

    GlobalLocalizeConfig m_cfg;
    CloudType::Ptr m_base_map;     // 原始地图(冻结), 供 lifelong 刷新时叠加
    std::vector<Level> m_pyr;      // [0]=最细, back()=最粗
    double m_ox = 0.0, m_oy = 0.0; // 栅格世界原点
    double m_res = 0.5;            // 最细分辨率
    int m_theta_bins = 60;
    double m_theta_res = 0.0;
    std::vector<Eigen::Vector2d> m_scan2d;
    CloudType::Ptr m_map;
    CloudType::Ptr m_map_down;
    pcl::KdTreeFLANN<PointType> m_kdtree;
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> m_gicp;
    bool m_ready = false;
};

} // namespace rosiwit_slam
