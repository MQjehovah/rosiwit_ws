// src/algorithms/localization/global/bbs_global_localizer.cpp
#include "algorithms/localization/global/bbs_global_localizer.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/registration_metrics.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace rosiwit_slam {

void BbsGlobalLocalizer::configure(const GlobalLocalizeConfig& cfg) {
    m_cfg = cfg;
    m_res = cfg.voxel_resolution > 0 ? cfg.voxel_resolution : 0.5;
    m_theta_bins = std::max(8, cfg.bbs_theta_bins);
    m_theta_res = 2.0 * M_PI / m_theta_bins;
    m_gicp.setMaximumIterations(cfg.icp_max_iter);
    m_gicp.setMaxCorrespondenceDistance(cfg.icp_max_corr_dist);
    m_gicp.setTransformationEpsilon(1e-8);
    m_gicp.setEuclideanFitnessEpsilon(1e-6);
}

bool BbsGlobalLocalizer::setMap(const std::string& map_path,
                                const CloudType::Ptr& global_map) {
    (void)map_path;
    m_ready = false;
    m_pyr.clear();
    if (!global_map || global_map->empty()) return false;
    m_map = filterFinite(global_map);

    // 1) z 带投影到 2D 命中计数
    const double zmin = m_cfg.bbs_z_min, zmax = m_cfg.bbs_z_max;
    double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(m_map->size());
    for (const auto& p : m_map->points) {
        if (p.z < zmin || p.z > zmax) continue;
        pts.emplace_back(p.x, p.y);
        min_x = std::min(min_x, static_cast<double>(p.x));
        max_x = std::max(max_x, static_cast<double>(p.x));
        min_y = std::min(min_y, static_cast<double>(p.y));
        max_y = std::max(max_y, static_cast<double>(p.y));
    }
    if (pts.size() < 64) return false;

    m_ox = min_x;
    m_oy = min_y;
    const int w0 = static_cast<int>((max_x - min_x) / m_res) + 1;
    const int h0 = static_cast<int>((max_y - min_y) / m_res) + 1;
    if (w0 <= 0 || h0 <= 0) return false;

    Level l0;
    l0.w = w0; l0.h = h0; l0.res = m_res;
    l0.grid.assign(static_cast<size_t>(w0) * h0, 0.0f);
    for (const auto& p : pts) {
        const int col = static_cast<int>((p.x() - m_ox) / m_res);
        const int row = static_cast<int>((p.y() - m_oy) / m_res);
        if (col < 0 || row < 0 || col >= w0 || row >= h0) continue;
        l0.grid[static_cast<size_t>(row) * w0 + col] += 1.0f;
    }
    const float denom = static_cast<float>(std::max(1.0, m_cfg.bbs_min_points_per_cell));
    for (auto& v : l0.grid) v = std::min(1.0f, v / denom);
    m_pyr.push_back(std::move(l0));

    // 2) 2x2 最大池化金字塔
    const int target_levels = std::max(1, std::min(8, m_cfg.bbs_max_levels));
    for (int lv = 1; lv < target_levels; ++lv) {
        const Level& prev = m_pyr.back();
        if (prev.w <= 1 || prev.h <= 1) break;
        Level cur;
        cur.w = (prev.w + 1) / 2;
        cur.h = (prev.h + 1) / 2;
        cur.res = prev.res * 2.0;
        cur.grid.assign(static_cast<size_t>(cur.w) * cur.h, 0.0f);
        for (int r = 0; r < cur.h; ++r)
            for (int c = 0; c < cur.w; ++c) {
                float m = 0.0f;
                for (int dr = 0; dr < 2; ++dr)
                    for (int dc = 0; dc < 2; ++dc) {
                        const int pr = 2 * r + dr, pc = 2 * c + dc;
                        if (pr < prev.h && pc < prev.w)
                            m = std::max(m, prev.grid[static_cast<size_t>(pr) * prev.w + pc]);
                    }
                cur.grid[static_cast<size_t>(r) * cur.w + c] = m;
            }
        m_pyr.push_back(std::move(cur));
    }

    // 3) 全图用于 GICP 精配准与覆盖率评估
    m_map_down = downsampleCloud(m_map, m_res);
    m_kdtree.setInputCloud(m_map_down);

    m_ready = true;
    std::cout << "[BBS] map " << w0 << "x" << h0 << " res=" << m_res
              << " levels=" << m_pyr.size() << std::endl;
    return true;
}

bool BbsGlobalLocalizer::projectScan(const CloudType::Ptr& scan) {
    m_scan2d.clear();
    if (!scan) return false;
    const double zmin = m_cfg.bbs_z_min, zmax = m_cfg.bbs_z_max;
    std::vector<Eigen::Vector2d> pts;
    for (const auto& p : scan->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.z < zmin || p.z > zmax) continue;
        pts.emplace_back(p.x, p.y);
    }
    if (pts.size() < 10) return false;
    const size_t step = std::max<size_t>(1, pts.size() / 1500);
    for (size_t i = 0; i < pts.size(); i += step) m_scan2d.push_back(pts[i]);
    return m_scan2d.size() >= 10;
}

double BbsGlobalLocalizer::scoreNode(const Level& lv, int xi, int yi, int theta) const {
    const double tx = m_ox + xi * lv.res;
    const double ty = m_oy + yi * lv.res;
    const double ang = theta * m_theta_res;
    const double c = std::cos(ang), s = std::sin(ang);
    double sum = 0.0;
    for (const auto& p : m_scan2d) {
        const double x = c * p.x() - s * p.y() + tx;
        const double y = s * p.x() + c * p.y() + ty;
        const int col = static_cast<int>(std::floor((x - m_ox) / lv.res));
        const int row = static_cast<int>(std::floor((y - m_oy) / lv.res));
        sum += lv.at(col, row);
    }
    return sum;
}

bool BbsGlobalLocalizer::localize(const CloudType::Ptr& scan, GlobalCandidate& out) {
    if (!m_ready || m_pyr.empty()) return false;
    if (!projectScan(scan)) return false;

    const int top = static_cast<int>(m_pyr.size()) - 1;
    const Level& coarse = m_pyr[top];

    // 初始搜索集: 最粗层全枚举 (x, y, theta)
    std::priority_queue<Node, std::vector<Node>, NodeCmp> q;
    for (int theta = 0; theta < m_theta_bins; ++theta)
        for (int x = 0; x < coarse.w; ++x)
            for (int y = 0; y < coarse.h; ++y) {
                const double s = scoreNode(coarse, x, y, theta);
                if (s > 0.0) q.push({s, top, x, y, theta});
            }
    if (q.empty()) return false;

    // 分支定界
    Node best{-1.0, -1, 0, 0, 0};
    while (!q.empty()) {
        const Node n = q.top(); q.pop();
        if (n.score < best.score) break;          // 上界剪枝 (max-heap)
        if (n.level == 0) { best = n; break; }     // 首个叶节点即最优
        const Level& child = m_pyr[n.level - 1];
        for (int dx = 0; dx < 2; ++dx)
            for (int dy = 0; dy < 2; ++dy) {
                const int cx = 2 * n.x + dx;
                const int cy = 2 * n.y + dy;
                if (cx >= child.w || cy >= child.h) continue;
                const double s = scoreNode(child, cx, cy, n.theta);
                if (s > 0.0) q.push({s, n.level - 1, cx, cy, n.theta});
            }
    }
    if (best.level != 0) return false;

    // 2D 结果 → 3D 位姿 (z=0, roll/pitch=0)
    const double yaw = best.theta * m_theta_res;
    PoseStamped guess;
    guess.trans = V3D(m_ox + best.x * m_res, m_oy + best.y * m_res, 0.0);
    guess.rot = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix();

    // GICP 精配准到全图
    CloudType::Ptr scan_down = downsampleCloud(scan, m_res);
    if (!scan_down || scan_down->size() < 10) return false;
    m_gicp.setInputSource(scan_down);
    m_gicp.setInputTarget(m_map_down);
    CloudType aligned;
    m_gicp.align(aligned, poseToMatrix(guess).cast<float>());

    Eigen::Matrix4d T = poseToMatrix(guess);
    double score = static_cast<double>(best.score);
    if (m_gicp.hasConverged()) {
        T = m_gicp.getFinalTransformation().cast<double>();
        const CoverageStats cs = evaluateCoverage(m_kdtree, *scan_down, T, m_res);
        score = cs.coverage;
        if (cs.coverage < m_cfg.min_score) return false;
    } else if (best.score <= 0.0) {
        return false;
    }

    out.pose = matrixToPose(T);
    out.score = score;
    out.id = -1;
    return true;
}

} // namespace rosiwit_slam
