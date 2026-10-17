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

    // 安装旋转 base<-lidar (rpy 度, ZYX)。BBS 的 2D(yaw-only) 模型要求"水平安装":
    // 雷达有安装角时(本机 pitch 41.67°), 先把扫描/地图都转到 base 系再投影,
    // 最后把候选位姿换回雷达系(见 toLidarPose)。全部为 0 时等价于原行为。
    m_r_bl = (Eigen::AngleAxisd(m_cfg.bbs_mount_yaw_deg * M_PI / 180.0, V3D::UnitZ())
              * Eigen::AngleAxisd(m_cfg.bbs_mount_pitch_deg * M_PI / 180.0, V3D::UnitY())
              * Eigen::AngleAxisd(m_cfg.bbs_mount_roll_deg * M_PI / 180.0, V3D::UnitX()))
                 .toRotationMatrix();
    {
        const auto& rr = m_cfg.bbs_mount_r;
        const bool identity = std::fabs(rr[0] - 1.0) < 1e-9 && std::fabs(rr[4] - 1.0) < 1e-9 &&
                              std::fabs(rr[8] - 1.0) < 1e-9 && std::fabs(rr[1]) < 1e-9 &&
                              std::fabs(rr[2]) < 1e-9 && std::fabs(rr[3]) < 1e-9 &&
                              std::fabs(rr[5]) < 1e-9 && std::fabs(rr[6]) < 1e-9 &&
                              std::fabs(rr[7]) < 1e-9;
        if (!identity) {
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) m_r_bl(r, c) = rr[r * 3 + c];
            std::cout << "[BBS] mount from matrix (TF): "
                      << m_r_bl.row(0) << " | " << m_r_bl.row(1) << " | "
                      << m_r_bl.row(2) << std::endl;
        }
    }
    m_have_mount = (std::fabs(m_cfg.bbs_mount_roll_deg) + std::fabs(m_cfg.bbs_mount_pitch_deg)
                    + std::fabs(m_cfg.bbs_mount_yaw_deg)) > 1e-6;
    if (!m_have_mount) {
        std::cout << "[BBS] WARN: 未获得安装变换(也没注入 bbs_mount_r) -> 按雷达系投影; "
                     "俯装/侧装雷达下候选会偏, 请改用 fpfh_ransac 或修好 TF" << std::endl;
    }
    std::cout << "[BBS] mount rpy(deg)=[" << m_cfg.bbs_mount_roll_deg << " "
              << m_cfg.bbs_mount_pitch_deg << " " << m_cfg.bbs_mount_yaw_deg
              << "] car-frame projection=" << (m_have_mount ? 1 : 0) << std::endl;
}

CloudType::Ptr BbsGlobalLocalizer::toCarFrame(const CloudType::Ptr& in) const {
    if (!m_have_mount || !in) return in;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = m_r_bl.cast<float>();
    CloudType::Ptr out(new CloudType());
    pcl::transformPointCloud(*in, *out, T);
    return out;
}

PoseStamped BbsGlobalLocalizer::toLidarPose(const PoseStamped& in) const {
    if (!m_have_mount) return in;
    // T_mapL<-curL = R_bl^-1 * T_mapB<-curB * R_bl  (conjugation: 两帧相差一个常量安装旋转)
    const Eigen::Matrix3d Rb = m_r_bl;
    PoseStamped out = in;
    out.rot = Rb.transpose() * in.rot * Rb;
    out.trans = Rb.transpose() * in.trans;
    return out;
}

bool BbsGlobalLocalizer::setMap(const std::string& map_path,
                                const CloudType::Ptr& global_map) {
    (void)map_path;
    if (!global_map || global_map->empty()) return false;
    m_base_map = filterFinite(global_map);
    return rebuild(m_base_map);
}

void BbsGlobalLocalizer::updateEnvironment(const CloudType::Ptr& overlay) {
    if (!m_base_map) return;
    CloudType::Ptr merged(new CloudType());
    *merged = *m_base_map;
    if (overlay && !overlay->empty()) *merged += *overlay;
    rebuild(merged);
}

bool BbsGlobalLocalizer::rebuild(const CloudType::Ptr& src) {
    m_ready = false;
    m_pyr.clear();
    if (!src || src->empty()) return false;
    if (m_have_mount) {
        m_map = toCarFrame(src);       // 2D 金字塔 + GICP 目标都在车体系
    } else {
        m_map = src;
    }

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
              << " origin=(" << m_ox << ", " << m_oy << ")"
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

void BbsGlobalLocalizer::searchLeaves(int max_k, std::vector<Node>& out) {
    out.clear();
    if (m_pyr.empty()) return;
    const int top = static_cast<int>(m_pyr.size()) - 1;
    const Level& coarse = m_pyr[top];

    std::priority_queue<Node, std::vector<Node>, NodeCmp> q;
    for (int theta = 0; theta < m_theta_bins; ++theta)
        for (int x = 0; x < coarse.w; ++x)
            for (int y = 0; y < coarse.h; ++y) {
                const double s = scoreNode(coarse, x, y, theta);
                if (s > 0.0) q.push({s, top, x, y, theta});
            }
    if (q.empty()) return;

    const double keep_ratio = 0.85;  // 只保留接近最优的叶节点作为候选
    double best = -1.0;
    while (!q.empty() && static_cast<int>(out.size()) < max_k) {
        const Node n = q.top(); q.pop();
        if (best >= 0.0 && n.score < best * keep_ratio) break;
        if (n.level == 0) { out.push_back(n); if (n.score > best) best = n.score; continue; }
        const Level& child = m_pyr[n.level - 1];
        for (int dx = 0; dx < 2; ++dx)
            for (int dy = 0; dy < 2; ++dy) {
                const int cx = 2 * n.x + dx, cy = 2 * n.y + dy;
                if (cx >= child.w || cy >= child.h) continue;
                const double s = scoreNode(child, cx, cy, n.theta);
                if (s > 0.0) q.push({s, n.level - 1, cx, cy, n.theta});
            }
    }
}

bool BbsGlobalLocalizer::refineCandidate(const Node& n, const CloudType::Ptr& scan_down,
                                         GlobalCandidate& out) {
    const double yaw = n.theta * m_theta_res;
    PoseStamped guess;
    guess.trans = V3D(m_ox + n.x * m_res, m_oy + n.y * m_res, 0.0);
    guess.rot = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix();

    m_gicp.setInputSource(scan_down);
    m_gicp.setInputTarget(m_map_down);
    CloudType aligned;
    m_gicp.align(aligned, poseToMatrix(guess).cast<float>());

    Eigen::Matrix4d T = poseToMatrix(guess);
    double score = static_cast<double>(n.score);
    if (m_gicp.hasConverged()) {
        T = m_gicp.getFinalTransformation().cast<double>();
        const CoverageStats cs = evaluateCoverage(m_kdtree, *scan_down, T, m_res);
        score = cs.coverage;
    }
    out.pose = matrixToPose(T);
    out.score = score;
    out.id = -1;
    return true;
}

int BbsGlobalLocalizer::localizeTopK(const CloudType::Ptr& scan, int max_k,
                                     std::vector<GlobalCandidate>& out) {
    out.clear();
    if (!m_ready || m_pyr.empty()) return 0;
    CloudType::Ptr scan_c = toCarFrame(scan);   // 与地图同一(车体)系
    if (!projectScan(scan_c)) return 0;

    CloudType::Ptr scan_down = downsampleCloud(scan_c, m_res);
    if (!scan_down || scan_down->size() < 10) return 0;

    std::vector<Node> leaves;
    searchLeaves(std::max(1, max_k * 2), leaves);
    for (const auto& n : leaves) {
        GlobalCandidate c;
        refineCandidate(n, scan_down, c);
        c.pose = toLidarPose(c.pose);   // 换回雷达系(核心/上层约定)
        if (c.score >= m_cfg.min_score) out.push_back(std::move(c));
    }
    std::sort(out.begin(), out.end(),
              [](const GlobalCandidate& a, const GlobalCandidate& b) { return a.score > b.score; });
    for (size_t i = 0; i < out.size() && i < 5; ++i) {
        const double yaw = std::atan2(out[i].pose.rot(1, 0), out[i].pose.rot(0, 0)) * 180.0 / M_PI;
        std::cout << "[BBS] cand[" << i << "] score=" << out[i].score
                  << " pose=(" << out[i].pose.trans.x() << ", " << out[i].pose.trans.y()
                  << ") yaw_deg=" << yaw << " z=" << out[i].pose.trans.z() << std::endl;
    }
    if (static_cast<int>(out.size()) > max_k) out.resize(max_k);
    return static_cast<int>(out.size());
}

bool BbsGlobalLocalizer::localize(const CloudType::Ptr& scan, GlobalCandidate& out) {
    std::vector<GlobalCandidate> cands;
    if (localizeTopK(scan, 1, cands) < 1) return false;
    out = cands.front();
    return true;
}

} // namespace rosiwit_slam
