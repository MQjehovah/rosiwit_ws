#include "io/robot_map_export.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rosiwit_slam {
namespace io {

namespace {

// ---------------------------------------------------------------------------
// 真机验证的导出规则（改之前先看 docs/robot_map_export.md）
// ---------------------------------------------------------------------------
// 1) z 锚点 = **地图原点附近的地面** 平移到 0。
//    不能用"全局最低强峰"：图一旦有 z 漂移（长坡道/回环），全局峰值会被漂移区
//    拉走。2026-11-19 genz 图实测 detectFloorZ 取到 -3.93，把整图抬高 3.93 m，
//    原点地面变成 +2.36 -> 重定位 fitness 0.68，永远达不到 0.89 -> 一直"定位丢失"。
// 2) 关键帧点 = **关键帧周围 R 米内的整片点云**（自适应体素压到点数上限），
//    不是"只拥有最近邻格子里的点"。
//    原因：机器人 localization_conf.json 的 kf_map_filter_num=2 会**隔一个丢一个
//    关键帧**。每帧只带 1 m 格子时，丢一半等于在地图上开 1 m 宽的沟：覆盖下降
//    49%，fitness 卡在 0.78；改成整片点云后 fitness 0.96、inlier 0.996。
// 3) 关键帧位姿扁平 z=0/roll=pitch=0，角度毫弧度，点=关键帧局部坐标。
// ---------------------------------------------------------------------------

constexpr double kGroundRadius = 12.0;   // 原点地面统计半径 (m)
constexpr double kKfRadius     = 8.0;    // 每帧覆盖半径 (m)
constexpr size_t kKfMaxPoints  = 2400;   // 每帧点数上限（自适应体素）
constexpr size_t kGroundMinPts = 200;    // 原点附近至少这么多点才可信
constexpr double kMaxKfGapWarn = 3.0;    // 隔帧后关键帧最大间隙告警阈值 (m)

inline long long cellKey(int cx, int cy) {
    return (static_cast<long long>(cx) << 32) ^ static_cast<unsigned int>(cy);
}

// 分位数（q=0.02 -> 地面）
double percentileOf(std::vector<float>& v, double q) {
    if (v.empty()) return 0.0;
    const size_t i = std::min(v.size() - 1, static_cast<size_t>(q * (v.size() - 1)));
    std::nth_element(v.begin(), v.begin() + i, v.end());
    return static_cast<double>(v[i]);
}

// 自适应体素：从 0.3 m 起，超过上限就放大 leaf 重来
void voxelDownsample(const std::vector<Eigen::Vector3f>& cand, size_t cap,
                     std::vector<Eigen::Vector3f>& outv) {
    outv.clear();
    double leaf = 0.30;
    for (int attempt = 0; attempt < 6; ++attempt) {
        outv.clear();
        std::unordered_set<long long> seen;
        seen.reserve(cand.size() * 2 + 1);
        const double inv = 1.0 / leaf;
        for (const auto& p : cand) {
            const long long kx = static_cast<long long>(std::floor(p.x() * inv));
            const long long ky = static_cast<long long>(std::floor(p.y() * inv));
            const long long kz = static_cast<long long>(std::floor(p.z() * inv));
            const long long key = (kx * 73856093LL) ^ (ky * 19349663LL) ^ (kz * 83492791LL);
            if (seen.insert(key).second) outv.push_back(p);
            if (outv.size() >= cap * 4) break;   // 早停，下一轮换更大 leaf
        }
        if (outv.size() <= cap) return;
        leaf *= 1.35;
    }
    if (outv.size() > cap) outv.resize(cap);
}

} // namespace

bool exportRobotXzMap(const std::string& path,
                      const std::string& map_name,
                      const std::vector<RobotKfSample>& track,
                      const std::vector<Eigen::Vector3f>& world_points,
                      double kf_step,
                      size_t* out_kf,
                      size_t* out_pts) {
    if (track.empty() || world_points.empty()) {
        std::cerr << "[robot_map_export] 空的轨迹或点云，跳过导出" << std::endl;
        return false;
    }

    // ---- 关键帧: 沿轨迹按距离抽稀 ----
    const double step = std::max(0.05, kf_step);
    std::vector<RobotKfSample> kfs;
    kfs.reserve(track.size());
    for (const auto& s : track) {
        if (kfs.empty()) { kfs.push_back(s); continue; }
        const auto& last = kfs.back();
        if (std::hypot(s.x - last.x, s.y - last.y) >= step) kfs.push_back(s);
    }
    if (kfs.size() < 2) kfs = track;
    if (kfs.empty()) return false;

    // ---- 世界点建 1 m 网格索引（只做一次）+ 原点附近的 z ----
    std::unordered_map<long long, std::vector<int>> grid;
    grid.reserve(world_points.size() / 4 + 16);
    std::vector<float> zs_origin;
    zs_origin.reserve(4096);
    const double gx = kfs.front().x, gy = kfs.front().y;
    for (size_t i = 0; i < world_points.size(); ++i) {
        const auto& p = world_points[i];
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) continue;
        grid[cellKey(static_cast<int>(std::floor(p.x())),
                     static_cast<int>(std::floor(p.y())))]
            .push_back(static_cast<int>(i));
        if (std::hypot(static_cast<double>(p.x()) - gx,
                       static_cast<double>(p.y()) - gy) <= kGroundRadius) {
            zs_origin.push_back(p.z());
        }
    }

    // ---- 地面: 原点附近 z 的 2% 分位（真机地图的地面都在 0 附近）----
    double ground_z;
    const bool ground_from_origin = zs_origin.size() >= kGroundMinPts;
    if (ground_from_origin) {
        ground_z = percentileOf(zs_origin, 0.02);
    } else {
        std::vector<float> all;
        all.reserve(world_points.size());
        for (const auto& p : world_points)
            if (std::isfinite(p.z())) all.push_back(p.z());
        ground_z = percentileOf(all, 0.02);
        std::cerr << "[robot_map_export] WARN 原点附近点不足(" << zs_origin.size()
                  << ")，地面退回全局 2% 分位\n";
    }

    // ---- 逐关键帧取半径内点云 ----
    std::vector<XzMapKeyframe> out(kfs.size());
    const int rcell = static_cast<int>(std::ceil(kKfRadius));
    const double r2 = kKfRadius * kKfRadius;
    std::vector<Eigen::Vector3f> cand, sel;
    size_t total = 0;
    for (size_t i = 0; i < kfs.size(); ++i) {
        out[i].pose.x = kfs[i].x;
        out[i].pose.y = kfs[i].y;
        out[i].pose.z = 0.0;
        out[i].pose.roll = 0.0;
        out[i].pose.pitch = 0.0;
        out[i].pose.yaw = kfs[i].yaw;   // 弧度，writeXzMap 会 ×1000 存毫弧度

        cand.clear();
        const int cx = static_cast<int>(std::floor(kfs[i].x));
        const int cy = static_cast<int>(std::floor(kfs[i].y));
        for (int dx = -rcell; dx <= rcell; ++dx) {
            for (int dy = -rcell; dy <= rcell; ++dy) {
                auto it = grid.find(cellKey(cx + dx, cy + dy));
                if (it == grid.end()) continue;
                for (int pi : it->second) {
                    const auto& p = world_points[pi];
                    const double ex = static_cast<double>(p.x()) - kfs[i].x;
                    const double ey = static_cast<double>(p.y()) - kfs[i].y;
                    if (ex * ex + ey * ey <= r2) cand.push_back(p);
                }
            }
        }
        if (cand.empty()) continue;

        voxelDownsample(cand, kKfMaxPoints, sel);

        const double yaw = kfs[i].yaw;
        const double cs = std::cos(yaw), sn = std::sin(yaw);
        out[i].points.reserve(sel.size());
        for (const auto& p : sel) {
            const double ex = static_cast<double>(p.x()) - kfs[i].x;
            const double ey = static_cast<double>(p.y()) - kfs[i].y;
            // 关键帧局部坐标（只按 yaw 旋转）+ 地面归零
            out[i].points.emplace_back(static_cast<float>(cs * ex + sn * ey),
                                       static_cast<float>(-sn * ex + cs * ey),
                                       static_cast<float>(static_cast<double>(p.z()) - ground_z));
        }
        total += out[i].points.size();
    }

    out.erase(std::remove_if(out.begin(), out.end(),
                             [](const XzMapKeyframe& k) { return k.points.empty(); }),
              out.end());
    if (out.empty()) return false;

    // ---- 自检: 本地化会隔帧(kf_map_filter_num=2)保留, 量一下隔帧后的最大空隙 ----
    double max_gap = 0.0, med_gap = 0.0;
    {
        std::vector<double> gaps;
        gaps.reserve(out.size());
        for (size_t i = 0; i < out.size(); ++i) {
            double best = std::numeric_limits<double>::max();
            for (size_t j = 0; j < out.size(); j += 2) {   // 本地化只保留偶数号
                if (i == j) { best = 0.0; break; }
                const double d = std::hypot(out[i].pose.x - out[j].pose.x,
                                            out[i].pose.y - out[j].pose.y);
                if (d < best) best = d;
            }
            if (std::isfinite(best)) gaps.push_back(best);
        }
        if (!gaps.empty()) {
            std::sort(gaps.begin(), gaps.end());
            max_gap = gaps.back();
            med_gap = gaps[gaps.size() / 2];
        }
    }

    long long pts = 0;
    const bool ok = writeXzMap(path, map_name, out, &pts);
    if (ok) {
        std::vector<size_t> per;
        per.reserve(out.size());
        for (const auto& k : out) per.push_back(k.points.size());
        std::sort(per.begin(), per.end());
        const size_t med_pts = per.empty() ? 0 : per[per.size() / 2];

        std::cout << "[robot_map_export] " << path << " kf=" << out.size()
                  << " pts=" << pts << " step=" << step << "m"
                  << " ground_z=" << ground_z
                  << " (" << (ground_from_origin ? "origin" : "global") << ")"
                  << " pts/kf_med=" << med_pts
                  << " gap@kf_filter2 med=" << med_gap << " max=" << max_gap << "m"
                  << std::endl;
        if (med_pts < 400) {
            std::cerr << "[robot_map_export] WARN pts/kf 中位 " << med_pts
                      << " 偏低：kf_map_filter_num 隔帧后会出空洞（真机症状 fitness 卡 0.75）\n";
        }
        if (max_gap > kMaxKfGapWarn) {
            std::cerr << "[robot_map_export] WARN 隔帧后关键帧最大间隙 " << max_gap
                      << " m > " << kMaxKfGapWarn << " m，覆盖可能不足\n";
        }
    }
    if (out_kf) *out_kf = out.size();
    if (out_pts) *out_pts = static_cast<size_t>(pts);
    return ok;
}

} // namespace io
} // namespace rosiwit_slam
