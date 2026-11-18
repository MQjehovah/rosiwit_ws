#include "io/robot_map_export.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace rosiwit_slam {
namespace io {

namespace {

// 地面高度: z 直方图里最低的强峰（0.133 m 粗直方图，真机验证过；
// 细直方图会被墙角/噪点带偏 0.8 m 以上）。
double detectFloorZ(const std::vector<Eigen::Vector3f>& pts) {
    constexpr int NB = 60;
    constexpr double LO = -4.0, HI = 4.0;
    std::vector<int> hist(NB, 0);
    for (const auto& p : pts) {
        if (!std::isfinite(p.z())) continue;
        const int i = static_cast<int>((p.z() - LO) / (HI - LO) * NB);
        if (i >= 0 && i < NB) ++hist[i];
    }
    const int mx = *std::max_element(hist.begin(), hist.end());
    if (mx <= 0) return 0.0;
    for (int i = 0; i < NB; ++i) {
        if (hist[i] > 0.35 * mx) return LO + (i + 0.5) * (HI - LO) / NB;
    }
    return 0.0;
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

    const double floor_z = detectFloorZ(world_points);

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

    // ---- 点归属最近关键帧（局部坐标：只按 yaw 旋转 + 平移）----
    std::vector<XzMapKeyframe> out(kfs.size());
    for (size_t i = 0; i < kfs.size(); ++i) {
        out[i].pose.x = kfs[i].x;
        out[i].pose.y = kfs[i].y;
        out[i].pose.z = 0.0;
        out[i].pose.roll = 0.0;
        out[i].pose.pitch = 0.0;
        out[i].pose.yaw = kfs[i].yaw;   // 弧度，writeXzMap 会 ×1000 存毫弧度
    }

    for (const auto& p : world_points) {
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) continue;
        size_t best = 0;
        float best_d2 = std::numeric_limits<float>::max();
        for (size_t i = 0; i < kfs.size(); ++i) {
            const float dx = p.x() - static_cast<float>(kfs[i].x);
            const float dy = p.y() - static_cast<float>(kfs[i].y);
            const float d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best = i; }
        }
        const double yaw = kfs[best].yaw;
        const double cy = std::cos(yaw), sy = std::sin(yaw);
        const double dx = static_cast<double>(p.x()) - kfs[best].x;
        const double dy = static_cast<double>(p.y()) - kfs[best].y;
        const Eigen::Vector3f local(static_cast<float>(cy * dx + sy * dy),
                                    static_cast<float>(-sy * dx + cy * dy),
                                    static_cast<float>(static_cast<double>(p.z()) - floor_z));
        out[best].points.push_back(local);
    }

    // 丢掉空的关键帧（保留 id 连续）
    out.erase(std::remove_if(out.begin(), out.end(),
                             [](const XzMapKeyframe& k) { return k.points.empty(); }),
              out.end());
    if (out.empty()) return false;

    long long pts = 0;
    const bool ok = writeXzMap(path, map_name, out, &pts);
    if (ok) {
        std::cout << "[robot_map_export] " << path << " kf=" << out.size()
                  << " pts=" << pts << " floor_z=" << floor_z
                  << " step=" << step << "m" << std::endl;
    }
    if (out_kf) *out_kf = out.size();
    if (out_pts) *out_pts = static_cast<size_t>(pts);
    return ok;
}

} // namespace io
} // namespace rosiwit_slam
