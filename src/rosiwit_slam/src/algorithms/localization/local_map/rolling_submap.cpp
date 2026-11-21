#include "algorithms/localization/local_map/rolling_submap.h"

#include <cmath>
#include <iostream>

namespace rosiwit_slam {

int64_t RollingSubmap::key(double x, double y, double z) const {
  const double v = (cfg_.voxel > 1e-6) ? cfg_.voxel : 0.15;
  const int64_t ix = static_cast<int64_t>(std::floor(x / v));
  const int64_t iy = static_cast<int64_t>(std::floor(y / v));
  const int64_t iz = static_cast<int64_t>(std::floor(z / v));
  // 每轴 21 位（约 ±1e5 m @0.15m），够仓库尺度
  return ((ix & 0x1FFFFF) << 42) | ((iy & 0x1FFFFF) << 21) | (iz & 0x1FFFFF);
}

void RollingSubmap::reset(const PoseStamped& anchor, double now_s) {
  anchor_ = anchor;
  created_s_ = now_s;
  pts_.clear();
  active_ = true;
  have_robot_ = false;
  capacity_warned_ = false;
  std::cout << "[submap] opened: anchor=(" << anchor.trans.x() << ", "
            << anchor.trans.y() << ") radius=" << cfg_.radius << std::endl;
}

void RollingSubmap::sweepFar() {
  if (!have_robot_) return;
  const double r = cfg_.radius;
  if (r <= 0.0) return;
  const double r2 = r * r;
  for (auto it = pts_.begin(); it != pts_.end();) {
    if ((it->second - robot_anchor_).squaredNorm() > r2) {
      it = pts_.erase(it);
    } else {
      ++it;
    }
  }
  sweep_origin_ = robot_anchor_;
}

void RollingSubmap::update(const CloudType& scan_body, const PoseStamped& pose_world) {
  if (!active_ || scan_body.empty()) return;

  // world = pose * body
  const M3D& R_wb = pose_world.rot;
  const V3D& t_wb = pose_world.trans;

  // anchor 系 = T_GS^-1 * world
  const M3D R_gs_inv = anchor_.rot.transpose();
  const V3D t_gs = anchor_.trans;

  robot_anchor_ = R_gs_inv * (t_wb - t_gs);
  have_robot_ = true;

  const double r = cfg_.radius;
  const double r2 = (r > 0.0) ? r * r : 0.0;
  const bool cap = (cfg_.max_points > 0 && pts_.size() >= cfg_.max_points);

  if (cap) {
    if (!capacity_warned_) {
      std::cout << "[submap] capacity reached (" << cfg_.max_points
                << "), stop inserting" << std::endl;
      capacity_warned_ = true;
    }
  } else {
    for (const auto& p : scan_body.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      const V3D w = R_wb * V3D(p.x, p.y, p.z) + t_wb;
      const V3D a = R_gs_inv * (w - t_gs);
      if (r2 > 0.0 && (a - robot_anchor_).squaredNorm() > r2) continue;  // 半径外直接丢
      pts_.emplace(key(a.x(), a.y(), a.z()), a);                          // 体素去重
    }
  }

  // 机器人走出一段距离后清扫一次（避免每帧 O(n) 全扫）
  if (have_robot_ && (robot_anchor_ - sweep_origin_).norm() > std::max(1.0, 0.4 * r)) {
    sweepFar();
  }
}

CloudType::Ptr RollingSubmap::pointsWorld() const {
  CloudType::Ptr out(new CloudType());
  if (!active_ || pts_.empty()) return out;
  out->reserve(pts_.size());
  for (const auto& kv : pts_) {
    const V3D w = anchor_.rot * kv.second + anchor_.trans;
    PointType p;
    p.x = static_cast<float>(w.x());
    p.y = static_cast<float>(w.y());
    p.z = static_cast<float>(w.z());
    p.intensity = 0.0f;
    out->push_back(p);
  }
  out->width = static_cast<uint32_t>(out->size());
  out->height = 1;
  out->is_dense = false;
  return out;
}

}  // namespace rosiwit_slam
