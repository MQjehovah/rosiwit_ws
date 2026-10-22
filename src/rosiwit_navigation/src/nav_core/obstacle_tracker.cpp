#include "rosiwit_navigation/nav_core/obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace rosiwit_navigation
{
namespace core
{

double TrackedObstacle::speed() const
{
  return std::hypot(vx, vy);
}

ObstacleTracker::ObstacleTracker(const ObstacleTrackerConfig & config)
  : config_(config)
{
}

std::vector<ObstacleTracker::Cluster> ObstacleTracker::cluster(
  const std::vector<Point2D> & points) const
{
  std::vector<Cluster> clusters;
  if (points.empty()) return clusters;

  const size_t n = std::min(points.size(), config_.max_points);
  const double cell = std::max(0.05, config_.cluster_distance);
  auto keyOf = [cell](double x, double y) -> int64_t {
    const int64_t ix = static_cast<int64_t>(std::floor(x / cell));
    const int64_t iy = static_cast<int64_t>(std::floor(y / cell));
    return (ix << 32) ^ (iy & 0xffffffffLL);
  };

  std::unordered_map<int64_t, std::vector<int>> buckets;
  buckets.reserve(n * 2);
  for (size_t i = 0; i < n; ++i) {
    buckets[keyOf(points[i].x, points[i].y)].push_back(static_cast<int>(i));
  }

  std::vector<char> visited(n, 0);
  const double d2 = config_.cluster_distance * config_.cluster_distance;
  for (size_t seed = 0; seed < n; ++seed) {
    if (visited[seed]) continue;
    std::vector<int> queue{static_cast<int>(seed)};
    visited[seed] = 1;
    Cluster c;
    int count = 0;
    for (size_t q = 0; q < queue.size(); ++q) {
      const int idx = queue[q];
      const Point2D & p = points[idx];
      c.x += p.x;
      c.y += p.y;
      ++count;

      const int64_t ix = static_cast<int64_t>(std::floor(p.x / cell));
      const int64_t iy = static_cast<int64_t>(std::floor(p.y / cell));
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          const int64_t key = ((ix + dx) << 32) ^ ((iy + dy) & 0xffffffffLL);
          auto it = buckets.find(key);
          if (it == buckets.end()) continue;
          for (int j : it->second) {
            if (visited[j]) continue;
            const double ddx = points[j].x - p.x;
            const double ddy = points[j].y - p.y;
            if (ddx * ddx + ddy * ddy <= d2) {
              visited[j] = 1;
              queue.push_back(j);
            }
          }
        }
      }
    }
    if (count > 0) {
      c.x /= count;
      c.y /= count;
      c.points = count;
      double max_r2 = 0.0;
      for (int idx : queue) {
        const double dx = points[idx].x - c.x;
        const double dy = points[idx].y - c.y;
        max_r2 = std::max(max_r2, dx * dx + dy * dy);
      }
      c.radius = std::max(0.1, std::sqrt(max_r2) + 0.05);
      // 一面长墙/货架会形成半径数米的连通簇; 它不是可跟踪的动态圆,
      // 不能把整面墙膨胀成 predicted obstacle.
      if (c.radius > config_.max_radius) continue;
      clusters.push_back(c);
    }
  }
  return clusters;
}

void ObstacleTracker::update(const std::vector<Point2D> & points, double stamp, double dt)
{
  double step = dt;
  if (step <= 0.0) {
    step = (last_stamp_ >= 0.0 && stamp > last_stamp_) ? (stamp - last_stamp_)
                                                       : config_.dt_default;
  }
  step = std::max(1e-3, step);
  last_stamp_ = stamp;

  const auto clusters = cluster(points);
  std::vector<char> track_used(tracks_.size(), 0);
  std::vector<char> cluster_used(clusters.size(), 0);

  for (size_t ci = 0; ci < clusters.size(); ++ci) {
    const Cluster & c = clusters[ci];
    int best = -1;
    double best_d = config_.association_distance * config_.association_distance;
    for (size_t ti = 0; ti < tracks_.size(); ++ti) {
      if (track_used[ti]) continue;
      const double dx = tracks_[ti].x - c.x;
      const double dy = tracks_[ti].y - c.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d) {
        best_d = d2;
        best = static_cast<int>(ti);
      }
    }

    if (best >= 0) {
      TrackedObstacle & t = tracks_[best];
      const double vx_meas = (c.x - t.x) / step;
      const double vy_meas = (c.y - t.y) / step;
      const double alpha = std::max(0.0, std::min(1.0, config_.velocity_smoothing));
      t.vx = alpha * vx_meas + (1.0 - alpha) * t.vx;
      t.vy = alpha * vy_meas + (1.0 - alpha) * t.vy;
      if (std::hypot(t.vx, t.vy) > config_.max_speed) {
        t.vx = 0.0;
        t.vy = 0.0;
      }
      t.x = c.x;
      t.y = c.y;
      t.radius = c.radius;
      ++t.hits;
      t.misses = 0;
      t.confirmed = t.hits >= config_.confirm_hits;
      track_used[best] = 1;
      cluster_used[ci] = 1;
    } else {
      TrackedObstacle t;
      t.id = next_id_++;
      t.x = c.x;
      t.y = c.y;
      t.radius = c.radius;
      t.hits = 1;
      t.misses = 0;
      t.confirmed = config_.confirm_hits <= 1;
      tracks_.push_back(t);
      track_used.push_back(1);
      cluster_used[ci] = 1;
    }
  }

  (void)cluster_used;

  // 未关联轨迹: 用预测位置外推，连续丢失超过阈值后删除。
  for (size_t ti = 0; ti < tracks_.size(); ++ti) {
    if (track_used[ti]) continue;
    TrackedObstacle & t = tracks_[ti];
    t.x += t.vx * step;
    t.y += t.vy * step;
    ++t.misses;
    t.confirmed = false;
  }
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(),
                   [this](const TrackedObstacle & t) {
                     return t.misses > config_.max_misses;
                   }),
    tracks_.end());
}

std::vector<TrackedObstacle> ObstacleTracker::tracked(bool confirmed_only) const
{
  std::vector<TrackedObstacle> out;
  out.reserve(tracks_.size());
  for (const auto & t : tracks_) {
    if (!confirmed_only || t.confirmed) out.push_back(t);
  }
  return out;
}

std::vector<TrackedObstacle> ObstacleTracker::predicted(double horizon) const
{
  std::vector<TrackedObstacle> out;
  out.reserve(tracks_.size());
  for (const auto & t : tracks_) {
    if (!t.confirmed) continue;
    // 静止目标已经由静态地图和实时 /scan 障碍层表达; 这里只预测运动目标,
    // 否则货架/墙段会被当成永久动态圆持续叠加到局部 costmap.
    if (t.speed() < config_.min_prediction_speed) continue;
    TrackedObstacle p = t;
    p.x += p.vx * horizon;
    p.y += p.vy * horizon;
    out.push_back(p);
  }
  return out;
}

void ObstacleTracker::clear()
{
  tracks_.clear();
  next_id_ = 1;
  last_stamp_ = -1.0;
}

}  // namespace core
}  // namespace rosiwit_navigation
