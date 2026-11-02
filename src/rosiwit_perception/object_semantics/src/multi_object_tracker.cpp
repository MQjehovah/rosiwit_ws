// 多目标 3D 跟踪实现
#include "object_semantics/multi_object_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rosiwit_perception {
namespace {

constexpr double kBigCost = 1e6;

Eigen::Vector2d toWorld(const RobotState& robot, const Eigen::Vector2d& p_base) {
  if (!robot.valid) {
    return p_base;
  }
  return (robot.R_world_base.block<2, 2>(0, 0) * p_base) + robot.position_world.head<2>();
}

Eigen::Vector2d toBase(const RobotState& robot, const Eigen::Vector2d& p_world) {
  if (!robot.valid) {
    return p_world;
  }
  return robot.R_world_base.block<2, 2>(0, 0).transpose() * (p_world - robot.position_world.head<2>());
}

Eigen::Matrix2d rotateCov(const RobotState& robot, const Eigen::Matrix2d& cov_base) {
  if (!robot.valid) {
    return cov_base;
  }
  const Eigen::Matrix2d R = robot.R_world_base.block<2, 2>(0, 0);
  return R * cov_base * R.transpose();
}

}  // namespace

void MultiObjectTracker::solveAssignment(const std::vector<std::vector<double>>& cost,
                                    std::vector<int>& row_to_col) {
  const size_t n = cost.size();
  const size_t m = (n > 0) ? cost[0].size() : 0;
  row_to_col.assign(n, -1);
  if (n == 0 || m == 0) {
    return;
  }
  const size_t dim = std::max(n, m);
  // 匈牙利算法（O(dim^3)，n,m <= 64 足够）
  std::vector<std::vector<double>> a(dim + 1, std::vector<double>(dim + 1, 0.0));
  for (size_t i = 1; i <= dim; ++i) {
    for (size_t j = 1; j <= dim; ++j) {
      double c = 0.0;
      if (i <= n && j <= m) {
        c = cost[i - 1][j - 1];
      } else {
        c = kBigCost;  // 补齐的虚拟行列
      }
      a[i][j] = c;
    }
  }
  std::vector<double> u(dim + 1, 0.0), v(dim + 1, 0.0);
  std::vector<int> p(dim + 1, 0), way(dim + 1, 0);
  for (size_t i = 1; i <= dim; ++i) {
    p[0] = static_cast<int>(i);
    size_t j0 = 0;
    std::vector<double> minv(dim + 1, std::numeric_limits<double>::infinity());
    std::vector<char> used(dim + 1, false);
    do {
      used[j0] = true;
      const size_t i0 = static_cast<size_t>(p[j0]);
      double delta = std::numeric_limits<double>::infinity();
      size_t j1 = 0;
      for (size_t j = 1; j <= dim; ++j) {
        if (used[j]) {
          continue;
        }
        const double cur = a[i0][j] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = static_cast<int>(j0);
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (size_t j = 0; j <= dim; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const size_t j1 = static_cast<size_t>(way[j0]);
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }
  for (size_t j = 1; j <= dim; ++j) {
    const size_t i = static_cast<size_t>(p[j]);
    if (i >= 1 && i <= n && j <= m) {
      row_to_col[i - 1] = static_cast<int>(j - 1);
    }
  }
}

void MultiObjectTracker::toOutput(const Track& tr, double stamp, const RobotState& robot,
                            ObjectTrack& out) const {
  out.id = tr.id;
  out.class_id = tr.last_obs.box.class_id;
  out.class_name = tr.last_obs.box.class_name;
  out.confirmed = tr.confirmed;
  out.coasting = tr.coasting;
  out.depth_valid = tr.depth_valid;
  out.age = std::max(0.0, stamp - tr.first_stamp);

  const Eigen::Vector2d p_base = toBase(robot, tr.kf.position());
  const Eigen::Vector2d v_world(tr.kf.x(2), tr.kf.x(3));
  const Eigen::Vector2d v_base = robot.valid
      ? Eigen::Vector2d(robot.R_world_base.block<2, 2>(0, 0).transpose() *
                        (v_world - robot.velocity_world.head<2>()))
      : v_world;

  out.position = p_base;
  out.ground = p_base;
  out.velocity_base = v_base;
  out.velocity_world = v_world;
  out.z = tr.last_obs.z > 0.0 ? tr.last_obs.z : 1.0;
  out.distance = p_base.norm();
  out.confidence = tr.last_obs.confidence;
  out.box = tr.last_obs.box;
  // 输出位姿在 base_link, 协方差也要转回 base_link（KF 内部是 world 系）
  const Eigen::Matrix2d cov_world = tr.kf.P.block<2, 2>(0, 0);
  if (robot.valid) {
    const Eigen::Matrix2d R = robot.R_world_base.block<2, 2>(0, 0);
    out.cov_xy = R.transpose() * cov_world * R;
    out.cov_velocity = R.transpose() * tr.kf.P.block<2, 2>(2, 2) * R;
  } else {
    out.cov_xy = cov_world;
    out.cov_velocity = tr.kf.P.block<2, 2>(2, 2);
  }

  const double speed = v_base.norm();
  const double max_speed = tr.last_obs.max_speed_mps > 0.0
                               ? tr.last_obs.max_speed_mps
                               : params_.velocity_max_mps;
  out.velocity_valid = tr.confirmed && tr.depth_valid && out.age >= params_.velocity_min_age_s &&
                       speed <= max_speed;
  out.in_fov = out.distance > 0.2 && out.distance < 8.0 && std::fabs(p_base.x()) < 6.0;
}

void MultiObjectTracker::update(double stamp, const std::vector<ObjectObservation>& observations,
                           const RobotState& robot, std::vector<ObjectTrack>& tracks) {
  const double dt = (have_last_stamp_ && stamp > last_stamp_)
                        ? std::min(0.5, stamp - last_stamp_)
                        : 1.0 / 10.0;
  have_last_stamp_ = true;
  last_stamp_ = stamp;

  // ---- 1) 预测 ----
  for (auto& tr : tracks_) {
    tr.kf.predict(dt, params_.process_accel);
    tr.coasting = true;
  }

  // ---- 2) 关联（马氏距离门 + 欧氏上限 + 匈牙利）----
  const size_t n_tr = tracks_.size();
  const size_t n_ob = observations.size();
  std::vector<std::vector<double>> cost(n_tr, std::vector<double>(n_ob, kBigCost));
  for (size_t i = 0; i < n_tr; ++i) {
    for (size_t j = 0; j < n_ob; ++j) {
      // 类别门: 人不会变成车，不同类别的框/轨迹不参与关联
      if (params_.assoc_require_same_class &&
          tracks_[i].last_obs.box.class_id != observations[j].box.class_id) {
        continue;
      }
      const Eigen::Vector2d z_world = toWorld(robot, observations[j].position);
      const double dist = (z_world - tracks_[i].kf.position()).norm();
      if (dist > params_.assoc_max_dist) {
        continue;
      }
      const Eigen::Matrix2d R = rotateCov(robot, observations[j].cov_xy) +
                                Eigen::Matrix2d::Identity() * (params_.meas_sigma_floor *
                                                               params_.meas_sigma_floor);
      const double d2 = tracks_[i].kf.mahalanobisSq(z_world, R);
      if (d2 > params_.assoc_gate_chi2) {
        continue;
      }
      cost[i][j] = d2;
    }
  }
  std::vector<int> row_to_col;
  solveAssignment(cost, row_to_col);

  std::vector<bool> obs_used(n_ob, false);
  for (size_t i = 0; i < n_tr; ++i) {
    const int col = (i < row_to_col.size()) ? row_to_col[i] : -1;
    if (col < 0 || col >= static_cast<int>(n_ob) || cost[i][col] >= kBigCost * 0.5) {
      continue;
    }
    auto& tr = tracks_[i];
    const auto& ob = observations[col];
    const Eigen::Vector2d z_world = toWorld(robot, ob.position);
    const Eigen::Matrix2d R = rotateCov(robot, ob.cov_xy) +
                              Eigen::Matrix2d::Identity() * (params_.meas_sigma_floor *
                                                             params_.meas_sigma_floor);
    tr.kf.update(z_world, R);
    tr.last_update_stamp = stamp;
    tr.last_obs = ob;
    tr.depth_valid = ob.depth_valid;
    tr.coasting = false;
    tr.hits += 1;
    tr.misses = 0;
    if (!tr.confirmed && tr.hits >= params_.confirm_hits) {
      tr.confirmed = true;
    }
    obs_used[col] = true;
  }

  // ---- 3) 未关联: 生命周期 ----
  for (auto& tr : tracks_) {
    if (tr.coasting) {
      tr.misses += 1;
      const bool expired = (stamp - tr.last_update_stamp) > params_.max_coast_s;
      const bool tentative_dead = !tr.confirmed && tr.misses > params_.tentative_max_miss;
      if (expired || tentative_dead) {
        tr.hits = -1000000;  // 标记删除
      }
    }
  }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [](const Track& t) { return t.hits < 0; }),
                tracks_.end());

  // ---- 4) 新目标 ----
  for (size_t j = 0; j < n_ob; ++j) {
    if (obs_used[j]) {
      continue;
    }
    if (static_cast<int>(tracks_.size()) >= params_.max_tracks) {
      break;
    }
    Track tr;
    tr.id = next_id_++;
    tr.first_stamp = stamp;
    tr.last_update_stamp = stamp;
    tr.hits = 1;
    tr.last_obs = observations[j];
    tr.depth_valid = observations[j].depth_valid;
    tr.coasting = false;
    tr.kf.x = Eigen::Vector4d::Zero();
    tr.kf.x.head<2>() = toWorld(robot, observations[j].position);
    tr.kf.P = Eigen::Matrix4d::Identity() * 0.25;
    tr.confirmed = (params_.confirm_hits <= 1);
    tracks_.push_back(tr);
  }

  // ---- 5) 输出 ----
  tracks.clear();
  for (const auto& tr : tracks_) {
    ObjectTrack out;
    toOutput(tr, stamp, robot, out);
    tracks.push_back(out);
  }
}

}  // namespace rosiwit_perception
