#include "lqr_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace rosiwit_navigation {
namespace controllers {

namespace {

struct Mat3 { double m[3][3]; };
struct Mat32 { double m[3][2]; };
struct Mat2 { double m[2][2]; };

Mat3 zero3() {
  Mat3 r{};
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = 0.0;
  return r;
}

Mat3 mul3(const Mat3 & a, const Mat3 & b) {
  Mat3 r = zero3();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) r.m[i][j] += a.m[i][k] * b.m[k][j];
  return r;
}

Mat3 transpose3(const Mat3 & a) {
  Mat3 r = zero3();
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[j][i];
  return r;
}

// a(3x3) * b(3x2)
Mat32 mul32(const Mat3 & a, const Mat32 & b) {
  Mat32 r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 2; ++j) {
      r.m[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) r.m[i][j] += a.m[i][k] * b.m[k][j];
    }
  return r;
}

// a(3x3)^T * b(3x2)
Mat32 mulT32(const Mat3 & a, const Mat32 & b) {
  Mat32 r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 2; ++j) {
      r.m[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) r.m[i][j] += a.m[k][i] * b.m[k][j];
    }
  return r;
}

// a(3x2)^T * b(3x3)
void mulT23(const Mat32 & a, const Mat3 & b, double out[2][3]) {
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 3; ++j) {
      out[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) out[i][j] += a.m[k][i] * b.m[k][j];
    }
}

// a(3x2)^T * b(3x2)
Mat2 mulT22(const Mat32 & a, const Mat32 & b) {
  Mat2 r{};
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j) {
      r.m[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) r.m[i][j] += a.m[k][i] * b.m[k][j];
    }
  return r;
}

}  // namespace

LqrController::LqrController() : name_("lqr"), logger_("lqr") {}

LqrController::~LqrController() = default;

bool LqrController::initialize(const core::ControllerConfig& config) {
  cfg_ = config;
  max_vx_ = config.max_linear_velocity > 0 ? config.max_linear_velocity : 0.3;
  max_vw_ = config.max_angular_velocity > 0 ? config.max_angular_velocity : 0.5;
  max_ax_ = config.max_accel_x > 0 ? config.max_accel_x : 0.3;
  max_aw_ = config.max_accel_theta > 0 ? config.max_accel_theta : 0.4;
  goal_tol_ = config.xy_goal_tolerance > 0 ? config.xy_goal_tolerance : 0.1;
  lookahead_time_ = config.lookahead_time > 0 ? config.lookahead_time : 1.2;
  min_lookahead_ = config.min_lookahead_distance > 0 ? config.min_lookahead_distance : 0.30;
  max_lookahead_ = config.max_lookahead_distance > 0 ? config.max_lookahead_distance : 0.90;
  if (max_lookahead_ < min_lookahead_) max_lookahead_ = min_lookahead_;
  q_x_ = config.lqr_q_x > 0 ? config.lqr_q_x : 5.0;
  q_y_ = config.lqr_q_y > 0 ? config.lqr_q_y : 5.0;
  q_theta_ = config.lqr_q_theta > 0 ? config.lqr_q_theta : 1.0;
  r_v_ = config.lqr_r_v > 0 ? config.lqr_r_v : 1.0;
  r_w_ = config.lqr_r_w > 0 ? config.lqr_r_w : 1.0;
  initialized_ = true;
  LOG_INFO(logger_, "LQR initialized: vmax=%.2f wmax=%.2f lookahead %.2f..%.2f Q(%.1f,%.1f,%.1f) R(%.1f,%.1f)",
           max_vx_, max_vw_, min_lookahead_, max_lookahead_, q_x_, q_y_, q_theta_, r_v_, r_w_);
  return true;
}

void LqrController::setPath(const core::Path& path) {
  path_ = path;
  idx_ = 0;
  goal_reached_ = false;
}

void LqrController::reset() {
  idx_ = 0;
  goal_reached_ = false;
  last_v_ = 0.0;
  last_w_ = 0.0;
  path_.points.clear();
}

double LqrController::normAngle(double a) const {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

size_t LqrController::findNearest(const core::Pose2D& pose) const {
  if (path_.points.empty()) return 0;
  const size_t n = path_.points.size();
  const size_t begin = idx_ > 2 ? idx_ - 2 : 0;
  const size_t end = std::min(n, idx_ + 120);
  size_t best = begin;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = begin; i < end; ++i) {
    const double dx = path_.points[i].pose.x - pose.x;
    const double dy = path_.points[i].pose.y - pose.y;
    const double d = dx * dx + dy * dy;
    if (d < best_d) { best_d = d; best = i; }
  }
  if (best_d > 1.5 * 1.5) {   // 被推出窗口才回退全路径
    for (size_t i = 0; i < n; ++i) {
      const double dx = path_.points[i].pose.x - pose.x;
      const double dy = path_.points[i].pose.y - pose.y;
      const double d = dx * dx + dy * dy;
      if (d < best_d) { best_d = d; best = i; }
    }
  }
  return best;
}

size_t LqrController::lookaheadIndex(size_t from, double dist) const {
  if (path_.points.empty()) return 0;
  size_t i = std::min(from, path_.points.size() - 1);
  double acc = 0.0;
  while (i + 1 < path_.points.size() && acc < dist) {
    acc += std::hypot(path_.points[i + 1].pose.x - path_.points[i].pose.x,
                      path_.points[i + 1].pose.y - path_.points[i].pose.y);
    ++i;
  }
  return i;
}

double LqrController::pathTangent(size_t i) const {
  const size_t n = path_.points.size();
  if (n < 2) return 0.0;
  size_t a = i > 0 ? i - 1 : 0;
  size_t b = std::min(n - 1, i + 1);
  if (a == b) {
    const auto & pa = path_.points[a].pose;
    const auto & pb = path_.points[b].pose;
    const double dx = pb.x - pa.x, dy = pb.y - pa.y;
    if (std::hypot(dx, dy) > 1e-6) return std::atan2(dy, dx);
    return pa.theta;
  }
  const auto & pa = path_.points[a].pose;
  const auto & pb = path_.points[b].pose;
  const double dx = pb.x - pa.x, dy = pb.y - pa.y;
  if (std::hypot(dx, dy) > 1e-6) return std::atan2(dy, dx);
  return path_.points[i].pose.theta;
}

double LqrController::pathCurvature(size_t i) const {
  const size_t n = path_.points.size();
  if (n < 5) return 0.0;
  const size_t a = i > 2 ? i - 2 : 0;
  const size_t b = i;
  const size_t c = std::min(n - 1, i + 2);
  const auto & pa = path_.points[a].pose;
  const auto & pb = path_.points[b].pose;
  const auto & pc = path_.points[c].pose;
  const double ab = std::hypot(pb.x - pa.x, pb.y - pa.y);
  const double bc = std::hypot(pc.x - pb.x, pc.y - pb.y);
  const double ca = std::hypot(pa.x - pc.x, pa.y - pc.y);
  const double area2 = std::fabs((pb.x - pa.x) * (pc.y - pa.y) - (pc.x - pa.x) * (pb.y - pa.y));
  if (ab < 1e-6 || bc < 1e-6 || ca < 1e-6 || area2 < 1e-9) return 0.0;
  return 2.0 * area2 / (ab * bc * ca);
}

core::VelocityCommand LqrController::computeVelocityCommand(
    const core::Pose2D& pose, const core::VelocityCommand& vel) {
  if (!initialized_ || path_.points.empty()) return core::VelocityCommand(0.0, 0.0, 0.0);

  // 1) 路径投影 (只在前向窗口内, 防止往返路径跳到回程)
  idx_ = std::max(idx_, findNearest(pose));

  // 2) 前视点 + 参考曲率
  const double v_now = std::fabs(vel.linear_x);
  double L = std::min(max_lookahead_,
                      std::max(min_lookahead_, v_now * lookahead_time_));
  const size_t la = lookaheadIndex(idx_, L);
  const core::Pose2D ref = path_.points[la].pose;
  const double ref_yaw = pathTangent(la);
  const double kappa = pathCurvature(la);
  last_kappa_ = kappa;

  // 3) 误差状态 (全局系, 与参考实现一致)
  const double e_lat_x = pose.x - ref.x;
  const double e_lat_y = pose.y - ref.y;
  const double e_theta = normAngle(pose.theta - ref_yaw);
  last_lateral_ = std::hypot(e_lat_x, e_lat_y);
  last_heading_ = e_theta;

  // 4) 参考输入: 速度 + 曲率前馈
  double v_ref = max_vx_;
  // 航向偏差大时降速 (不原地转, 边走边纠)
  v_ref *= std::max(0.25, 1.0 - std::fabs(e_theta) / 1.2);
  // 末端减速
  const auto & end = path_.points.back().pose;
  const double d_end = std::hypot(end.x - pose.x, end.y - pose.y);
  if (d_end < 1.0) v_ref = std::min(v_ref, std::max(0.06, d_end));
  const double u_r[2] = {v_ref, v_ref * kappa};

  // 5) 离散 Riccati 迭代 (A, B 随参考航向变化)
  const double dt = control_dt_;
  Mat3 A = zero3();
  A.m[0][0] = 1.0; A.m[1][1] = 1.0; A.m[2][2] = 1.0;
  A.m[0][2] = -u_r[0] * std::sin(ref_yaw) * dt;
  A.m[1][2] = u_r[0] * std::cos(ref_yaw) * dt;

  Mat32 B{};
  B.m[0][0] = std::cos(ref_yaw) * dt;
  B.m[1][0] = std::sin(ref_yaw) * dt;
  B.m[2][1] = dt;

  Mat3 Q = zero3();
  Q.m[0][0] = q_x_; Q.m[1][1] = q_y_; Q.m[2][2] = q_theta_;
  Mat2 R{};
  R.m[0][0] = r_v_; R.m[1][1] = r_w_;

  Mat3 P = Q;
  for (int it = 0; it < 60; ++it) {
    const Mat3 PA = mul3(P, A);
    const Mat32 PB = mul32(P, B);
    const Mat2 BtPB = mulT22(B, PB);
    Mat2 temp{};
    temp.m[0][0] = R.m[0][0] + BtPB.m[0][0];
    temp.m[0][1] = R.m[0][1] + BtPB.m[0][1];
    temp.m[1][0] = R.m[1][0] + BtPB.m[1][0];
    temp.m[1][1] = R.m[1][1] + BtPB.m[1][1];
    const double det = temp.m[0][0] * temp.m[1][1] - temp.m[0][1] * temp.m[1][0];
    if (std::fabs(det) < 1e-12) break;
    Mat2 inv{};
    inv.m[0][0] = temp.m[1][1] / det;
    inv.m[0][1] = -temp.m[0][1] / det;
    inv.m[1][0] = -temp.m[1][0] / det;
    inv.m[1][1] = temp.m[0][0] / det;

    const Mat32 AtPB = mulT32(A, PB);      // A^T P B  (3x2)
    double BtPA[2][3];
    mulT23(B, PA, BtPA);                   // B^T P A  (2x3)
    const Mat3 AtPA = mul3(transpose3(A), PA);

    double tmp[3][2];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 2; ++j) {
        tmp[i][j] = 0.0;
        for (int k = 0; k < 2; ++k) tmp[i][j] += AtPB.m[i][k] * inv.m[k][j];
      }
    Mat3 Pn = zero3();
    double diff = 0.0;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        double corr = 0.0;
        for (int k = 0; k < 2; ++k) corr += tmp[i][k] * BtPA[k][j];
        Pn.m[i][j] = Q.m[i][j] + AtPA.m[i][j] - corr;
        diff = std::max(diff, std::fabs(Pn.m[i][j] - P.m[i][j]));
      }
    P = Pn;
    if (diff < 1e-8) break;
  }

  // 6) 反馈增益 K = -(R + B^T P B)^-1 B^T P A
  const Mat2 BtPB = mulT22(B, mul32(P, B));
  Mat2 temp{};
  temp.m[0][0] = R.m[0][0] + BtPB.m[0][0];
  temp.m[0][1] = R.m[0][1] + BtPB.m[0][1];
  temp.m[1][0] = R.m[1][0] + BtPB.m[1][0];
  temp.m[1][1] = R.m[1][1] + BtPB.m[1][1];
  const double det = temp.m[0][0] * temp.m[1][1] - temp.m[0][1] * temp.m[1][0];
  Mat2 inv{};
  if (std::fabs(det) > 1e-12) {
    inv.m[0][0] = temp.m[1][1] / det;
    inv.m[0][1] = -temp.m[0][1] / det;
    inv.m[1][0] = -temp.m[1][0] / det;
    inv.m[1][1] = temp.m[0][0] / det;
  } else {
    inv.m[0][0] = 1.0; inv.m[1][1] = 1.0;
  }
  const Mat3 PA = mul3(P, A);
  double BtPA[2][3];
  mulT23(B, PA, BtPA);
  double K[2][3];
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 3; ++j) {
      K[i][j] = 0.0;
      for (int k = 0; k < 2; ++k) K[i][j] -= inv.m[i][k] * BtPA[k][j];
    }

  // 7) u = u_r + K e
  const double e[3] = {e_lat_x, e_lat_y, e_theta};
  double u_v = u_r[0];
  double u_w = u_r[1];
  for (int j = 0; j < 3; ++j) {
    u_v += K[0][j] * e[j];
    u_w += K[1][j] * e[j];
  }

  // 8) 限幅 + 加速度限幅
  u_v = std::max(0.0, std::min(max_vx_, u_v));
  u_w = std::max(-max_vw_, std::min(max_vw_, u_w));
  const double dv_max = max_ax_ * dt;
  const double dw_max = max_aw_ * dt;
  u_v = std::max(last_v_ - dv_max, std::min(last_v_ + dv_max, u_v));
  u_w = std::max(last_w_ - dw_max, std::min(last_w_ + dw_max, u_w));
  last_v_ = u_v;
  last_w_ = u_w;

  return core::VelocityCommand(u_v, 0.0, u_w);
}

bool LqrController::isGoalReached(const core::Pose2D& pose) const {
  if (path_.points.empty()) return false;
  // 往返式路径终点就在起点旁边: 必须同时要求沿路径进度到末端
  if (idx_ + 3 < path_.points.size()) return false;
  const auto & end = path_.points.back().pose;
  return std::hypot(end.x - pose.x, end.y - pose.y) < goal_tol_;
}

size_t LqrController::getCurrentWaypointIndex() const { return idx_; }

double LqrController::getProgress() const {
  if (path_.points.empty()) return 1.0;
  return static_cast<double>(idx_) / static_cast<double>(path_.points.size());
}

void LqrController::setVelocityLimit(double linear_x, double angular_z) {
  if (linear_x > 0) max_vx_ = linear_x;
  if (angular_z > 0) max_vw_ = angular_z;
}

std::string LqrController::getName() const { return name_; }
std::string LqrController::getVersion() const { return "1.0.0"; }

std::string LqrController::getDiagnostics() const {
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "controller=lqr idx=%zu lateral=%.3f heading=%.3f kappa=%.3f v=%.3f w=%.3f",
                idx_, last_lateral_, last_heading_, last_kappa_, last_v_, last_w_);
  return std::string(buf);
}

}  // namespace controllers
}  // namespace rosiwit_navigation
