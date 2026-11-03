#include "dwa_controller.hpp"
#include <algorithm>
#include <limits>
#include <cstdio>
#include <string>

namespace rosiwit_navigation {
namespace controllers {

DwaController::DwaController()
    : name_("dwa"), logger_("dwa"), idx_(0),
      max_vx_(0.25), min_vx_(0.0), max_vw_(0.5),
      max_accel_(1.0), max_dvw_(2.0),
      goal_tol_(0.1), dt_(0.1), sim_time_(1.0), lookahead_(0.8),
      initialized_(false), goal_reached_(false) {}

DwaController::~DwaController() = default;

bool DwaController::initialize(const core::ControllerConfig& config) {
    cfg_ = config;
    max_vx_ = config.max_linear_velocity > 0 ? config.max_linear_velocity : 0.5;
    // reverse: planner may emit velocity_x<0 segments; keep one-way when disabled
    min_vx_ = config.allow_reverse ? -0.5 * max_vx_ : 0.0;
    max_vw_ = config.max_angular_velocity > 0 ? config.max_angular_velocity : 1.0;
    max_accel_ = config.max_accel_x > 0 ? config.max_accel_x :
      (config.kinematics.max_velocity_x > 0 ? config.kinematics.max_velocity_x : 0.5);
    max_dvw_ = config.max_accel_theta > 0 ? config.max_accel_theta : 1.0;
    goal_tol_ = config.xy_goal_tolerance > 0 ? config.xy_goal_tolerance : 0.1;
    lookahead_ = config.lookahead_distance > 0 ?
      std::max(0.6, config.lookahead_distance) : 0.8;
    footprint_ = config.footprint;
    footprint_samples_ = core::sampleFootprint(footprint_, config.footprint_sample_spacing);
    initialized_ = true;
    LOG_INFO(logger_, "DWA initialized: vmax=%.2f, wmax=%.2f, accel=%.2f, waccel=%.2f, footprint=%zu samples=%zu",
             max_vx_, max_vw_, max_accel_, max_dvw_,
             footprint_.size(), footprint_samples_.size());
    return true;
}

void DwaController::setPath(const core::Path& path) {
    path_ = path;
    // 新路径可能包含绕行段, 不能沿用旧路径的进度.
    idx_ = 0;
    goal_reached_ = false;
}

core::VelocityCommand DwaController::computeVelocityCommand(
    const core::Pose2D& pose, const core::VelocityCommand& vel) {
    if (path_.points.empty() || !initialized_) return {0, 0, 0};

    // 路径重规划后 idx_ 可能停在旧路径位置; 用当前位姿校正, 但绝不倒退.
    idx_ = std::max(idx_, findClosest(pose));

    filterObstacles(pose);
    auto samples = search(pose, vel);
    last_samples_ = static_cast<int>(samples.size());
    if (samples.empty()) {
        // 没有可行样本 (全被障碍/转向互斥掉): 朝前视点原地转, 不要盲目前进.
        const double bear = std::atan2(last_carrot_y_ - pose.y, last_carrot_x_ - pose.x);
        const double err = normAngle(bear - pose.theta);
        if (std::fabs(err) > 0.15) {
            return {0.0, 0.0, (err > 0.0 ? 1.0 : -1.0) * 0.4 * max_vw_};
        }
        return {0, 0, 0};
    }

    // Pick best
    auto best = std::min_element(samples.begin(), samples.end(),
        [](const Sample& a, const Sample& b) { return a.cost < b.cost; });
    last_best_v_ = best->v;
    last_best_w_ = best->w;
    last_best_cost_ = best->cost;
    last_min_clearance_ = best->clearance;
    last_cross_track_ = pathCrossTrack(pose);

    // 平滑推进 idx_，最多 +2 防止前视点跳变
    double px = pose.x, py = pose.y, pt = pose.theta;
    size_t max_ci = idx_;
    for (double t = 0; t < sim_time_; t += dt_) {
        px += best->v * std::cos(pt) * dt_;
        py += best->v * std::sin(pt) * dt_;
        pt += best->w * dt_;
        size_t ci = findClosest({px, py, pt});
        if (ci > max_ci) max_ci = ci;
    }
    idx_ = std::min(idx_ + 2, max_ci);

    double dxg = path_.points.back().pose.x - pose.x;
    double dyg = path_.points.back().pose.y - pose.y;
    if (std::sqrt(dxg * dxg + dyg * dyg) < goal_tol_) {
        goal_reached_ = true;
        return {0, 0, 0};
    }

    return {best->v, 0, best->w};
}

struct VelocityBounds { double min_vx, max_vx, min_vw, max_vw; };

std::vector<DwaController::Sample> DwaController::search(
    const core::Pose2D& pose, const core::VelocityCommand& vel) {
    auto bounds = dynamicWindow(vel);
    std::vector<Sample> samples;
    const int nv = 10, nw = 10;
    const double dv = (bounds.max_vx - bounds.min_vx) / nv;
    const double dw_ = (bounds.max_vw - bounds.min_vw) / nw;

    // 前视 carrot: 沿路径弧长 lookahead_ 的点 (nav2/base_local_planner DWA 的
    // goal_front critic). 代价用"到前方路径点的距离"而不是"到整条路径的欧氏
    // 距离": 往返式清洁路径两条车道只差 0.46m, 欧氏距离分不清去程/回程,
    // 车会在折返点附近来回摆 (实车 2026-11-03 22:42 转 45s 不走).
    double cx = pose.x + lookahead_ * std::cos(pose.theta);
    double cy = pose.y + lookahead_ * std::sin(pose.theta);
    if (!path_.points.empty()) {
        size_t k = idx_;
        double acc = 0.0;
        while (k + 1 < path_.points.size() && acc < lookahead_) {
            acc += std::hypot(path_.points[k + 1].pose.x - path_.points[k].pose.x,
                              path_.points[k + 1].pose.y - path_.points[k].pose.y);
            ++k;
        }
        cx = path_.points[k].pose.x;
        cy = path_.points[k].pose.y;
    }
    const double d0 = std::max(0.30, std::hypot(cx - pose.x, cy - pose.y));
    last_carrot_x_ = cx;
    last_carrot_y_ = cy;
    // 前视点跑到车侧后方时先转再走 (nav2 rotate_to_heading):
    // 只看代价时, 180° 下"闷头往前开"反而可能更便宜, 实车就会冲出去.
    const double bear = std::atan2(cy - pose.y, cx - pose.x);
    const bool turn_first = std::fabs(normAngle(bear - pose.theta)) > 1.40;

    for (int i = 0; i <= nv; ++i) {
        for (int j = 0; j <= nw; ++j) {
            Sample s;
            s.v = bounds.min_vx + i * dv;
            s.w = bounds.min_vw + j * dw_;
            const core::Pose2D end_pose = simulate(pose, s);
            if (s.clearance < 0.01) { continue; }   // 碰撞轨迹直接丢弃
            if (turn_first && std::fabs(s.v) > 0.06) { continue; }  // 先原地转

            const double dxc = cx - end_pose.x;
            const double dyc = cy - end_pose.y;
            const double d_carrot = std::hypot(dxc, dyc);
            // 车头朝向前视点 (goal_front / alignment)
            const double want = std::atan2(dyc, dxc);
            const double herr = std::fabs(normAngle(want - end_pose.theta)) / M_PI;
            // 横向偏差, 把候选拉回路径中心线
            const double cross = std::min(1.0, pathCrossTrack(end_pose) / 0.4);
            const double car = std::min(1.0, d_carrot / d0);
            const double twirl = std::fabs(s.w) / std::max(1e-6, max_vw_);
            const double dv_cmd = std::fabs(s.v - vel.linear_x) / std::max(1e-6, max_vx_);
            const double dw_cmd = std::fabs(s.w - vel.angular_z) / std::max(1e-6, max_vw_);
            const double smooth = 0.5 * (dv_cmd + dw_cmd);
            const double speed_reward =
                std::min(1.0, std::fabs(s.v) / std::max(1e-6, max_vx_));
            const double clearance_violation =
                (s.clearance < 0.25) ? 1.0 - s.clearance / 0.25 : 0.0;
            // 原地来回摆头 (oscillation critic): 车没走却在反向转 -> 重罚.
            double osc = 0.0;
            if (std::fabs(vel.linear_x) < 0.03 && std::fabs(vel.angular_z) > 0.05 &&
                s.w * vel.angular_z < 0.0) {
                osc = 0.80;
            }
            s.heading = 1.0 - herr;
            // 归一化 critic 组合: goal_front / heading / path / obstacle /
            // twirling / oscillation / 速度奖励.
            s.cost = 1.00 * car + 0.60 * herr + 0.80 * cross +
                     0.50 * clearance_violation + 0.15 * twirl +
                     0.15 * smooth - 0.20 * speed_reward + osc;
            samples.push_back(s);
        }
    }
    return samples;
}

DwaController::VelocityBounds DwaController::dynamicWindow(const core::VelocityCommand& vel) const {
    VelocityBounds b;
    // 动态窗口是"一个控制步内"可达到的速度; 用 sim_time_ 会把 1.5s
    // 的加速度一次性塞进 50ms 控制周期, 造成 8m/s² 级跳变.
    b.min_vx = std::max(min_vx_, vel.linear_x - max_accel_ * dt_);
    b.max_vx = std::min(max_vx_, vel.linear_x + max_accel_ * dt_);
    b.min_vw = std::max(-max_vw_, vel.angular_z - max_dvw_ * dt_);
    b.max_vw = std::min(max_vw_, vel.angular_z + max_dvw_ * dt_);
    return b;
}

core::Pose2D DwaController::simulate(const core::Pose2D& pose, Sample& s) const {
    double px = pose.x, py = pose.y, pt = pose.theta;
    s.clearance = 5.0;  // start with large clearance
    for (double t = 0; t < sim_time_; t += dt_) {
        px += s.v * std::cos(pt) * dt_;
        py += s.v * std::sin(pt) * dt_;
        pt += s.w * dt_;
        double step_min = 5.0;
        if (!near_obstacles_.empty()) {
            if (footprint_samples_.empty()) {
                for (const auto& obs : near_obstacles_) {
                    double dx = px - obs.x, dy = py - obs.y;
                    double d = std::sqrt(dx * dx + dy * dy) - obs.radius;
                    if (d < step_min) step_min = d;
                }
            } else {
                // 长方形车体: 每个 footprint 采样点都要离障碍物足够远.
                const double c = std::cos(pt), sn = std::sin(pt);
                for (const auto& sample : footprint_samples_) {
                    const double wx = px + c * sample.first - sn * sample.second;
                    const double wy = py + sn * sample.first + c * sample.second;
                    for (const auto& obs : near_obstacles_) {
                        double dx = wx - obs.x, dy = wy - obs.y;
                        double d = std::sqrt(dx * dx + dy * dy) - obs.radius;
                        if (d < step_min) step_min = d;
                    }
                }
            }
        }
        // 记录全程最小距离，不提前退出（碰撞后继续模拟，但距离为负时 penalty 递增）
        if (step_min < s.clearance) s.clearance = step_min;

        // 局部 costmap: footprint 非空时按整个车体取最大代价, 否则只看中心.
        if (has_local_costmap_) {
            const int v = core::footprintCost(
                local_costmap_, core::Pose2D(px, py, pt), footprint_samples_);
            if (v >= 100) {
                s.clearance = std::min(s.clearance, 0.0);
            } else if (v > 0) {
                s.clearance = std::min(s.clearance,
                    (100.0 - static_cast<double>(v)) / 100.0 * 0.5);
            }
        }
        // 已判定碰撞: 这条候选必然被丢弃, 不必再算完整个仿真周期.
        if (s.clearance < 0.005) { break; }
    }
    return {px, py, pt};
}

// 只保留一个仿真周期内可能碰到的障碍: 车最多跑 sim_time*max_vx (0.3m),
// 加上车体外接半径 0.73m 和余量, 1.2m 以外的不可能撞上.
void DwaController::filterObstacles(const core::Pose2D& pose) {
    near_obstacles_.clear();
    const double reach = sim_time_ * max_vx_ + 1.20;
    near_obstacles_.reserve(obstacles_.size());
    for (const auto& o : obstacles_) {
        const double d = std::hypot(o.x - pose.x, o.y - pose.y) - o.radius;
        if (d < reach) { near_obstacles_.push_back(o); }
    }
}

double DwaController::headingCost(const core::Pose2D& pose) const {
    if (path_.points.empty()) return 1;
    // 找距离当前位置 0.5m 的前视点，避免用 idx_+3 太近导致抖动
    size_t target = idx_;
    double best_dist_diff = std::numeric_limits<double>::max();
    for (size_t i = idx_; i < path_.points.size(); ++i) {
        double d = std::sqrt(std::pow(path_.points[i].pose.x - pose.x, 2) +
                             std::pow(path_.points[i].pose.y - pose.y, 2));
        double diff = std::abs(d - 0.5);
        if (diff < best_dist_diff) { best_dist_diff = diff; target = i; }
    }
    double dx = path_.points[target].pose.x - pose.x;
    double dy = path_.points[target].pose.y - pose.y;
    double goal_angle = std::atan2(dy, dx);
    return 1.0 - std::abs(normAngle(goal_angle - pose.theta)) / M_PI;
}

double DwaController::pathCrossTrack(const core::Pose2D& pose) const {
    if (path_.points.empty()) return 0.0;
    const size_t n = path_.points.size();
    // 只统计 idx_ 之后的前向路径 (至少给 40 个点/约 3m 的窗口):
    // 路径含折返时, 用整条路径会把"回程"当成横向偏差 0.
    const size_t end = std::min(n, idx_ + 40);
    double best = std::numeric_limits<double>::max();
    for (size_t i = idx_; i < end; ++i) {
        const double dx = path_.points[i].pose.x - pose.x;
        const double dy = path_.points[i].pose.y - pose.y;
        const double d = dx * dx + dy * dy;
        if (d < best) best = d;
    }
    return std::sqrt(best == std::numeric_limits<double>::max() ? 0.0 : best);
}

double DwaController::clearanceCost(const core::Pose2D& pose) const {
    double min_d = std::numeric_limits<double>::max();
    for (const auto& obs : obstacles_) {
        double dx = pose.x - obs.x, dy = pose.y - obs.y;
        double d = std::sqrt(dx * dx + dy * dy) - obs.radius;
        if (d < min_d) min_d = d;
    }
    return std::min(min_d, 5.0) / 5.0;
}

size_t DwaController::findClosest(const core::Pose2D& pose) const {
    size_t best = 0; double best_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_.points.size(); ++i) {
        double dx = path_.points[i].pose.x - pose.x;
        double dy = path_.points[i].pose.y - pose.y;
        double d = dx * dx + dy * dy;
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

bool DwaController::isGoalReached(const core::Pose2D& pose) const {
    // 路径被清空表示导航失败/等待重规划, 不是到达目标.
    if (path_.points.empty()) return false;
    double dx = path_.points.back().pose.x - pose.x;
    double dy = path_.points.back().pose.y - pose.y;
    return std::sqrt(dx * dx + dy * dy) < goal_tol_;
}

size_t DwaController::getCurrentWaypointIndex() const { return idx_; }
double DwaController::getProgress() const {
    if (path_.points.empty()) return 1.0;
    return static_cast<double>(idx_) / path_.points.size();
}
void DwaController::reset() { idx_ = 0; goal_reached_ = false; path_.points.clear(); }
void DwaController::setVelocityLimit(double vx, double vw) { max_vx_ = vx; max_vw_ = vw; }
std::string DwaController::getName() const { return name_; }
std::string DwaController::getVersion() const { return "1.0.0"; }
void DwaController::setObstacles(const core::ObstacleArray& obs) {
    obstacles_ = obs;
    near_obstacles_ = obs;
}

std::string DwaController::getDiagnostics() const {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "controller=dwa samples=%d best_v=%.3f best_w=%.3f cost=%.3f clearance=%.3f cross=%.3f idx=%zu",
        last_samples_, last_best_v_, last_best_w_, last_best_cost_,
        last_min_clearance_, last_cross_track_, idx_);
    return std::string(buf);
}

void DwaController::setLocalCostmap(const core::CostmapGrid& costmap) {
    local_costmap_ = costmap;
    has_local_costmap_ = !local_costmap_.data.empty() &&
        local_costmap_.info.width > 0 && local_costmap_.info.height > 0;
}
double DwaController::normAngle(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

}}  // namespaces
