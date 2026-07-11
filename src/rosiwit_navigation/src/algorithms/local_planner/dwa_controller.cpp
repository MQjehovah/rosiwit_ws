#include "dwa_controller.hpp"
#include <algorithm>
#include <limits>

namespace rosiwit_navigation {
namespace controllers {

DwaController::DwaController()
    : name_("dwa"), logger_("dwa"), idx_(0),
      max_vx_(0.5), min_vx_(-0.1), max_vw_(1.0),
      max_accel_(0.5), max_dvw_(1.0),
      goal_tol_(0.1), dt_(0.1), sim_time_(1.5),
      initialized_(false), goal_reached_(false) {}

DwaController::~DwaController() = default;

bool DwaController::initialize(const core::ControllerConfig& config) {
    cfg_ = config;
    max_vx_ = config.max_linear_velocity > 0 ? config.max_linear_velocity : 0.5;
    max_vw_ = config.max_angular_velocity > 0 ? config.max_angular_velocity : 1.0;
    max_accel_ = config.kinematics.max_velocity_x > 0 ? config.kinematics.max_velocity_x : 0.5;
    goal_tol_ = config.xy_goal_tolerance > 0 ? config.xy_goal_tolerance : 0.1;
    initialized_ = true;
    LOG_INFO(logger_, "DWA initialized: vmax=%.2f, wmax=%.2f, accel=%.2f", max_vx_, max_vw_, max_accel_);
    return true;
}

void DwaController::setPath(const core::Path& path) {
    path_ = path; idx_ = 0; goal_reached_ = false;
    LOG_INFO(logger_, "DWA path set: %zu points", path_.points.size());
}

core::VelocityCommand DwaController::computeVelocityCommand(
    const core::Pose2D& pose, const core::VelocityCommand& vel) {
    if (path_.points.empty() || !initialized_) return {0, 0, 0};

    auto samples = search(pose, vel);
    if (samples.empty()) return {0, 0, vel.angular_z};

    // Pick best
    auto best = std::min_element(samples.begin(), samples.end(),
        [](const Sample& a, const Sample& b) { return a.cost < b.cost; });

    // Simulate forward to find waypoint index
    double px = pose.x, py = pose.y, pt = pose.theta;
    for (double t = 0; t < sim_time_; t += dt_) {
        px += best->v * std::cos(pt) * dt_;
        py += best->v * std::sin(pt) * dt_;
        pt += best->w * dt_;
        size_t ci = findClosest({px, py, pt});
        if (ci > idx_) idx_ = ci;
    }

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
    int nv = 6, nw = 8;
    double dv = (bounds.max_vx - bounds.min_vx) / nv;
    double dw_ = (bounds.max_vw - bounds.min_vw) / nw;

    for (int i = 0; i <= nv; ++i) {
        for (int j = 0; j <= nw; ++j) {
            Sample s;
            s.v = bounds.min_vx + i * dv;
            s.w = bounds.min_vw + j * dw_;
            simulate(pose, s);
            s.heading = headingCost({pose.x, pose.y, pose.theta});
            s.clearance = clearanceCost({pose.x, pose.y, pose.theta});
            s.velocity = (std::abs(s.v) / max_vx_ + std::abs(s.w) / max_vw_) * 0.5;
            double h = 1.0 - s.heading;
            double c = 1.0 - std::min(s.clearance / 2.0, 1.0);
            double v = 1.0 - s.velocity;
            s.cost = 0.5 * h + 0.3 * c + 0.2 * v;
            if (s.clearance > 0.01 || s.heading < 0.1)
                samples.push_back(s);
        }
    }
    return samples;
}

DwaController::VelocityBounds DwaController::dynamicWindow(const core::VelocityCommand& vel) const {
    VelocityBounds b;
    b.min_vx = std::max(min_vx_, vel.linear_x - max_accel_ * dt_);
    b.max_vx = std::min(max_vx_, vel.linear_x + max_accel_ * dt_);
    b.min_vw = std::max(-max_vw_, vel.angular_z - max_dvw_ * dt_);
    b.max_vw = std::min(max_vw_, vel.angular_z + max_dvw_ * dt_);
    return b;
}

void DwaController::simulate(const core::Pose2D& pose, Sample& s) const {
    double px = pose.x, py = pose.y, pt = pose.theta;
    double min_dist = std::numeric_limits<double>::max();
    for (double t = 0; t < sim_time_; t += dt_) {
        px += s.v * std::cos(pt) * dt_;
        py += s.v * std::sin(pt) * dt_;
        pt += s.w * dt_;
        for (const auto& obs : obstacles_) {
            double dx = px - obs.x, dy = py - obs.y;
            double d = std::sqrt(dx * dx + dy * dy) - obs.radius;
            if (d < min_dist) min_dist = d;
            if (d < 0.1) { s.clearance = 0; return; }
        }
    }
    s.clearance = min_dist;
}

double DwaController::headingCost(const core::Pose2D& pose) const {
    if (path_.points.empty()) return 1;
    size_t target = std::min(idx_ + 5, path_.points.size() - 1);
    double dx = path_.points[target].pose.x - pose.x;
    double dy = path_.points[target].pose.y - pose.y;
    double goal_angle = std::atan2(dy, dx);
    return 1.0 - std::abs(normAngle(goal_angle - pose.theta)) / M_PI;
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
    if (path_.points.empty()) return true;
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
void DwaController::setObstacles(const core::ObstacleArray& obs) { obstacles_ = obs; }
double DwaController::normAngle(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

}}  // namespaces
