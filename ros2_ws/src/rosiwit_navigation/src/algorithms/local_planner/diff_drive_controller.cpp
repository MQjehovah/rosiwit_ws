#include "diff_drive_controller.hpp"
#include <algorithm>

namespace rosiwit_navigation {
namespace controllers {

DiffDriveController::DiffDriveController()
    : name_("diff_drive"), logger_("diff_drive"), idx_(0),
      max_vx_(0.5), max_vw_(1.0), lookahead_(0.6), goal_tol_(0.1),
      initialized_(false), goal_reached_(false) {}

DiffDriveController::~DiffDriveController() = default;

bool DiffDriveController::initialize(const core::ControllerConfig& config) {
    cfg_ = config;
    max_vx_ = config.max_linear_velocity > 0 ? config.max_linear_velocity : 0.5;
    max_vw_ = config.max_angular_velocity > 0 ? config.max_angular_velocity : 1.0;
    lookahead_ = config.lookahead_distance > 0 ? config.lookahead_distance : 0.6;
    goal_tol_ = config.xy_goal_tolerance > 0 ? config.xy_goal_tolerance : 0.1;
    initialized_ = true;
    LOG_INFO(logger_, "DiffDrive initialized: vmax=%.2f, wmax=%.2f, lookahead=%.2f",
             max_vx_, max_vw_, lookahead_);
    return true;
}

void DiffDriveController::setPath(const core::Path& path) {
    path_ = path;
    idx_ = 0;
    goal_reached_ = false;
    LOG_INFO(logger_, "Path set: %zu points", path_.points.size());
}

core::VelocityCommand DiffDriveController::computeVelocityCommand(
    const core::Pose2D& pose, const core::VelocityCommand& /*vel*/) {
    if (path_.points.empty() || !initialized_)
        return {0, 0, 0};

    // Find closest point on path
    size_t closest = findClosest(pose);

    // Look ahead
    double la = lookahead_;
    size_t target_idx = closest;
    double accum_dist = 0;
    for (size_t i = closest + 1; i < path_.points.size(); ++i) {
        double dx = path_.points[i].pose.x - path_.points[i - 1].pose.x;
        double dy = path_.points[i].pose.y - path_.points[i - 1].pose.y;
        accum_dist += std::sqrt(dx * dx + dy * dy);
        if (accum_dist >= la) { target_idx = i; break; }
        target_idx = i;
    }
    idx_ = target_idx;

    // Check goal
    double dxg = path_.points.back().pose.x - pose.x;
    double dyg = path_.points.back().pose.y - pose.y;
    if (std::sqrt(dxg * dxg + dyg * dyg) < goal_tol_) {
        goal_reached_ = true;
        return {0, 0, 0};
    }

    // Pure pursuit on lookahead point
    const auto& target = path_.points[target_idx].pose;
    return purePursuit(pose, target);
}

bool DiffDriveController::isGoalReached(const core::Pose2D& pose) const {
    if (path_.points.empty()) return true;
    double dx = path_.points.back().pose.x - pose.x;
    double dy = path_.points.back().pose.y - pose.y;
    return std::sqrt(dx * dx + dy * dy) < goal_tol_;
}

size_t DiffDriveController::getCurrentWaypointIndex() const { return idx_; }
double DiffDriveController::getProgress() const {
    if (path_.points.empty()) return 1.0;
    return static_cast<double>(idx_) / path_.points.size();
}
void DiffDriveController::reset() { idx_ = 0; goal_reached_ = false; path_.points.clear(); }
void DiffDriveController::setVelocityLimit(double vx, double vw) { max_vx_ = vx; max_vw_ = vw; }
std::string DiffDriveController::getName() const { return name_; }
std::string DiffDriveController::getVersion() const { return "1.0.0"; }
void DiffDriveController::setObstacles(const core::ObstacleArray&) {}

size_t DiffDriveController::findClosest(const core::Pose2D& pose) const {
    size_t best = 0;
    double best_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_.points.size(); ++i) {
        double dx = path_.points[i].pose.x - pose.x;
        double dy = path_.points[i].pose.y - pose.y;
        double d = dx * dx + dy * dy;
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

core::VelocityCommand DiffDriveController::purePursuit(
    const core::Pose2D& pose, const core::Pose2D& target) const {
    double dx = target.x - pose.x;
    double dy = target.y - pose.y;
    double alpha = normAngle(std::atan2(dy, dx) - pose.theta);

    double vx = std::clamp(max_vx_ * std::cos(alpha), -max_vx_ * 0.5, max_vx_);
    double wz = std::clamp(2.0 * std::sin(alpha) / lookahead_ * max_vx_, -max_vw_, max_vw_);
    return {vx, 0, wz};
}

double DiffDriveController::normAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

}}  // namespaces
