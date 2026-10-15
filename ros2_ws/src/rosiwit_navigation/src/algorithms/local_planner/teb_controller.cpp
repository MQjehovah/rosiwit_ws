#include "teb_controller.hpp"
#include <algorithm>
#include <limits>

namespace rosiwit_navigation {
namespace controllers {

TebController::TebController()
    : name_("teb"), logger_("teb"), idx_(0),
      max_vx_(0.5), max_vw_(1.0), goal_tol_(0.1),
      initialized_(false), goal_reached_(false) {}

TebController::~TebController() = default;

bool TebController::initialize(const core::ControllerConfig& config) {
    cfg_ = config;
    max_vx_ = config.max_linear_velocity > 0 ? config.max_linear_velocity : 0.5;
    max_vw_ = config.max_angular_velocity > 0 ? config.max_angular_velocity : 1.0;
    goal_tol_ = config.xy_goal_tolerance > 0 ? config.xy_goal_tolerance : 0.1;
    initialized_ = true;
    LOG_INFO(logger_, "TEB initialized: vmax=%.2f, wmax=%.2f", max_vx_, max_vw_);
    return true;
}

void TebController::setPath(const core::Path& path) {
    path_ = path;
    idx_ = 0;
    goal_reached_ = false;
    initTeb();
    // Run a few optimization iterations
    for (int i = 0; i < 50; ++i) optimize();
    LOG_INFO(logger_, "TEB path set: %zu poses -> %zu teb poses", path_.points.size(), teb_.size());
}

void TebController::initTeb() {
    teb_.clear();
    if (path_.points.empty()) return;
    double total_dt = path_.points.size() * 0.1;
    for (size_t i = 0; i < path_.points.size(); ++i) {
        double dt = (i == 0) ? 0.1 : (total_dt / path_.points.size());
        teb_.push_back({path_.points[i].pose.x, path_.points[i].pose.y,
                        path_.points[i].pose.theta, dt});
    }
}

void TebController::optimize() {
    if (teb_.size() < 3) return;
    double alpha = 0.1;  // step size
    int n = static_cast<int>(teb_.size());

    // Gradient descent on interior poses
    for (int iter = 0; iter < 10; ++iter) {
        std::vector<Pose> grad(n, {0,0,0,0});

        for (int i = 1; i < n - 1; ++i) {
            // Smoothness: pull toward midpoint of neighbors
            grad[i].x = (teb_[i-1].x + teb_[i+1].x) * 0.5 - teb_[i].x;
            grad[i].y = (teb_[i-1].y + teb_[i+1].y) * 0.5 - teb_[i].y;
            grad[i].theta = (teb_[i-1].theta + teb_[i+1].theta) * 0.5 - teb_[i].theta;

            // Obstacle repulsion
            for (const auto& obs : obstacles_) {
                double dx = teb_[i].x - obs.x;
                double dy = teb_[i].y - obs.y;
                double d = std::sqrt(dx*dx + dy*dy);
                if (d < 0.5 && d > 0.01) {
                    double force = 0.3 / (d * d);
                    grad[i].x += dx / d * force;
                    grad[i].y += dy / d * force;
                }
            }

            // Time optimal: reduce dt
            if (i > 0) {
                grad[i].dt -= 0.01;
            }
        }

        // Apply gradient
        for (int i = 1; i < n - 1; ++i) {
            teb_[i].x += alpha * grad[i].x;
            teb_[i].y += alpha * grad[i].y;
            teb_[i].theta += alpha * std::clamp(grad[i].theta, -0.1, 0.1);
            teb_[i].dt = std::max(teb_[i].dt + alpha * grad[i].dt, 0.05);
        }
    }

    // Add/remove poses based on distance
    addPoses();
    removePoses();
}

void TebController::addPoses() {
    for (size_t i = 1; i < teb_.size(); ++i) {
        double d = dist(teb_[i-1].x, teb_[i-1].y, teb_[i].x, teb_[i].y);
        if (d > 0.3) {
            Pose mid;
            mid.x = (teb_[i-1].x + teb_[i].x) * 0.5;
            mid.y = (teb_[i-1].y + teb_[i].y) * 0.5;
            mid.theta = (teb_[i-1].theta + teb_[i].theta) * 0.5;
            mid.dt = (teb_[i-1].dt + teb_[i].dt) * 0.5;
            teb_.insert(teb_.begin() + static_cast<std::ptrdiff_t>(i), mid);
            ++i;
        }
    }
}

void TebController::removePoses() {
    for (size_t i = 1; i < teb_.size(); ++i) {
        double d = dist(teb_[i-1].x, teb_[i-1].y, teb_[i].x, teb_[i].y);
        if (d < 0.05 && teb_.size() > 3) {
            teb_.erase(teb_.begin() + static_cast<std::ptrdiff_t>(i));
            --i;
        }
    }
}

core::VelocityCommand TebController::computeVelocityCommand(
    const core::Pose2D& pose, const core::VelocityCommand& /*vel*/) {
    if (teb_.size() < 2 || !initialized_) return {0, 0, 0};

    // Find closest TEB pose
    size_t closest = 0;
    double best_d = std::numeric_limits<double>::max();
    for (size_t i = 1; i < teb_.size(); ++i) {
        double d = dist(teb_[i].x, teb_[i].y, pose.x, pose.y);
        if (d < best_d) { best_d = d; closest = i; }
    }
    idx_ = closest;

    // Look ahead 3 poses
    size_t target = std::min(closest + 3, teb_.size() - 1);
    const auto& tp = teb_[target];

    double dx = tp.x - pose.x;
    double dy = tp.y - pose.y;
    double alpha = normAngle(std::atan2(dy, dx) - pose.theta);
    double lookahead = std::sqrt(dx*dx + dy*dy);

    // Goal check
    double dxg = teb_.back().x - pose.x;
    double dyg = teb_.back().y - pose.y;
    if (std::sqrt(dxg*dxg + dyg*dyg) < goal_tol_) {
        goal_reached_ = true;
        return {0, 0, 0};
    }

    double vx = std::clamp(max_vx_ * std::cos(alpha), -max_vx_ * 0.5, max_vx_);
    double wz = lookahead > 0.01
        ? std::clamp(2.0 * std::sin(alpha) / lookahead * max_vx_, -max_vw_, max_vw_)
        : 0;

    return {vx, 0, wz};
}

bool TebController::isGoalReached(const core::Pose2D& pose) const {
    if (teb_.empty()) return true;
    double dx = teb_.back().x - pose.x;
    double dy = teb_.back().y - pose.y;
    return std::sqrt(dx*dx + dy*dy) < goal_tol_;
}

size_t TebController::getCurrentWaypointIndex() const { return idx_; }
double TebController::getProgress() const {
    if (teb_.empty()) return 1.0;
    return static_cast<double>(idx_) / teb_.size();
}
void TebController::reset() { teb_.clear(); idx_ = 0; goal_reached_ = false; }
void TebController::setVelocityLimit(double vx, double vw) { max_vx_ = vx; max_vw_ = vw; }
std::string TebController::getName() const { return name_; }
std::string TebController::getVersion() const { return "1.0.0"; }
void TebController::setObstacles(const core::ObstacleArray& obs) { obstacles_ = obs; }
double TebController::normAngle(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}
double TebController::dist(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

}}  // namespaces
