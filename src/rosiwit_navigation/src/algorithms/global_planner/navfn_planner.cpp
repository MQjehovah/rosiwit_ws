#include "rosiwit_navigation/nav_core/logger.hpp"
// ============================================================
// Diffbot Navigation - NavFn 规划器实现
// ============================================================

#include "navfn_planner.hpp"
#include "rosiwit_navigation/nav_core/exceptions.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <mutex>
#include <thread>

namespace rosiwit_navigation
{
namespace planners
{

NavFnPlanner::NavFnPlanner()
    : planner_name_("navfn")
    , logger_(core::Logger("navfn_planner"))
    , nx_(0), ny_(0)
    , planning_active_(false)
{
}

NavFnPlanner::~NavFnPlanner()
{
}

bool NavFnPlanner::initialize(const core::PlannerConfig& config)
{
    config_ = config;
    planner_name_ = config.name.empty() ? "navfn" : config.name;
    LOG_INFO(logger_, "NavFn planner initialized");
    return true;
}

void NavFnPlanner::setCostmap(const core::Costmap& costmap)
{
    costmap_ = std::make_shared<core::Costmap>(costmap);

    if (!costmap_->grid) {
        LOG_ERROR(logger_, "Invalid costmap provided");
        return;
    }

    nx_ = costmap_->grid->info.width;
    ny_ = costmap_->grid->info.height;

    // 全链路统一语义 (nav_msgs/OccupancyGrid): -1 unknown / 0..99 代价 / 100 lethal.
    // 不做换算: 层、Costmap、消息、planner 内部数组是同一套数值.
    costmap_data_.assign(static_cast<size_t>(nx_) * ny_, 0);
    const bool use_data = costmap_->data.size() == costmap_data_.size();
    for (size_t i = 0; i < costmap_data_.size(); ++i) {
        if (use_data) {
            costmap_data_[i] = costmap_->data[i];
        } else if (i < costmap_->grid->data.size()) {
            costmap_data_[i] = costmap_->grid->data[i];
        }
    }

    potential_.resize(nx_ * ny_, -1.0f);
    gradx_.resize(nx_ * ny_, 0);
    grady_.resize(nx_ * ny_, 0);

    LOG_INFO(logger_, "Costmap set: %dx%d", nx_, ny_);
}

core::Result<core::Path> NavFnPlanner::plan(
    const core::Pose2D& start, const core::Pose2D& goal)
{
    if (!costmap_) {
        return core::Result<core::Path>::error(
            core::ErrorCode::NOT_INITIALIZED, "Costmap not set");
    }

    planning_active_ = true;

    // 计算势场
    if (!computePotential(goal)) {
        planning_active_ = false;
        return core::Result<core::Path>::error(
            core::ErrorCode::GOAL_IN_OBSTACLE, "Goal invalid");
    }

    // 寻找路径
    core::Path path;
    if (findPath(start, goal, path)) {
        planning_active_ = false;
        return core::Result<core::Path>::ok(path);
    }

    planning_active_ = false;
    return core::Result<core::Path>::error(
        core::ErrorCode::NO_VALID_PATH, "No valid path found");
}

void NavFnPlanner::planAsync(
    const core::Pose2D& start, const core::Pose2D& goal,
    std::function<void(const core::Result<core::Path>&)> callback)
{
    // 在后台线程执行规划
    std::thread([this, start, goal, callback]() {
        auto result = plan(start, goal);
        callback(result);
    }).detach();
}

void NavFnPlanner::cancel()
{
    planning_active_ = false;
}

bool NavFnPlanner::isPlanning() const
{
    return planning_active_;
}

std::string NavFnPlanner::getName() const
{
    return planner_name_;
}

std::string NavFnPlanner::getVersion() const
{
    return "1.0.0";
}

void NavFnPlanner::reset()
{
    planning_active_ = false;
    potential_.clear();
    gradx_.clear();
    grady_.clear();
}

bool NavFnPlanner::computePotential(const core::Pose2D& goal)
{
    // 使用波前传播计算势场
    std::fill(potential_.begin(), potential_.end(), -1.0f);

    // 目标点势场值为0
    float resolution = static_cast<float>(costmap_->grid->info.resolution);
    float origin_x = static_cast<float>(costmap_->grid->info.origin.position.x);
    float origin_y = static_cast<float>(costmap_->grid->info.origin.position.y);

    int goal_x = static_cast<int>(std::floor((goal.x - origin_x) / resolution));
    int goal_y = static_cast<int>(std::floor((goal.y - origin_y) / resolution));

    if (goal_x < 0 || goal_x >= static_cast<int>(nx_) ||
        goal_y < 0 || goal_y >= static_cast<int>(ny_) ||
        !isPassable(static_cast<unsigned int>(goal_x), static_cast<unsigned int>(goal_y))) {
        return false;
    }
    potential_[goal_y * nx_ + goal_x] = 0.0f;

    // 波前传播
    std::queue<std::pair<int, int>> wavefront;
    wavefront.push({goal_x, goal_y});

    while (!wavefront.empty() && planning_active_) {
        auto [x, y] = wavefront.front();
        wavefront.pop();

        float current_potential = potential_[y * nx_ + x];

        // 8方向传播
        const int dx[] = {-1, 0, 1, 0, -1, 1, -1, 1};
        const int dy[] = {0, -1, 0, 1, -1, -1, 1, 1};

        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (nx >= 0 && nx < static_cast<int>(nx_) &&
                ny >= 0 && ny < static_cast<int>(ny_)) {
                if (potential_[ny * nx_ + nx] < 0 &&
                    isPassable(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))) {
                    potential_[ny * nx_ + nx] = current_potential +
                        (i < 4 ? 1.0f : 1.414f);  // 直线vs对角线
                    wavefront.push({nx, ny});
                }
            }
        }
    }
    return planning_active_;
}

bool NavFnPlanner::findPath(
    const core::Pose2D& start, const core::Pose2D& goal, core::Path& path)
{
    float resolution = static_cast<float>(costmap_->grid->info.resolution);
    float origin_x = static_cast<float>(costmap_->grid->info.origin.position.x);
    float origin_y = static_cast<float>(costmap_->grid->info.origin.position.y);

    int start_x = static_cast<int>(std::floor((start.x - origin_x) / resolution));
    int start_y = static_cast<int>(std::floor((start.y - origin_y) / resolution));

    if (start_x < 0 || start_x >= static_cast<int>(nx_) ||
        start_y < 0 || start_y >= static_cast<int>(ny_)) {
        return false;
    }

    // 从起点沿势场梯度下降到目标
    std::vector<core::PathPoint> path_points;
    int current_x = start_x;
    int current_y = start_y;

    while (planning_active_) {
        // 添加当前位置到路径
        float world_x = origin_x + current_x * resolution;
        float world_y = origin_y + current_y * resolution;
        core::Pose2D pose(world_x, world_y, 0.0);
        core::PathPoint pp(pose);
        pp.velocity_x = 1.0;   // 前进 (0 = 旋转段标记, 见 trajectory_parameterizer)
        path_points.push_back(pp);

        // 找到势场最小值方向
        float min_potential = potential_[current_y * nx_ + current_x];
        int next_x = current_x;
        int next_y = current_y;

        const int dx[] = {-1, 0, 1, 0, -1, 1, -1, 1};
        const int dy[] = {0, -1, 0, 1, -1, -1, 1, 1};

        for (int i = 0; i < 8; ++i) {
            int nx = current_x + dx[i];
            int ny = current_y + dy[i];

            if (nx >= 0 && nx < static_cast<int>(nx_) &&
                ny >= 0 && ny < static_cast<int>(ny_)) {
                float pot = potential_[ny * nx_ + nx];
                if (pot >= 0 && pot < min_potential) {
                    min_potential = pot;
                    next_x = nx;
                    next_y = ny;
                }
            }
        }

        // 到达目标或无法继续
        if (min_potential == 0.0f || (next_x == current_x && next_y == current_y)) {
            break;
        }

        current_x = next_x;
        current_y = next_y;
    }

    if (!path_points.empty()) {
        // 设置路径
        path.points = path_points;
        return true;
    }

    return false;
}

int8_t NavFnPlanner::getCost(unsigned int x, unsigned int y)
{
    if (x >= nx_ || y >= ny_) {
        return -1;   // 界外 = 不可通行
    }
    return costmap_data_[y * nx_ + x];
}

bool NavFnPlanner::isPassable(unsigned int x, unsigned int y)
{
    const int8_t c = getCost(x, y);
    return c >= 0 && c < 100;   // -1 unknown 与 100 lethal 都不可通行
}

}  // namespace planners
}  // namespace rosiwit_navigation