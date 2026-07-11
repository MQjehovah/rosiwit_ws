#include "rosiwit_navigation/nav_core/logger.hpp"
// ============================================================
// Diffbot Navigation - A* 规划器实现
// ============================================================

#include "astar_planner.hpp"
#include "rosiwit_navigation/nav_core/exceptions.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

namespace rosiwit_navigation
{
namespace planners
{

AStarPlanner::AStarPlanner()
    : planner_name_("astar")
    , logger_(core::Logger("astar_planner"))
    , planning_active_(false)
    , nx_(0), ny_(0)
    , max_iterations_(AStarConstants::kDefaultMaxIterations)
    , planning_timeout_seconds_(AStarConstants::kDefaultTimeoutSeconds)
    , iterations_(0)
    , use_weighted_heuristic_(false)
{
    // 超时和迭代次数使用 AStarConstants 默认值，
    // 可通过 configure(Config) 或 ROS2 参数在运行时调整
}

AStarPlanner::~AStarPlanner()
{
}

bool AStarPlanner::initialize(const core::PlannerConfig& config)
{
    config_ = config;
    planner_name_ = config.name.empty() ? "astar" : config.name;
    LOG_INFO(logger_, "A* planner initialized");
    return true;
}

void AStarPlanner::setCostmap(const core::Costmap& costmap)
{
    costmap_ = std::make_shared<core::Costmap>(costmap);

    if (!costmap_->grid) {
        LOG_ERROR(logger_, "Invalid costmap provided");
        return;
    }

    nx_ = costmap_->grid->info.width;
    ny_ = costmap_->grid->info.height;

    // 复制代价数据（OccupancyGrid: 0=free, 100=occupied, -1=unknown）
    costmap_data_.resize(nx_ * ny_);
    for (size_t i = 0; i < costmap_->grid->data.size(); ++i) {
        int8_t v = costmap_->grid->data[i];
        if (v == 0) {
            costmap_data_[i] = 0;                     // free
        } else if (v < 0 || v >= 100) {
            costmap_data_[i] = 255;                   // unknown or occupied → lethal
        } else {
            costmap_data_[i] = static_cast<unsigned char>(v * 255 / 100);  // scale
        }
    }

    // 初始化节点数组
    nodes_.resize(nx_ * ny_);

    // 初始化关闭列表（vector<bool> O(1) 数组访问，缓存友好）
    closed_list_.resize(nx_ * ny_, 0);

    // 障碍物膨胀（BFS计算到最近障碍物的距离）
    inflateCostmap();

    // 大网格（>100K 节点）启用加权启发式加速收敛
    use_weighted_heuristic_ = (nx_ * ny_ > 100000);

    LOG_INFO(logger_, "Costmap set: %dx%d, weighted_heuristic=%s",
        nx_, ny_, use_weighted_heuristic_ ? "true" : "false");
}

void AStarPlanner::setInflationRadius(double radius_meters)
{
    inflation_radius_ = radius_meters;
}

void AStarPlanner::inflateCostmap()
{
    if (inflation_radius_ <= 0.0) return;
    if (!costmap_) return;

    double res = costmap_->resolution;
    if (res <= 0.0) res = 0.05;

    int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / res));
    if (radius_cells < 1) return;

    // 安全缓冲区：障碍物往外 safety_cells 格标为 lethal（强制绕行）
    int safety_cells = std::max(1, static_cast<int>(std::ceil(0.15 / res)));

    // BFS: 从所有障碍物格子出发计算距离
    std::vector<float> dist(nx_ * ny_, std::numeric_limits<float>::max());
    std::queue<std::pair<int, int>> q;

    for (unsigned int y = 0; y < ny_; ++y) {
        for (unsigned int x = 0; x < nx_; ++x) {
            if (costmap_data_[y * nx_ + x] >= AStarConstants::kObstacleThreshold) {
                dist[y * nx_ + x] = 0.0f;
                q.push({x, y});
            }
        }
    }

    const int dx[4] = {-1, 0, 1, 0};
    const int dy[4] = {0, -1, 0, 1};

    while (!q.empty()) {
        auto [cx, cy] = q.front(); q.pop();
        float cd = dist[cy * nx_ + cx];
        float nd = cd + 1.0f;

        if (nd > radius_cells) continue;

        for (int i = 0; i < 4; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];
            if (nx >= 0 && nx < static_cast<int>(nx_) &&
                ny >= 0 && ny < static_cast<int>(ny_)) {
                size_t idx = static_cast<size_t>(ny) * nx_ + static_cast<size_t>(nx);
                if (nd < dist[idx]) {
                    dist[idx] = nd;
                    q.push({nx, ny});
                }
            }
        }
    }

    // 根据距离设置膨胀代价
    const unsigned char lethal = AStarConstants::kObstacleThreshold;
    float inv_radius = 1.0f / (radius_cells - safety_cells);

    for (size_t i = 0; i < nx_ * ny_; ++i) {
        if (costmap_data_[i] >= lethal) continue;

        float d = dist[i];
        if (d <= safety_cells) {
            // 安全缓冲区 → lethal，规划器必须绕行
            costmap_data_[i] = lethal;
        } else if (d <= radius_cells) {
            // 膨胀区 → 梯度代价（靠近障碍物越高）
            float ratio = 1.0f - (d - safety_cells) * inv_radius;
            costmap_data_[i] = std::max(costmap_data_[i],
                static_cast<unsigned char>(ratio * (lethal - 1)));
        }
    }

    LOG_INFO(logger_, "Costmap inflated: radius=%.2fm (%d cells)",
        inflation_radius_, radius_cells);
}

core::Result<core::Path> AStarPlanner::plan(
    const core::Pose2D& start, const core::Pose2D& goal)
{
    if (!costmap_) {
        return core::Result<core::Path>::error(
            core::ErrorCode::NOT_INITIALIZED, "Costmap not set");
    }

    clearSearchData();
    planning_active_ = true;
    iterations_ = 0;
    planning_start_time_ = std::chrono::steady_clock::now();

    // 确保 costmap 已设置
    if (!costmap_ || !costmap_->grid) {
        planning_active_ = false;
        LOG_ERROR(logger_, "Costmap not initialized");
        return core::Result<core::Path>::error(
            core::ErrorCode::PLANNING_FAILED, "Costmap not initialized");
    }

    float resolution = costmap_->grid->info.resolution;
    float origin_x = costmap_->grid->info.origin.position.x;
    float origin_y = costmap_->grid->info.origin.position.y;

    int start_x = static_cast<int>((start.x - origin_x) / resolution);
    int start_y = static_cast<int>((start.y - origin_y) / resolution);
    int goal_x = static_cast<int>((goal.x - origin_x) / resolution);
    int goal_y = static_cast<int>((goal.y - origin_y) / resolution);

    // 检查边界
    if (!isValid(start_x, start_y) || !isValid(goal_x, goal_y)) {
        planning_active_ = false;
        return core::Result<core::Path>::error(
            core::ErrorCode::START_IN_OBSTACLE, "Start or goal invalid");
    }

    if (isObstacle(start_x, start_y) || isObstacle(goal_x, goal_y)) {
        planning_active_ = false;
        return core::Result<core::Path>::error(
            core::ErrorCode::GOAL_IN_OBSTACLE, "Start or goal in obstacle");
    }

    // 起点=终点时直接返回
    if (start_x == goal_x && start_y == goal_y) {
        planning_active_ = false;
        core::Path path;
        core::PathPoint pt;
        pt.pose = start;
        path.points.push_back(pt);
        return core::Result<core::Path>::ok(path);
    }

    // 初始化起点
    AStarNode& start_node = nodes_[start_y * nx_ + start_x];
    start_node.x = start_x;
    start_node.y = start_y;
    start_node.g_cost = 0;
    start_node.h_cost = heuristic(start_x, start_y, goal_x, goal_y);
    start_node.f_cost = start_node.g_cost + start_node.h_cost;
    start_node.parent_x = -1;
    start_node.parent_y = -1;

    open_list_.push(start_node);
    closed_list_[start_y * nx_ + start_x] = 1;
    closed_indices_.push_back(start_y * nx_ + start_x);

    // A*搜索（带迭代限制和超时保护）
    while (!open_list_.empty() && planning_active_) {
        // 检查迭代次数限制
        if (++iterations_ > max_iterations_) {
            planning_active_ = false;
            LOG_WARN(logger_, "A* exceeded max iterations (%d)", max_iterations_);
            return core::Result<core::Path>::error(
                core::ErrorCode::TIMEOUT, "Planning iteration limit exceeded");
        }

        // 检查超时
        auto elapsed = std::chrono::steady_clock::now() - planning_start_time_;
        if (std::chrono::duration<double>(elapsed).count() > planning_timeout_seconds_) {
            planning_active_ = false;
            LOG_WARN(logger_, "A* planning timeout (%.2fs)", planning_timeout_seconds_);
            return core::Result<core::Path>::error(
                core::ErrorCode::TIMEOUT, "Planning timeout exceeded");
        }

        AStarNode current = open_list_.top();
        open_list_.pop();

        // 到达目标
        if (current.x == goal_x && current.y == goal_y) {
            core::Path path;
            reconstructPath(current, path);
            planning_active_ = false;
            return core::Result<core::Path>::ok(path);
        }

        // 扩展8个方向邻居
        const int dx[] = {-1, 0, 1, 0, -1, 1, -1, 1};
        const int dy[] = {0, -1, 0, 1, -1, -1, 1, 1};

        for (int i = 0; i < 8; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!isValid(nx, ny) || closed_list_[ny * nx_ + nx]) {
                continue;
            }

            if (isObstacle(nx, ny)) {
                continue;
            }

            // 计算代价
            double move_cost = (i < 4) ? AStarConstants::kStraightCost : AStarConstants::kDiagonalCost;
            double cost_factor = 1.0 + getCost(nx, ny) / AStarConstants::kMaxCost * AStarConstants::kCostFactorScale;

            AStarNode& neighbor = nodes_[ny * nx_ + nx];
            double new_g_cost = current.g_cost + move_cost * cost_factor;

            if (neighbor.parent_x == -1 || new_g_cost < neighbor.g_cost) {
                neighbor.x = nx;
                neighbor.y = ny;
                neighbor.g_cost = new_g_cost;
                neighbor.h_cost = heuristic(nx, ny, goal_x, goal_y);
                neighbor.f_cost = neighbor.g_cost + neighbor.h_cost;
                neighbor.parent_x = current.x;
                neighbor.parent_y = current.y;

                open_list_.push(neighbor);
                closed_list_[ny * nx_ + nx] = 1;
                closed_indices_.push_back(ny * nx_ + nx);
            }
        }
    }

    planning_active_ = false;
    return core::Result<core::Path>::error(
        core::ErrorCode::NO_VALID_PATH, "No valid path found");
}

void AStarPlanner::planAsync(
    const core::Pose2D& start, const core::Pose2D& goal,
    std::function<void(const core::Result<core::Path>&)> callback)
{
    std::thread([this, start, goal, callback]() {
        auto result = plan(start, goal);
        callback(result);
    }).detach();
}

void AStarPlanner::cancel()
{
    planning_active_ = false;
}

bool AStarPlanner::isPlanning() const
{
    return planning_active_;
}

std::string AStarPlanner::getName() const
{
    return planner_name_;
}

std::string AStarPlanner::getVersion() const
{
    return "1.0.0";
}

void AStarPlanner::reset()
{
    planning_active_ = false;
    clearSearchData();
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) const
{
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    double h;

    if (simple_config_.use_diagonal_moves) {
        // 八分距离（无 sqrt，适合网格搜索）
        double diag = static_cast<double>(std::min(dx, dy));
        double straight = static_cast<double>(std::max(dx, dy) - std::min(dx, dy));
        h = diag * AStarConstants::kDiagonalCost + straight * AStarConstants::kStraightCost;
    } else {
        // 曼哈顿距离（无 sqrt）
        h = static_cast<double>(dx + dy);
    }

    // 大网格加权启发式：使用可配置权重加速搜索
    if (use_weighted_heuristic_) {
        h *= simple_config_.heuristic_weight;
    }
    return h;
}

bool AStarPlanner::isValid(int x, int y) const
{
    return x >= 0 && x < static_cast<int>(nx_) &&
           y >= 0 && y < static_cast<int>(ny_);
}

bool AStarPlanner::isObstacle(int x, int y) const
{
    return getCost(x, y) >= AStarConstants::kObstacleThreshold;
}

unsigned char AStarPlanner::getCost(int x, int y) const
{
    if (!isValid(x, y)) {
        return 255;
    }
    return costmap_data_[y * nx_ + x];
}

void AStarPlanner::reconstructPath(const AStarNode& goal_node, core::Path& path)
{
    std::vector<core::PathPoint> path_points;

    float resolution = costmap_->grid->info.resolution;
    float origin_x = costmap_->grid->info.origin.position.x;
    float origin_y = costmap_->grid->info.origin.position.y;

    // 从目标回溯到起点
    std::vector<std::pair<int, int>> waypoints;
    int current_x = goal_node.x;
    int current_y = goal_node.y;

    while (current_x >= 0 && current_y >= 0) {
        waypoints.push_back({current_x, current_y});

        AStarNode& node = nodes_[current_y * nx_ + current_x];
        current_x = node.parent_x;
        current_y = node.parent_y;
    }

    // 反转路径（从起点到目标）
    std::reverse(waypoints.begin(), waypoints.end());

    for (const auto& [x, y] : waypoints) {
        float world_x = origin_x + x * resolution;
        float world_y = origin_y + y * resolution;
        core::Pose2D pose(world_x, world_y, 0.0);
        path_points.push_back(core::PathPoint(pose));
    }

    path.points = path_points;
}

void AStarPlanner::clearSearchData()
{
    open_list_ = std::priority_queue<AStarNode>();

    // 使用 vector::assign 快速清空（O(n)但缓存友好）
    closed_list_.assign(closed_list_.size(), 0);

    // 只重置被访问过的节点（避免大网格全量重置）
    for (int idx : closed_indices_) {
        if (idx >= 0 && idx < static_cast<int>(nodes_.size())) {
            nodes_[idx] = AStarNode();  // 重置为默认构造
        }
    }
    closed_indices_.clear();
}

void AStarPlanner::configure(const Config& config)
{
    Config temp = config;
    // 栅格分辨率安全下限
    if (temp.grid_resolution <= 0.0) {
        temp.grid_resolution = AStarConstants::kDefaultGridResolution;
        LOG_WARN(logger_,
            "grid_resolution=%.6f 非法，重置为默认值 %.3f",
            config.grid_resolution, temp.grid_resolution);
    }

    // 同步运行时可调整参数
    max_iterations_ = temp.max_iterations;
    planning_timeout_seconds_ = temp.timeout_seconds;

    // 栅格分辨率变更时触发重新初始化
    if (std::abs(temp.grid_resolution - simple_config_.grid_resolution) > 1e-9) {
        LOG_INFO(logger_,
            "Grid resolution changed: %.4f → %.4f, reinitializing",
            simple_config_.grid_resolution, temp.grid_resolution);
        nx_ = 0;
        ny_ = 0;
        nodes_.clear();
        closed_list_.clear();
        costmap_data_.clear();
    }

    simple_config_ = temp;
}

AStarResult AStarPlanner::plan(
    const std::shared_ptr<OccupancyGrid>& grid,
    const PathPoint& start,
    const PathPoint& goal)
{
    AStarResult result;
    result.success = false;

    // 边界检查：网格为空
    if (!grid || grid->data.empty()) {
        LOG_ERROR(logger_, "A* plan(): null or empty occupancy grid");
        return result;
    }

    const unsigned int nx = grid->info.width;
    const unsigned int ny = grid->info.height;
    const double resolution = grid->info.resolution;
    const double origin_x = grid->info.origin.x;
    const double origin_y = grid->info.origin.y;

    // 世界坐标转网格坐标
    auto worldToGrid = [&](double wx, double wy) -> std::pair<int, int> {
        int gx = static_cast<int>((wx - origin_x) / resolution);
        int gy = static_cast<int>((wy - origin_y) / resolution);
        return {gx, gy};
    };

    auto grid_start = worldToGrid(start.x, start.y);
    int sx = grid_start.first;
    int sy = grid_start.second;
    auto grid_goal = worldToGrid(goal.x, goal.y);
    int gx = grid_goal.first;
    int gy = grid_goal.second;

    // 边界检查：起点/终点越界
    if (sx < 0 || sx >= static_cast<int>(nx) || sy < 0 || sy >= static_cast<int>(ny)) {
        LOG_WARN(logger_,
            "A* plan(): start (%d, %d) out of bounds [0-%u, 0-%u]", sx, sy, nx, ny);
        return result;
    }
    if (gx < 0 || gx >= static_cast<int>(nx) || gy < 0 || gy >= static_cast<int>(ny)) {
        LOG_WARN(logger_,
            "A* plan(): goal (%d, %d) out of bounds [0-%u, 0-%u]", gx, gy, nx, ny);
        return result;
    }

    // 起点 = 终点 → 返回单点路径
    if (sx == gx && sy == gy) {
        result.success = true;
        result.iterations = 0;
        result.elapsed_ms = 0.0;
        PathPoint p;
        p.x = start.x;
        p.y = start.y;
        p.theta = 0.0;
        result.path = {p};
        return result;
    }

    // 障碍物检测：起点或终点在障碍物上
    const int obstacle_threshold = simple_config_.obstacle_cost_threshold;
    auto isObstacleGrid = [&](int x, int y) -> bool {
        if (x < 0 || x >= static_cast<int>(nx) || y < 0 || y >= static_cast<int>(ny)) return true;
        return grid->data[y * nx + x] >= obstacle_threshold;
    };

    if (isObstacleGrid(sx, sy)) {
        LOG_WARN(logger_, "A* plan(): start (%d, %d) is on obstacle", sx, sy);
        return result;
    }
    if (isObstacleGrid(gx, gy)) {
        LOG_WARN(logger_, "A* plan(): goal (%d, %d) is on obstacle", gx, gy);
        return result;
    }

    // ===================================================================
    // A* 核心搜索
    // ===================================================================
    const int max_iter = simple_config_.max_iterations;
    const double timeout = simple_config_.timeout_seconds;
    const double h_weight = simple_config_.heuristic_weight;
    const bool use_diag = simple_config_.use_diagonal_moves;

    auto start_time = std::chrono::steady_clock::now();

    // 预分配节点数组（用 vector 代替 unordered_map 提升性能）
    struct SearchNode {
        double g = std::numeric_limits<double>::infinity();
        double f = std::numeric_limits<double>::infinity();
        int parent_x = -1;
        int parent_y = -1;
        bool closed = false;
    };

    std::vector<SearchNode> search_nodes(nx * ny);

    auto heuristic = [&](int x, int y) -> double {
        int dx = std::abs(x - gx);
        int dy = std::abs(y - gy);
        if (use_diag) {
            if (dx > dy) return h_weight * (AStarConstants::kDiagonalCost * dy + (dx - dy));
            else return h_weight * (AStarConstants::kDiagonalCost * dx + (dy - dx));
        } else {
            return h_weight * (dx + dy);
        }
    };

    // 使用索引对优先队列（小顶堆，按 f 值排序）
    struct OpenNode {
        int x, y;
        double f;
        bool operator>(const OpenNode& other) const { return f > other.f; }
    };
    std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open_set;

    // 初始化起点
    search_nodes[sy * nx + sx].g = 0.0;
    search_nodes[sy * nx + sx].f = heuristic(sx, sy);
    open_set.push({sx, sy, search_nodes[sy * nx + sx].f});

    int iter = 0;
    bool found = false;

    // 移动方向：8邻域
    const int dx_diag[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy_diag[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    const double cost_diag[8] = {1.0, 1.414, 1.0, 1.414, 1.0, 1.414, 1.0, 1.414};
    const int dir_count = use_diag ? 8 : 4;

    while (!open_set.empty() && iter < max_iter) {
        // 超时检查（每 1000 次迭代检查一次）
        if (iter % 1000 == 0) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(now - start_time).count();
            if (elapsed > timeout * 1000.0) {
                LOG_WARN(logger_, "A* timeout after %d iterations (%.1f ms)", iter, elapsed);
                break;
            }
        }

        OpenNode current = open_set.top();
        open_set.pop();

        SearchNode& cur_node = search_nodes[current.y * nx + current.x];
        if (cur_node.closed) continue;
        cur_node.closed = true;

        if (current.x == gx && current.y == gy) {
            found = true;
            break;
        }

        for (int d = 0; d < dir_count; ++d) {
            int nx2 = current.x + dx_diag[d];
            int ny2 = current.y + dy_diag[d];

            if (nx2 < 0 || nx2 >= static_cast<int>(nx) ||
                ny2 < 0 || ny2 >= static_cast<int>(ny)) continue;

            if (isObstacleGrid(nx2, ny2)) continue;

            // 对角线移动需要检查相邻格无障碍
            if (d % 2 == 1) {
                if (isObstacleGrid(current.x + dx_diag[d], current.y) ||
                    isObstacleGrid(current.x, current.y + dy_diag[d])) {
                    continue;
                }
            }

            double new_g = cur_node.g + cost_diag[d] * resolution;
            SearchNode& neighbor = search_nodes[ny2 * nx + nx2];

            if (new_g < neighbor.g) {
                neighbor.g = new_g;
                neighbor.f = new_g + heuristic(nx2, ny2);
                neighbor.parent_x = current.x;
                neighbor.parent_y = current.y;
                open_set.push({nx2, ny2, neighbor.f});
            }
        }
        ++iter;
    }

    // ===================================================================
    // 构建返回结果
    // ===================================================================
    result.iterations = iter;

    auto end_time = std::chrono::steady_clock::now();
    result.elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    if (!found) {
        LOG_INFO(logger_,
            "A* plan(): no path found after %d iterations (%.1f ms)", iter, result.elapsed_ms);
        return result;
    }

    // 回溯路径
    std::vector<std::pair<int, int>> grid_path;
    int cx = gx, cy = gy;
    while (cx != sx || cy != sy) {
        grid_path.push_back({cx, cy});
        SearchNode& node = search_nodes[cy * nx + cx];
        int px = node.parent_x;
        int py = node.parent_y;
        if (px < 0 || py < 0) break;
        cx = px;
        cy = py;
    }
    grid_path.push_back({sx, sy});

    // 反转（起点→终点）并转为世界坐标
    for (auto it = grid_path.rbegin(); it != grid_path.rend(); ++it) {
        PathPoint p;
        p.x = origin_x + it->first * resolution + resolution * 0.5;
        p.y = origin_y + it->second * resolution + resolution * 0.5;
        p.theta = 0.0;
        result.path.push_back(p);
    }

    result.success = true;
    LOG_INFO(logger_,
        "A* plan(): found path length=%zu after %d iterations (%.1f ms)",
        result.path.size(), iter, result.elapsed_ms);

    return result;
}

}  // namespace planners
}  // namespace rosiwit_navigation