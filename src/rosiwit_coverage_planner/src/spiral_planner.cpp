// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include "coverage_planner/spiral_planner.hpp"
#include "coverage_planner/zigzag_planner.hpp"
#include <algorithm>
#include <queue>
#include <cmath>
#include <limits>

namespace coverage_planner
{

SpiralPlanner::SpiralPlanner()
: robot_radius_(0.3),
  coverage_resolution_(0.1),
  spiral_direction_(SpiralDirection::CLOCKWISE),
  enable_fallback_(true),
  enable_turn_optimization_(false),
  turn_angle_threshold_(0.1),
  turn_merge_distance_(10.0)
{
}

void SpiralPlanner::reset()
{
    coverage_mask_.clear();
    turn_optimizer_.reset();
}

PlannerResult SpiralPlanner::plan(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::Pose & start_pose,
    const PlannerConfig & config)
{
    PlannerResult result;
    result.success = false;
    result.coverage_rate = 0.0;
    result.path_length = 0.0;
    result.turn_count = 0;

    // 保存配置参数
    robot_radius_ = config.robot_radius;
    coverage_resolution_ = config.coverage_resolution;
    enable_fallback_ = true;  // 默认启用降级策略

    // 保存P0优化参数
    enable_turn_optimization_ = config.enable_turn_optimization;
    turn_angle_threshold_ = config.turn_angle_threshold;
    turn_merge_distance_ = config.turn_merge_distance;

    // 膨胀地图
    inflated_map_ = MapUtils::inflateMap(map, robot_radius_);

    // 转换起始位置到栅格坐标
    Point2D start_grid = MapUtils::worldToGrid(
        map, start_pose.position.x, start_pose.position.y);

    // 检查起始位置是否有效
    if (!MapUtils::isInBounds(inflated_map_, start_grid.x, start_grid.y) ||
        MapUtils::isObstacle(inflated_map_, start_grid.x, start_grid.y)) {
        result.error_message = "Start position is out of bounds or on an obstacle";

        // 尝试降级到弓字形算法
        if (enable_fallback_) {
            return fallbackToZigzag(map, start_pose, config);
        }
        return result;
    }

    // 获取区域边界
    RegionBoundary boundary = getRegionBoundary(inflated_map_);

    if (boundary.width() <= 0 || boundary.height() <= 0) {
        result.error_message = "No valid free region found in the map";

        if (enable_fallback_) {
            return fallbackToZigzag(map, start_pose, config);
        }
        return result;
    }

    // 检查是否为凸区域
    if (!isConvexRegion(inflated_map_, boundary)) {
        // 分解区域
        std::vector<RegionBoundary> sub_regions = decomposeRegion(inflated_map_);

        if (sub_regions.empty()) {
            // 无法分解，降级到弓字形
            if (enable_fallback_) {
                return fallbackToZigzag(map, start_pose, config);
            }
            result.error_message = "Failed to decompose non-convex region";
            return result;
        }

        // 对每个子区域生成螺旋路径
        std::vector<std::vector<Point2D>> spiral_paths;
        for (const auto & region : sub_regions) {
            std::vector<Point2D> spiral = generateSpiralPath(inflated_map_, region, start_grid);
            if (!spiral.empty()) {
                spiral_paths.push_back(spiral);
            }
        }

        // 连接所有螺旋路径
        std::vector<Point2D> full_path = connectSpiralRegions(inflated_map_, spiral_paths, start_grid);

        // 填充未被覆盖的区域
        full_path = fillUncoveredAreas(inflated_map_, full_path);

        // 简化路径
        full_path = simplifyPath(full_path);

        // 转换为世界坐标
        result.path = convertToPath(map, full_path);
    } else {
        // 单一凸区域，直接螺旋
        result.path = performSpiralPlanning(inflated_map_, start_grid, boundary);
    }

    if (result.path.empty()) {
        if (enable_fallback_) {
            return fallbackToZigzag(map, start_pose, config);
        }
        result.error_message = "Failed to generate spiral path";
        return result;
    }

    // 计算统计数据
    CoverageStats stats = CoverageCalculator::calculateCoverage(
        map, result.path, robot_radius_);

    result.coverage_rate = stats.coverage_rate;
    result.path_length = stats.path_length;
    result.turn_count = stats.turn_count;
    result.success = true;

    // === P0优化：转弯优化 ===
    if (enable_turn_optimization_ && result.path.size() > 3) {
        // 配置TurnOptimizer
        TurnOptimizerConfig turn_config;
        turn_config.angle_threshold = turn_angle_threshold_;
        turn_config.merge_distance_threshold = turn_merge_distance_;
        turn_config.enable_merge = true;
        turn_config.enable_smooth = false;

        // 执行转弯优化
        TurnOptimizeResult optimize_result = turn_optimizer_.optimize(
            result.path, inflated_map_, turn_config);

        if (optimize_result.success && optimize_result.optimized_turn_count < result.turn_count) {
            result.path = optimize_result.optimized_path;
            result.turn_count = optimize_result.optimized_turn_count;

            // 重新计算路径长度
            CoverageStats optimized_stats = CoverageCalculator::calculateCoverage(
                map, result.path, robot_radius_);
            result.path_length = optimized_stats.path_length;
        }
    }

    return result;
}

RegionBoundary SpiralPlanner::getRegionBoundary(const nav_msgs::msg::OccupancyGrid & map)
{
    RegionBoundary boundary;
    boundary.x_min = map.info.width;
    boundary.x_max = 0;
    boundary.y_min = map.info.height;
    boundary.y_max = 0;

    bool found_free = false;

    for (size_t y = 0; y < map.info.height; ++y) {
        for (size_t x = 0; x < map.info.width; ++x) {
            if (MapUtils::isFree(map, static_cast<int>(x), static_cast<int>(y))) {
                found_free = true;
                boundary.x_min = std::min(boundary.x_min, static_cast<int>(x));
                boundary.x_max = std::max(boundary.x_max, static_cast<int>(x));
                boundary.y_min = std::min(boundary.y_min, static_cast<int>(y));
                boundary.y_max = std::max(boundary.y_max, static_cast<int>(y));
            }
        }
    }

    if (!found_free) {
        return RegionBoundary(0, 0, 0, 0);
    }

    return boundary;
}

bool SpiralPlanner::isConvexRegion(
    const nav_msgs::msg::OccupancyGrid & map,
    const RegionBoundary & region)
{
    // 简化的凸性检查：检查每行每列是否有连续的自由空间
    for (int y = region.y_min; y <= region.y_max; ++y) {
        int free_count = 0;
        bool in_free = false;

        for (int x = region.x_min; x <= region.x_max; ++x) {
            if (MapUtils::isFree(map, x, y)) {
                if (!in_free) {
                    in_free = true;
                    free_count++;
                }
            } else {
                in_free = false;
            }
        }

        // 如果一行有多个自由区域分段，说明可能不是凸区域
        if (free_count > 1) {
            return false;
        }
    }

    return true;
}

std::vector<RegionBoundary> SpiralPlanner::decomposeRegion(
    const nav_msgs::msg::OccupancyGrid & map)
{
    std::vector<RegionBoundary> regions;

    // 使用简单的行扫描分解方法
    std::vector<bool> row_visited(map.info.height, false);

    for (size_t y = 0; y < map.info.height; ++y) {
        if (row_visited[y]) {
            continue;
        }

        // 找到该行的自由区域
        int x_start = -1;
        int x_end = -1;

        for (size_t x = 0; x < map.info.width; ++x) {
            if (MapUtils::isFree(map, static_cast<int>(x), static_cast<int>(y))) {
                if (x_start == -1) {
                    x_start = static_cast<int>(x);
                }
                x_end = static_cast<int>(x);
            }
        }

        if (x_start == -1) {
            row_visited[y] = true;
            continue;
        }

        // 向下扩展区域
        int y_start = static_cast<int>(y);
        int y_end = static_cast<int>(y);

        for (size_t ny = y + 1; ny < map.info.height; ++ny) {
            bool row_continuous = true;

            for (int x = x_start; x <= x_end; ++x) {
                if (!MapUtils::isFree(map, x, static_cast<int>(ny))) {
                    row_continuous = false;
                    break;
                }
            }

            if (row_continuous) {
                y_end = static_cast<int>(ny);
                row_visited[ny] = true;
            } else {
                break;
            }
        }

        regions.push_back(RegionBoundary(x_start, x_end, y_start, y_end));
        row_visited[y] = true;
    }

    return regions;
}

std::vector<geometry_msgs::msg::PoseStamped> SpiralPlanner::performSpiralPlanning(
    const nav_msgs::msg::OccupancyGrid & map,
    const Point2D & start_grid,
    const RegionBoundary & region)
{
    // 生成螺旋路径
    std::vector<Point2D> spiral_path = generateSpiralPath(map, region, start_grid);

    // 填充未被覆盖的区域
    spiral_path = fillUncoveredAreas(map, spiral_path);

    // 简化路径
    spiral_path = simplifyPath(spiral_path);

    // 转换为世界坐标
    return convertToPath(map, spiral_path);
}

std::vector<Point2D> SpiralPlanner::generateSpiralPath(
    const nav_msgs::msg::OccupancyGrid & map,
    const RegionBoundary & region,
    const Point2D & start)
{
    std::vector<Point2D> spiral_path;

    if (region.width() <= 0 || region.height() <= 0) {
        return spiral_path;
    }

    // 初始化覆盖掩码
    coverage_mask_.resize(map.info.height, std::vector<bool>(map.info.width, false));

    // 从边界开始螺旋
    RegionBoundary current_boundary = region;
    int direction = 0;  // 0:右, 1:下, 2:左, 3:上

    // 螺旋移动的方向
    const int dx[] = {1, 0, -1, 0};
    const int dy[] = {0, 1, 0, -1};

    Point2D current_pos;

    // 找到距离起始点最近的边界点
    double min_dist = std::numeric_limits<double>::max();

    // 检查四个边上的点
    for (int x = current_boundary.x_min; x <= current_boundary.x_max; ++x) {
        Point2D top(x, current_boundary.y_min);
        Point2D bottom(x, current_boundary.y_max);

        if (MapUtils::isFree(map, top.x, top.y)) {
            double dist = start.distanceTo(top);
            if (dist < min_dist) {
                min_dist = dist;
                current_pos = top;
                direction = (spiral_direction_ == SpiralDirection::CLOCKWISE) ? 1 : 3;
            }
        }

        if (MapUtils::isFree(map, bottom.x, bottom.y)) {
            double dist = start.distanceTo(bottom);
            if (dist < min_dist) {
                min_dist = dist;
                current_pos = bottom;
                direction = (spiral_direction_ == SpiralDirection::CLOCKWISE) ? 3 : 1;
            }
        }
    }

    for (int y = current_boundary.y_min; y <= current_boundary.y_max; ++y) {
        Point2D left(current_boundary.x_min, y);
        Point2D right(current_boundary.x_max, y);

        if (MapUtils::isFree(map, left.x, left.y)) {
            double dist = start.distanceTo(left);
            if (dist < min_dist) {
                min_dist = dist;
                current_pos = left;
                direction = (spiral_direction_ == SpiralDirection::CLOCKWISE) ? 0 : 2;
            }
        }

        if (MapUtils::isFree(map, right.x, right.y)) {
            double dist = start.distanceTo(right);
            if (dist < min_dist) {
                min_dist = dist;
                current_pos = right;
                direction = (spiral_direction_ == SpiralDirection::CLOCKWISE) ? 2 : 0;
            }
        }
    }

    // 如果没有找到边界点，使用起始点
    if (min_dist == std::numeric_limits<double>::max()) {
        current_pos = start;
        direction = 0;
    }

    // 从起始点到边界点的路径
    std::vector<Point2D> connection = findPath(map, start, current_pos);
    spiral_path.insert(spiral_path.end(), connection.begin(), connection.end());

    // 开始螺旋移动
    int steps_in_direction = 0;
    int max_steps = (spiral_direction_ == SpiralDirection::CLOCKWISE) ?
        current_boundary.width() : current_boundary.height();

    int spiral_iterations = 0;
    int max_iterations = std::max(region.width(), region.height()) * 4;

    while (spiral_iterations < max_iterations &&
           isValidBoundary(map, current_boundary)) {

        // 沿当前方向移动
        while (steps_in_direction < max_steps) {
            int nx = current_pos.x + dx[direction];
            int ny = current_pos.y + dy[direction];

            // 检查是否在边界内且为自由空间
            if (!current_boundary.contains(Point2D(nx, ny)) ||
                !MapUtils::isFree(map, nx, ny)) {
                break;
            }

            // 移动到下一个点
            current_pos = Point2D(nx, ny);
            spiral_path.push_back(current_pos);
            coverage_mask_[ny][nx] = true;
            steps_in_direction++;
        }

        // 转向（顺时针或逆时针）
        if (spiral_direction_ == SpiralDirection::CLOCKWISE) {
            direction = (direction + 1) % 4;
        } else {
            direction = (direction + 3) % 4;
        }

        // 根据新方向调整步数
        if (direction == 0 || direction == 2) {
            max_steps = current_boundary.width();
        } else {
            max_steps = current_boundary.height();
        }

        steps_in_direction = 0;

        // 缩小边界
        current_boundary = shrinkBoundary(current_boundary, 1);

        spiral_iterations++;
    }

    return spiral_path;
}

RegionBoundary SpiralPlanner::shrinkBoundary(
    const RegionBoundary & boundary,
    int steps)
{
    RegionBoundary shrunk;
    shrunk.x_min = boundary.x_min + steps;
    shrunk.x_max = boundary.x_max - steps;
    shrunk.y_min = boundary.y_min + steps;
    shrunk.y_max = boundary.y_max - steps;

    return shrunk;
}

bool SpiralPlanner::isValidBoundary(
    const nav_msgs::msg::OccupancyGrid & map,
    const RegionBoundary & boundary)
{
    if (boundary.x_min >= boundary.x_max || boundary.y_min >= boundary.y_max) {
        return false;
    }

    // 检查边界上是否还有自由空间
    for (int x = boundary.x_min; x <= boundary.x_max; ++x) {
        if (MapUtils::isFree(map, x, boundary.y_min) ||
            MapUtils::isFree(map, x, boundary.y_max)) {
            return true;
        }
    }

    for (int y = boundary.y_min; y <= boundary.y_max; ++y) {
        if (MapUtils::isFree(map, boundary.x_min, y) ||
            MapUtils::isFree(map, boundary.x_max, y)) {
            return true;
        }
    }

    return false;
}

std::vector<Point2D> SpiralPlanner::fillUncoveredAreas(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<Point2D> & spiral_path)
{
    std::vector<Point2D> full_path = spiral_path;

    // 查找未被覆盖的自由区域
    std::vector<Point2D> uncovered;

    for (size_t y = 0; y < map.info.height; ++y) {
        for (size_t x = 0; x < map.info.width; ++x) {
            if (MapUtils::isFree(map, static_cast<int>(x), static_cast<int>(y)) &&
                !coverage_mask_[y][x]) {
                uncovered.push_back(Point2D(static_cast<int>(x), static_cast<int>(y)));
            }
        }
    }

    if (uncovered.empty()) {
        return full_path;
    }

    // 使用弓字形方法填充未被覆盖的区域
    if (!full_path.empty()) {
        Point2D last_point = full_path.back();

        // 找到最近的未覆盖点
        double min_dist = std::numeric_limits<double>::max();
        Point2D nearest_uncovered;

        for (const auto & point : uncovered) {
            double dist = last_point.distanceTo(point);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_uncovered = point;
            }
        }

        // 连接到最近的未覆盖点
        std::vector<Point2D> connection = findPath(map, last_point, nearest_uncovered);
        full_path.insert(full_path.end(), connection.begin(), connection.end());

        // 对未覆盖区域使用简单的弓字形填充
        // 按Y坐标排序
        std::sort(uncovered.begin(), uncovered.end(),
            [](const Point2D & a, const Point2D & b) { return a.y < b.y; });

        bool forward = true;
        int current_y = -1;

        for (const auto & point : uncovered) {
            if (current_y != point.y) {
                current_y = point.y;
                forward = !forward;
            }

            if (!coverage_mask_[point.y][point.x]) {
                // 从当前点到下一个点的路径
                if (!full_path.empty() && full_path.back() != point) {
                    std::vector<Point2D> segment = findPath(map, full_path.back(), point);
                    full_path.insert(full_path.end(), segment.begin(), segment.end());
                }

                full_path.push_back(point);
                coverage_mask_[point.y][point.x] = true;
            }
        }
    }

    return full_path;
}

std::vector<Point2D> SpiralPlanner::connectSpiralRegions(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<std::vector<Point2D>> & spiral_paths,
    const Point2D & start)
{
    std::vector<Point2D> full_path;

    if (spiral_paths.empty()) {
        return full_path;
    }

    // 从起始点开始连接
    Point2D current_pos = start;

    for (size_t i = 0; i < spiral_paths.size(); ++i) {
        const auto & spiral = spiral_paths[i];

        if (spiral.empty()) {
            continue;
        }

        // 找到最近的螺旋起点
        double min_dist = std::numeric_limits<double>::max();
        size_t nearest_idx = 0;

        for (size_t j = 0; j < spiral.size(); ++j) {
            double dist = current_pos.distanceTo(spiral[j]);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = j;
            }
        }

        // 连接到螺旋起点
        if (current_pos != spiral[nearest_idx]) {
            std::vector<Point2D> connection = findPath(map, current_pos, spiral[nearest_idx]);
            full_path.insert(full_path.end(), connection.begin(), connection.end());
        }

        // 添加螺旋路径
        full_path.insert(full_path.end(), spiral.begin(), spiral.end());

        // 更新当前位置
        if (!full_path.empty()) {
            current_pos = full_path.back();
        }
    }

    return full_path;
}

std::vector<Point2D> SpiralPlanner::findPath(
    const nav_msgs::msg::OccupancyGrid & map,
    const Point2D & start,
    const Point2D & goal)
{
    if (start == goal) {
        return {start};
    }

    // 使用BFS
    std::vector<std::vector<bool>> visited(
        map.info.height, std::vector<bool>(map.info.width, false));
    std::vector<std::vector<Point2D>> parent(
        map.info.height, std::vector<Point2D>(map.info.width, Point2D(-1, -1)));

    std::queue<Point2D> queue;
    queue.push(start);
    visited[start.y][start.x] = true;

    const int dx[] = {0, 0, 1, -1};
    const int dy[] = {1, -1, 0, 0};

    bool found = false;

    while (!queue.empty() && !found) {
        Point2D current = queue.front();
        queue.pop();

        for (int i = 0; i < 4; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (nx == goal.x && ny == goal.y) {
                parent[ny][nx] = current;
                found = true;
                break;
            }

            if (MapUtils::isInBounds(map, nx, ny) &&
                !visited[ny][nx] &&
                MapUtils::isFree(map, nx, ny)) {
                visited[ny][nx] = true;
                parent[ny][nx] = current;
                queue.push(Point2D(nx, ny));
            }
        }
    }

    std::vector<Point2D> path;
    if (found) {
        Point2D current = goal;
        while (current.x != -1 && current.y != -1) {
            path.push_back(current);
            current = parent[current.y][current.x];
        }
        std::reverse(path.begin(), path.end());
    } else {
        path = {start, goal};  // 如果找不到路径，直接连接
    }

    return path;
}

std::vector<Point2D> SpiralPlanner::simplifyPath(const std::vector<Point2D> & path)
{
    if (path.size() < 3) {
        return path;
    }

    std::vector<Point2D> simplified;
    simplified.push_back(path.front());

    for (size_t i = 1; i < path.size() - 1; ++i) {
        int dx1 = path[i].x - path[i - 1].x;
        int dy1 = path[i].y - path[i - 1].y;
        int dx2 = path[i + 1].x - path[i].x;
        int dy2 = path[i + 1].y - path[i].y;

        if (dx1 != dx2 || dy1 != dy2) {
            simplified.push_back(path[i]);
        }
    }

    simplified.push_back(path.back());
    return simplified;
}

std::vector<geometry_msgs::msg::PoseStamped> SpiralPlanner::convertToPath(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<Point2D> & grid_points)
{
    std::vector<geometry_msgs::msg::PoseStamped> path;

    if (grid_points.empty()) {
        return path;
    }

    std::string frame_id = map.header.frame_id;

    for (size_t i = 0; i < grid_points.size(); ++i) {
        double world_x, world_y;
        MapUtils::gridToWorld(map, grid_points[i].x, grid_points[i].y, world_x, world_y);

        double yaw = 0.0;
        if (i < grid_points.size() - 1) {
            int dx = grid_points[i + 1].x - grid_points[i].x;
            int dy = grid_points[i + 1].y - grid_points[i].y;
            yaw = std::atan2(dy, dx);
        } else if (i > 0) {
            int dx = grid_points[i].x - grid_points[i - 1].x;
            int dy = grid_points[i].y - grid_points[i - 1].y;
            yaw = std::atan2(dy, dx);
        }

        path.push_back(PathUtils::createPoseStamped(world_x, world_y, yaw, frame_id));
    }

    return path;
}

PlannerResult SpiralPlanner::fallbackToZigzag(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::Pose & start_pose,
    const PlannerConfig & config)
{
    ZigzagPlanner zigzag;
    PlannerResult result = zigzag.plan(map, start_pose, config);

    if (!result.success) {
        result.error_message = "Spiral planner failed and zigzag fallback also failed: " +
            result.error_message;
    }

    return result;
}

}  // namespace coverage_planner