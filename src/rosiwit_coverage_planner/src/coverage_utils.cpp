// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include "coverage_planner/coverage_utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <limits>

namespace coverage_planner
{

// ==================== MapUtils实现 ====================

bool MapUtils::isInBounds(const nav_msgs::msg::OccupancyGrid & map, int x, int y)
{
    return x >= 0 && x < static_cast<int>(map.info.width) &&
           y >= 0 && y < static_cast<int>(map.info.height);
}

bool MapUtils::isObstacle(const nav_msgs::msg::OccupancyGrid & map, int x, int y, int threshold)
{
    if (!isInBounds(map, x, y)) {
        return true;  // 边界外视为障碍物
    }
    int index = y * map.info.width + x;
    return map.data[index] >= threshold || map.data[index] < 0;
}

bool MapUtils::isFree(const nav_msgs::msg::OccupancyGrid & map, int x, int y, int threshold)
{
    if (!isInBounds(map, x, y)) {
        return false;
    }
    int index = y * map.info.width + x;
    return map.data[index] >= 0 && map.data[index] < threshold;
}

nav_msgs::msg::OccupancyGrid MapUtils::inflateMap(
    const nav_msgs::msg::OccupancyGrid & map,
    double robot_radius)
{
    nav_msgs::msg::OccupancyGrid inflated = map;

    int inflation_cells = static_cast<int>(std::ceil(robot_radius / map.info.resolution));
    std::vector<int> obstacle_indices;

    // 收集所有障碍物位置
    for (size_t i = 0; i < map.data.size(); ++i) {
        if (map.data[i] >= 50 || map.data[i] < 0) {
            obstacle_indices.push_back(static_cast<int>(i));
        }
    }

    // 膨胀障碍物
    for (int idx : obstacle_indices) {
        int ox = idx % map.info.width;
        int oy = idx / map.info.width;

        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                int nx = ox + dx;
                int ny = oy + dy;

                if (isInBounds(map, nx, ny)) {
                    double dist = std::sqrt(dx * dx + dy * dy) * map.info.resolution;
                    if (dist <= robot_radius) {
                        int nidx = ny * map.info.width + nx;
                        inflated.data[nidx] = 100;  // 标记为障碍物
                    }
                }
            }
        }
    }

    return inflated;
}

std::vector<Point2D> MapUtils::getReachableCells(
    const nav_msgs::msg::OccupancyGrid & map,
    const Point2D & start)
{
    std::vector<Point2D> reachable;

    if (!isInBounds(map, start.x, start.y) || isObstacle(map, start.x, start.y)) {
        return reachable;
    }

    std::vector<std::vector<bool>> visited(
        map.info.height, std::vector<bool>(map.info.width, false));

    std::queue<Point2D> queue;
    queue.push(start);
    visited[start.y][start.x] = true;

    const int dx[] = {0, 0, 1, -1};
    const int dy[] = {1, -1, 0, 0};

    while (!queue.empty()) {
        Point2D current = queue.front();
        queue.pop();
        reachable.push_back(current);

        for (int i = 0; i < 4; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (isInBounds(map, nx, ny) &&
                !visited[ny][nx] &&
                isFree(map, nx, ny)) {
                visited[ny][nx] = true;
                queue.push(Point2D(nx, ny));
            }
        }
    }

    return reachable;
}

Point2D MapUtils::worldToGrid(
    const nav_msgs::msg::OccupancyGrid & map,
    double world_x, double world_y)
{
    int grid_x = static_cast<int>((world_x - map.info.origin.position.x) / map.info.resolution);
    int grid_y = static_cast<int>((world_y - map.info.origin.position.y) / map.info.resolution);
    return Point2D(grid_x, grid_y);
}

void MapUtils::gridToWorld(
    const nav_msgs::msg::OccupancyGrid & map,
    int grid_x, int grid_y,
    double & world_x, double & world_y)
{
    world_x = map.info.origin.position.x + (grid_x + 0.5) * map.info.resolution;
    world_y = map.info.origin.position.y + (grid_y + 0.5) * map.info.resolution;
}

std::vector<Point2D> MapUtils::getFreeCells(const nav_msgs::msg::OccupancyGrid & map)
{
    std::vector<Point2D> free_cells;

    for (size_t y = 0; y < map.info.height; ++y) {
        for (size_t x = 0; x < map.info.width; ++x) {
            if (isFree(map, static_cast<int>(x), static_cast<int>(y))) {
                free_cells.push_back(Point2D(static_cast<int>(x), static_cast<int>(y)));
            }
        }
    }

    return free_cells;
}

int MapUtils::getOptimalScanDirection(const nav_msgs::msg::OccupancyGrid & map)
{
    // 计算水平方向的扫描线数量
    int horizontal_scans = 0;
    for (size_t y = 0; y < map.info.height; ++y) {
        bool in_free_space = false;
        for (size_t x = 0; x < map.info.width; ++x) {
            if (isFree(map, static_cast<int>(x), static_cast<int>(y))) {
                if (!in_free_space) {
                    horizontal_scans++;
                    in_free_space = true;
                }
            } else {
                in_free_space = false;
            }
        }
    }

    // 计算垂直方向的扫描线数量
    int vertical_scans = 0;
    for (size_t x = 0; x < map.info.width; ++x) {
        bool in_free_space = false;
        for (size_t y = 0; y < map.info.height; ++y) {
            if (isFree(map, static_cast<int>(x), static_cast<int>(y))) {
                if (!in_free_space) {
                    vertical_scans++;
                    in_free_space = true;
                }
            } else {
                in_free_space = false;
            }
        }
    }

    // 返回扫描线较少的方向（转弯次数少）
    return (horizontal_scans <= vertical_scans) ? 0 : 1;  // 0: 水平, 1: 垂直
}

// ==================== PathUtils实现 ====================

double PathUtils::calculatePathLength(const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
    if (path.size() < 2) {
        return 0.0;
    }

    double total_length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        total_length += distanceBetween(path[i - 1], path[i]);
    }

    return total_length;
}

int PathUtils::calculateTurnCount(const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
    if (path.size() < 3) {
        return 0;
    }

    int turn_count = 0;
    const double turn_threshold = M_PI / 6;  // 30度视为转弯

    for (size_t i = 1; i < path.size() - 1; ++i) {
        double angle = angleBetween(path[i - 1], path[i + 1]);
        if (angle > turn_threshold) {
            turn_count++;
        }
    }

    return turn_count;
}

std::vector<geometry_msgs::msg::PoseStamped> PathUtils::smoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    double resolution)
{
    if (path.size() < 3) {
        return path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> smoothed;
    smoothed.push_back(path.front());

    for (size_t i = 1; i < path.size() - 1; ++i) {
        double dist_prev = distanceBetween(path[i - 1], path[i]);
        double dist_next = distanceBetween(path[i], path[i + 1]);

        // 如果点间距太小，跳过
        if (dist_prev < resolution * 0.5 && dist_next < resolution * 0.5) {
            continue;
        }

        smoothed.push_back(path[i]);
    }

    smoothed.push_back(path.back());
    return smoothed;
}

std::vector<geometry_msgs::msg::PoseStamped> PathUtils::interpolatePath(
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    double max_distance)
{
    if (path.size() < 2) {
        return path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> interpolated;
    interpolated.push_back(path.front());

    for (size_t i = 1; i < path.size(); ++i) {
        double dist = distanceBetween(path[i - 1], path[i]);

        if (dist > max_distance) {
            int num_points = static_cast<int>(std::ceil(dist / max_distance)) - 1;

            for (int j = 1; j <= num_points; ++j) {
                double t = static_cast<double>(j) / (num_points + 1);
                geometry_msgs::msg::PoseStamped interp_pose;
                interp_pose.header = path[i].header;
                interp_pose.pose.position.x = path[i - 1].pose.position.x +
                    t * (path[i].pose.position.x - path[i - 1].pose.position.x);
                interp_pose.pose.position.y = path[i - 1].pose.position.y +
                    t * (path[i].pose.position.y - path[i - 1].pose.position.y);
                interp_pose.pose.position.z = 0.0;

                // 计算朝向
                double yaw = std::atan2(
                    path[i].pose.position.y - path[i - 1].pose.position.y,
                    path[i].pose.position.x - path[i - 1].pose.position.x);

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                // 使用 tf2_geometry_msgs 的 toMsg 函数
                interp_pose.pose.orientation = tf2::toMsg(q);

                interpolated.push_back(interp_pose);
            }
        }

        interpolated.push_back(path[i]);
    }

    return interpolated;
}

double PathUtils::distanceBetween(
    const geometry_msgs::msg::PoseStamped & pose1,
    const geometry_msgs::msg::PoseStamped & pose2)
{
    double dx = pose2.pose.position.x - pose1.pose.position.x;
    double dy = pose2.pose.position.y - pose1.pose.position.y;
    double dz = pose2.pose.position.z - pose1.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double PathUtils::angleBetween(
    const geometry_msgs::msg::PoseStamped & pose1,
    const geometry_msgs::msg::PoseStamped & pose2)
{
    double dx = pose2.pose.position.x - pose1.pose.position.x;
    double dy = pose2.pose.position.y - pose1.pose.position.y;
    return std::atan2(dy, dx);
}

geometry_msgs::msg::PoseStamped PathUtils::createPoseStamped(
    double x, double y, double yaw,
    const std::string & frame_id)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp.sec = 0;
    pose.header.stamp.nanosec = 0;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);

    return pose;
}

// ==================== CoverageCalculator实现 ====================

CoverageStats CoverageCalculator::calculateCoverage(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    double robot_radius)
{
    CoverageStats stats;

    // VULN-001修复：防止整数溢出（Critical安全漏洞）
    // 检查地图尺寸是否超过安全限制（46340x46340是32位int的最大安全值）
    if (map.info.width > 46340 || map.info.height > 46340) {
        RCLCPP_ERROR(rclcpp::get_logger("coverage_utils"),
            "Map size (%u x %u) exceeds safe limit (max 46340x46340), rejecting to prevent integer overflow",
            map.info.width, map.info.height);
        stats.coverage_rate = 0.0;
        return stats;  // 返回空统计，拒绝处理超大地图
    }

    // 使用size_t避免溢出
    stats.total_cells = static_cast<size_t>(map.info.width) * static_cast<size_t>(map.info.height);
    stats.free_cells = 0;
    stats.covered_cells = 0;
    stats.coverage_rate = 0.0;
    stats.path_length = PathUtils::calculatePathLength(path);
    stats.turn_count = PathUtils::calculateTurnCount(path);

    // 计算空闲栅格数
    for (size_t i = 0; i < map.data.size(); ++i) {
        if (map.data[i] >= 0 && map.data[i] < 50) {
            stats.free_cells++;
        }
    }

    // 计算覆盖栅格
    std::vector<std::vector<bool>> coverage_mask = markCoverage(map, path, robot_radius);

    for (size_t y = 0; y < map.info.height; ++y) {
        for (size_t x = 0; x < map.info.width; ++x) {
            if (coverage_mask[y][x]) {
                stats.covered_cells++;
            }
        }
    }

    // 计算覆盖率
    if (stats.free_cells > 0) {
        stats.coverage_rate = static_cast<double>(stats.covered_cells) / stats.free_cells;
    }

    return stats;
}

std::vector<std::vector<bool>> CoverageCalculator::markCoverage(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    double robot_radius)
{
    std::vector<std::vector<bool>> coverage_mask(
        map.info.height, std::vector<bool>(map.info.width, false));

    int inflation_cells = static_cast<int>(std::ceil(robot_radius / map.info.resolution));

    for (const auto & pose : path) {
        Point2D grid_pos = MapUtils::worldToGrid(
            map, pose.pose.position.x, pose.pose.position.y);

        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                int nx = grid_pos.x + dx;
                int ny = grid_pos.y + dy;

                if (MapUtils::isInBounds(map, nx, ny)) {
                    double dist = std::sqrt(dx * dx + dy * dy) * map.info.resolution;
                    if (dist <= robot_radius) {
                        coverage_mask[ny][nx] = true;
                    }
                }
            }
        }
    }

    return coverage_mask;
}

double CoverageCalculator::calculateCoverageRate(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<std::vector<bool>> & coverage_mask)
{
    int free_cells = 0;
    int covered_cells = 0;

    for (size_t y = 0; y < map.info.height; ++y) {
        for (size_t x = 0; x < map.info.width; ++x) {
            if (MapUtils::isFree(map, static_cast<int>(x), static_cast<int>(y))) {
                free_cells++;
                if (coverage_mask[y][x]) {
                    covered_cells++;
                }
            }
        }
    }

    if (free_cells == 0) {
        return 0.0;
    }

    return static_cast<double>(covered_cells) / free_cells;
}

}  // namespace coverage_planner