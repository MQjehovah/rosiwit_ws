// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include "coverage_planner/zigzag_planner.hpp"
#include <algorithm>
#include <queue>
#include <cmath>
#include <chrono>
#include <limits>
#include "rclcpp/rclcpp.hpp"

namespace coverage_planner
{

ZigzagPlanner::ZigzagPlanner()
    : robot_radius_(0.3),
      coverage_resolution_(0.1),
      enable_optimization_(true),
      enable_map_preprocessing_(true),
      enable_pca_direction_(true),
      enable_zone_decomposition_(false),  // 默认关闭，通过参数开启
      enable_turn_optimization_(false),   // 默认关闭，通过参数开启
      zone_min_area_(100),
      zone_max_count_(20),
      zone_merge_threshold_(0.2),
      turn_angle_threshold_(0.1),
      turn_merge_distance_(10.0)
{
}

void ZigzagPlanner::reset()
{
    inflated_map_ = nav_msgs::msg::OccupancyGrid();
    preprocessed_map_ = nav_msgs::msg::OccupancyGrid();
    visited_mask_.clear();
    zone_decomposer_.reset();
    turn_optimizer_.reset();
}

PlannerResult ZigzagPlanner::plan(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::Pose & start_pose,
    const PlannerConfig & config)
{
    // 开始计时
    auto start_time = std::chrono::high_resolution_clock::now();

    PlannerResult result;
    result.success = false;
    result.coverage_rate = 0.0;
    result.path_length = 0.0;
    result.turn_count = 0;
    result.planning_time_ms = 0.0;

    // 检查地图有效性
    if (map.data.empty() || map.info.width == 0 || map.info.height == 0) {
        result.error_message = "Invalid map: empty or zero dimensions";
        return result;
    }

    // 设置规划参数
    robot_radius_ = config.robot_radius;
    coverage_resolution_ = config.coverage_resolution;
    enable_optimization_ = config.enable_optimization;
    enable_map_preprocessing_ = config.enable_map_preprocessing;
    enable_pca_direction_ = config.enable_pca_direction;

    // 设置P0优化参数
    enable_zone_decomposition_ = config.enable_zone_decomposition;
    enable_turn_optimization_ = config.enable_turn_optimization;
    zone_min_area_ = config.zone_min_area;
    zone_max_count_ = config.zone_max_count;
    turn_angle_threshold_ = config.turn_angle_threshold;
    turn_merge_distance_ = config.turn_merge_distance;

    // === P0优化：地图预处理 ===
    nav_msgs::msg::OccupancyGrid working_map;
    if (config.enable_map_preprocessing) {
        // 创建预处理配置
        PreprocessConfig preprocess_config;
        preprocess_config.enable_morphology = true;
        preprocess_config.morphology_kernel_size = config.morphology_kernel_size;
        preprocess_config.opening_iterations = config.opening_iterations;
        preprocess_config.closing_iterations = config.closing_iterations;
        preprocess_config.min_obstacle_size = config.min_obstacle_size;
        preprocess_config.max_hole_size = config.max_hole_size;
        preprocess_config.obstacle_merge_distance = config.obstacle_merge_distance;

        // 执行预处理
        working_map = map_preprocessor_.preprocess(map, preprocess_config);
        preprocessed_map_ = working_map;
    } else {
        working_map = map;
    }

    // 膨胀地图
    inflated_map_ = MapUtils::inflateMap(working_map, robot_radius_);

    // 转换起始位置到栅格坐标
    Point2D start_grid = MapUtils::worldToGrid(
        map, start_pose.position.x, start_pose.position.y);

    // 检查起始位置是否有效
    if (!MapUtils::isInBounds(inflated_map_, start_grid.x, start_grid.y) ||
        MapUtils::isObstacle(inflated_map_, start_grid.x, start_grid.y)) {
        result.error_message = "Start position is out of bounds or on an obstacle";
        return result;
    }

    // === P0优化：扫描方向选择 ===
    int scan_direction = config.direction_optimization;
    ScanDirectionResult direction_result;

    // 方向选择逻辑
    if (scan_direction == 3) {  // PCA分析
        ScanDirectionConfig scan_config;
        scan_config.enable_pca = true;
        scan_config.enable_mbr = false;
        scan_config.fallback_to_scanline = config.fallback_to_scanline;
        scan_direction = selectOptimalDirectionOptimized(inflated_map_, scan_config, direction_result);
    } else if (scan_direction == 4) {  // 长边优先（综合PCA + MBR）
        ScanDirectionConfig scan_config;
        scan_config.enable_pca = config.enable_pca_direction;
        scan_config.enable_mbr = config.enable_mbr_analysis;
        scan_config.aspect_ratio_threshold = config.aspect_ratio_threshold;
        scan_config.pca_threshold = config.pca_confidence_threshold;
        scan_config.fallback_to_scanline = config.fallback_to_scanline;
        scan_direction = selectOptimalDirectionOptimized(inflated_map_, scan_config, direction_result);
    } else if (scan_direction == 2) {  // 自动选择（传统方法）
        scan_direction = selectOptimalDirection(inflated_map_);
    }

    // 记录方向选择结果（用于统计）
    result.principal_angle = direction_result.principal_angle;
    result.aspect_ratio = direction_result.aspect_ratio;
    result.direction_method = direction_result.method_used;

    // === P0优化：分区规划 ===
    if (enable_zone_decomposition_) {
        result.path = performZonePlanning(
            inflated_map_, start_pose, config, scan_direction);

        if (!result.path.empty()) {
            // 分区规划成功，跳过传统扫描线规划
            // 计算统计数据
            CoverageStats stats = CoverageCalculator::calculateCoverage(
                working_map, result.path, robot_radius_);

            result.coverage_rate = stats.coverage_rate;
            result.path_length = stats.path_length;
            result.turn_count = stats.turn_count;
            result.success = true;

            // === P0优化：转弯优化 ===
            if (enable_turn_optimization_) {
                auto optimized_path = optimizeTurns(result.path, inflated_map_, config);
                if (!optimized_path.empty()) {
                    // 重新计算统计
                    CoverageStats optimized_stats = CoverageCalculator::calculateCoverage(
                        working_map, optimized_path, robot_radius_);

                    result.path = optimized_path;
                    result.turn_count = optimized_stats.turn_count;
                    result.path_length = optimized_stats.path_length;
                    // 覆盖率保持不变（转弯优化不改变覆盖点）
                }
            }

            // 结束计时
            auto end_time = std::chrono::high_resolution_clock::now();
            result.planning_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

            return result;
        }
    }

    // 传统扫描线规划（无分区规划或分区规划失败）
    result.path = performScanlinePlanning(inflated_map_, start_grid, scan_direction);

    if (result.path.empty()) {
        result.error_message = "Failed to generate coverage path";
        return result;
    }

    // 计算统计数据
    CoverageStats stats = CoverageCalculator::calculateCoverage(
        working_map, result.path, robot_radius_);

    result.coverage_rate = stats.coverage_rate;
    result.path_length = stats.path_length;
    result.turn_count = stats.turn_count;
    result.success = true;

    // 结束计时
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    result.planning_time_ms = static_cast<double>(duration.count());

    // 输出统计信息
    if (config.enable_statistics_output) {
        RCLCPP_INFO(rclcpp::get_logger("zigzag_planner"),
            "Coverage planning completed: %.2f%% coverage, %d turns, %.2f m path, %.1f ms",
            result.coverage_rate * 100.0,
            result.turn_count,
            result.path_length,
            result.planning_time_ms);

        if (config.enable_pca_direction && direction_result.confidence > 0) {
            RCLCPP_INFO(rclcpp::get_logger("zigzag_planner"),
                "Direction optimization: method=%s, confidence=%.2f, aspect_ratio=%.2f",
                direction_result.method_used.c_str(),
                direction_result.confidence,
                direction_result.aspect_ratio);
        }
    }

    return result;
}

int ZigzagPlanner::selectOptimalDirection(const nav_msgs::msg::OccupancyGrid & map)
{
    // 传统方法：使用扫描线数量统计
    return MapUtils::getOptimalScanDirection(map);
}

int ZigzagPlanner::selectOptimalDirectionOptimized(
    const nav_msgs::msg::OccupancyGrid & map,
    const ScanDirectionConfig & config,
    ScanDirectionResult & result_info)
{
    // 使用ScanDirectionOptimizer进行优化方向选择
    ScanDirectionConfig scan_config;
    scan_config.enable_pca = config.enable_pca;
    scan_config.enable_mbr = config.enable_mbr;
    scan_config.aspect_ratio_threshold = config.aspect_ratio_threshold;
    scan_config.pca_threshold = config.pca_threshold;
    scan_config.fallback_to_scanline = config.fallback_to_scanline;

    // 分析最优方向
    result_info = scan_optimizer_.analyzeOptimalDirection(map, scan_config);

    // 如果置信度太低，使用传统方法作为备选
    if (result_info.confidence < 0.5 && config.fallback_to_scanline) {
        int traditional_direction = selectOptimalDirection(map);
        result_info.method_used = "fallback_to_scanline";
        return traditional_direction;
    }

    return result_info.direction;
}

std::vector<ScanLine> ZigzagPlanner::extractScanlines(
    const nav_msgs::msg::OccupancyGrid & map,
    int scan_direction)
{
    std::vector<ScanLine> scanlines;

    if (scan_direction == 0) {  // 水平扫描
        for (size_t y = 0; y < map.info.height; ++y) {
            bool in_segment = false;
            int x_start = 0;

            for (size_t x = 0; x <= map.info.width; ++x) {
                bool is_free = (x < map.info.width) &&
                    MapUtils::isFree(map, static_cast<int>(x), static_cast<int>(y));

                if (is_free && !in_segment) {
                    // 开始新线段
                    in_segment = true;
                    x_start = static_cast<int>(x);
                } else if (!is_free && in_segment) {
                    // 结束线段
                    in_segment = false;
                    scanlines.push_back(ScanLine(
                        static_cast<int>(y), x_start, static_cast<int>(x - 1),
                        scanlines.size() % 2 == 0));
                }
            }

            // 处理最后一个线段
            if (in_segment) {
                scanlines.push_back(ScanLine(
                    static_cast<int>(y), x_start, static_cast<int>(map.info.width - 1),
                    scanlines.size() % 2 == 0));
            }
        }
    } else {  // 垂直扫描
        for (size_t x = 0; x < map.info.width; ++x) {
            bool in_segment = false;
            int y_start = 0;

            for (size_t y = 0; y <= map.info.height; ++y) {
                bool is_free = (y < map.info.height) &&
                    MapUtils::isFree(map, static_cast<int>(x), static_cast<int>(y));

                if (is_free && !in_segment) {
                    // 开始新线段
                    in_segment = true;
                    y_start = static_cast<int>(y);
                } else if (!is_free && in_segment) {
                    // 结束线段
                    in_segment = false;
                    scanlines.push_back(ScanLine(
                        static_cast<int>(x), y_start, static_cast<int>(y - 1),
                        scanlines.size() % 2 == 0));
                }
            }

            // 处理最后一个线段
            if (in_segment) {
                scanlines.push_back(ScanLine(
                    static_cast<int>(x), y_start, static_cast<int>(map.info.height - 1),
                    scanlines.size() % 2 == 0));
            }
        }
    }

    return scanlines;
}

std::vector<Point2D> ZigzagPlanner::connectScanlines(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<ScanLine> & scanlines,
    const Point2D & start)
{
    std::vector<Point2D> path;

    if (scanlines.empty()) {
        return path;
    }

    // 找到离起点最近的扫描线起点
    Point2D current = findNearestScanlineStart(map, scanlines, start);
    path.push_back(current);

    size_t current_scanline_idx = 0;

    // 查找当前扫描线
    for (size_t i = 0; i < scanlines.size(); ++i) {
        const ScanLine & sl = scanlines[i];
        if (sl.y == current.y &&
            (sl.x_start == current.x || sl.x_end == current.x)) {
            current_scanline_idx = i;
            break;
        }
    }

    // 按顺序遍历所有扫描线
    for (size_t i = current_scanline_idx; i < scanlines.size(); ++i) {
        const ScanLine & sl = scanlines[i];

        // 执行当前扫描线
        std::vector<Point2D> scanline_path;
        if (sl.is_forward) {
            for (int x = sl.x_start; x <= sl.x_end; ++x) {
                scanline_path.push_back(Point2D(x, sl.y));
            }
        } else {
            for (int x = sl.x_end; x >= sl.x_start; --x) {
                scanline_path.push_back(Point2D(x, sl.y));
            }
        }

        // 连接当前点到扫描线起点
        Point2D scanline_start = sl.is_forward ?
            Point2D(sl.x_start, sl.y) : Point2D(sl.x_end, sl.y);

        if (current.x != scanline_start.x || current.y != scanline_start.y) {
            std::vector<Point2D> connection = findConnectionPath(map, current, scanline_start);
            if (!connection.empty()) {
                path.insert(path.end(), connection.begin(), connection.end());
            }
        }

        // 添加扫描线路径
        path.insert(path.end(), scanline_path.begin(), scanline_path.end());

        // 更新当前位置
        current = sl.is_forward ?
            Point2D(sl.x_end, sl.y) : Point2D(sl.x_start, sl.y);
    }

    return path;
}

std::vector<Point2D> ZigzagPlanner::findConnectionPath(
    const nav_msgs::msg::OccupancyGrid & map,
    const Point2D & start,
    const Point2D & goal)
{
    std::vector<Point2D> path;

    // BFS寻路
    std::queue<Point2D> queue;
    std::vector<std::vector<bool>> visited(map.info.height, std::vector<bool>(map.info.width, false));
    std::vector<std::vector<Point2D>> parent(map.info.height, std::vector<Point2D>(map.info.width, Point2D(-1, -1)));

    queue.push(start);
    visited[start.y][start.x] = true;

    // 8方向移动
    const int dx[] = {-1, 0, 1, 0, -1, 1, -1, 1};
    const int dy[] = {0, -1, 0, 1, -1, -1, 1, 1};

    bool found = false;

    while (!queue.empty() && !found) {
        Point2D current = queue.front();
        queue.pop();

        for (int i = 0; i < 8; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (MapUtils::isInBounds(map, nx, ny) &&
                !visited[ny][nx] &&
                MapUtils::isFree(map, nx, ny)) {

                visited[ny][nx] = true;
                parent[ny][nx] = current;

                if (nx == goal.x && ny == goal.y) {
                    found = true;
                    break;
                }

                queue.push(Point2D(nx, ny));
            }
        }
    }

    if (!found) {
        // 直接连接（可能穿过障碍物）
        path.push_back(goal);
        return path;
    }

    // 回溯路径
    Point2D current = goal;
    while (current.x != start.x || current.y != start.y) {
        path.push_back(current);
        current = parent[current.y][current.x];
    }

    // 反转路径
    std::reverse(path.begin(), path.end());

    return path;
}

Point2D ZigzagPlanner::findNearestScanlineStart(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<ScanLine> & scanlines,
    const Point2D & start)
{
    Point2D nearest;
    double min_dist = std::numeric_limits<double>::max();

    for (const ScanLine & sl : scanlines) {
        // 检查扫描线起点
        Point2D sl_start(sl.is_forward ? sl.x_start : sl.x_end, sl.y);

        // 检查起点是否可达
        if (MapUtils::isFree(map, sl_start.x, sl_start.y)) {
            double dist = std::sqrt(
                std::pow(sl_start.x - start.x, 2) +
                std::pow(sl_start.y - start.y, 2));

            if (dist < min_dist) {
                min_dist = dist;
                nearest = sl_start;
            }
        }

        // 检查扫描线终点
        Point2D sl_end(sl.is_forward ? sl.x_end : sl.x_start, sl.y);

        if (MapUtils::isFree(map, sl_end.x, sl_end.y)) {
            double dist = std::sqrt(
                std::pow(sl_end.x - start.x, 2) +
                std::pow(sl_end.y - start.y, 2));

            if (dist < min_dist) {
                min_dist = dist;
                nearest = sl_end;
            }
        }
    }

    return nearest;
}

std::vector<Point2D> ZigzagPlanner::simplifyPath(const std::vector<Point2D> & path)
{
    if (path.size() < 3) {
        return path;
    }

    std::vector<Point2D> simplified;
    simplified.push_back(path[0]);

    // 去除同方向的中间点
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Point2D prev = path[i - 1];
        Point2D curr = path[i];
        Point2D next = path[i + 1];

        // 计算方向
        int dx1 = curr.x - prev.x;
        int dy1 = curr.y - prev.y;
        int dx2 = next.x - curr.x;
        int dy2 = next.y - curr.y;

        // 如果方向改变，保留该点
        if (dx1 != dx2 || dy1 != dy2) {
            simplified.push_back(curr);
        }
    }

    simplified.push_back(path.back());

    return simplified;
}

std::vector<geometry_msgs::msg::PoseStamped> ZigzagPlanner::convertToPath(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<Point2D> & grid_path)
{
    std::vector<geometry_msgs::msg::PoseStamped> path;

    // 如果启用优化，先简化路径
    std::vector<Point2D> working_path = enable_optimization_ ?
        simplifyPath(grid_path) : grid_path;

    for (const Point2D & pt : working_path) {
        geometry_msgs::msg::PoseStamped pose;

        // 转换为世界坐标
        double world_x, world_y;
        MapUtils::gridToWorld(map, pt.x, pt.y, world_x, world_y);
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;

        // 设置朝向（根据路径方向）
        if (path.size() > 0) {
            double dx = pose.pose.position.x - path.back().pose.position.x;
            double dy = pose.pose.position.y - path.back().pose.position.y;
            pose.pose.orientation.z = std::sin(std::atan2(dy, dx) / 2.0);
            pose.pose.orientation.w = std::cos(std::atan2(dy, dx) / 2.0);
        } else {
            pose.pose.orientation.w = 1.0;  // 默认朝向
        }

        path.push_back(pose);
    }

    return path;
}

bool ZigzagPlanner::hasLineOfSight(
    const nav_msgs::msg::OccupancyGrid & map,
    const Point2D & p1,
    const Point2D & p2)
{
    // Bresenham直线算法检查视线
    int dx = std::abs(p2.x - p1.x);
    int dy = std::abs(p2.y - p1.y);
    int sx = (p1.x < p2.x) ? 1 : -1;
    int sy = (p1.y < p2.y) ? 1 : -1;
    int err = dx - dy;

    int x = p1.x;
    int y = p1.y;

    while (x != p2.x || y != p2.y) {
        if (!MapUtils::isFree(map, x, y)) {
            return false;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }

    return true;
}

nav_msgs::msg::OccupancyGrid ZigzagPlanner::preprocessMap(
    const nav_msgs::msg::OccupancyGrid & map,
    const PlannerConfig & config)
{
    PreprocessConfig preprocess_config;
    preprocess_config.enable_morphology = true;
    preprocess_config.morphology_kernel_size = config.morphology_kernel_size;
    preprocess_config.opening_iterations = config.opening_iterations;
    preprocess_config.closing_iterations = config.closing_iterations;
    preprocess_config.min_obstacle_size = config.min_obstacle_size;
    preprocess_config.max_hole_size = config.max_hole_size;
    preprocess_config.obstacle_merge_distance = config.obstacle_merge_distance;

    return map_preprocessor_.preprocess(map, preprocess_config);
}

std::vector<geometry_msgs::msg::PoseStamped> ZigzagPlanner::performScanlinePlanning(
    const nav_msgs::msg::OccupancyGrid & map,
    const Point2D & start_grid,
    int scan_direction)
{
    // 提取扫描线
    std::vector<ScanLine> scanlines = extractScanlines(map, scan_direction);

    if (scanlines.empty()) {
        return std::vector<geometry_msgs::msg::PoseStamped>();
    }

    // 连接扫描线形成路径
    std::vector<Point2D> grid_path = connectScanlines(map, scanlines, start_grid);

    // 转换为世界坐标路径
    return convertToPath(map, grid_path);
}

std::vector<geometry_msgs::msg::PoseStamped> ZigzagPlanner::performZonePlanning(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::Pose & start_pose,
    const PlannerConfig & config,
    int scan_direction)
{
    // 配置ZoneDecomposer
    ZoneDecomposerConfig zone_config;
    zone_config.min_zone_area = zone_min_area_;
    zone_config.max_zone_count = zone_max_count_;
    zone_config.enable_rectangular_split = true;
    zone_config.enable_pca_direction = enable_pca_direction_;
    zone_config.merge_threshold = zone_merge_threshold_;

    // 执行分区分解
    DecompositionResult decomp_result = zone_decomposer_.decompose(map, zone_config);

    if (!decomp_result.success || decomp_result.zones.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("zigzag_planner"),
                    "Zone decomposition failed: %s", decomp_result.error_message.c_str());
        return std::vector<geometry_msgs::msg::PoseStamped>();
    }

    RCLCPP_INFO(rclcpp::get_logger("zigzag_planner"),
                "Zone decomposition successful: %d zones, %.1fms",
                static_cast<int>(decomp_result.zones.size()),
                decomp_result.decomposition_time_ms);

    // 转换起点为栅格坐标
    Point2D start_grid = MapUtils::worldToGrid(map, start_pose.position.x, start_pose.position.y);
    cv::Point start_point(start_grid.x, start_grid.y);

    // 按访问顺序规划各区域
    std::vector<geometry_msgs::msg::PoseStamped> full_path;

    for (int zone_id : decomp_result.visit_order) {
        // 找到对应的Zone
        const Zone * current_zone = nullptr;
        for (const auto & zone : decomp_result.zones) {
            if (zone.id == zone_id) {
                current_zone = &zone;
                break;
            }
        }

        if (current_zone == nullptr) {
            continue;
        }

        // 获取该区域的扫描路径
        int zone_scan_dir = current_zone->optimal_scan_direction;
        std::vector<cv::Point> zone_grid_path = zone_decomposer_.getZonePath(
            map, *current_zone, zone_scan_dir);

        // 转换为世界坐标路径
        for (const auto & grid_pt : zone_grid_path) {
            geometry_msgs::msg::PoseStamped pose;

            double world_x, world_y;
            MapUtils::gridToWorld(map, grid_pt.x, grid_pt.y, world_x, world_y);

            pose.pose.position.x = map.info.origin.position.x +
                (grid_pt.x + 0.5) * map.info.resolution;
            pose.pose.position.y = map.info.origin.position.y +
                (grid_pt.y + 0.5) * map.info.resolution;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;

            if (!full_path.empty()) {
                double dx = pose.pose.position.x - full_path.back().pose.position.x;
                double dy = pose.pose.position.y - full_path.back().pose.position.y;
                pose.pose.orientation.z = std::sin(std::atan2(dy, dx) / 2.0);
                pose.pose.orientation.w = std::cos(std::atan2(dy, dx) / 2.0);
            }

            full_path.push_back(pose);
        }
    }

    return full_path;
}

std::vector<geometry_msgs::msg::PoseStamped> ZigzagPlanner::optimizeTurns(
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    const nav_msgs::msg::OccupancyGrid & map,
    const PlannerConfig & config)
{
    if (path.size() < 3) {
        return path;
    }

    // 配置TurnOptimizer
    TurnOptimizerConfig turn_config;
    turn_config.angle_threshold = turn_angle_threshold_;
    turn_config.merge_distance_threshold = turn_merge_distance_;
    turn_config.enable_merge = true;
    turn_config.enable_smooth = false;  // 简化实现，不启用平滑

    // 执行转弯优化
    TurnOptimizeResult optimize_result = turn_optimizer_.optimize(path, map, turn_config);

    if (!optimize_result.success) {
        RCLCPP_WARN(rclcpp::get_logger("zigzag_planner"),
                    "Turn optimization failed");
        return path;
    }

    RCLCPP_INFO(rclcpp::get_logger("zigzag_planner"),
                "Turn optimization: %d -> %d turns (%.1f%% reduction)",
                optimize_result.original_turn_count,
                optimize_result.optimized_turn_count,
                optimize_result.reduction_rate * 100.0);

    return optimize_result.optimized_path;
}

}  // namespace coverage_planner