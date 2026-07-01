// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include "coverage_planner/zone_decomposer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <queue>
#include <limits>
#include <chrono>
#include <cmath>

namespace coverage_planner
{

ZoneDecomposer::ZoneDecomposer()
{
}

void ZoneDecomposer::reset()
{
    connected_components_img_ = cv::Mat();
    visited_mask_.clear();
}

DecompositionResult ZoneDecomposer::decompose(
    const nav_msgs::msg::OccupancyGrid & map,
    const ZoneDecomposerConfig & config)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    DecompositionResult result;
    result.success = false;
    result.total_free_cells = 0;

    // 检查地图有效性
    if (map.data.empty() || map.info.width == 0 || map.info.height == 0) {
        result.error_message = "Invalid map: empty or zero dimensions";
        return result;
    }

    // Step 1: 连通域分析
    std::vector<Zone> zones = findConnectedComponents(map, config.min_zone_area);

    if (zones.empty()) {
        result.error_message = "No valid zones found in map";
        return result;
    }

    // VULN-004修复：Zone数量上限检查，防止资源耗尽DoS（Critical安全漏洞）
    if (static_cast<int>(zones.size()) > config.max_zone_count) {
        RCLCPP_WARN(rclcpp::get_logger("zone_decomposer"),
            "Zone count (%zu) exceeds max limit (%d), truncating to prevent DoS",
            zones.size(), config.max_zone_count);
        // 截断zones到最大数量，保留最大的区域
        std::sort(zones.begin(), zones.end(),
            [](const Zone & a, const Zone & b) { return a.area > b.area; });
        zones.resize(config.max_zone_count);
    }

    // Step 2: 计算每个区域的最优扫描方向
    for (auto & zone : zones) {
        zone.optimal_scan_direction = computeOptimalScanDirection(zone, map);
    }

    // Step 3: 查找区域连接通道
    std::vector<ConnectionChannel> channels = findConnectionChannels(
        zones, map, config.connection_search_radius);

    // Step 4: 计算访问顺序
    // 默认从第一个区域的质心开始
    cv::Point start_point = zones.empty() ? cv::Point(0, 0) : zones[0].centroid;
    std::vector<int> visit_order = computeVisitOrder(zones, channels, start_point);

    // 构建结果
    result.success = true;
    result.zones = std::move(zones);
    result.channels = std::move(channels);
    result.visit_order = std::move(visit_order);

    // 计算总空闲栅格数
    for (const auto & zone : result.zones) {
        result.total_free_cells += zone.area;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.decomposition_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    return result;
}

std::vector<Zone> ZoneDecomposer::findConnectedComponents(
    const nav_msgs::msg::OccupancyGrid & map,
    int min_area)
{
    std::vector<Zone> zones;

    const int width = map.info.width;
    const int height = map.info.height;

    // 初始化标签矩阵
    std::vector<std::vector<int>> labels(height, std::vector<int>(width, -1));

    // 创建可视化图像
    connected_components_img_ = cv::Mat::zeros(height, width, CV_8UC3);

    int current_label = 0;

    // BFS遍历查找连通域
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // 只处理未标记的空闲栅格
            if (labels[y][x] == -1 && isFree(map, x, y)) {
                // 泛洪填充
                std::vector<cv::Point> points = floodFill(map, x, y, current_label, labels);

                // 检查区域面积
                if (static_cast<int>(points.size()) >= min_area) {
                    Zone zone;
                    zone.id = current_label;
                    zone.type = ZoneType::RECTANGULAR;
                    zone.free_cells = points;
                    zone.area = static_cast<int>(points.size());
                    zone.contour = points;  // 简化：使用所有点作为轮廓
                    zone.centroid = computeCentroid(points);
                    zone.bounding_box = computeBoundingBox(points);

                    zones.push_back(zone);

                    // 绘制连通域（可视化）
                    cv::Vec3b color(
                        (current_label * 37) % 256,
                        (current_label * 73) % 256,
                        (current_label * 113) % 256);
                    for (const auto & pt : points) {
                        connected_components_img_.at<cv::Vec3b>(pt.y, pt.x) = color;
                    }

                    ++current_label;
                } else {
                    // 小区域重新标记为-1（不作为独立区域）
                    for (const auto & pt : points) {
                        labels[pt.y][pt.x] = -1;
                    }
                }
            }
        }
    }

    return zones;
}

std::vector<cv::Point> ZoneDecomposer::floodFill(
    const nav_msgs::msg::OccupancyGrid & map,
    int start_x,
    int start_y,
    int label,
    std::vector<std::vector<int>> & labels)
{
    std::vector<cv::Point> points;
    std::queue<cv::Point> queue;

    queue.push(cv::Point(start_x, start_y));
    labels[start_y][start_x] = label;

    // 4邻域方向
    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {-1, 0, 1, 0};

    while (!queue.empty()) {
        cv::Point current = queue.front();
        queue.pop();
        points.push_back(current);

        // 遍历4邻域
        for (int i = 0; i < 4; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            // 检查边界和空闲状态
            if (isInBounds(map, nx, ny) &&
                labels[ny][nx] == -1 &&
                isFree(map, nx, ny)) {
                labels[ny][nx] = label;
                queue.push(cv::Point(nx, ny));
            }
        }
    }

    return points;
}

int ZoneDecomposer::computeOptimalScanDirection(
    const Zone & zone,
    const nav_msgs::msg::OccupancyGrid & map)
{
    if (zone.free_cells.size() < 10) {
        return 0;  // 默认水平扫描
    }

    double principal_angle = 0.0;
    double aspect_ratio = 1.0;

    performPCA(zone.free_cells, principal_angle, aspect_ratio);

    // 根据长宽比和主方向选择扫描方向
    // 主方向接近0度或180度 -> 水平扫描
    // 主方向接近90度 -> 垂直扫描
    // 长宽比 > 阈值时，沿长边扫描

    const double ANGLE_THRESHOLD = M_PI / 4;  // 45度

    if (aspect_ratio > 2.0) {
        // 长宽比大于2，沿长边扫描
        if (std::abs(principal_angle) < ANGLE_THRESHOLD ||
            std::abs(principal_angle - M_PI) < ANGLE_THRESHOLD) {
            return 0;  // 水平扫描（长边水平）
        } else {
            return 1;  // 垂直扫描（长边垂直）
        }
    }

    // 根据主方向选择
    if (std::abs(principal_angle - M_PI / 2) < ANGLE_THRESHOLD) {
        return 1;  // 垂直扫描
    }

    return 0;  // 默认水平扫描
}

void ZoneDecomposer::performPCA(
    const std::vector<cv::Point> & points,
    double & principal_angle,
    double & aspect_ratio)
{
    if (points.size() < 2) {
        principal_angle = 0.0;
        aspect_ratio = 1.0;
        return;
    }

    // 计算质心
    double cx = 0.0, cy = 0.0;
    for (const auto & pt : points) {
        cx += pt.x;
        cy += pt.y;
    }
    cx /= points.size();
    cy /= points.size();

    // 构建协方差矩阵
    double cxx = 0.0, cxy = 0.0, cyy = 0.0;
    for (const auto & pt : points) {
        double dx = pt.x - cx;
        double dy = pt.y - cy;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }
    cxx /= points.size();
    cxy /= points.size();
    cyy /= points.size();

    // 计算特征值
    double trace = cxx + cyy;
    double det = cxx * cyy - cxy * cxy;
    double discriminant = std::sqrt(trace * trace / 4.0 - det);

    double eigenvalue1 = trace / 2.0 + discriminant;
    double eigenvalue2 = trace / 2.0 - discriminant;

    // 确保特征值为正
    eigenvalue1 = std::max(eigenvalue1, 1e-6);
    eigenvalue2 = std::max(eigenvalue2, 1e-6);

    // 计算主方向角度
    principal_angle = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);

    // 计算长宽比
    aspect_ratio = std::sqrt(eigenvalue1 / eigenvalue2);
}

std::vector<ConnectionChannel> ZoneDecomposer::findConnectionChannels(
    const std::vector<Zone> & zones,
    const nav_msgs::msg::OccupancyGrid & map,
    int search_radius)
{
    std::vector<ConnectionChannel> channels;

    // 遍历所有区域对
    for (size_t i = 0; i < zones.size(); ++i) {
        for (size_t j = i + 1; j < zones.size(); ++j) {
            // 检查是否相邻
            if (areNeighbors(zones[i], zones[j], map, search_radius)) {
                ConnectionChannel channel;
                channel.zone_a_id = zones[i].id;
                channel.zone_b_id = zones[j].id;

                // 找到最近的点对作为连接点
                double min_dist = std::numeric_limits<double>::max();
                for (const auto & pt_a : zones[i].free_cells) {
                    for (const auto & pt_b : zones[j].free_cells) {
                        double d = distance(pt_a, pt_b);
                        if (d < min_dist) {
                            min_dist = d;
                            channel.point_a = pt_a;
                            channel.point_b = pt_b;
                        }
                    }
                }

                channel.distance = min_dist;
                channel.is_reachable = (min_dist < search_radius * 2);

                channels.push_back(channel);
            }
        }
    }

    return channels;
}

bool ZoneDecomposer::areNeighbors(
    const Zone & zone_a,
    const Zone & zone_b,
    const nav_msgs::msg::OccupancyGrid & map,
    int search_radius)
{
    // 快速检查：外接矩形是否相邻
    cv::Rect expanded_a = zone_a.bounding_box;
    expanded_a.x -= search_radius;
    expanded_a.y -= search_radius;
    expanded_a.width += 2 * search_radius;
    expanded_a.height += 2 * search_radius;

    if ((expanded_a & zone_b.bounding_box).area() == 0) {
        return false;  // 外接矩形不相交，肯定不相邻
    }

    // 更精确的检查：是否存在距离小于阈值的点对
    for (const auto & pt_a : zone_a.free_cells) {
        for (const auto & pt_b : zone_b.free_cells) {
            if (distance(pt_a, pt_b) <= search_radius * 2) {
                return true;
            }
        }
    }

    return false;
}

std::vector<int> ZoneDecomposer::computeVisitOrder(
    const std::vector<Zone> & zones,
    const std::vector<ConnectionChannel> & channels,
    const cv::Point & start_point)
{
    if (zones.empty()) {
        return {};
    }

    std::vector<int> visit_order;
    std::vector<bool> visited(zones.size(), false);

    // 构建邻接表
    std::vector<std::vector<std::pair<int, double>>> adj(zones.size());
    for (const auto & channel : channels) {
        if (channel.is_reachable) {
            adj[channel.zone_a_id].push_back({channel.zone_b_id, channel.distance});
            adj[channel.zone_b_id].push_back({channel.zone_a_id, channel.distance});
        }
    }

    // 贪心算法：每次选择最近的未访问区域
    int current_zone = 0;

    // 找到距离起点最近的区域
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < zones.size(); ++i) {
        double d = distance(start_point, zones[i].centroid);
        if (d < min_dist) {
            min_dist = d;
            current_zone = static_cast<int>(i);
        }
    }

    visit_order.push_back(zones[current_zone].id);
    visited[current_zone] = true;

    // 贪心遍历
    while (visit_order.size() < zones.size()) {
        int next_zone = -1;
        double min_next_dist = std::numeric_limits<double>::max();

        // 优先通过连接通道选择
        for (const auto & [neighbor, dist] : adj[current_zone]) {
            if (!visited[neighbor] && dist < min_next_dist) {
                min_next_dist = dist;
                next_zone = neighbor;
            }
        }

        // 如果没有通过连接通道的相邻区域，使用欧氏距离
        if (next_zone == -1) {
            for (size_t i = 0; i < zones.size(); ++i) {
                if (!visited[i]) {
                    double d = distance(zones[current_zone].centroid, zones[i].centroid);
                    if (d < min_next_dist) {
                        min_next_dist = d;
                        next_zone = static_cast<int>(i);
                    }
                }
            }
        }

        if (next_zone == -1) {
            break;  // 没有更多可访问的区域
        }

        visit_order.push_back(zones[next_zone].id);
        visited[next_zone] = true;
        current_zone = next_zone;
    }

    return visit_order;
}

std::vector<cv::Point> ZoneDecomposer::getZonePath(
    const nav_msgs::msg::OccupancyGrid & map,
    const Zone & zone,
    int scan_direction)
{
    std::vector<cv::Point> path;

    if (zone.free_cells.empty()) {
        return path;
    }

    // 根据扫描方向排序点
    std::vector<cv::Point> sorted_points = zone.free_cells;

    if (scan_direction == 0) {
        // 水平扫描：按Y排序，Y相同时按X排序
        std::sort(sorted_points.begin(), sorted_points.end(),
            [](const cv::Point & a, const cv::Point & b) {
                if (a.y != b.y) return a.y < b.y;
                return a.x < b.x;
            });
    } else {
        // 垂直扫描：按X排序，X相同时按Y排序
        std::sort(sorted_points.begin(), sorted_points.end(),
            [](const cv::Point & a, const cv::Point & b) {
                if (a.x != b.x) return a.x < b.x;
                return a.y < b.y;
            });
    }

    // 提取扫描线并生成路径
    // 这里简化处理：直接返回排序后的点
    // 实际应该提取扫描线段并连接
    path = sorted_points;

    return path;
}

cv::Point ZoneDecomposer::computeCentroid(const std::vector<cv::Point> & points) const
{
    if (points.empty()) {
        return cv::Point(0, 0);
    }

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto & pt : points) {
        sum_x += pt.x;
        sum_y += pt.y;
    }

    return cv::Point(
        static_cast<int>(sum_x / points.size()),
        static_cast<int>(sum_y / points.size()));
}

cv::Rect ZoneDecomposer::computeBoundingBox(const std::vector<cv::Point> & points) const
{
    if (points.empty()) {
        return cv::Rect();
    }

    int min_x = points[0].x, max_x = points[0].x;
    int min_y = points[0].y, max_y = points[0].y;

    for (const auto & pt : points) {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }

    return cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
}

}  // namespace coverage_planner