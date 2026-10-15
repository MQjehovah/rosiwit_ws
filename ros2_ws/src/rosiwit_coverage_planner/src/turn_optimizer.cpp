// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include "coverage_planner/turn_optimizer.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>

namespace coverage_planner
{

TurnOptimizer::TurnOptimizer()
{
}

void TurnOptimizer::reset()
{
    turn_points_.clear();
}

TurnOptimizeResult TurnOptimizer::optimize(
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    const nav_msgs::msg::OccupancyGrid & map,
    const TurnOptimizerConfig & config)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    TurnOptimizeResult result;
    result.success = false;
    
    // 检查路径有效性
    if (path.size() < 3) {
        result.optimized_path = path;
        result.success = true;
        result.original_turn_count = 0;
        result.optimized_turn_count = 0;
        return result;
    }
    
    // Step 1: 检测转弯点
    result.original_turns = detectTurnPoints(path, config.angle_threshold);
    result.original_turn_count = static_cast<int>(result.original_turns.size());
    
    if (result.original_turns.empty()) {
        result.optimized_path = path;
        result.success = true;
        result.optimized_turn_count = 0;
        return result;
    }
    
    // Step 2: 合并相邻转弯点
    std::vector<TurnPoint> merged_turns;
    if (config.enable_merge) {
        merged_turns = mergeTurnPoints(result.original_turns, config);
        result.merged_count = static_cast<int>(result.original_turns.size() - merged_turns.size());
    } else {
        merged_turns = result.original_turns;
    }
    
    result.optimized_turns = merged_turns;
    result.optimized_turn_count = static_cast<int>(merged_turns.size());
    
    // Step 3: 重建路径（可选）
    // 目前简化处理：不重建路径，只统计转弯减少
    // 完整实现需要根据合并后的转弯点重建路径
    result.optimized_path = path;  // 暂时保持原路径
    
    // 计算转弯减少率
    if (result.original_turn_count > 0) {
        result.reduction_rate = static_cast<double>(result.merged_count) / result.original_turn_count;
    }
    
    result.success = true;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.optimization_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    return result;
}

int TurnOptimizer::countTurns(
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    double angle_threshold) const
{
    if (path.size() < 3) {
        return 0;
    }
    
    int turn_count = 0;
    double prev_direction = computeDirection(path[0].pose.position, path[1].pose.position);
    
    for (size_t i = 2; i < path.size(); ++i) {
        double current_direction = computeDirection(path[i-1].pose.position, path[i].pose.position);
        double angle_change = computeAngleDifference(prev_direction, current_direction);
        
        if (angle_change > angle_threshold) {
            ++turn_count;
        }
        
        prev_direction = current_direction;
    }
    
    return turn_count;
}

std::vector<TurnPoint> TurnOptimizer::detectTurnPoints(
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    double angle_threshold) const
{
    std::vector<TurnPoint> turns;
    
    if (path.size() < 3) {
        return turns;
    }
    
    // 使用滑动窗口计算方向（更稳定）
    for (size_t i = 1; i < path.size() - 1; ++i) {
        // 计算前后方向
        double direction_before = computeSegmentDirection(path, i - 1, 2);
        double direction_after = computeSegmentDirection(path, i + 1, 2);
        
        // 计算角度变化
        double angle_change = computeAngleDifference(direction_before, direction_after);
        
        if (angle_change > angle_threshold) {
            TurnPoint turn;
            turn.index = i;
            turn.angle_change = angle_change;
            turn.angle = direction_after;  // 转弯后的方向
            turn.position = path[i].pose.position;
            turn.type = classifyTurn(angle_change);
            
            // 标记可优化的转弯点
            turn.can_merge = (turn.type != TurnType::U_TURN);  // U形转弯不合并
            turn.can_smooth = (turn.type == TurnType::GENTLE || turn.type == TurnType::MEDIUM);
            
            turns.push_back(turn);
        }
    }
    
    // 检查相邻转弯点是否可合并
    for (size_t i = 0; i < turns.size(); ++i) {
        if (i > 0) {
            double dist = distance(turns[i-1].position, turns[i].position);
            if (dist < 20.0) {  // 20米内的相邻转弯点可能可合并
                turns[i].merge_candidate_index = i - 1;
                turns[i-1].merge_candidate_index = i;
            }
        }
    }
    
    return turns;
}

double TurnOptimizer::computeDirection(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2) const
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    
    // 计算方向角（范围 [0, 2π)）
    double angle = std::atan2(dy, dx);
    if (angle < 0) {
        angle += 2.0 * M_PI;
    }
    
    return angle;
}

double TurnOptimizer::computeAngleDifference(double angle1, double angle2) const
{
    double diff = std::abs(angle1 - angle2);
    
    // 考虑角度周期性
    if (diff > M_PI) {
        diff = 2.0 * M_PI - diff;
    }
    
    return diff;
}

TurnType TurnOptimizer::classifyTurn(double angle_change) const
{
    // 根据角度变化量分类转弯
    if (angle_change < 0.1) {
        return TurnType::NONE;
    } else if (angle_change < M_PI / 4.0) {  // < 45°
        return TurnType::GENTLE;
    } else if (angle_change < M_PI / 2.0) {  // < 90°
        return TurnType::MEDIUM;
    } else if (angle_change > M_PI - 0.2) {  // 接近 180°
        return TurnType::U_TURN;
    } else {
        return TurnType::SHARP;
    }
}

std::vector<TurnPoint> TurnOptimizer::mergeTurnPoints(
    const std::vector<TurnPoint> & turns,
    const TurnOptimizerConfig & config) const
{
    if (turns.size() <= 1) {
        return turns;
    }
    
    std::vector<TurnPoint> merged;
    std::vector<bool> processed(turns.size(), false);
    
    for (size_t i = 0; i < turns.size(); ++i) {
        if (processed[i]) {
            continue;
        }
        
        // 检查是否可以与下一个转弯点合并
        if (i + 1 < turns.size() && !processed[i + 1]) {
            // 检查距离和角度条件
            double dist = distance(turns[i].position, turns[i + 1].position);
            double angle_diff = computeAngleDifference(
                turns[i].angle, turns[i + 1].angle);
            
            // 合并条件：距离小于阈值，且两个转弯角度相近（小角度转弯可以合并）
            bool can_merge = (dist < config.merge_distance_threshold * 0.05) &&  // 转换为米
                            (angle_diff < config.merge_angle_threshold) &&
                            turns[i].can_merge && turns[i + 1].can_merge &&
                            (turns[i].type == TurnType::GENTLE || 
                             turns[i + 1].type == TurnType::GENTLE);
            
            if (can_merge) {
                // 合成一个新的转弯点（取中间位置）
                TurnPoint merged_turn;
                merged_turn.index = turns[i].index;
                merged_turn.position.x = (turns[i].position.x + turns[i + 1].position.x) / 2.0;
                merged_turn.position.y = (turns[i].position.y + turns[i + 1].position.y) / 2.0;
                merged_turn.position.z = 0.0;
                merged_turn.angle_change = std::max(turns[i].angle_change, turns[i + 1].angle_change);
                merged_turn.angle = turns[i + 1].angle;
                merged_turn.type = classifyTurn(merged_turn.angle_change);
                merged_turn.can_merge = false;
                merged_turn.can_smooth = merged_turn.type == TurnType::GENTLE;
                
                merged.push_back(merged_turn);
                processed[i] = true;
                processed[i + 1] = true;
                continue;
            }
        }
        
        // 不能合并，保留原转弯点
        merged.push_back(turns[i]);
        processed[i] = true;
    }
    
    return merged;
}

std::vector<geometry_msgs::msg::PoseStamped> TurnOptimizer::rebuildPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & original_path,
    const std::vector<TurnPoint> & turns,
    const std::vector<TurnPoint> & merged_turns) const
{
    // 简化实现：目前不重建路径
    // 完整实现需要：
    // 1. 根据合并后的转弯点位置
    // 2. 删除被跳过的路径点
    // 3. 添加平滑插值点
    
    return original_path;
}

double TurnOptimizer::computeSegmentDirection(
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    size_t index,
    int window_size) const
{
    if (index >= path.size()) {
        return 0.0;
    }
    
    // 计算窗口内的平均方向
    double total_dx = 0.0;
    double total_dy = 0.0;
    int count = 0;
    
    for (int offset = 0; offset < window_size && index + offset < path.size(); ++offset) {
        size_t i = index + offset;
        size_t j = std::min(i + 1, path.size() - 1);
        
        total_dx += path[j].pose.position.x - path[i].pose.position.x;
        total_dy += path[j].pose.position.y - path[i].pose.position.y;
        ++count;
    }
    
    if (count == 0) {
        return 0.0;
    }
    
    double angle = std::atan2(total_dy / count, total_dx / count);
    if (angle < 0) {
        angle += 2.0 * M_PI;
    }
    
    return angle;
}

std::vector<geometry_msgs::msg::Point> TurnOptimizer::interpolateSmooth(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double angle1,
    double angle2,
    int num_points) const
{
    std::vector<geometry_msgs::msg::Point> points;
    
    // 简化实现：线性插值
    // 完整实现需要使用贝塞尔曲线或Dubins曲线
    
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        geometry_msgs::msg::Point pt;
        pt.x = p1.x + t * (p2.x - p1.x);
        pt.y = p1.y + t * (p2.y - p1.y);
        pt.z = 0.0;
        points.push_back(pt);
    }
    
    return points;
}

}  // namespace coverage_planner