// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include "coverage_planner/planner_context.hpp"
#include "coverage_planner/zigzag_planner.hpp"
#include "coverage_planner/spiral_planner.hpp"
#include <algorithm>

namespace coverage_planner
{

PlannerContext::PlannerContext()
: current_mode_(PlannerMode::ZIGZAG),
  current_planner_(nullptr)
{
    initializePlanners();
}

void PlannerContext::initializePlanners()
{
    // 创建弓字形规划器
    zigzag_planner_ = std::make_unique<ZigzagPlanner>();
    
    // 创建回字形规划器
    spiral_planner_ = std::make_unique<SpiralPlanner>();
    
    // 默认使用弓字形规划器
    current_planner_ = zigzag_planner_.get();
    current_mode_ = PlannerMode::ZIGZAG;
}

IPlanner* PlannerContext::selectPlanner(PlannerMode mode)
{
    switch (mode) {
        case PlannerMode::ZIGZAG:
            current_planner_ = zigzag_planner_.get();
            current_mode_ = PlannerMode::ZIGZAG;
            break;
            
        case PlannerMode::SPIRAL:
            current_planner_ = spiral_planner_.get();
            current_mode_ = PlannerMode::SPIRAL;
            break;
            
        default:
            current_planner_ = zigzag_planner_.get();
            current_mode_ = PlannerMode::ZIGZAG;
            break;
    }
    
    return current_planner_;
}

IPlanner* PlannerContext::selectPlanner(const std::string & mode_str)
{
    // 转换为小写以方便比较
    std::string mode_lower = mode_str;
    std::transform(mode_lower.begin(), mode_lower.end(), mode_lower.begin(), ::tolower);
    
    if (mode_lower == "zigzag") {
        return selectPlanner(PlannerMode::ZIGZAG);
    } else if (mode_lower == "spiral") {
        return selectPlanner(PlannerMode::SPIRAL);
    } else {
        // 默认使用弓字形
        return selectPlanner(PlannerMode::ZIGZAG);
    }
}

IPlanner* PlannerContext::getCurrentPlanner() const
{
    return current_planner_;
}

PlannerMode PlannerContext::getCurrentMode() const
{
    return current_mode_;
}

void PlannerContext::resetAll()
{
    if (zigzag_planner_) {
        zigzag_planner_->reset();
    }
    
    if (spiral_planner_) {
        spiral_planner_->reset();
    }
}

}  // namespace coverage_planner