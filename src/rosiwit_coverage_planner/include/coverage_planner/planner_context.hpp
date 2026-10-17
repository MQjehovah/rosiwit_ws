// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#ifndef COVERAGE_PLANNER__PLANNER_CONTEXT_HPP_
#define COVERAGE_PLANNER__PLANNER_CONTEXT_HPP_

#include <memory>
#include <string>
#include <stdexcept>
#include "coverage_planner/i_planner.hpp"

namespace coverage_planner
{

/**
 * @brief 规划模式枚举
 */
enum class PlannerMode
{
    ZIGZAG,    // 弓字形模式
    SPIRAL,    // 回字形模式
};

/**
 * @brief 策略上下文类
 * 
 * 管理不同的规划器实例，根据参数选择算法
 */
class PlannerContext
{
public:
    PlannerContext();
    ~PlannerContext() = default;

    /**
     * @brief 选择规划器
     * 
     * @param mode 规划模式
     * @return IPlanner* 规划器实例指针
     */
    IPlanner* selectPlanner(PlannerMode mode);

    /**
     * @brief 通过字符串选择规划器
     * 
     * @param mode_str 规划模式字符串 ("zigzag" 或 "spiral")
     * @return IPlanner* 规划器实例指针
     */
    IPlanner* selectPlanner(const std::string & mode_str);

    /**
     * @brief 获取当前规划器
     * 
     * @return IPlanner* 当前规划器实例指针
     */
    IPlanner* getCurrentPlanner() const;

    /**
     * @brief 获取当前规划模式
     * 
     * @return PlannerMode 当前模式
     */
    PlannerMode getCurrentMode() const;

    /**
     * @brief 重置所有规划器状态
     */
    void resetAll();

private:
    // 规划器实例
    std::unique_ptr<IPlanner> zigzag_planner_;
    std::unique_ptr<IPlanner> spiral_planner_;
    
    // 当前模式
    PlannerMode current_mode_;
    IPlanner* current_planner_;

    /**
     * @brief 初始化规划器
     */
    void initializePlanners();
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__PLANNER_CONTEXT_HPP_