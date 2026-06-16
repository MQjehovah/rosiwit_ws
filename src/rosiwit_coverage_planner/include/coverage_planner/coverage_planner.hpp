// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#ifndef COVERAGE_PLANNER__COVERAGE_PLANNER_HPP_
#define COVERAGE_PLANNER__COVERAGE_PLANNER_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "coverage_planner/planner_context.hpp"
#include "coverage_planner/i_planner.hpp"

namespace coverage_planner
{

/**
 * @brief ROS2全覆盖路径规划节点
 * 
 * 管理ROS2接口，订阅地图和初始位置，发布规划路径
 */
class CoveragePlannerNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * 
     * @param options ROS2节点选项
     */
    explicit CoveragePlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    /**
     * @brief 析构函数
     */
    ~CoveragePlannerNode() override;

private:
    // ========== ROS2接口 ==========
    
    // 地图订阅
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    // 初始位置订阅
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    
    // 覆盖路径发布
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // 触发规划服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
    
    // ========== 回调函数 ==========
    
    /**
     * @brief 地图回调函数
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    /**
     * @brief 初始位置回调函数
     */
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    /**
     * @brief 触发规划服务回调
     */
    void planServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // ========== 核心方法 ==========
    
    /**
     * @brief 执行全覆盖路径规划
     */
    bool planCoverage();
    
    /**
     * @brief 发布规划路径
     */
    void publishPath(const std::vector<geometry_msgs::msg::PoseStamped> & path);
    
    /**
     * @brief 初始化参数
     */
    void initializeParameters();
    
    /**
     * @brief 声明ROS2参数
     */
    void declareParameters();
    
    // ========== 内部状态 ==========
    
    // 规划上下文
    std::unique_ptr<PlannerContext> planner_context_;
    
    // 当前地图
    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_;
    
    // 起始位姿
    geometry_msgs::msg::Pose start_pose_;
    bool start_pose_received_;
    
    // 规划配置参数
    PlannerConfig planner_config_;
    
    // 规划模式
    PlannerMode coverage_mode_;
    
    // 规划结果
    PlannerResult last_result_;
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__COVERAGE_PLANNER_HPP_