// ============================================================
// Diffbot Navigation - NavFn 规划器
// 简化版的NavFn规划器实现
// ============================================================

#ifndef ROSIWIT_NAVIGATION__PLANNERS__NAVFN_PLANNER_HPP_
#define ROSIWIT_NAVIGATION__PLANNERS__NAVFN_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rosiwit_navigation/nav_core/i_planner.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rosiwit_navigation
{
namespace planners
{

/**
 * @class NavFnPlanner
 * @brief NavFn规划器实现
 *
 * 简化版的NavFn规划器，使用势场方法进行路径规划
 * 继承 IPlannerStrategy 抽象接口，支持策略模式
 */
class NavFnPlanner : public core::IPlannerStrategy
{
public:
    NavFnPlanner();
    ~NavFnPlanner() override;

    // IPlannerStrategy接口实现
    bool initialize(const core::PlannerConfig& config) override;
    void setCostmap(const core::Costmap& costmap) override;
    core::Result<core::Path> plan(const core::Pose2D& start, const core::Pose2D& goal) override;
    void planAsync(const core::Pose2D& start, const core::Pose2D& goal,
                   std::function<void(const core::Result<core::Path>&)> callback) override;
    void cancel() override;
    bool isPlanning() const override;
    std::string getName() const override;
    std::string getVersion() const override;
    void reset() override;

private:
    std::string planner_name_;
    rclcpp::Logger logger_;
    core::PlannerConfig config_;
    std::shared_ptr<core::Costmap> costmap_;
    std::vector<unsigned char> costmap_data_;
    unsigned int nx_, ny_;
    bool planning_active_;
    
    // NavFn核心数据
    std::vector<float> potential_;
    std::vector<int> gradx_, grady_;
    
    // 内部方法
    void computePotential(const core::Pose2D& goal);
    bool findPath(const core::Pose2D& start, const core::Pose2D& goal, core::Path& path);
    void addWavefrontPoint(unsigned int x, unsigned int y);
    unsigned char getCost(unsigned int x, unsigned int y);
};

}  // namespace planners
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__PLANNERS__NAVFN_PLANNER_HPP_