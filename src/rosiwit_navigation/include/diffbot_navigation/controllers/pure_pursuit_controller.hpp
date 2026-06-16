// ============================================================
// Diffbot Navigation - Pure Pursuit 控制器
// 路径跟踪控制器实现
// ============================================================

#ifndef DIFFBOT_NAVIGATION__CONTROLLERS__PURE_PURSUIT_CONTROLLER_HPP_
#define DIFFBOT_NAVIGATION__CONTROLLERS__PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "diffbot_navigation/core/i_controller.hpp"
#include "diffbot_navigation/core/types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffbot_navigation
{
namespace controllers
{

/**
 * @class PurePursuitController
 * @brief Pure Pursuit控制器实现
 *
 * 使用Pure Pursuit算法进行路径跟踪，
 * 适用于差速轮机器人
 * 继承 IControllerStrategy 抽象接口，支持策略模式
 */
class PurePursuitController : public core::IControllerStrategy
{
public:
    PurePursuitController();
    ~PurePursuitController() override;

    // IControllerStrategy接口实现
    bool initialize(const core::ControllerConfig& config) override;
    void setPath(const core::Path& path) override;
    core::VelocityCommand computeVelocityCommand(
        const core::Pose2D& current_pose,
        const core::VelocityCommand& current_velocity) override;
    bool isGoalReached(const core::Pose2D& current_pose) const override;
    size_t getCurrentWaypointIndex() const override;
    double getProgress() const override;
    void reset() override;
    void setVelocityLimit(double linear_x, double angular_z) override;
    std::string getName() const override;
    std::string getVersion() const override;
    void setObstacles(const core::ObstacleArray& obstacles) override;

private:
    std::string controller_name_;
    rclcpp::Logger logger_;
    core::ControllerConfig config_;
    core::Path path_;
    size_t current_waypoint_idx_;
    
    // 速度限制
    double max_linear_velocity_;
    double max_angular_velocity_;
    
    // Pure Pursuit参数
    double lookahead_distance_;
    double goal_tolerance_;
    
    // 状态标志
    bool initialized_;
    bool goal_reached_;
    
    // 内部方法
    size_t findLookaheadPoint(const core::Pose2D& current_pose) const;
    double calculateLookaheadDistance(double current_speed) const;
    core::VelocityCommand purePursuitAlgorithm(
        const core::Pose2D& current_pose,
        const core::Pose2D& lookahead_point) const;
    double normalizeAngle(double angle) const;
};

}  // namespace controllers
}  // namespace diffbot_navigation

#endif  // DIFFBOT_NAVIGATION__CONTROLLERS__PURE_PURSUIT_CONTROLLER_HPP_