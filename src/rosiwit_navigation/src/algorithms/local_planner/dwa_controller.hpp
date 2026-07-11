#ifndef ROSIWIT_NAVIGATION__CONTROLLERS__DWA_CONTROLLER_HPP_
#define ROSIWIT_NAVIGATION__CONTROLLERS__DWA_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include "rosiwit_navigation/nav_core/i_controller.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"
#include "rosiwit_navigation/nav_core/logger.hpp"

namespace rosiwit_navigation {
namespace controllers {

class DwaController : public core::IControllerStrategy {
public:
    DwaController();
    ~DwaController() override;

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
    struct Sample { double v, w; double heading, clearance, velocity; double cost; };
    struct VelocityBounds { double min_vx, max_vx, min_vw, max_vw; };

    std::string name_;
    core::Logger logger_;
    core::ControllerConfig cfg_;
    core::Path path_;
    core::ObstacleArray obstacles_;
    size_t idx_;
    double max_vx_, min_vx_, max_vw_, max_accel_, max_dvw_;
    double goal_tol_, dt_, sim_time_;
    bool initialized_, goal_reached_;

    std::vector<Sample> search(const core::Pose2D& pose, const core::VelocityCommand& vel);
    VelocityBounds dynamicWindow(const core::VelocityCommand& vel) const;
    void simulate(const core::Pose2D& pose, Sample& s) const;
    double headingCost(const core::Pose2D& pose) const;
    double clearanceCost(const core::Pose2D& pose) const;
    size_t findClosest(const core::Pose2D& pose) const;
    static double normAngle(double a);
};

}}  // namespaces
#endif
