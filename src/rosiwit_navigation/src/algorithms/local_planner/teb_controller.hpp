#ifndef ROSIWIT_NAVIGATION__CONTROLLERS__TEB_CONTROLLER_HPP_
#define ROSIWIT_NAVIGATION__CONTROLLERS__TEB_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include "rosiwit_navigation/nav_core/i_controller.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"
#include "rosiwit_navigation/nav_core/logger.hpp"

namespace rosiwit_navigation {
namespace controllers {

class TebController : public core::IControllerStrategy {
public:
    TebController();
    ~TebController() override;

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
    struct Pose { double x, y, theta, dt; };  // TEB pose with time

    std::string name_;
    core::Logger logger_;
    core::ControllerConfig cfg_;
    core::Path path_;
    core::ObstacleArray obstacles_;
    std::vector<Pose> teb_;
    size_t idx_;
    double max_vx_, max_vw_, goal_tol_;
    bool initialized_, goal_reached_;

    void initTeb();
    void optimize();
    void addPoses();
    void removePoses();
    double cost() const;
    double smoothnessCost() const;
    double obstacleCost() const;
    double timeCost() const;
    size_t findClosest(const core::Pose2D& pose) const;
    static double normAngle(double a);
    static double dist(double x1, double y1, double x2, double y2);
};

}}  // namespaces
#endif
