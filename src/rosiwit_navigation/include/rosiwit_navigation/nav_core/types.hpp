#ifndef ROSIWIT_NAVIGATION__CORE__TYPES_HPP_
#define ROSIWIT_NAVIGATION__CORE__TYPES_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <cmath>

namespace rosiwit_navigation
{
namespace core
{

struct Pose2D
{
  double x;
  double y;
  double theta;

  Pose2D() : x(0.0), y(0.0), theta(0.0) {}
  Pose2D(double x_in, double y_in, double theta_in) : x(x_in), y(y_in), theta(theta_in) {}

  double distanceTo(const Pose2D & other) const
  {
    double dx = x - other.x;
    double dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
  }
};

struct PathPoint
{
  Pose2D pose;
  double velocity_x;
  double velocity_theta;

  PathPoint() : velocity_x(0.0), velocity_theta(0.0) {}
  PathPoint(const Pose2D & p) : pose(p), velocity_x(0.0), velocity_theta(0.0) {}
};

struct Path
{
  std::vector<PathPoint> points;
  bool empty() const { return points.empty(); }
  size_t size() const { return points.size(); }
};

struct VelocityCommand
{
  double linear_x;
  double lateral_y;
  double angular_z;

  VelocityCommand() : linear_x(0.0), lateral_y(0.0), angular_z(0.0) {}
  VelocityCommand(double vx, double vy, double wz)
    : linear_x(vx), lateral_y(vy), angular_z(wz) {}
};

struct PlannerConfig
{
  std::string name;
  double resolution;
  int max_iterations;
  double timeout;
};

struct KinematicsLimits
{
  double max_velocity_x;
  double max_velocity_theta;
};

struct ControllerConfig
{
  double lookahead_distance;
  double xy_goal_tolerance;
  double slow_down_distance;
  double max_lookahead_distance;
  double min_lookahead_distance;
  double lookahead_gain;
  KinematicsLimits kinematics;
  double max_linear_velocity;
  double max_angular_velocity;
};

struct CostmapGridInfo
{
  unsigned int width;
  unsigned int height;
  double resolution;
  struct
  {
    struct
    {
      double x;
      double y;
    } position;
  } origin;
};

struct CostmapGrid
{
  CostmapGridInfo info;
  std::vector<int8_t> data;
};

struct Costmap
{
  std::shared_ptr<CostmapGrid> grid;
  unsigned int width;
  unsigned int height;
  double resolution;
  double origin_x;
  double origin_y;
  std::vector<unsigned char> data;
};

struct Obstacle
{
  double x;
  double y;
  double radius;
};

using ObstacleArray = std::vector<Obstacle>;

/**
 * @brief 激光扫描数据（纯C++，替代 sensor_msgs::LaserScan）
 */
struct RangeScan
{
  std::vector<float> ranges;
  std::vector<float> intensities;
  float angle_min = -3.14159f;
  float angle_max = 3.14159f;
  float angle_increment = 0.0174533f;
  float range_min = 0.1f;
  float range_max = 10.0f;
  size_t size() const { return ranges.size(); }
};

/**
 * @brief 点云数据（纯C++，替代 sensor_msgs::PointCloud2）
 */
struct PointCloud
{
  struct Point { float x, y, z; };
  std::vector<Point> points;
  bool empty() const { return points.empty(); }
  size_t size() const { return points.size(); }
};

struct TrajectoryPoint
{
  double x;
  double y;
  double theta;
  double vx;
  double vtheta;
  double time;
  double acceleration;
};

enum class ErrorCode
{
  SUCCESS = 0,
  NOT_INITIALIZED,
  PLANNING_FAILED,
  START_IN_OBSTACLE,
  GOAL_IN_OBSTACLE,
  TIMEOUT,
  NO_VALID_PATH,
  UNKNOWN
};

template<typename T>
class Result
{
public:
  static Result ok(const T & value) { return Result(value); }
  static Result error(ErrorCode code, const std::string & msg) { return Result(code, msg); }

  bool is_ok() const { return success_; }
  bool is_error() const { return !success_; }
  const T & value() const { return value_; }
  ErrorCode error_code() const { return error_code_; }
  std::string error_message() const { return error_message_; }

private:
  Result(const T & value) : success_(true), value_(value), error_code_(ErrorCode::SUCCESS) {}
  Result(ErrorCode code, const std::string & msg)
    : success_(false), error_code_(code), error_message_(msg) {}

  bool success_;
  T value_;
  ErrorCode error_code_;
  std::string error_message_;
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__CORE__TYPES_HPP_
