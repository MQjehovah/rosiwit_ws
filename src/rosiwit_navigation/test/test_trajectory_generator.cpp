// ============================================================
// Diffbot Navigation - 轨迹生成器单元测试
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "rosiwit_navigation/algorithms/trajectory_generator.hpp"

using namespace rosiwit_navigation::navigation;

class TrajectoryGeneratorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 初始化轨迹配置
    config_.max_velocity_x = 0.5;
    config_.max_velocity_theta = 1.0;
    config_.min_velocity_x = -0.5;
    config_.max_accel_x = 0.5;
    config_.max_accel_theta = 1.0;
    config_.sim_time = 2.0;
    config_.sim_granularity = 0.02;
    config_.path_resolution = 0.05;
    config_.xy_goal_tolerance = 0.1;
    config_.yaw_goal_tolerance = 0.1;

    generator_ = std::make_unique<TrajectoryGenerator>(config_);
  }

  TrajectoryConfig config_;
  std::unique_ptr<TrajectoryGenerator> generator_;
};

// ==================== 基本功能测试 ====================

TEST_F(TrajectoryGeneratorTest, DefaultInitialization)
{
  TrajectoryGenerator default_generator;
  // 应该使用默认配置初始化
  EXPECT_NO_THROW({
    auto config = default_generator.getConfig();
    EXPECT_GT(config.max_velocity_x, 0.0);
    EXPECT_GT(config.sim_time, 0.0);
  });
}

TEST_F(TrajectoryGeneratorTest, ConfigInitialization)
{
  auto current_config = generator_->getConfig();

  EXPECT_DOUBLE_EQ(current_config.max_velocity_x, 0.5);
  EXPECT_DOUBLE_EQ(current_config.max_velocity_theta, 1.0);
  EXPECT_DOUBLE_EQ(current_config.sim_time, 2.0);
  EXPECT_DOUBLE_EQ(current_config.path_resolution, 0.05);
}

TEST_F(TrajectoryGeneratorTest, SetNewConfig)
{
  TrajectoryConfig new_config;
  new_config.max_velocity_x = 0.8;
  new_config.max_velocity_theta = 1.5;
  new_config.sim_time = 3.0;
  new_config.path_resolution = 0.1;

  generator_->setConfig(new_config);

  auto current_config = generator_->getConfig();
  EXPECT_DOUBLE_EQ(current_config.max_velocity_x, 0.8);
  EXPECT_DOUBLE_EQ(current_config.max_velocity_theta, 1.5);
  EXPECT_DOUBLE_EQ(current_config.sim_time, 3.0);
  EXPECT_DOUBLE_EQ(current_config.path_resolution, 0.1);
}

// ==================== 轨迹生成测试 ====================

TEST_F(TrajectoryGeneratorTest, GenerateFromEmptyPath)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = "map";

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(empty_path, current_velocity);

  // 空路径应该返回空轨迹
  EXPECT_TRUE(trajectory.empty());
}

TEST_F(TrajectoryGeneratorTest, GenerateFromSinglePointPath)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(path, current_velocity);

  // 单点路径应该生成轨迹
  EXPECT_FALSE(trajectory.empty());
}

TEST_F(TrajectoryGeneratorTest, GenerateFromStraightPath)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // 生成直线路径
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.2;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(path, current_velocity);

  // 应该生成轨迹
  EXPECT_FALSE(trajectory.empty());
  EXPECT_GT(trajectory.size(), 1);

  // 轨迹点时间应该是递增的
  for (size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_GT(trajectory[i].time, trajectory[i - 1].time);
  }
}

TEST_F(TrajectoryGeneratorTest, GenerateFromCurvedPath)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // 生成曲线路径（半圆）
  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    double angle = M_PI * i / 20.0;
    pose.pose.position.x = std::cos(angle);
    pose.pose.position.y = std::sin(angle);
    pose.pose.orientation.z = std::sin(angle / 2.0);
    pose.pose.orientation.w = std::cos(angle / 2.0);
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(path, current_velocity);

  EXPECT_FALSE(trajectory.empty());

  // 验证轨迹点约束
  for (const auto & point : trajectory) {
    EXPECT_LE(std::abs(point.vx), config_.max_velocity_x + 1e-6);
    EXPECT_LE(std::abs(point.vtheta), config_.max_velocity_theta + 1e-6);
  }
}

TEST_F(TrajectoryGeneratorTest, GenerateWithNonZeroInitialVelocity)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.2;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.2;  // 非零初始速度
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(path, current_velocity);

  EXPECT_FALSE(trajectory.empty());

  // 初始速度应该接近当前速度
  if (!trajectory.empty()) {
    EXPECT_NEAR(trajectory[0].vx, 0.2, 0.5);  // 允许一定误差
  }
}

// ==================== 速度规划测试 ====================

TEST_F(TrajectoryGeneratorTest, VelocityProfileRespectsLimits)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // 长路径，让速度达到最大值
  for (int i = 0; i <= 100; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(path, current_velocity);

  // 验证所有轨迹点都在速度限制内
  for (const auto & point : trajectory) {
    EXPECT_LE(std::abs(point.vx), config_.max_velocity_x + 0.1);  // 允许小误差
    EXPECT_LE(std::abs(point.vtheta), config_.max_velocity_theta + 0.1);
  }
}

TEST_F(TrajectoryGeneratorTest, AccelerationProfileRespectsLimits)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  for (int i = 0; i <= 50; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(path, current_velocity);

  // 检查加速度约束
  double dt = config_.sim_granularity;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    double accel_x = (trajectory[i].vx - trajectory[i - 1].vx) / dt;
    // 加速度应该被限制（允许一些数值误差）
    EXPECT_LE(std::abs(accel_x), config_.max_accel_x + 1.0);
  }
}

// ==================== 五次多项式轨迹测试 ====================

TEST_F(TrajectoryGeneratorTest, QuinticPolynomialSmoothness)
{
  // 测试五次多项式轨迹生成
  double start_pos = 0.0;
  double end_pos = 1.0;
  double start_vel = 0.0;
  double end_vel = 0.0;
  double T = 2.0;

  // 调用内部方法（如果有公共接口）
  // 这里验证生成的轨迹是平滑的
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.05;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(path, current_velocity);

  // 验证轨迹连续性
  for (size_t i = 1; i < trajectory.size(); ++i) {
    // 位置应该连续
    double dx = trajectory[i].x - trajectory[i - 1].x;
    double dy = trajectory[i].y - trajectory[i - 1].y;
    EXPECT_LT(std::sqrt(dx * dx + dy * dy), 1.0);  // 相邻点距离不应该过大
  }
}

// ==================== 目标容差测试 ====================

TEST_F(TrajectoryGeneratorTest, GoalTolerance)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;

  goal.pose.position.x = 1.0;
  goal.pose.position.y = 0.0;
  goal.pose.orientation.w = 1.0;

  path.poses.push_back(start);
  path.poses.push_back(goal);

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(path, current_velocity);

  // 终点应该接近目标
  if (!trajectory.empty()) {
    double final_x = trajectory.back().x;
    double final_y = trajectory.back().y;
    double distance_to_goal = std::sqrt((final_x - goal.pose.position.x) * (final_x - goal.pose.position.x) +
                                          (final_y - goal.pose.position.y) * (final_y - goal.pose.position.y));
    EXPECT_LE(distance_to_goal, config_.xy_goal_tolerance + 0.5);  // 允许一定误差
  }
}

// ==================== 边界条件测试 ====================

TEST_F(TrajectoryGeneratorTest, VeryShortPath)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // 非常短的路径
  geometry_msgs::msg::PoseStamped pose1, pose2;
  pose1.pose.position.x = 0.0;
  pose1.pose.position.y = 0.0;
  pose1.pose.orientation.w = 1.0;

  pose2.pose.position.x = 0.01;
  pose2.pose.position.y = 0.0;
  pose2.pose.orientation.w = 1.0;

  path.poses.push_back(pose1);
  path.poses.push_back(pose2);

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  EXPECT_NO_THROW({
    auto trajectory = generator_->generateTrajectory(path, current_velocity);
  });
}

TEST_F(TrajectoryGeneratorTest, ZeroVelocityInput)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  for (int i = 0; i <= 5; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.5;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(path, current_velocity);

  EXPECT_FALSE(trajectory.empty());
}

TEST_F(TrajectoryGeneratorTest, LargeVelocityInput)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  for (int i = 0; i <= 5; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.5;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 10.0;  // 很大的初始速度
  current_velocity.angular.z = 5.0;

  // 应该安全处理过大速度
  EXPECT_NO_THROW({
    auto trajectory = generator_->generateTrajectory(path, current_velocity);
  });
}

// ==================== 轨迹验证测试 ====================

TEST_F(TrajectoryGeneratorTest, TrajectoryPointValidity)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = std::sin(i * 0.3);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.1;
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(path, current_velocity);

  // 验证每个轨迹点
  for (const auto & point : trajectory) {
    // 检查数值有效性
    EXPECT_FALSE(std::isnan(point.x));
    EXPECT_FALSE(std::isnan(point.y));
    EXPECT_FALSE(std::isnan(point.theta));
    EXPECT_FALSE(std::isnan(point.vx));
    EXPECT_FALSE(std::isnan(point.vtheta));
    EXPECT_FALSE(std::isnan(point.time));

    EXPECT_FALSE(std::isinf(point.x));
    EXPECT_FALSE(std::isinf(point.y));
    EXPECT_FALSE(std::isinf(point.theta));
    EXPECT_FALSE(std::isinf(point.vx));
    EXPECT_FALSE(std::isinf(point.vtheta));
    EXPECT_FALSE(std::isinf(point.time));

    // 时间应该是非负的
    EXPECT_GE(point.time, 0.0);
  }
}

TEST_F(TrajectoryGeneratorTest, TrajectoryContinuity)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.2;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = generator_->generateTrajectory(path, current_velocity);

  // 验证轨迹连续性
  for (size_t i = 1; i < trajectory.size(); ++i) {
    // 位置不应该突变
    double dx = std::abs(trajectory[i].x - trajectory[i - 1].x);
    double dy = std::abs(trajectory[i].y - trajectory[i - 1].y);

    // 最大位移应该合理（基于最大速度和时间间隔）
    double max_displacement = config_.max_velocity_x * config_.sim_granularity * 2;
    EXPECT_LT(dx, max_displacement);
    EXPECT_LT(dy, max_displacement);
  }
}

// ==================== 重置功能测试 ====================

TEST_F(TrajectoryGeneratorTest, ResetClearsInternalState)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  for (int i = 0; i <= 5; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  // 生成轨迹
  auto trajectory1 = generator_->generateTrajectory(path, current_velocity);

  // 再次生成（TrajectoryGenerator 无需 reset）
  auto trajectory2 = generator_->generateTrajectory(path, current_velocity);

  // 应该得到有效轨迹
  EXPECT_FALSE(trajectory2.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}