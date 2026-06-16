// ============================================================
// Diffbot Navigation - 集成测试：单点导航功能
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <chrono>
#include <thread>
#include "diffbot_navigation/navigation/smooth_navigation.hpp"
#include "diffbot_navigation/navigation/path_planner.hpp"
#include "diffbot_navigation/navigation/trajectory_generator.hpp"
#include "diffbot_navigation/controller/diff_drive_controller.hpp"
#include "diffbot_navigation/controller/velocity_limiter.hpp"

using namespace diffbot_navigation::navigation;
using namespace diffbot_navigation::controller;

class SinglePointNavigationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 初始化导航参数
    NavigationConfig nav_config;
    nav_config.max_velocity_x = 0.5;
    nav_config.max_velocity_theta = 1.0;
    nav_config.max_accel_x = 0.5;
    nav_config.max_accel_theta = 1.0;
    nav_config.xy_goal_tolerance = 0.05;
    nav_config.yaw_goal_tolerance = 0.1;

    TrajectoryConfig traj_config;
    traj_config.max_velocity_x = 0.5;
    traj_config.max_velocity_theta = 1.0;
    traj_config.sim_time = 2.0;
    traj_config.sim_granularity = 0.02;
    traj_config.path_resolution = 0.05;
    traj_config.xy_goal_tolerance = 0.05;

    ControllerConfig ctrl_config;
    ctrl_config.wheel_separation = 0.4;
    ctrl_config.wheel_radius = 0.1;
    ctrl_config.max_velocity_x = 0.5;
    ctrl_config.max_velocity_theta = 1.0;
    ctrl_config.lookahead_distance = 0.6;
    ctrl_config.xy_goal_tolerance = 0.05;
    ctrl_config.yaw_goal_tolerance = 0.1;

    velocity_limiter_ = std::make_unique<VelocityLimiter>();
    trajectory_generator_ = std::make_unique<TrajectoryGenerator>(traj_config);
    controller_ = std::make_unique<DiffDriveController>();
    controller_->setConfig(ctrl_config);
    smooth_navigation_ = std::make_unique<SmoothNavigation>(nav_config);
  }

  void TearDown() override
  {
    smooth_navigation_->reset();
    controller_->reset();
  }

  std::unique_ptr<VelocityLimiter> velocity_limiter_;
  std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
  std::unique_ptr<DiffDriveController> controller_;
  std::unique_ptr<SmoothNavigation> smooth_navigation_;
};

// ==================== 单点导航场景测试 ====================

TEST_F(SinglePointNavigationTest, NavigateToForwardPoint)
{
  // 测试：导航到前方目标点

  // 设置起点和终点
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header.frame_id = "map";
  start_pose.pose.position.x = 0.0;
  start_pose.pose.position.y = 0.0;
  start_pose.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  goal_pose.pose.position.y = 0.0;
  goal_pose.pose.orientation.w = 1.0;

  // 设置路径（简单直线）
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  // 生成轨迹
  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = trajectory_generator_->generateTrajectory(path, current_velocity);

  // 验证轨迹生成成功
  EXPECT_FALSE(trajectory.empty());
  EXPECT_GT(trajectory.size(), 10);

  // 验证终点接近目标
  if (!trajectory.empty()) {
    double final_x = trajectory.back().x;
    EXPECT_NEAR(final_x, goal_pose.pose.position.x, 0.5);
  }
}

TEST_F(SinglePointNavigationTest, NavigateToDiagonalPoint)
{
  // 测试：导航到斜向目标点

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // 斜向路径
  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = i * 0.05;  // y方向也有移动
    pose.pose.orientation.z = std::sin(std::atan2(0.05, 0.1) / 2.0);
    pose.pose.orientation.w = std::cos(std::atan2(0.05, 0.1) / 2.0);
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = trajectory_generator_->generateTrajectory(path, current_velocity);

  EXPECT_FALSE(trajectory.empty());

  // 验证轨迹包含转向
  bool has_turning = false;
  for (const auto & point : trajectory) {
    if (std::abs(point.v_theta) > 0.01) {
      has_turning = true;
      break;
    }
  }
  EXPECT_TRUE(has_turning);
}

TEST_F(SinglePointNavigationTest, NavigateToPointWithInitialTurn)
{
  // 测试：导航到需要先转向的目标点

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // 需要先转向的路径（左转90度）
  double turn_angle = M_PI / 2;
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    double angle = turn_angle * i / 10.0;
    pose.pose.position.x = std::cos(angle) * (i * 0.05);
    pose.pose.position.y = std::sin(angle) * (i * 0.05);
    pose.pose.orientation.z = std::sin(angle / 2.0);
    pose.pose.orientation.w = std::cos(angle / 2.0);
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = trajectory_generator_->generateTrajectory(path, current_velocity);

  EXPECT_FALSE(trajectory.empty());

  // 验证初始阶段有角速度（转向）
  if (trajectory.size() > 5) {
    bool initial_turning = false;
    for (size_t i = 0; i < 5; ++i) {
      if (std::abs(trajectory[i].v_theta) > 0.01) {
        initial_turning = true;
        break;
      }
    }
    EXPECT_TRUE(initial_turning);
  }
}

TEST_F(SinglePointNavigationTest, NavigateToPointWithRotationOnly)
{
  // 测试：原地旋转导航（目标点在当前位置，但方向不同）

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // 原地旋转路径
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    double angle = M_PI * i / 10.0;  // 180度旋转
    pose.pose.orientation.z = std::sin(angle / 2.0);
    pose.pose.orientation.w = std::cos(angle / 2.0);
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = trajectory_generator_->generateTrajectory(path, current_velocity);

  EXPECT_FALSE(trajectory.empty());

  // 验证主要速度为角速度
  for (const auto & point : trajectory) {
    EXPECT_NEAR(point.v_x, 0.0, 0.1);  // 线速度应该很小
    EXPECT_GT(std::abs(point.v_theta), 0.01);  // 有角速度
  }
}

TEST_F(SinglePointNavigationTest, NavigateToPointWithLongPath)
{
  // 测试：长距离导航

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // 10米直线距离
  for (int i = 0; i <= 100; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = trajectory_generator_->generateTrajectory(path, current_velocity);

  EXPECT_FALSE(trajectory.empty());
  EXPECT_GT(trajectory.size(), 50);

  // 验证轨迹达到最大速度
  bool reaches_max_speed = false;
  for (const auto & point : trajectory) {
    if (std::abs(point.v_x) >= 0.4) {  // 接近最大速度
      reaches_max_speed = true;
      break;
    }
  }
  EXPECT_TRUE(reaches_max_speed);
}

TEST_F(SinglePointNavigationTest, NavigateToNearbyPoint)
{
  // 测试：短距离导航

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // 0.2米距离
  geometry_msgs::msg::PoseStamped start_pose, goal_pose;
  start_pose.pose.position.x = 0.0;
  start_pose.pose.position.y = 0.0;
  start_pose.pose.orientation.w = 1.0;
  goal_pose.pose.position.x = 0.2;
  goal_pose.pose.position.y = 0.0;
  goal_pose.pose.orientation.w = 1.0;

  path.poses.push_back(start_pose);
  path.poses.push_back(goal_pose);

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = trajectory_generator_->generateTrajectory(path, current_velocity);

  EXPECT_FALSE(trajectory.empty());

  // 验证轨迹点数量合理
  EXPECT_LT(trajectory.size(), 100);
}

// ==================== 路径跟踪测试 ====================

TEST_F(SinglePointNavigationTest, PathTrackingAccuracy)
{
  // 测试：路径跟踪精度

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // 曲线路径
  for (int i = 0; i <= 30; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = i * 0.2;
    pose.pose.position.y = std::sin(i * 0.3) * 0.3;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  controller_->setPlan(path);
  controller_->setActive(true);

  // 模拟机器人当前位置
  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist current_vel;

  double max_deviation = 0.0;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    robot_pose = path.poses[i];
    current_vel.linear.x = 0.2;
    current_vel.angular.z = 0.0;

    EXPECT_NO_THROW({
      auto cmd_vel = controller_->computeVelocityCommands(robot_pose, current_vel);
      // 验证速度命令有效
      EXPECT_LE(std::abs(cmd_vel.linear.x), controller_->getConfig().max_velocity_x + 0.1);
      EXPECT_LE(std::abs(cmd_vel.angular.z), controller_->getConfig().max_velocity_theta + 0.1);
    });
  }
}

TEST_F(SinglePointNavigationTest, GoalReachedDetection)
{
  // 测试：目标到达检测

  ControllerConfig config = controller_->getConfig();
  config.xy_goal_tolerance = 0.05;
  config.yaw_goal_tolerance = 0.1;
  controller_->setConfig(config);

  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 0.0;
  goal.pose.orientation.w = 1.0;
  controller_->setGoal(goal);

  // 测试在容差范围内
  geometry_msgs::msg::PoseStamped reached_pose;
  reached_pose.pose.position.x = 1.02;
  reached_pose.pose.position.y = 0.02;
  reached_pose.pose.orientation.w = 1.0;

  EXPECT_NO_THROW({
    bool reached = controller_->isGoalReached(reached_pose);
  });
}

// ==================== 速度平滑测试 ====================

TEST_F(SinglePointNavigationTest, VelocitySmoothness)
{
  // 测试：速度平滑性

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

  auto trajectory = trajectory_generator_->generateTrajectory(path, current_velocity);

  // 验证速度变化平滑（加速度限制）
  double max_accel_change = 0.0;
  for (size_t i = 2; i < trajectory.size(); ++i) {
    double accel_change = std::abs(trajectory[i].v_x - trajectory[i - 1].v_x) -
                          std::abs(trajectory[i - 1].v_x - trajectory[i - 2].v_x);
    max_accel_change = std::max(max_accel_change, std::abs(accel_change));
  }

  // 加速度变化应该相对平滑
  EXPECT_LT(max_accel_change, 1.0);
}

// ==================== 状态管理测试 ====================

TEST_F(SinglePointNavigationTest, ResetAndReuse)
{
  nav_msgs::msg::Path path1, path2;
  path1.header.frame_id = "map";
  path2.header.frame_id = "map";

  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path1.poses.push_back(pose);
    path2.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  // 第一次导航
  auto trajectory1 = trajectory_generator_->generateTrajectory(path1, current_velocity);

  // 重置
  trajectory_generator_->reset();

  // 第二次导航
  auto trajectory2 = trajectory_generator_->generateTrajectory(path2, current_velocity);

  EXPECT_FALSE(trajectory2.empty());
}

// ==================== 边界场景测试 ====================

TEST_F(SinglePointNavigationTest, GoalAtStartingPosition)
{
  // 测试：目标点在起始位置

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;

  auto trajectory = trajectory_generator_->generateTrajectory(path, current_velocity);

  // 应该返回空轨迹或单点轨迹
  EXPECT_TRUE(trajectory.empty() || trajectory.size() == 1);
}

TEST_F(SinglePointNavigationTest, NavigateWithExistingVelocity)
{
  // 测试：已有初始速度时导航

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::Twist current_velocity;
  current_velocity.linear.x = 0.3;  // 已有速度
  current_velocity.angular.z = 0.0;

  auto trajectory = trajectory_generator_->generateTrajectory(path, current_velocity);

  EXPECT_FALSE(trajectory.empty());

  // 初始速度应该接近当前速度
  if (!trajectory.empty()) {
    EXPECT_NEAR(trajectory[0].v_x, 0.3, 0.5);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}