// ============================================================
// 轨迹生成器边界条件增强测试
// 对应需求: requirements.md § 空路径/单点路径边界处理
// 对应架构: architecture.md § 3.3 TrajectoryGenerator增强
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include "diffbot_navigation/navigation/trajectory_generator.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace diffbot_navigation::navigation;

// ============================================================
// Fixture: 轨迹生成器测试环境
// ============================================================
class TrajectoryGeneratorEdgeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    generator_ = std::make_unique<TrajectoryGenerator>();

    // 标准配置
    TrajectoryGenerator::Config config;
    config.max_linear_velocity = 1.0;
    config.max_angular_velocity = 1.5;
    config.max_linear_accel = 0.5;
    config.max_angular_accel = 1.0;
    config.dt = 0.05;
    config.xy_goal_tolerance = 0.1;
    config.yaw_goal_tolerance = 0.1;
    config.min_lookahead_distance = 0.2;
    config.max_lookahead_distance = 1.5;
    generator_->configure(config);

    // 默认速度
    default_velocity_.linear.x = 0.0;
    default_velocity_.angular.z = 0.0;
  }

  std::unique_ptr<TrajectoryGenerator> generator_;
  geometry_msgs::msg::Twist default_velocity_;
};

// ============================================================
// 辅助函数：创建测试路径
// ============================================================
nav_msgs::msg::Path createPath(const std::vector<std::pair<double, double>>& points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp.sec = 0;
  path.header.stamp.nanosec = 0;

  for (const auto& [x, y] : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;  // 无旋转
    path.poses.push_back(pose);
  }
  return path;
}

// ============================================================
// TC-TG-01: 空路径处理（核心修复用例）
// 期望：generateTrajectory 不应崩溃，应返回空轨迹或抛出明确异常
// ============================================================
TEST_F(TrajectoryGeneratorEdgeTest, HandleEmptyPath)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = "map";

  // 不应崩溃
  std::vector<diffbot_navigation::core::TrajectoryPoint> trajectory;

  EXPECT_NO_THROW({
    trajectory = generator_->generateTrajectory(empty_path, default_velocity_);
  });

  // 空路径应返回空轨迹
  EXPECT_TRUE(trajectory.empty())
    << "Empty path should produce empty trajectory, got " << trajectory.size() << " points";
}

// ============================================================
// TC-TG-02: 单点路径处理（核心修复用例）
// 期望：单个路径点应返回至少包含起点的轨迹（机器人已在目标点）
// ============================================================
TEST_F(TrajectoryGeneratorEdgeTest, HandleSinglePointPath)
{
  auto path = createPath({{1.0, 2.0}});  // 仅一个点

  std::vector<diffbot_navigation::core::TrajectoryPoint> trajectory;

  EXPECT_NO_THROW({
    trajectory = generator_->generateTrajectory(path, default_velocity_);
  });

  // 单点路径：轨迹应至少包含1个点（当前位姿/目标位姿）
  EXPECT_GE(trajectory.size(), 1u)
    << "Single point path should produce at least 1 trajectory point";

  // 轨迹终点应接近（或等于）路径唯一目标点
  if (!trajectory.empty()) {
    double dist = std::sqrt(
      std::pow(trajectory.back().x - path.poses.back().pose.position.x, 2) +
      std::pow(trajectory.back().y - path.poses.back().pose.position.y, 2)
    );
    EXPECT_LE(dist, 0.5) << "Trajectory end should be near the single path point";
  }
}

// ============================================================
// TC-TG-03: 路径点数=0 (empty但frame_id已设置)
// ============================================================
TEST_F(TrajectoryGeneratorEdgeTest, PathWithZeroPointsButValidHeader)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp.sec = 12345;

  // 明确清空poses
  path.poses.clear();
  ASSERT_EQ(path.poses.size(), 0u);

  auto trajectory = generator_->generateTrajectory(path, default_velocity_);
  EXPECT_TRUE(trajectory.empty())
    << "Path with zero pose points must return empty trajectory";
}

// ============================================================
// TC-TG-04: 两个相同点路径（起点==终点）
// 期望：应正确处理零长度路径而不崩溃
// ============================================================
TEST_F(TrajectoryGeneratorEdgeTest, DuplicateStartGoalPath)
{
  auto path = createPath({{0.0, 0.0}, {0.0, 0.0}});

  std::vector<diffbot_navigation::core::TrajectoryPoint> trajectory;
  EXPECT_NO_THROW({
    trajectory = generator_->generateTrajectory(path, default_velocity_);
  });

  // 应生成轨迹（至少包含当前位姿）
  EXPECT_GE(trajectory.size(), 1u);
}

// ============================================================
// TC-TG-05: 极大路径（1000+姿态点）性能与稳定性
// ============================================================
TEST_F(TrajectoryGeneratorEdgeTest, VeryLongPathHandling)
{
  std::vector<std::pair<double, double>> points;
  for (int i = 0; i <= 1000; ++i) {
    points.emplace_back(i * 0.1, std::sin(i * 0.01) * 5.0);
  }

  auto path = createPath(points);
  ASSERT_EQ(path.poses.size(), 1001u);

  // 应能在合理时间内完成
  auto start = std::chrono::steady_clock::now();

  std::vector<diffbot_navigation::core::TrajectoryPoint> trajectory;
  EXPECT_NO_THROW({
    trajectory = generator_->generateTrajectory(path, default_velocity_);
  });

  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count();

  // 1000点路径的处理应在500ms内完成
  EXPECT_LT(elapsed, 500)
    << "Very long path (1001 pts) took " << elapsed << "ms, expected < 500ms";

  // 应生成足够密集的轨迹点
  EXPECT_GT(trajectory.size(), 10u)
    << "Very long path should generate substantial trajectory";
}

// ============================================================
// TC-TG-06: 非零初始速度下的启动行为
// ============================================================
TEST_F(TrajectoryGeneratorEdgeTest, NonZeroInitialVelocity)
{
  auto path = createPath({{0.0, 0.0}, {5.0, 0.0}});
  geometry_msgs::msg::Twist moving_velocity;
  moving_velocity.linear.x = 0.5;   // 已有前进速度
  moving_velocity.angular.z = -0.1; // 微小旋转

  std::vector<diffbot_navigation::core::TrajectoryPoint> trajectory;
  EXPECT_NO_THROW({
    trajectory = generator_->generateTrajectory(path, moving_velocity);
  });

  EXPECT_GT(trajectory.size(), 1u);

  // 第一个轨迹点应反映当前速度状态
  if (trajectory.size() >= 2) {
    EXPECT_GT(trajectory[0].vx, 0.0)
      << "First trajectory point should consider initial velocity";
  }
}

// ============================================================
// TC-TG-07: 路径包含NaN坐标的容错处理
// ============================================================
TEST_F(TrajectoryGeneratorEdgeTest, PathWithNaNCoordinates)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // 合法点
  auto valid_pose = geometry_msgs::msg::PoseStamped();
  valid_pose.pose.position.x = 0.0;
  valid_pose.pose.position.y = 0.0;
  valid_pose.pose.orientation.w = 1.0;
  path.poses.push_back(valid_pose);

  // NaN点
  auto nan_pose = geometry_msgs::msg::PoseStamped();
  nan_pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  nan_pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  nan_pose.pose.orientation.w = 1.0;
  path.poses.push_back(nan_pose);

  // 不应崩溃（至少返回部分结果）
  std::vector<diffbot_navigation::core::TrajectoryPoint> trajectory;
  EXPECT_NO_THROW({
    trajectory = generator_->generateTrajectory(path, default_velocity_);
  });

  // 任何轨迹点都不应包含NaN
  for (const auto& pt : trajectory) {
    EXPECT_FALSE(std::isnan(pt.x)) << "Trajectory point x should not be NaN";
    EXPECT_FALSE(std::isnan(pt.y)) << "Trajectory point y should not be NaN";
  }
}

// ============================================================
// TC-TG-08: 极小路径（两点距离 < 1e-6）的处理
// ============================================================
TEST_F(TrajectoryGeneratorEdgeTest, MicroscopicPathHandling)
{
  auto path = createPath({{0.0, 0.0}, {1e-7, 1e-7}});
  ASSERT_EQ(path.poses.size(), 2u);

  std::vector<diffbot_navigation::core::TrajectoryPoint> trajectory;
  EXPECT_NO_THROW({
    trajectory = generator_->generateTrajectory(path, default_velocity_);
  });

  EXPECT_GE(trajectory.size(), 1u);
}

// ============================================================
// TC-TG-09: 超最大速度的初始速度
// ============================================================
TEST_F(TrajectoryGeneratorEdgeTest, InitialVelocityExceedsMax)
{
  auto path = createPath({{0.0, 0.0}, {5.0, 0.0}});
  geometry_msgs::msg::Twist excessive_velocity;
  excessive_velocity.linear.x = 10.0;    // 远超 max_linear_velocity=1.0
  excessive_velocity.angular.z = 5.0;    // 远超 max_angular_velocity=1.5

  std::vector<diffbot_navigation::core::TrajectoryPoint> trajectory;
  EXPECT_NO_THROW({
    trajectory = generator_->generateTrajectory(path, excessive_velocity);
  });

  // 第一个轨迹点速度应被限制到合法范围
  if (!trajectory.empty()) {
    EXPECT_LE(trajectory[0].vx, 1.0 + 0.01)
      << "Initial trajectory velocity should be clamped to max";
  }
}

// ============================================================
// TC-TG-10: header未初始化的路径
// ============================================================
TEST_F(TrajectoryGeneratorEdgeTest, UninitializedPathHeader)
{
  nav_msgs::msg::Path path;  // 默认构造, header 未显式设置
  path.header.frame_id = ""; // 空 frame_id

  auto pose = geometry_msgs::msg::PoseStamped();
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);

  // 不应崩溃
  EXPECT_NO_THROW({
    auto trajectory = generator_->generateTrajectory(path, default_velocity_);
  });
}
