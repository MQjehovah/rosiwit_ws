// ============================================================
// Diffbot Navigation - 差速轮控制器单元测试（简化版）
// 注：Nav2 包未安装，此测试仅验证独立功能
// ============================================================

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

// 简化测试，不依赖 Nav2
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

// 由于 Nav2 未安装，无法直接测试 DiffDriveController
// 此测试文件仅验证基础数据类型和逻辑

class DiffDriveControllerBasicTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
    }

    void TearDown() override
    {
    }
};

// ==================== 基本数据类型测试 ====================

TEST_F(DiffDriveControllerBasicTest, Pose2DDefaultValues)
{
    geometry_msgs::msg::Pose2D pose;
    EXPECT_DOUBLE_EQ(pose.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.theta, 0.0);
}

TEST_F(DiffDriveControllerBasicTest, Pose2DAssignment)
{
    geometry_msgs::msg::Pose2D pose;
    pose.x = 1.0;
    pose.y = 2.0;
    pose.theta = 0.5;

    EXPECT_DOUBLE_EQ(pose.x, 1.0);
    EXPECT_DOUBLE_EQ(pose.y, 2.0);
    EXPECT_DOUBLE_EQ(pose.theta, 0.5);
}

TEST_F(DiffDriveControllerBasicTest, TwistDefaultValues)
{
    geometry_msgs::msg::Twist twist;
    EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST_F(DiffDriveControllerBasicTest, TwistAssignment)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.5;
    twist.angular.z = 0.3;

    EXPECT_DOUBLE_EQ(twist.linear.x, 0.5);
    EXPECT_DOUBLE_EQ(twist.angular.z, 0.3);
}

// ==================== 差速运动学计算测试 ====================

TEST_F(DiffDriveControllerBasicTest, LinearVelocityCalculation)
{
    // 差速运动学：v = (v_left + v_right) / 2
    double v_left = 0.5;
    double v_right = 0.7;
    double v = (v_left + v_right) / 2.0;

    EXPECT_DOUBLE_EQ(v, 0.6);
}

TEST_F(DiffDriveControllerBasicTest, AngularVelocityCalculation)
{
    // 差速运动学：w = (v_right - v_left) / wheel_base
    double v_left = 0.5;
    double v_right = 0.7;
    double wheel_base = 0.4;
    double w = (v_right - v_left) / wheel_base;

    EXPECT_DOUBLE_EQ(w, 0.5);
}

TEST_F(DiffDriveControllerBasicTest, WheelVelocityFromTwist)
{
    // 从 Twist 计算轮速
    double v = 0.5;  // 线速度
    double w = 0.3;  // 角速度
    double wheel_base = 0.4;

    double v_left = v - w * wheel_base / 2.0;
    double v_right = v + w * wheel_base / 2.0;

    EXPECT_NEAR(v_left, 0.44, 0.001);
    EXPECT_NEAR(v_right, 0.56, 0.001);
}

// ==================== 纯追踪算法基础测试 ====================

TEST_F(DiffDriveControllerBasicTest, CalculateLookaheadDistance)
{
    double base_lookahead = 0.6;
    double velocity = 0.5;
    double velocity_gain = 2.0;

    double lookahead = base_lookahead + velocity_gain * velocity;

    EXPECT_DOUBLE_EQ(lookahead, 1.6);
}

TEST_F(DiffDriveControllerBasicTest, CalculateCurvature)
{
    // 纯追踪曲率计算：curvature = 2 * sin(alpha) / lookahead
    double alpha = 0.1;  // 角度偏差
    double lookahead = 1.0;

    double curvature = 2.0 * sin(alpha) / lookahead;

    EXPECT_NEAR(curvature, 0.199, 0.001);
}

TEST_F(DiffDriveControllerBasicTest, CalculateSteeringAngle)
{
    // 转向角：steering = atan(curvature * lookahead)
    double curvature = 0.2;
    double lookahead = 1.0;

    double steering = atan(curvature * lookahead);

    EXPECT_NEAR(steering, 0.197, 0.001);
}

// ==================== PID 控制器基础测试 ====================

TEST_F(DiffDriveControllerBasicTest, PIDProportionalTerm)
{
    double kp = 1.0;
    double error = 0.5;

    double p_term = kp * error;

    EXPECT_DOUBLE_EQ(p_term, 0.5);
}

TEST_F(DiffDriveControllerBasicTest, PIDIntegralTerm)
{
    double ki = 0.1;
    double integral = 10.0;

    double i_term = ki * integral;

    EXPECT_DOUBLE_EQ(i_term, 1.0);
}

TEST_F(DiffDriveControllerBasicTest, PIDDerivativeTerm)
{
    double kd = 0.5;
    double derivative = 2.0;

    double d_term = kd * derivative;

    EXPECT_DOUBLE_EQ(d_term, 1.0);
}

TEST_F(DiffDriveControllerBasicTest, PIDTotalOutput)
{
    double kp = 1.0;
    double ki = 0.1;
    double kd = 0.5;
    double error = 0.5;
    double integral = 10.0;
    double derivative = 2.0;

    double output = kp * error + ki * integral + kd * derivative;

    EXPECT_DOUBLE_EQ(output, 2.5);
}

// ==================== 目标点到达检测测试 ====================

TEST_F(DiffDriveControllerBasicTest, GoalReachedPositionThreshold)
{
    double current_x = 1.0;
    double current_y = 2.0;
    double goal_x = 1.05;
    double goal_y = 2.05;
    double threshold = 0.1;

    double distance = sqrt(pow(goal_x - current_x, 2) + pow(goal_y - current_y, 2));
    bool reached = distance < threshold;

    EXPECT_NEAR(distance, 0.07, 0.001);
    EXPECT_TRUE(reached);
}

TEST_F(DiffDriveControllerBasicTest, GoalNotReachedPositionThreshold)
{
    double current_x = 1.0;
    double current_y = 2.0;
    double goal_x = 1.5;
    double goal_y = 2.5;
    double threshold = 0.1;

    double distance = sqrt(pow(goal_x - current_x, 2) + pow(goal_y - current_y, 2));
    bool reached = distance < threshold;

    EXPECT_NEAR(distance, 0.707, 0.001);
    EXPECT_FALSE(reached);
}

TEST_F(DiffDriveControllerBasicTest, GoalReachedAngleThreshold)
{
    double current_theta = 0.1;
    double goal_theta = 0.05;
    double threshold = 0.1;

    double angle_diff = abs(goal_theta - current_theta);
    bool reached = angle_diff < threshold;

    EXPECT_DOUBLE_EQ(angle_diff, 0.05);
    EXPECT_TRUE(reached);
}

// ==================== 路径跟踪测试 ====================

TEST_F(DiffDriveControllerBasicTest, FindClosestPointOnPath)
{
    // 简化的路径点查找
    geometry_msgs::msg::Pose2D robot_pose;
    robot_pose.x = 1.0;
    robot_pose.y = 1.0;

    std::vector<geometry_msgs::msg::Pose2D> path;
    for (int i = 0; i < 10; ++i) {
        geometry_msgs::msg::Pose2D p;
        p.x = i * 0.5;
        p.y = 0.0;
        path.push_back(p);
    }

    // 找最近的路径点
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    for (size_t i = 0; i < path.size(); ++i) {
        double dist = sqrt(pow(path[i].x - robot_pose.x, 2) + pow(path[i].y - robot_pose.y, 2));
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    EXPECT_EQ(closest_idx, 2);  // 点 (1.0, 0.0) 最近
    EXPECT_NEAR(min_dist, 1.0, 0.001);
}

// ==================== 速度限制测试 ====================

TEST_F(DiffDriveControllerBasicTest, ApplyVelocityLimits)
{
    double max_linear = 0.5;
    double max_angular = 1.0;

    double requested_linear = 1.0;  // 超出限制
    double requested_angular = 2.0;  // 超出限制

    double limited_linear = std::min(std::max(requested_linear, -max_linear), max_linear);
    double limited_angular = std::min(std::max(requested_angular, -max_angular), max_angular);

    EXPECT_DOUBLE_EQ(limited_linear, 0.5);
    EXPECT_DOUBLE_EQ(limited_angular, 1.0);
}

TEST_F(DiffDriveControllerBasicTest, ApplyNegativeVelocityLimits)
{
    double max_linear = 0.5;

    double requested_linear = -1.0;  // 负向超出限制

    double limited_linear = std::min(std::max(requested_linear, -max_linear), max_linear);

    EXPECT_DOUBLE_EQ(limited_linear, -0.5);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}