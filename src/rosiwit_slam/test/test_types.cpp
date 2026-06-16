/**
 * @file test_types.cpp
 * @brief 单元测试 - 核心类型定义
 * @author AI Development Team - Test Engineer
 * @date 2026-04-24
 */

#include <gtest/gtest.h>
#include "fast_lio2_slam/common/types.h"
#include <cmath>

namespace fast_lio2_slam {
namespace test {

class TypesTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 测试前初始化
    }
};

// ==================== State 结构体测试 ====================

TEST_F(TypesTest, State_DefaultInitialization) {
    State state;

    // 验证默认值
    EXPECT_NEAR(state.position.x(), 0.0, 1e-9);
    EXPECT_NEAR(state.position.y(), 0.0, 1e-9);
    EXPECT_NEAR(state.position.z(), 0.0, 1e-9);

    EXPECT_NEAR(state.velocity.x(), 0.0, 1e-9);
    EXPECT_NEAR(state.velocity.y(), 0.0, 1e-9);
    EXPECT_NEAR(state.velocity.z(), 0.0, 1e-9);

    EXPECT_NEAR(state.acc_bias.x(), 0.0, 1e-9);
    EXPECT_NEAR(state.gyro_bias.x(), 0.0, 1e-9);

    // 重力默认值
    EXPECT_NEAR(state.gravity.z(), -9.81, 1e-6);
}

TEST_F(TypesTest, State_ToSE3) {
    State state;
    state.position = Vector3d(1.0, 2.0, 3.0);
    state.rotation = Quaterniond(Eigen::AngleAxisd(M_PI/4, Vector3d::UnitZ()));

    SE3d se3 = state.toSE3();

    EXPECT_NEAR(se3.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(se3.translation().y(), 2.0, 1e-9);
    EXPECT_NEAR(se3.translation().z(), 3.0, 1e-9);
}

TEST_F(TypesTest, State_ToMatrix) {
    State state;
    state.position = Vector3d(1.0, 2.0, 3.0);
    state.rotation = Quaterniond::Identity();

    Matrix4d T = state.toMatrix();

    // 验证齐次矩阵
    EXPECT_NEAR(T(0, 3), 1.0, 1e-9);
    EXPECT_NEAR(T(1, 3), 2.0, 1e-9);
    EXPECT_NEAR(T(2, 3), 3.0, 1e-9);
    EXPECT_NEAR(T(3, 3), 1.0, 1e-9);

    // 旋转矩阵应为单位阵
    EXPECT_NEAR(T(0, 0), 1.0, 1e-9);
    EXPECT_NEAR(T(1, 1), 1.0, 1e-9);
    EXPECT_NEAR(T(2, 2), 1.0, 1e-9);
}

TEST_F(TypesTest, State_FromSE3) {
    SE3d se3 = SE3d(SO3d(), Vector3d(10.0, 20.0, 30.0));

    State state;
    state.fromSE3(se3);

    EXPECT_NEAR(state.position.x(), 10.0, 1e-9);
    EXPECT_NEAR(state.position.y(), 20.0, 1e-9);
    EXPECT_NEAR(state.position.z(), 30.0, 1e-9);
}

// ==================== ImuData 结构体测试 ====================

TEST_F(TypesTest, ImuData_DefaultInitialization) {
    ImuData imu;

    EXPECT_NEAR(imu.timestamp, 0.0, 1e-9);
    EXPECT_NEAR(imu.acc.x(), 0.0, 1e-9);
    EXPECT_NEAR(imu.gyro.x(), 0.0, 1e-9);
}

TEST_F(TypesTest, ImuData_InitializedValues) {
    ImuData imu(1.0, Vector3d(0.1, 0.2, 9.8), Vector3d(0.01, 0.02, 0.03));

    EXPECT_NEAR(imu.timestamp, 1.0, 1e-9);
    EXPECT_NEAR(imu.acc.x(), 0.1, 1e-9);
    EXPECT_NEAR(imu.acc.y(), 0.2, 1e-9);
    EXPECT_NEAR(imu.acc.z(), 9.8, 1e-9);
}

// ==================== ImuBuffer 测试 ====================

TEST_F(TypesTest, ImuBuffer_PushAndSize) {
    ImuBuffer buffer(100);

    EXPECT_EQ(buffer.size(), 0);

    ImuData imu1(0.1, Vector3d::Zero(), Vector3d::Zero());
    ImuData imu2(0.2, Vector3d::Zero(), Vector3d::Zero());

    buffer.push(imu1);
    buffer.push(imu2);

    EXPECT_EQ(buffer.size(), 2);
}

TEST_F(TypesTest, ImuBuffer_Clear) {
    ImuBuffer buffer(100);

    buffer.push(ImuData(0.1, Vector3d::Zero(), Vector3d::Zero()));
    buffer.push(ImuData(0.2, Vector3d::Zero(), Vector3d::Zero()));

    EXPECT_EQ(buffer.size(), 2);

    buffer.clear();

    EXPECT_EQ(buffer.size(), 0);
}

TEST_F(TypesTest, ImuBuffer_GetImuInRange) {
    ImuBuffer buffer(100);

    // 添加5个IMU数据
    for (int i = 0; i < 5; ++i) {
        buffer.push(ImuData(i * 0.1, Vector3d(i, 0, 0), Vector3d::Zero()));
    }

    // 获取时间范围 [0.15, 0.35] 内的数据
    auto imus = buffer.getImuInRange(0.15, 0.35);

    EXPECT_GE(imus.size(), 2);
}

TEST_F(TypesTest, ImuBuffer_MaxSize) {
    ImuBuffer buffer(3);  // 最大容量为3

    for (int i = 0; i < 5; ++i) {
        buffer.push(ImuData(i * 0.1, Vector3d::Zero(), Vector3d::Zero()));
    }

    // 由于容量限制，应该只保留最新的3个
    EXPECT_LE(buffer.size(), 3);
}

// ==================== LidarParams 测试 ====================

TEST_F(TypesTest, LidarParams_DefaultValues) {
    LidarParams params;

    EXPECT_NEAR(params.voxel_size, 0.2, 1e-6);
    EXPECT_NEAR(params.min_range, 0.5, 1e-6);
    EXPECT_NEAR(params.max_range, 100.0, 1e-6);
    EXPECT_EQ(params.scan_line, 16);
}

// ==================== IekfParams 测试 ====================

TEST_F(TypesTest, IekfParams_DefaultValues) {
    IekfParams params;

    EXPECT_EQ(params.max_iterations, 5);
    EXPECT_NEAR(params.converge_threshold, 0.001, 1e-6);
    EXPECT_NEAR(params.position_noise, 0.01, 1e-6);
}

// ==================== 数学工具测试 ====================

TEST_F(TypesTest, RotationMatrixFromEuler) {
    Vector3d euler(0.1, 0.2, 0.3);  // roll, pitch, yaw

    Matrix3d R = Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                 Eigen::AngleAxisd(euler[0], Vector3d::UnitX()).toRotationMatrix();

    // 验证旋转矩阵的正交性
    Matrix3d Rt = R.transpose();
    Matrix3d I = R * Rt;

    EXPECT_NEAR(I(0, 0), 1.0, 1e-9);
    EXPECT_NEAR(I(1, 1), 1.0, 1e-9);
    EXPECT_NEAR(I(2, 2), 1.0, 1e-9);
}

TEST_F(TypesTest, QuaternionNormalization) {
    Quaterniond q(0.5, 0.5, 0.5, 0.5);
    q.normalize();

    EXPECT_NEAR(q.norm(), 1.0, 1e-9);
}

} // namespace test
} // namespace fast_lio2_slam

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}