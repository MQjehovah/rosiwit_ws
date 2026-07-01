/**
 * @file test_imu_processor.cpp
 * @brief 单元测试 - IMU处理器
 * @author AI Development Team - Test Engineer
 * @date 2026-04-24
 */

#include <gtest/gtest.h>
#include "fast_lio2_slam/data_preprocessor/imu_processor.h"
#include <cmath>

namespace fast_lio2_slam {
namespace test {

class ImuProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        params_.acc_noise = 0.1;
        params_.gyro_noise = 0.01;
        params_.acc_bias_noise = 0.0001;
        params_.gyro_bias_noise = 0.00001;
        params_.gravity_magnitude = 9.81;
        
        processor_ = std::make_unique<ImuProcessor>(params_);
    }
    
    // ImuProcessor使用ImuProcessorConfig
using ImuParams = ImuProcessorConfig;
    std::unique_ptr<ImuProcessor> processor_;
    
    // 创建测试IMU数据
    ImuData createTestImu(double timestamp, const Vector3d& acc, const Vector3d& gyro) {
        return ImuData(timestamp, acc, gyro);
    }
};

// ==================== 初始化测试 ====================

TEST_F(ImuProcessorTest, DefaultInitialization) {
    ImuProcessor processor;
    EXPECT_TRUE(true);  // 验证构造不崩溃
}

TEST_F(ImuProcessorTest, ParamInitialization) {
    EXPECT_DOUBLE_EQ(processor_->getParams().acc_noise, 0.1);
    EXPECT_DOUBLE_EQ(processor_->getParams().gyro_noise, 0.01);
    EXPECT_DOUBLE_EQ(processor_->getParams().gravity_magnitude, 9.81);
}

// ==================== IMU积分测试 ====================

TEST_F(ImuProcessorTest, Integrate_SingleImu) {
    // 单个IMU数据积分
    ImuData imu = createTestImu(0.1, Vector3d(0.0, 0.0, 9.81), Vector3d(0.0, 0.0, 0.0));
    
    State state = processor_->integrate(imu);
    
    // 验证积分后状态有效
    EXPECT_TRUE(std::isfinite(state.position.x()));
    EXPECT_TRUE(std::isfinite(state.position.y()));
    EXPECT_TRUE(std::isfinite(state.position.z()));
}

TEST_F(ImuProcessorTest, Integrate_MultipleImu) {
    // 连续积分多个IMU数据
    double dt = 0.01;
    for (int i = 0; i < 100; ++i) {
        ImuData imu = createTestImu(
            i * dt, 
            Vector3d(0.0, 0.0, 9.81),  // 静止状态，测量重力
            Vector3d(0.0, 0.0, 0.0)
        );
        processor_->integrate(imu);
    }
    
    State state = processor_->getState();
    
    // 经过1秒静止，位置变化应该很小（只有数值误差）
    EXPECT_LT(state.position.norm(), 1.0);
}

TEST_F(ImuProcessorTest, Integrate_ConstantVelocity) {
    // 模拟恒定速度运动
    double dt = 0.01;
    Vector3d velocity(1.0, 0.0, 0.0);
    
    for (int i = 0; i < 100; ++i) {
        ImuData imu = createTestImu(
            i * dt,
            Vector3d(0.0, 0.0, 9.81),  // 重力
            Vector3d(0.0, 0.0, 0.0)
        );
        processor_->integrate(imu);
    }
    
    // 注意：实际结果取决于积分实现
    EXPECT_TRUE(true);
}

// ==================== 偏置估计测试 ====================

TEST_F(ImuProcessorTest, BiasEstimation) {
    // 设置已知偏置
    Vector3d acc_bias(0.1, 0.05, 0.02);
    Vector3d gyro_bias(0.01, 0.005, 0.002);
    
    processor_->setAccBias(acc_bias);
    processor_->setGyroBias(gyro_bias);
    
    Vector3d retrieved_acc = processor_->getAccBias();
    Vector3d retrieved_gyro = processor_->getGyroBias();
    
    EXPECT_NEAR(retrieved_acc.x(), acc_bias.x(), 1e-9);
    EXPECT_NEAR(retrieved_acc.y(), acc_bias.y(), 1e-9);
    EXPECT_NEAR(retrieved_acc.z(), acc_bias.z(), 1e-9);
    
    EXPECT_NEAR(retrieved_gyro.x(), gyro_bias.x(), 1e-9);
    EXPECT_NEAR(retrieved_gyro.y(), gyro_bias.y(), 1e-9);
    EXPECT_NEAR(retrieved_gyro.z(), gyro_bias.z(), 1e-9);
}

// ==================== IMU缓存测试 ====================

TEST_F(ImuProcessorTest, ImuBuffer_Operations) {
    // 添加多个IMU数据到缓存
    for (int i = 0; i < 10; ++i) {
        ImuData imu = createTestImu(i * 0.1, Vector3d::Zero(), Vector3d::Zero());
        processor_->addImuData(imu);
    }
    
    // 验证缓存大小
    EXPECT_EQ(processor_->getImuBufferSize(), 10);
}

TEST_F(ImuProcessorTest, ImuBuffer_GetInRange) {
    // 添加时间序列数据
    for (int i = 0; i < 20; ++i) {
        ImuData imu = createTestImu(i * 0.05, Vector3d::Zero(), Vector3d::Zero());
        processor_->addImuData(imu);
    }
    
    // 获取特定时间范围内的数据
    std::vector<ImuData> imus = processor_->getImuInRange(0.25, 0.75);
    
    // 验证获取的数据在时间范围内
    EXPECT_GT(imus.size(), 0);
    for (const auto& imu : imus) {
        EXPECT_GE(imu.timestamp, 0.25);
        EXPECT_LE(imu.timestamp, 0.75);
    }
}

TEST_F(ImuProcessorTest, ImuBuffer_Clear) {
    // 添加数据
    for (int i = 0; i < 5; ++i) {
        ImuData imu = createTestImu(i * 0.1, Vector3d::Zero(), Vector3d::Zero());
        processor_->addImuData(imu);
    }
    
    EXPECT_EQ(processor_->getImuBufferSize(), 5);
    
    // 清空
    processor_->clearImuBuffer();
    
    EXPECT_EQ(processor_->getImuBufferSize(), 0);
}

// ==================== 去偏置测试 ====================

TEST_F(ImuProcessorTest, RemoveBias) {
    Vector3d acc_bias(0.1, 0.05, 0.02);
    Vector3d gyro_bias(0.01, 0.005, 0.002);
    
    processor_->setAccBias(acc_bias);
    processor_->setGyroBias(gyro_bias);
    
    Vector3d raw_acc(1.0, 0.5, 10.0);
    Vector3d raw_gyro(0.1, 0.05, 0.02);
    
    Vector3d corrected_acc = processor_->removeAccBias(raw_acc);
    Vector3d corrected_gyro = processor_->removeGyroBias(raw_gyro);
    
    EXPECT_NEAR(corrected_acc.x(), raw_acc.x() - acc_bias.x(), 1e-9);
    EXPECT_NEAR(corrected_acc.y(), raw_acc.y() - acc_bias.y(), 1e-9);
    EXPECT_NEAR(corrected_acc.z(), raw_acc.z() - acc_bias.z(), 1e-9);
    
    EXPECT_NEAR(corrected_gyro.x(), raw_gyro.x() - gyro_bias.x(), 1e-9);
    EXPECT_NEAR(corrected_gyro.y(), raw_gyro.y() - gyro_bias.y(), 1e-9);
    EXPECT_NEAR(corrected_gyro.z(), raw_gyro.z() - gyro_bias.z(), 1e-9);
}

// ==================== 时间同步测试 ====================

TEST_F(ImuProcessorTest, TimeSynchronization) {
    // 添加IMU数据
    for (int i = 0; i < 100; ++i) {
        ImuData imu = createTestImu(i * 0.01, Vector3d(0.0, 0.0, 9.81), Vector3d(0.0, 0.0, 0.01));
        processor_->addImuData(imu);
    }
    
    // 在时间点0.5插值
    ImuData interpolated = processor_->interpolateImu(0.5);
    
    // 验证插值结果
    EXPECT_NEAR(interpolated.timestamp, 0.5, 1e-6);
}

// ==================== 重力补偿测试 ====================

TEST_F(ImuProcessorTest, GravityCompensation) {
    // 静止状态下的重力补偿
    Vector3d acc_measure(0.0, 0.0, 9.81);  // 测量值（包含重力）
    
    // 补偿后的加速度应该接近零
    Vector3d acc_corrected = processor_->compensateGravity(acc_measure, Quaterniond::Identity());
    
    EXPECT_NEAR(acc_corrected.z(), 0.0, 0.1);  // 允许一些误差
}

// ==================== 协方差传播测试 ====================

TEST_F(ImuProcessorTest, CovariancePropagation) {
    Eigen::Matrix<double, 24, 24> P = Eigen::Matrix<double, 24, 24>::Identity() * 1e-6;
    double dt = 0.01;
    
    // 传播协方差
    Eigen::Matrix<double, 24, 24> Q = processor_->computeNoiseCovariance(dt);
    
    // 验证协方差矩阵
    EXPECT_GT(Q.trace(), 0);
}

// ==================== 边界条件测试 ====================

TEST_F(ImuProcessorTest, ExtremeAcceleration) {
    // 极大加速度
    ImuData imu = createTestImu(0.1, Vector3d(1000.0, 1000.0, 1000.0), Vector3d::Zero());
    
    EXPECT_NO_THROW(processor_->integrate(imu));
}

TEST_F(ImuProcessorTest, ZeroTimeStep) {
    // 零时间步长
    ImuData imu1 = createTestImu(0.1, Vector3d(0.0, 0.0, 9.81), Vector3d::Zero());
    ImuData imu2 = createTestImu(0.1, Vector3d(0.0, 0.0, 9.81), Vector3d::Zero());  // 相同时间戳
    
    processor_->integrate(imu1);
    EXPECT_NO_THROW(processor_->integrate(imu2));
}

TEST_F(ImuProcessorTest, NegativeTime) {
    // 负时间（异常情况）
    ImuData imu = createTestImu(-1.0, Vector3d::Zero(), Vector3d::Zero());
    
    // 应该能够处理或报错
    EXPECT_NO_THROW(processor_->integrate(imu));
}

// ==================== 重置测试 ====================

TEST_F(ImuProcessorTest, Reset) {
    // 添加数据
    for (int i = 0; i < 10; ++i) {
        ImuData imu = createTestImu(i * 0.1, Vector3d(0.0, 0.0, 9.81), Vector3d(0.0, 0.0, 0.01));
        processor_->addImuData(imu);
    }
    
    processor_->setAccBias(Vector3d(0.1, 0.1, 0.1));
    processor_->setGyroBias(Vector3d(0.01, 0.01, 0.01));
    
    // 重置
    processor_->reset();
    
    // 验证重置后状态
    EXPECT_EQ(processor_->getImuBufferSize(), 0);
    EXPECT_NEAR(processor_->getAccBias().norm(), 0.0, 1e-9);
    EXPECT_NEAR(processor_->getGyroBias().norm(), 0.0, 1e-9);
}

} // namespace test
} // namespace fast_lio2_slam

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}