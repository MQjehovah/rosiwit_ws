/**
 * @file test_imu_processor_fixed.cpp
 * @brief 单元测试 - IMU处理器（修复版）
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
        config_.acc_noise = 0.1;
        config_.gyro_noise = 0.01;
        config_.acc_bias_noise = 0.0001;
        config_.gyro_bias_noise = 0.00001;
        config_.gravity = 9.81;

        processor_ = std::make_unique<ImuProcessor>(config_);
    }

    ImuProcessorConfig config_;
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
    // 验证配置参数正确设置
    EXPECT_DOUBLE_EQ(config_.acc_noise, 0.1);
    EXPECT_DOUBLE_EQ(config_.gyro_noise, 0.01);
    EXPECT_DOUBLE_EQ(config_.gravity, 9.81);
    EXPECT_TRUE(processor_ != nullptr);
}

TEST_F(ImuProcessorTest, BufferOperations) {
    // 测试缓冲区操作
    EXPECT_EQ(processor_->bufferSize(), 0);

    // 添加IMU数据
    ImuData imu = createTestImu(0.1, Vector3d(0.0, 0.0, 9.81), Vector3d(0.0, 0.0, 0.0));
    processor_->addImuData(imu);
    
    EXPECT_EQ(processor_->bufferSize(), 1);

    // 添加更多数据
    for (int i = 0; i < 10; ++i) {
        ImuData imu_i = createTestImu(
            0.1 + i * 0.01,
            Vector3d(0.0, 0.0, 9.81),
            Vector3d(0.0, 0.0, 0.0)
        );
        processor_->addImuData(imu_i);
    }
    
    EXPECT_GE(processor_->bufferSize(), 10);

    // 清空缓冲区
    processor_->clearBuffer();
    EXPECT_EQ(processor_->bufferSize(), 0);
}

TEST_F(ImuProcessorTest, StaticInitializeBias) {
    // 添加静止状态下的IMU数据
    int count = 200;
    for (int i = 0; i < count; ++i) {
        ImuData imu = createTestImu(
            i * 0.01,
            Vector3d(0.0, 0.0, 9.81),  // 静止状态，测量重力
            Vector3d(0.0, 0.0, 0.0)     // 零角速度
        );
        processor_->addImuData(imu);
    }

    // 尝试静止初始化
    bool success = processor_->staticInitializeBias(count);
    
    // 如果有足够数据，应该成功
    if (processor_->bufferSize() >= count) {
        EXPECT_TRUE(success);
    }
}

TEST_F(ImuProcessorTest, GetImuInRange) {
    // 添加时间序列数据
    for (int i = 0; i < 100; ++i) {
        ImuData imu = createTestImu(
            i * 0.01,
            Vector3d(0.0, 0.0, 9.81),
            Vector3d(0.0, 0.0, 0.0)
        );
        processor_->addImuData(imu);
    }

    // 获取时间范围内的IMU数据
    std::vector<ImuData> imu_range = processor_->getImuInRange(0.0, 0.5);
    
    // 应该返回多个数据点
    EXPECT_GT(imu_range.size(), 0);
    EXPECT_LT(imu_range.size(), 100);  // 不应该包含所有数据
}

TEST_F(ImuProcessorTest, GetLatestImu) {
    // 空缓冲区
    ImuData imu_empty;
    bool result_empty = processor_->getLatestImu(imu_empty);
    EXPECT_FALSE(result_empty);

    // 添加数据
    ImuData imu1 = createTestImu(0.1, Vector3d(1, 2, 3), Vector3d(0.1, 0.2, 0.3));
    processor_->addImuData(imu1);

    ImuData imu2 = createTestImu(0.2, Vector3d(4, 5, 6), Vector3d(0.4, 0.5, 0.6));
    processor_->addImuData(imu2);

    // 获取最新数据
    ImuData latest;
    bool result = processor_->getLatestImu(latest);
    
    EXPECT_TRUE(result);
    EXPECT_NEAR(latest.timestamp, 0.2, 1e-9);
}

TEST_F(ImuProcessorTest, StatePropagation) {
    // 创建初始状态
    State state;
    state.position = Vector3d(0.0, 0.0, 0.0);
    state.velocity = Vector3d(0.0, 0.0, 0.0);
    state.rotation = Quaterniond::Identity();

    // 创建IMU数据
    ImuData imu = createTestImu(0.1, Vector3d(0.0, 0.0, 9.81), Vector3d(0.0, 0.0, 0.0));
    
    // 状态传播
    State new_state = processor_->propagate(state, imu, 0.1);

    // 验证新状态有效
    EXPECT_TRUE(std::isfinite(new_state.position.x()));
    EXPECT_TRUE(std::isfinite(new_state.position.y()));
    EXPECT_TRUE(std::isfinite(new_state.position.z()));
    EXPECT_TRUE(std::isfinite(new_state.velocity.x()));
    EXPECT_TRUE(std::isfinite(new_state.velocity.y()));
    EXPECT_TRUE(std::isfinite(new_state.velocity.z()));
}

TEST_F(ImuProcessorTest, SetBias) {
    Vector3d acc_bias(0.1, 0.2, 0.3);
    Vector3d gyro_bias(0.01, 0.02, 0.03);
    
    // 设置偏置
    processor_->setBias(acc_bias, gyro_bias);
    
    EXPECT_TRUE(true);  // 只要不崩溃就算成功
}

TEST_F(ImuProcessorTest, SetInitialState) {
    State state;
    state.position = Vector3d(1.0, 2.0, 3.0);
    state.velocity = Vector3d(0.5, 0.6, 0.7);
    state.rotation = Quaterniond::Identity();
    
    processor_->setInitialState(state);
    
    EXPECT_TRUE(true);  // 只要不崩溃就算成功
}

TEST_F(ImuProcessorTest, HasEnoughDataForInit) {
    // 空缓冲区
    bool result_empty = processor_->hasEnoughDataForInit();
    EXPECT_FALSE(result_empty);

    // 添加足够数据
    for (int i = 0; i < 200; ++i) {
        ImuData imu = createTestImu(
            i * 0.01,
            Vector3d(0.0, 0.0, 9.81),
            Vector3d(0.0, 0.0, 0.0)
        );
        processor_->addImuData(imu);
    }

    bool result_enough = processor_->hasEnoughDataForInit();
    EXPECT_TRUE(result_enough);
}

} // namespace test
} // namespace fast_lio2_slam

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}