/**
 * @file test_iekf_estimator.cpp
 * @brief 单元测试 - IEKF状态估计器
 * @author AI Development Team - Test Engineer
 * @date 2026-04-24
 */

#include <gtest/gtest.h>
#include "fast_lio2_slam/fast_lio2_core/iekf_estimator.h"
#include <cmath>

namespace fast_lio2_slam {
namespace test {

class IekfEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.max_iterations = 5;
        config_.converge_threshold = 1e-4;
        config_.point_noise = 0.02;
        config_.acc_noise = 0.1;
        config_.gyro_noise = 0.01;

        estimator_ = std::make_unique<IekfEstimator>(config_);
    }

    IekfConfig config_;
    std::unique_ptr<IekfEstimator> estimator_;
};

// ==================== 初始化测试 ====================

TEST_F(IekfEstimatorTest, DefaultInitialization) {
    IekfEstimator est;
    State state = est.getState();

    // 默认状态应为零
    EXPECT_NEAR(state.position.norm(), 0.0, 1e-9);
}

TEST_F(IekfEstimatorTest, ConfigInitialization) {
    State state = estimator_->getState();
    EXPECT_TRUE(estimator_->isInitialized());
}

TEST_F(IekfEstimatorTest, SetInitialState) {
    State initial_state;
    initial_state.position = Vector3d(1.0, 2.0, 3.0);
    initial_state.velocity = Vector3d(0.1, 0.2, 0.3);

    estimator_->setInitialState(initial_state);
    State retrieved = estimator_->getState();

    EXPECT_NEAR(retrieved.position.x(), 1.0, 1e-9);
    EXPECT_NEAR(retrieved.position.y(), 2.0, 1e-9);
    EXPECT_NEAR(retrieved.position.z(), 3.0, 1e-9);
    EXPECT_NEAR(retrieved.velocity.x(), 0.1, 1e-9);
}

// ==================== IMU预测测试 ====================

TEST_F(IekfEstimatorTest, ImuPrediction_ZeroInput) {
    State initial_state;
    initial_state.position = Vector3d(0.0, 0.0, 0.0);
    estimator_->setInitialState(initial_state);

    // 零加速度、零角速度的IMU数据
    ImuData imu(0.1, Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0));

    // 由于没有输入，预测后状态应接近初始状态
    estimator_->predict(imu, 0.1);

    State state = estimator_->getState();
    // 位置应该变化很小或保持接近零
    EXPECT_LT(state.position.norm(), 1.0);  // 松散约束
}


TEST_F(IekfEstimatorTest, ImuPrediction_NonZeroAcceleration) {
    State initial_state;
    initial_state.position = Vector3d(0.0, 0.0, 0.0);
    initial_state.velocity = Vector3d(0.0, 0.0, 0.0);
    estimator_->setInitialState(initial_state);

    // 恒定加速度输入
    ImuData imu(0.1, Vector3d(1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0));
    estimator_->predict(imu, 0.1);

    State state = estimator_->getState();
    // 位置应该有变化（运动状态）
    EXPECT_GT(state.position.x(), 0.0);
}

// ==================== 协方差传播测试 ====================

TEST_F(IekfEstimatorTest, CovariancePropagation) {
    State initial_state;
    estimator_->setInitialState(initial_state);

    ImuData imu(0.1, Vector3d(0.1, 0.1, 0.1), Vector3d(0.01, 0.01, 0.01));
    estimator_->predict(imu, 0.1);

    // 协方差应该增长
    MatrixXd covariance = estimator_->getCovariance();
    EXPECT_GT(covariance.trace(), 0.0);
}

// ==================== 测量更新测试 ====================

TEST_F(IekfEstimatorTest, MeasurementUpdate_BasicPoints) {
    State initial_state;
    initial_state.position = Vector3d(0.0, 0.0, 0.0);
    estimator_->setInitialState(initial_state);

    std::vector<Vector3d> points;
    points.push_back(Vector3d(1.0, 0.0, 0.0));
    points.push_back(Vector3d(0.0, 1.0, 0.0));
    points.push_back(Vector3d(0.0, 0.0, 1.0));
    
    std::vector<Vector3d> map_points;
    map_points.push_back(Vector3d(1.1, 0.0, 0.0));
    map_points.push_back(Vector3d(0.0, 1.1, 0.0));
    map_points.push_back(Vector3d(0.0, 0.0, 1.1));
    
    std::vector<std::pair<int, int>> matches;
    matches.push_back(std::make_pair(0, 0));
    matches.push_back(std::make_pair(1, 1));
    matches.push_back(std::make_pair(2, 2));
    
    bool updated = estimator_->update(points, map_points, matches);
    EXPECT_TRUE(updated);
}


TEST_F(IekfEstimatorTest, IterationConvergence) {
    IekfEstimator conv_estimator;
    
    State initial_state;
    conv_estimator.setInitialState(initial_state);
    
    // 逐步预测和更新，观察迭代收敛
    for (int i = 0; i < 5; ++i) {
        ImuData imu(0.1, Vector3d(0.1, 0.1, 0.1), Vector3d(0.01, 0.01, 0.01));
        conv_estimator.predict(imu, 0.1);
        
        std::vector<Vector3d> points;
        points.push_back(Vector3d(1.0, 0.0, 0.0));
        
        std::vector<Vector3d> map_points;
        map_points.push_back(Vector3d(1.0 + 0.1 * i, 0.0, 0.0));
        
        std::vector<std::pair<int, int>> matches;
        matches.push_back(std::make_pair(0, 0));
        
        conv_estimator.update(points, map_points, matches);
    }
    
    // 最终状态应该稳定
    State final_state = conv_estimator.getState();
    EXPECT_LT(final_state.position.norm(), 10.0);  // 状态应该合理
}

// ==================== 状态获取测试 ====================

TEST_F(IekfEstimatorTest, StateRetrieval) {
    State state = estimator_->getState();
    
    // 检查状态结构完整性
    EXPECT_EQ(state.position.size(), 3);
    EXPECT_EQ(state.velocity.size(), 3);
}

TEST_F(IekfEstimatorTest, CovarianceSize) {
    MatrixXd cov = estimator_->getCovariance();
    
    // 协方差矩阵维度正确
    EXPECT_EQ(cov.rows(), 18);
    EXPECT_EQ(cov.cols(), 18);
}

}  // namespace test
}  // namespace fast_lio2_slam

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}