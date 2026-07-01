// ============================================================
// Diffbot Navigation - 参数管理器单元测试
// 测试参数获取、设置、验证和回调机制
// ============================================================

#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "diffbot_navigation/core/parameter_manager.hpp"

using namespace diffbot_navigation::core;

class ParameterManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::NodeOptions options;
        options.allow_undeclared_parameters(true);
        node_ = std::make_shared<rclcpp::Node>("test_parameter_manager", options);
        param_manager_ = std::make_unique<ParameterManager>(node_);
    }

    void TearDown() override
    {
        param_manager_.reset();
        node_.reset();
    }

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<ParameterManager> param_manager_;
};

// ==================== 基本参数操作测试 ====================

TEST_F(ParameterManagerTest, GetParameterWithDefault)
{
    // 获取未设置的参数，返回默认值
    double value = param_manager_->getParameter<double>("test_double", 0.5);
    EXPECT_DOUBLE_EQ(value, 0.5);
    
    int int_value = param_manager_->getParameter<int>("test_int", 10);
    EXPECT_EQ(int_value, 10);
    
    std::string str_value = param_manager_->getParameter<std::string>("test_string", "default");
    EXPECT_EQ(str_value, "default");
}

TEST_F(ParameterManagerTest, SetAndGetParameter)
{
    // 设置参数
    param_manager_->setParameter<double>("test_velocity", 0.8);
    
    // 获取参数
    double value = param_manager_->getParameter<double>("test_velocity", 0.0);
    EXPECT_DOUBLE_EQ(value, 0.8);
}

TEST_F(ParameterManagerTest, SetParameterOverridesNodeParameter)
{
    // 先通过节点声明参数
    node_->declare_parameter("node_param", 1.0);
    
    // 参数管理器获取节点参数
    double node_value = param_manager_->getParameter<double>("node_param", 0.0);
    EXPECT_DOUBLE_EQ(node_value, 1.0);
    
    // 参数管理器设置新值
    param_manager_->setParameter<double>("node_param", 2.0);
    
    // 获取新值
    double new_value = param_manager_->getParameter<double>("node_param", 0.0);
    EXPECT_DOUBLE_EQ(new_value, 2.0);
}

TEST_F(ParameterManagerTest, SetMultipleParameters)
{
    param_manager_->setParameter<double>("max_velocity", 0.5);
    param_manager_->setParameter<double>("max_acceleration", 1.0);
    param_manager_->setParameter<double>("tolerance", 0.1);
    param_manager_->setParameter<double>("timeout", 5.0);
    
    EXPECT_DOUBLE_EQ(param_manager_->getParameter<double>("max_velocity", 0.0), 0.5);
    EXPECT_DOUBLE_EQ(param_manager_->getParameter<double>("max_acceleration", 0.0), 1.0);
    EXPECT_DOUBLE_EQ(param_manager_->getParameter<double>("tolerance", 0.0), 0.1);
    EXPECT_DOUBLE_EQ(param_manager_->getParameter<double>("timeout", 0.0), 5.0);
}

// ==================== 参数类型测试 ====================

TEST_F(ParameterManagerTest, IntegerParameter)
{
    param_manager_->setParameter<int>("int_param", 42);
    int value = param_manager_->getParameter<int>("int_param", 0);
    EXPECT_EQ(value, 42);
}

TEST_F(ParameterManagerTest, DoubleParameter)
{
    param_manager_->setParameter<double>("double_param", 3.14159);
    double value = param_manager_->getParameter<double>("double_param", 0.0);
    EXPECT_DOUBLE_EQ(value, 3.14159);
}

TEST_F(ParameterManagerTest, StringParameter)
{
    param_manager_->setParameter<std::string>("string_param", "hello_world");
    std::string value = param_manager_->getParameter<std::string>("string_param", "");
    EXPECT_EQ(value, "hello_world");
}

TEST_F(ParameterManagerTest, BoolParameter)
{
    param_manager_->setParameter<bool>("bool_param", true);
    bool value = param_manager_->getParameter<bool>("bool_param", false);
    EXPECT_TRUE(value);
    
    param_manager_->setParameter<bool>("bool_param", false);
    value = param_manager_->getParameter<bool>("bool_param", true);
    EXPECT_FALSE(value);
}

// ==================== 参数回调测试 ====================

TEST_F(ParameterManagerTest, ParameterChangeCallback)
{
    int callback_count = 0;
    double received_value = 0.0;
    
    param_manager_->registerParameterCallback<double>("callback_param",
        [&](const double& value) {
            callback_count++;
            received_value = value;
        });
    
    param_manager_->setParameter<double>("callback_param", 1.0);
    
    EXPECT_EQ(callback_count, 1);
    EXPECT_DOUBLE_EQ(received_value, 1.0);
    
    param_manager_->setParameter<double>("callback_param", 2.0);
    
    EXPECT_EQ(callback_count, 2);
    EXPECT_DOUBLE_EQ(received_value, 2.0);
}

// ==================== 参数检查测试 ====================

TEST_F(ParameterManagerTest, HasParameter)
{
    EXPECT_FALSE(param_manager_->hasParameter("new_param"));
    
    param_manager_->setParameter<double>("new_param", 1.0);
    
    EXPECT_TRUE(param_manager_->hasParameter("new_param"));
}

TEST_F(ParameterManagerTest, HasParameterWithNodeDeclared)
{
    EXPECT_FALSE(param_manager_->hasParameter("node_declared_param"));
    
    node_->declare_parameter("node_declared_param", 1.0);
    
    EXPECT_TRUE(param_manager_->hasParameter("node_declared_param"));
}

// ==================== 边界情况测试 ====================

TEST_F(ParameterManagerTest, EmptyParameterName)
{
    // 空名称参数应该被处理
    EXPECT_NO_THROW(param_manager_->setParameter<double>("", 1.0));
    EXPECT_NO_THROW(param_manager_->getParameter<double>("", 0.0));
}

TEST_F(ParameterManagerTest, ParameterWithSpecialCharacters)
{
    param_manager_->setParameter<double>("param_with/slashes", 1.0);
    param_manager_->setParameter<double>("param_with.dots", 2.0);
    param_manager_->setParameter<double>("param_with_underscores", 3.0);
    
    EXPECT_DOUBLE_EQ(param_manager_->getParameter<double>("param_with/slashes", 0.0), 1.0);
    EXPECT_DOUBLE_EQ(param_manager_->getParameter<double>("param_with.dots", 0.0), 2.0);
    EXPECT_DOUBLE_EQ(param_manager_->getParameter<double>("param_with_underscores", 0.0), 3.0);
}

TEST_F(ParameterManagerTest, VeryLargeDoubleValue)
{
    double large_value = 1e308;
    param_manager_->setParameter<double>("large_param", large_value);
    
    double value = param_manager_->getParameter<double>("large_param", 0.0);
    EXPECT_DOUBLE_EQ(value, large_value);
}

TEST_F(ParameterManagerTest, VerySmallDoubleValue)
{
    double small_value = 1e-308;
    param_manager_->setParameter<double>("small_param", small_value);
    
    double value = param_manager_->getParameter<double>("small_param", 0.0);
    EXPECT_DOUBLE_EQ(value, small_value);
}

TEST_F(ParameterManagerTest, NegativeDoubleValue)
{
    param_manager_->setParameter<double>("negative_double", -100.5);
    
    double value = param_manager_->getParameter<double>("negative_double", 0.0);
    EXPECT_DOUBLE_EQ(value, -100.5);
}

// ==================== 线程安全测试 ====================

TEST_F(ParameterManagerTest, ThreadSafeSetGet)
{
    const int num_threads = 10;
    const int ops_per_thread = 100;
    std::vector<std::thread> threads;
    std::atomic<int> errors{0};
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&, i]() {
            for (int j = 0; j < ops_per_thread; ++j) {
                std::string param_name = "thread_" + std::to_string(i) + "_param_" + std::to_string(j);
                
                try {
                    param_manager_->setParameter<double>(param_name, i + j);
                    double value = param_manager_->getParameter<double>(param_name, 0.0);
                    
                    if (value != i + j) {
                        errors++;
                    }
                } catch (...) {
                    errors++;
                }
            }
        });
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    EXPECT_EQ(errors.load(), 0);
}

// ==================== PrintAllParameters测试 ====================

TEST_F(ParameterManagerTest, PrintAllParameters)
{
    param_manager_->setParameter<double>("param1", 1.0);
    param_manager_->setParameter<std::string>("param2", "test");
    param_manager_->setParameter<bool>("param3", true);
    
    // 打印应该不会抛出异常
    EXPECT_NO_THROW(param_manager_->printAllParameters());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}