// test/test_slam_factory.cpp
#include <gtest/gtest.h>
#include "slam_core/slam_factory.h"

TEST(SlamFactory, CreateUnknownReturnsNull) {
    auto p = rosiwit_slam::SlamFactory::create("nonexistent_algo");
    EXPECT_EQ(p, nullptr);
}

TEST(SlamFactory, ListContainsFastLio2) {
    auto names = rosiwit_slam::SlamFactory::listNames();
    // 占位阶段尚未注册,先断言包含 factory 机制;A3 完成后注册 fast_lio2
    EXPECT_GE(names.size(), 1u);
    bool has_fast_lio2 = false;
    for (auto& n : names) if (n == "fast_lio2") has_fast_lio2 = true;
    EXPECT_TRUE(has_fast_lio2);
}
