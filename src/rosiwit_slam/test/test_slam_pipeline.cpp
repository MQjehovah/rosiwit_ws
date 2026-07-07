#include <gtest/gtest.h>
#include "slam_core/slam_pipeline.h"
#include "slam_core/slam_factory.h"

using namespace rosiwit_slam;

TEST(SlamPipeline, CreatePipeline) {
    auto pipe = std::make_unique<SlamPipeline>();
    EXPECT_FALSE(pipe->init("nonexistent.yaml"));
}

TEST(SlamPipeline, CreateFromFactory) {
    auto algo = SlamFactory::create("fast_lio2_pipeline");
    EXPECT_NE(algo, nullptr);
}
