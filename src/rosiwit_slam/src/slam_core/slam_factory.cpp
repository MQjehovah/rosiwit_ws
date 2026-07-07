// src/slam_core/slam_factory.cpp
#include "slam_core/slam_factory.h"
// Task C1 将在此 #include "algorithms/fast_lio2/fast_lio2_algorithm.h"

namespace rosiwit_slam {

std::unique_ptr<ISlamAlgorithm> SlamFactory::create(const std::string& name) {
    // Task C1: if (name == "fast_lio2") return std::make_unique<FastLio2Algorithm>();
    (void)name;
    return nullptr;
}

std::vector<std::string> SlamFactory::listNames() {
    return { "fast_lio2" };  // 占位声明;create() 在 Task C1 才真正支持
}

} // namespace rosiwit_slam
