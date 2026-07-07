// src/slam_core/slam_factory.cpp
#include "slam_core/slam_factory.h"
#include "algorithms/fast_lio2/fast_lio2_algorithm.h"

namespace rosiwit_slam {

std::unique_ptr<ISlamAlgorithm> SlamFactory::create(const std::string& name) {
    if (name == "fast_lio2") return std::make_unique<FastLio2Algorithm>();
    return nullptr;
}

std::vector<std::string> SlamFactory::listNames() {
    return { "fast_lio2" };
}

} // namespace rosiwit_slam
