// include/slam_core/slam_factory.h
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "slam_core/i_slam_algorithm.h"

namespace rosiwit_slam {

class SlamFactory {
public:
    static std::unique_ptr<ISlamAlgorithm> create(const std::string& name);
    static std::vector<std::string> listNames();
};

} // namespace rosiwit_slam
