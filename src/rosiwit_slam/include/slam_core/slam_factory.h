// include/slam_core/slam_factory.h
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "slam_core/i_slam_algorithm.h"
#include "slam_core/i_frontend.h"

namespace rosiwit_slam {

class SlamFactory {
public:
    static std::unique_ptr<ISlamAlgorithm> create(const std::string& name);
    static std::vector<std::string> listNames();
    static std::unique_ptr<IFrontend> createFrontend(const std::string& name);
};

} // namespace rosiwit_slam
