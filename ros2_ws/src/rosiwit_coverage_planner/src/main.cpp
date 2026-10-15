// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include <rclcpp/rclcpp.hpp>
#include "coverage_planner/coverage_planner.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<coverage_planner::CoveragePlannerNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    
    return 0;
}