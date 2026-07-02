#pragma once

// Standard library (lightweight, universally used)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// PCL core
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/search/kdtree.h>

// Project types
#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/common/sophus_se3.hpp"
