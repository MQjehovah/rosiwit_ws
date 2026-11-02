// include/slam_core/point_xyzirgb.h
// 地图点的"全字段"表示: xyz + 强度 + rgb.
//
// 为什么需要它: 地图只产出一个 map.pcd。纯 LiDAR 前端只能给 xyz+强度,
// LIVO 前端还能给颜色; 用这个类型写盘, 两种前端产出的都是同一个文件,
// 有颜色时就是 xyz+intensity+rgb, 没有时就是 xyz+intensity —— 下游按字段自适应.
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace rosiwit_slam {

struct EIGEN_ALIGN16 PointXYZIRGB {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  PCL_ADD_RGB;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace rosiwit_slam

POINT_CLOUD_REGISTER_POINT_STRUCT(
    rosiwit_slam::PointXYZIRGB,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, rgb, rgb))
