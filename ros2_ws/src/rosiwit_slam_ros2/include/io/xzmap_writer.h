#pragma once
// xzmap 写出模块 —— 设备端 3D 点云地图格式 (xz_3d_slam 自定义 JSON, 单文件)
//
// 与 SLAM 核心完全解耦: 只依赖 std + Eigen。
// 调用方负责把关键帧整理成本模块的中性结构 (XzMapKeyframe)。
//
// 坐标/单位约定（真机验证，务必保持）:
//   位姿: body -> world, 平移 m, 欧拉角 ZYX (roll/pitch/yaw) **弧度**
//   点云: 关键帧**局部(body)系**, 与位姿配套: world_pt = pose * body_pt
//   量化: 平移 mm；欧拉角 **mrad**(rad*1000)；点 mm
//
// 机器人侧读取: nano_gicp_localization.cpp:9336
//     pcl::getTransformation(tf.x*0.001, ..., tf.yaw*0.001)   // 弧度
//     p_local → local2global(p_local, pose)
#include <string>
#include <vector>
#include <Eigen/Eigen>

namespace rosiwit_slam {
namespace io {

struct XzMapPose {
    double x = 0, y = 0, z = 0;            // world <- body (m)
    double roll = 0, pitch = 0, yaw = 0;   // ZYX (rad)
};

struct XzMapKeyframe {
    XzMapPose pose;
    std::vector<Eigen::Vector3f> points;   // body 系 (m)
};

bool writeXzMap(const std::string& path,
                const std::string& map_name,
                const std::vector<XzMapKeyframe>& keyframes,
                long long* out_point_count = nullptr);

} // namespace io
} // namespace rosiwit_slam
