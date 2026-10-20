#pragma once
// xzmap 写出模块 —— 设备端 3D 点云地图格式 (xz_3d_slam 自定义 JSON, 单文件)
//
// 与 SLAM 核心完全解耦: 只依赖 std + Eigen, 不感知任何 SLAM 类型/模块。
// 调用方负责把关键帧数据整理为本模块定义的中性结构 (XzMapKeyframe)。
//
// 坐标系约定:
//   位姿: body -> world, 平移单位 m, 欧拉角 ZYX (roll/pitch/yaw) 单位 rad
//   点云: body 系 (与位姿配套, world_pt = pose * body_pt), 单位 m
//
// 格式规则 (对齐设备端 xz_3d_slam 的 save_global_map_aft_sam / fromJsonExpandMap):
//   顶层键: map_name / points_size / key_frame_size / kf_frame_poses /
//           key_frame_points / key_frame_points_2d
//   id 从 1 开始; 数值为 int
//   量化: 平移 xyz * 1000 -> mm; 欧拉角 * 1000 -> mrad; 点 m -> mm (llround)
//   key_frame_points_2d 每帧空数组占位 (设备端加载空层仅告警不阻断)
//   文件为流式分块写出, 不整体驻留内存
#include <string>
#include <vector>
#include <Eigen/Eigen>

namespace rosiwit_slam {
namespace io {

struct XzMapPose {
    double x = 0, y = 0, z = 0;            // world <- body 平移 (m)
    double roll = 0, pitch = 0, yaw = 0;   // ZYX 欧拉角 (rad)
};

struct XzMapKeyframe {
    XzMapPose pose;
    std::vector<Eigen::Vector3f> points;   // body 系点云 (m)
};

// 写出 xzmap 文件; 成功返回 true, out_point_count 返回全部关键帧点数之和 (可空)
bool writeXzMap(const std::string& path,
                const std::string& map_name,
                const std::vector<XzMapKeyframe>& keyframes,
                long long* out_point_count = nullptr);

} // namespace io
} // namespace rosiwit_slam
