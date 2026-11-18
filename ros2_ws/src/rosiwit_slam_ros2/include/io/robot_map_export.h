#pragma once
// 机器人 (xz_3d_slam) 地图导出：把"全局点云 + 轨迹"转成设备端可直接定位的 xzmap。
//
// 这是**接口层格式适配**，不参与 SLAM 核心计算，因此放在 ROS 接口层
// (与 ros1_ws/.../src/io/xzmap_writer 对应)。
//
// 约定（每条都在真机上验证过，改了就会"贴墙但定位丢失"）:
//   1) 位姿扁平: z=0, roll=0, pitch=0，只保留 yaw (mapping_conf: fix_z/fix_rotation)
//   2) 角度单位 **毫弧度**（rad*1000），机器人侧 tf.yaw*0.001 后直接进 pcl::getTransformation
//   3) 关键帧点 = 关键帧局部坐标（机器人侧会 local2global 再变换回去）
//   4) 地图坐标以地面 z=0
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "io/xzmap_writer.h"

namespace rosiwit_slam {
namespace io {

/// 轨迹采样（由节点在收到新位姿时记录）
struct RobotKfSample {
    double x = 0, y = 0, z = 0;
    double yaw = 0;   // rad
};

/// 导出机器人 xzmap。
/// @param path        输出 .xzmap 路径
/// @param map_name    地图名（写入 map_name 字段）
/// @param track       轨迹采样（按时间顺序）
/// @param world_points 全局点云（世界坐标）
/// @param kf_step     关键帧间距(m)，建议 1.0（对齐 keyframeMeterGap）
/// @param out_kf      返回关键帧数（可空）
/// @param out_pts     返回写入点数（可空）
bool exportRobotXzMap(const std::string& path,
                      const std::string& map_name,
                      const std::vector<RobotKfSample>& track,
                      const std::vector<Eigen::Vector3f>& world_points,
                      double kf_step,
                      size_t* out_kf = nullptr,
                      size_t* out_pts = nullptr);

} // namespace io
} // namespace rosiwit_slam
