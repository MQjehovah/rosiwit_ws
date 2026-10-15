// ============================================================================
// 旧 3d_slam 消息/数值转换 (计划 T5 §5.1)
//
// 纯逻辑: 只依赖 ROS 消息类型与核心的 GridInfo/CloudType, 不碰 ROS 句柄,
// 所以可以直接单测。所有"旧语义"都在这里集中定义, 便于对照 legacy.cpp 行号。
// ============================================================================
#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "slam_core/slam_types.h"
#include "xzrobot_ros_dependency/SlamInfo.h"

namespace rosiwit_slam_ros1 {

/// 旧 `/map` 的行序: **核心栅格原样** (row 0 = 最小 y), 不翻转 —— 旧实现是
/// 直接往 nav_msgs/OccupancyGrid::data 里按 v*cols+u 填的 (legacy.cpp:1737-1747),
/// 而它自己的 log-odds 栅格的 v 轴方向就是"从最小 y 起"。见设计文档 M3。
/// 注意: `origin.position.z = map_area * 1e-5` 是**调用方**加的 (legacy.cpp:1749)。
nav_msgs::OccupancyGrid toLegacyGrid(const rosiwit_slam::GridInfo& gi, const std::string& frame);

/// rosiwit 新接口的 `/map` 与 `grid_map` 行序: **竖直翻转** (row 0 = 最大 y)。
/// 值域保留核心原样 (0/100/-1)。
nav_msgs::OccupancyGrid toFlippedGrid(const rosiwit_slam::GridInfo& gi, const std::string& frame);

/// 组装 `slam/info` (设计文档 §5.3 字段表)。
/// `update_request` 恒 0; `matching_degree` 恒 0 —— D3 已知偏差
/// (旧实现此处其实是 localiza.getFitness(); 核心已有
///  LocalizationPipeline::confidence() 但未经 ISlamAlgorithm 暴露)。
xzrobot_ros_dependency::SlamInfo buildSlamInfo(int8_t slam_state,
                                               int8_t location_status,
                                               const std::string& map_name,
                                               double x, double y, double yaw,
                                               float map_area);

/// 告警 JSON —— 与旧 `json_warning_code::AlgorithmWarningCode::toJson(vector<int>)` 同形:
///   {"faults":[{"code":"..","content":"..","level":"..","module":"xz_3d_slam",
///               "name":"..","type":"algorithm fault"}]}
/// 键序照 nlohmann 默认的字母序; module/type 是常量
/// (json_warning_code.cpp:26-29 的 AlgorithmModule_SLAM → "xz_3d_slam";
///  json_warning_code.h:68 的成员默认 → "algorithm fault")。
std::string buildWarningJson(const std::string& code, const std::string& level,
                             const std::string& name, const std::string& content);

/// 一条 faults 记录
struct WarningFault {
    std::string code, level, name, content;
};

/// 多条合并成一个 `{"faults":[...]}` —— 对应旧实现
/// `AlgorithmWarningCode::toJson(const std::vector<int>&)` 的形状 (多码一次成型)。
std::string buildWarningJson(const std::vector<WarningFault>& faults);

sensor_msgs::PointCloud2 toPointCloud2(const rosiwit_slam::CloudType& c,
                                       const std::string& frame, const ros::Time& stamp);

/// 旧的 `scan_f_map` 用的老类型。旧实现走的是
/// `pcl::toROSMsg` + `sensor_msgs::convertPointCloud2ToPointCloud` (legacy.cpp:1005/1096),
/// 这里保持同一条转换链。
sensor_msgs::PointCloud toOldPointCloud(const rosiwit_slam::CloudType& c,
                                        const std::string& frame, const ros::Time& stamp);

/// 旧 TF / slam_info 用的 yaw: `atan2(R(0,1), R(0,0))` (legacy.cpp:1921)。
/// **注意这不是"机器人的 yaw"**: 对 `R = Rot(Z,θ)` 而言 `R(0,1) = -sinθ`、`R(0,0) = cosθ`,
/// 所以本函数返回 **`-θ`**; 旧实现紧接着用 `AngleAxis(-yaw, Z)` (legacy.cpp:1922) 再取负,
/// 两者合起来正好还原成 `Rot(Z,θ)`。所以"取 yaw"与"用 yaw"必须成对照抄, 单独看会以为符号错了。
double legacyYawFromRot(const Eigen::Matrix3d& R);

/// **标准航向角** (map 系下绕 Z 逆时针为正): `atan2(R(1,0), R(0,0))`。
/// 旧 `slam_info.robot_pose.theta` 用的 `curr_pose.yaw` 就是它 —— 旁证: legacy.cpp:1172-1183
/// 拿 yaw 做 `dx=0.06cos(yaw) / dy=0.06sin(yaw)` 的航向偏移, 并按 `sin(yaw/2)/cos(yaw/2)`
/// 造 Z 轴四元数, 两者都是标准航向约定。注意它恒等于 `-legacyYawFromRot(R)`
/// (后者是 Eigen 的 `atan2(R(0,1),R(0,0))` 约定, 旧 TF 里再取一次负才还原)。
double stdYawFromRot(const Eigen::Matrix3d& R);

/// 照抄旧实现的"当前帧点云塑形"(legacy.cpp:982-996 定位 / 1071-1085 建图):
///   1. 抽稀到约 2000 点: `down_sample = (N < 2000) ? 1 : round(N/2000)` (N 是输入点数)
///   2. 再乘 `downsample_multiplier` (旧实现给"原始帧"乘 2)
///   3. `z ∈ (0, z_max)` 过滤 —— 定位用 2.0、建图用 1.0
///      (旧实现给"原始帧"的 z 过滤是注释掉的, 用 `z_max <= 0` 表示不过滤)
/// 这是下游(导航/感知/可视化)习惯的数据形态, 不能省。
rosiwit_slam::CloudType::Ptr legacyScanShape(const rosiwit_slam::CloudType& in,
                                             double z_max,
                                             int downsample_multiplier = 1);

} // namespace rosiwit_slam_ros1
