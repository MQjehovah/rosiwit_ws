// ============================================================================
// rosiwit_slam_ros1 节点入口 (计划 T4 之后只剩"装配"这一步)
//
// 一个进程、一个算法核心实例, 由 SlamEngine 持有; 上面挂两套 ROS 接口:
//   - RosiwitInterface : rosiwit_slam 原有的新接口 (始终挂)
//   - LegacyXz3dSlamInterface : 旧 xz_3d_slam 的接口 (input_profile=legacy 时挂, T5-T11)
//
// 原来的 376 行单类已拆到 slam_engine.* / rosiwit_interface.*, 内容逐字搬移。
// ============================================================================
#include <ros/ros.h>

#include <memory>

#include "slam_engine.h"
#include "slam_ros1_interface.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosiwit_slam_ros1");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    rosiwit_slam_ros1::SlamEngine engine(nh, pnh);
    if (!engine.ok()) {
        return 0;
    }

    // 唯一对外接口: 老接口优先, 新输出只保留不冲突项.
    rosiwit_slam_ros1::SlamRos1Interface iface(nh, pnh, engine);

    ROS_INFO("[rosiwit_slam_ros1] node up: profile=%s (single interface)",
             engine.inputProfile().c_str());
    ros::spin();
    return 0;
}
