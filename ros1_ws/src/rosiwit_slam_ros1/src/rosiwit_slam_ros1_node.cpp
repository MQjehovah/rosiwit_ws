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

#include "rosiwit_interface.h"
#include "slam_engine.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosiwit_slam_ros1");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    rosiwit_slam_ros1::SlamEngine engine(nh, pnh);
    if (!engine.ok()) {
        // 与原实现一致: SlamEngine 构造里已经 ROS_FATAL + ros::shutdown(),
        // 这里直接收工 (退出码仍是 0, 不改变 roslaunch/respawn 的既有行为)。
        return 0;
    }

    rosiwit_slam_ros1::RosiwitInterface rosiwit_iface(nh, pnh, engine);
    // T5..T11: input_profile=legacy 时在此挂旧 3d_slam 接口
    //   rosiwit_slam_ros1::LegacyXz3dSlamInterface legacy_iface(nh, pnh, engine, rosiwit_iface);

    ROS_INFO("[rosiwit_slam_ros1] node up: profile=%s", engine.inputProfile().c_str());
    ros::spin();
    return 0;
}
