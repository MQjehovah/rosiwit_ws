// ============================================================================
// rosiwit_slam_ros1 节点入口
//
// 一个进程、一个算法核心实例、一个对外接口:
//   SlamRos1Interface = 老 xz_3d_slam 兼容接口(基线) + 不冲突的新输出
// ============================================================================
#include <ros/ros.h>

#include <memory>

#include "slam_engine.h"
#include "facade.h"

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

    ROS_INFO("[rosiwit_slam_ros1] node up (single interface)");
    ros::spin();
    return 0;
}
