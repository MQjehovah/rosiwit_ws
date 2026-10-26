// ============================================================================
// rosiwit_slam_ros1 节点入口
//
// 只做三件事: ros::init -> 构造 SlamRosNode(组装根) -> ros::spin()。
// 组装根内部持有 SlamAlgorithm(算法) 与 SlamRos1Interface(ROS 适配);
// 主线程就是 ROS 线程, 只跑 topic/service 回调。
// ============================================================================
#include <ros/ros.h>

#include "node/slam_ros_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosiwit_slam_ros1");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    rosiwit_slam_ros1::SlamRosNode node(nh, pnh);
    if (!node.ok()) {
        return 0;
    }
    ROS_INFO("[rosiwit_slam_ros1] node up (SlamRosNode / SlamAlgorithm / SlamRos1Interface)");
    ros::spin();
    return 0;
}
