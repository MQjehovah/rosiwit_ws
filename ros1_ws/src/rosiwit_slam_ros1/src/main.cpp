// ============================================================================
// rosiwit_slam_ros1 节点入口
//
// ros::init -> 构造 SlamRosNode(组装根) -> 两个 spinner -> 等退出。
// 组装根内部持有 SlamAlgorithm(算法) 与 SlamRos1Interface(ROS 适配)。
//
// 为什么不是 ros::spin():
//   单线程 spin 时, 任何一个服务回调阻塞 (切图等地图就绪 / 切模式重建核心 /
//   存图等待) 都会让 IMU/雷达回调与 0.1s 发布定时器一起停摆 —— 表现为
//   "切图期间点云不动、定位瞬间丢失"。所以:
//     io_spinner  (1 线程, 默认队列): 话题 + 定时器, 保持原有串行顺序
//     srv_spinner (1 线程, 服务队列): 服务, 阻塞只影响服务自身
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

    // 声明顺序 = 析构反向 -> spinner 先停, 再析构 node
    ros::AsyncSpinner srv_spinner(1, node.serviceQueue());
    ros::AsyncSpinner io_spinner(1);
    srv_spinner.start();
    io_spinner.start();
    ros::waitForShutdown();
    srv_spinner.stop();
    io_spinner.stop();
    return 0;
}
