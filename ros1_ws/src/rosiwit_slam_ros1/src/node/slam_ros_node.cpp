#include "node/slam_ros_node.h"

#include <exception>
#include <string>
#include <utility>
#include <vector>

namespace rosiwit_slam_ros1
{

namespace
{

/// lane 声明属于"执行资源策略", 不属于算法。需要时可通过参数覆盖 CPU 线程数。
std::vector<rosiwit_common::LaneSpec> makeLanes(ros::NodeHandle & pnh)
{
  int cpu_threads = 2;
  int feature_threads = 1;
  pnh.param<int>("pool_cpu_threads", cpu_threads, 2);
  pnh.param<int>("pool_feature_threads", feature_threads, 1);
  return {
    // 写盘/改地图: 必须串行 (避免两个保存/切图同时改同一个目录)
    {"disk_write", 1, 8, rosiwit_common::LaneOverflow::kReject},
    // 读盘/载图: 串行 (切图有 generation 保护, 串行更简单)
    {"disk_read", 1, 8, rosiwit_common::LaneOverflow::kReject},
    // CPU 突发: 实时栅格 / 质量体检 / 缩略图
    {"cpu", cpu_threads, 16, rosiwit_common::LaneOverflow::kReject},
    // 建库 (fpfh/scan_context): 独占低优先, 满就丢
    {"feature", feature_threads, 4, rosiwit_common::LaneOverflow::kDropOldest},
  };
}

/// 一行说明当前 lane 配置 (日志用, 别写死数字)
std::string describeLanes(ros::NodeHandle & pnh)
{
  std::string out;
  for (const auto & spec : makeLanes(pnh)) {
    if (!out.empty()) { out += ", "; }
    out += spec.name + ":" + std::to_string(spec.threads);
  }
  return out;
}

}  // namespace

SlamRosNode::SlamRosNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
: nh_(nh), pnh_(pnh), pool_(makeLanes(pnh)), algo_(nh, pnh, &pool_)
{
  if (!algo_.ok()) {
    ROS_FATAL("[slam_node] algorithm init failed");
    return;
  }

  // 接口层: 只在算法之后构造 —— 它会在构造函数里 addTickHook(), 必须先于 start()
  iface_.reset(new SlamRos1Interface(nh_, pnh_, algo_));

  // 输入订阅 (唯一一处): 回调只转发给算法, 由算法的 FrontEnd 线程消费
  imu_sub_ = nh_.subscribe(algo_.imuTopic(), 1000,
    &SlamAlgorithm::onImuMsg, &algo_);
  lidar_sub_ = nh_.subscribe(algo_.lidarTopic(), 100,
    &SlamAlgorithm::onLidarMsg, &algo_);
  init_pose_sub_ = nh_.subscribe("/initialpose", 10,
    &SlamAlgorithm::onInitialPoseMsg, &algo_);

  if (algo_.useVo()) {
    vo_sub_ = nh_.subscribe(algo_.voTopic(), 100,
      &SlamAlgorithm::onVoMsg, &algo_);
  }
  if (algo_.useAbsolute()) {
    absolute_sub_ = nh_.subscribe(algo_.absoluteTopic(), 100,
      &SlamAlgorithm::onAbsoluteMsg, &algo_);
  }

  algo_.start();
  ok_ = true;
  ROS_INFO("[slam_node] up: lanes={%s}, frontend thread + 50Hz tick started%s",
           describeLanes(pnh).c_str(),
           algo_.executor() ? " (executor injected)" : " (NO executor!)");
}

SlamRosNode::~SlamRosNode()
{
  // 关停顺序 (固定, 每一步都不能换):
  //   1. 停算法线程 (FrontEnd -> Tick): 它们是"会调进接口"的一方, 必须最先停;
  //      SlamRos1Interface 的析构还会再摘一次钩子做双保险。
  //   2. 拆接口 (发布者/服务/定时器) —— tick 停了以后不会再有钩子读快照。
  //      接口析构内部会等切图任务在途计数归零, 那些任务只碰 engine_, 此时仍然安全。
  //   3. 最后关执行器: 算法/接口关停时仍可能投递任务 (如飞行记录落盘)。
  algo_.shutdown();
  iface_.reset();
  pool_.shutdown(true);
}

}  // namespace rosiwit_slam_ros1
