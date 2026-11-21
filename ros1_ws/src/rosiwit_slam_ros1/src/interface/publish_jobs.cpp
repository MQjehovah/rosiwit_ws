// ============================================================================
// publish_jobs.cpp — 低价值/耗时消息: 构造 + 发布都在线程池里做
//
// 线程模型 (§docs/ros1_thread_model.md):
//   io  线程: 订阅回调 + 20ms 心跳 (位姿/TF/slam_info/告警码, 都是小消息)
//   pool     : 点云抽稀/转换、栅格转 msg、2D 匹配、在线标定 —— 含它们自己的 publish
// 大消息不再经过任何"输出线程": ROS1 的 publish() 在调用线程上序列化, 与其来回转手,
// 不如就在算它的那条池线程上发完。这些话题没有顺序要求 (最新状态才有意义)。
//
// 去重: 每类任务一个 busy 标志, 上一份还在算就丢掉本帧。
// 生命周期: 任务体捕获 this -> AuxJobGuard + waitForAuxJobs() 保证析构时已收尾。
// ============================================================================
#include "ros1_slam_interface.h"

#include <memory>

#include <ros/ros.h>

#include "outputs.h"

#include "convert.h"
#include "internal.h"

using namespace rosiwit_slam;
using namespace rosiwit_slam_ros1::interface_internal;

namespace rosiwit_slam_ros1 {

// ---------------------------------------------------------------- 调度 (IO/发布线程)
void Ros1SlamInterface::scheduleScanJob() {
    if (scan_job_busy_.exchange(true)) return;   // 上一帧还在转 -> 丢这帧
    aux_jobs_.submit(engine_, "pool", "scan_pub",
        [this] { buildScanMessages(); },
        [this] { scan_job_busy_.store(false); });
}

void Ros1SlamInterface::scheduleGridJob() {
    if (grid_job_busy_.exchange(true)) return;
    aux_jobs_.submit(engine_, "pool", "grid_pub",
        [this] { buildGridMessage(); },
        [this] { grid_job_busy_.store(false); });
}

void Ros1SlamInterface::scheduleGlobalMapJob() {
    if (global_map_job_busy_.exchange(true)) return;
    aux_jobs_.submit(engine_, "pool", "global_map_pub",
        [this] { buildGlobalMapMessage(); },
        [this] { global_map_job_busy_.store(false); });
}

void Ros1SlamInterface::schedulePlanarJob() {
    if (!planar_constraint_enabled_) return;
    if (planar_job_busy_.exchange(true)) return;
    // 2D 匹配是纯计算 (可能几十 ms): 放在发布线程上会直接顶住位姿/TF
    aux_jobs_.submit(engine_, "pool", "planar_step",
        [this] { planarConstraintStep(); },
        [this] { planar_job_busy_.store(false); });
}

// ---------------------------------------------------------------- 构造 (池线程)
void Ros1SlamInterface::buildScanMessages() {
    SlamOutput out;
    if (!engine_.latestOutput(out) || !out.world_cloud || out.world_cloud->empty()) return;

    const CloudType& world = *out.world_cloud;   // 核心给的就是 map 系下的当前帧
    const ros::Time stamp = out.pose.time > 0 ? ros::Time(out.pose.time) : ros::Time::now();
    const SlamMode mode = slam_mode_.load();

    const bool want_f = ros1_pointcloud_puber_.getNumSubscribers() > 0;              // /scan_f_map
    const bool want_dbg = debug_publish_ && pointcloud_puber_.getNumSubscribers() > 0; // /curr_scan_point

    std::shared_ptr<sensor_msgs::PointCloud2> scan_point;   // /curr_scan_point
    std::shared_ptr<sensor_msgs::PointCloud>  scan_f;       // /scan_f_map
    std::shared_ptr<nav_msgs::Path>           path;         // /mapping_path

    if (mode == SlamMode::LOCALIZATION) {
        // 定位: scan_f_map 抽稀 z∈(0.1,2); curr_scan_point 只在 debug 打开时发
        if (want_f || want_dbg) {
            CloudType::Ptr shaped = scanShape(world, kLocZMax, 1, scan_ground_z_, kScanZMin);
            if (shaped && !shaped->empty()) {
                if (want_f) {
                    scan_f = std::make_shared<sensor_msgs::PointCloud>(
                        toOldPointCloud(*shaped, kMapFrame, stamp));
                }
                if (want_dbg) {
                    scan_point = std::make_shared<sensor_msgs::PointCloud2>(
                        toPointCloud2(*shaped, kMapFrame, stamp));
                }
            }
        }
    } else if (mode == SlamMode::MAPPING) {
        // 建图: curr_scan_point 无条件 (有订阅就发), scan_f_map 抽稀
        if (want_dbg) {
            scan_point = std::make_shared<sensor_msgs::PointCloud2>(
                toPointCloud2(world, kMapFrame, stamp));
        }
        if (want_f) {
            CloudType::Ptr shaped = scanShape(world, kMapZMax, 1, scan_ground_z_, kScanZMin);
            if (shaped && !shaped->empty()) {
                scan_f = std::make_shared<sensor_msgs::PointCloud>(
                    toOldPointCloud(*shaped, kMapFrame, stamp));
            }
        }
        // 轨迹整条重发很贵: 10Hz 触发里只保留 1Hz 实际构造
        const double now = ros::Time::now().toSec();
        if (path_puber_.getNumSubscribers() > 0 &&
            now - last_path_build_s_.load() >= 1.0) {
            nav_msgs::Path msg;
            {
                std::lock_guard<std::mutex> lk(path_mu_);
                if (!mapping_path_msg_.poses.empty()) {
                    mapping_path_msg_.header.frame_id = kMapFrame;
                    msg = mapping_path_msg_;   // 锁内拷贝, 锁外序列化
                }
            }
            if (!msg.poses.empty()) {
                last_path_build_s_.store(now);
                path = std::make_shared<nav_msgs::Path>(std::move(msg));
            }
        }
    } else {
        return;
    }

    // 就地发布: 这些话题都是"最新状态", 丢中间帧不影响语义, 也不需要和别的线程排序
    if (scan_point) pointcloud_puber_.publish(*scan_point);
    if (scan_f)     ros1_pointcloud_puber_.publish(*scan_f);
    if (path)       path_puber_.publish(*path);
}

void Ros1SlamInterface::buildGridMessage() {
    std::shared_ptr<nav_msgs::OccupancyGrid> msg;
    const auto snap = realtime_grid_.latestPtr();
    if (snap) {
        msg = std::make_shared<nav_msgs::OccupancyGrid>(toRosGrid(*snap, kMapFrame));
        const double area = static_cast<double>(snap->width) * snap->height *
                            snap->resolution * snap->resolution;
        realtime_grid_map_area_.store(area);
        // legacy.cpp:1749 把建图面积塞进 origin.position.z
        msg->info.origin.position.z = area * 0.00001;
        setStaticMap(msg);         // shared_ptr: 不再复制 640KB
    } else {
        // 没有新栅格也照发上一张 (legacy.cpp:909 无条件 publish) —— 直接复用快照
        auto cur = staticMapSnapshot();
        if (!cur || cur->data.empty()) return;
        realtime_grid_map_puber_.publish(*cur);
        return;
    }
    realtime_grid_map_puber_.publish(*msg);   // latched: 订阅者随到随取
}

void Ros1SlamInterface::buildGlobalMapMessage() {
    PointVec pts;
    if (!engine_.getGlobalMap(pts) || pts.empty()) return;
    CloudType cloud;
    cloud.reserve(pts.size());
    for (auto& p : pts) cloud.push_back(p);
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;
    point_map_puber_.publish(toPointCloud2(cloud, kMapFrame, ros::Time::now()));
}

void Ros1SlamInterface::waitForAuxJobs() { aux_jobs_.shutdown("Ros1SlamInterface"); }

void Ros1OutputsInterface::scheduleCalibJob() {
    const ros::Time now = ros::Time::now();
    if (!last_calib_update_.isZero() &&
        (now - last_calib_update_).toSec() < 2.0) {
        return;
    }
    if (calib_busy_.exchange(true)) return;
    last_calib_update_ = now;
    // 在线标定是纯计算: 原来在 50Hz 输出线程上跑, 标定变慢会直接拖 /lio_odom
    aux_jobs_.submit(engine_, "pool", "calib",
        [this] { engine_.updateCalibration(); },
        [this] { calib_busy_.store(false); });
}

void Ros1OutputsInterface::waitForAuxJobs() { aux_jobs_.shutdown("Ros1OutputsInterface"); }

}  // namespace rosiwit_slam_ros1
