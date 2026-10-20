// ============================================================================
// Ros1SlamInterface 告警：码表加载、频率窗口、告警发布与抑制窗口。
// ============================================================================

#include "ros1_slam_interface.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "convert.h"
#include "internal.h"
#include "map_adapter.h"

using namespace rosiwit_slam;
using namespace rosiwit_slam_ros1::interface_internal;

namespace rosiwit_slam_ros1 {
void Ros1SlamInterface::loadWarningCodeTable() {
    // M6: 从**本包 share**读, 不读 /opt —— 旧实现用的是编译期 PROJECT_PATH
    // (json_warning_code.cpp:168 的 project_path)。
    const std::string path =
        ros::package::getPath("rosiwit_slam_ros1") + "/conf/warning_code.json";
    std::ifstream fin(path);
    if (!fin.good()) {
        ROS_WARN_STREAM("[ros1_slam] warning_code.json not readable: " << path);
        return;
    }
    try {
        const YAML::Node n = YAML::LoadFile(path);   // JSON 是 YAML 的子集
        if (!n.IsMap()) {
            ROS_WARN_STREAM("[ros1_slam] warning_code.json is not a map: " << path);
            return;
        }
        for (auto it = n.begin(); it != n.end(); ++it) {
            const std::string key = it->first.as<std::string>();
            int code = 0;
            try { code = std::stoi(key); } catch (...) { continue; }
            const YAML::Node v = it->second;
            WarningInfo wi;
            wi.level   = v["level"]   ? v["level"].as<std::string>()   : "FAULT";
            wi.name    = v["name"]    ? v["name"].as<std::string>()    : "";
            wi.content = v["content"] ? v["content"].as<std::string>() : "";
            warning_code_table_[code] = wi;
        }
        std::cout << "[ros1_slam] warning_code table loaded: " << warning_code_table_.size()
                  << " codes from " << path << std::endl;
        // 自定义码 (json 未定义, 需在 app 侧同步登记): 位姿-里程计长期背离
        {
            WarningInfo wi;
            wi.level = "WARNING";
            wi.name = "pose odom inconsistent";
            wi.content = "slam pose deviates from wheel odometry (possible wrong lock)";
            warning_code_table_[20003505] = wi;
        }
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("[ros1_slam] warning_code.json parse error (" << e.what()
                        << "): " << path);
    }
}

void Ros1SlamInterface::addWarningCode(int code) {
    std::lock_guard<std::mutex> lk(warning_code_mtx_);
    if (std::find(pending_warning_codes_.begin(), pending_warning_codes_.end(), code) ==
        pending_warning_codes_.end()) {
        pending_warning_codes_.push_back(code);
    }
}

void Ros1SlamInterface::setWarningSuppression(bool on) {
    warning_code_start_task_ = on;
    warning_code_suppress_ticks_ = 0;
}

void Ros1SlamInterface::tickFreqWindow(bool is_lidar) {
    // legacy.cpp:620-641 (imu) 与 740-751 (lidar): 各自一个 1s 窗口, 窗口结束时
    // 若计数 <= 阈值就报对应码。阈值: imu <= 160 -> 20003002, lidar <= 6 -> 20003001。
    // odom 的窗口只打印不报警 (legacy.cpp:2706-2718) -> 我们**不发** 20003003。
    // 注意这套只能在"消息有来但偏慢"时触发: 回调不触发就没有窗口结束。
    const ros::Time now = ros::Time::now();
    FreqWindow& w = is_lidar ? lidar_freq_ : imu_freq_;
    if (w.start.isZero()) w.start = now;
    ++w.count;
    const double dt = (now - w.start).toSec();
    if (dt < 1.0) return;

    const int count = w.count;
    w.count = 0;
    w.start = now;

    if (is_lidar) {
        std::cout << "laser_fre_count_ is : " << count << std::endl;
        if (count <= 6) addWarningCode(20003001);
    } else {
        std::cout << "imu_fre_count_ is : " << count << std::endl;
        if (count <= 160) addWarningCode(20003002);
    }
}

void Ros1SlamInterface::timerPublishWarningCode(const ros::TimerEvent&) {
    // 抑制窗口 (legacy.cpp:459-468): 只在 start_task 之后才生效
    if (warning_code_start_task_) {
        ++warning_code_suppress_ticks_;
        if (warning_code_suppress_ticks_ < 100) return;
        warning_code_start_task_ = false;
        warning_code_suppress_ticks_ = 0;
    }

    std::vector<int> codes;
    {
        std::lock_guard<std::mutex> lk(warning_code_mtx_);
        codes.swap(pending_warning_codes_);
    }
    if (codes.empty()) return;   // 没有码就不发 (legacy.cpp:543)

    std::vector<WarningFault> faults;
    faults.reserve(codes.size());
    for (int c : codes) {
        WarningFault f;
        auto it = warning_code_table_.find(c);
        if (it != warning_code_table_.end()) {
            f.code = std::to_string(c);
            f.level = it->second.level;
            f.name = it->second.name;
            f.content = it->second.content;
        } else {
            // 旧实现未定义码的兜底 (json_warning_code.cpp 的 else 分支):
            // name="not define warning code", level="FAULT", code=begin_code_+998
            f.code = std::to_string(20003001 + 998);
            f.level = "FAULT";
            f.name = "not define warning code";
            f.content = "xz_3d_slam not define this warning code! code is " + std::to_string(c);
        }
        faults.push_back(f);
    }
    const std::string json = buildWarningJson(faults);
    std_msgs::String msg;
    msg.data = json;
    warning_code_puber_.publish(msg);
    std::cout << "timerPublishWarningCode, string_warning_codes is : " << json << std::endl;
}

// ---------------------------------------------------------------- [T7] clients

} // namespace rosiwit_slam_ros1
