/**
 * @file types.h
 * @brief FAST-LIO2 SLAM - 核心类型定义
 * @author AI Development Team
 * @date 2026-04-24
 *
 * 定义系统中的核心数据结构，包括状态向量、IMU数据、点云数据等
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
// Sophus SE3实现 (简化版，基于Eigen)
#include "sophus_se3.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <memory>
#include <deque>
#include <mutex>

namespace fast_lio2_slam {

// ============== 基础类型别名 ==============
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Quaterniond = Eigen::Quaterniond;
// Sophus常用类型别名（避免与Eigen冲突）
using SE3d = Sophus::SE3<double>;
using SO3d = Sophus::SO3<double>;

// PCL点类型别名
using PointType = pcl::PointXYZINormal;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;
using PointCloudConstPtr = pcl::PointCloud<PointType>::ConstPtr;

// ============== 状态向量 (24维) ==============
/**
 * @brief 系统状态向量
 *
 * 包含位置、姿态、速度、加速度偏置、陀螺仪偏置、重力、外参
 * 对应FAST-LIO2的状态定义
 */
struct State {
    // 位姿状态
    Vector3d position = Vector3d::Zero();          // 位置 (世界坐标系)
    Quaterniond rotation = Quaterniond::Identity(); // 姿态四元数
    Vector3d velocity = Vector3d::Zero();          // 速度

    // IMU偏置
    Vector3d acc_bias = Vector3d::Zero();          // 加速度计偏置
    Vector3d gyro_bias = Vector3d::Zero();         // 陀螺仪偏置

    // 重力向量
    Vector3d gravity = Vector3d(0, 0, -9.81);     // 重力 (世界坐标系Z轴向下)

    // LiDAR-IMU外参
    Vector3d ext_R = Vector3d::Zero();            // LiDAR-IMU旋转 (欧拉角)
    Vector3d ext_T = Vector3d::Zero();            // LiDAR-IMU平移

    // 时间戳
    double timestamp = 0.0;

    // 协方差矩阵 (24x24)
    Eigen::Matrix<double, 24, 24> covariance =
        Eigen::Matrix<double, 24, 24>::Identity() * 1e-6;

    // ============ 成员函数 ============

    State() = default;

    /**
     * @brief 转换为SE3姿态
     */
    SE3d toSE3() const {
        return SE3d(rotation, position);
    }

    /**
     * @brief 转换为4x4齐次变换矩阵
     */
    Matrix4d toMatrix() const {
        Matrix4d T = Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        T.block<3, 1>(0, 3) = position;
        return T;
    }

    /**
     * @brief 从SE3设置状态
     */
    void fromSE3(const SE3d& se3) {
        position = se3.translation();
        rotation = se3.rotation().quaternion();
    }

    /**
     * @brief 获取旋转矩阵
     */
    Matrix3d getRotationMatrix() const {
        return rotation.toRotationMatrix();
    }

    /**
     * @brief 获取状态向量 (用于IEKF)
     */
    Eigen::Matrix<double, 24, 1> toStateVector() const {
        Eigen::Matrix<double, 24, 1> vec;
        vec.setZero();
        vec.block<3, 1>(0, 0) = position;
        vec.block<3, 1>(3, 0) = Vector3d(rotation.x(), rotation.y(), rotation.z()); // 四元数虚部
        vec.block<3, 1>(6, 0) = velocity;
        vec.block<3, 1>(9, 0) = acc_bias;
        vec.block<3, 1>(12, 0) = gyro_bias;
        vec.block<3, 1>(15, 0) = gravity;
        vec.block<3, 1>(18, 0) = ext_R;
        vec.block<3, 1>(21, 0) = ext_T;
        return vec;
    }
};

// ============== IMU数据 ==============
/**
 * @brief IMU测量数据结构
 */
struct ImuData {
    double timestamp = 0.0;         // 时间戳 (秒)
    Vector3d acc = Vector3d::Zero();   // 加速度 (m/s^2)
    Vector3d gyro = Vector3d::Zero();  // 角速度 (rad/s)

    ImuData() = default;
    ImuData(double t, const Vector3d& a, const Vector3d& g)
        : timestamp(t), acc(a), gyro(g) {}
};

// ============== 点云数据 ==============
/**
 * @brief 点云数据结构
 */
struct PointCloudData {
    double timestamp = 0.0;         // 扫描开始时间戳
    PointCloudPtr cloud;            // 点云指针
    int scan_id = 0;                // 扫描ID (递增)

    PointCloudData() {
        cloud.reset(new pcl::PointCloud<PointType>());
    }

    PointCloudData(double t, PointCloudPtr c, int id = 0)
        : timestamp(t), cloud(c), scan_id(id) {}
};

// ============== 里程计数据 ==============
/**
 * @brief 外部里程计数据结构
 */
struct OdomData {
    double timestamp = 0.0;
    Vector3d position = Vector3d::Zero();
    Quaterniond rotation = Quaterniond::Identity();
    Vector3d linear_velocity = Vector3d::Zero();
    Vector3d angular_velocity = Vector3d::Zero();

    // 协方差 (6x6: x,y,z,roll,pitch,yaw)
    Eigen::Matrix<double, 6, 6> covariance =
        Eigen::Matrix<double, 6, 6>::Identity() * 0.01;

    OdomData() = default;

    SE3d toSE3() const {
        return SE3d(rotation, position);
    }
};

// ============== IMU缓冲区 ==============
/**
 * @brief 线程安全的IMU数据缓冲区
 */
class ImuBuffer {
public:
    ImuBuffer(size_t max_size = 1000) : max_size_(max_size) {}

    void push(const ImuData& imu) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.size() >= max_size_) {
            buffer_.pop_front();
        }
        buffer_.push_back(imu);
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.empty();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size();
    }

    /**
     * @brief 获取时间范围内的IMU数据
     */
    std::vector<ImuData> getImuInRange(double t_start, double t_end) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<ImuData> result;

        for (const auto& imu : buffer_) {
            if (imu.timestamp >= t_start && imu.timestamp <= t_end) {
                result.push_back(imu);
            }
        }
        return result;
    }

    /**
     * @brief 获取最新IMU数据
     */
    bool getLatest(ImuData& imu) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) return false;
        imu = buffer_.back();
        return true;
    }

    /**
     * @brief 获取指定时间最近的IMU数据
     */
    bool getNearest(double timestamp, ImuData& imu, double& dt) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) return false;

        double min_dt = std::numeric_limits<double>::max();
        bool found = false;

        for (const auto& data : buffer_) {
            double diff = std::abs(data.timestamp - timestamp);
            if (diff < min_dt) {
                min_dt = diff;
                imu = data;
                found = true;
            }
        }

        dt = min_dt;
        return found;
    }

private:
    mutable std::mutex mutex_;
    std::deque<ImuData> buffer_;
    size_t max_size_;
};

// ============== 特征点信息 ==============
/**
 * @brief 特征点结构 (用于ikd-tree)
 */
struct PointWithInfo {
    PointType point;
    int feature_type = 0;   // 0: 未分类, 1: 平面点, 2: 边缘点
    int scan_id = 0;
    double curvature = 0.0;
    bool is_selected = false;
};

// ============== 位姿图节点 ==============
/**
 * @brief 位姿图节点
 */
struct PoseNode {
    int id;
    SE3d pose;
    double timestamp;
    int scan_id;
    bool is_loop_closure = false;

    PoseNode() : id(-1), timestamp(0), scan_id(0) {}
    PoseNode(int i, const SE3d& p, double t, int sid)
        : id(i), pose(p), timestamp(t), scan_id(sid) {}
};

// ============== 闭环约束 ==============
/**
 * @brief 闭环检测约束
 */
struct LoopConstraint {
    int from_id;
    int to_id;
    SE3d relative_pose;
    double score;
    double yaw_diff;

    LoopConstraint() : from_id(-1), to_id(-1), score(0), yaw_diff(0) {}
    LoopConstraint(int f, int t, const SE3d& rel, double s, double y)
        : from_id(f), to_id(t), relative_pose(rel), score(s), yaw_diff(y) {}
};

// ============== 参数配置结构体 ==============
/**
 * @brief IMU参数配置
 */
struct ImuParams {
    double acc_noise = 0.1;          // 加速度计噪声 (m/s^2)
    double gyro_noise = 0.01;        // 陀螺仪噪声 (rad/s)
    double acc_bias_noise = 0.0001;  // 加速度计偏置噪声
    double gyro_bias_noise = 0.00001; // 陀螺仪偏置噪声
    double gravity_magnitude = 9.81; // 重力大小

    Vector3d acc_bias_init = Vector3d::Zero();
    Vector3d gyro_bias_init = Vector3d::Zero();
};

/**
 * @brief LiDAR参数配置
 */
struct LidarParams {
    int scan_line = 16;              // 扫描线数
    double scan_period = 0.1;        // 扫描周期 (秒)
    double max_range = 100.0;        // 最大距离 (米)
    double min_range = 0.5;          // 最小距离 (米)
    double max_angle = 30.0;         // 最大扫描角度 (度)

    // 外参: LiDAR到IMU
    Vector3d translation = Vector3d::Zero();
    Vector3d rotation_euler = Vector3d::Zero();

    // 点云预处理
    double voxel_size = 0.2;        // 体素滤波大小
    int min_points_per_scan = 100;  // 最小点数
};

/**
 * @brief IEKF参数配置
 */
struct IekfParams {
    int max_iterations = 5;          // IEKF最大迭代次数
    double converge_threshold = 0.001; // 收敛阈值

    // 测量噪声协方差
    double position_noise = 0.01;
    double rotation_noise = 0.01;

    // 地图管理
    double map_update_distance = 0.2; // 地图更新最小距离
    double map_leaf_size = 0.2;       // 地图降采样大小
};

/**
 * @brief 闭环检测参数
 */
struct LoopClosureParams {
    bool enable = true;
    double detection_rate = 0.5;     // 检测频率 (Hz)
    int min_interval = 50;           // 最小闭环间隔 (帧)
    double threshold = 0.3;          // 闭环阈值

    // Scan Context参数
    int ring_num = 20;
    int sector_num = 60;
    double max_range = 80.0;

    // GTSAM参数
    int max_iterations = 20;
    double relative_pose_noise = 0.1;
};

/**
 * @brief 里程计融合参数
 */
struct OdomFusionParams {
    bool enable = false;
    std::string fusion_mode = "loose";  // "loose" or "tight"

    // 松耦合参数
    double lidar_weight = 0.7;
    double odom_weight = 0.3;

    // 协方差
    double position_cov = 0.05;
    double rotation_cov = 0.02;
};

// ============== 全局定位参数 ==============

/**
 * @brief 定位状态枚举
 */
enum class LocalizationState {
    UNINITIALIZED = 0,   // 未初始化
    LOCALIZING    = 1,   // 正在定位
    LOCALIZED     = 2,   // 已定位
    LOST          = 3    // 丢失定位
};

/**
 * @brief 全局定位参数
 */
struct LocalizationParams {
    bool enable = true;
    std::string mode = "manual";  // "auto" or "manual"

    // Scan Context参数（粗定位）
    int scan_context_ring_num = 20;
    int scan_context_sector_num = 60;
    double scan_context_max_range = 80.0;
    double scan_context_dist_threshold = 0.3;
    int scan_context_candidate_count = 5;

    // 精配准参数
    std::string fine_alignment_method = "ndt";  // "ndt" or "icp"
    int fine_alignment_max_iterations = 50;
    double fine_alignment_convergence_threshold = 0.01;
    double fine_alignment_resolution = 1.0;
    double fine_alignment_voxel_size = 0.5;

    // 验证参数
    double validation_min_fitness_score = 0.7;
    double validation_min_inlier_ratio = 0.5;
    double validation_max_position_error = 2.0;
    double validation_max_rotation_error = 0.5;

    // 搜索参数
    int search_max_candidates = 10;
    bool use_initial_pose_hint = true;
};

/**
 * @brief 定位结果结构
 */
struct LocalizationResult {
    bool success = false;
    SE3d estimated_pose;
    double fitness_score = 0.0;
    double inlier_ratio = 0.0;
    int matched_keyframe_id = -1;
    LocalizationState state = LocalizationState::UNINITIALIZED;
    std::string error_message;

    // 时间信息
    double coarse_time_ms = 0.0;
    double fine_time_ms = 0.0;
    double total_time_ms = 0.0;

    LocalizationResult() = default;
};

// ============== 地图管理相关结构 ==============

/**
 * @brief 会话信息结构
 *
 * 用于多会话建图，记录每次建图会话的元数据
 */
struct SessionInfo {
    std::string session_id;           // 会话ID
    std::string name;                 // 会话名称
    double start_time = 0.0;          // 开始时间
    double end_time = 0.0;            // 结束时间
    int frame_count = 0;              // 帧数
    SE3d start_pose;                  // 起始位姿
    SE3d end_pose;                    // 结束位姿
    std::vector<int> submap_ids;      // 包含的子地图ID
    bool is_merged = false;           // 是否已合并

    SessionInfo() = default;

    SessionInfo(const std::string& id, const std::string& n)
        : session_id(id), name(n) {}
};

/**
 * @brief 地图元数据结构
 *
 * 用于描述地图的基本信息和质量指标
 */
struct MapMetadata {
    std::string map_name;             // 地图名称
    std::string version = "1.0";      // 版本号
    double created_time = 0.0;        // 创建时间
    double modified_time = 0.0;       // 修改时间
    int64_t total_points = 0;         // 总点数
    int total_submaps = 0;            // 总子地图数
    int total_sessions = 0;           // 总会话数

    // 地图范围
    Vector3d map_center = Vector3d::Zero();      // 地图中心
    Vector3d map_size = Vector3d::Zero();        // 地图尺寸

    // 传感器信息
    std::string lidar_type;           // 激光雷达类型
    std::string frame_id = "world";   // 坐标系ID

    // 精度信息
    double avg_point_density = 0.0;   // 平均点密度
    double map_quality_score = 0.0;   // 地图质量评分

    MapMetadata() = default;

    /**
     * @brief 计算地图覆盖面积
     */
    double computeCoverageArea() const {
        return map_size.x() * map_size.y();  // XY平面投影面积
    }
};

/**
 * @brief 地图统计信息结构
 *
 * 用于运行时统计地图的状态和性能指标
 */
struct MapStatistics {
    int64_t total_points = 0;         // 总点数
    int total_submaps = 0;            // 子地图数
    int active_submaps = 0;           // 活跃子地图数
    int total_sessions = 0;           // 会话数

    // 内存使用
    double memory_usage_mb = 0.0;     // 内存占用 (MB)
    double memory_limit_mb = 2048.0;  // 内存限制 (MB)

    // 性能指标
    double avg_insert_time_ms = 0.0;  // 平均插入时间
    double avg_query_time_ms = 0.0;   // 平均查询时间

    // 点云质量
    double avg_density = 0.0;         // 平均密度
    double min_density = 0.0;         // 最小密度
    double max_density = 0.0;         // 最大密度

    // 地图范围
    Vector3d min_bound = Vector3d::Zero();
    Vector3d max_bound = Vector3d::Zero();

    // 更新统计
    int total_updates = 0;            // 总更新次数
    double last_update_time = 0.0;    // 最后更新时间

    MapStatistics() = default;

    /**
     * @brief 计算内存使用百分比
     */
    double memoryUsagePercent() const {
        return (memory_usage_mb / memory_limit_mb) * 100.0;
    }

    /**
     * @brief 是否内存紧张
     */
    bool isMemoryCritical() const {
        return memoryUsagePercent() > 80.0;
    }
};

/**
 * @brief 子地图信息结构（用于持久化）
 */
struct SubmapInfo {
    int id;                           // 子地图ID
    SE3d center_pose;                 // 中心位姿
    int point_count;                  // 点数
    std::vector<int> frame_ids;       // 包含的帧ID
    double timestamp_start;           // 开始时间戳
    double timestamp_end;             // 结束时间戳
    Vector3d min_bound;               // 边界最小值
    Vector3d max_bound;               // 边界最大值
    std::vector<float> descriptor;    // 描述子（用于回环检测）

    SubmapInfo() : id(-1), point_count(0),
                   timestamp_start(0), timestamp_end(0),
                   min_bound(Vector3d::Zero()), max_bound(Vector3d::Zero()) {}
};

} // namespace fast_lio2_slam// 为pcl::PointXYZINormal添加比较运算符 (用于std::set等容器)
namespace pcl {
inline bool operator<(const PointXYZINormal& a, const PointXYZINormal& b) {
    if (std::abs(a.x - b.x) > 1e-6) return a.x < b.x;
    if (std::abs(a.y - b.y) > 1e-6) return a.y < b.y;
    if (std::abs(a.z - b.z) > 1e-6) return a.z < b.z;
    return false;
}

inline bool operator==(const PointXYZINormal& a, const PointXYZINormal& b) {
    return std::abs(a.x - b.x) < 1e-6 &&
           std::abs(a.y - b.y) < 1e-6 &&
           std::abs(a.z - b.z) < 1e-6;
}
}  // namespace pcl