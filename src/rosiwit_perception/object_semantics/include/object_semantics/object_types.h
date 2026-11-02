// 3D 语义目标核心数据结构（ROS 无关）
//
// 设计要点（见 docs/superpowers/plans/2026-11-01-person-3d-semantics.md）:
//   * 类别是一等公民: 检测框/观测/跟踪结果都带 class_id + class_name,
//     首类为 person, 后续新增 car/bicycle/... 不需要改核心结构
//   * 位置统一用"目标中心"（人取躯干中心，步态摆动小），另给 ground（z=0 投影）
//   * 速度输出两套: velocity_base（相对机器人，避障用）/ velocity_world（世界系，界面用）
//   * 跟踪在 world 系做 KF，输出时再转到 base_link —— 这样"静止的目标"世界速度就是 0,
//     不会因为机器人前进而被当成目标在动
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rosiwit_perception {

/// 相机内参（彩色与深度各自一套；深度未对齐时必须按内参映射）
struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  int width = 0;
  int height = 0;

  bool valid() const { return fx > 1.0 && fy > 1.0 && width > 0 && height > 0; }
};

/// base_link <- camera_optical 的外参
struct Extrinsics {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  bool valid() const { return R.allFinite() && t.allFinite(); }
};

/// 一张 16UC1 深度图（单位 mm），不持有 ROS 类型
struct DepthImage {
  int width = 0;
  int height = 0;
  float scale_mm = 1.0f;  // 原始值 * scale_mm = 毫米（16UC1 一般为 1.0）
  std::vector<uint16_t> data;

  bool valid() const { return width > 0 && height > 0 && data.size() == static_cast<size_t>(width) * height; }
  uint16_t at(int u, int v) const { return data[static_cast<size_t>(v) * width + u]; }
};

/// 2D 检测框（彩色像素坐标）+ 类别
struct DetectionBox {
  double u0 = 0.0, v0 = 0.0, u1 = 0.0, v1 = 0.0;
  double score = 0.0;
  int class_id = 0;            ///< 检测网络输出的类别索引（COCO: person=0）
  std::string class_name;      ///< 业务名，由节点按 class_id 映射；核心不假设类别集合

  double width() const { return u1 - u0; }
  double height() const { return v1 - v0; }
  double cx() const { return 0.5 * (u0 + u1); }
  double cy() const { return 0.5 * (v0 + v1); }
};

/// 语义类别定义（检测类别索引 -> 业务名 + 尺寸先验）
///
/// 尺寸先验只用于"合理性降置信", 不参与定位计算；人的身高区间和车辆不同,
/// 新增类别时在配置里给出各自的区间即可。
struct SemanticClass {
  int id = 0;
  std::string name = "person";
  double min_height_m = 0.8;
  double max_height_m = 2.2;
  /// 该类别的速度合理性上限（m/s）: 人 2.5, 车辆要更大
  double max_speed_mps = 2.5;

  bool valid() const { return !name.empty(); }
};

struct DepthParams {
  double roi_w_ratio = 0.6;    ///< 框内水平中央比例
  double roi_v_min = 0.15;     ///< 框内起始高度比例（避开头部背景）
  double roi_v_max = 0.85;     ///< 框内结束高度比例（避开地面）
  int depth_min_mm = 100;
  int depth_max_mm = 8000;
  double min_valid_ratio = 0.30;
  /// 深度噪声模型 σ_z = a*z^2 + b（米），现场按附录 A 拟合
  double sigma_z_a = 0.0025;
  double sigma_z_b = 0.010;
  /// 横向误差（像素量化）：σ_lat = z * px_sigma / fx
  double px_sigma = 0.5;
  /// 地面投影的有效距离上限（米）
  double ground_max_range = 8.0;
};

/// 单帧单个检测的 3D 观测（跟踪器输入）
struct ObjectObservation {
  DetectionBox box;           ///< 含 class_id / class_name
  bool depth_valid = false;   ///< ROI 深度有效率够
  bool ambiguous = false;     ///< 同框多峰（多人重叠），MVP 只标记
  double distance = 0.0;      ///< 目标中心到机器人原点的水平距离（m）
  double depth_median_m = 0.0;
  double depth_near_m = 0.0;  ///< ROI 10 分位
  double depth_far_m = 0.0;   ///< ROI 90 分位
  double valid_ratio = 0.0;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();  ///< base_link 下目标中心 (x,y)
  double z = 0.0;                                      ///< 目标中心高度（base_link）
  Eigen::Vector2d ground = Eigen::Vector2d::Zero();    ///< z=0 地面投影 (x,y)
  Eigen::Matrix2d cov_xy = Eigen::Matrix2d::Identity() * 0.04;
  double confidence = 0.0;
  double max_speed_mps = 2.5;  ///< 来自 SemanticClass，用于 velocity_valid
};

struct TrackerParams {
  double assoc_gate_chi2 = 9.0;
  double assoc_max_dist = 1.0;
  int confirm_hits = 3;
  int tentative_max_miss = 2;
  double max_coast_s = 0.5;
  double process_accel = 0.5;      ///< 加速度谱密度（m/s²）
  double velocity_min_age_s = 0.3;
  double velocity_max_mps = 2.5;
  /// 关联时测量噪声下限，避免深度完美时 R 太小把新目标吸走
  double meas_sigma_floor = 0.05;
  int max_tracks = 64;
  /// 不同类别之间不关联（人不会变成车）；同 id 才算候选
  bool assoc_require_same_class = true;
};

/// 一个已跟踪目标（输出，base_link 系）
struct ObjectTrack {
  uint32_t id = 0;
  int class_id = 0;
  std::string class_name = "person";
  bool confirmed = false;
  bool coasting = false;       ///< 本帧没有观测，靠预测续命
  bool depth_valid = false;
  bool velocity_valid = false;
  bool in_fov = true;

  Eigen::Vector2d position = Eigen::Vector2d::Zero();  ///< 目标中心 (x,y)
  double z = 0.0;
  Eigen::Vector2d ground = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity_base = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity_world = Eigen::Vector2d::Zero();
  Eigen::Matrix2d cov_xy = Eigen::Matrix2d::Identity() * 0.04;
  Eigen::Matrix2d cov_velocity = Eigen::Matrix2d::Identity() * 0.25;

  double distance = 0.0;     ///< 最近距离（m，目标中心水平距离）
  double confidence = 0.0;
  double age = 0.0;          ///< 秒
  DetectionBox box;          ///< 最近一次观测到的框（coasting 时沿用上一次）
};

/// 机器人自身状态（时间戳对齐后）
struct RobotState {
  bool valid = false;
  Eigen::Vector3d position_world = Eigen::Vector3d::Zero();  ///< base_link 在 odom/world 的位置
  Eigen::Matrix3d R_world_base = Eigen::Matrix3d::Identity();///< world <- base_link
  Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();  ///< 机器人世界系速度
};

}  // namespace rosiwit_perception
