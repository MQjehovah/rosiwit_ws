// 前置深度相机 3D 语义目标 —— ROS1 薄节点
//
// 实施方案: docs/superpowers/plans/2026-11-01-person-3d-semantics.md
// 算法全部在 object_semantics 核心库（无 ROS 依赖, 有单测）; 本文件只负责:
//   订阅 color/depth/camera_info/tf/odom -> 检测(外部输入或 RKNN) -> 深度关联 -> 跟踪 -> 发布
//
// 类别不写死: classes 参数给出"检测类别索引 -> 业务名 + 尺寸先验", 首类为 person,
// 后续加 car/bicycle 只改配置, 不改核心与消息。
//
// 话题:
//   /perception/objects                  SemanticObjectArray（全部类别）
//   /perception/persons                  SemanticObjectArray（仅 person 子集, 供导航/兼容）
//   /perception/objects/markers          visualization_msgs/MarkerArray
//   /perception/objects/image            sensor_msgs/Image（标注图, publish_image 打开时）
//   /perception/objects/status           std_msgs/String (JSON)
//   /perception/objects/log              std_msgs/String (JSONL, 每目标每帧一行)
//   /perception/objects/detections_2d    ObjectDetection2DArray（detector=external 时的输入）
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "object_semantics/depth_associator.h"
#include "object_semantics/detector_postprocess.h"
#include "object_semantics/multi_object_tracker.h"
#include "rosiwit_perception_ros1/ObjectDetection2DArray.h"
#include "rosiwit_perception_ros1/SemanticObject.h"
#include "rosiwit_perception_ros1/SemanticObjectArray.h"

#ifdef HAVE_RKNN
#include "rknn_api.h"
#endif

namespace {

using rosiwit_perception::CameraIntrinsics;
using rosiwit_perception::DepthAssociator;
using rosiwit_perception::DepthImage;
using rosiwit_perception::DetectionBox;
using rosiwit_perception::Extrinsics;
using rosiwit_perception::Letterbox;
using rosiwit_perception::MultiObjectTracker;
using rosiwit_perception::ObjectObservation;
using rosiwit_perception::ObjectTrack;
using rosiwit_perception::RobotState;
using rosiwit_perception::SemanticClass;

/// 检测后端: RKNN 模型 / 外部 2D 框（无 NPU 时的 bring-up 与回放）
class DetectorBackend {
 public:
  virtual ~DetectorBackend() = default;
  virtual bool init(ros::NodeHandle& pnh) = 0;
  virtual bool ready() const = 0;
  virtual bool detect(const cv::Mat& bgr, const ros::Time& stamp,
                      std::vector<DetectionBox>& boxes) = 0;
  virtual const char* name() const = 0;

  /// 关心的类别索引（配置里 classes 的 id）；空表示全部
  void setWantedClassIds(const std::vector<int>& ids) { wanted_class_ids_ = ids; }

 protected:
  bool wanted(int class_id) const {
    return wanted_class_ids_.empty() ||
           std::find(wanted_class_ids_.begin(), wanted_class_ids_.end(), class_id) !=
               wanted_class_ids_.end();
  }

  std::vector<int> wanted_class_ids_;
};

/// 外部检测: 订阅 ObjectDetection2DArray（同一帧时间戳, 容差 external_max_age_s）
class ExternalDetector : public DetectorBackend {
 public:
  bool init(ros::NodeHandle& pnh) override {
    pnh.param<std::string>("external_detections_topic", topic_,
                           "/perception/objects/detections_2d");
    pnh.param<double>("external_max_age_s", max_age_, 0.2);
    sub_ = pnh.subscribe(topic_, 1, &ExternalDetector::callback, this);
    ROS_INFO("[object_semantics] detector=external (%s)", topic_.c_str());
    return true;
  }
  bool ready() const override { return true; }
  bool detect(const cv::Mat&, const ros::Time& stamp, std::vector<DetectionBox>& boxes) override {
    boxes.clear();
    // 外部检测可能比彩色慢一帧, 允许 max_age_ 内的最近结果（bring-up/回放用）
    if (!have_ || std::fabs((last_stamp_ - stamp).toSec()) > max_age_) {
      return false;
    }
    for (const auto& box : last_boxes_) {
      if (wanted(box.class_id)) {
        boxes.push_back(box);
      }
    }
    return true;
  }
  const char* name() const override { return "external"; }

 private:
  void callback(const rosiwit_perception_ros1::ObjectDetection2DArray::ConstPtr& msg) {
    last_boxes_.clear();
    for (const auto& det : msg->detections) {
      DetectionBox box;
      box.u0 = det.x_min;
      box.v0 = det.y_min;
      box.u1 = det.x_max;
      box.v1 = det.y_max;
      box.score = det.score;
      box.class_id = det.class_id;
      box.class_name = det.class_name;
      last_boxes_.push_back(box);
    }
    last_stamp_ = msg->header.stamp;
    have_ = true;
  }

  std::string topic_;
  double max_age_ = 0.2;
  ros::Subscriber sub_;
  std::vector<DetectionBox> last_boxes_;
  ros::Time last_stamp_;
  bool have_ = false;
};

#ifdef HAVE_RKNN
/// RKNN YOLOv8（板端）。需 rknn_api.h + librknnrt + .rknn 模型。
/// 模型可以是 person-only (C=5) 或任意 COCO 子集 (C=4+nc)，类别由解码给出。
class RknnDetector : public DetectorBackend {
 public:
  ~RknnDetector() override {
    if (ctx_ != 0) rknn_destroy(ctx_);
  }
  bool init(ros::NodeHandle& pnh) override {
    pnh.param<std::string>("model_path", model_path_, "");
    pnh.param<int>("input_size", input_size_, 416);
    pnh.param<double>("conf_threshold", conf_, 0.35);
    pnh.param<double>("nms_iou", nms_iou_, 0.45);
    pnh.param<int>("max_objects", max_objects_, 20);
    if (model_path_.empty()) {
      ROS_ERROR("[object_semantics] model_path 为空");
      return false;
    }
    FILE* fp = fopen(model_path_.c_str(), "rb");
    if (fp == nullptr) {
      ROS_ERROR("[object_semantics] 打不开模型: %s", model_path_.c_str());
      return false;
    }
    fseek(fp, 0, SEEK_END);
    const long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    model_.resize(static_cast<size_t>(size));
    if (fread(model_.data(), 1, model_.size(), fp) != model_.size()) {
      fclose(fp);
      ROS_ERROR("[object_semantics] 读模型失败");
      return false;
    }
    fclose(fp);
    if (rknn_init(&ctx_, model_.data(), model_.size(), 0, nullptr) != 0) {
      ROS_ERROR("[object_semantics] rknn_init 失败（librknnrt 版本与模型不匹配？）");
      ctx_ = 0;
      return false;
    }
    ready_ = true;
    ROS_INFO("[object_semantics] detector=rknn (%s, %d)", model_path_.c_str(), input_size_);
    return true;
  }
  bool ready() const override { return ready_; }

  bool detect(const cv::Mat& bgr, const ros::Time&, std::vector<DetectionBox>& boxes) override {
    boxes.clear();
    if (!ready_) return false;
    const Letterbox lb = Letterbox::fit(bgr.cols, bgr.rows, input_size_, input_size_);
    cv::Mat in;
    cv::resize(bgr, in, cv::Size(static_cast<int>(bgr.cols * lb.scale),
                                 static_cast<int>(bgr.rows * lb.scale)));
    cv::Mat canvas(input_size_, input_size_, CV_8UC3, cv::Scalar(114, 114, 114));
    in.copyTo(canvas(cv::Rect(lb.pad_x, lb.pad_y, in.cols, in.rows)));

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = static_cast<uint32_t>(canvas.total() * canvas.elemSize());
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = canvas.data;
    if (rknn_inputs_set(ctx_, 1, inputs) != 0 || rknn_run(ctx_, nullptr) != 0) {
      return false;
    }
    // 输出张量数以模型为准（不能假定 3 个）
    rknn_input_output_num io_num;
    memset(&io_num, 0, sizeof(io_num));
    if (rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num)) != 0 ||
        io_num.n_output == 0) {
      ROS_ERROR_THROTTLE(5.0, "[object_semantics] rknn_query 输出数量失败");
      return false;
    }
    const uint32_t out_num = io_num.n_output;
    std::vector<rknn_output> outputs(out_num);
    memset(outputs.data(), 0, outputs.size() * sizeof(rknn_output));
    for (auto& o : outputs) o.want_float = 1;
    if (rknn_outputs_get(ctx_, out_num, outputs.data(), nullptr) != 0) {
      return false;
    }
    // 从输出属性取 [1, C, N] / [1, N, C]，避免按整除猜通道数（84*8400 也能被 5 整除）
    rknn_tensor_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.index = 0;
    int channels = 0;
    int num_boxes = 0;
    bool row_major = false;
    if (rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)) == 0 &&
        attr.n_dims == 3) {
      // [1, C, N] 或 [1, N, C]; 通道数总是较小的一维（5 或 4+nc << N）
      const uint32_t d1 = attr.dims[1];
      const uint32_t d2 = attr.dims[2];
      if (d1 >= 5 && d2 >= 5) {
        if (d1 <= d2) {
          channels = static_cast<int>(d1);
          num_boxes = static_cast<int>(d2);
          row_major = false;
        } else {
          channels = static_cast<int>(d2);
          num_boxes = static_cast<int>(d1);
          row_major = true;
        }
      }
    }
    const size_t total = outputs[0].size / sizeof(float);
    if (channels < 5 || static_cast<size_t>(channels) * num_boxes != total) {
      rknn_outputs_release(ctx_, out_num, outputs.data());
      ROS_ERROR_THROTTLE(5.0,
                        "[object_semantics] 无法识别 RKNN 输出形状（%zu 个 float），"
                        "期望 YOLOv8 [1,C,N] 或 [1,N,C]", total);
      return false;
    }
    const float* data = static_cast<const float*>(outputs[0].buf);
    boxes = rosiwit_perception::decodeDetections(
        std::vector<float>(data, data + total), channels, num_boxes,
        row_major, static_cast<float>(conf_), static_cast<float>(nms_iou_), max_objects_, lb,
        wanted_class_ids_);
    rknn_outputs_release(ctx_, out_num, outputs.data());
    return true;
  }
  const char* name() const override { return "rknn"; }

 private:
  rknn_context ctx_ = 0;
  std::vector<uint8_t> model_;
  std::string model_path_;
  int input_size_ = 416;
  double conf_ = 0.35;
  double nms_iou_ = 0.45;
  int max_objects_ = 20;
  bool ready_ = false;
};
#else
/// 没编进 RKNN: 启动即报错（方案 §5.2 要求不静默降级到 CPU）
class RknnDetector : public DetectorBackend {
 public:
  bool init(ros::NodeHandle&) override {
    ROS_FATAL("[object_semantics] 未编译 RKNN 支持（缺 rknn_api.h/librknnrt），不能使用 detector=rknn");
    return false;
  }
  bool ready() const override { return false; }
  bool detect(const cv::Mat&, const ros::Time&, std::vector<DetectionBox>&) override { return false; }
  const char* name() const override { return "rknn(missing)"; }
};
#endif

}  // namespace

class ObjectSemanticsNode {
 public:
  ObjectSemanticsNode() : nh_(), pnh_("~"), tf_listener_() {
    loadParams();
    loadClasses();
    if (!initDetector()) {
      ros::shutdown();
      return;
    }
    initPublishers();
    initSubscribers();
    ROS_INFO("[object_semantics] ready: detector=%s classes=%zu base=%s color=%s depth=%s",
             detector_->name(), classes_.size(), base_frame_.c_str(), color_frame_.c_str(),
             depth_frame_.c_str());
  }

  void spin() {
    ros::Rate rate(30);
    while (ros::ok()) {
      ros::spinOnce();
      processIfReady();
      rate.sleep();
    }
  }

 private:
  void loadParams() {
    pnh_.param<std::string>("detector", detector_type_, "rknn");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("color_frame", color_frame_, "camera_01_color_optical_frame");
    pnh_.param<std::string>("depth_frame", depth_frame_, "camera_01_depth_optical_frame");
    pnh_.param<std::string>("color_topic", color_topic_, "/camera_01/color/image_raw");
    pnh_.param<std::string>("depth_topic", depth_topic_, "/camera_01/depth/image_raw");
    pnh_.param<std::string>("color_info_topic", color_info_topic_, "/camera_01/color/camera_info");
    pnh_.param<std::string>("depth_info_topic", depth_info_topic_, "/camera_01/depth/camera_info");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    pnh_.param<double>("depth_sync_tolerance", depth_tolerance_, 0.05);
    pnh_.param<bool>("publish_image", publish_image_, true);
    pnh_.param<bool>("publish_markers", publish_markers_, true);
    pnh_.param<bool>("publish_log", publish_log_, true);
    pnh_.param<bool>("publish_persons_alias", publish_persons_alias_, true);
    pnh_.param<double>("max_frame_gap_s", max_frame_gap_, 0.5);

    pnh_.param<double>("roi_w_ratio", depth_params_.roi_w_ratio, 0.6);
    pnh_.param<double>("roi_v_min", depth_params_.roi_v_min, 0.15);
    pnh_.param<double>("roi_v_max", depth_params_.roi_v_max, 0.85);
    pnh_.param<int>("depth_min_mm", depth_params_.depth_min_mm, 100);
    pnh_.param<int>("depth_max_mm", depth_params_.depth_max_mm, 8000);
    pnh_.param<double>("min_valid_ratio", depth_params_.min_valid_ratio, 0.30);

    pnh_.param<double>("assoc_gate_chi2", tracker_params_.assoc_gate_chi2, 9.0);
    pnh_.param<double>("assoc_max_dist", tracker_params_.assoc_max_dist, 1.0);
    pnh_.param<int>("confirm_hits", tracker_params_.confirm_hits, 3);
    pnh_.param<double>("max_coast_s", tracker_params_.max_coast_s, 0.5);
    pnh_.param<double>("process_accel", tracker_params_.process_accel, 0.5);
    pnh_.param<double>("velocity_min_age_s", tracker_params_.velocity_min_age_s, 0.3);
    pnh_.param<double>("velocity_max_mps", tracker_params_.velocity_max_mps, 2.5);
    tracker_.configure(tracker_params_);
  }

  /// classes: [{id, name, min_height, max_height}, ...]；缺省为 person(0)
  void loadClasses() {
    XmlRpc::XmlRpcValue v;
    if (!pnh_.getParam("classes", v) || v.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        v.size() == 0) {
      SemanticClass person;
      person.id = 0;
      person.name = "person";
      person.min_height_m = 0.8;
      person.max_height_m = 2.2;
      classes_.push_back(person);
      ROS_WARN("[object_semantics] 未配置 classes, 默认 person(0) 0.8-2.2 m");
      return;
    }
    for (int i = 0; i < v.size(); ++i) {
      if (v[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_WARN("[object_semantics] classes[%d] 不是 struct, 跳过", i);
        continue;
      }
      SemanticClass cls;
      if (v[i].hasMember("id")) {
        cls.id = static_cast<int>(v[i]["id"]);
      } else {
        cls.id = i;
      }
      if (v[i].hasMember("name")) {
        cls.name = static_cast<std::string>(v[i]["name"]);
      } else {
        ROS_WARN("[object_semantics] classes[%d] 缺 name, 跳过", i);
        continue;
      }
      if (v[i].hasMember("min_height")) {
        cls.min_height_m = static_cast<double>(v[i]["min_height"]);
      }
      if (v[i].hasMember("max_height")) {
        cls.max_height_m = static_cast<double>(v[i]["max_height"]);
      }
      if (v[i].hasMember("max_speed")) {
        cls.max_speed_mps = static_cast<double>(v[i]["max_speed"]);
      }
      if (v[i].hasMember("surface_offset")) {
        cls.surface_offset_m = static_cast<double>(v[i]["surface_offset"]);
      }
      classes_.push_back(cls);
    }
    if (classes_.empty()) {
      SemanticClass person;
      person.id = 0;
      person.name = "person";
      classes_.push_back(person);
    }
    for (const auto& cls : classes_) {
      ROS_INFO("[object_semantics] class %d = %s (h %.2f-%.2f m, offset %.2f m, vmax %.1f m/s)",
               cls.id, cls.name.c_str(), cls.min_height_m, cls.max_height_m,
               cls.surface_offset_m, cls.max_speed_mps);
    }
  }

  bool initDetector() {
    if (detector_type_ == "external") {
      detector_.reset(new ExternalDetector());
    } else {
      detector_.reset(new RknnDetector());
    }
    detector_->setWantedClassIds(wantedClassIds());
    if (!detector_->init(pnh_)) {
      ROS_FATAL("[object_semantics] detector init failed; node will not start");
      return false;
    }
    return true;
  }

  std::vector<int> wantedClassIds() const {
    std::vector<int> ids;
    ids.reserve(classes_.size());
    for (const auto& cls : classes_) {
      ids.push_back(cls.id);
    }
    return ids;
  }

  const SemanticClass* findClass(int class_id) const {
    for (const auto& cls : classes_) {
      if (cls.id == class_id) {
        return &cls;
      }
    }
    return nullptr;
  }

  int personClassId() const {
    for (const auto& cls : classes_) {
      if (cls.name == "person") {
        return cls.id;
      }
    }
    return -1;
  }

  void initPublishers() {
    objects_pub_ = nh_.advertise<rosiwit_perception_ros1::SemanticObjectArray>(
        "/perception/objects", 10);
    persons_pub_ = nh_.advertise<rosiwit_perception_ros1::SemanticObjectArray>(
        "/perception/persons", 10);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/perception/objects/markers", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/perception/objects/status", 10);
    log_pub_ = nh_.advertise<std_msgs::String>("/perception/objects/log", 100);
    image_pub_ = nh_.advertise<sensor_msgs::Image>("/perception/objects/image", 10);
  }

  void initSubscribers() {
    color_info_sub_ = nh_.subscribe(color_info_topic_, 1, &ObjectSemanticsNode::colorInfoCB, this);
    depth_info_sub_ = nh_.subscribe(depth_info_topic_, 1, &ObjectSemanticsNode::depthInfoCB, this);
    depth_sub_ = nh_.subscribe(depth_topic_, 1, &ObjectSemanticsNode::depthCB, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &ObjectSemanticsNode::odomCB, this);
    color_sub_ = nh_.subscribe(color_topic_, 1, &ObjectSemanticsNode::colorCB, this);
  }

  // ---------------- callbacks ----------------
  void colorInfoCB(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    color_info_ = *msg;
    color_info_valid_ = true;
  }
  void depthInfoCB(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    depth_info_ = *msg;
    depth_info_valid_ = true;
  }
  void depthCB(const sensor_msgs::Image::ConstPtr& msg) {
    const bool u16 = msg->encoding == "16UC1" || msg->encoding == "mono16";
    const bool f32 = msg->encoding == "32FC1";
    if (!u16 && !f32) {
      ROS_ERROR_THROTTLE(5.0,
                         "[object_semantics] depth encoding 必须是 16UC1 或 32FC1, got %s",
                         msg->encoding.c_str());
      return;
    }
    depth_img_.width = static_cast<int>(msg->width);
    depth_img_.height = static_cast<int>(msg->height);
    depth_img_.scale_mm = 1.0f;
    depth_img_.data.resize(static_cast<size_t>(msg->width) * msg->height);

    if (u16) {
      const size_t row_bytes = static_cast<size_t>(msg->width) * sizeof(uint16_t);
      if (msg->step == row_bytes) {
        memcpy(depth_img_.data.data(), msg->data.data(), row_bytes * msg->height);
      } else {
        for (uint32_t v = 0; v < msg->height; ++v) {
          memcpy(depth_img_.data.data() + static_cast<size_t>(v) * msg->width,
                 msg->data.data() + static_cast<size_t>(v) * msg->step, row_bytes);
        }
      }
    } else {
      // 32FC1 = 米（Gazebo openni / 部分实机驱动）: 转成毫米，0/NaN/Inf 记空洞
      for (uint32_t v = 0; v < msg->height; ++v) {
        const float* src =
            reinterpret_cast<const float*>(msg->data.data() + static_cast<size_t>(v) * msg->step);
        uint16_t* dst = depth_img_.data.data() + static_cast<size_t>(v) * msg->width;
        for (uint32_t u = 0; u < msg->width; ++u) {
          const float m = src[u];
          if (!std::isfinite(m) || m <= 0.0f) {
            dst[u] = 0;
          } else {
            dst[u] = static_cast<uint16_t>(std::min(65535.0f, m * 1000.0f));
          }
        }
      }
    }
    depth_stamp_ = msg->header.stamp;
    depth_ready_ = true;
  }
  void odomCB(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_ = *msg;
    odom_valid_ = true;
  }
  void colorCB(const sensor_msgs::Image::ConstPtr& msg) {
    try {
      // toCvCopy 让 Mat 自己持有像素；toCvShare 的临时 CvImagePtr 析构后数据会悬空
      last_color_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR_THROTTLE(5.0, "[object_semantics] cv_bridge: %s", e.what());
      return;
    }
    last_color_stamp_ = msg->header.stamp;
    color_ready_ = true;
  }

  // ---------------- 主流程 ----------------
  void processIfReady() {
    if (!color_ready_ || !color_info_valid_ || !depth_info_valid_) return;
    if (last_color_stamp_ == processed_stamp_) return;
    if (depth_ready_ && std::fabs((last_color_stamp_ - depth_stamp_).toSec()) > max_frame_gap_) {
      depth_ready_ = false;  // 深度太久没更新, 这一帧标 depth_valid=false
    }
    processed_stamp_ = last_color_stamp_;
    const ros::WallTime t0 = ros::WallTime::now();

    // 1) 外参 base_link <- color_optical（按彩色时间戳查）
    Extrinsics base_from_color;
    if (!lookupExtrinsics(last_color_stamp_, base_from_color)) {
      ROS_WARN_THROTTLE(5.0, "[object_semantics] 缺少 TF %s <- %s",
                        base_frame_.c_str(), color_frame_.c_str());
      return;
    }
    CameraIntrinsics color;
    color.fx = color_info_.K[0];
    color.fy = color_info_.K[4];
    color.cx = color_info_.K[2];
    color.cy = color_info_.K[5];
    color.width = static_cast<int>(color_info_.width);
    color.height = static_cast<int>(color_info_.height);
    CameraIntrinsics depth;
    depth.fx = depth_info_.K[0];
    depth.fy = depth_info_.K[4];
    depth.cx = depth_info_.K[2];
    depth.cy = depth_info_.K[5];
    depth.width = static_cast<int>(depth_info_.width != 0 ? depth_info_.width : depth_img_.width);
    depth.height = static_cast<int>(depth_info_.height != 0 ? depth_info_.height : depth_img_.height);

    // 2) 检测。检测失败不等于"画面没目标", 但也不能沿用上一帧结果:
    //    按空观测继续走跟踪, 让已有 track 进入 coasting 并在 0.5 s 后自然消失。
    std::vector<DetectionBox> boxes;
    const bool detected = detector_->detect(last_color_, last_color_stamp_, boxes);
    if (!detected) {
      boxes.clear();
    }
    const bool depth_fresh = depth_ready_ &&
        std::fabs((last_color_stamp_ - depth_stamp_).toSec()) <= depth_tolerance_;

    // 3) 深度关联（类别只影响尺寸先验）
    depth_assoc_.configure(depth_params_, color, depth, base_from_color);
    std::vector<ObjectObservation> obs;
    obs.reserve(boxes.size());
    for (const auto& det_box : boxes) {
      const SemanticClass* cls = findClass(det_box.class_id);
      if (cls == nullptr) {
        continue;  // 配置里没声明这个类别
      }
      DetectionBox box = det_box;
      box.class_name = cls->name;
      ObjectObservation ob;
      ob.max_speed_mps = cls->max_speed_mps;
      if (!depth_fresh) {
        // 深度不新鲜: 用地面投影回退（全 0 深度图 -> depth_valid=false）
        DepthImage empty;
        if (!depth_assoc_.associate(box, empty, *cls, ob)) continue;
      } else if (!depth_assoc_.associate(box, depth_img_, *cls, ob)) {
        continue;
      }
      obs.push_back(ob);
    }

    // 4) 跟踪（world 系；无 odom 时退化为 base 系）
    RobotState robot;
    robot = robotState(last_color_stamp_);
    std::vector<ObjectTrack> tracks;
    tracker_.update(last_color_stamp_.toSec(), obs, robot, tracks);

    // 5) 发布
    publishObjects(tracks);
    if (publish_markers_) publishMarkers(tracks);
    if (publish_image_) publishAnnotated(tracks);
    if (publish_log_) publishLog(tracks, color.width, color.height);
    const double depth_valid_ratio = obs.empty() ? 0.0
        : static_cast<double>(std::count_if(obs.begin(), obs.end(),
                                            [](const ObjectObservation& o) { return o.depth_valid; })) /
              static_cast<double>(obs.size());
    publishStatus(static_cast<int>(tracks.size()),
                  (ros::WallTime::now() - t0).toSec() * 1e3, depth_valid_ratio,
                  detected ? "ok" : "no_detection");
  }

  bool lookupExtrinsics(const ros::Time& stamp, Extrinsics& out) {
    try {
      tf::StampedTransform tf;
      tf_listener_.lookupTransform(base_frame_, color_frame_, stamp, tf);
      const auto& q = tf.getRotation();
      const Eigen::Quaterniond eq(q.w(), q.x(), q.y(), q.z());
      out.R = eq.normalized().toRotationMatrix();
      out.t = Eigen::Vector3d(tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z());
      return true;
    } catch (const tf::TransformException&) {
      return false;
    }
  }

  RobotState robotState(const ros::Time& stamp) {
    RobotState rs;
    // base_link 在 odom 下的位姿（odom 即 world）
    try {
      tf::StampedTransform tf;
      tf_listener_.lookupTransform("odom", base_frame_, stamp, tf);
      const auto& q = tf.getRotation();
      const Eigen::Quaterniond eq(q.w(), q.x(), q.y(), q.z());
      rs.R_world_base = eq.normalized().toRotationMatrix();
      rs.position_world = Eigen::Vector3d(tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z());
      rs.valid = true;
    } catch (const tf::TransformException&) {
      rs.valid = false;
    }
    if (rs.valid && odom_valid_) {
      // odom twist 在 child_frame(base_link) 下 -> 转到 world
      const Eigen::Vector3d v_base(odom_.twist.twist.linear.x,
                                   odom_.twist.twist.linear.y,
                                   odom_.twist.twist.linear.z);
      rs.velocity_world = rs.R_world_base * v_base;
      // twist 可能是旧数据（静止时为零）: 超过 0.2 s 就只信位置差分（KF 自己会平滑）
      if (std::fabs((stamp - odom_.header.stamp).toSec()) > 0.2) {
        rs.velocity_world.setZero();
      }
    }
    return rs;
  }

  void fillObject(const ObjectTrack& tr, rosiwit_perception_ros1::SemanticObject& p,
                  const std_msgs::Header& header) const {
    p.header = header;
    p.id = tr.id;
    p.class_id = tr.class_id;
    p.class_name = tr.class_name;
    p.pose.pose.position.x = tr.position.x();
    p.pose.pose.position.y = tr.position.y();
    p.pose.pose.position.z = tr.z;
    p.pose.pose.orientation.w = 1.0;
    p.pose.covariance[0] = tr.cov_xy(0, 0);
    p.pose.covariance[7] = tr.cov_xy(1, 1);
    p.velocity_base.x = tr.velocity_base.x();
    p.velocity_base.y = tr.velocity_base.y();
    p.velocity_world.x = tr.velocity_world.x();
    p.velocity_world.y = tr.velocity_world.y();
    p.ground_position.x = tr.ground.x();
    p.ground_position.y = tr.ground.y();
    p.distance = static_cast<float>(tr.distance);
    p.confidence = static_cast<float>(tr.confidence);
    p.depth_valid = tr.depth_valid;
    p.velocity_valid = tr.velocity_valid;
    p.in_fov = tr.in_fov;
    p.confirmed = tr.confirmed;
    p.coasting = tr.coasting;
    p.age = static_cast<float>(tr.age);
  }

  void publishObjects(const std::vector<ObjectTrack>& tracks) {
    std_msgs::Header header;
    header.stamp = last_color_stamp_;
    header.frame_id = base_frame_;

    rosiwit_perception_ros1::SemanticObjectArray all;
    all.header = header;
    for (const auto& tr : tracks) {
      rosiwit_perception_ros1::SemanticObject p;
      fillObject(tr, p, header);
      all.objects.push_back(p);
    }
    objects_pub_.publish(all);

    // person 子集: 给导航/旧接口用，类别集合变多也不影响这些消费者
    if (publish_persons_alias_) {
      const int person_id = personClassId();
      if (person_id >= 0) {
        rosiwit_perception_ros1::SemanticObjectArray persons;
        persons.header = header;
        for (const auto& tr : tracks) {
          if (tr.class_id == person_id) {
            rosiwit_perception_ros1::SemanticObject p;
            fillObject(tr, p, header);
            persons.objects.push_back(p);
          }
        }
        persons_pub_.publish(persons);
      }
    }
  }

  void publishMarkers(const std::vector<ObjectTrack>& tracks) {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);
    int id = 0;
    for (const auto& tr : tracks) {
      const bool is_person = tr.class_id == personClassId();
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = base_frame_;
      sphere.header.stamp = last_color_stamp_;
      sphere.ns = "objects";
      sphere.id = id++;
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position.x = tr.position.x();
      sphere.pose.position.y = tr.position.y();
      sphere.pose.position.z = tr.z;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.35;
      sphere.color.r = is_person ? 1.0f : 0.1f;
      sphere.color.g = is_person ? 0.3f : 0.8f;
      sphere.color.b = is_person ? 0.1f : 1.0f;
      sphere.color.a = 0.8f;
      if (tr.coasting) {
        sphere.color.g = 0.9f;
      }
      arr.markers.push_back(sphere);

      visualization_msgs::Marker text;
      text.header = sphere.header;
      text.ns = "object_labels";
      text.id = id++;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose = sphere.pose;
      text.pose.position.z += 0.4;
      text.scale.z = 0.22;
      text.color.r = text.color.g = text.color.b = 1.0f;
      text.color.a = 1.0f;
      std::ostringstream os;
      os.setf(std::ios::fixed); os.precision(1);
      os << "ID " << tr.id << " " << tr.class_name << " | " << tr.distance << "m | "
         << tr.velocity_base.norm() << "m/s";
      text.text = os.str();
      arr.markers.push_back(text);

      if (tr.velocity_base.norm() > 0.05) {
        visualization_msgs::Marker arrow;
        arrow.header = sphere.header;
        arrow.ns = "object_velocity";
        arrow.id = id++;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.pose = sphere.pose;
        arrow.scale.x = 0.05;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.color.g = 1.0f;
        arrow.color.a = 0.9f;
        arrow.points.resize(2);
        arrow.points[0].x = tr.position.x();
        arrow.points[0].y = tr.position.y();
        arrow.points[0].z = 0.05;
        arrow.points[1].x = tr.position.x() + tr.velocity_base.x();
        arrow.points[1].y = tr.position.y() + tr.velocity_base.y();
        arrow.points[1].z = 0.05;
        arr.markers.push_back(arrow);
      }
    }
    markers_pub_.publish(arr);
  }

  void publishAnnotated(const std::vector<ObjectTrack>& tracks) {
    cv::Mat canvas = last_color_.clone();
    for (const auto& tr : tracks) {
      cv::Scalar color(0, 200, 0);
      if (tr.class_id != personClassId()) color = cv::Scalar(200, 120, 0);
      if (tr.velocity_base.norm() > 0.3) color = cv::Scalar(0, 0, 255);
      if (!tr.depth_valid) color = cv::Scalar(0, 165, 255);
      cv::rectangle(canvas,
                    cv::Point(static_cast<int>(tr.box.u0), static_cast<int>(tr.box.v0)),
                    cv::Point(static_cast<int>(tr.box.u1), static_cast<int>(tr.box.v1)), color, 2);
      char text[128];
      std::snprintf(text, sizeof(text), "ID %u %s %.1fm %.1fm/s%s", tr.id,
                    tr.class_name.c_str(), tr.distance, tr.velocity_base.norm(),
                    tr.velocity_valid ? "" : " (v?)");
      cv::putText(canvas, text, cv::Point(static_cast<int>(tr.box.u0),
                                          std::max(12, static_cast<int>(tr.box.v0) - 6)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }
    std_msgs::Header header;
    header.stamp = last_color_stamp_;
    header.frame_id = color_frame_;
    image_pub_.publish(cv_bridge::CvImage(header, "bgr8", canvas).toImageMsg());
  }

  void publishLog(const std::vector<ObjectTrack>& tracks, int img_w, int img_h) {
    std::ostringstream os;
    os.setf(std::ios::fixed); os.precision(3);
    for (const auto& tr : tracks) {
      os << "{\"stamp\":" << last_color_stamp_.toSec() << ",\"id\":" << tr.id
         << ",\"cls_id\":" << tr.class_id
         << ",\"cls\":\"" << tr.class_name << "\""
         << ",\"u\":" << tr.box.cx() << ",\"v\":" << tr.box.cy()
         << ",\"img_w\":" << img_w << ",\"img_h\":" << img_h
         << ",\"x\":" << tr.position.x() << ",\"y\":" << tr.position.y()
         << ",\"z\":" << tr.z << ",\"vx\":" << tr.velocity_base.x()
         << ",\"vy\":" << tr.velocity_base.y()
         << ",\"wvx\":" << tr.velocity_world.x() << ",\"wvy\":" << tr.velocity_world.y()
         << ",\"dist\":" << tr.distance
         << ",\"conf\":" << tr.confidence
         << ",\"depth_valid\":" << (tr.depth_valid ? 1 : 0)
         << ",\"vel_valid\":" << (tr.velocity_valid ? 1 : 0)
         << ",\"coasting\":" << (tr.coasting ? 1 : 0)
         << "}\n";
    }
    if (!os.str().empty()) {
      std_msgs::String msg;
      msg.data = os.str();
      log_pub_.publish(msg);
    }
  }

  void publishStatus(int tracks, double latency_ms, double depth_valid_ratio,
                     const std::string& code) {
    std::ostringstream os;
    os.setf(std::ios::fixed); os.precision(1);
    os << "{\"detector\":\"" << detector_->name() << "\",\"objects\":" << tracks
       << ",\"latency_ms\":" << latency_ms
       << ",\"depth_valid_ratio\":" << depth_valid_ratio * 100.0
       << ",\"code\":\"" << code << "\"}";
    std_msgs::String msg;
    msg.data = os.str();
    status_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf::TransformListener tf_listener_;

  std::string detector_type_ = "rknn";
  std::string base_frame_ = "base_link";
  std::string color_frame_;
  std::string depth_frame_;
  std::string color_topic_;
  std::string depth_topic_;
  std::string color_info_topic_;
  std::string depth_info_topic_;
  std::string odom_topic_;
  double depth_tolerance_ = 0.05;
  double max_frame_gap_ = 0.5;
  bool publish_image_ = true;
  bool publish_markers_ = true;
  bool publish_log_ = true;
  bool publish_persons_alias_ = true;

  rosiwit_perception::DepthParams depth_params_;
  rosiwit_perception::TrackerParams tracker_params_;
  std::vector<SemanticClass> classes_;
  DepthAssociator depth_assoc_;
  MultiObjectTracker tracker_;
  std::unique_ptr<DetectorBackend> detector_;

  sensor_msgs::CameraInfo color_info_, depth_info_;
  bool color_info_valid_ = false, depth_info_valid_ = false;
  cv::Mat last_color_;
  ros::Time last_color_stamp_, processed_stamp_, depth_stamp_;
  bool color_ready_ = false, depth_ready_ = false;
  DepthImage depth_img_;
  nav_msgs::Odometry odom_;
  bool odom_valid_ = false;

  ros::Publisher objects_pub_, persons_pub_, markers_pub_, status_pub_, log_pub_, image_pub_;
  ros::Subscriber color_sub_, depth_sub_, color_info_sub_, depth_info_sub_, odom_sub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_semantics");
  ObjectSemanticsNode node;
  if (!ros::ok()) return 1;
  node.spin();
  return 0;
}
