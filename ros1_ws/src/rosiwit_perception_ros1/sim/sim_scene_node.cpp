// 离线闭环仿真的场景节点（仅用于验证, 不参与产品链路）
//
// 用已知几何渲染:
//   /camera_01/color/image_raw      bgr8   640x480（目标剪影, 供标注图肉眼检查）
//   /camera_01/depth/image_raw      16UC1  320x200（地面 + 目标包围盒, 含噪声/空洞）
//   /camera_01/{color,depth}/camera_info
//   /perception/objects/detections_2d       完美检测框（模拟 YOLO, 带 class_id）
//   /sim/ground_truth                       真值（位置/速度/距离/是否可见）
//   TF: odom -> base_link -> camera_01_{color,depth}_optical_frame
//   /odom                                   机器人位姿 + twist
//
// 场景: 4 个人（1 个近处站立、1 个站立、1 个横穿、1 个走远）+ 1 辆停放车辆,
// 机器人先静止 1 s, 再以 0.2 m/s、yaw=0.12 前进。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "rosiwit_perception_ros1/ObjectDetection2DArray.h"
#include "rosiwit_perception_ros1/SimGroundTruth.h"

namespace {

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
};
Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/// 只用到绕 z 的旋转，写死成 3x3 避免为仿真引入 Eigen
struct Mat3 {
  double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

  Vec3 operator*(const Vec3& v) const {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
  }
  Mat3 operator*(const Mat3& o) const {
    Mat3 r;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
      }
    }
    return r;
  }
  Mat3 transposed() const {
    Mat3 r;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        r.m[i][j] = m[j][i];
      }
    }
    return r;
  }
};

Vec3 rotateXY(const Vec3& p, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * p.x - s * p.y, s * p.x + c * p.y, p.z};
}

Mat3 rotZ(double yaw) {
  Mat3 r;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  r.m[0][0] = c; r.m[0][1] = -s;
  r.m[1][0] = s; r.m[1][1] = c;
  return r;
}

/// base_link <- camera_optical
const Mat3 kBaseFromCam = [] {
  Mat3 r;
  r.m[0][0] = 0; r.m[0][1] = 0; r.m[0][2] = 1;
  r.m[1][0] = -1; r.m[1][1] = 0; r.m[1][2] = 0;
  r.m[2][0] = 0; r.m[2][1] = -1; r.m[2][2] = 0;
  return r;
}();
const Vec3 kBaseToCam{0.525, 0.0, 0.713};  // 方案 §3 实测安装

struct Intrinsics {
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  int width = 0, height = 0;
};

struct Target {
  int gt_id = 0;
  int class_id = 0;
  std::string name;
  double w = 0.5, d = 0.3, h = 1.7;  // 全尺寸（m）
  double x0 = 0.0, y0 = 0.0, vx = 0.0, vy = 0.0;
  bool cylinder = false;  // 人用竖直圆柱渲染, 车辆用有朝向的包围盒
  double yaw = 0.0;       // 车辆朝向（绕 z；0 = 车头朝 +x）

  Vec3 centerAt(double t) const { return {x0 + vx * t, y0 + vy * t, h * 0.5}; }
  Vec3 velocity() const { return {vx, vy, 0.0}; }
};

struct Box3 {
  double x0, x1, y0, y1, z0, z1;
};

/// 射线-AABB（slab 法）；t 为沿 dir 的参数（dir 的 z 分量已归一为 1，故 t 即相机 z 深度）
bool rayBox(const Vec3& o, const Vec3& d, const Box3& b, double& t) {
  double tmin = 1e-4;
  double tmax = 1e9;
  const double oa[3] = {o.x, o.y, o.z};
  const double da[3] = {d.x, d.y, d.z};
  const double lo[3] = {b.x0, b.y0, b.z0};
  const double hi[3] = {b.x1, b.y1, b.z1};
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(da[i]) < 1e-9) {
      if (oa[i] < lo[i] || oa[i] > hi[i]) {
        return false;
      }
      continue;
    }
    double t1 = (lo[i] - oa[i]) / da[i];
    double t2 = (hi[i] - oa[i]) / da[i];
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmin > tmax) {
      return false;
    }
  }
  t = tmin;
  return true;
}

/// 射线-竖直圆柱（半径 r, 高度 [0,h]）；人物用圆柱更接近真实躯干轮廓
bool rayCylinder(const Vec3& o, const Vec3& d, double cx, double cy, double r, double h,
                 double& t) {
  const double a = d.x * d.x + d.y * d.y;
  if (a < 1e-9) {
    return false;
  }
  const double ox = o.x - cx;
  const double oy = o.y - cy;
  const double b = 2.0 * (d.x * ox + d.y * oy);
  const double c = ox * ox + oy * oy - r * r;
  const double disc = b * b - 4.0 * a * c;
  if (disc < 0.0) {
    return false;
  }
  const double sq = std::sqrt(disc);
  const double roots[2] = {(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)};
  for (int i = 0; i < 2; ++i) {
    const double s = roots[i];
    if (s <= 1e-4) {
      continue;
    }
    const double z = o.z + s * d.z;
    if (z >= 0.0 && z <= h) {
      t = s;
      return true;
    }
  }
  return false;
}

struct RobotState {
  double x = 0.0, y = 0.0, yaw = 0.12, speed = 0.0;

  Vec3 position() const { return {x, y, 0.0}; }
  Vec3 velocityWorld() const { return {speed * std::cos(yaw), speed * std::sin(yaw), 0.0}; }
  Mat3 R_world_base() const { return rotZ(yaw); }
};

RobotState robotAt(double t) {
  RobotState r;
  r.yaw = 0.12;
  if (t <= 1.0) {
    return r;
  }
  r.speed = 0.20;
  const double dt = t - 1.0;
  r.x = r.speed * dt * std::cos(r.yaw);
  r.y = r.speed * dt * std::sin(r.yaw);
  return r;
}

struct CameraPose {
  Mat3 R_world_cam;
  Vec3 t_world_cam;
};

CameraPose cameraPose(const RobotState& robot) {
  CameraPose c;
  c.R_world_cam = robot.R_world_base() * kBaseFromCam;
  c.t_world_cam = robot.position() + robot.R_world_base() * kBaseToCam;
  return c;
}

bool project(const Vec3& p_world, const CameraPose& cam, const Intrinsics& intr, double& u,
             double& v) {
  const Vec3 p_cam = cam.R_world_cam.transposed() * (p_world - cam.t_world_cam);
  if (p_cam.z < 0.05) {
    return false;
  }
  u = intr.fx * p_cam.x / p_cam.z + intr.cx;
  v = intr.fy * p_cam.y / p_cam.z + intr.cy;
  return true;
}

/// 投影目标包围盒的 8 个角点，返回像素框；有任何角点在相机后方则视为不可见
bool projectBox(const Target& target, const Vec3& center, const CameraPose& cam,
                const Intrinsics& intr, double& u0, double& v0, double& u1, double& v1) {
  const double hx = target.w * 0.5;
  const double hy = target.d * 0.5;
  const double lx[2] = {-hx, hx};
  const double ly[2] = {-hy, hy};
  const double zs[2] = {0.0, target.h};
  double min_u = 1e9, max_u = -1e9, min_v = 1e9, max_v = -1e9;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      const Vec3 offset = rotateXY({lx[i], ly[j], 0.0}, target.yaw);
      for (int k = 0; k < 2; ++k) {
        double u = 0.0, v = 0.0;
        if (!project({center.x + offset.x, center.y + offset.y, zs[k]}, cam, intr, u, v)) {
          return false;
        }
        min_u = std::min(min_u, u);
        max_u = std::max(max_u, u);
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);
      }
    }
  }
  u0 = std::max(0.0, min_u);
  v0 = std::max(0.0, min_v);
  u1 = std::min(static_cast<double>(intr.width - 1), max_u);
  v1 = std::min(static_cast<double>(intr.height - 1), max_v);
  return u1 > u0 && v1 > v0;
}

sensor_msgs::CameraInfo makeCameraInfo(const Intrinsics& k, const std::string& frame,
                                       const ros::Time& stamp) {
  sensor_msgs::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = frame;
  info.width = static_cast<uint32_t>(k.width);
  info.height = static_cast<uint32_t>(k.height);
  info.distortion_model = "plumb_bob";
  info.D.assign(5, 0.0);
  info.K[0] = k.fx; info.K[1] = 0; info.K[2] = k.cx;
  info.K[3] = 0;    info.K[4] = k.fy; info.K[5] = k.cy;
  info.K[6] = 0;    info.K[7] = 0;    info.K[8] = 1;
  info.R[0] = 1; info.R[1] = 0; info.R[2] = 0;
  info.R[3] = 0; info.R[4] = 1; info.R[5] = 0;
  info.R[6] = 0; info.R[7] = 0; info.R[8] = 1;
  info.P[0] = k.fx; info.P[1] = 0; info.P[2] = k.cx; info.P[3] = 0;
  info.P[4] = 0;    info.P[5] = k.fy; info.P[6] = k.cy; info.P[7] = 0;
  info.P[8] = 0;    info.P[9] = 0;    info.P[10] = 1;  info.P[11] = 0;
  return info;
}

}  // namespace

class SimSceneNode {
 public:
  SimSceneNode() : nh_(), pnh_("~"), rng_(7) {
    pnh_.param<double>("rate", rate_, 10.0);
    int seed = 7;
    pnh_.param<int>("seed", seed, 7);
    rng_.seed(static_cast<uint32_t>(seed));
    pnh_.param<bool>("noise", noise_, true);
    pnh_.param<double>("max_range", max_range_, 8.0);
    pnh_.param<double>("depth_dropout", dropout_, 0.10);
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh_.param<std::string>("color_frame", color_frame_, "camera_01_color_optical_frame");
    pnh_.param<std::string>("depth_frame", depth_frame_, "camera_01_depth_optical_frame");

    color_ = {478.118, 477.981, 324.378, 242.398, 640, 480};
    depth_ = {153.504, 153.504, 159.589, 99.437, 320, 200};

    // 两个独立场景，避免把"多人同框遮挡"（MVP 不处理）混进验收：
    //   persons: 4 人分车道, 互不遮挡; 1 人站立/2 人沿视线走/1 人横向慢移
    //   car:     1 辆停放车辆, 车头与机器人 heading 对齐
    pnh_.param<std::string>("scenario", scenario_, "persons");
    if (scenario_ == "car") {
      targets_ = {
          {1, 2, "car", 4.00, 1.80, 1.55, 7.45, 0.90, 0.0, 0.0, false, 0.12},
      };
    } else {
      targets_ = {
          {1, 0, "person", 0.36, 0.36, 1.70, 4.0, 0.0, 0.0, 0.0, true, 0.0},     // 站立
          {2, 0, "person", 0.36, 0.36, 1.70, 2.6, 1.6, 0.6, 0.0, true, 0.0},     // 左车道走远
          {3, 0, "person", 0.36, 0.36, 1.70, 3.4, -1.3, 0.5, 0.0, true, 0.0},    // 右车道走远
          {4, 0, "person", 0.36, 0.36, 1.70, 6.0, 3.2, 0.0, -0.10, true, 0.0},   // 左车道横移
      };
    }

    color_pub_ = nh_.advertise<sensor_msgs::Image>("/camera_01/color/image_raw", 2);
    depth_pub_ = nh_.advertise<sensor_msgs::Image>("/camera_01/depth/image_raw", 2);
    color_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera_01/color/camera_info", 2);
    depth_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera_01/depth/camera_info", 2);
    det_pub_ = nh_.advertise<rosiwit_perception_ros1::ObjectDetection2DArray>(
        "/perception/objects/detections_2d", 2);
    gt_pub_ = nh_.advertise<rosiwit_perception_ros1::SimGroundTruth>("/sim/ground_truth", 2);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

  }

  void spin() {
    ros::Rate rate(rate_);
    ros::Time start = ros::Time::now();
    while (ros::ok()) {
      const ros::Time stamp = ros::Time::now();
      const double t = (stamp - start).toSec();
      const RobotState robot = robotAt(t);
      const CameraPose cam = cameraPose(robot);

      publishTf(robot, stamp);
      // 留几毫秒让 listener 收到 TF, 再发图像（避免首帧 extrapolation）
      ros::Duration(0.005).sleep();

      publishDepth(cam, stamp, t);
      publishColor(cam, stamp, t);
      color_info_pub_.publish(makeCameraInfo(color_, color_frame_, stamp));
      depth_info_pub_.publish(makeCameraInfo(depth_, depth_frame_, stamp));
      publishOdom(robot, stamp);
      publishDetectionsAndGt(cam, robot, stamp, t);

      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  tf::Transform cameraTf() const {
    tf::Transform tf_cam;
    tf_cam.setOrigin(tf::Vector3(kBaseToCam.x, kBaseToCam.y, kBaseToCam.z));
    tf::Matrix3x3 rot(kBaseFromCam.m[0][0], kBaseFromCam.m[0][1], kBaseFromCam.m[0][2],
                      kBaseFromCam.m[1][0], kBaseFromCam.m[1][1], kBaseFromCam.m[1][2],
                      kBaseFromCam.m[2][0], kBaseFromCam.m[2][1], kBaseFromCam.m[2][2]);
    tf::Quaternion q;
    rot.getRotation(q);
    tf_cam.setRotation(q);
    return tf_cam;
  }

  void publishTf(const RobotState& robot, const ros::Time& stamp) {
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(robot.x, robot.y, 0.0));
    tf.setRotation(tf::createQuaternionFromYaw(robot.yaw));
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(tf, stamp, odom_frame_, base_frame_));
    // 相机外参不变，但按同一条 TF 流每帧重发（仿真里不用 latched 静态发布）
    const tf::Transform cam = cameraTf();
    tf_broadcaster_.sendTransform(tf::StampedTransform(cam, stamp, base_frame_, color_frame_));
    tf_broadcaster_.sendTransform(tf::StampedTransform(cam, stamp, base_frame_, depth_frame_));
  }

  void publishOdom(const RobotState& robot, const ros::Time& stamp) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = robot.x;
    odom.pose.pose.position.y = robot.y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot.yaw);
    // twist 在 base_link 下（机器人沿自身 heading 前进）
    odom.twist.twist.linear.x = robot.speed;
    odom_pub_.publish(odom);
  }

  void publishDepth(const CameraPose& cam, const ros::Time& stamp, double t) {
    sensor_msgs::Image img;
    img.header.stamp = stamp;
    img.header.frame_id = depth_frame_;
    img.height = static_cast<uint32_t>(depth_.height);
    img.width = static_cast<uint32_t>(depth_.width);
    img.encoding = "16UC1";
    img.is_bigendian = 0;
    img.step = static_cast<uint32_t>(depth_.width * sizeof(uint16_t));
    img.data.resize(img.step * img.height);

    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::normal_distribution<double> normal(0.0, 1.0);
    auto* row = reinterpret_cast<uint16_t*>(img.data.data());

    for (int v = 0; v < depth_.height; ++v) {
      for (int u = 0; u < depth_.width; ++u) {
        const Vec3 ray_cam{(u - depth_.cx) / depth_.fx, (v - depth_.cy) / depth_.fy, 1.0};
        const Vec3 dir = cam.R_world_cam * ray_cam;
        double best = 1e9;

        // 地面 z=0
        if (std::fabs(dir.z) > 1e-9) {
          const double s = -cam.t_world_cam.z / dir.z;
          if (s > 0.1) {
            best = std::min(best, s);
          }
        }
        // 目标: 人 = 竖直圆柱, 车 = AABB
        for (const auto& target : targets_) {
          const Vec3 c = target.centerAt(t);
          double s = 0.0;
          bool hit = false;
          if (target.cylinder) {
            hit = rayCylinder(cam.t_world_cam, dir, c.x, c.y, target.w * 0.5, target.h, s);
          } else {
            // 有朝向的包围盒: 把射线转到盒体坐标系再做 AABB
            const Vec3 o_local = rotateXY({cam.t_world_cam.x - c.x, cam.t_world_cam.y - c.y,
                                           cam.t_world_cam.z},
                                          -target.yaw);
            const Vec3 d_local = rotateXY(dir, -target.yaw);
            const Box3 box{-target.w * 0.5, target.w * 0.5, -target.d * 0.5,
                           target.d * 0.5, 0.0, target.h};
            hit = rayBox(o_local, d_local, box, s);
          }
          if (hit && s > 0.1 && s < best) {
            best = s;
          }
        }

        uint16_t mm = 0;
        if (best < max_range_) {
          double d = best;
          if (noise_) {
            const double sigma = 0.0025 * d * d + 0.010;
            d += normal(rng_) * sigma;
          }
          if (uni(rng_) >= dropout_) {
            const long value = std::lround(d * 1000.0);
            mm = static_cast<uint16_t>(std::max<long>(100, std::min<long>(8000, value)));
          }
        }
        row[v * depth_.width + u] = mm;
      }
    }
    depth_pub_.publish(img);
  }

  void publishColor(const CameraPose& cam, const ros::Time& stamp, double t) {
    sensor_msgs::Image img;
    img.header.stamp = stamp;
    img.header.frame_id = color_frame_;
    img.height = static_cast<uint32_t>(color_.height);
    img.width = static_cast<uint32_t>(color_.width);
    img.encoding = "bgr8";
    img.is_bigendian = 0;
    img.step = static_cast<uint32_t>(color_.width * 3);
    img.data.assign(img.step * img.height, 24);

    for (const auto& target : targets_) {
      const Vec3 c = target.centerAt(t);
      double u0 = 0.0, v0 = 0.0, u1 = 0.0, v1 = 0.0;
      if (!projectBox(target, c, cam, color_, u0, v0, u1, v1)) {
        continue;
      }
      const bool car = target.class_id == 2;
      const uint8_t b = car ? 40 : 90;
      const uint8_t g = car ? 60 : 70;
      const uint8_t r = car ? 90 : 120;
      for (int v = static_cast<int>(v0); v <= static_cast<int>(v1); ++v) {
        uint8_t* row = img.data.data() + static_cast<size_t>(v) * img.step;
        for (int u = static_cast<int>(u0); u <= static_cast<int>(u1); ++u) {
          row[u * 3 + 0] = b;
          row[u * 3 + 1] = g;
          row[u * 3 + 2] = r;
        }
      }
    }
    color_pub_.publish(img);
  }

  void publishDetectionsAndGt(const CameraPose& cam, const RobotState& robot,
                              const ros::Time& stamp, double t) {
    rosiwit_perception_ros1::ObjectDetection2DArray dets;
    dets.header.stamp = stamp;
    dets.header.frame_id = color_frame_;

    rosiwit_perception_ros1::SimGroundTruth gt;
    gt.header.stamp = stamp;
    gt.header.frame_id = base_frame_;

    std::uniform_real_distribution<double> jitter(-1.5, 1.5);
    const Mat3 R_bw = robot.R_world_base().transposed();
    const Vec3 robot_v = robot.velocityWorld();

    struct Proj {
      bool box_ok = false;
      double u0 = 0.0, v0 = 0.0, u1 = 0.0, v1 = 0.0;
      Vec3 center, center_base;
      double range = 0.0;
      bool fully_inside = false;
    };
    std::vector<Proj> projs(targets_.size());
    for (size_t i = 0; i < targets_.size(); ++i) {
      Proj& p = projs[i];
      p.center = targets_[i].centerAt(t);
      p.center_base = R_bw * (p.center - robot.position());
      p.box_ok = projectBox(targets_[i], p.center, cam, color_, p.u0, p.v0, p.u1, p.v1);
      p.fully_inside = p.box_ok && p.u0 > 1.0 && p.v0 > 1.0 &&
                       p.u1 < color_.width - 2.0 && p.v1 < color_.height - 2.0;
      p.range = std::hypot(p.center_base.x, p.center_base.y);
    }

    for (size_t i = 0; i < targets_.size(); ++i) {
      const Target& target = targets_[i];
      const Proj& p = projs[i];
      // MVP 不做多人同框拆分: 更近目标的框遮住本目标 >20% 面积时不纳入验收，
      // 否则深度 ROI 会量到前面的人, 那是已知限制而不是精度问题。
      bool occluded = false;
      const double area_i = std::max(1.0, (p.u1 - p.u0) * (p.v1 - p.v0));
      for (size_t j = 0; j < targets_.size(); ++j) {
        if (j == i || !projs[j].box_ok) continue;
        if (projs[j].range >= p.range - 0.2) continue;
        const double iw = std::min(p.u1, projs[j].u1) - std::max(p.u0, projs[j].u0);
        const double ih = std::min(p.v1, projs[j].v1) - std::max(p.v0, projs[j].v0);
        if (iw > 0.0 && ih > 0.0 && iw * ih > 0.20 * area_i) {
          occluded = true;
          break;
        }
      }
      const bool visible = p.fully_inside && !occluded && p.range < max_range_ &&
                           (p.u1 - p.u0) >= 4.0 && (p.v1 - p.v0) >= 8.0;
      if (!logged_boxes_) {
        ROS_INFO("[sim_scene] gt %d %s box=(%.1f,%.1f)-(%.1f,%.1f) inside=%d occ=%d "
                 "range=%.2f vis=%d",
                 target.gt_id, target.name.c_str(), p.u0, p.v0, p.u1, p.v1,
                 p.fully_inside ? 1 : 0, occluded ? 1 : 0, p.range, visible ? 1 : 0);
      }

      rosiwit_perception_ros1::SimTarget gt_target;
      gt_target.gt_id = target.gt_id;
      gt_target.class_id = target.class_id;
      gt_target.class_name = target.name;
      gt_target.center_base.x = p.center_base.x;
      gt_target.center_base.y = p.center_base.y;
      gt_target.center_base.z = p.center_base.z;
      gt_target.velocity_base = [&] {
        geometry_msgs::Vector3 v;
        const Vec3 vb = R_bw * (target.velocity() - robot_v);
        v.x = vb.x; v.y = vb.y; v.z = 0.0;
        return v;
      }();
      gt_target.velocity_world.x = target.vx;
      gt_target.velocity_world.y = target.vy;
      gt_target.range = static_cast<float>(p.range);
      gt_target.visible = visible;
      gt_target.depth_valid = visible && p.range < max_range_;
      gt.targets.push_back(gt_target);

      if (!visible) {
        continue;
      }
      const double u0 = p.u0, v0 = p.v0, u1 = p.u1, v1 = p.v1;
      rosiwit_perception_ros1::ObjectDetection2D det;
      det.header = dets.header;
      det.class_id = target.class_id;
      det.class_name = target.name;
      det.score = 0.95f;
      det.x_min = static_cast<float>(std::max(0.0, u0 + jitter(rng_)));
      det.y_min = static_cast<float>(std::max(0.0, v0 + jitter(rng_)));
      det.x_max = static_cast<float>(std::min(static_cast<double>(color_.width - 1),
                                              u1 + jitter(rng_)));
      det.y_max = static_cast<float>(std::min(static_cast<double>(color_.height - 1),
                                              v1 + jitter(rng_)));
      dets.detections.push_back(det);
    }

    gt_pub_.publish(gt);
    det_pub_.publish(dets);
    logged_boxes_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher color_pub_, depth_pub_, color_info_pub_, depth_info_pub_, det_pub_, gt_pub_,
      odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  double rate_ = 10.0;
  bool noise_ = true;
  double max_range_ = 8.0;
  double dropout_ = 0.10;
  std::string base_frame_, odom_frame_, color_frame_, depth_frame_, scenario_;
  Intrinsics color_, depth_;
  std::vector<Target> targets_;
  std::mt19937 rng_;
  bool logged_boxes_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_scene");
  SimSceneNode node;
  node.spin();
  return 0;
}
