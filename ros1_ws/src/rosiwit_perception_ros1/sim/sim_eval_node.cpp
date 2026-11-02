// 仿真评测节点（仅用于验证, 不参与产品链路）
//
// 订阅 /perception/objects 与 /sim/ground_truth, 逐帧按类别做最近邻匹配，统计:
//   - 位置 RMSE（分距离段）、速度 RMSE（世界系）、ID 切换、连续率、虚假 confirmed track
// 达到 frames 后打印报告并退出；退出码 0=PASS, 1=FAIL。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "rosiwit_perception_ros1/SemanticObjectArray.h"
#include "rosiwit_perception_ros1/SimGroundTruth.h"

namespace {

constexpr int kBinCount = 4;
const char* kBinNames[kBinCount] = {"<1m", "1-3m", "3-5m", "5-8m"};

int binOf(double range) {
  if (range < 1.0) return 0;
  if (range < 3.0) return 1;
  if (range < 5.0) return 2;
  return 3;
}

struct Stats {
  long visible = 0;
  long matched = 0;
  long switches = 0;
  double pos_sse = 0.0;
  long pos_n = 0;
  double dx_sum = 0.0;
  double dy_sum = 0.0;
  double vel_sse = 0.0;
  long vel_n = 0;
  double dvx_sum = 0.0;
  double dvy_sum = 0.0;
  double bin_sse[kBinCount] = {0.0, 0.0, 0.0, 0.0};
  long bin_n[kBinCount] = {0, 0, 0, 0};

  double posRmse() const { return pos_n > 0 ? std::sqrt(pos_sse / pos_n) : 0.0; }
  double velRmse() const { return vel_n > 0 ? std::sqrt(vel_sse / vel_n) : 0.0; }
  double binRmse(int i) const { return bin_n[i] > 0 ? std::sqrt(bin_sse[i] / bin_n[i]) : 0.0; }
};

struct GtFrame {
  ros::Time stamp;
  std::vector<rosiwit_perception_ros1::SimTarget> targets;
};

}  // namespace

class SimEvalNode {
 public:
  SimEvalNode() : nh_(), pnh_("~") {
    pnh_.param<int>("frames", frames_to_eval_, 110);
    pnh_.param<int>("warmup_frames", warmup_frames_, 15);
    pnh_.param<int>("acquisition_grace_frames", acquisition_grace_frames_, 5);
    pnh_.param<double>("match_radius", match_radius_, 1.0);
    pnh_.param<double>("min_match_ratio", min_match_ratio_, 0.95);
    pnh_.param<double>("max_pos_rmse", max_pos_rmse_, 0.15);
    pnh_.param<double>("max_vel_rmse", max_vel_rmse_, 0.15);
    pnh_.param<int>("max_id_switches", max_id_switches_, 0);
    pnh_.param<double>("max_false_rate", max_false_rate_, 0.05);
    pnh_.param<std::string>("report_path", report_path_,
                            "/tmp/object_semantics_sim_report.txt");
    pnh_.param<std::string>("csv_path", csv_path_, "");
    if (!csv_path_.empty()) {
      csv_.open(csv_path_.c_str());
      csv_ << "stamp,gt_id,visible,matched,dx,dy,range,track_id,nearest_dist\n";
    }

    const double bin_limits[kBinCount] = {0.20, 0.15, 0.30, 0.45};
    for (int i = 0; i < kBinCount; ++i) {
      std::ostringstream key;
      key << "max_pos_rmse_bin" << i;
      pnh_.param<double>(key.str(), bin_limit_[i], bin_limits[i]);
    }

    gt_sub_ = nh_.subscribe("/sim/ground_truth", 20, &SimEvalNode::gtCB, this);
    obj_sub_ = nh_.subscribe("/perception/objects", 20, &SimEvalNode::objectsCB, this);
    status_sub_ = nh_.subscribe("/perception/objects/status", 20, &SimEvalNode::statusCB, this);
    ROS_INFO("[sim_eval] waiting for /perception/objects (%d frames after %d warmup)",
             frames_to_eval_, warmup_frames_);
  }

  int exitCode() const { return pass_ ? 0 : 1; }

 private:
  void gtCB(const rosiwit_perception_ros1::SimGroundTruth::ConstPtr& msg) {
    GtFrame frame;
    frame.stamp = msg->header.stamp;
    frame.targets = msg->targets;
    gt_buffer_.push_back(frame);
    while (gt_buffer_.size() > 300) {
      gt_buffer_.pop_front();
    }
  }

  void statusCB(const std_msgs::String::ConstPtr& msg) {
    const std::string& s = msg->data;
    const std::string key = "\"latency_ms\":";
    const size_t pos = s.find(key);
    if (pos != std::string::npos) {
      latency_sum_ += std::atof(s.c_str() + pos + key.size());
      latency_n_ += 1;
    }
  }

  const GtFrame* findGt(const ros::Time& stamp) const {
    const GtFrame* best = nullptr;
    double best_dt = 0.06;
    for (auto it = gt_buffer_.rbegin(); it != gt_buffer_.rend(); ++it) {
      const double dt = std::fabs((it->stamp - stamp).toSec());
      if (dt < best_dt) {
        best_dt = dt;
        best = &(*it);
      }
    }
    return best;
  }

  void objectsCB(const rosiwit_perception_ros1::SemanticObjectArray::ConstPtr& msg) {
    if (done_) {
      return;
    }
    const GtFrame* gt = findGt(msg->header.stamp);
    if (gt == nullptr) {
      return;
    }
    if (frame_index_++ < warmup_frames_) {
      return;
    }

    const auto& tracks = msg->objects;

    // 同类别、1 m 内做全局最近邻贪心匹配（场景里目标彼此分得开）
    struct Pair {
      double d;
      int gt;
      int tr;
    };
    std::vector<Pair> pairs;
    for (size_t i = 0; i < gt->targets.size(); ++i) {
      const auto& g = gt->targets[i];
      if (!g.visible) continue;
      for (size_t j = 0; j < tracks.size(); ++j) {
        if (tracks[j].class_id != g.class_id) continue;
        const double dx = tracks[j].pose.pose.position.x - g.center_base.x;
        const double dy = tracks[j].pose.pose.position.y - g.center_base.y;
        const double d = std::hypot(dx, dy);
        if (d <= match_radius_) {
          pairs.push_back({d, static_cast<int>(i), static_cast<int>(j)});
        }
      }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d < b.d; });

    std::vector<int> gt_assign(gt->targets.size(), -1);
    std::vector<int> tr_assign(tracks.size(), -1);
    for (const auto& p : pairs) {
      if (gt_assign[p.gt] >= 0 || tr_assign[p.tr] >= 0) continue;
      gt_assign[p.gt] = p.tr;
      tr_assign[p.tr] = p.gt;
    }

    for (size_t i = 0; i < gt->targets.size(); ++i) {
      const auto& g = gt->targets[i];
      const bool matched = gt_assign[i] >= 0;
      if (csv_) {
        int nearest_id = -1;
        double nearest_d = 1e9;
        double nearest_dx = 0.0;
        double nearest_dy = 0.0;
        for (const auto& t : tracks) {
          if (t.class_id != g.class_id) continue;
          const double dx = t.pose.pose.position.x - g.center_base.x;
          const double dy = t.pose.pose.position.y - g.center_base.y;
          const double d = std::hypot(dx, dy);
          if (d < nearest_d) {
            nearest_d = d;
            nearest_id = static_cast<int>(t.id);
            nearest_dx = dx;
            nearest_dy = dy;
          }
        }
        if (matched) {
          nearest_id = static_cast<int>(tracks[gt_assign[i]].id);
          nearest_d = 0.0;
          nearest_dx = tracks[gt_assign[i]].pose.pose.position.x - g.center_base.x;
          nearest_dy = tracks[gt_assign[i]].pose.pose.position.y - g.center_base.y;
        }
        csv_ << msg->header.stamp.toSec() << "," << g.gt_id << "," << (g.visible ? 1 : 0) << ","
             << (matched ? 1 : 0) << "," << nearest_dx << "," << nearest_dy << "," << g.range
             << "," << nearest_id << "," << nearest_d << "\n";
      }
      if (!g.visible) {
        visible_streak_[g.gt_id] = 0;
        continue;
      }
      // 目标刚进入视野时 tracker 需要 confirm_hits 帧确认, 这段不算漏检
      int& streak = visible_streak_[g.gt_id];
      ++streak;
      if (streak <= acquisition_grace_frames_) {
        continue;
      }
      Stats& st = by_class_[g.class_id];
      st.visible += 1;
      Stats& gst = by_gt_[g.gt_id];
      gst.visible += 1;
      if (gt_assign[i] >= 0) {
        const auto& tr = tracks[gt_assign[i]];
        st.matched += 1;
        const double dx = tr.pose.pose.position.x - g.center_base.x;
        const double dy = tr.pose.pose.position.y - g.center_base.y;
        const double e2 = dx * dx + dy * dy;
        st.pos_sse += e2;
        st.pos_n += 1;
        st.dx_sum += dx;
        st.dy_sum += dy;
        gst.matched += 1;
        gst.pos_sse += e2;
        gst.pos_n += 1;
        gst.dx_sum += dx;
        gst.dy_sum += dy;
        const int b = binOf(g.range);
        st.bin_sse[b] += e2;
        st.bin_n[b] += 1;

        if (tr.velocity_valid) {
          const double dvx = tr.velocity_world.x - g.velocity_world.x;
          const double dvy = tr.velocity_world.y - g.velocity_world.y;
          st.vel_sse += dvx * dvx + dvy * dvy;
          st.dvx_sum += dvx;
          st.dvy_sum += dvy;
          st.vel_n += 1;
        }

        auto it = last_track_id_.find(g.gt_id);
        if (it != last_track_id_.end() && it->second != static_cast<int>(tr.id)) {
          st.switches += 1;
        }
        last_track_id_[g.gt_id] = static_cast<int>(tr.id);
      }
    }

    // 虚假 confirmed track: 附近没有任何同类真值（包含视野外目标，避免把 coasting 算错）
    for (size_t j = 0; j < tracks.size(); ++j) {
      if (!tracks[j].confirmed) continue;
      confirmed_track_frames_ += 1;
      bool near_gt = false;
      for (const auto& g : gt->targets) {
        if (g.class_id != tracks[j].class_id) continue;
        const double d = std::hypot(tracks[j].pose.pose.position.x - g.center_base.x,
                                    tracks[j].pose.pose.position.y - g.center_base.y);
        if (d <= match_radius_) {
          near_gt = true;
          break;
        }
      }
      if (!near_gt) {
        false_track_frames_ += 1;
      }
    }

    if (++eval_frames_ >= frames_to_eval_) {
      done_ = true;
      report();
      ros::shutdown();
    }
  }

  void report() {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(3);
    os << "=== object_semantics simulated acceptance ===\n";
    os << "evaluated frames: " << eval_frames_ << " (warmup " << warmup_frames_ << ")\n";

    bool pass = true;
    for (const auto& kv : by_class_) {
      const Stats& st = kv.second;
      const double match_ratio = st.visible > 0 ? static_cast<double>(st.matched) / st.visible : 0.0;
      os << "class " << kv.first << ": visible=" << st.visible << " matched=" << st.matched
         << " (" << match_ratio * 100.0 << "%)"
         << " id_switches=" << st.switches
         << " pos_rmse=" << st.posRmse() << " m"
         << " vel_rmse=" << st.velRmse() << " m/s (n=" << st.vel_n << ")\n";
      os << "  pos_rmse by range:";
      for (int b = 0; b < kBinCount; ++b) {
        os << " " << kBinNames[b] << "=" << st.binRmse(b) << " (n=" << st.bin_n[b] << ")";
      }
      os << "\n";
      if (st.pos_n > 0) {
        os << "  mean signed error: dx=" << (st.dx_sum / st.pos_n)
           << " dy=" << (st.dy_sum / st.pos_n) << " m";
        if (st.vel_n > 0) {
          os << ", dvx=" << (st.dvx_sum / st.vel_n) << " dvy=" << (st.dvy_sum / st.vel_n)
             << " m/s";
        }
        os << "\n";
      }

      if (match_ratio < min_match_ratio_) {
        os << "  FAIL: match ratio " << match_ratio << " < " << min_match_ratio_ << "\n";
        pass = false;
      }
      if (st.posRmse() > max_pos_rmse_) {
        os << "  FAIL: pos rmse " << st.posRmse() << " > " << max_pos_rmse_ << "\n";
        pass = false;
      }
      if (st.velRmse() > max_vel_rmse_) {
        os << "  FAIL: vel rmse " << st.velRmse() << " > " << max_vel_rmse_ << "\n";
        pass = false;
      }
      if (st.switches > max_id_switches_) {
        os << "  FAIL: id switches " << st.switches << " > " << max_id_switches_ << "\n";
        pass = false;
      }
      for (int b = 0; b < kBinCount; ++b) {
        if (st.bin_n[b] >= 20 && st.binRmse(b) > bin_limit_[b]) {
          os << "  FAIL: pos rmse " << kBinNames[b] << " " << st.binRmse(b) << " > "
             << bin_limit_[b] << "\n";
          pass = false;
        }
      }
    }

    os << "per-target:\n";
    for (const auto& kv : by_gt_) {
      const Stats& st = kv.second;
      const double ratio = st.visible > 0 ? static_cast<double>(st.matched) / st.visible : 0.0;
      os << "  gt " << kv.first << ": visible=" << st.visible << " matched=" << st.matched
         << " (" << ratio * 100.0 << "%) pos_rmse=" << st.posRmse();
      if (st.pos_n > 0) {
        os << " dx=" << (st.dx_sum / st.pos_n) << " dy=" << (st.dy_sum / st.pos_n);
      }
      os << "\n";
    }

    const double false_rate = confirmed_track_frames_ > 0
                                  ? static_cast<double>(false_track_frames_) /
                                        confirmed_track_frames_
                                  : 0.0;
    os << "false confirmed track frames: " << false_track_frames_ << "/"
       << confirmed_track_frames_ << " (" << false_rate * 100.0 << "%)\n";
    if (false_rate > max_false_rate_) {
      os << "  FAIL: false track rate " << false_rate << " > " << max_false_rate_ << "\n";
      pass = false;
    }
    if (latency_n_ > 0) {
      os << "node latency: mean " << (latency_sum_ / latency_n_) << " ms (n=" << latency_n_
         << ")\n";
    }
    os << "RESULT: " << (pass ? "PASS" : "FAIL") << "\n";

    std::printf("%s", os.str().c_str());
    std::fflush(stdout);
    std::ofstream out(report_path_.c_str());
    if (out) {
      out << os.str();
    }
    pass_ = pass;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber gt_sub_, obj_sub_, status_sub_;

  int frames_to_eval_ = 110;
  int warmup_frames_ = 15;
  int acquisition_grace_frames_ = 5;
  double match_radius_ = 1.0;
  double min_match_ratio_ = 0.95;
  double max_pos_rmse_ = 0.15;
  double max_vel_rmse_ = 0.15;
  int max_id_switches_ = 0;
  double max_false_rate_ = 0.05;
  double bin_limit_[kBinCount] = {0.20, 0.15, 0.30, 0.45};
  std::string report_path_;
  std::string csv_path_;
  std::ofstream csv_;

  std::deque<GtFrame> gt_buffer_;
  std::map<int, Stats> by_class_;
  std::map<int, Stats> by_gt_;
  std::map<int, int> last_track_id_;
  std::map<int, int> visible_streak_;
  int frame_index_ = 0;
  int eval_frames_ = 0;
  long confirmed_track_frames_ = 0;
  long false_track_frames_ = 0;
  double latency_sum_ = 0.0;
  long latency_n_ = 0;
  bool done_ = false;
  bool pass_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_eval");
  SimEvalNode node;
  ros::spin();
  return node.exitCode();
}
