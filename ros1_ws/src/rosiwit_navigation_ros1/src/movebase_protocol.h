// ============================================================================
// movebase_protocol.h
// robot_movebase 的 action 协议解析 (纯 C++, 不依赖 ROS)
//
// 该层只负责:
//   * 把 RobotNaviCmdAction 的 goal 字段解释成内部命令 (navi_cmd -> Command)
//   * 路径索引裁剪 / 路径文件解析 / 航向补齐 / 进度计算
// 真正的运动执行在节点 (rosiwit_navigation_ros1_node.cpp) 里完成。
//
// 单独拆出来是为了能用 gtest 直接验证协议语义, 不需要 ROS1 环境。
// 报文形状取自 robot_movebase:
//   action/NaviCmdAction.action
//   src/RobotMovebaseNode.cpp: RobotCmdDectorNode() (switch navi_cmd)
//   src/RobotMovebaseNode.cpp: RobotActionReadFileNode()
// ============================================================================
#ifndef ROSIWIT_NAVIGATION_ROS1__MOVEBASE_PROTOCOL_H_
#define ROSIWIT_NAVIGATION_ROS1__MOVEBASE_PROTOCOL_H_

#include <cstddef>
#include <string>
#include <vector>

#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace movebase
{

// navi_cmd 取值 (与 NaviCmdAction.action 的注释一致)
enum class Command : int
{
  CANCEL = 0,          // 取消导航
  SINGLE_POINT = 1,    // 单点导航
  TWO_POINTS = 2,      // 两点导航
  FOLLOW_PATH = 3,     // 稠密点/文件路径跟踪
  ENTER_ELEVATOR = 4,  // 入梯 (核心不支持)
  EXIT_ELEVATOR = 5,   // 出梯 (核心不支持)
  ROTATE_180 = 6,      // 原地旋转 180°
  BACK_OFF = 7,        // 倒车
  COVERAGE = 20,       // 动态覆盖清洁 (tracking_path=区域多边形)
  UNKNOWN = 100,
};

// tracking_path_result 取值 (照抄 NaviCmdAction.action 与节点里的实际用值)
enum ResultCode : int
{
  RESULT_SUCCESS = 1,
  RESULT_POSE_ERROR = -1,           // 获取坐标异常
  RESULT_GLOBAL_MAP_ERROR = -2,     // 获取全局地图异常
  RESULT_LOCAL_MAP_ERROR = -3,      // 获取局部地图异常
  RESULT_PLAN_FAILED = -4,          // 无法规划
  RESULT_FILE_ERROR = -5,           // 无法加载路径文件
  RESULT_PERCEPTION_TIMEOUT = -6,   // 感知异常超时
  RESULT_DOCKING_POSE_ERROR = -7,   // 充电桩过程中坐标异常
  RESULT_GATE_TIMEOUT = -8,         // 闸控超时
  RESULT_STUCK = -15,               // 长时间不动 / 禁行区 (旧实现的 -15)
  RESULT_NO_PROGRESS = -16,         // 长时间不动 (旧实现的 -16)
};

// action goal 的 ROS 无关镜像
struct Goal
{
  int navi_cmd = 0;
  double robot_vel = 0.0;
  long path_index = 0;
  long path_segment_index = 0;
  double backing_dist = 0.0;
  std::vector<core::Pose2D> tracking_path;
  std::string file_path;
  // cmd=1 且 tracking_path 有 >=2 点时, 用最后一点当目标 (见 decodeGoal 注释)
  bool single_point_use_last_pose = false;
};

struct Decoded
{
  Command command = Command::UNKNOWN;
  bool valid = false;       // 字段自洽 (路径索引、点数等)
  bool supported = false;   // 当前核心能否执行
  int reject_result = RESULT_PLAN_FAILED;  // !valid / !supported 时回的结果码

  double speed = 0.0;       // <=0 表示用节点默认速度
  long path_segment_index = 0;
  long path_index = 0;
  bool use_file = false;    // 需要节点解析 file_path

  std::vector<core::Pose2D> path;  // FOLLOW_PATH: 待跟踪路径 (已从 path_index 裁剪)
  std::vector<core::Pose2D> polygon;  // COVERAGE: 区域多边形
  core::Pose2D start;              // TWO_POINTS: 显式起点
  bool has_start = false;
  core::Pose2D target;             // 单点/两点目标
  double back_distance = 0.3;      // BACK_OFF 距离 (m)
};

// goal 字段 -> 内部命令。不读取文件 (use_file=true 时由节点调用 loadPathFile)。
Decoded decodeGoal(const Goal & goal);

// 解析 robot_movebase 的路径文件: {"path":[[{"x":..,"y":..}, ...], ...], ...}
// 只关心 "path" 键, 其余键 (map_area / path_percentage) 忽略。
bool parsePathJson(const std::string & text,
                   std::vector<std::vector<core::Pose2D>> & segments,
                   std::string * error = nullptr);

bool loadPathFile(const std::string & file,
                  std::vector<std::vector<core::Pose2D>> & segments,
                  std::string * error = nullptr);

// 路径点没有角度时按相邻点方向补航向 (旧实现 RobotPlanTranfrom 的行为).
void fillPathYaw(std::vector<core::Pose2D> & path);

// 与旧实现一致: path_index 越界时退到最后一个点; 负数当 0.
std::vector<core::Pose2D> slicePath(const std::vector<core::Pose2D> & path, long start_index);

double pathLength(const std::vector<core::Pose2D> & path);

double clampPercent(double value);

}  // namespace movebase
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION_ROS1__MOVEBASE_PROTOCOL_H_
