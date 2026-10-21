// ============================================================================
// movebase_protocol.cpp — 见 movebase_protocol.h
// ============================================================================
#include "movebase_protocol.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>

namespace rosiwit_navigation
{
namespace movebase
{
namespace
{

constexpr double kEps = 1e-9;

void skipWs(const std::string & s, size_t & i)
{
  while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) { ++i; }
}

bool parseQuoted(const std::string & s, size_t & i, std::string & out)
{
  skipWs(s, i);
  if (i >= s.size() || s[i] != '"') { return false; }
  ++i;
  out.clear();
  while (i < s.size()) {
    const char c = s[i++];
    if (c == '"') { return true; }
    if (c == '\\' && i < s.size()) {
      out.push_back(s[i++]);
      continue;
    }
    out.push_back(c);
  }
  return false;
}

bool parseNumber(const std::string & s, size_t & i, double & out)
{
  skipWs(s, i);
  if (i >= s.size()) { return false; }
  const char * begin = s.c_str() + i;
  char * end = nullptr;
  const double value = std::strtod(begin, &end);
  if (end == begin) { return false; }
  i += static_cast<size_t>(end - begin);
  if (!std::isfinite(value)) { return false; }
  out = value;
  return true;
}

bool expectChar(const std::string & s, size_t & i, char c)
{
  skipWs(s, i);
  if (i >= s.size() || s[i] != c) { return false; }
  ++i;
  return true;
}

// 跳过一个任意 JSON 值 (字符串/数字/对象/数组/字面量). 用于忽略不关心的键.
bool skipValue(const std::string & s, size_t & i)
{
  skipWs(s, i);
  if (i >= s.size()) { return false; }
  const char c = s[i];
  if (c == '"') {
    std::string ignored;
    return parseQuoted(s, i, ignored);
  }
  if (c == '{' || c == '[') {
    const char open = c;
    const char close = (c == '{') ? '}' : ']';
    int depth = 0;
    while (i < s.size()) {
      const char cur = s[i];
      if (cur == '"') {
        std::string ignored;
        if (!parseQuoted(s, i, ignored)) { return false; }
        continue;
      }
      if (cur == open) { ++depth; }
      if (cur == close) {
        --depth;
        ++i;
        if (depth == 0) { return true; }
        continue;
      }
      ++i;
    }
    return false;
  }
  // 数字 / true / false / null
  const size_t start = i;
  const char * begin = s.c_str() + i;
  char * end = nullptr;
  (void)std::strtod(begin, &end);
  if (end != begin) {
    i += static_cast<size_t>(end - begin);
    return true;
  }
  while (i < s.size() && (std::isalpha(static_cast<unsigned char>(s[i])))) { ++i; }
  return i > start;
}

bool parsePointObject(const std::string & s, size_t & i, core::Pose2D & point)
{
  if (!expectChar(s, i, '{')) { return false; }
  bool has_x = false;
  bool has_y = false;
  while (true) {
    skipWs(s, i);
    if (i >= s.size()) { return false; }
    if (s[i] == '}') { ++i; break; }
    if (s[i] == ',') { ++i; continue; }
    std::string key;
    if (!parseQuoted(s, i, key)) { return false; }
    if (!expectChar(s, i, ':')) { return false; }
    if (key == "x" || key == "y") {
      double value = 0.0;
      if (!parseNumber(s, i, value)) { return false; }
      if (key == "x") { point.x = value; has_x = true; }
      else { point.y = value; has_y = true; }
    } else if (!skipValue(s, i)) {
      return false;
    }
  }
  return has_x && has_y;
}

bool parseSegment(const std::string & s, size_t & i, std::vector<core::Pose2D> & segment)
{
  if (!expectChar(s, i, '[')) { return false; }
  while (true) {
    skipWs(s, i);
    if (i >= s.size()) { return false; }
    if (s[i] == ']') { ++i; return true; }
    if (s[i] == ',') { ++i; continue; }
    core::Pose2D point;
    if (!parsePointObject(s, i, point)) { return false; }
    segment.push_back(point);
  }
}

bool parseFlatSegmentBody(const std::string & s, size_t & i,
                          std::vector<core::Pose2D> & segment)
{
  // 已经位于 '[' 之后; 支持 "path": [{"x":..,"y":..}, ...] 的扁平写法.
  while (true) {
    skipWs(s, i);
    if (i >= s.size()) { return false; }
    if (s[i] == ']') { ++i; return true; }
    if (s[i] == ',') { ++i; continue; }
    core::Pose2D point;
    if (!parsePointObject(s, i, point)) { return false; }
    segment.push_back(point);
  }
}

bool findKey(const std::string & s, size_t & i, const std::string & wanted)
{
  while (i < s.size()) {
    skipWs(s, i);
    if (i >= s.size()) { return false; }
    if (s[i] != '"') {
      ++i;
      continue;
    }
    std::string key;
    if (!parseQuoted(s, i, key)) { return false; }
    skipWs(s, i);
    if (i < s.size() && s[i] == ':') {
      if (key == wanted) {
        ++i;
        return true;
      }
      ++i;
      if (!skipValue(s, i)) { return false; }
      continue;
    }
  }
  return false;
}

void setError(std::string * error, const std::string & message)
{
  if (error != nullptr) { *error = message; }
}

}  // namespace

void fillPathYaw(std::vector<core::Pose2D> & path)
{
  for (size_t i = 0; i < path.size(); ++i) {
    const size_t from = (i + 1 < path.size()) ? i : (i > 0 ? i - 1 : i);
    const size_t to = (i + 1 < path.size()) ? i + 1 : i;
    const double dx = path[to].x - path[from].x;
    const double dy = path[to].y - path[from].y;
    if (std::fabs(dx) < kEps && std::fabs(dy) < kEps) { continue; }
    path[i].theta = std::atan2(dy, dx);
  }
}

std::vector<core::Pose2D> slicePath(const std::vector<core::Pose2D> & path, long start_index)
{
  if (path.empty()) { return {}; }
  long index = std::max<long>(0, start_index);
  if (index >= static_cast<long>(path.size())) {
    index = static_cast<long>(path.size()) - 1;
  }
  return std::vector<core::Pose2D>(
      path.begin() + static_cast<std::ptrdiff_t>(index), path.end());
}

double pathLength(const std::vector<core::Pose2D> & path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return length;
}

double clampPercent(double value)
{
  if (!std::isfinite(value)) { return 0.0; }
  return std::max(0.0, std::min(1.0, value));
}

bool parsePathJson(const std::string & text,
                   std::vector<std::vector<core::Pose2D>> & segments,
                   std::string * error)
{
  segments.clear();
  size_t i = 0;
  if (!findKey(text, i, "path")) {
    setError(error, "缺少 \"path\" 键");
    return false;
  }
  if (!expectChar(text, i, '[')) {
    setError(error, "\"path\" 不是数组");
    return false;
  }

  // 扁平写法: "path": [{x,y}, ...]
  {
    size_t probe = i;
    skipWs(text, probe);
    if (probe < text.size() && text[probe] == '{') {
      std::vector<core::Pose2D> flat;
      if (!parseFlatSegmentBody(text, i, flat)) {
        setError(error, "扁平 path 数组解析失败");
        return false;
      }
      if (!flat.empty()) {
        fillPathYaw(flat);
        segments.push_back(std::move(flat));
      }
      return !segments.empty();
    }
  }

  while (true) {
    skipWs(text, i);
    if (i >= text.size()) {
      setError(error, "path 数组未闭合");
      return false;
    }
    if (text[i] == ']') { ++i; break; }
    if (text[i] == ',') { ++i; continue; }
    if (text[i] != '[') {
      setError(error, "path 段不是数组");
      return false;
    }
    std::vector<core::Pose2D> segment;
    if (!parseSegment(text, i, segment)) {
      setError(error, "path 段解析失败");
      return false;
    }
    if (!segment.empty()) {
      fillPathYaw(segment);
      segments.push_back(std::move(segment));
    }
  }
  if (segments.empty()) {
    setError(error, "path 为空");
    return false;
  }
  return true;
}

bool loadPathFile(const std::string & file,
                  std::vector<std::vector<core::Pose2D>> & segments,
                  std::string * error)
{
  if (file.empty()) {
    setError(error, "文件名为空");
    return false;
  }
  std::ifstream input(file.c_str());
  if (!input.is_open()) {
    setError(error, "打开文件失败: " + file);
    return false;
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  if (buffer.str().empty()) {
    setError(error, "文件内容为空: " + file);
    return false;
  }
  if (!parsePathJson(buffer.str(), segments, error)) {
    if (error != nullptr && !error->empty()) { *error = file + ": " + *error; }
    return false;
  }
  return true;
}

Decoded decodeGoal(const Goal & goal)
{
  Decoded out;
  out.path_segment_index = std::max<long>(0, goal.path_segment_index);
  out.path_index = std::max<long>(0, goal.path_index);
  out.speed = goal.robot_vel;

  switch (goal.navi_cmd) {
    case 0:
      out.command = Command::CANCEL;
      out.valid = true;
      out.supported = true;
      return out;

    case 1: {
      out.command = Command::SINGLE_POINT;
      if (!goal.file_path.empty()) {
        // 由节点加载文件后取 [path_segment_index][path_index]
        out.use_file = true;
        out.valid = true;
        out.supported = true;
        return out;
      }
      if (goal.tracking_path.empty()) {
        out.reject_result = RESULT_POSE_ERROR;
        return out;
      }
      size_t index = static_cast<size_t>(out.path_index);
      if (index >= goal.tracking_path.size()) {
        index = goal.tracking_path.size() - 1;
      }
      out.target = goal.tracking_path[index];
      out.valid = true;
      out.supported = true;
      return out;
    }

    case 2: {
      out.command = Command::TWO_POINTS;
      if (goal.tracking_path.size() < 2) {
        out.reject_result = RESULT_POSE_ERROR;
        return out;
      }
      out.start = goal.tracking_path[0];
      out.target = goal.tracking_path[1];
      out.has_start = true;
      out.valid = true;
      out.supported = true;
      return out;
    }

    case 3: {
      out.command = Command::FOLLOW_PATH;
      if (!goal.file_path.empty()) {
        out.use_file = true;
        out.valid = true;
        out.supported = true;
        return out;
      }
      if (goal.tracking_path.size() < 2) {
        out.reject_result = RESULT_POSE_ERROR;
        return out;
      }
      out.path = slicePath(goal.tracking_path, out.path_index);
      out.valid = true;
      out.supported = true;
      return out;
    }

    case 6:
      out.command = Command::ROTATE_180;
      out.valid = true;
      out.supported = true;
      return out;

    case 20:
      out.command = Command::COVERAGE;
      // 只读 tracking_path 作为区域多边形; 其余字段忽略.
      if (goal.tracking_path.size() < 3) {
        out.reject_result = RESULT_POSE_ERROR;
        return out;
      }
      out.polygon = goal.tracking_path;
      out.speed = goal.robot_vel;
      out.valid = true;
      out.supported = true;
      return out;

    case 7:
      out.command = Command::BACK_OFF;
      // 旧实现: backing_dist <= 0.1 时按 0.3m 倒车.
      out.back_distance = goal.backing_dist > 0.1 ? goal.backing_dist : 0.3;
      out.valid = true;
      out.supported = true;
      return out;

    case 4:
    case 5:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
      // 电梯/沥水/延边/禁行区等命令需要旧框架的外部模块, 当前核心不具备.
      out.command = Command::UNKNOWN;
      out.reject_result = RESULT_PLAN_FAILED;
      return out;

    default:
      out.command = Command::UNKNOWN;
      out.reject_result = RESULT_PLAN_FAILED;
      return out;
  }
}

}  // namespace movebase
}  // namespace rosiwit_navigation
