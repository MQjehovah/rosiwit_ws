// src/algorithms/fast_livo2/parse_config.h
#pragma once

#include <string>

#include "algorithms/fast_livo2/fast_livo2_config.h"

namespace rosiwit_slam {

/// 解析 FAST-LIVO2 前端配置。失败时返回 false 并写入 err。
bool parseFastLivo2Config(const std::string& yaml_path, FastLivo2Config& out, std::string& err);

}  // namespace rosiwit_slam
