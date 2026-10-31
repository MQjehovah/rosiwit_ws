#ifndef FAST_LIVO2_UTILS_H
#define FAST_LIVO2_UTILS_H

#include <cstdint>
#include <vector>

/// int64 图层参数转 int，越界抛 std::out_of_range。
std::vector<int> convertToIntVectorSafe(const std::vector<int64_t>& int64_vector);

#endif  // FAST_LIVO2_UTILS_H
