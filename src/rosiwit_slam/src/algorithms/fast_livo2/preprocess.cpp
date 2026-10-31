// src/algorithms/fast_livo2/preprocess.cpp
#include "algorithms/fast_livo2/preprocess.h"

#include <cmath>

namespace rosiwit_slam {

void FastLivo2Preprocess::configure(const FastLivo2Config& cfg) {
    stride_ = cfg.point_filter_num > 0 ? cfg.point_filter_num : 1;
    blind_sqr_ = cfg.blind > 0.0 ? cfg.blind * cfg.blind : 0.0;
}

bool FastLivo2Preprocess::process(const CloudType& in, CloudType& out) const {
    out.clear();
    if (in.empty()) return false;

    out.reserve(in.size() / static_cast<size_t>(stride_) + 1);
    for (size_t i = 0; i < in.size(); ++i) {
        if (stride_ > 1 && (i % static_cast<size_t>(stride_)) != 0) continue;

        const PointType& p = in[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

        const double range_sqr = static_cast<double>(p.x) * p.x +
                                 static_cast<double>(p.y) * p.y +
                                 static_cast<double>(p.z) * p.z;
        if (range_sqr <= blind_sqr_) continue;

        out.push_back(p);
    }
    return !out.empty();
}

}  // namespace rosiwit_slam
