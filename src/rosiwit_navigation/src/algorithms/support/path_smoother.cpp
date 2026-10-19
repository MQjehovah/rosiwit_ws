#include "path_smoother.hpp"
#include <algorithm>
#include <cmath>

namespace rosiwit_navigation {
namespace planners {

core::Path smoothPath(const core::Path & raw_path,
    const std::shared_ptr<core::CostmapGrid> costmap,
    const SmootherConfig & config)
{
    if (raw_path.points.size() < 3) return raw_path;
    auto path = raw_path;
    size_t N = path.points.size();
    struct P { double x, y; };
    std::vector<P> pts(N);
    for (size_t i = 0; i < N; ++i) {
        pts[i].x = path.points[i].pose.x;
        pts[i].y = path.points[i].pose.y;
    }
    auto cellCost = [&](double wx, double wy) -> int {
        if (!costmap) return 0;
        const double ox = costmap->info.origin.position.x;
        const double oy = costmap->info.origin.position.y;
        const double res = costmap->info.resolution;
        const int mx = static_cast<int>(std::floor((wx - ox) / res));
        const int my = static_cast<int>(std::floor((wy - oy) / res));
        if (mx < 0 || mx >= static_cast<int>(costmap->info.width) ||
            my < 0 || my >= static_cast<int>(costmap->info.height)) return 100;
        const size_t idx = static_cast<size_t>(my) * costmap->info.width +
                           static_cast<size_t>(mx);
        const int v = costmap->data[idx];
        return v < 0 ? 100 : v;
    };
    auto segmentMaxCost = [&](const P & a, const P & b) -> int {
        if (!costmap) return 0;
        const double res = costmap->info.resolution > 0.0 ?
                           costmap->info.resolution : 0.05;
        const double dist = std::hypot(b.x - a.x, b.y - a.y);
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / (res * 0.5))));
        int max_cost = 0;
        for (int s = 0; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            max_cost = std::max(max_cost,
                cellCost(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
        }
        return max_cost;
    };
    auto segmentClear = [&](const P & a, const P & b) -> bool {
        return segmentMaxCost(a, b) < 100;
    };
    for (int iter = 0; iter < config.max_iterations; ++iter) {
        std::vector<P> f(N, {0,0});
        for (size_t i = 1; i < N - 1; ++i) {
            f[i].x = 0.5 * (pts[i-1].x + pts[i+1].x) - pts[i].x;
            f[i].y = 0.5 * (pts[i-1].y + pts[i+1].y) - pts[i].y;
        }
        double a = config.alpha_start - (config.alpha_start - config.alpha_end) * iter / config.max_iterations;
        for (size_t i = 1; i < N - 1; ++i) {
            P n{pts[i].x + f[i].x * a, pts[i].y + f[i].y * a};
            if (!segmentClear(pts[i-1], n)) continue;
            if (!segmentClear(n, pts[i+1])) continue;
            const int old_cost = std::max(
                segmentMaxCost(pts[i-1], pts[i]),
                segmentMaxCost(pts[i], pts[i+1]));
            const int new_cost = std::max(
                segmentMaxCost(pts[i-1], n),
                segmentMaxCost(n, pts[i+1]));
            if (new_cost > old_cost + config.max_cost_increase) continue;
            pts[i] = n;
        }
    }
    for (size_t i = 0; i < N; ++i) {
        path.points[i].pose.x = pts[i].x;
        path.points[i].pose.y = pts[i].y;
        if (i < N - 1)
            path.points[i].pose.theta = std::atan2(pts[i+1].y - pts[i].y, pts[i+1].x - pts[i].x);
        else if (N > 1)
            path.points[i].pose.theta = path.points[i-1].pose.theta;
    }
    return path;
}

}}  // namespaces
