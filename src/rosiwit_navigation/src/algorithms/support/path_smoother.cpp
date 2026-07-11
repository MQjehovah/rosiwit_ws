#include "path_smoother.hpp"
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
    auto obs = [&](double wx, double wy) -> bool {
        if (!costmap) return false;
        double ox = costmap->info.origin.position.x;
        double oy = costmap->info.origin.position.y;
        double res = costmap->info.resolution;
        int mx = static_cast<int>((wx - ox) / res);
        int my = static_cast<int>((wy - oy) / res);
        if (mx < 0 || mx >= static_cast<int>(costmap->info.width) ||
            my < 0 || my >= static_cast<int>(costmap->info.height)) return true;
        size_t idx = static_cast<size_t>(my) * costmap->info.width + static_cast<size_t>(mx);
        return costmap->data[idx] >= 100;
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
            if (!obs(n.x, n.y)) pts[i] = n;
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
