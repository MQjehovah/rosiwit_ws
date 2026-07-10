// ============================================================
// rosiwit_navigation - 路径平滑器
// 使用梯度下降法平滑 A* 规划的栅格路径
// ============================================================

#include "rosiwit_navigation/algorithms/path_smoother.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rosiwit_navigation
{
namespace planners
{

nav_msgs::msg::Path smoothPath(
    const nav_msgs::msg::Path & raw_path,
    const nav_msgs::msg::OccupancyGrid::SharedPtr costmap,
    const SmootherConfig & config)
{
    if (raw_path.poses.size() < 3) {
        return raw_path;
    }

    auto path = raw_path;
    size_t N = path.poses.size();

    // 复制为可迭代的点数组
    struct Point2D { double x, y; };
    std::vector<Point2D> pts(N);
    for (size_t i = 0; i < N; ++i) {
        pts[i].x = path.poses[i].pose.position.x;
        pts[i].y = path.poses[i].pose.position.y;
    }

    // 碰撞检测 lambda
    auto isInObstacle = [&](double wx, double wy) -> bool {
        if (!costmap) return false;
        double ox = costmap->info.origin.position.x;
        double oy = costmap->info.origin.position.y;
        double res = costmap->info.resolution;
        int mx = static_cast<int>((wx - ox) / res);
        int my = static_cast<int>((wy - oy) / res);
        if (mx < 0 || mx >= static_cast<int>(costmap->info.width) ||
            my < 0 || my >= static_cast<int>(costmap->info.height)) {
            return true;  // 超出地图边界视为障碍物
        }
        // 注意：inflateCostmap 已经把障碍物附近标为 254+，这里直接用
        return costmap->data[my * costmap->info.width + mx] >= 100;
    };

    // 梯度下降平滑迭代
    for (int iter = 0; iter < config.max_iterations; ++iter) {
        // 计算每个内点的"平滑力"
        std::vector<Point2D> forces(N, {0.0, 0.0});

        for (size_t i = 1; i < N - 1; ++i) {
            // 平滑力：拉向相邻点的中点
            forces[i].x = 0.5 * (pts[i - 1].x + pts[i + 1].x) - pts[i].x;
            forces[i].y = 0.5 * (pts[i - 1].y + pts[i + 1].y) - pts[i].y;
        }

        // 应用平滑力
        double alpha = config.alpha_start -
            (config.alpha_start - config.alpha_end) * iter / config.max_iterations;

        for (size_t i = 1; i < N - 1; ++i) {
            Point2D new_pt;
            new_pt.x = pts[i].x + forces[i].x * alpha;
            new_pt.y = pts[i].y + forces[i].y * alpha;

            // 碰撞检测：如果新位置进入障碍物，回退到原位置
            if (!isInObstacle(new_pt.x, new_pt.y)) {
                pts[i] = new_pt;
            }
        }
    }

    // 写回结果
    for (size_t i = 0; i < N; ++i) {
        path.poses[i].pose.position.x = pts[i].x;
        path.poses[i].pose.position.y = pts[i].y;

        // 根据相邻点计算朝向
        if (i < N - 1) {
            double yaw = std::atan2(pts[i + 1].y - pts[i].y,
                                    pts[i + 1].x - pts[i].x);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            path.poses[i].pose.orientation = tf2::toMsg(q);
        } else {
            path.poses[i].pose.orientation = path.poses[i - 1].pose.orientation;
        }
    }

    return path;
}

}  // namespace planners
}  // namespace rosiwit_navigation
