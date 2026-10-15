// tools/traj_eval.cpp
// 轨迹评估: 读取两条 TUM 格式轨迹 (est 与 gt), 输出 ATE/RPE.
// TUM 行: timestamp tx ty tz qx qy qz qw
// 用法: traj_eval <est.txt> <gt.txt>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "slam_core/trajectory_metrics.h"

using namespace rosiwit_slam;

static bool loadTum(const std::string& path, std::vector<PoseStamped>& out) {
    std::ifstream in(path);
    if (!in.good()) return false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        double t, x, y, z, qx, qy, qz, qw;
        if (!(ss >> t >> x >> y >> z >> qx >> qy >> qz >> qw)) continue;
        PoseStamped p;
        p.time = t;
        p.trans = V3D(x, y, z);
        p.rot = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
        out.push_back(p);
    }
    return !out.empty();
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: traj_eval <est.txt> <gt.txt>   (TUM: t x y z qx qy qz qw)\n";
        return 1;
    }
    std::vector<PoseStamped> est, gt;
    if (!loadTum(argv[1], est)) { std::cerr << "[traj_eval] cannot read " << argv[1] << "\n"; return 1; }
    if (!loadTum(argv[2], gt)) { std::cerr << "[traj_eval] cannot read " << argv[2] << "\n"; return 1; }
    const TrajMetrics m = evaluateTrajectory(est, gt);
    std::printf("n=%d  ATE_rmse=%.4f m  ATE_mean=%.4f m  RPE_trans_rmse=%.4f m  RPE_rot_rmse=%.4f deg\n",
                m.n, m.ate_rmse, m.ate_mean, m.rpe_trans_rmse, m.rpe_rot_rmse_deg);
    return 0;
}
