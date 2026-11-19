#!/usr/bin/env bash
# 离线导出: 建图只跑一次, 之后用本脚本反复导出 xzmap/pgm (不重跑 SLAM)
# 用法: export_map_offline.sh <run_dir> [resolution=0.05] [kf_step=1.0]
# 参数: PGM_CLOSE PGM_DILATE PGM_FREE_R PGM_MIN_H PGM_MAX_H PGM_BLOCK
set -e
RUN=${1:?run_dir}
RES=${2:-0.05}
KF=${3:-1.0}
DIR=$(cd "$(dirname "$0")" && pwd)
echo "[export] run=$RUN res=$RES kf=$KF"
python3 "$DIR/export_robot_xzmap.py" "$RUN" "$RUN/map.xzmap" --kf-step "$KF"
python3 "$DIR/gen_pgm_local.py" "$RUN" "$RUN/map.pgm" "$RES"
cp -f "$RUN/odom_q.csv" "$RUN/traj.csv"
echo "[export] done: $RUN/map.xzmap  $RUN/map.pgm  $RUN/map.yaml"
