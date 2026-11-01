#!/usr/bin/env bash
# 播包 + 出图: 启动 livo_node, 等订阅建立, 播 bag, 结束后让节点排空队列并存 PCD.
#
#   ./run_livo_bag.sh <bag_dir> [out_dir] [config_path]
#
# 产物: <out_dir>/lio_odom.csv, <out_dir>/node.log, <out_dir>/node_map.pcd
set -uo pipefail

BAG=${1:?usage: run_livo_bag.sh <bag_dir> [out_dir] [config_path]}
OUT=${2:-$PWD/livo_run}
PKG_SHARE=$(ros2 pkg prefix rosiwit_slam_ros2)/share/rosiwit_slam_ros2
CFG=${3:-$PKG_SHARE/config/fast_livo2_ntu.yaml}

LIDAR_TOPIC=$(python3 - "$CFG" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
print(cfg.get('global', {}).get('input', {}).get('lidar_topic', '/os1_cloud_node1/points'))
PY
)
IMU_TOPIC=$(python3 - "$CFG" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
print(cfg.get('global', {}).get('input', {}).get('imu_topic', '/imu/imu'))
PY
)
IMG_TOPIC=$(python3 - "$CFG" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
print(cfg.get('global', {}).get('input', {}).get('image_topic', '/left/image_raw'))
PY
)

mkdir -p "$OUT"
ros2 launch rosiwit_slam_ros2 fast_livo2.launch.py \
  config_path:="$CFG" output_pcd:="$OUT/node_map.pcd" use_rviz:=false use_sim_time:=true \
  > "$OUT/node.log" 2>&1 &
LPID=$!

for i in $(seq 1 40); do
  a=$(ros2 topic info "$LIDAR_TOPIC" 2>/dev/null | grep -c 'Subscription count: 1' || true)
  b=$(ros2 topic info "$IMU_TOPIC" 2>/dev/null | grep -c 'Subscription count: 1' || true)
  c=$(ros2 topic info "$IMG_TOPIC" 2>/dev/null | grep -c 'Subscription count: 1' || true)
  [ "$a" -ge 1 ] && [ "$b" -ge 1 ] && [ "$c" -ge 1 ] && break
  sleep 1
done
sleep 2
# ros2 launch 不转发 SIGINT, 必须直接给节点进程发
NODE_PID=$(pgrep -f 'lib/rosiwit_slam_ros2/livo_node' | head -1)
echo "node pid=$NODE_PID"

stdbuf -oL ros2 topic echo /lio_odom --csv > "$OUT/lio_odom.csv" 2>"$OUT/echo.err" &
EPID=$!
sleep 3

timeout 3600 ros2 bag play "$BAG" --clock > "$OUT/play.log" 2>&1
echo "play done, draining..."

kill -INT "$NODE_PID" 2>/dev/null || true
for i in $(seq 1 360); do
  grep -aq 'map saved\|map save FAILED' "$OUT/node.log" && break
  sleep 5
done

sleep 2
kill -INT $EPID 2>/dev/null || true
sleep 3
kill -9 $LPID 2>/dev/null || true
kill -9 "$NODE_PID" 2>/dev/null || true

grep -a 'processed\|map accumulate\|voxel 0.1\|map saved' "$OUT/node.log" \
  | sed 's/\x1b\[[0-9;]*m//g' | tail -8
ls -lh "$OUT"/node_map.pcd 2>/dev/null || echo "no pcd"
