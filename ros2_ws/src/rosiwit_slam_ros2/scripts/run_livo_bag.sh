#!/usr/bin/env bash
# 播包 -> 标准节点 (SlamPipeline + fast_livo2_frontend) -> 存图
#
#   ./run_livo_bag.sh <bag_dir> [out_dir] [config]
#
# 产物: <out_dir>/lio_odom.csv  <out_dir>/node.log  <out_dir>/map.pcd  <out_dir>/map_rgb.pcd
#
# 说明: 节点收到 SIGINT 不会自动存图, 这里在播包结束后调用 save_map 服务,
#       彩色地图写到 <path 去掉 .pcd>_rgb.pcd.
set -uo pipefail

BAG=${1:?usage: run_livo_bag.sh <bag_dir> [out_dir] [config]}
OUT=${2:-$PWD/livo_run}
CFG=${3:-fast_livo2_ntu.yaml}

PKG_SHARE="$(ros2 pkg prefix rosiwit_slam_ros2)/share/rosiwit_slam_ros2"
CFG_PATH="$PKG_SHARE/config/$CFG"

LIDAR_TOPIC=$(python3 - "$CFG_PATH" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
print(cfg.get('global', {}).get('input', {}).get('lidar_topic', '/os1_cloud_node1/points'))
PY
)
IMU_TOPIC=$(python3 - "$CFG_PATH" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
print(cfg.get('global', {}).get('input', {}).get('imu_topic', '/imu/imu'))
PY
)
IMG_TOPIC=$(python3 - "$CFG_PATH" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
print(cfg.get('global', {}).get('input', {}).get('image_topic', '/left/image_raw'))
PY
)

mkdir -p "$OUT"
ros2 launch rosiwit_slam_ros2 fast_livo2.launch.py \
  config:="$CFG" use_rviz:=false use_sim_time:=true > "$OUT/node.log" 2>&1 &
LPID=$!

for i in $(seq 1 40); do
  a=$(ros2 topic info "$LIDAR_TOPIC" 2>/dev/null | grep -c 'Subscription count: 1' || true)
  b=$(ros2 topic info "$IMU_TOPIC" 2>/dev/null | grep -c 'Subscription count: 1' || true)
  c=1
  [ -n "$IMG_TOPIC" ] && c=$(ros2 topic info "$IMG_TOPIC" 2>/dev/null | grep -c 'Subscription count: 1' || true)
  [ "$a" -ge 1 ] && [ "$b" -ge 1 ] && [ "$c" -ge 1 ] && break
  sleep 1
done
sleep 2

stdbuf -oL ros2 topic echo /lio_odom --csv > "$OUT/lio_odom.csv" 2>"$OUT/echo.err" &
EPID=$!
sleep 3

timeout 3600 ros2 bag play "$BAG" --clock > "$OUT/play.log" 2>&1
echo "play done, saving map..."

ros2 service call /save_map rosiwit_slam_ros2/srv/SaveMap "{path: '$OUT/map.pcd'}" \
  > "$OUT/save_map.log" 2>&1
grep -a 'rgb map' "$OUT/node.log" | tail -2

sleep 2
kill -INT $EPID 2>/dev/null || true
sleep 2
kill -INT $LPID 2>/dev/null || true
sleep 5
kill -9 $LPID 2>/dev/null || true

echo "=== odom: $(wc -l < "$OUT/lio_odom.csv")"
ls -lh "$OUT"/map*.pcd 2>/dev/null || echo "no pcd"
