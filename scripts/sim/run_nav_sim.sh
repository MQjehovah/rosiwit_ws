#!/usr/bin/env bash
# 在 WSL/Gazebo 里跑一段导航验证 (A/B 或单次), 记录 /nav_diagnostics、/cmd_vel、代价地图频率。
#
#   ./scripts/sim/run_nav_sim.sh --driver goal --label new
#   ./scripts/sim/run_nav_sim.sh --driver stress --label new
#   ./scripts/sim/run_nav_sim.sh --driver goal --label old --binary ~/sim_verify/node_old
#
# 前置: ROS1 melodic + ~/rosiwit_ws/ros1_ws 已 catkin_make, 见 docs/dev/nav-ros1-thread-model.md
# 注意: 不要开 `set -u` —— sourcing /opt/ros/*/setup.bash 会引用未定义变量。
set -o pipefail

DRIVER=goal
LABEL=run
BINARY=""
WORLD=""
GUI=false
TOLERANCE=""
BAND=""
CONFIG=""
SPAWN=""
GOAL=""
MAPFILE=""

while [ $# -gt 0 ]; do
  case "$1" in
    --driver) DRIVER=$2; shift 2 ;;
    --label) LABEL=$2; shift 2 ;;
    --binary) BINARY=$2; shift 2 ;;
    --world) WORLD=$2; shift 2 ;;
    --tolerance) TOLERANCE=$2; shift 2 ;;
    --band) BAND=$2; shift 2 ;;
    --config) CONFIG=$2; shift 2 ;;
    --spawn) SPAWN=$2; shift 2 ;;
    --goal) GOAL=$2; shift 2 ;;
    --map) MAPFILE=$2; shift 2 ;;
    *) echo "unknown arg: $1"; exit 2 ;;
  esac
done

WS=${ROSIWIT_WS:-$HOME/rosiwit_ws}
source /opt/ros/melodic/setup.bash
source "$WS/ros1_ws/devel/setup.bash"
export OMP_NUM_THREADS=2
OUT=$HOME/sim_verify
mkdir -p "$OUT"

NODE=$WS/ros1_ws/devel/lib/rosiwit_navigation_ros1/rosiwit_navigation_ros1_node
if [ -n "$BINARY" ]; then
  cp "$BINARY" "$NODE"
fi
case "$DRIVER" in
  goal) SCRIPT=$(dirname "$0")/nav_sim_goal.py ;;
  stress) SCRIPT=$(dirname "$0")/nav_sim_stress.py ;;
  *) echo "unknown driver: $DRIVER"; exit 2 ;;
esac

pkill -f 'roslaunch rosiwit_starter_ros1' 2>/dev/null
pkill -f gzserver 2>/dev/null
pkill -f rosiwit_navigation_ros1_node 2>/dev/null
pkill -f rosiwit_slam_ros1_node 2>/dev/null
sleep 5

LOG=$OUT/sim_${LABEL}.log
EXTRA=""
if [ -n "$WORLD" ]; then EXTRA="world_name:=$WORLD"; fi
# 用独立配置覆盖 goal_tolerance (实车 0.10, 仿真默认 0.20) 或整份配置
CFG=$CONFIG
if [ -z "$CFG" ] && [ -n "$TOLERANCE" ]; then
  CFG=$OUT/sim_cfg_${LABEL}.yaml
  sed -e "s/^goal_tolerance_xy:.*/goal_tolerance_xy: $TOLERANCE/" \
      -e "s/^goal_tolerance_yaw:.*/goal_tolerance_yaw: $TOLERANCE/" \
      "$WS/ros1_ws/src/rosiwit_navigation_ros1/config/simulation.yaml" > "$CFG"
  echo "config override: $CFG (tolerance=$TOLERANCE)"
fi
if [ -n "$CFG" ]; then EXTRA="$EXTRA navigation_config:=$CFG"; fi
if [ -n "$MAPFILE" ]; then EXTRA="$EXTRA map:=$MAPFILE"; fi
if [ -n "$SPAWN" ]; then
  SX=$(echo "$SPAWN" | cut -d, -f1)
  SY=$(echo "$SPAWN" | cut -d, -f2)
  SYAW=$(echo "$SPAWN" | cut -d, -f3)
  EXTRA="$EXTRA spawn_x:=$SX spawn_y:=$SY spawn_yaw:=${SYAW:-0.0}"
fi
echo "=== launch ($LABEL) -> $LOG ==="
setsid nohup roslaunch rosiwit_starter_ros1 sim_slam_nav.launch \
  gui:=$GUI rviz:=false use_drive:=false $EXTRA > "$LOG" 2>&1 < /dev/null &

# gazebo 的 world 加载慢时, launch 里的 spawn_model 会超时退出 -> 这里补一次
for _ in $(seq 1 40); do
  timeout 2 rosservice list 2>/dev/null | grep -q '^/gazebo/spawn_urdf_model$' && break
  sleep 1
done
for attempt in 1 2 3 4 5 6; do
  timeout 6 rostopic echo -n1 /odom >/dev/null 2>&1 && break
  echo "odom not up, spawn attempt $attempt"
  timeout 40 rosrun gazebo_ros spawn_model -urdf -param robot_description \
    -model mrobot -x "${SX:-0.0}" -y "${SY:-0.0}" -z 0.1 -Y "${SYAW:-0.0}" -unpause \
    >/dev/null 2>&1
  sleep 5
done
for _ in $(seq 1 40); do
  timeout 3 rostopic echo -n1 /nav_diagnostics >/dev/null 2>&1 && break
  sleep 2
done
sleep 8

PID=$(pgrep -f 'devel/lib/rosiwit_navigation_ros1/rosiwit_navigation_ros1_node' | head -1)
if [ -z "$PID" ]; then
  echo "NAV NODE NOT RUNNING"
  tail -30 "$LOG"
  exit 1
fi
echo "nav node pid=$PID threads=$(ls /proc/$PID/task | wc -l)"

echo "=== nav test ($DRIVER) ==="
timeout 600 python2 "$SCRIPT" "$LABEL" 150 "$BAND" "$GOAL" 2>&1 | tee "$OUT/run_${LABEL}.log" \
  | grep -E '^\{' | tee -a "$OUT/results.jsonl"

echo "=== post-run health ==="
PID2=$(pgrep -f 'devel/lib/rosiwit_navigation_ros1/rosiwit_navigation_ros1_node' | head -1)
if [ -n "$PID2" ]; then
  echo "node alive: pid=$PID2 threads=$(ls /proc/$PID2/task | wc -l) cpu=$(ps -o pcpu= -p "$PID2")"
else
  echo "NODE DIED DURING RUN"
fi
echo "process has died: $(grep -a -c 'process has died' "$LOG")"

pkill -INT -f 'roslaunch rosiwit_starter_ros1' 2>/dev/null
sleep 4
pkill -f gzserver 2>/dev/null
pkill -f rosiwit_navigation_ros1_node 2>/dev/null
pkill -f rosiwit_slam_ros1_node 2>/dev/null
echo "=== done $LABEL ==="
