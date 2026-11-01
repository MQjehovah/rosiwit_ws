#!/usr/bin/env bash
# 四控制器对比: 固定起点/终点/世界, 每个 (控制器 x 场景) 跑 N 次, 输出 CSV + markdown 表
#
#   ./scripts/sim/compare_controllers.sh                     # 默认 4 控制器 x house_short x 1 次
#   CONTROLLERS="rpp teb" SCENES="house_short house_long" RUNS=3 ./scripts/sim/compare_controllers.sh
#
# 依赖 run_nav_sim.sh 的 --goal / --config / --band; 结果写到 $OUT/results_compare.csv
set -u
source /opt/ros/melodic/setup.bash
WS=${ROSIWIT_WS:-$HOME/rosiwit_ws}
source "$WS/ros1_ws/devel/setup.bash"
export OMP_NUM_THREADS=2
OUT=$HOME/sim_verify
mkdir -p "$OUT"
HERE=$(cd "$(dirname "$0")" && pwd)

CONTROLLERS=${CONTROLLERS:-"rpp mppi dwa teb"}
SCENES=${SCENES:-"house_short"}
RUNS=${RUNS:-1}
PROFILE=${PROFILE:-clean}
WORLD_DIR=$WS/ros1_ws/src/rosiwit_simulation_ros1/world
CFG_SRC=$WS/ros1_ws/src/rosiwit_navigation_ros1/config/simulation.yaml

CSV=$OUT/results_compare.csv
[ -f "$CSV" ] || echo "scene,profile,controller,run,status,xy_err_m,yaw_err_rad,wall_s,cmd_mean_v,cmd_max_v,cmd_max_dv,cmd_max_dw,xtrack_rms_m,min_clearance_m,safety_scale_min,replan_count,replan_fail" > "$CSV"

scene_args() {
  case "$1" in
    house_short) echo "--goal 1.0,2.0" ;;
    house_long)  echo "--band 5,7" ;;
    corridor)    echo "--world $WORLD_DIR/large.world --spawn -18,-18,1.57 --band 8,12" ;;
    doorway)     echo "--world $WORLD_DIR/doorway.world --goal 0.0,6.0" ;;
    blocked)     echo "--goal 1.0,2.0" ;;
    *) echo "" ;;
  esac
}

for scene in $SCENES; do
  extra=$(scene_args "$scene")
  for ctrl in $CONTROLLERS; do
    cfg=$OUT/cfg_${scene}_${ctrl}.yaml
    sed -e "s/^controller_name:.*/controller_name: \"$ctrl\"/" \
        -e "s/^motion_profile:.*/motion_profile: \"$PROFILE\"/" \
        "$CFG_SRC" > "$cfg"
    for run in $(seq 1 "$RUNS"); do
      label="cmp_${scene}_${ctrl}_${run}"
      echo "=== $label ==="
      bash "$HERE/run_nav_sim.sh" --driver goal --label "$label" --config "$cfg" $extra 2>&1 \
        | grep -E '^\{' | tail -1 >> "$OUT/compare_raw.jsonl"
      line=$(tail -1 "$OUT/compare_raw.jsonl")
      python2 - "$line" "$scene" "$PROFILE" "$ctrl" "$run" >> "$CSV" <<'PYEOF'
import json, sys
raw, scene, profile, ctrl, run = sys.argv[1:6]
try:
    d = json.loads(raw)
except Exception:
    d = {}
cols = ['status','xy_err_m','yaw_err_rad','wall_s','cmd_mean_v','cmd_max_v','cmd_max_dv',
        'cmd_max_dw','xtrack_rms_m','min_clearance_m','safety_scale_min','replan_count','replan_fail']
vals = [str(d.get(c, '')) for c in cols]
print(','.join([scene, profile, ctrl, str(run)] + vals))
PYEOF
    done
  done
done

echo
echo "=== CSV: $CSV ==="
column -s, -t "$CSV" 2>/dev/null || cat "$CSV"
