#!/usr/bin/env bash
# 无界面跑闭环仿真验收（persons + car 两个场景）；全部 PASS 返回 0。
# 用法: scripts/run_sim_test.sh [额外 roslaunch 参数...]
set -o pipefail

WS=$(cd "$(dirname "$0")/../../.." && pwd)

if [ -f /opt/ros/melodic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/melodic/setup.bash
fi
if [ -f "$WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/devel/setup.bash"
else
  echo "未找到 $WS/devel/setup.bash，请先 catkin_make" >&2
  exit 2
fi

run_scenario() {
  local scenario="$1"
  shift
  local log="/tmp/object_semantics_sim_${scenario}.log"
  timeout 90 roslaunch rosiwit_perception_ros1 sim_object_semantics.launch \
    "scenario:=$scenario" "$@" >"$log" 2>&1
  local status=$?
  cat "$log"
  if [ $status -eq 124 ]; then
    echo "SIM TIMEOUT ($scenario)" >&2
    return 1
  fi
  if grep -q "RESULT: PASS" "$log"; then
    return 0
  fi
  echo "SIM FAIL ($scenario)" >&2
  return 1
}

FAILED=0
run_scenario persons || FAILED=1
run_scenario car || FAILED=1
exit $FAILED
