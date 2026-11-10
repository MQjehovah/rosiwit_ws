#!/bin/bash
# 0.75m 门洞验收: 跑 N 次 (默认 3), 每次打印 JSON 摘要 + 门带内物理判定
# 判据 (不看 goal_status, 因为 odom 打滑会造成假成功):
#   gate_verdict[collision_0.70] == PASS  且  safety_hist 门带内无 STOP
# 用法: door_accept.sh [N]
N=${1:-3}
E=/mnt/e/rosiwit_ws
for i in $(seq 1 "$N"); do
  echo "########## run $i / $N ##########"
  bash "$E/scripts/sim/door_accept_once.sh" "door075_r$i"
done
