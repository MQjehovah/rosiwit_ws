#!/bin/bash
# 单次 0.75/0.80m 门洞验收 + 旁路录像 (输出 JSON 摘要 + odom/真值双口径判定)
# 用法: door_accept_once.sh [label]
source /opt/ros/melodic/setup.bash
source ~/rosiwit_ws/ros1_ws/devel/setup.bash
export OMP_NUM_THREADS=2
SIM=$HOME/rosiwit_ws/ros1_ws/src/rosiwit_simulation_ros1
E=/mnt/e/rosiwit_ws
LABEL=${1:-door075}
cp "$E/scripts/sim/trace_nav.py" "$E/scripts/sim/nav_sim_goal.py" ~/rosiwit_ws/scripts/sim/
OUT=$HOME/sim_verify/trace_door.csv
rm -f "$OUT" "$OUT.path"

pkill -f 'roslaunch.*door_test' 2>/dev/null; pkill -f gzserver 2>/dev/null
pkill -f map_pub.py 2>/dev/null; pkill -f trace_nav.py 2>/dev/null
pkill -f spawn_model 2>/dev/null
sleep 5
setsid nohup roslaunch "$HOME/sim_verify/door_test3.launch" > /tmp/door_t.log 2>&1 < /dev/null &
for i in $(seq 1 60); do
  timeout 2 rosservice list 2>/dev/null | grep -q '^/gazebo/spawn_urdf_model$' && break; sleep 1
done
# 冲模型: 直到 /odom 出现 (每次重置模型, 避免 "already exists")
ok=0
for a in $(seq 1 8); do
  if timeout 5 rostopic echo -n1 /odom >/dev/null 2>&1; then ok=1; break; fi
  timeout 30 rosrun gazebo_ros spawn_model -urdf -param robot_description -model mrobot \
    -x 0 -y 0 -z 0.1 -Y 0 -unpause >/dev/null 2>&1
  sleep 4
done
echo "odom_ready=$ok"
# 等 TF: map -> base_footprint 可用 (节点才有位姿)
tfok=0
for i in $(seq 1 30); do
  if timeout 4 rosrun tf tf_echo map base_footprint 2>/dev/null | grep -q Translation; then tfok=1; break; fi
  sleep 2
done
echo "tf_ready=$tfok"
for i in $(seq 1 20); do
  timeout 4 rostopic hz /scan 2>/dev/null | grep -q 'average rate' && break; sleep 2
done
setsid nohup python2 $HOME/sim_verify/map_pub.py \
  "$SIM/maps/doorway/map.pgm" "$SIM/maps/doorway/map.yaml" > /tmp/map_pub_t.log 2>&1 < /dev/null &
for i in $(seq 1 20); do
  timeout 4 rostopic echo -n1 /map/info >/dev/null 2>&1 && break; sleep 1
done
# 等 /nav_diagnostics (节点主循环已起) 再发目标
dok=0
for i in $(seq 1 30); do
  if timeout 4 rostopic echo -n1 /nav_diagnostics >/dev/null 2>&1; then dok=1; break; fi
  sleep 2
done
echo "diag_ready=$dok"
subs=0
for i in $(seq 1 20); do
  subs=$(timeout 3 rostopic info /goal_pose 2>/dev/null | sed -n '/Subscribers:/,$p' | grep -c '\*')
  [ "$subs" -ge 1 ] && break
  sleep 2
done
echo "goal_pose subscribers=$subs"
sleep 2
setsid nohup python2 $HOME/rosiwit_ws/scripts/sim/trace_nav.py "$OUT" 20 > /tmp/trace.log 2>&1 < /dev/null &
sleep 2
timeout 220 python2 "$HOME/rosiwit_ws/scripts/sim/nav_sim_goal.py" "$LABEL" 190 "" 0.0,3.8,1.5708 2>&1 \
  | grep -E '^\{' | tail -1
sleep 2
pkill -f trace_nav.py 2>/dev/null
pkill -f map_pub.py 2>/dev/null
pkill -INT -f 'roslaunch.*door_test' 2>/dev/null; sleep 2; pkill -f gzserver 2>/dev/null
echo "########## analyze ##########"
python2 "$E/scripts/sim/analyze_trace.py" "$OUT" 2.9
