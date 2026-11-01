#!/bin/bash
# 单次 0.75m 门洞验收 + 旁路录像 (输出 JSON 摘要 + 物理判定)
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
sleep 4
setsid nohup roslaunch "$HOME/sim_verify/door_test3.launch" > /tmp/door_t.log 2>&1 < /dev/null &
for i in $(seq 1 45); do
  timeout 2 rosservice list 2>/dev/null | grep -q '^/gazebo/spawn_urdf_model$' && break; sleep 1
done
for a in 1 2 3 4 5 6; do
  timeout 6 rostopic echo -n1 /odom >/dev/null 2>&1 && break
  timeout 40 rosrun gazebo_ros spawn_model -urdf -param robot_description -model mrobot \
    -x 0 -y 0 -z 0.1 -Y 0 -unpause >/dev/null 2>&1; sleep 5
done
for i in $(seq 1 20); do
  timeout 4 rostopic hz /scan 2>/dev/null | grep -q 'average rate' && break; sleep 2
done
setsid nohup python2 $HOME/sim_verify/map_pub.py \
  "$SIM/maps/doorway/map.pgm" "$SIM/maps/doorway/map.yaml" > /tmp/map_pub_t.log 2>&1 < /dev/null &
for i in $(seq 1 15); do
  timeout 4 rostopic echo -n1 /map/info >/dev/null 2>&1 && break; sleep 1
done
subs=0
for i in $(seq 1 25); do
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
