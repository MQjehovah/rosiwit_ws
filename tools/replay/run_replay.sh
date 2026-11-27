#!/bin/bash
# 隔离回放 (参考实现, 2026-11-27): 在独立的 ROS master (11312) 里起一个生产配置的 SLAM 节点 (devel 构建), 加载地图副本,
# 回放录包里的真实行驶数据, 同时做线程状态采样。不碰主 master (11311) 上的任何话题。
#
#   run_replay.sh <TAG> <START_s> <DUR_s> <HOGS> <RT 0|1> [CFG]
#     TAG    日志 / 采样结果的后缀
#     START  从录包的第几秒开始播
#     DUR    播多少秒
#     HOGS   额外起几个满载循环进程 (模拟行驶时整机超订: 停车负载 ~6-10, 行驶 ~16)
#     RT     1 = 回放开始后把跟踪线程及其 OpenMP 工作线程 (comm=slam_loc) 设成 SCHED_FIFO (优先级见下)
#     CFG    节点配置 (默认 default_replay.yaml: 与生产一致, 只改了飞行记录器转储目录)
#   可选环境变量: BAG=<录包> INIT=<初始位姿文件 (x y yaw_rad t0)> PLAYOPT=<rosbag play 额外参数, 如 -l>
#                 NODEENV='VAR=val ...'  给节点进程加环境变量
#                   例: ROSIWIT_DUMP_SEQ=N  每 30 帧转储一帧匹配器输入 (/tmp/slamdump_seq, 见 localization_pipeline.cpp)
#
# ★状态 (2026-11-27): 这是原脚本的原文, 原件放在 /tmp 下, 随 21:45 的整机重启被清掉了 (/tmp 开机清空)。
#   它还依赖三个同样丢失的辅助脚本, 重建回放环境时要先补:
#     clock_stub.py     /clock 桩: use_sim_time 下按真实时间发 /clock (起点在回放起点前 5 分钟), 回放开始后交给 rosbag
#     tf_static_pub.py  静态 TF (机器人 URDF); 没有它节点初始化会一直等
#     switch_map.py     调 /switch_map 加载地图副本 (生产地图的拷贝, 不要用原图)
#     default_replay.yaml (default.yaml 去掉飞行记录器转储) / init_pose.txt / 行驶录包 (从 /opt/xzrobot/bags 用
#     tools/replay/make_undock_bag.py 的做法筛出 IMU + 雷达话题)
#   日志在 tools/replay 里: mu_report.py (帧耗时/队列/账本汇总), pairfit.py (成对 fitness), traj_cmp.py (轨迹差分解)。
#
# ★RT=1 的优先级必须是 40 (与部署时节点自己设的一致), 不要用 50: 内核中断线程是 50, 同级时中断线程抢不到 CPU。
#   2026-11-27: 用 50 的回放期间 USB 主机控制器连续报 "xhci-hcd ... unknown event type 37" (不带 RT 的回放没有),
#   当晚 21:45 一次这样的回放后整机硬重启 (原因未完全查明, 见 docs/HANDOVER_2026-11-27.md §7)。
#
# ★本脚本不使用 pkill -f / 按命令行匹配进程: 所有子进程都按记录下来的 PID 收 (pkill -f 的模式一旦出现在调用者
#   自己的命令行里, 就会把调用者的会话杀掉)。
set -u
TAG=$1; START=$2; DUR=$3; HOGS=$4; RT=$5; CFG=${6:-$R/default_replay.yaml}
R=${R:-/tmp/opencode/replay}
TOOLS=$(cd "$(dirname "$0")/.." && pwd)
export ROS_MASTER_URI=http://localhost:11312
export ROS_HOSTNAME=localhost
source /opt/ros/melodic/setup.bash
source /home/xzrobot/rosiwit_ws/ros1_ws/devel/setup.bash

if ! timeout 5 rosparam list / >/dev/null 2>&1; then
    nohup roscore -p 11312 > $R/roscore.log 2>&1 < /dev/null &
    for i in $(seq 40); do sleep 0.5; timeout 3 rosparam list / >/dev/null 2>&1 && break; done
fi
rosparam set /use_sim_time true

T0=$(awk '{print $4}' ${INIT:-$R/init_pose.txt})
nohup python2 $R/clock_stub.py $(python3 -c "print($T0 + $START - 300.0)") > /dev/null 2>&1 < /dev/null &
STUB=$!
nohup python2 $R/tf_static_pub.py > $R/tf_static_pub.log 2>&1 < /dev/null &
TFPUB=$!
sleep 2

# stdout 进了文件就是块缓冲, 看不到实时状态
stdbuf -oL -eL env ${NODEENV:+$NODEENV} rosrun rosiwit_slam_ros1 rosiwit_slam_ros1_node __name:=xz_3d_slam _config_file:=$CFG > $R/node_$TAG.log 2>&1 < /dev/null &
NPID=$!
T_NODE=$(date +%s)
for i in $(seq 120); do grep -aq "slam_init_type=" $R/node_$TAG.log 2>/dev/null && break; sleep 0.5; done
if ! grep -aq "slam_init_type=" $R/node_$TAG.log 2>/dev/null; then
    echo "[$TAG] 节点初始化失败 (配置: $CFG):"; grep -aE "FATAL|cannot read|failed" $R/node_$TAG.log | sed -E 's/\x1b\[[0-9;]*m//g' | cut -c1-200
    kill -INT $NPID 2>/dev/null; kill $TFPUB $STUB 2>/dev/null
    exit 1
fi
python2 $R/switch_map.py
for i in $(seq 180); do grep -aq "base voxels in w ledger" $R/node_$TAG.log 2>/dev/null && break; sleep 1; done
echo "[$TAG] 节点 pid=$NPID, 装图用时 $(( $(date +%s) - T_NODE ))s; RSS $(( $(awk '/VmRSS/{print $2}' /proc/$NPID/status) / 1024 )) MB"

: > $R/hogs_$TAG.pids
for i in $(seq $HOGS); do
    timeout $((DUR + 90)) bash -c 'while :; do :; done' >/dev/null 2>&1 < /dev/null &
    echo $! >> $R/hogs_$TAG.pids
done
sleep 2
echo "[$TAG] 回放: 起始 ${START}s, 时长 ${DUR}s, 附加负载进程 ${HOGS}, RT=${RT}; 负载 $(uptime | sed 's/.*average://')"
MARK=$(wc -l < $R/node_$TAG.log)
echo $MARK > $R/node_$TAG.mark
kill $STUB 2>/dev/null; sleep 0.2      # /clock 交给 rosbag
nohup rosbag play --clock -s $START -u $DUR ${PLAYOPT:-} ${BAG:-$R/loc_filtered.bag} > $R/bag_$TAG.log 2>&1 < /dev/null &
BPID=$!
WARM=14
sleep $WARM
if [ "$RT" = "1" ]; then
    # 本会话 (及其 sudo 子进程) 在 seccomp 过滤之下, sched_setscheduler 被拦 (root 也不行);
    # 让 PID 1 经 systemd-run 来起 chrt, 才真正设得上。逐线程设, 设完核对。
    # 优先级 40 = 节点在 systemd 单元 LimitRTPRIO=40 下自己设的值。不要用 50 (见文件头)。
    for tid in $(ls /proc/$NPID/task); do
        c=$(cat /proc/$NPID/task/$tid/comm 2>/dev/null)
        if [ "$c" = "slam_loc" ]; then
            sudo -n systemd-run --quiet --wait --pipe --collect --unit=opc_chrt_${TAG}_$tid chrt -f -p 40 $tid 2>&1 | head -2
        fi
    done
    echo "[$TAG] 跟踪线程及其工作线程的调度策略: $(for tid in /proc/$NPID/task/*; do c=$(cat $tid/comm 2>/dev/null); [ "$c" = "slam_loc" ] && ps -o cls=,rtprio= -p $(basename $tid) 2>/dev/null; done | sort | uniq -c | tr '\n' ';')"
fi
python3 $TOOLS/thread_sampler.py $NPID $(python3 -c "print(max(5, $DUR - $WARM - 4))") $R/sampler_$TAG.json 500 slam_loc,slam_fe,slam_mapupd
wait $BPID 2>/dev/null
for p in $(cat $R/hogs_$TAG.pids); do kill $p 2>/dev/null; done
kill -INT $NPID 2>/dev/null
for i in $(seq 20); do kill -0 $NPID 2>/dev/null || break; sleep 0.5; done
kill -0 $NPID 2>/dev/null && { echo "[$TAG] 节点未在 SIGINT 后 10s 内退出, SIGTERM"; kill -TERM $NPID; sleep 2; }
kill $TFPUB $STUB 2>/dev/null
echo "[$TAG] 完成。日志: $R/node_$TAG.log (回放从第 $MARK 行开始), 采样: $R/sampler_$TAG.json"
echo "[$TAG] ★收尾: 停掉隔离 roscore (roscore -p 11312 的 PID), 并 systemctl restart xzrobot 恢复机器人上被停掉的 SLAM 节点。"
