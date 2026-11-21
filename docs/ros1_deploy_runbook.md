# rosiwit_slam_ros1 部署手册（设备 14003AO7 实操，2026-11-21）

> 目标：用我们自己的节点替换厂商 `xz_3d_slam_node`，**只换 SLAM 节点**，底盘/相机/App 仍由厂商 `xzrobot.service` 管理。

## 0. 结论先行（这次实测）

| 项 | 结果 |
|---|---|
| 设备原生编译 | ✅ 8 核 `nice -n 19 catkin_make -j3`，**约 15 分钟**（服务器 qemu 仿真要 ~50 分钟） |
| deb 安装 | ✅ `rosiwit-slam-ros1 0.2.0 arm64`，`ldd` 零缺失 |
| 持久替换 | ✅ 改厂商 launch 一行 + `systemctl restart xzrobot`，重启后自动起我们的节点 |
| App 兼容 | ✅ App 自动调用 `/switch_map`（`map_name: "1楼"`），`/slam/info` 正常 |
| 踩坑 1 | **deb 必须包含 `package.xml`**，否则 `$(find rosiwit_slam_ros1)` 失败 → `xzrobot_start.py` 崩（`ResourceNotFound`），SLAM 起不来 |
| 踩坑 2 | **`robot_monitor` 会用 gdb attach 高 CPU 进程**（`gdb -q -n -batch -p <pid> -ex "thread N" -ex bt`），ptrace 会把节点冻住 → 表现为"卡死" |

## 1. 设备原生编译（推荐）

设备已具备完整编译环境（ceres 1.13 / pcl 1.8 / opencv 3.2 / eigen 3.3.4 / yaml-cpp / gtest / gcc 7.5），**只缺 Sophus**。

```bash
# 1) 准备源码 + Sophus（从 amd64 镜像里取：tar -C /usr/local/include -czf - sophus）
mkdir -p ~/rosiwit_build && cd ~/rosiwit_build
curl -sSL -o src.tgz    '<OSS: build_src.tgz>'
curl -sSL -o sophus.tgz '<OSS: sophus.tgz>'
tar xzf src.tgz && tar xzf sophus.tgz -C /tmp
sudo cp -r /tmp/sophus /usr/local/include/

# 2) 工作区里不要重复编译厂商 msg 包（设备 /opt/ros 里已装）
rm -rf ros1_ws/src/xzrobot_ros_dependency

# 3) 编译（nice 降低对厂商节点的影响）
cd ros1_ws && source /opt/ros/melodic/setup.bash
nice -n 19 catkin_make -j3 -DCATKIN_ENABLE_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_DISABLE_FIND_PACKAGE_GTSAM=ON
```

## 2. 打包 deb（**必须含 package.xml**）

```bash
cd ~/rosiwit_build
rm -rf debroot && mkdir -p debroot/DEBIAN \
  debroot/opt/ros/melodic/lib/rosiwit_slam_ros1 \
  debroot/opt/ros/melodic/share/rosiwit_slam_ros1
cp ros1_ws/devel/lib/rosiwit_slam_ros1/* debroot/opt/ros/melodic/lib/rosiwit_slam_ros1/
cp -a ros1_ws/devel/lib/librosiwit*.so* debroot/opt/ros/melodic/lib/
cp -a ros1_ws/src/rosiwit_slam_ros1/{launch,config,package.xml} \
      debroot/opt/ros/melodic/share/rosiwit_slam_ros1/     # ← package.xml 不能漏
printf '%s\n' 'Package: rosiwit-slam-ros1' 'Version: 0.2.0' 'Section: misc' \
  'Priority: optional' 'Architecture: arm64' 'Maintainer: Rosiwit <dev@rosiwit.com>' \
  'Description: rosiwit_slam ROS1 interface node (arm64)' > debroot/DEBIAN/control
dpkg-deb --build debroot rosiwit-slam-ros1_0.2.0_arm64.deb
sudo dpkg -i rosiwit-slam-ros1_0.2.0_arm64.deb
source /opt/ros/melodic/setup.bash && rospack find rosiwit_slam_ros1   # 必须能找到
```

> 漏 `package.xml` 的表现：`systemctl restart xzrobot` 后整个 `xzrobot_all` 反复 `process has died`，
> journal 里是 `rospkg.common.ResourceNotFound: rosiwit_slam_ros1`，且 `rosnode list | grep xz_3d_slam` 为空。

## 3. 关掉会 gdb attach 的监控（调试期）

`robot_monitor` 是那个会在高 CPU 时 `gdb -batch -p <pid> -ex bt` 的进程，ptrace 期间节点被冻住。
调试期间禁用（可逆）：

```bash
rosnode kill /robot_monitor
sudo mv /opt/ros/melodic/lib/robot_monitor/robot_monitor \
        /opt/ros/melodic/lib/robot_monitor/robot_monitor.disabled
# 恢复：sudo mv .../robot_monitor.disabled .../robot_monitor && sudo systemctl restart xzrobot
```

`xz_3d_monitor` **不是**元凶（它只做 livox 话题频率与 CPU 统计），不用动。

## 4. 持久替换（只改一行）

厂商由 `xzrobot_start.py:198` 拉起 `/opt/ros/melodic/share/xz_3d_slam/launch/xz_3d_slam.launch`，
里面第 34 行是唯一的 SLAM 节点定义。**只替换这一行**（其余保持厂商原样）：

```bash
F=/opt/ros/melodic/share/xz_3d_slam/launch/xz_3d_slam.launch
sudo cp -a $F $F.bak_vendor
sudo sed -i '34s|.*|  <node name="xz_3d_slam" pkg="rosiwit_slam_ros1" type="rosiwit_slam_ros1_node" output="log">\n    <param name="config_file" value="$(find rosiwit_slam_ros1)/config/default.yaml" />\n  </node>|' $F
python3 -c "import xml.etree.ElementTree as ET;ET.parse('$F');print('XML_OK')"

sudo systemctl restart xzrobot
```

节点名保持 `xz_3d_slam`：`xzrobot_all` / App / `xz_3d_monitor` 都按这个部署名找它。

## 5. 验证

```bash
rosnode list | grep '^/xz_3d_slam$'                 # 有
pgrep -c -f rosiwit_slam_ros1_node                  # 1（厂商进程为 0）
rosparam list | grep /xz_3d_slam/config_file        # 有
rosservice list | grep -cE '^/(slam/mode_switch|switch_map|relocation|stop_map_operation|global_localization|finish_task|start_task|slam/dump_recorder|update_map_operation)$'   # 9
rostopic hz /slam/info                              # ~10 Hz
rostopic echo -n1 /slam/info                        # map_name / location_status
```

切图（App 通常自己会调）：

```bash
rosservice call /switch_map "{map_dir: '/opt/xzrobot/maps/1794281848635', map_name: '3AO7_XC2', \
  initial_pose: {header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}"
```

之后 `location_status` 需要一次正确的初始位姿（App 里给，或发 `/initialpose`）；位姿不对时会看到
`initial seed rejected: fitness=... < 0.8 -> INIT`，然后 `LOST` —— 这是**保护**，不是故障。

## 6. 回滚

```bash
F=/opt/ros/melodic/share/xz_3d_slam/launch/xz_3d_slam.launch
sudo mv $F.bak_vendor $F
sudo systemctl restart xzrobot
# 需要时再恢复 robot_monitor 二进制（见 §3）
```

## 6.5 假定位（"明明是错的说成功"）—— 2026-11-21 现场

**现象**：`/slam/info` 报 `location_status=0`(LOCALIZED)、`matching≈0.80`，但位姿明显是错的。

**日志证据**（`/tmp/<uuid>/xz_3d_slam-*.log`）：

```
initial seed rejected: fitness=0.775956 < 0.8 -> INIT        # App 种子被拒, 正确
[locstate] mode=LOCALIZATION st=1 matching=0.804 pos=(0.105,0.010)   # 4s 后在 0.804 处被接受
[DropProbe] fitness=0.79~0.84 correction_m=0.03 target_points=12195 accepted=1
```

**机制**：种子接受线 `global.reloc_accept_score=0.80`，而种子判定只看**局部** GICP fitness
（局部子图仅 1.2 万点）。在错误位姿附近只要有可配的墙面，fitness 就能到 0.80~0.84 →
被判"定位成功"；之后局部子图围绕错误位姿裁剪，永远"自洽"，不会自己掉出来。

**处置**（已在本机执行）：

```bash
C=/opt/ros/melodic/share/rosiwit_slam_ros1/config/default.yaml
sudo cp -a $C $C.bak_080
sudo sed -i 's/reloc_accept_score: 0.80/reloc_accept_score: 0.85/' $C   # 与厂商 th_fitness_relocalization 对齐
sudo systemctl restart xzrobot
```

结果：`location_status` 从 0 变为 **2 (LOST)** —— 弱锁被拒，界面如实显示未定位。
阈值链保持合法：`min_coverage 0.35 < lost 0.75 < credible 0.80 ≤ reloc_accept 0.85`。

**代价**：如果真实位姿也只能到 0.82 左右，会一直 LOST（宁可失败也不给错位姿）。
此时要么 App 里人工给一次初始位姿，要么把 `reloc_accept_score` 降到 0.82 再观察。

**根治方向（尚未做）**：种子接受加"连续 N 帧 healthy 才报 LOCALIZED"或引入全局一致性
（scan-context 多候选 / 更大范围校验），而不是单帧局部 fitness。

## 7. 已知差异/待办

- 我们节点启动是 `startup_mode: mapping`（`config/default.yaml`）。App 切图后会变 LOCALIZATION；
  若要开机即定位，改这个参数。
- `/lio_odom`、`/slam/fusion_diag` 仍在发；`/world_cloud` 已删除（导航改订原始雷达，见
  `docs/ros1_thread_model.md` §2）。
- 本次没跑长时间定位精度对比（需要实车走一段）；建议上机后跑 10 分钟并看
  `[locstate]` 的 `matching` 与 `location_status` 抖动。

## 8. 2026-11-22 更新（0.2.2：相对锚判据 + overlay 沉淀）

### 8.1 新增配置（`localization.submap` 与 `localization.maintainer`）

```yaml
localization:
  submap:
    anchor_min_inliers: 50     # 绝对下限(防 3 个点也当锚)
    anchor_min_ratio: 0.5      # inliers / 匹配前可见点数(dynamic_filter 之前)
    anchor_min_eig_ratio: 0.001
    anchor_patience: 5         # 连续弱锚帧 -> 开会话地图
    anchor_close_patience: 10  # 连续好锚帧 -> 关
  maintainer:
    persistency_unit: session  # frame|view|session|day
    persistency_hit_gain: 8    # 每个单位 +8 -> anchor 128 = 16 个会话
    persistency_anchor: 128
    persist_hits: 3            # 按 unit 计票: session => 3 个会话
    delta_min: 0.3
    novel_min_cluster: 3       # 26 邻域最小簇(按整帧扫描体素计数)
```

### 8.2 为什么改

实车窄过道整帧只有 ~176 个 0.4m 体素、inliers 最多 ~105；`anchor_min_inliers=500`
永远不可达 → overlay 不写、`session_map` 不关。改为相对可见证据后：
`inl=170/vis=172` → `anchor=OK`，`session_map` 10 帧后关闭，`novel_commits` 2~5/s。

### 8.3 新日志字段

`[quality]`（1 Hz）新增 `vis=`、`novel_votes=`、`novel_commits=`。
`[maint]`（每 100 帧）：

```
[maint] scan=3627 seen=3088 base=262 kd=71 cand=11 cluster_rej=0 votes=11 commits=11
```

- `seen`：落在已沉淀体素的点；`base`：被底图同体素挡住；`kd`：离底图 < `delta_min`；
- `cand`：候选体素；`cluster_rej`：被最小簇拒掉；`votes/commits`：本帧计票/写入。
- 排查"新结构不沉淀"：看 `cand` 是否涨、`cluster_rej` 是否吃掉、`base/kd` 是否挡住。

### 8.4 已知风险

- **`dynamic_filter` 的选择偏差**：关掉后同一帧 fitness 从 0.86 掉到 0.72。
  当前 `[quality]` 的 fitness 是在滤除后的点上算的；健康判定可能偏乐观。
- **簇阈值 3** 对稀疏雷达偏严（仿真 16 线在 3~4m 处点数密度低）；设备万集 722 更密，
  先在设备上看 `[maint] cluster_rej` 再定。
- **`persistency_cov`** 在 `w=8/会话`、`anchor=128` 下需要 **16 个会话** 才会 >0，
  前期一直是 0.000 属正常。
