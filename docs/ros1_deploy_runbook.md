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

## 7. 已知差异/待办

- 我们节点启动是 `startup_mode: mapping`（`config/default.yaml`）。App 切图后会变 LOCALIZATION；
  若要开机即定位，改这个参数。
- `/lio_odom`、`/slam/fusion_diag` 仍在发；`/world_cloud` 已删除（导航改订原始雷达，见
  `docs/ros1_thread_model.md` §2）。
- 本次没跑长时间定位精度对比（需要实车走一段）；建议上机后跑 10 分钟并看
  `[locstate]` 的 `matching` 与 `location_status` 抖动。
