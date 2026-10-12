# slam-mapping —— rosiwit_slam 离线建图镜像

镜像 `rosiwit/slam-mapping:humble`。把仓库里的 `src/rosiwit_slam` 建图能力封装成镜像：
启动时指定一个录包目录，容器内自动「起 SLAM 节点 → 播包 → 实时打印进度 → 调服务存图 → 退出」。

> **已部署**: 服务器 `192.168.31.34`（xzrobot），测试录包 `~/bags/bag_local`，产物目录 `~/maps`。

## 1. 目录结构

构建上下文 = **仓库根目录**（见根目录 `.dockerignore`）：

```
<repo>/
├── .dockerignore                       # 限定上下文只含 docker/ 与 src/rosiwit_slam
├── docker/
│   ├── README.md                       # 多镜像总览/约定
│   ├── build.sh                        # 构建辅助脚本
│   └── slam-mapping/                   # ← 本镜像
│       ├── Dockerfile
│       ├── entrypoint.sh
│       ├── ros.asc                     # ROS 源续期签名密钥
│       ├── README.md
│       └── scripts/
│           └── normalize_bag.py        # 录包归一化(rosbag2/zip/裸db3/ROS1 bag)
└── src/
    └── rosiwit_slam/                   # 源码头, 直接 COPY, 不复制
```

源码更新后**无需拷贝**，直接重新 build。

## 2. 构建镜像

在**仓库根目录**执行：

```bash
# 方式一: 辅助脚本
./docker/build.sh slam-mapping

# 方式二: 原生
docker build -f docker/slam-mapping/Dockerfile -t rosiwit/slam-mapping:humble .
```

> 首次构建需下载依赖并完整编译（约 10~20 分钟）；有 apt 缓存时会快很多。

## 3. 运行建图

```bash
# 录包目录挂到 /data/bag (只读), 产物目录挂到 /output
docker run --rm \
    -v ~/bags/bag_local:/data/bag:ro \
    -v ~/maps:/output \
    rosiwit/slam-mapping:humble
```

运行时终端输出形如：

```
[MAP-START]   bag=/data/bag duration=158.99s rate=1.0 config=livox.yaml
[MAP-READY]   slam 节点就绪
[MAP-PROGRESS] 10% (已播放 16s / 包时长 158.994s)
[MAP-PROGRESS] 50% ...
[MAP-PROGRESS] 100% 播包完成, 等待 SLAM 收尾...
[MAP-SAVE]    调用 /save_map -> /output/map.pcd
[MAP-FINISH]  建图完成, 产物在 /output
```

结束后 `$HOME/maps` 里得到：

- `map.pcd`         —— 3D 点云地图（全局融合）
- `map.xzmap`       —— 设备端 3D 关键帧地图（自研 JSON 格式：`kf_frame_poses` + `key_frame_points`，int 量化 mm/mrad，id 从 1 起；由 rosiwit_slam 在 `/save_map` 时原生导出，模块 `src/io/xzmap_writer`，与 SLAM 核心解耦）
- `map.pcd.scdb`    —— ScanContext 关键帧检索库（重定位用）
- `map.pgm/.yaml`   —— 2D 栅格地图（`GRID_MAP=0` 可关）
- `map.png`         —— 2D 栅格 PNG 预览（≤2048px）
- `manifest.json`   —— 元数据（关键帧数/点数/栅格分辨率与 origin/源 bag 配置）
- `slam_node.log`   —— 完整 SLAM 运行日志

也可以后台跑并跟踪进度：

```bash
docker run -d --name slam_map \
    -v ~/bags/bag_local:/data/bag:ro -v ~/maps:/output \
    rosiwit/slam-mapping:humble
docker logs -f slam_map          # 实时看进度
```

## 4. 可调参数（`docker run -e KEY=VALUE`）

| 变量 | 默认 | 说明 |
|---|---|---|
| `BAG_PATH` | `/data/bag` | 容器内 rosbag2 目录（含 metadata.yaml） |
| `CONFIG_FILE` | `livox.yaml` | 包内 config 下的配置（`livox.yaml` / `default.yaml` / `sim_vlp16.yaml`） |
| `LIDAR_TOPIC` | `/livox/lidar` | 雷达话题 |
| `IMU_TOPIC` | `/livox/imu` | IMU 话题 |
| `RATE` | `1.0` | 播包倍速（`2.0` = 两倍速跑完，进度百分比按倍速换算） |
| `OUTPUT_DIR` | `/output` | 地图输出目录（挂载点） |
| `MAP_NAME` | `map` | 输出文件名前缀 |
| `GRID_MAP` | `1` | 是否同时保存 2D 栅格（1/0） |

例：用默认配置（话题 `/lidar` `/imu`）+ 三倍速：

```bash
docker run --rm \
    -v ~/bags/rosbag2_2026_08_10-14_30_01:/data/bag:ro -v ~/maps:/output \
    -e CONFIG_FILE=default.yaml -e RATE=3.0 \
    rosiwit/slam-mapping:humble
```

## 5. 服务器（192.168.31.34）上的日常使用

```bash
cd <repo>                      # 含 docker/ 与 src/
git pull                       # 或 rsync 最新源码
./docker/build.sh slam-mapping
docker run --rm -v ~/bags/bag_local:/data/bag:ro -v ~/maps:/output rosiwit/slam-mapping:humble
```

## 6. 工作原理与已知坑（本镜像已全部处理）

1. **底座**: 服务器无法直连 Docker Hub，复用本地已有 `ros:humble-ros-base`；apt 走 packages.ros.org + Ubuntu 官方源。
2. **ROS 源签名密钥过期**: 老底座内置密钥 2025-06 到期，apt 会拒绝 ROS 源。镜像内用 `COPY docker/slam-mapping/ros.asc`（rosdistro 官方续期版，有效期至 2030）注入并 `gpg --dearmor` 转二进制——
   - 不能现场 curl 下载：网络中间层缓存会返回旧证书；
   - 不能放 armored 原文：老镜像的 gpgv 2.2.27 不认 ASCII 格式 keyring（`invalid packet ctb=2d`）。
3. **rosbag2 ABI 混版**: 基础镜像预装 rosbag2 0.15.9，单独装 rosbag2-py 0.15.16 会符号冲突导致 `ros2 bag play` 起不来；Dockerfile 显式整套安装一致版本的 rosbag2 全家桶。
4. **`/save_map` 参数语义**: 传的是**文件名**（节点内部拼 `map_directory`，默认为节点工作目录 `/ws`），传绝对路径会导致 PCL 打开失败并使节点崩溃；entrypoint 传文件名后再拷贝到 `/output`。`/save_grid_map` 则用完整路径。
5. 进度 = 已播放时长 × 倍速 ÷ 包时长（从 `metadata.yaml` 解析），播完调 `/save_map` + `/save_grid_map`（带超时与 success 校验，节点异常会打印日志尾部并以非 0 退出）。
