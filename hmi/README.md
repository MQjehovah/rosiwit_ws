# rosiwit HMI

调试 rosiwit SLAM 机器人的浏览器前端：状态面板 + Three.js 三维视图 + 话题/服务探查。
Vue 3 + TypeScript + Vite + Three.js。

数据分两条路：

```
浏览器 ──ws://…:8089──> rosbridge_websocket   TF / 定位状态 / 日志 / 控制服务（小消息）
浏览器 ──http://…:9100──> hmi_bridge          点云 / 栅格（服务端降采样 + 量化 + zlib）
```

`hmi_bridge` 是可选的，不部署也能用 rosbridge 的 CBOR 通道；但大点云和 `/map` 在远程链路上
基本没法用，详见 [`bridge/README.md`](bridge/README.md)。

## 快速开始

开发机需要 Node ≥ 18。Windows 侧如果通过 `\\wsl.localhost\...` 打开本仓库，不要在 UNC 路径下直接跑 npm
（9P 文件系统会让 esbuild 崩掉），统一用 WSL：

```bash
cd ~/rosiwit_ws/hmi

# 一次性：把免 root 的 Node 装到 ~/.local/node
curl -fsSL https://npmmirror.com/mirrors/node/v18.20.4/node-v18.20.4-linux-x64.tar.xz -o /tmp/node.tar.xz
mkdir -p ~/.local && tar -xJf /tmp/node.tar.xz -C ~/.local
mv ~/.local/node-v18.20.4-linux-x64 ~/.local/node

./dev.sh install       # npm install
./dev.sh dev           # vite，监听 0.0.0.0:5173
./dev.sh test          # 解码 / TF / CBOR 单测（不连设备）
./dev.sh bridge-test   # 桥接端到端测试（自动起 demo bridge）
./dev.sh build         # vue-tsc + 生产构建
```

浏览器打开 `http://<开发机 IP>:5173/`。

### 地址配置

连接地址在顶栏左侧的胶囊里改，会存到 localStorage。也可以用 `.env.local`（已被 gitignore，
避免把隧道 token 提交）：

```bash
cp .env.example .env.local
# VITE_ROSBRIDGE_URL=wss://test.rosiwit.com/remote-terminal/tunnel/<SN>/<TOKEN>/127.0.0.1/8089/
# VITE_TUNNEL_URL=wss://…（同上，会出现在顶栏的「远程隧道」预设里）
```

> 端口别搞混：**8089 才是 rosbridge**；9090 是厂商的 `http_bridge_node`（App 用的 HTTP 通道）。
> 桥接默认是 **9100**（可用 `--port` 改）。

## 订阅模型（RViz 风格）

**只有 TF 自动订阅**（`/tf`、`/tf_static`），其它全部由用户在「数据源」里显式勾选，
选择会存到 localStorage，刷新后保留。

| 数据源 | 话题 | 默认 | 说明 |
|---|---|---|---|
| TF 坐标轴 | `/tf`, `/tf_static` | ✅ 常订阅 | 只控制坐标轴显示 |
| 定位状态 | `/slam/info` | ⬜ | 状态面板 / 顶栏徽章 |
| 融合诊断 | `/slam/fusion_diag` | ⬜ | |
| 系统日志 | `/rosout_agg` | ⬜ | 右栏「日志」 |
| 当前帧（已配准） | `/scan_f_map`（+`/curr_scan_point`） | ⬜ | map 系，未开 debug 也有 |
| 激光雷达 | `/scan` | ⬜ | LaserScan |
| 全局地图点云 | `/global_map` | ⬜ | 需要 `debug_publish: true` |
| 2D 点云 | `/points_2d` | ⬜ | |
| 机器人足迹 | `/robot_footprint` | ⬜ | 机器人本体只依赖 TF，常显示 |
| 路径 | `/global_path`, `/local_path` | ⬜ | |
| 雷达原始帧 | `/vanjee_points722` | ⬜ | 「抓帧」按需取一帧 |
| 栅格地图 | `/map` | ⬜ | 「抓取」按需取一份快照 |
| 行驶轨迹 | 本地累积 | ✅ | 不占带宽 |

## 大数据怎么传

### 方案 A：hmi_bridge（推荐，已在设备上部署验证）

[`bridge/hmi_bridge.py`](bridge/hmi_bridge.py) 在机器人本地降采样/量化/zlib，浏览器用 HTTP 拉二进制帧：

- `/map` 4300 万格 → **实测 1.4MB**（机器人本地 6.4s，隧道约 15s），按需快照。
- `/scan_f_map` → 体素 5cm + int16 厘米 + zlib，**约 4KB/帧**，HMI 端约 4Hz 轮询。
- `/global_map`、`/vanjee` → 一次性抓取，体素 0.2 / 0.05m（vanjee 5000 点约 17KB）。
- 客户端是 `src/ros/bridge.ts`，用 `fetch` + `DecompressionStream('deflate')`，无第三方依赖。
- 部署：`bridge/hmi-bridge.service`（systemd）+ `bridge/README.md`。

HMI 会自动从 rosbridge 地址推导桥接地址（`8089→9100`、`ws(s)→http(s)`），也可以在
「数据源 → 二进制桥接」里手填。远程隧道需要把目标端口改成 9100。

### 方案 B：rosbridge CBOR

没有桥接时，大消息用 rosbridge 的 `compression: "cbor"`（`src/ros/cbor.ts` 自带最小 CBOR 解码器）。
点云只发原始字节，比 JSON 小，但 `/map` 仍要 1~2 分钟且期间实时数据暂停，所以是显式按钮 + 本地缓存。

## 打开全局地图点云（可选）

`/global_map`、`/curr_scan_point` 由 `rosiwit_slam_ros1` 的 `debug_publish` 开关控制，默认关闭：

```bash
# 设备上（xzrobot / xzyz2022!）
sudo sed -i 's/debug_publish: false/debug_publish: true/' \
  /opt/ros/melodic/share/rosiwit_slam_ros1/config/default.yaml
sudo systemctl restart xzrobot
```

⚠️ **会重启整个 xzrobot bringup（SLAM/导航都会中断）**，请在机器人空闲时操作。
`/global_map` 是 latched 的，每次进模式只发一份。

## 界面

- **顶栏**：连接状态胶囊（点开改地址/局域网/远程隧道/断开）、定位状态、模式、匹配度、地图名、左右栏开关。
- **左栏**
  - **定位状态**：`location_status`、`slam_state`、匹配度条 + 近 48s 趋势、地图/面积/TF 帧数，
    以及完整位姿 **xyz（m）+ rpy（°）**。没订阅 `/slam/info` 时显示「未订阅」并提供一键订阅。
  - **数据源**：桥接开关/地址/检测、图层勾选（勾选即订阅）、点大小、抓帧/抓取按钮、数据说明。
  - **初始位姿**、**控制**：折叠，减少干扰。
- **中间三维视图**：拖动旋转、右键平移、滚轮缩放；双击放置初始位姿；`M` 测距、`F` 适配、`T` 顶视、`Esc` 取消；
  右下角工具条（顶视/适配/跟随/测距/截图）。
- **右栏**
  - 话题：默认「只看活跃」（已订阅或近期有数据），点击行临时订阅并在「消息」里查看。
  - 日志：按级别过滤、暂停、按来源静音（点击来源名）、连续重复消息合并计数。
  - 消息：任意话题的截断 JSON、暂停、复制。

## 坐标与渲染约定

- Three.js 场景直接用 ROS 约定：**Z 轴向上**，`map` 系为世界原点。
- 机器人位姿优先 `tf: map→base_link`，回退到 `/slam/info.robot_pose`。
- 位姿显示完整的 **xyz / rpy（ZYX）**；渲染机器人时用完整四元数。
- 点云按消息 `header.frame_id` 查 TF 后放到 map 系；`/scan_f_map` 本身就是 `map` 系。
- 栅格地图用 `map.info.origin` 摆放，`z=0.01`；降采样按块取最大占据值，墙线保持连续。

## 目录

```
hmi/
├── bridge/
│   ├── hmi_bridge.py    # ROS 侧降采样/压缩桥（ROS1，Python 2.7/3，无额外依赖）
│   ├── hmi-bridge.service # systemd 单元（PartOf=xzrobot）
│   └── README.md        # 部署、API、帧格式、实测数据
├── src/
│   ├── ros/
│   │   ├── client.ts    # rosbridge 客户端（JSON + CBOR、订阅/发布/服务、重连）
│   │   ├── bridge.ts    # hmi_bridge HTTP 二进制客户端
│   │   ├── cbor.ts      # 最小 CBOR 解码（typed-array tag）
│   │   ├── decode.ts    # PointCloud2 / PointCloud / OccupancyGrid / LaserScan / Path / Polygon
│   │   ├── tf.ts        # TF 树、lookup、四元数 → rpy
│   │   ├── gridCache.ts # 栅格快照 IndexedDB 缓存
│   │   └── app.ts       # 全局状态、订阅编排、桥接、快照、服务
│   ├── three/
│   │   ├── SceneView.vue# 三维视图、交互、桥接轮询、动画循环
│   │   ├── layers.ts    # 点云/栅格/机器人/TF/路径/轨迹
│   │   └── gridImage.ts # OccupancyGrid → RGBA 纹理
│   ├── components/      # ConnectionPill / ControlPanel / InspectorPanel
│   └── App.vue, main.ts
├── tests/               # 解码/TF/CBOR 单测 + 桥接端到端测试
└── dev.sh
```

## 已知限制

- 只支持 ROS1 rosbridge；换 ROS2 时替换 `ros/client.ts` / `ros/bridge.ts` 的传输层。
- rosbridge `cbor-raw` 在设备 0.11.16 上不可用（`OutboundMessage._buff` 未填充），所以走结构化 `cbor`。
- 桥接是 HTTP 轮询（3~4 Hz），刻意不做推送：慢链路自然降频，不会积压内存。
- `/map` 是快照而非连续更新；需要刷新就再点一次「抓取」。
- 桥接没有鉴权，默认绑定 `0.0.0.0:9100`；建议只通过隧道/内网访问，必要时 `--host 127.0.0.1`。
- 位姿的 z/rpy 是 TF 真值；如果 `map→odom` 的 z 有偏移，HMI 会如实显示（这通常是有用的诊断信息）。
