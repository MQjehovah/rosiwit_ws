# rosiwit HMI

调试 rosiwit SLAM 机器人的浏览器前端：状态面板 + Three.js 三维视图 + 话题/服务探查。
Vue 3 + TypeScript + Vite + Three.js，直接消费机器人上的 **rosbridge (ROS1)**。

```
浏览器 ──ws://192.168.1.100:8089──> rosbridge_websocket ──> /slam/info /tf /scan_f_map /map ...
```

## 快速开始

开发机需要 Node ≥ 18。Windows 侧如果通过 `\\wsl.localhost\...` 打开本仓库，不要在 UNC 路径下直接跑 npm
（9P 文件系统会让 esbuild 崩掉），统一用 WSL：

```bash
cd ~/rosiwit_ws/hmi

# 一次性：把免 root 的 Node 装到 ~/.local/node
curl -fsSL https://npmmirror.com/mirrors/node/v18.20.4/node-v18.20.4-linux-x64.tar.xz -o /tmp/node.tar.xz
mkdir -p ~/.local && tar -xJf /tmp/node.tar.xz -C ~/.local
mv ~/.local/node-v18.20.4-linux-x64 ~/.local/node

./dev.sh install     # npm install
./dev.sh dev         # vite，监听 0.0.0.0:5173
./dev.sh test        # 纯解码/TF 单测（不连设备）
./dev.sh build       # vue-tsc + 生产构建
```

浏览器打开 `http://<开发机 IP>:5173/`。默认连接 `ws://192.168.1.100:8089`，地址可以在左上角改，
会存到 localStorage。

> 端口别搞混：**8089 才是 rosbridge**；9090 是厂商的 `http_bridge_node`（App 用的 HTTP 通道，不是
> rosbridge 协议）。

## 现在能看到什么

| 数据 | 话题 | 频率 / 单帧大小 | 说明 |
|---|---|---|---|
| 定位状态 | `/slam/info` | ~7 Hz / 43 B | 定位状态、匹配度、地图名、位姿 |
| TF | `/tf`, `/tf_static` | ~100 Hz | `map→base_link` 驱动机器人位姿与坐标轴 |
| **当前帧（已配准）** | `/scan_f_map` | 10 Hz / ~37 KB | `sensor_msgs/PointCloud`，点已经在 **map 系** |
| 原始雷达 | `/vanjee_points722` | 10 Hz / ~1 MB | 需手动「抓一帧」，不持续订阅 |
| **栅格地图** | `/map` | 1 Hz / **43 MB** | 7453×5782（4300 万格），手动抓一份快照 |
| 全局地图点云 | `/global_map` | latched | **需要 `debug_publish: true`** |
| 更细的当前帧 | `/curr_scan_point` | 10 Hz | **需要 `debug_publish: true`** |

所以**不开 `debug_publish` 也能**看到：二维全局栅格地图 + 当前帧在其中的配准位置 + 机器人位姿/轨迹。

### 为什么大消息要单独处理

rosbridge 默认把消息编成 JSON。`/map` 一帧 4300 万个 `int8`，JSON 文本 100 MB 以上，实测 2 分钟都收不完，
而且 rosbridge 是单线程的，期间所有话题一起停更。

因此 HMI 对大消息订阅使用 rosbridge 的二进制通道：

```
{ op: "subscribe", topic: "/scan_f_map", compression: "cbor", queue_length: 1 }
```

CBOR 里的数值数组用 typed-array tag 打包，体积接近原始字节。前端 `src/ros/cbor.ts` 是一个
最小 CBOR 解码器（不引第三方库），`src/ros/decode.ts` 负责把 `PointCloud2` / `PointCloud` /
`OccupancyGrid` 解成渲染用的 Float32Array。

即便如此，`/map` 的单帧 43 MB 在设备端要逐个打包，实测约 **1~2 分钟**，期间实时数据暂停。
所以它是显式的「抓取栅格地图」按钮，而不是持续订阅。抓完后降采样到 800 万像素纹理，
按 3×3 取最大占据值，墙线保持连续。

## 打开全局地图点云（可选）

`/global_map`、`/curr_scan_point` 由 `rosiwit_slam_ros1` 的 `debug_publish` 开关控制，默认关闭：

```bash
# 设备上（xzrobot / xzyz2022!）
sudo sed -i 's/debug_publish: false/debug_publish: true/' \
  /opt/ros/melodic/share/rosiwit_slam_ros1/config/default.yaml
sudo systemctl restart xzrobot
```

⚠️ **会重启整个 xzrobot bringup（SLAM/导航都会中断）**，请在机器人空闲时操作。
改完回来点左侧「重新加载全局地图」。`/global_map` 是 latched 的，每次进模式只发一份。

如果不想动设备，也可以直接抓一份 `/map` 栅格快照，二维地图信息是等价的。

## 界面

- **左栏**
  - rosbridge：地址、连接状态、数据年龄（最近一条 `/slam/info` 或 `/tf` 距现在多久）。
  - 定位状态：`location_status`（0 正常 / 1 恢复中 / 2 失败）、`slam_state`、匹配度进度条（0.80 刻度）、地图/面积/位姿。
  - 图层：点云、扫描、栅格、路径、机器人、TF、轨迹。关掉即退订。
  - 地图/点云：重新加载全局地图、抓一帧雷达、抓取栅格地图、复制 `debug_publish` 命令。
  - 初始位姿：地图上点选 → 发布 `/initialpose`；或填数值后调用 `/relocation`。
  - 控制：模式切换（1=定位 / 0=建图）、结束建图、dump recorder（都会二次确认）。
- **中间三维视图**：左键旋转、右键平移、滚轮缩放；`Esc` 取消点选；顶视/适配内容/跟随/截图。
- **右栏**
  - 话题：常用话题目录 + hz/KB/年龄，点击行临时订阅并在「消息」里看最新一条。
  - 日志：`/rosout_agg`，可按级别过滤/暂停。
  - 消息：任意话题的截断 JSON 预览、暂停、复制。

常用话题目录来自设备实测（56 个），点「扫描全部」才会调用 `/rosapi/topics` —— 这个服务在这台设备上
要 8 s 以上，期间 rosbridge 会卡住，所以不做自动刷新。

## 坐标与渲染约定

- Three.js 场景直接用 ROS 约定：**Z 轴向上**，`map` 系为世界原点。
- 机器人位姿优先 `tf: map→base_link`，回退到 `/slam/info.robot_pose`。
- 点云按消息 `header.frame_id` 查 TF 后放到 map 系；`/scan_f_map` 本身就是 `map` 系。
- 栅格地图用 `map.info.origin` 摆放，`z=0.01` 铺在地面上。

## 目录

```
hmi/
├── src/
│   ├── ros/
│   │   ├── client.ts    # rosbridge 客户端（JSON + CBOR 二进制、订阅/发布/服务、重连）
│   │   ├── cbor.ts      # 最小 CBOR 解码（含 typed-array tag）
│   │   ├── decode.ts    # PointCloud2 / PointCloud / OccupancyGrid / LaserScan / Path / Polygon
│   │   ├── tf.ts        # TF 树 + 任意两帧 lookup（BFS）
│   │   └── app.ts       # 全局状态、订阅编排、快照、服务封装
│   ├── three/
│   │   ├── SceneView.vue# 三维视图、交互、动画循环
│   │   ├── layers.ts    # 点云/栅格/机器人/TF/路径/轨迹图层
│   │   └── gridImage.ts # OccupancyGrid → RGBA 纹理（分块取最大值）
│   ├── components/      # ControlPanel / InspectorPanel
│   └── App.vue, main.ts
├── tests/               # 解码与 TF 单测（含真实 cbor 帧 fixture）
└── dev.sh
```

## 已知限制

- 只支持 ROS1 rosbridge；换 ROS2 时替换 `ros/client.ts` 的传输层即可。
- rosbridge `cbor-raw` 在这台设备的 0.11.16 上不可用（`OutboundMessage._buff` 未填充），所以走结构化 `cbor`。
- `/map` 快照期间实时数据会暂停（rosbridge 单线程），这是设备端序列化的限制。
- `rosbridge` 的 API 文档/场景图等没做，当前只覆盖调试所需。
