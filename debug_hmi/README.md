# debug_hmi

调试 rosiwit SLAM 机器人的浏览器前端：状态面板 + Three.js 三维视图 + RViz 风格动态话题显示。
Vue 3 + TypeScript + Vite + Three.js。

只有一条数据通道：

```
浏览器 ──ws(s)://…:9100──> debug_bridge（WebSocket + protobuf）
```

TF、定位状态、日志、服务调用以及所有点云 / 栅格 / 激光 / 路径话题都走这里。
厂商的 `http_bridge`（9090）保持不动，本项目不再使用 rosbridge。

## 快速开始

开发机需要 Node ≥ 18。Windows 侧如果通过 `\\wsl.localhost\...` 打开本仓库，不要在 UNC 路径下直接跑 npm
（9P 文件系统会让 esbuild 崩掉），统一用 WSL：

```bash
cd ~/rosiwit_ws/debug_hmi

# 一次性：把免 root 的 Node 装到 ~/.local/node
curl -fsSL https://npmmirror.com/mirrors/node/v18.20.4/node-v18.20.4-linux-x64.tar.xz -o /tmp/node.tar.xz
mkdir -p ~/.local && tar -xJf /tmp/node.tar.xz -C ~/.local
mv ~/.local/node-v18.20.4-linux-x64 ~/.local/node

./dev.sh install       # npm install
./dev.sh dev           # vite，监听 0.0.0.0:5173
./dev.sh test          # 解码 / TF / PCD 单测（不连设备）
./dev.sh bridge-test   # debug_bridge 端到端测试（demo + WebSocket + protobuf）
./dev.sh build         # vue-tsc + 生产构建
```

浏览器打开 `http://<开发机 IP>:5173/`。

### 地址配置

连接地址在顶栏左侧的胶囊里改，会存到 localStorage。也可以用 `.env.local`（已被 gitignore）：

```bash
cp .env.example .env.local
# VITE_BRIDGE_URL=wss://test.rosiwit.com/remote-terminal/tunnel/<SN>/<TOKEN>/127.0.0.1/9100/
# VITE_TUNNEL_URL=wss://…（同上，会出现在顶栏的「远程隧道」预设里）
```

## 显示模型（RViz 风格）

- **TF 常订阅**（`/tf`、`/tf_static`），只控制坐标轴显示。
- `/slam/info` 由前端「定位状态插件」消费：兼容数字枚举与字符串，统一翻译成可读文本。
- 其余全部是**动态显示项**：从话题列表里选择话题 → 自动识别类型 → 选择渲染方式。
  显示项、颜色、点大小、频率都会存到 localStorage，刷新后保留。

| 消息类型 | 默认渲染 | 可调参数 |
|---|---|---|
| `sensor_msgs/PointCloud2` / `PointCloud` | 点云 | 点大小、固定色/高度着色、体素、点数上限、频率 |
| `nav_msgs/OccupancyGrid` | 栅格 | 频率 / 单帧抓取 |
| `sensor_msgs/LaserScan` | 激光点 | 点大小、颜色、频率 |
| `nav_msgs/Path` | 折线 | 颜色、频率 |
| `geometry_msgs/PolygonStamped` | 闭合线 | 颜色、频率 |
| `sensor_msgs/Image` / `CompressedImage` | 图像 | 频率或手动抓取；点开缩略图看大图 |
| 其它 | JSON（右栏查看） | — |

每个显示项实时显示：**速率 Hz、每帧字节、累计流量、最后更新时间、点数/格数**。
频率最低可选 **「手动」**：服务端只保持订阅和最新缓存，不自动推送，由「抓取」按钮主动要一帧。
`/map`、`/global_map` 这类大话题也建议用「手动」+「抓取」。

图像话题由服务端转成 JPEG（`Image` 支持 rgb8/bgr8/mono8/mono16/32FC1；`CompressedImage` 直接转发 jpeg/png），
前端用缩略图或点击放大显示。

面板底部显示**所有话题合计的总流量和当前总速率**（服务端按实际发送字节统计）。

## 大数据：debug_bridge

机器人侧的 [`../debug_bridge`](../debug_bridge) 是通用话题传输层，不随前端显示逻辑改动：

1. 按话题真实类型订阅（`getTopicTypes` + `roslib.message.get_message_class`）
2. 点云体素降采样（可传 `voxel`，0 = 不降采样）
3. 坐标量化成 int16 厘米
4. zlib 压缩
5. protobuf（手写 wire format，Python 2.7 无 protobuf runtime）后通过 WebSocket 推送

协议见 `../debug_bridge/proto/debug_bridge.proto`，浏览器端用 `protobufjs/light` + 生成的 JSON descriptor。

实测：

- `/map` 4300 万格 → **1.4MB zlib**，机器人生成 6.4s，隧道约 15s
- `/curr_scan_point` 约 4.6k 点 → **17KB zlib/帧**
- `/scan` LaserScan 4000 字节 ranges/帧
- `/camera_01/color/image_raw`（640×480 rgb8）→ 服务端转 JPEG **约 17~26KB/帧**
- `/detected/camera_02/image/compressed` → 直接转发原始 JPEG
- `/global_map`（设备地图 599 万点，5cm 体素）**二十多 MB zlib**

### 全局地图

`/global_map` 就是一个普通点云话题，和 `/curr_scan_point` 一样按显示项订阅 / 手动抓取。
它由 `rosiwit_slam_ros1` 的 `debug_publish` 开关控制，默认关闭：

```bash
# 设备上（xzrobot / xzyz2022!）
sudo sed -i 's/debug_publish: false/debug_publish: true/' \
  /opt/ros/melodic/share/rosiwit_slam_ros1/config/default.yaml
sudo systemctl restart xzrobot
```

⚠️ **会重启整个 xzrobot bringup（SLAM/导航都会中断）**，请在机器人空闲时操作。

当前版本的 SLAM 节点只在“进模式”时尝试发一次 `/global_map`，地图还没装好时会错过。
遇到这种情况，直接在左侧加载本机 PCD 地图即可，不需要走话题。

## 本地 PCD

左侧「加载 PCD 地图」直接读本机文件，支持 `ascii` / `binary` / `binary_compressed`（LZF），
解析后限制在 120 万点并写入 IndexedDB，刷新页面自动恢复。

机器上的 `map.xzmap` 可以用 `../debug_bridge/tools/xzmap_to_pcd.py` 转成标准 PCD：

```bash
python3 xzmap_to_pcd.py ~/Downloads/map.xzmap -o ~/Downloads/map_0.3m.pcd --voxel 0.3 --max 800000
```

## 界面

- **顶栏**：连接胶囊（改地址 / 局域网 / 远程隧道 / 断开）、定位状态、模式、匹配度、地图名、左右栏开关。
- **左栏**
  - **定位状态**：插件解析 `/slam/info`，显示状态文本、匹配度条、地图/面积/TF 帧数、xyz + rpy。
  - **显示项**：添加话题、渲染方式、点大小、颜色、着色、体素、上限、频率、抓取、实时速率/总流量。
  - **初始位姿**、**控制**：折叠，减少干扰。
- **中间三维视图**：拖动旋转、右键平移、滚轮缩放；双击放置初始位姿；`M` 测距、`F` 适配、`T` 顶视；
  右下角工具条（顶视/适配/跟随/测距/截图）。
- **右栏**：话题速率/流量表、日志（过滤/暂停/按来源静音/重复合并）、任意话题 JSON。

## 机器人模型

三维视图默认加载 `public/models/robot.glb`（由 `gt.glb` 优化而来：29MB → 3.5MB，约 30 万三角面 → 12 万）。
模型直接使用 GLB 自身尺寸（根节点已带毫米→米缩放，不再二次缩放），脚底对齐 `z=0`，
并绕 X 轴转 90° 让机身立起来；显示时用 `map→base_footprint` 的 z 作为地面高度，
参考网格也随之抬到该高度，避免悬空。Z 轴保持 0°（再加 180° 会让车头朝后）。

替换模型：把新的 GLB 放到 `public/models/robot.glb` 即可；如果姿态不对，改
`src/three/layers.ts` 里 `loadModel()` 的 `qx` 旋转。

## 坐标与渲染约定

- Three.js 场景直接用 ROS 约定：**Z 轴向上**，`map` 系为世界原点。
- 机器人位姿优先 `tf: map→base_link`，回退到 `/slam/info.robot_pose`。
- 位姿显示完整的 **xyz / rpy（ZYX）**；渲染机器人时用完整四元数。
- 点云按消息 `header.frame_id` 查 TF 后放到 map 系。
- 栅格地图用 `map.info.origin` 摆放，`z=0.01`；降采样按块取最大占据值，墙线保持连续。

## 目录

```
debug_hmi/
├── src/
│   ├── ros/
│   │   ├── bridge.ts    # debug_bridge 客户端（WebSocket + protobuf）
│   │   ├── debugRos.ts  # 内置流（TF/状态/日志）到旧话题 API 的兼容层
│   │   ├── proto/debug_bridge.json # 由 debug_bridge/proto 生成
│   │   ├── decode.ts    # PointCloud2 / PointCloud / OccupancyGrid / LaserScan / Path / Polygon
│   │   ├── mapFile.ts   # 标准 PCD 解析（ascii / binary / binary_compressed）
│   │   ├── mapCache.ts  # PCD IndexedDB 缓存
│   │   ├── gridCache.ts # 栅格快照 IndexedDB 缓存
│   │   ├── tf.ts        # TF 树、lookup、四元数 → rpy
│   │   └── app.ts       # 全局状态、显示项、订阅编排、快照、服务
│   ├── three/           # SceneView / layers / gridImage
│   ├── components/      # ConnectionPill / ControlPanel / InspectorPanel
│   └── App.vue, main.ts
├── scripts/             # gen-proto.mjs / test-ros.sh / test-bridge.sh
├── tests/               # 解码/TF/PCD 单测、bridge 端到端、真机探针
└── dev.sh

debug_bridge/
├── debug_bridge.py      # ROS 侧 WebSocket + protobuf 桥（通用话题传输）
├── debug-bridge.service # systemd 单元（PartOf=xzrobot）
├── proto/               # debug_bridge.proto
├── tools/xzmap_to_pcd.py
└── README.md
```

## 已知限制

- 只支持 ROS1。
- 桥接没有鉴权，默认 `0.0.0.0:9100`；建议只走隧道/内网，必要时 `--host 127.0.0.1`。
- `/global_map` 和普通话题一样，设备不发布时就没有数据；可用本地 PCD 导入作为替代。
- 超大点云（数百万点）解码和渲染会占用较多内存，建议用体素/点数上限控制。
- 位姿的 z/rpy 是 TF 真值；如果 `map→odom` 的 z 有偏移，debug_hmi 会如实显示。
