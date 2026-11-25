# debug_bridge

给浏览器 [debug_hmi](../debug_hmi) 用的 **WebSocket + protobuf** 通用话题传输桥。

机器人侧只负责：按话题真实类型订阅 → 归一化 → 降采样/量化/压缩 → 推送。
显示方式（点大小、颜色、渲染类型）完全由前端决定，后端不为每个 UI 功能加专用流。

## 为什么

| 话题 | 原始 | 问题 |
|---|---|---|
| `/map` | 43 MB / 帧，1 Hz | rosbridge JSON 要 >100MB 文本，实测 2 分钟收不完 |
| `/vanjee_points722` | 1 MB / 帧，10 Hz | 10 MB/s，远程隧道不现实 |
| `/curr_scan_point` | ~4.6k 点 / 帧 | JSON 仍会明显膨胀 |

桥接在本地做三件事：

1. **降采样**：点云按体素（请求里可指定，0 = 不降采样）
2. **量化**：坐标转 int16 厘米（3 个点 = 6 字节）
3. **zlib**：再塞进 protobuf `bytes`

### 设备实测（aarch64 / Ubuntu 18.04 / Melodic）

| 数据 | 原始 | 传输 | 耗时 |
|---|---|---|---|
| `/map`（7453×5782 = 43,093,246 格） | 43 MB | **1.4 MB** | 机器人生成 6.4s；隧道约 15s |
| `/curr_scan_point`（约 4.6k 点） | ~74 KB | **17 KB/帧** | 实时 |
| `/scan`（LaserScan，4000 字节 ranges） | 16 KB | **16 KB/帧** | 实时 |
| `/global_map`（599 万点，5cm 体素） | 73 MB PCD | **二十多 MB** | 单帧，数十秒 |

## 协议

协议定义在 [`proto/debug_bridge.proto`](proto/debug_bridge.proto)，包名 `debug_bridge`。
浏览器端用 `protobufjs/light` + 由该 proto 生成的 `debug_hmi/src/ros/proto/debug_bridge.json`
（`node scripts/gen-proto.mjs`）。

```
服务器 -> 浏览器：Envelope { hello | tf | slam | log | grid | cloud | laser | path | polygon | stat | raw | status | error | ... }
浏览器 -> 服务器：ClientCommand { action, stream, rate_hz, voxel, max_points, refresh, ... }
```

### 内置流（`stream` = 固定名字）

- `tf`：`/tf` + `/tf_static`
- `status`：`/slam/info`（SlamInfo）
- `logs`：`/rosout_agg`

### 任意话题（`stream` = 话题名，例如 `/curr_scan_point`）

`SUBSCRIBE` 时服务端用 `getTopicTypes` 查真实类型，按类型归一化：

| ROS 类型 | 帧 | 说明 |
|---|---|---|
| `sensor_msgs/PointCloud2` / `PointCloud` | `CloudFrame` | 可传体素 / 点数上限；源点云有 `intensity` 时额外带强度通道 |
| `nav_msgs/OccupancyGrid` | `GridFrame` | zlib int8 栅格，带 `stream` 话题名 |
| `sensor_msgs/LaserScan` | `LaserFrame` | float32 ranges，带 `stream` |
| `nav_msgs/Path` | `PathFrame` | 带 `stream` |
| `geometry_msgs/PolygonStamped` | `PolygonFrame` | 带 `stream` |
| `sensor_msgs/Image` | `ImageFrame` | 服务端转 JPEG（rgb8/bgr8/mono8/mono16/32FC1） |
| `sensor_msgs/CompressedImage` | `ImageFrame` | 直接转发 jpeg/png 字节 |
| 其它 | `RawMessage` | `extract_values` → JSON，超过 300KB 截断 |

`CloudFrame.intensity`（field 10）是 uint8，与 `xyz_cm` 同顺序、zlib 压缩，只在源点云含
`intensity` 字段时出现：雷达是反射强度，`/slam/debug/overlay_map` 是持久度 `w`(0~255)。
服务端只传数值，不烤颜色；客户端自己上色。

命令：

- `SUBSCRIBE`：按 `rate_hz` 推送最新一帧（只推新数据，不重复发同一 `stamp`）。
  `manual=true`（`rate_hz=0`）时只建立订阅和最新缓存，**不自动推送**。
- `UNSUBSCRIBE`：停止推送。
- `REQUEST`：一次性抓取。`stream` 传话题名；点云/栅格/激光/路径/图像都能用；
  手动模式就是靠它取帧。`/global_map` 和普通话题一样，只等话题本身，不再读设备文件。
- `GET_TOPICS`：返回 `[name, type]`，前端用来做话题下拉框。
- `CALL_SERVICE`：白名单服务。
- `SET_INITIAL_POSE`：发布 `/initialpose`。

### 统计

服务端按客户端实际发出的帧统计，约 2Hz 推 `TopicStat`：

```
topic, count, bytes（最后一帧）, total_bytes（累计）, rate_hz, age_ms
```

前端直接用它显示速率 / 每帧大小 / 累计流量；把所有话题的 `total_bytes`、`rate_hz` 相加就是全局总流量/总速率。

Python 2.7（Melodic）没有 `google.protobuf` runtime，所以服务端按 protobuf wire format
手工编码；字段编号与 `.proto` 一一对应，浏览器用生成好的 descriptor 解码。

## 部署

```bash
# 1) 放文件（设备能访问 GitHub 时最省事）
mkdir -p ~/debug_bridge
curl -fsSL https://raw.githubusercontent.com/MQjehovah/rosiwit_ws/master/debug_bridge/debug_bridge.py \
  -o ~/debug_bridge/debug_bridge.py

# 2) systemd
sudo cp debug-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now debug-bridge
systemctl status --no-pager debug-bridge

# 3) 看日志
journalctl -u debug-bridge -f
```

单元里的 `PartOf=xzrobot.service` 保证 xzrobot 重启时桥接跟着重启；`Restart=always` 兜底。
只依赖 `rospy` + `tornado`（rosbridge 已经带了），Python 2.7。

远程访问：隧道目标端口用 **9100**：

```
wss://<host>/remote-terminal/tunnel/<SN>/<TOKEN>/127.0.0.1/9100/
```

## 本地联调（不需要 ROS/设备）

```bash
python3 debug_bridge.py --demo --port 9100
```

会生成假 TF / 状态 / 日志 / 点云 / 栅格 / 激光 / 路径 / 多边形，协议与真实模式完全一致。
`debug_hmi/scripts/test-bridge.sh` 就是用这个模式做端到端测试。

## xzmap → PCD

```bash
python3 tools/xzmap_to_pcd.py map.xzmap -o map.pcd --voxel 0.3 --max 800000
```

按关键帧位姿把机体系点云变换到世界系，体素降采样，写标准 binary PCD
（`FIELDS x y z`，float32），RViz / PCL / debug_hmi 都能直接读。

## 限制

- 单帧点云进入缓存前最多保留 **100 万点**（`DEBUG_BRIDGE_MAX_CLOUD_POINTS` 可调，0 = 不限）。`/global_map` 这类 600 万点话题全量 Python 对象会吃掉 1.5GB+ 内存，HMI 显示上限通常只有 30 万，100 万已经足够。
- 没有客户端再订阅的普通话题会自动 `unregister` ROS 订阅并释放缓存；避免点过一次 `/global_map` 后永久占用内存和 CPU。
- 订阅是推送（服务器按 `rate_hz` 发），不是轮询；慢客户端由 TCP 背压自然降频。
- 栅格按话题订阅时每个新消息都会编码；`/map` 很大，建议用 `REQUEST` 单帧。
- `/global_map` 只是普通点云话题；设备不发布时没有数据，可在前端导入本地 PCD。
- 没有鉴权，默认绑定 `0.0.0.0:9100`。建议只走隧道/内网，必要时 `--host 127.0.0.1`。
- 服务端编码是手写 protobuf wire format；改 `.proto` 时记得同步
  `debug_bridge.py` 的字段编号，并重新生成 `debug_bridge.json`。
