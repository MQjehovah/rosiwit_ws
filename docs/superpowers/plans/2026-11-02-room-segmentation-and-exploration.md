# 房间分割 + 自动探索联合实施方案

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 2D 栅格地图上稳定切出"可清洁区域"（房间/走廊），并把区域信息接进自动探索与覆盖规划：探索时按区域推进、覆盖时按房间排序并可按时段/房间续扫。

**Architecture:** 分割算法落在 **ROS 无关**的 `src/rosiwit_navigation`（nav core），输入是 `OccupancyGrid` 的只读视图，输出"每格房间标签 + 每个房间的多边形/面积/门/邻接"。`rosiwit_navigation_ros2` 加一个薄节点把这个结果发布成 `/room_labels`（int8 栅格）、markers 和 `<map>.rooms.json`。覆盖侧复用已有的 `CoverageGrid`（它本来就是"给一个多边形，铺 Boustrophedon"），按房间顺序跑。探索侧改造 `frontier_explorer.py` 的打分与结束判据，并在有 `/room_labels` 时按区域优先。

**Tech Stack:** C++17 + Eigen + OpenCV（形态学/分水岭/轮廓，nav core 首次引入 OpenCV）；Python 3（`rclpy` + numpy，探索节点与评测）；ROS2 Humble；gtest + pytest。

**Spec:** 本文件（无独立 spec 文档；需求来自 "清洁机器人需要房间/区域分割 + 联合自动探索"）。

## Global Constraints

- 分割算法层（`src/rosiwit_navigation`）**不得依赖 ROS**；只允许 Eigen / 标准库 / OpenCV。
- `src/rosiwit_navigation/test/test_*.cpp` 会被 CMake 自动 glob 进 ctest；新增测试文件名必须是 `test_room_segmentation.cpp` 这类 `test_*.cpp`。
- 栅格分辨率按 0.05 m 设计（与 `mapping.grid.resolution` 一致），但必须按 `grid.resolution` 读取、不得写死。
- 房间标签用 `nav_msgs/OccupancyGrid`（int8）：`-1` = 未知/墙/不可通行，`0..126` = 房间 id，`127` = 走廊/通道。
- 参数一律从 yaml 读，不得写死；新增参数必须有默认值且与本文给出的默认值一致。
- 注释与文档用中文，标识符用英文；每个 task 单独一次 commit。
- 探索节点的现有行为必须保持可回退：新增打分项都由参数开关（默认关 → 打开），旧的纯距离打分仍是 `frontier_cost_mode: distance`。

## Review Focus

- **未知区域未走完时分割**（无人值守建图/探索中途）：未知格不能被当成墙导致房间被切碎；标签里保留 unknown，面积与覆盖率只按已知自由格统计。
- **开放式空间（客厅+餐厅无门）**：可能出现过度切分；必须能用"面积/细实比"规则合并，且允许 App 手动合并（本方案只保证输出可合并的 id 图）。
- **门被家具/半开门堵住**：门宽参数（`door_width_max`）决定切断阈值；同一站点门宽差异大时要能自适应（取窄通道宽度分布的低谷）。
- **两次分割之间房间 id 跳变**：下游（覆盖记录/断点续扫）依赖 id 稳定，必须有 id 匹配与缓存。
- **窄门通行性**：房间多边形必须按机器人 footprint 内缩，否则"路径规划出来了但走不进去"（当前 `CoverageGrid` 已有 `obstacle_dilation`，需确认对门口同样生效）。

---

### Task 1: 房间分割骨架 + 门洞切断

**Files:**
- Create: `src/rosiwit_navigation/include/rosiwit_navigation/nav_core/room_segmentation.hpp`
- Create: `src/rosiwit_navigation/src/nav_core/room_segmentation.cpp`
- Create: `src/rosiwit_navigation/test/test_room_segmentation.cpp`
- Modify: `src/rosiwit_navigation/CMakeLists.txt`（`find_package(OpenCV REQUIRED)` + 链接 `${OpenCV_LIBS}`）

**Interfaces:**
- Consumes: 无（本 task 定义基础类型）
- Produces:
  - `struct GridView { const int8_t* data; unsigned width, height; double resolution, origin_x, origin_y; }`
  - `struct Point2D { double x, y; }`
  - `struct RoomParams { int free_thresh = 50; double door_width_max = 1.10; double min_area_m2 = 2.0; double corridor_max_solidity = 0.15; double corridor_min_length_m = 6.0; int min_region_cells = 200; }`
  - `struct Room { int id; int64_t area_cells; double area_m2; double centroid_x, centroid_y; std::vector<Point2D> polygon; std::vector<int> neighbor_ids; bool is_corridor; std::vector<Point2D> doors; }`
  - `struct RoomSegmentationResult { bool ok; std::vector<int16_t> labels; std::vector<Room> rooms; double elapsed_ms; }`
  - `std::vector<std::vector<int>> connectedRegions(const GridView& g, const RoomParams& p)` — 返回种子连通域（每项是格索引的升序下标列表）

- [ ] **Step 1: 写失败测试**

```cpp
// test/test_room_segmentation.cpp
#include <gtest/gtest.h>
#include "rosiwit_navigation/nav_core/room_segmentation.hpp"

namespace core = rosiwit_navigation::core;

namespace {
// 20 m x 10 m 空间, 中点一道墙(墙厚 0.2 m), 墙上留 door_w 宽的门
std::vector<int8_t> twoRoomGrid(double door_w, double res = 0.05);
}

TEST(RoomSegmentation, DoorWiderThanThresholdSplitsRooms) {
  auto cells = twoRoomGrid(0.9);
  core::GridView g{cells.data(), 400, 200, 0.05, 0.0, 0.0};
  core::RoomParams p;
  p.door_width_max = 1.10;
  auto regions = core::connectedRegions(g, p);
  EXPECT_EQ(regions.size(), 2u);
}

TEST(RoomSegmentation, DoorNarrowerThanThresholdKeepsOneRegion) {
  auto cells = twoRoomGrid(0.9);
  core::GridView g{cells.data(), 400, 200, 0.05, 0.0, 0.0};
  core::RoomParams p;
  p.door_width_max = 0.50;   // 比门窄 -> 不切断
  auto regions = core::connectedRegions(g, p);
  EXPECT_EQ(regions.size(), 1u);
}
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd src/rosiwit_navigation && cmake -S . -B build && cmake --build build --target test_room_segmentation`
Expected: 编译失败（`room_segmentation.hpp` 不存在）

- [ ] **Step 3: 实现**

`connectedRegions()` 的算法（用 OpenCV）：
1. `free = (data >= 0 && data < free_thresh)` → `CV_8UC1`。
2. `occupied = (data >= occupied_thresh || data < 0 ? 不，未知不算障碍)` → 只把 `data >= occupied_thresh` 当障碍；未知格按"不可通行"处理但**不参与膨胀**（避免把未知边界当墙切房间）。
3. `dilate(occupied, k, iterations=1)`，`k = 2*floor(door_width_max/2/resolution)+1`（奇数方核）。
4. `blocked = occupied_dilated | unknown`；`seeds = free & !blocked`。
5. `connectedComponentsWithStats(seeds)` → 丢弃 `area < min_region_cells` 的连通域。
6. 返回每个连通域的格下标（`y*width+x`）。

- [ ] **Step 4: 跑测试确认通过**

Run: `ctest --test-dir build -R test_room_segmentation --output-on-failure`
Expected: 2 tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/rosiwit_navigation/include/rosiwit_navigation/nav_core/room_segmentation.hpp \
        src/rosiwit_navigation/src/nav_core/room_segmentation.cpp \
        src/rosiwit_navigation/test/test_room_segmentation.cpp \
        src/rosiwit_navigation/CMakeLists.txt
git commit -m "feat(nav-core): room segmentation skeleton + door-width seed splitting"
```

### Task 2: 距离变换 + 标记分水岭（切开无门的连续大空间）

**Files:**
- Modify: `src/rosiwit_navigation/src/nav_core/room_segmentation.cpp`
- Modify: `src/rosiwit_navigation/test/test_room_segmentation.cpp`

**Interfaces:**
- Consumes: Task 1 的类型与 `connectedRegions()`
- Produces: `RoomSegmentationResult segmentRooms(const GridView& g, const RoomParams& p)`；`labels` 长度 = `width*height`，`-1` 表示非房间格

- [ ] **Step 1: 写失败测试**

```cpp
TEST(RoomSegmentation, WatershedSplitsDumbbellAtNeck) {
  // 两个 6x6 m 房间, 中间 1.5 m 宽、2 m 长通道 (宽于 door_width_max, 不会被堵)
  auto cells = dumbbellGrid(6.0, 1.5, 2.0);
  core::GridView g{cells.data(), 320, 160, 0.05, 0.0, 0.0};
  core::RoomParams p;
  p.door_width_max = 1.10;
  auto r = core::segmentRooms(g, p);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.rooms.size(), 2u);
  // 面积守恒: 房间总面积 = 已知自由格面积 (误差 1%)
  EXPECT_NEAR(totalRoomArea(r), knownFreeArea(g), 0.01 * knownFreeArea(g));
}
```

- [ ] **Step 2: 跑测试确认失败**

Run: `ctest --test-dir build -R test_room_segmentation`
Expected: FAIL — `segmentRooms` 未定义

- [ ] **Step 3: 实现**

1. `dist = cv::distanceTransform(free_without_obstacles, DIST_L2, DIST_MASK_PRECISE)`。
2. 以 Task 1 的种子连通域为 marker（每个种子一个 label id），在 `-dist` 上做 `cv::watershed`。
3. 把 watershed 结果映射回 `labels`：`marker>0` 的格 = 房间格，其余 `-1`；走廊/通道由 Task 3 判定。
4. 每个 label 汇总 `area_cells`、`area_m2 = area_cells*res^2`、`centroid`。
5. `elapsed_ms` 用 `std::chrono::steady_clock` 记录。

- [ ] **Step 4: 跑测试确认通过**

Run: `ctest --test-dir build -R test_room_segmentation --output-on-failure`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(nav-core): watershed split of continuous free space into rooms"
```

### Task 3: 后处理（碎片合并 / 走廊判定 / 最小区块）

**Files:**
- Modify: `src/rosiwit_navigation/src/nav_core/room_segmentation.cpp`
- Modify: `src/rosiwit_navigation/test/test_room_segmentation.cpp`

**Interfaces:**
- Produces: `segmentRooms()` 输出的 `rooms[i].is_corridor`；`labels` 中走廊格值为 `127`

- [ ] **Step 1: 写失败测试**

```cpp
TEST(RoomSegmentation, TinyRegionMergedIntoNeighbor) {
  auto cells = roomWithNookGrid();          // 6x6 m 房间 + 0.6 m x 0.6 m 凹角
  core::GridView g{cells.data(), 160, 160, 0.05, 0.0, 0.0};
  core::RoomParams p; p.min_area_m2 = 2.0;
  auto r = core::segmentRooms(g, p);
  EXPECT_EQ(r.rooms.size(), 1u);            // 凹角被并入
}

TEST(RoomSegmentation, LongNarrowRegionFlaggedAsCorridor) {
  auto cells = corridorGrid(10.0, 1.0);     // 10 m x 1 m
  core::GridView g{cells.data(), 240, 60, 0.05, 0.0, 0.0};
  core::RoomParams p;
  auto r = core::segmentRooms(g, p);
  ASSERT_EQ(r.rooms.size(), 1u);
  EXPECT_TRUE(r.rooms[0].is_corridor);
}
```

- [ ] **Step 2: 跑测试确认失败** — Expected: FAIL（`rooms.size()==2` / `is_corridor==false`）

- [ ] **Step 3: 实现**

- 面积 `< min_area_m2` 的房间：找相邻房间（4-邻域标签）合并到共享边界最长者。
- 走廊判定（两个条件都满足）：`area_m2 / (max_x_span*max_y_span) < corridor_max_solidity` **且** `max(span_x, span_y) > corridor_min_length_m`。
- 走廊房间从 `rooms` 移除，其格在 `labels` 中置 `127`。
- 移除后重排 id（`0..N-1` 连续）。

- [ ] **Step 4: 跑测试确认通过** — Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(nav-core): room post-processing (merge tiny regions, flag corridors)"
```

### Task 4: 多边形 / 门 / 邻接 / JSON 持久化

**Files:**
- Modify: `src/rosiwit_navigation/src/nav_core/room_segmentation.cpp`
- Modify: `src/rosiwit_navigation/include/rosiwit_navigation/nav_core/room_segmentation.hpp`
- Modify: `src/rosiwit_navigation/test/test_room_segmentation.cpp`

**Interfaces:**
- Produces:
  - `bool saveRoomsJson(const std::string& path, const RoomSegmentationResult& r, const std::string& map_id)`（不依赖 yaml-cpp：手写 JSON）
  - `bool loadRoomsJson(const std::string& path, RoomSegmentationResult& r)`

- [ ] **Step 1: 写失败测试**

```cpp
TEST(RoomSegmentation, PolygonAreaAndDoors) {
  auto cells = twoRoomGrid(0.9);
  core::GridView g{cells.data(), 400, 200, 0.05, 0.0, 0.0};
  core::RoomParams p;
  auto r = core::segmentRooms(g, p);
  ASSERT_EQ(r.rooms.size(), 2u);
  for (const auto& room : r.rooms) {
    EXPECT_NEAR(polygonArea(room.polygon), room.area_m2, 0.05 * room.area_m2);
  }
  EXPECT_EQ(r.rooms[0].neighbor_ids.size(), 1u);
  EXPECT_EQ(r.rooms[0].doors.size(), 1u);
  EXPECT_NEAR(r.rooms[0].doors[0].x, 10.0, 0.3);   // 门在墙中点
}

TEST(RoomSegmentation, JsonRoundTrip) {
  auto r = core::segmentRooms(makeTwoRoomGrid(0.9), {});
  ASSERT_TRUE(core::saveRoomsJson("/tmp/rooms_test.json", r, "siteA"));
  core::RoomSegmentationResult back;
  ASSERT_TRUE(core::loadRoomsJson("/tmp/rooms_test.json", back));
  ASSERT_EQ(back.rooms.size(), r.rooms.size());
  EXPECT_EQ(back.rooms[0].id, 0);
  EXPECT_NEAR(back.rooms[0].area_m2, r.rooms[0].area_m2, 1e-6);
  EXPECT_EQ(back.rooms[0].doors.size(), r.rooms[0].doors.size());
}
```

- [ ] **Step 2: 跑测试确认失败** — Expected: FAIL（无 `polygonArea` / JSON 函数）

- [ ] **Step 3: 实现**

- 多边形：`cv::findContours` + `cv::approxPolyDP(eps = 1.5*resolution)`，转到世界坐标 `origin + (c+0.5)*res`。
- 门：两房间标签在 4-邻域相接的格对集合，取"最窄连通段"：对每个相接段求两端点距离，`< door_width_max*1.2` 且是该对房间最短段 → 门中心 = 段中点。
- `neighbor_ids`：由相接格对统计（去重、排序）。
- JSON schema（`rooms_json v1`）：
  ```json
  {"version":1,"map_id":"siteA","resolution":0.05,"grid":[400,200],
   "rooms":[{"id":0,"area_m2":90.1,"centroid":[5.0,5.0],
             "polygon":[[0.0,0.0],[10.0,0.0]],
             "doors":[[10.0,5.0]],"neighbors":[1],"is_corridor":false}]}
  ```

- [ ] **Step 4: 跑测试确认通过** — Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(nav-core): room polygon/door/neighbour extraction + rooms.json"
```

### Task 5: 稳态化（两次分割 id 不跳变）

**Files:**
- Modify: `src/rosiwit_navigation/include/rosiwit_navigation/nav_core/room_segmentation.hpp`
- Modify: `src/rosiwit_navigation/src/nav_core/room_segmentation.cpp`
- Modify: `src/rosiwit_navigation/test/test_room_segmentation.cpp`

**Interfaces:**
- Produces: `void matchRoomIds(const RoomSegmentationResult& prev, RoomSegmentationResult& cur)` — 按"质心距离 < 0.5 m 且面积差 < 30%"匹配，保留 prev 的 id

- [ ] **Step 1: 写失败测试**

```cpp
TEST(RoomSegmentation, IdsStableAcrossMinorMapGrowth) {
  auto r1 = core::segmentRooms(makeTwoRoomGrid(0.9), {});
  auto cells2 = makeTwoRoomGrid(0.9);
  addFreeCells(cells2, /*right room 右下 0.5 m^2*/);   // 模拟地图增长
  auto r2 = core::segmentRooms(...);
  core::matchRoomIds(r1, r2);
  EXPECT_EQ(r2.rooms[0].id, r1.rooms[0].id);
  EXPECT_EQ(r2.rooms[1].id, r1.rooms[1].id);
}
```

- [ ] **Step 2: 跑测试确认失败** — Expected: FAIL（`matchRoomIds` 未定义）

- [ ] **Step 3: 实现** — 贪心匹配：对 `cur` 每个房间找 `prev` 中质心最近且过阈值的未匹配房间，继承 id；未匹配的分配 `max_id+1`；最后重排保证 `0..N-1` 连续且 `depth` 一致。

- [ ] **Step 4: 跑测试确认通过** — Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(nav-core): stable room ids across successive segmentations"
```

### Task 6: ROS2 节点 `room_segmentation`

**Files:**
- Create: `ros2_ws/src/rosiwit_navigation_ros2/src/room_segmentation_node.cpp`
- Create: `ros2_ws/src/rosiwit_navigation_ros2/launch/rooms.launch.py`
- Create: `ros2_ws/src/rosiwit_navigation_ros2/config/rooms.yaml`
- Modify: `ros2_ws/src/rosiwit_navigation_ros2/CMakeLists.txt`（新 target + install）
- Test: `ros2_ws/src/rosiwit_navigation_ros2/test/test_room_node_smoke.py`

**Interfaces:**
- Consumes: Task 2/4/5 的 core API
- Produces（ROS 接口）:
  - 订阅 `/map`（`transient_local`）
  - 发布 `/room_labels`（`nav_msgs/OccupancyGrid`，int8：-1/0..126/127）、`/room_markers`（`MarkerArray`）
  - 服务 `~/segment`（`std_srvs/Trigger`，立即重算）、`~/save_rooms`（`std_srvs/Trigger`，写 `<save_dir>/<map_id>.rooms.json`）
  - 参数：`map_topic`、`room_labels_topic`、`door_width_max`、`min_area_m2`、`corridor_min_length_m`、`period`（默认 5.0 s）、`save_dir`

- [ ] **Step 1: 写失败测试（集成，launch 起节点）**

```python
# test/test_room_node_smoke.py
def test_publishes_room_labels(tmp_bag_grid_pub):
    # 测试夹具把 twoRoomGrid 以 OccupancyGrid 发到 /map
    msg = wait_for_topic('/room_labels', timeout=10)
    assert msg.info.width == 400 and msg.info.height == 200
    assert sorted(set(msg.data)) [:2] == [-1, 0]
    assert max(msg.data) >= 1        # 至少两个房间
```

- [ ] **Step 2: 跑测试确认失败** — Run: `colcon test --packages-select rosiwit_navigation_ros2 --ctest-args -R room_node_smoke` → FAIL（executable 不存在）

- [ ] **Step 3: 实现** — 节点在 `period` 到达或服务调用时：取 `/map` → `segmentRooms()` → `matchRoomIds(prev)` → 发布 label 栅格（`int8`，`info` 直接拷贝 `/map` 的）+ markers（每个房间一个 `TEXT_VIEW_FACING` 显示 id/面积，门用红点）。

- [ ] **Step 4: 跑测试确认通过** — Run 同上 → PASS

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(nav-ros2): room_segmentation node (labels, markers, save_rooms)"
```

### Task 7: 覆盖规划按房间执行 + 房间顺序

**Files:**
- Modify: `ros2_ws/src/rosiwit_coverage_planner_ros2/src/coverage_planner_node.cpp`
- Modify: `ros2_ws/src/rosiwit_coverage_planner_ros2/include/coverage_planner/coverage_planner.hpp`
- Modify: `ros2_ws/src/rosiwit_coverage_planner_ros2/config/*.yaml`
- Test: `ros2_ws/src/rosiwit_coverage_planner_ros2/test/test_room_order.py`

**Interfaces:**
- Consumes: `/room_labels`（Task 6）、`rooms.json`（Task 4）
- Produces: `std::vector<int> orderRooms(const std::vector<Room>& rooms, Point2D start)`（最近邻 + 2-opt）；服务 `~/plan_rooms`（`std_srvs/Trigger`，按房间顺序规划并拼接 `/coverage_path`）

- [ ] **Step 1: 写失败测试**

```python
def test_room_order_visits_each_room_once(three_room_scene):
    order = plan_rooms(start=(0, 0))
    assert sorted(order) == [0, 1, 2]
    assert path_length(order) <= 1.10 * nearest_neighbour_length(three_room_scene, (0, 0))
```

- [ ] **Step 2: 跑测试确认失败** — Expected: FAIL（服务不存在）

- [ ] **Step 3: 实现**

- 参数 `use_rooms: true`、`room_inset: 0.15`（多边形按 footprint 内缩，保证能进去）。
- 每个房间跑一次现有 zigzag/spiral 规划（`CoverageGrid::initialize` 或节点自带 planner），输出子路径。
- 房间间用 `orderRooms()` 排序，子路径首尾用现有 A*/hybrid A* 连接。

- [ ] **Step 4: 跑测试确认通过** — Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(coverage): per-room coverage with nearest-neighbour+2-opt room order"
```

### Task 8: 探索打分改成"真实路径代价"

**Files:**
- Modify: `ros2_ws/src/rosiwit_exploration_ros2/rosiwit_exploration_ros2/frontier_explorer.py`
- Modify: `ros2_ws/src/rosiwit_exploration_ros2/config/exploration_params.yaml`
- Test: `ros2_ws/src/rosiwit_exploration_ros2/test/test_frontier_logic.py`

**Interfaces:**
- Produces: `frontier_cost_mode: distance|path`（默认 `distance`，新行为需显式打开）；`path_lengths(nav_reachable, start) -> np.ndarray`（每格从起点出发的可通行距离，`inf` = 不可达）

- [ ] **Step 1: 写失败测试**

```python
def test_path_cost_prefers_shorter_detour():
    # U 形障碍: A 直线 2 m 但要走 8 m; B 直线 5 m 且直达 5 m
    cands = best_frontier_with(cost_mode='path')
    assert cands[0].x == pytest.approx(B.x)

def test_distance_mode_keeps_old_behaviour():
    cands = best_frontier_with(cost_mode='distance')
    assert cands[0].x == pytest.approx(A.x)
```

- [ ] **Step 2: 跑测试确认失败** — Run: `cd ros2_ws/src/rosiwit_exploration_ros2 && python -m pytest test/test_frontier_logic.py -k path_cost -v` → FAIL

- [ ] **Step 3: 实现** — 8-邻域 Dijkstra（堆），代价 `1.0 / 1.414`；`best_frontier()` 用它替换 `math.hypot` 的 `dist`，并把 `max_goal_distance`/`min_goal_distance` 也按路径长度判断（新增 `distance_mode` 参数，默认 `euclid` 保持兼容）。

- [ ] **Step 4: 跑测试确认通过** — Expected: PASS（两个用例都过）

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(exploration): optional path-length frontier cost (Dijkstra on costmap)"
```

### Task 9: 信息增益（视野内未知格计数）

**Files:**
- Modify: `ros2_ws/src/rosiwit_exploration_ros2/rosiwit_exploration_ros2/frontier_explorer.py`
- Modify: `ros2_ws/src/rosiwit_exploration_ros2/config/exploration_params.yaml`
- Test: `ros2_ws/src/rosiwit_exploration_ros2/test/test_frontier_logic.py`

**Interfaces:**
- Produces: `information_gain(grid, unknown, row, col, yaw, max_range_m, rays) -> int`；新参数 `gain_rays: 24`、`gain_range: 4.0`、`gain_weight: 0.5`、`enable_gain: true`

- [ ] **Step 1: 写失败测试**

```python
def test_gain_prefers_open_side():
    # 两个等距前沿: 一个朝开放未知区, 一个朝死胡同
    cands = best_frontier_with(enable_gain=True)
    assert cands[0].size_hint == 'open'
    assert gain_of('open') > 2 * gain_of('dead_end')

def test_gain_disabled_keeps_distance_order():
    cands = best_frontier_with(enable_gain=False)
    assert cands[0].size_hint == 'dead_end'   # 距离相同 -> 索引小者
```

- [ ] **Step 2: 跑测试确认失败** → FAIL（无 `information_gain`）

- [ ] **Step 3: 实现** — 从候选格沿 `rays` 个方向（覆盖 ±60° 的扇区，朝向取"从机器人到候选"的方向）步进 `resolution`，遇 occupied/已知格停止；对每个首次命中的未知格计数（用 set 去重）。`score = w_path*path_len − gain_weight*gain`（`mode=path`），或 `dist − gain_weight*gain`（`mode=distance`）。

- [ ] **Step 4: 跑测试确认通过** → PASS

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(exploration): view-cone information gain in frontier scoring"
```

### Task 10: 结束判据（剩余未知面积）

**Files:**
- Modify: `ros2_ws/src/rosiwit_exploration_ros2/rosiwit_exploration_ros2/frontier_explorer.py`
- Modify: `ros2_ws/src/rosiwit_exploration_ros2/config/exploration_params.yaml`
- Test: `ros2_ws/src/rosiwit_exploration_ros2/test/test_frontier_logic.py`

**Interfaces:**
- Produces: `finish_unknown_ratio: 0.02`；`finish()` 输出原因字符串（`reason=no_frontier|unknown_ratio|user_stop`），并通过 `~/status` 服务返回

- [ ] **Step 1: 写失败测试**

```python
def test_finish_when_unknown_below_ratio():
    st = run_tick_with(unknown_ratio=0.01, patience=1)
    assert st.finished and 'unknown_ratio' in st.reason

def test_not_finish_when_unknown_large():
    st = run_tick_with(unknown_ratio=0.20, patience=1)
    assert not st.finished
```

- [ ] **Step 2: 跑测试确认失败** → FAIL

- [ ] **Step 3: 实现** — 每 tick 计算 `unknown_ratio = unknown_cells / (free_cells + unknown_cells)`（只统计地图凸包内，避免把地图外的 -1 算进去）；`unknown_ratio <= finish_unknown_ratio && empty_cycles >= min(finish_patience, 3)` → `finish('unknown_ratio')`。

- [ ] **Step 4: 跑测试确认通过** → PASS

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(exploration): finish on remaining unknown ratio with reason code"
```

### Task 11: 按区域推进（消费 `/room_labels`）

**Files:**
- Modify: `ros2_ws/src/rosiwit_exploration_ros2/rosiwit_exploration_ros2/frontier_explorer.py`
- Modify: `ros2_ws/src/rosiwit_exploration_ros2/config/exploration_params.yaml`
- Test: `ros2_ws/src/rosiwit_exploration_ros2/test/test_frontier_logic.py`

**Interfaces:**
- Produces: 订阅 `/room_labels`（可选）；参数 `region_pref: true`、`region_switch_margin: 3.0`（m，切换到别的区域需要比本区域最优目标少走 3 m）

- [ ] **Step 1: 写失败测试**

```python
def test_prefers_frontier_in_current_region():
    cands = best_frontier_with(region_pref=True, robot_in_region=0)
    assert cands[0].region == 0            # 另一区域目标即使近 2 m 也不选

def test_switches_when_other_region_much_better():
    cands = best_frontier_with(region_pref=True, robot_in_region=0, other_region_delta=5.0)
    assert cands[0].region == 1
```

- [ ] **Step 2: 跑测试确认失败** → FAIL

- [ ] **Step 3: 实现** — 候选的 `region = room_labels[row, col]`（`127`/`-1` 视为"走廊/未知"= 不属于任何区域）；打分加 `region_penalty = 0 if same else region_switch_margin`。

- [ ] **Step 4: 跑测试确认通过** → PASS

- [ ] **Step 5: Commit**

```bash
git commit -am "feat(exploration): region-aware frontier preference via /room_labels"
```

### Task 12: 探索评测脚本（覆盖率–里程–时间基线）

**Files:**
- Create: `scripts/sim/explore_eval.py`
- Create: `docs/artifacts/explore_baseline.json`（首批基线数据）

**Interfaces:**
- Produces: `python3 scripts/sim/explore_eval.py --scenario <world> --runs 5 --out <json>`
  输出字段：`coverage_pct`（结束时已知自由面积/场景可通行面积）、`travel_m`、`duration_s`、`finish_reason`、`failed_goals`、`room_seg_ms`

- [ ] **Step 1: 写失败测试（脚本自测，不需要仿真）**

```python
# scripts/sim/test_explore_eval.py
def test_metrics_from_trace(tmp_path):
    trace = make_synthetic_trace(coverage=0.93, travel=81.0, dur=300.0)
    m = summarize(trace)
    assert m['coverage_pct'] == pytest.approx(93.0)
    assert m['travel_m'] == pytest.approx(81.0)
```

- [ ] **Step 2: 跑测试确认失败** → FAIL

- [ ] **Step 3: 实现** — 读 `/map` 快照序列 + `/lio_odom`（或 `/odom`）→ 计算覆盖率曲线、里程、时间、结束原因；`--scenario` 只负责启动/停止仿真与 rosbag 录制；结果写 JSON 并打印表格。

- [ ] **Step 4: 跑测试确认通过** → PASS

- [ ] **Step 5: 在仿真里跑基线**（`distance` 模式 + `enable_gain: false` + `region_pref: false`），5 次取中位，存 `docs/artifacts/explore_baseline.json`。

- [ ] **Step 6: Commit**

```bash
git commit -am "test(exploration): explore_eval harness + baseline metrics"
```

### Task 13: 验收跑 + 文档

**Files:**
- Modify: `README.md`（TODO 增加"房间分割 + 区域化探索/覆盖"一项，含验收数字）
- Create: `docs/dev/room-segmentation-and-exploration.md`（结果表 + 参数说明 + 失败模式）
- Modify: `ros2_ws/src/rosiwit_exploration_ros2/README.md`、`ros2_ws/src/rosiwit_coverage_planner_ros2/README.md`（新参数与接口）

- [ ] **Step 1: 验收跑（同一场景，3 组配置）**

| 配置 | 期望 |
|---|---|
| 基线（distance / 无 gain / 无 region） | 记录在 `explore_baseline.json` |
| + path cost | 里程 −15%、时间 −15% |
| + gain + region_pref | 里程 −25%、覆盖率 +5%、跨区域往返 −50% |

Run: `python3 scripts/sim/explore_eval.py --scenario <world> --runs 5 --compare baseline,path,path_gain_region`

- [ ] **Step 2: 房间分割验收**（3–5 张真实站点地图）

Run: `ros2 run rosiwit_navigation_ros2 room_segmentation_node --ros-args -p map_file:=<pgm>`（离线模式）
判定：IoU > 0.8、房间数误差 ≤ 1、门召回 > 90%、耗时 < 300 ms（100×100 m / 0.05 m）。

- [ ] **Step 3: 写文档** — 结果表 + 参数含义 + 已知失败模式（开放空间过度切分、家具假门、未知区边界）。

- [ ] **Step 4: 最终验证** — `ctest --test-dir src/rosiwit_navigation/build`（全绿）、`colcon test --packages-select rosiwit_navigation_ros2 rosiwit_coverage_planner_ros2 rosiwit_exploration_ros2`、`python -m pytest`。

- [ ] **Step 5: Commit**

```bash
git commit -am "docs: room segmentation + region-aware exploration results and TODO entry"
```
