# rosiwit_common — 通用并发基础层

> ROS 无关、零第三方依赖。**业务层不创建线程、不写 mutex/condition_variable。**

## 组件

| 文件 | 作用 | 关键语义 |
|---|---|---|
| `channel.hpp` | 有界 MPMC 队列 | 满时 `kBlock/kDropOldest/kDropNewest`；`close()` 唤醒所有等待者；带计数 |
| `cancel.hpp` | 协作式取消 | `CancelToken` 只置位不杀线程；长循环里 `token.cancelled()` |
| `thread.hpp` | RAII 线程 | **析构 = requestStop + join（永不 detach）**；OS 线程名 |
| `periodic.hpp` | 定周期线程 | `sleep_until(next += period)` 无漂移；落后超一周期则重新对齐 |
| `executor.hpp` | 执行器接口 | `submit(lane, opts, job, done)` / `cancel` / `shutdown(drain)` / `stats` |
| `thread_pool.hpp` | lane 化线程池 | `threads=1` 即**串行执行器**（写盘/改地图）；满时按策略；支持优先级 |
| `snapshot.hpp` | 不可变快照槽 | `shared_ptr<const T>` + atomic 发布/读取，`version` 单调 |

## 使用约束（架构约定）

1. **业务层不持有线程**：只声明 lane 与任务；线程/池由组装根创建并注入。
2. **串行性靠 lane 保证**，不靠业务层加锁：例如写地图 = `LaneSpec{"disk_write", 1}`。
3. **取消是协作式的**：任务体必须周期性检查 `CancelToken`；被取消的任务仍会回调 `done(id,false)`。
4. **关停顺序**：先停生产（close channel）→ 停业务线程 → 最后 `shutdown` 池（业务线程关停时可能还要投递任务）。

## 示例

```cpp
using namespace rosiwit_common;

ThreadPool pool({
  {"disk_write", /*threads=*/1, /*capacity=*/8,  LaneOverflow::kReject},
  {"cpu",        /*threads=*/2, /*capacity=*/16, LaneOverflow::kReject},
});

pool.submit("disk_write", {"save_map", 0},
  [](CancelToken t) { saveMap(t); },            // 任务体
  [](JobId id, bool ok) { /* 回到业务线程的“待应用”队列 */ });
```
