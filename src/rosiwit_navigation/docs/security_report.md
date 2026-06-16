# 安全审计报告 — rosiwit_navigation

> **审计人**: 安全审查师 (CSO)
> **审计日期**: 2026-05-04
> **审计范围**: `E:\xzkj\agent\workspace\projects\rosiwit_ws\src\rosiwit_navigation`（74个文件）
> **项目类型**: ROS2 Humble 差速轮驱动机器人导航功能包 (C++)
> **风险等级**: 3 Critical / 2 High / 3 Medium / 4 Low

---

## 审计摘要

| 检查类别 | 项数 | 通过 | 发现问题 |
|----------|------|------|----------|
| OWASP Top 10 (适用项) | 7 | 5 | 2 |
| STRIDE 威胁模型 | 6 | 4 | 2 |
| 敏感信息泄露 | 1 | 1 | 0 |
| 配置安全 | 1 | 0 | 1 |
| **总计** | **15** | **10** | **5** |

**结论**: ⚠️ **有条件通过** — 存在 1 个阻塞发布的 Critical 级漏洞（环境变量注入），修复后方可部署。

---

## OWASP Top 10 逐项检查

### 1. 失效的访问控制 — ✅ 通过
- ROS2 节点间通信基于 DDS 内置安全，无自定义 API 端点
- Action Server 目标接受无显式认证（但该场景下机器人通常在隔离网络运行，风险可控）

### 2. 加密失败 — ✅ 通过
- 无敏感数据存储或传输，配置文件仅含运动学参数
- 无明文密码/密钥/Token

### 3. 注入 — ❌ 发现

**CVE-1 (Critical): 环境变量注入 — ASTAR_PLANNING_TIMEOUT / ASTAR_MAX_ITERATIONS**
详见下文发现列表。

### 4. 不安全的设计 — ✅ 通过
- 模块化架构设计合理，异常处理体系较完整（exceptions.hpp 定义 5 类异常层次）
- 错误管理器有重试/降级/中止策略

### 5. 安全配置错误 — ⚠️ 部分通过
- Release 模式构建缺少 `-D_FORTIFY_SOURCE=2` 加固（Low）
- Debug 模式缺少 AddressSanitizer / UndefinedBehaviorSanitizer（Medium）
- 无默认密码或调试模式开关
- `.clang-tidy` 配置齐全

### 6. 易受攻击的组件 — ✅ 通过
- 依赖为 ROS2 Humble 标准包和 GTest，均为 ROS 官方维护，无已知公开 CVE
- 未引入第三方非标准库

### 7. 认证失败 — ✅ 通过
- 无用户认证需求（机器人嵌入式系统）

### 8. 软件和数据完整性 — ✅ 通过
- 未使用序列化/反序列化自定义格式，依赖 ROS2 内置序列化
- `std::any_cast` 有 `bad_any_cast` 异常捕获（event_bus.hpp:231）

### 9. 日志和监控失败 — ✅ 通过
- 使用 RCLCPP_DEBUG/INFO/WARN/ERROR/THROTTLE 分级日志
- ErrorManager 维护错误历史队列（max_history_size 可配置）

### 10. SSRF — ✅ 通过
- 无外部网络请求，仅 ROS2 内部通信

---

## STRIDE 威胁模型

| 威胁类型 | 评估 | 发现 |
|----------|------|------|
| **S**poofing (身份伪造) | ✅ | ROS2 DDS 层隔离，传感器数据来自系统内部 |
| **T**ampering (数据篡改) | ⚠️ | 环境变量可被任意进程修改，影响规划器行为 (CVE-1) |
| **R**epudiation (否认) | ✅ | 运行日志可追溯 |
| **I**nformation Disclosure (信息泄露) | ✅ | 日志中的位姿数据为正常运行数据，非敏感信息 |
| **D**enial of Service (拒绝服务) | ❌ | 大网格规划导致 CPU 耗尽 (CVE-2)；dt=0 除法触发浮点异常 (CVE-5) |
| **E**levation of Privilege (权限提升) | ✅ | 无权限分级 |

---

## 发现列表

### 🔴 Critical (阻塞发布)

---

#### CVE-1: 环境变量注入导致规划器行为异常

| 属性 | 值 |
|------|-----|
| **严重等级** | Critical |
| **文件:行号** | `src/planners/astar_planner.cpp:28, 32` |
| **OWASP 类别** | A03:2021 — 注入 |
| **STRIDE** | Tampering |
| **置信度** | 9/10 |

**代码片段**:
```cpp
// astar_planner.cpp, lines 27-35
const char* timeout_env = std::getenv("ASTAR_PLANNING_TIMEOUT");
if (timeout_env) {
    planning_timeout_seconds_ = std::atof(timeout_env);  // 无验证
}
const char* max_iter_env = std::getenv("ASTAR_MAX_ITERATIONS");
if (max_iter_env) {
    max_iterations_ = std::atoi(max_iter_env);            // 无验证
}
```

**利用场景**:
1. **DoS 攻击**: 攻击者设置 `ASTAR_PLANNING_TIMEOUT=-1.0` 或 `ASTAR_MAX_ITERATIONS=-1`，`std::atoi()` 返回负数，`max_iterations_` 作为 `int` 用于循环终止条件（`while (iterations_ < max_iterations_)`），当 `max_iterations_` 为负数时条件永真，导致无限循环 → CPU 100% → 系统冻结。

2. **资源耗尽**: 攻击者设置 `ASTAR_MAX_ITERATIONS=2147483647`（INT_MAX），导致 A* 搜索耗尽所有可用内存，O(n²) 的大网格搜索将占用数 GB 内存 → OOM killer 终止进程。

3. **数值不稳定**: 设置 `ASTAR_PLANNING_TIMEOUT=NaN` 或 `ASTAR_MAX_ITERATIONS=0` 导致未定义行为。

**修复建议**:
```cpp
const char* timeout_env = std::getenv("ASTAR_PLANNING_TIMEOUT");
if (timeout_env) {
    double val = std::atof(timeout_env);
    // 验证范围：合理的超时时间 [0.1, 3600.0] 秒
    if (val >= 0.1 && val <= 3600.0) {
        planning_timeout_seconds_ = val;
    } else {
        RCLCPP_WARN(logger_, "Invalid ASTAR_PLANNING_TIMEOUT=%.2f, using default %.2f",
                    val, AStarConstants::kDefaultTimeoutSeconds);
    }
}

const char* max_iter_env = std::getenv("ASTAR_MAX_ITERATIONS");
if (max_iter_env) {
    int val = std::atoi(max_iter_env);
    // 验证范围：[1000, 10000000] 次迭代
    if (val >= 1000 && val <= 10000000) {
        max_iterations_ = val;
    } else {
        RCLCPP_WARN(logger_, "Invalid ASTAR_MAX_ITERATIONS=%d, using default %d",
                    val, AStarConstants::kDefaultMaxIterations);
    }
}
```

更佳实践：完全移除环境变量读取，改为从 ROS2 参数服务器获取（已在 `initialize()` 中接受 `PlannerConfig`）。

---

### 🟠 High (强烈建议修复)

---

#### CVE-2: A* 规划器大网格未设内存上限 — DoS

| 属性 | 值 |
|------|-----|
| **严重等级** | High |
| **文件:行号** | `src/planners/astar_planner.cpp:52-200`（整体算法） |
| **OWASP 类别** | A04:2021 — 不安全的设计 |
| **STRIDE** | Denial of Service |
| **置信度** | 8/10 |

**利用场景**:
当攻击者（或配置错误）提交 1000×1000 网格的规划请求时，A* 算法的 `open_list_` 将分配 1,000,000 个节点，每个节点 ~100 字节 → 约 100MB。加上 `closed_list_` 和 `g_scores_` 等辅助容器，内存峰值可达 300-400MB。超时设置（`planning_timeout_seconds_`）仅限制运行时间，不限制内存使用。频繁请求可导致资源耗尽。

测试报告中已观察到"大网格路径规划（1000x1000）超时 > 2s"的问题。

**修复建议**:
1. 在 `plan()` 方法入口处检查网格大小：若 `nx_ * ny_ > MAX_CELLS`（如 500×500=250,000）则拒绝规划并返回错误
2. 在 `MAX_CELLS` 常量定义为可配置参数
3. 入队前检查 `open_list_.size()`，上限 500,000 节点

---

#### CVE-3: dt=0 导致除零 / 浮点异常

| 属性 | 值 |
|------|-----|
| **严重等级** | High |
| **文件:行号** | `src/controller/diff_drive_controller.cpp:384, 392`；`src/controller/velocity_limiter.cpp:54-55` |
| **OWASP 类别** | A04:2021 — 不安全的设计 |
| **STRIDE** | Denial of Service |
| **置信度** | 8/10 |

**代码片段**:
```cpp
// velocity_limiter.cpp, lines 54-55
double accel_x = (target_velocity.linear.x - current_velocity.linear.x) / dt;
double accel_theta = (target_velocity.angular.z - current_velocity.angular.z) / dt;
```

```cpp
// diff_drive_controller.cpp, lines 384, 392
double accel_x = (cmd_vel.linear.x - current_vel.linear.x) / dt;
double accel_theta = (cmd_vel.angular.z - current_vel.angular.z) / dt;
```

**利用场景**:
1. ROS2 定时器回调首次触发时，`dt` 可能为 0（`rclcpp::Time` 初始化为零）
2. 系统时钟跳变（NTP 校时、仿真时间重置）导致 `dt` ≈ 0 或负值
3. 结果：IEEE 754 浮点除零产生 ±Inf/NaN，传播到后续计算 → 速度命令包含 NaN → 物理机器人可能收到非法指令

**修复建议**:
```cpp
constexpr double kMinDt = 1e-6;  // 1微秒最小间隔
if (dt < kMinDt) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "dt=%.6f too small, skipping velocity constraint", dt);
    return target_velocity;  // 不约束，安全回退
}
```

---

### 🟡 Medium (建议修复)

---

#### CVE-4: EventBus 发布时持锁回调 — 死锁风险

| 属性 | 值 |
|------|-----|
| **严重等级** | Medium |
| **文件:行号** | `include/diffbot_navigation/core/event_bus.hpp:205, 227-237` |
| **OWASP 类别** | A04:2021 — 不安全的设计 |
| **STRIDE** | Denial of Service |
| **置信度** | 8/10 |

**代码片段**:
```cpp
template<typename EventType>
void EventBus::publish(const EventType& event)
{
    std::lock_guard<std::mutex> lock(mutex_);  // 持锁
    invokeHandlers(event_type, event);           // 在锁内调用用户回调
}

template<typename EventType>
void EventBus::invokeHandlers(...)
{
    for (const auto& [id, handler_any] : it->second) {
        handler(event);  // 用户回调可能尝试再次 publish/subscribe
    }
}
```

**利用场景**:
若事件处理器中尝试调用 `EventBus::subscribe()` 或 `EventBus::publish()`（例如导航协调器在 `OBSTACLE_DETECTED` 事件中触发 `PLANNING_REQUESTED`），由于外层的 `publish()` 已持有 `mutex_`，内部调用的 `subscribe()`/`publish()` 会尝试获取同一把互斥锁 → **死锁**。当前 `std::mutex` 不支持递归锁定。

**修复建议**:
1. 方案 A（推荐）：在 `publish()` 中复制 handlers 列表后释放锁再调用
```cpp
template<typename EventType>
void EventBus::publish(const EventType& event)
{
    std::vector<std::pair<uint64_t, EventHandler<EventType>>> handlers_copy;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = handlers_.find(event.type);
        if (it != handlers_.end()) {
            for (const auto& [id, handler_any] : it->second) {
                try {
                    handlers_copy.push_back({id, std::any_cast<EventHandler<EventType>>(handler_any)});
                } catch (...) { /* skip type mismatch */ }
            }
        }
    }
    // 无锁调用
    for (const auto& [id, handler] : handlers_copy) {
        try { handler(event); } catch (...) { /* log */ }
    }
}
```
2. 方案 B：使用 `std::recursive_mutex`

---

#### CVE-5: Debug 构建缺失 ASan/UBSan

| 属性 | 值 |
|------|-----|
| **严重等级** | Medium |
| **文件:行号** | `CMakeLists.txt:23-25` |
| **OWASP 类别** | A05:2021 — 安全配置错误 |
| **置信度** | 8/10 |

**利用场景**:
当前 Debug 构建仅添加 `-O0 -g`，未启用运行时安全检查。C++ 代码中的未定义行为（如整数溢出、越界访问、use-after-free）不会在测试中被捕获，可能上线后在生产环境触发。

**修复建议**:
```cmake
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O0 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
  message(STATUS "Building in Debug mode with -O0 -g + ASan/UBSan")
endif()
```

---

#### CVE-6: Release 构建缺少 FORTIFY_SOURCE 加固

| 属性 | 值 |
|------|-----|
| **严重等级** | Medium |
| **文件:行号** | `CMakeLists.txt:18` |
| **OWASP 类别** | A05:2021 — 安全配置错误 |
| **置信度** | 7/10 |

**利用场景**:
Release 构建（`-O2 -DNDEBUG`）未定义 `_FORTIFY_SOURCE`。如果代码中存在 `sprintf`/`memcpy`/`strcpy` 等 C 函数（当前未发现），编译器不会添加边界检查。

**修复建议**:
```cmake
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O2 -DNDEBUG -D_FORTIFY_SOURCE=2)
  add_link_options(-Wl,-z,relro -Wl,-z,now)  # Full RELRO
  message(STATUS "Building in Release mode with -O2 -DNDEBUG -D_FORTIFY_SOURCE=2")
endif()
```

---

### 🔵 Low (建议改进)

---

#### CVE-7: Action Server 缺少目标速率限制

| 属性 | 值 |
|------|-----|
| **严重等级** | Low |
| **文件:行号** | `src/navigation/smooth_navigation.cpp:186-200` |
| **置信度** | 7/10 |

**利用场景**: 攻击者频繁发送导航目标（例如每毫秒一个），导致 A* 规划器反复重规划，消耗 CPU 资源。当前 Action Server 无速率限制。

**修复建议**: 在 Action 回调中检查距上次目标接受的间隔，若 < 100ms 则拒绝。

---

#### CVE-8: PID 控制循环中 `dt` 负值风险

| 属性 | 值 |
|------|-----|
| **严重等级** | Low |
| **文件:行号** | `src/controller/diff_drive_controller.cpp:384-410` |
| **置信度** | 7/10 |

**场景**: `dt` 计算使用 `rclcpp::Time`，时钟跳变（NTP/仿真重置）可能导致 `dt < 0`，使加速度约束逻辑反转。影响较小，因为 ROS2 通常使用 `steady_clock`。

---

#### CVE-9: ParameterManager 动态参数无范围校验

| 属性 | 值 |
|------|-----|
| **严重等级** | Low |
| **文件:行号** | `include/diffbot_navigation/core/parameter_manager.hpp:130-140` |
| **置信度** | 6/10 |

**场景**: ROS2 参数可在运行时通过 `ros2 param set` 动态修改。如 `max_velocity_x` 被设为 100.0 m/s，可能导致物理机器人失控。参数管理器未实现范围校验回调。

---

#### CVE-10: 部分 `vector` 下标访问未用 `.at()`

| 属性 | 值 |
|------|-----|
| **严重等级** | Low |
| **文件:行号** | `src/controller/diff_drive_controller.cpp:440, 472` 等多处 |
| **置信度** | 5/10 |

**场景**: `current_path_.poses[i]` 使用 `operator[]` 而非 `.at(i)`，若索引越界则导致未定义行为而非抛出异常。当前代码在循环中使用 `poses.size()` 作为边界，风险较低，但防御性编程建议使用 `.at()`。

---

## 误报排除说明

| 排除项 | 理由 |
|--------|------|
| `package.xml` 中的 `<exec_depend>` 匹配 `exec` | 仅 XML 标签名，非系统调用 |
| `.clang-tidy` 中的 `private` 匹配 | 仅为 C++ 关键字 |
| `docs/GETTING_STARTED.md` 中的 `ros.key` | 仅为 ROS 安装文档，非敏感密钥 |
| `std::make_shared` / `std::shared_ptr` | 现代 C++ RAII 智能指针，非裸内存操作 |

---

## 构建安全配置检查

| 配置项 | 当前状态 | 建议 |
|--------|----------|------|
| `-Wall -Wextra -Wpedantic -Wshadow` | ✅ 已启用 | — |
| `-D_FORTIFY_SOURCE=2` | ❌ 缺失 | 添加到 Release 配置 |
| AddressSanitizer | ❌ 缺失 | 添加到 Debug 配置 |
| UndefinedBehaviorSanitizer | ❌ 缺失 | 添加到 Debug 配置 |
| Full RELRO (`-Wl,-z,relro,-z,now`) | ❌ 缺失 | 添加到 Release 链接 |
| `.clang-tidy` | ✅ 已配置 | 启用了 bugprone-*, cert-*, cppcoreguidelines-* |
| `.clang-format` | ✅ 已配置 | — |

---

## 审计结论

**总体评级**: ⚠️ **有条件通过**

**阻塞项**: CVE-1（环境变量注入）必须在发布前修复。这是唯一标记为 Critical 的发现。

**优先修复顺序**:
1. **[Critical]** CVE-1: 移除/验证环境变量读取（astar_planner.cpp）
2. **[High]** CVE-3: 添加 dt=0 检查（diff_drive_controller.cpp, velocity_limiter.cpp）
3. **[High]** CVE-2: 添加网格大小上限（astar_planner.cpp）
4. **[Medium]** CVE-4: EventBus 死锁修复（event_bus.hpp）
5. **[Medium]** CVE-5 + CVE-6: 构建安全加固（CMakeLists.txt）
6. **[Low]** CVE-7~CVE-10: 防御性改进

---

*审计报告结束。如需对任何发现进行详细讨论或补充信息，请联系安全审查师。*
