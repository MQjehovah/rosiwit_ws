# rosiwit_slam 变更记录

所有重要更改将记录在本文件中。格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)。

---

## [1.2.0] - 2026-05-05

### 新增
- **集成 Launch 文件**: `simulator_slam_demo.launch.py`（位于 rosiwit_simulator 包），集成 Gazebo 仿真器与 SLAM 节点，支持自动轨迹运动建图
- **Docker 安全加固**: 容器 `cap_drop ALL` + 最小 `cap_add`、`no-new-privileges`、非 root 用户、资源限制、日志轮转、健康检查

### 变更
- **包名规范化**: 所有 launch 文件中的包名引用从 `fast_lio2_slam` 统一为 `rosiwit_slam`
- **可执行文件重命名**: `${PROJECT_NAME}` (rosiwit_slam) 不再同时用于 `rosidl_generate_interfaces` 和 `add_executable`，引入 `set(NODE_NAME fast_lio2_node)` 替代可执行目标名
- **文档更新**: 项目 README、API 参考、架构文档同步更新包名和节点名引用

### 修复
- **BUG-001 (High)**: `livox_avia.launch.py` 第 20/27/28/29 行仍引用 `fast_lio2_slam`，已全部改为 `rosiwit_slam`/`fast_lio2_node`
- **BUG-002 (Medium)**: `CMakeLists.txt` BUILD_TESTING 块未防护 Sophus 依赖，添加 `find_package(Sophus QUIET)` 检查
- **编译修复 #1**: `CMakeLists.txt` 目标名冲突 — `rosidl_generate_interfaces` 和 `add_executable` 都使用 `${PROJECT_NAME}`，引入 `NODE_NAME` 变量
- **编译修复 #2**: `CMakeLists.txt` 缺少 C 语言 — `project(... LANGUAGES CXX)` 改为 `LANGUAGES C CXX`
- **编译修复 #3**: `CMakeLists.txt` 无效库导出 — 删除 `ament_export_libraries(fast_lio2_slam_lib)`
- **编译修复 #4**: `fast_lio2_node.h` 双重 `shared_ptr` — `std::shared_ptr<Pose::SharedPtr>` 改为 `Pose::SharedPtr`
- **编译修复 #5**: `sophus_se3.hpp` 模板歧义 — `.block<N,M>()` 改为 `.template block<N,M>()`，显式 `(Matrix3)` 转换
- **编译修复 #6**: `global_localizer.h` 重复定义 — 删除与 `types.h` 重复的 `LocalizationState`/`LocalizationResult`
- **Launch 修复**: `fast_lio2.launch.py` 和 `livox_avia.launch.py` 包名/可执行名更新为 `rosiwit_slam`/`fast_lio2_node`

### 安全
- **SEC-001 (Critical)**: Dockerfile 创建 `slam_user` 但未 `USER` 切换，容器以 root 运行 → 已修复
- **SEC-002 (High)**: `docker-compose.yml` 使用 `privileged: true` → 已改为最小权限
- **SEC-003 (High)**: `fetch_ntu_viral.py` 使用 `shell=True` 存在命令注入风险 → 已修复
- **SEC-004 (High)**: `ROS_DOMAIN_ID=0` 默认域暴露 → 已改为 `42`
- **SEC-005 (High)**: Jenkinsfile `--privileged` CI 逃逸 → 已修复

### 测试
- **第 1 轮**: 120 用例，111 通过 (92.5%)，7 失败，2 跳过
- **第 2 轮 (回归)**: 120 用例，118 通过，0 失败，2 跳过 — ✅ 全部通过

---

## [1.1.0] - 2026-04-24

### 新增
- 建图功能增强模块
- MapServer ROS2 服务接口
- MapPersistence 持久化模块
- MapQuality 质量评估模块
- 多会话建图支持
- 子地图系统
- 完善配置参数

---

## [1.0.0] - 2026-04-20

### 新增
- FAST-LIO2 核心 SLAM 算法实现
- ROS2 Humble 节点封装
- Velodyne VLP-16 配置
- Livox Avia 配置
- Ouster OS1 配置
- IEKF 状态估计器
- iKD-Tree 空间索引
- Scan Context 闭环检测
- 基础 RViz 可视化配置
