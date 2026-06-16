# ROS2 Coverage Planner

ROS2 全覆盖路径规划功能包，为清洁机器人、割草机器人等场景提供**弓字形（Zigzag）和回字形（Spiral）**两种全覆盖路径规划算法。

## 功能特性

### P0 核心功能（已实现）
- ✅ 弓字形路径规划器（ZigzagPlanner）
  - BSA扫描线算法
  - 障碍物打断恢复机制
  - 自动选择最优扫描方向

- ✅ 回字形路径规划器（SpiralPlanner）
  - 从外向内的螺旋覆盖
  - 非凸区域分解
  - 降级策略（失败时切换到弓字形）

- ✅ ROS2 接口层
  - 地图订阅和路径发布
  - 触发规划服务接口
  - 参数动态配置

### P1 增强功能（已实现）
- ✅ 覆盖率统计工具（CoverageStats）
- ✅ 路径优化与平滑
- ✅ 单元测试框架

## 目录结构

```
ros2_coverage_planner/
├── include/coverage_planner/
│   ├── i_planner.hpp           # 规划器抽象接口
│   ├── planner_context.hpp     # 策略上下文
│   ├── zigzag_planner.hpp      # 弓字形规划器
│   ├── spiral_planner.hpp      # 回字形规划器
│   ├── coverage_utils.hpp      # 工具类
│   ├── coverage_planner.hpp    # ROS2节点
│   ├── zone_decomposer.hpp     # 分区规划器 [新增]
│   ├── turn_optimizer.hpp      # 转弯优化器 [新增]
│   ├── map_preprocessor.hpp    # 地图预处理器 [新增]
│   └── scan_direction_optimizer.hpp # 扫描方向优化器 [新增]
│
├── src/
│   ├── zigzag_planner.cpp      # 弓字形算法实现
│   ├── spiral_planner.cpp      # 回字形算法实现
│   ├── coverage_utils.cpp      # 工具类实现
│   ├── planner_context.cpp     # 策略上下文实现
│   └ coverage_planner_node.cpp # ROS2节点实现
│   ├── zone_decomposer.cpp      # 分区规划器实现 [新增]
│   ├── turn_optimizer.cpp       # 转弯优化器实现 [新增]
│   ├── map_preprocessor.cpp     # 地图预处理器实现 [新增]
│   └── scan_direction_optimizer.cpp # 扫描方向优化器实现 [新增]
│
├── test/
│   ├── test_zigzag_planner.cpp # 弓字形测试
│   ├── test_spiral_planner.cpp # 回字形测试
│   └ test_coverage_utils.cpp  # 工具类测试
│   ├── test_zone_decomposer.cpp # 分区规划器测试 [新增]
│   └── test_turn_optimizer.cpp  # 转弯优化器测试 [新增]
│
├── docs/
│   ├── api.md                   # API参考文档
│   ├── architecture.md          # 架构说明
│   ├── changelog.md             # 变更记录
│   ├── security_report.md       # 安全审计报告
│   ├── requirements.md          # 需求文档
│   └── deployment_report.md     # 部署报告
│
├── config/
│   └ coverage_params.yaml     # 参数配置文件
│
├── launch/
│   └ coverage_planner.launch.py # Launch文件
│
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 安装与编译

### 前置要求
- ROS2 Humble
- Eigen3
- tf2

### 编译步骤

```bash
# 1. 创建ROS2工作空间（如果尚未创建）
mkdir -p ~/ros2_ws/src

# 2. 克隆功能包到工作空间
cd ~/ros2_ws/src
# 将本功能包放置于此

# 3. 编译
cd ~/ros2_ws
colcon build --packages-select ros2_coverage_planner

# 4. 加载环境变量
source install/setup.bash
```

## 使用方法

### 1. Launch启动

```bash
ros2 launch ros2_coverage_planner coverage_planner.launch.py
```

### 2. 带参数启动

```bash
ros2 launch ros2_coverage_planner coverage_planner.launch.py \
    coverage_mode:=zigzag \
    robot_radius:=0.3 \
    coverage_resolution:=0.1
```

### 3. 触发规划

收到地图和初始位置后自动触发，或通过服务手动触发：

```bash
# 触发规划服务
ros2 service call /plan_coverage std_srvs/srv/Trigger
```

### 4. 查看路径

```bash
# 查看发布的路径
ros2 topic echo /coverage_path

# RViz可视化
rviz2 -d <your_config>
```

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `coverage_mode` | string | zigzag | 规划模式：zigzag 或 spiral |
| `robot_radius` | double | 0.3 | 机器人半径（米） |
| `coverage_resolution` | double | 0.1 | 覆盖路径分辨率（米） |
| `inflation_radius` | double | 0.25 | 障碍物膨胀半径（米） |
| `enable_optimization` | bool | true | 启用路径优化 |
| `direction_optimization` | int | 2 | 扫描方向：0=水平，1=垂直，2=自动 |
| `frame_id` | string | map | 路径参考坐标系 |

## ROS2接口

### 订阅话题
- `/map` (nav_msgs/OccupancyGrid) - 栅格地图
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped) - 初始位置

### 发布话题
- `/coverage_path` (nav_msgs/Path) - 覆盖路径

### 服务接口
- `/plan_coverage` (std_srvs/Trigger) - 触发规划

## 算法说明

### 弓字形算法（Zigzag）
适用于矩形房间，执行水平或垂直来回扫描：
1. 根据地图形状选择最优扫描方向
2. 提取扫描线段
3. 遇到障碍物打断后恢复
4. 连接所有扫描线段

### 回字形算法（Spiral）
适用于凸区域，从外向内螺旋覆盖：
1. 从边界开始螺旋移动
2. 每完成一圈向内缩小边界
3. 非凸区域自动分解
4. 降级策略：失败时切换到弓字形

## 覆盖率统计

规划完成后提供以下统计数据：
- **coverage_rate**: 覆盖率 (0.0-1.0)
- **path_length**: 路径长度（米）
- **turn_count**: 转弯次数

## 单元测试

```bash
# 运行所有测试
colcon test --packages-select ros2_coverage_planner

# 查看测试结果
colcon test-result --verbose
```

## 典型使用场景

1. **清洁机器人**：弓字形模式覆盖矩形房间
2. **割草机器人**：回字形模式覆盖开阔草地
3. **巡检机器人**：全覆盖路径保证区域检查

## 性能指标

| 指标 | 弓字形 | 回字形 |
|------|--------|--------|
| 矩形房间覆盖率 | >99% | >99% |
| 复杂区域覆盖率 | >85% | >85% |
| 100x100栅格规划时间 | <1s | <2s |

## 依赖项

- ROS2 Humble
- Eigen3 (>=3.3)
- tf2
- nav_msgs
- geometry_msgs

## 许可证

Apache-2.0

## 作者

Your Name (your_email@example.com)

## 参考

- [Nav2 Coverage Server](https://navigation.ros.org/)
- [BSA Algorithm Paper](https://doi.org/10.1109/ICRA.2001.932737)---

## 📦 部署指南

### Docker容器化部署

#### 1. 使用Dockerfile构建镜像

```bash
# 构建生产环境镜像
docker build -t ros2_coverage_planner:latest -f Dockerfile .

# 构建开发环境镜像（带编译工具）
docker build -t ros2_coverage_planner:dev -f Dockerfile --target development .

# 构建测试环境镜像
docker build -t ros2_coverage_planner:test -f Dockerfile --target testing .
```

#### 2. 使用docker-compose多服务编排

```bash
# 启动生产服务
docker-compose up coverage_planner

# 启动开发环境（挂载源代码，实时编译）
docker-compose --profile dev up dev

# 运行测试
docker-compose --profile test up test

# 演示模式（包含模拟地图发布者）
docker-compose --profile demo up
```

#### 3. Docker运行示例

```bash
# 直接运行容器
docker run -it --rm \
    --network ros2_coverage_network \
    ros2_coverage_planner:latest \
    ros2 launch ros2_coverage_planner coverage_planner.launch.py

# 挂载配置文件覆盖参数
docker run -it --rm \
    -v ./config/custom_params.yaml:/ros2_ws/src/ros2_coverage_planner/config/coverage_params.yaml \
    ros2_coverage_planner:latest
```

### Ubuntu 22.04原生部署

#### 系统要求
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- 至少4GB内存（大地图场景）
- GCC 11+ 或 Clang 14+

#### 安装依赖

```bash
# 安装ROS2 Humble（如未安装）
sudo apt update && sudo apt install -y \
    software-properties-common curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base

# 安装开发工具
sudo apt install -y \
    build-essential cmake git \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    googletest libgtest-dev

# 初始化rosdep
rosdep init
rosdep update
```

#### 编译和运行

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 克隆项目（假设）
git clone <repository_url> ros2_coverage_planner

# 安装依赖
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# 编译（Release模式）
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_coverage_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# 加载环境
source install/setup.bash

# 运行节点
ros2 launch ros2_coverage_planner coverage_planner.launch.py
```

---

## 🧪 测试执行指南

### 单元测试

```bash
# 编译测试版本
colcon build --packages-select ros2_coverage_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

# 运行所有测试
colcon test --packages-select ros2_coverage_planner

# 查看测试结果详情
colcon test-result --verbose

# 生成测试覆盖率报告（需lcov）
colcon test --packages-select ros2_coverage_planner
lcov --capture --directory build/ros2_coverage_planner --output-file coverage.info
genhtml coverage.info --output-directory coverage_report
```

### 集成测试

```bash
# 启动测试地图发布者
ros2 topic pub /map nav_msgs/OccupancyGrid \
    "{header: {frame_id: 'map'}, info: {resolution: 0.05, width: 100, height: 100,
    origin: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1}}},
    data: [0,0,0,...]}" --rate 1.0

# 启动规划节点
ros2 launch ros2_coverage_planner coverage_planner.launch.py coverage_mode:=zigzag

# 设置起始位置
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 1.0, y: 1.0, z: 0},
    orientation: {w: 1}}}}"

# 触发规划服务
ros2 service call /plan_coverage std_srvs/srv/Trigger

# 查看规划路径
ros2 topic echo /coverage_path
```

### 性能基准测试

```bash
# 测试100x100地图性能（应<500ms）
time ros2 run ros2_coverage_planner test_performance --map-size 100

# 测试500x500地图性能（应<3s）
time ros2 run ros2_coverage_planner test_performance --map-size 500

# 内存占用测试
valgrind --tool=massif ros2 launch ros2_coverage_planner coverage_planner.launch.py
```

---

## 🔒 安全修复说明

### 已修复的安全漏洞（2026-04-28）

#### VULN-001: 整数溢出风险 [Critical] ✅ 已修复
- **位置**: `coverage_utils.cpp:346`
- **修复**: 添加溢出检查（max 46340x46340），使用size_t类型
- **影响**: 防止恶意超大地图导致整数溢出和内存访问异常

#### VULN-002: 资源耗尽DoS风险 [High] ✅ 已修复
- **位置**: `coverage_planner_node.cpp:131-135`
- **修复**: 将WARN改为ERROR，添加return拒绝超大地图
- **影响**: 防止超大地图耗尽系统资源

#### VULN-003: Resolution除零崩溃 [High] ✅ 已修复
- **位置**: `coverage_utils.cpp:45, 124`
- **修复**: 在mapCallback中添加resolution>0验证
- **影响**: 防止恶意地图消息导致除零崩溃

### 安全审计报告
详见项目根目录的 `security_report.md`（已通过OWASP Top 10和STRIDE威胁模型审计）。

---

## 🚀 CI/CD流程

### GitHub Actions自动化流水线

项目已配置完整的CI/CD流程（`.github/workflows/ci.yml`），包含：

#### Job 1: 代码质量检查
- ✅ Clang-format格式化检查
- ✅ Cppcheck静态分析
- ✅ 代码规范验证

#### Job 2: 编译验证
- ✅ Ubuntu 22.04 + ROS2 Humble环境
- ✅ Release模式编译
- ✅ 构建产物上传

#### Job 3: 单元测试
- ✅ Google Test框架执行
- ✅ 覆盖率统计
- ✅ 测试结果上传

#### Job 4: 安全扫描
- ✅ CodeQL SAST扫描
- ✅ TruffleHog密钥检查
- ✅ 依赖安全审计

#### Job 5: Docker镜像构建
- ✅ 多阶段镜像构建
- ✅ 镜像测试验证
- ✅ 缓存优化

#### Job 6: 发布准备
- ✅ Changelog生成
- ✅ Release artifacts打包

### 手动触发CI

```bash
# 推送到main分支触发完整CI
git push origin main

# 创建PR触发CI（不构建Docker镜像）
git checkout -b feature/new-algorithm
git push origin feature/new-algorithm

# 创建Release触发发布流程
git tag v1.0.0
git push origin v1.0.0
gh release create v1.0.0
```

---

## 🎯 性能调优建议

### 1. 大地图优化
```yaml
# config/coverage_params.yaml
coverage_mode: "zigzag"  # 弓字形在大地图上更快
max_map_width: 500       # 限制最大尺寸
max_map_height: 500
coverage_resolution: 0.1 # 降低分辨率加速规划
```

### 2. 障碍物环境优化
```yaml
coverage_mode: "spiral"  # 回字形在复杂环境中更平滑
robot_radius: 0.2        # 减小膨胀半径提高覆盖率
```

### 3. 实时性能监控
```bash
# 监控规划节点CPU/内存
ros2 topic hz /coverage_path
top -p $(pgrep -f coverage_planner_node)

# 查看规划统计
ros2 service call /get_statistics coverage_planner/srv/GetStatistics
```

---

## ❓ 常见问题FAQ

### Q1: 编译失败找不到Eigen3
```bash
sudo apt install libeigen3-dev
# 确认CMakeLists.txt中正确声明：
find_package(Eigen3 REQUIRED)
```

### Q2: 测试覆盖率低于80%
- 检查是否编译了测试版本：`-DBUILD_TESTING=ON`
- 运行所有测试：`colcon test --packages-select ros2_coverage_planner`
- 查看缺失覆盖的模块：`colcon test-result --verbose`

### Q3: Docker容器无法连接ROS话题
- 确保使用相同的ROS_DOMAIN_ID
- 使用共享网络：`docker-compose --network host`
- 检查DDS配置：`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

### Q4: 地图尺寸超过限制
- 修改配置：`max_map_width` 和 `max_map_height`
- 注意：超过46340x46340会被安全机制拒绝

### Q5: 路径规划结果为空
- 检查地图是否发布：`ros2 topic echo /map`
- 检查起始位置：`ros2 topic echo /initialpose`
- 检查日志：`ros2 launch ... --log-level debug`

---

## 🤝 贡献指南

### 提交代码
1. Fork项目仓库
2. 创建feature分支：`git checkout -b feature/new-feature`
3. 提交更改：`git commit -m "Add new feature"`
4. 推送分支：`git push origin feature/new-feature`
5. 创建Pull Request

### 代码规范
- 使用clang-format格式化（项目根目录有`.clang-format`）
- 遵循ROS2编码规范
- 添加单元测试覆盖新功能
- 更新README文档

### 测试要求
- 新功能必须有单元测试
- 覆盖率不低于80%
- 通过CI所有检查

---

## 📄 许可证

Apache 2.0 License - 详见 `LICENSE` 文件

---

## 📚 参考资料

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [BSA Algorithm Paper](https://arxiv.org/abs/...)（待补充）
- [Coverage Path Planning Survey](https://arxiv.org/abs/...)

---

## 📞 联系方式

- 项目仓库：[GitHub链接]
- 问题反馈：GitHub Issues
- 维护者：AI Development Team

---

## 📊 测试结果（2026-04-29）

### 转弯优化修复验证

修复转弯合并条件后，四大优化功能已验证有效：

| 地图 | 覆盖率 | 转弯减少 | 预处理变化 | 分区数量 |
|------|--------|---------|-----------|---------|
| map.pgm | 96.03% | **39.7%** ✅ | 4.55% | 1 |
| layer_0 | 97.22% | **42.6%** ✅ | 1.33% | 4 |
| layer_3 | 98.33% | **38.4%** ✅ | 6.00% | 1 |

**修复内容**：
- 放宽转弯合并条件（GENTLE + MEDIUM类型均可合并）
- 增加合并距离阈值（10→50栅格）
- SHARP转弯特殊处理（距离限制20栅格）

**测试报告**: `map/test_results_v2.json`
**可视化对比图**: 35张PNG图片已生成

---

**最后更新**: 2026-04-29 | **版本**: 2.0 | **状态**: ✅ Production Ready + Bug Fixed