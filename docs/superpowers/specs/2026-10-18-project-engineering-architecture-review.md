# 项目工程化与架构改进建议（2026-10-18）

> 依据：全仓库结构核查 + 2026-10-17/18 深夜实战（推行丢失事故链 6 起连续修复）的一手证据。
> 每条建议标注了实证来源。先说结论：**这个项目的底子是对的**——可插拔管线、核心无 ROS 依赖、
> 事故驱动测试与文档纪律（§14-17）都是多数团队做不到的。下面的建议按"还债优先级"排序。

## 〇、值得保持的优势（不要在重构中丢掉）

| 优势 | 实证 |
|---|---|
| 阶段可插拔管线（工厂模式） | `localization_factory.cpp`：matcher/prior/local_map/monitor/relocalizer 全部可替换，今晚 EKF 先验 2 小时上线即受益 |
| 核心/ROS 薄层分离 | 核心零 ROS 依赖，7+10 个测试直接编译核心，无需 ROS 环境 |
| 事故驱动测试纪律 | 每起事故一个回归测试（`test_gicp_matcher_guard`/`test_reloc_disambig`/ZombieBand），今晚全部发挥作用 |
| 开发约束文档 | dev-constraints §5/§11/§14-17 的坑位记录多次避免二次踩坑 |
| 非破坏性叠加地图维护 | overlay + revision 设计干净，切图不清图语义清晰 |

## 一、工程化改进（按 ROI 排序）

### 1. CI 完全缺失 —— 最高优先级
**实证**：无 GitHub Actions；Jenkinsfile 只在 simulator 里。17 个测试二进制全部手工跑
（今晚每次改动 = 本地手跑 + tar + ssh + 机器人编译，一轮 15 分钟）。
**建议**：
- GitHub Actions：`ros1_ws` melodic 容器构建 + 全部测试（docker/ 已有 build.sh 可作种子）
- PR 必须绿；nightly 加 ROS2 容器 job（顺带解决"ROS2 回归永久 blocked"问题）
- 核心库独立构建 job（纯 cmake，无 ROS），强制守住"核心无 ROS 依赖"红线

### 2. 日志没有时间戳 —— 最便宜的史诗级改进
**实证**：`rosiwit_slam_swap.log` 无时间戳，今晚取证全靠 PubProbe tick 数（10s 一个）推算
事件时刻，"31m 瞬移发生在哪两帧之间"花了一小时。
**建议**：rosconsole 开 `%time`/`%thread`；或核心内统一诊断 logger（见架构#5）。
一次改动，今后所有现场排查从小时级降到分钟级。

### 3. 观测性：从 print 探针到"飞行记录仪"
**实证**：MatchProbe/DropProbe/LocProbe 是今晚破案主力，但都是 cout、采样式（512 帧一条）、
事后无法追溯丢帧区间。
**建议**：环形缓冲记录最近 N 帧（fitness/lag/src/tgt/status/事件），LOST/DEGRADED 翻转或
SIGUSR2 时落盘。现场"刚才丢了"→ 一个文件回放全程。

### 4. 构建系统
**实证**：`file(GLOB_RECURSE)` 收源（新增文件需重跑 cmake；误入文件会被静默编译）；
include/pch.hpp 存在但未启用 PCH（PCL/Eigen 重型头，全量编译慢）。
**建议**：显式源列表或至少 `CONFIGURE_DEPENDS`；对核心库启用 PCH/unity build。

### 5. 仓库卫生
**实证**：双包树并存（`src/*` 带 COLCON_IGNORE + `ros2_ws/src/*_ros2`）；模拟器二进制资产
（模型/贴图/地图/.DS_Store）直接入库；无版本 tag；部署靠手工 tar+ssh 脚本（不在库内）。
**建议**：
- 完成 src/ → ros2_ws/ 迁移收尾，删旧树
- 大资产移 Git LFS 或独立 asset 仓
- 语义化 tag + CHANGELOG；部署脚本入库 + 机器人侧版本可查询（/slam/info 或 rosparam），
  现场问题能映射到精确代码版本（今晚全程靠记忆对版本）

### 6. 测试深化
**实证**：单测覆盖事故路径很好；但 PCL <20 点段错误这类"退化输入崩溃"是靠机器人死人发现的。
**建议**：
- 模糊/属性测试：对 matcher/prior/local_map 灌退化输入（空云/16 点/NaN/全同点）
- **rosbag 回放测试框架**：把现场问题（快速旋转丢失至今开着口子）录成 bag → 固定回放
  断言 fitness/状态轨迹 → 现场事故自动变回归用例。这是把"现场排障能力"产品化的关键一步

## 二、架构改进（按风险排序）

### 1. 线程模型 —— 所有重大事故的公共因子
**实证**：单线程 `ros::spin()`。今晚：FPFH 构建冻结 44.6s、/relocation 排队数十秒（§14-1）、
歧义跟踪 CPU 打满致 /scan 饿到 0.67Hz。后台构建线程是第一步，但**重定位检索（~15s）仍在
回调内**，冷启动期间输出断流。
**建议**（分三步，不要一步到位）：
1. 重定位检索移入 worker（结果经 mailbox 回主循环）——这是 §14-1 的完结篇
2. 明确"回调预算契约"：传感器回调只允许 ms 级工作，重活全部 async（§5 有原则无强制）
3. 传感器输入独立 callback queue（发布/服务永不被计算饿死）

### 2. 状态机三驾马车 → 单一权威状态机
**实证**：MonitorLevel（core monitor）→ pipeline `m_status` → ROS 层 LostMonitor/location_status
三层各自判"丢"，中间还有过桥条款把 LOST 静默吞掉（今晚僵尸带 bug #2）。
**建议**：显式状态机（INIT/TRACKING/DEGRADED/TENTATIVE/LOST + 事件转移表），单一写入者，
各层只读。转移表进文档，新增"平庸持续→LOST"这类规则时改一处而不是三处。

### 3. 位姿仲裁权分散
**实证**：matcher、EKF 先验、假设管理器、有界恢复、冷启动都能写 `m_pose`，采纳规则各异
（今晚 31m 瞬移 = 冷启动候选无门控直写；假设采纳可达性门控只覆盖了重播种路径）。
**建议**：位姿仲裁组件统一裁决（新息/可达性/歧义三重门控），歧义期假设管理器是唯一写者。
冷启动接受、有界恢复、假设切换共用同一套门控参数。

### 4. 隐式契约显式化
**实证**：GICP 目标缓存靠指针判等（PCL setInputTarget 副作用驱动）；reloc map 快照指针比对；
overlay revision + cache_move + 目标缓存三者交互决定失效——全对，但全靠注释维系。
**建议**：引入 MapView/版本对象封装（不可变快照 + 显式失效语义），契约写在头文件接口上
而不是实现注释里。新人或三个月后的自己不会再踩。

### 5. ROS 层 1127 行上帝接口
**实证**：`legacy_xz3dslam_interface.cpp` 混装协议仿真（topic/service/TF）、业务逻辑
（告警码/模式机/抑制窗）、坐标换算。applyInitPose 的收敛（单一实现）证明拆分有效。
**建议**：拆成 LegacyProtocolAdapter（纯翻译，表驱动，可对拍 golden 测试——test fixtures
基础已在）+ InterfaceController（业务）。每拆一段，ROS2 层就能复用一段。

### 6. 配置治理
**实证**：default.yaml 600+ 行单文件；今晚事故链之一 = accept.min_coverage(0.35) /
monitor.credible_fitness(0.75) / reloc_accept_score(0.85) 三个阈值互相打架。
**建议**：加载时一致性校验（断言 reloc_accept ≥ credible > min_coverage 等不变量）；
阶段化 include；把"调这个参数会牵动谁"写进每个阈值的注释（部分已做，保持）。

## 三、落地路线图

| 阶段 | 内容 | 预期 |
|---|---|---|
| 本周 | 日志时间戳；GH Actions（build+test）；tag v1.0 | 排障分钟级；PR 有门禁 |
| 两周内 | 重定位检索出回调；配置不变量校验；bag 回放测试框架 | 冷启动不再断流；现场事故自动回归化 |
| 一个月 | 状态机统一；飞行记录仪；核心独立 CI job | 僵尸类 bug 结构性绝迹 |
| 一季度 | 位姿仲裁统一；ROS 层拆分；src/ 迁移收尾 + LFS | ROS2 复用成为现实 |

**总原则**：以上每一项都对应一个今晚真实发生过的事故或排障痛点——不追新概念，
只还已经欠下、且继续产生利息的债。
