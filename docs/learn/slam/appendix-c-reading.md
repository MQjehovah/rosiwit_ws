# 附录 C 推荐阅读

> 按“先建立直觉，再读实现”的顺序组织。

## 一、建立直觉

| 材料 | 类型 | 为什么读 |
|---|---|---|
| Probabilistic Robotics（Thrun 等） | 书 | 状态估计视角的经典，前 4 章足够建立框架 |
| 《视觉 SLAM 十四讲》（高翔） | 书 | 中文，数学推导完整，适合补基础 |
| State Estimation for Robotics（Barfoot） | 书 | 李群与状态估计的系统推导 |

## 二、激光 SLAM 与 LIO

| 论文 / 项目 | 要点 |
|---|---|
| LOAM（Zhang & Singh, 2014） | 两级 scan-to-scan / scan-to-map 的源头 |
| LeGO-LOAM | 地面分割 + 轻量化 |
| LIO-SAM | 因子图 + IMU 预积分 + 回环 |
| FAST-LIO / FAST-LIO2 | IEKF + ikd-tree，嵌入式友好 |
| Point-LIO | 逐点更新，高动态场景 |
| KISS-ICP | 极简但强基线，适合做对照实验 |

## 三、回环与地点识别

| 论文 / 项目 | 要点 |
|---|---|
| Scan Context | 2D 描述子，旋转鲁棒 |
| SegMap / 学习型描述子 | 语义/学习方式检索 |
| iSAM2 | 增量图优化 |

## 四、全局定位与重定位

| 论文 / 项目 | 要点 |
|---|---|
| BBS（Branch-and-Bound Scan Matching） | 2D 全局搜索，有最优性保证 |
| FPFH + RANSAC | 6DoF 特征匹配式全局定位 |
| Cartographer | 子图 + 分支定界回环的工程范例 |

## 五、多传感器融合与健康

| 材料 | 要点 |
|---|---|
| MSCKF 系列 | 滤波式视觉惯性融合的经典 |
| VINS-Mono | 优化式紧耦合，工程实现完整 |
| Nav2 文档 | 工程取舍、行为树、代价地图与恢复策略 |
| robot_localization | 松耦合 EKF 的工程实现 |

## 六、评估与工具

| 工具 | 用途 |
|---|---|
| evo | ATE/RPE 计算与轨迹对比 |
| rosbag / ros2 bag | 数据录制与回放 |
| rerun / Foxglove | 多模态时序可视化 |
| Ceres / gtsam | 优化求解 |

## 七、阅读方法建议

1. **先读摘要与系统图**，画出“数据流 + 状态 + 约束”，再读公式；
2. 每个系统回答三个问题：**状态是什么、观测是什么、失败怎么被发现**；
3. 不要试图一次读懂所有推导，先建立“它为什么这么设计”的直觉；
4. 读实现时优先看：初始化、更新、失败处理三处——那里藏着最多工程经验。

## 八、与本目录的对应关系

| 章节 | 建议对照阅读 |
|---|---|
| 03 配准 | KISS-ICP、GICP 论文 |
| 04 LIO | FAST-LIO2、LIO-SAM |
| 06 后端 | iSAM2、gtsam 示例 |
| 07 回环 | Scan Context、Cartographer |
| 08 重定位 | BBS、FPFH-RANSAC |
| 09 融合 | VINS-Mono、robot_localization |
| 10 健康 | Nav2 的恢复与监控设计 |
| 11 评估 | evo 文档 |
