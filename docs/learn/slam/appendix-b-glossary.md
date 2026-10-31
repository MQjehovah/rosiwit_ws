# 附录 B 术语表（中英对照）

## 位姿与坐标

| 中文 | 英文 | 说明 |
|---|---|---|
| 位姿 | pose | 位置 + 姿态，SE(3) 元素 |
| 刚体变换 | rigid transform | 只有旋转与平移，不改变形状 |
| 世界系 | world frame | 全局参考系（map 或重力对齐系） |
| 体体系 | body frame | 固定在机器人上的坐标系 |
| 外参 | extrinsic | 传感器之间的刚体变换 |
| 内参 | intrinsic | 传感器自身参数（如激光角分辨率） |
| 里程计系 | odom frame | 连续但有漂移的参考系 |
| 修正量 | correction | 全局系与里程计系之间的变换 |

## 配准

| 中文 | 英文 | 说明 |
|---|---|---|
| 配准 | registration | 求两片点云之间的变换 |
| 对应点 | correspondence | 点与点的匹配关系 |
| 最近邻 | nearest neighbor | 最常见的对应方式 |
| 内点 | inlier | 残差在阈值内的对应 |
| 外点 | outlier | 错误对应（动态物体等） |
| 鲁棒核 | robust kernel | 抑制外点影响的损失函数 |
| 覆盖度 | coverage / fitness | 内点占比，跨配准器统一口径 |
| 收敛 | convergence | 迭代满足停止条件 |
| 退化 | degeneracy | 某方向约束不足，解不唯一 |

## LIO 与滤波

| 中文 | 英文 | 说明 |
|---|---|---|
| 预积分 | preintegration | IMU 两帧间的相对运动积分 |
| 零偏 | bias | 传感器系统性偏差 |
| 随机游走 | random walk | 零偏随时间漂移的模型 |
| 误差状态 | error state | 相对名义状态的小量 |
| 紧耦合 | tightly coupled | 原始观测进同一估计器 |
| 松耦合 | loosely coupled | 子系统先出位姿再融合 |
| 去畸变 | deskew / motion compensation | 补偿扫描期间的运动 |

## 地图

| 中文 | 英文 | 说明 |
|---|---|---|
| 点云地图 | point cloud map | 3D 点集 |
| 占据栅格 | occupancy grid | 0..100 的栅格地图 |
| 未知 | unknown | `-1`，既非墙也非自由 |
| 代价 | cost | `1..99`，可通行但有风险 |
| 占据 | occupied | `100`，不可通行 |
| 距离场 | distance field / ESDF | 到最近障碍的距离 |
| 截断符号距离 | TSDF | 体素内的符号距离 |
| 叠加层 | overlay | 运行期变化，不改先验图 |

## 后端与重定位

| 中文 | 英文 | 说明 |
|---|---|---|
| 位姿图 | pose graph | 节点位姿 + 边约束 |
| 因子图 | factor graph | 更通用的图模型 |
| 回环 | loop closure | 重访同一地点形成的约束 |
| 地点识别 | place recognition | 找出“这是哪里” |
| 描述子 | descriptor | 用于检索的紧凑表示 |
| 分支定界 | branch and bound | 全局最优搜索 |
| 多假设 | multi-hypothesis | 同时跟踪多个候选解 |
| 歧义 | ambiguity | 多个候选分数接近 |

## 健康与评估

| 中文 | 英文 | 说明 |
|---|---|---|
| 健康度 | health | 位姿可信程度的整体判断 |
| 降级 | degraded | 可用但不可信 |
| 丢失 | lost | 不可用，需重定位 |
| 新息 | innovation | 观测与预测之差 |
| 卡方检验 | chi-square test | 判断观测是否离群 |
| 归一化误差平方 | NEES | 校验协方差是否诚实 |
| 绝对轨迹误差 | ATE | 全局一致性指标 |
| 相对位姿误差 | RPE | 局部漂移率指标 |

## 工程

| 中文 | 英文 | 说明 |
|---|---|---|
| 看门狗 | watchdog | 超时检测并主动停车 |
| 回调队列 | callback queue | ROS 消息回调的执行队列 |
| 灰度 | canary / staged rollout | 小范围先验证 |
| 启动自检 | startup self-check | 启动阶段的断言检查 |
| 回放 | replay | 用录制的数据离线复现 |
