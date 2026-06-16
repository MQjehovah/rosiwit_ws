# rosiwit_slam 关键函数说明

**版本**: 1.0.0
**更新日期**: 2026-04-24

---

## 目录

1. [IEKF状态估计关键函数](#iekf状态估计关键函数)
2. [地图管理关键函数](#地图管理关键函数)
3. [IMU处理关键函数](#imu处理关键函数)
4. [闭环检测关键函数](#闭环检测关键函数)
5. [点云滤波关键函数](#点云滤波关键函数)

---

## IEKF状态估计关键函数

### IekfEstimator::predict()

**文件**: `include/fast_lio2_slam/fast_lio2_core/iekf_estimator.h`

**功能**: IMU预测步，通过IMU积分传播状态和协方差

**算法原理**:
```
状态传播:
- 位置: p = p + v*dt + 0.5*(R*(acc - ba) + g)*dt^2
- 姿态: R = R * Exp((gyro - bg)*dt)
- 速度: v = v + (R*(acc - ba) + g)*dt

协方差传播:
- P = F*P*F^T + Q
```

**参数**:
- `imu`: IMU测量数据

**返回**: 无（更新内部状态）

**使用示例**:
```cpp
ImuData imu;
imu.timestamp = t;
imu.acc = Vector3d(0.1, 0.2, 9.8);
imu.gyro = Vector3d(0.01, 0.02, 0.03);

estimator.predict(imu);
```

---

### IekfEstimator::update()

**文件**: `include/fast_lio2_slam/fast_lio2_core/iekf_estimator.h`

**功能**: LiDAR更新步，通过点云配准残差更新状态

**算法原理**:
```
迭代更新流程:
1. 特征提取: 从点云提取平面点/边缘点
2. 最近邻搜索: 在iKD-Tree中查找对应点
3. 残差计算: 计算点到平面距离
4. 状态更新: x = x + K*(z - H*x)
5. 协方差更新: P = (I - K*H)*P
6. 迭代检查: 检查收敛条件
```

**参数**:
- `cloud`: 点云数据
- `kd_tree`: iKD-Tree地图指针

**返回**: `bool` - 更新是否成功

**使用示例**:
```cpp
PointCloudPtr cloud;
IKdTree::Ptr kd_tree;

bool success = estimator.update(cloud, kd_tree);
if (success) {
    State state = estimator.getState();
}
```

---

### IekfEstimator::computePointResidual()

**文件**: `include/fast_lio2_slam/fast_lio2_core/iekf_estimator.h`

**功能**: 计算点到平面/边缘残差

**算法原理**:
```
平面点残差:
- r = (p - p_near) · n_near
- 其中 p_near 是最近邻点，n_near 是法向量

边缘点残差:
- r = ||(p - p1) × (p - p2)|| / ||p1 - p2||
- 其中 p1, p2 是两个最近邻边缘点
```

**参数**:
- `point`: 当前点位置
- `nearest_points`: 最近邻点集
- `feature_type`: 特征类型（1=平面点, 2=边缘点）

**返回**: `double` - 残差值

---

## 地图管理关键函数

### MapManager::addPointCloud()

**文件**: `include/fast_lio2_slam/map_manager/map_manager.h`

**功能**: 将点云添加到地图并进行voxel滤波

**算法原理**:
```
流程:
1. 将点云转换到世界坐标系 (根据位姿)
2. 对点云进行voxel滤波降采样
3. 添加到活跃子地图或创建新子地图
4. 如果子地图点数超过阈值，创建新子地图
5. 更新地图统计数据
```

**参数**:
- `cloud`: 点云数据（原始坐标系）
- `pose`: 点云位姿（世界坐标系）
- `frame_id`: 帧ID

**使用示例**:
```cpp
PointCloudPtr cloud = ...;
SE3d pose = estimator.getState().toSE3();
int frame_id = 100;

map_manager.addPointCloud(cloud, pose, frame_id);
```

---

### MapManager::getLocalMap()

**文件**: `include/fast_lio2_slam/map_manager/map_manager.h`

**功能**: 获取指定半径内的局部地图

**算法原理**:
```
流程:
1. 找到当前位姿所在的活跃子地图
2. 计算子地图中心与当前位置的距离
3. 收集距离小于半径的所有子地图
4. 合并子地图点云
5. 返回合并后的点云
```

**参数**:
- `pose`: 当前位姿
- `radius`: 搜索半径（米）

**返回**: `PointCloudPtr` - 局部地图点云

**使用示例**:
```cpp
SE3d current_pose = estimator.getState().toSE3();
double radius = 30.0;

PointCloudPtr local_map = map_manager.getLocalMap(current_pose, radius);
```

---

### MapManager::saveMap()

**文件**: `include/fast_lio2_slam/map_manager/map_manager.h`

**功能**: 保存地图到文件

**支持的格式**:
- PCD (点云库标准格式)
- PLY (Polygon格式)
- BIN (二进制格式)

**参数**:
- `path`: 保存路径
- `format`: 文件格式 ("pcd", "ply", "bin")

**返回**: `bool` - 保存是否成功

**使用示例**:
```cpp
// 通过ROS2服务
// 或直接调用:
bool success = map_manager.saveMap("/home/user/map/output.pcd", "pcd");
```

---

### MapManager::loadMap()

**文件**: `include/fast_lio2_slam/map_manager/map_manager.h`

**功能**: 从文件加载地图

**参数**:
- `path`: 地图文件路径
- `merge`: 是否合并到现有地图

**返回**: `bool` - 加载是否成功

**使用示例**:
```cpp
// 加载并替换现有地图
bool success = map_manager.loadMap("/home/user/map/previous.pcd", false);

// 加载并合并到现有地图
bool success = map_manager.loadMap("/home/user/map/session2.pcd", true);
```

---

### MapManager::getVisualizationCloud()

**文件**: `include/fast_lio2_slam/map_manager/map_manager.h`

**功能**: 获取用于可视化的降采样点云

**参数**:
- `voxel_size`: Voxel滤波大小（默认0.5m）

**返回**: `PointCloudPtr` - 降采样后的点云

**使用示例**:
```cpp
// 用于RViz可视化
PointCloudPtr viz_cloud = map_manager.getVisualizationCloud(0.3);
// 发布到ROS2话题
pcl::toROSMsg(*viz_cloud, cloud_msg);
pub_map_->publish(cloud_msg);
```

---

## IMU处理关键函数

### ImuProcessor::predict()

**文件**: `include/fast_lio2_slam/data_preprocessor/imu_processor.h`

**功能**: IMU预积分和状态预测

**算法原理**:
```
IMU预积分:
- ΔR = ∏ Exp((gyro - bg) * dt)
- ΔV = Σ ΔR * (acc - ba) * dt
- ΔP = Σ ΔV * dt

协方差传播:
- 使用雅可比矩阵进行误差传播
```

**参数**:
- `t_start`: 开始时间
- `t_end`: 结束时间
- `state_init`: 初始状态

**返回**: `State` - 预测后的状态

---

### ImuProcessor::undistortPointCloud()

**文件**: `include/fast_lio2_slam/data_preprocessor/imu_processor.h`

**功能**: 点云运动畸变校正

**算法原理**:
```
运动畸变产生原因:
- 激光雷达扫描期间机器人运动
- 各点测量时刻不同，但初始都在同一坐标系

校正方法:
1. 计算每个点的精确时间戳
2. 通过IMU预测该时刻的位姿
3. 将点变换到扫描结束时刻的坐标系
```

**参数**:
- `cloud`: 待校正点云
- `t_start`: 扫描开始时间
- `t_end`: 扫描结束时间
- `imu_data`: 时间区间内的IMU数据

**使用示例**:
```cpp
std::vector<ImuData> imu_between = imu_buffer.getImuBetween(t0, t1);
imu_processor.undistortPointCloud(cloud, t0, t1, imu_between);
```

---

### ImuProcessor::estimateInitialBias()

**文件**: `include/fast_lio2_slam/data_preprocessor/imu_processor.h`

**功能**: 在静止状态下估计初始IMU偏置

**算法原理**:
```
静止状态假设:
- 加速度计: acc_bias = average(acc) - gravity
- 陀螺仪: gyro_bias = average(gyro)

条件:
- 机器人保持静止一段时间
- 收集足够的IMU数据
```

**参数**:
- `acc_bias`: 输出加速度计偏置
- `gyro_bias`: 输出陀螺仪偏置

**返回**: `bool` - 估计是否成功

---

## 闭环检测关键函数

### ScanContext::makeDescriptor()

**文件**: `include/fast_lio2_slam/loop_closure/scan_context.h`

**功能**: 从点云生成Scan Context描述子

**算法原理**:
```
Scan Context描述:
1. 极坐标编码:
   - 将点云投影到极坐标平面
   - 分为ring_num个环和sector_num个扇区
   - 计算每个区域的最大高度值

2. 环键值:
   - 计算每个环的平均高度
   - 用于快速搜索匹配候选
```

**参数**:
- `cloud`: 点云数据
- `timestamp`: 时间戳
- `scan_id`: 扫描ID
- `pose`: 位姿

**返回**: `ScanContextDescriptor` - 描述子

---

### ScanContext::detectLoop()

**文件**: `include/fast_lio2_slam/loop_closure/scan_context.h`

**功能**: 检测闭环约束

**算法原理**:
```
闭环检测流程:
1. 计算查询描述子与历史描述子的相似度
2. 使用环键值快速筛选候选帧
3. 排除近邻帧（避免假阳性）
4. 通过阈值判断是否为闭环
5. 估计相对旋转角度
```

**参数**:
- `query`: 查询描述子
- `constraint`: 输出闭环约束

**返回**: `bool` - 是否检测到闭环

**使用示例**:
```cpp
ScanContextDescriptor query_desc = scan_context.makeDescriptor(cloud, t, id, pose);
LoopConstraint constraint;

if (scan_context.detectLoop(query_desc, constraint)) {
    // 闭环检测成功，添加到后端优化
    backend.addLoopConstraint(constraint);
}
```

---

### ScanContext::computeSimilarity()

**文件**: `include/fast_lio2_slam/loop_closure/scan_context.h`

**功能**: 计算两个描述子之间的相似度

**算法原理**:
```
相似度计算:
- 使用余弦距离或欧氏距离
- 考虑旋转偏移（通过列偏移搜索最佳匹配）

公式:
- dist = ||desc1 - shift(desc2, angle)||
```

**参数**:
- `desc1`: 第一个描述子
- `desc2`: 第二个描述子

**返回**: `double` - 相似度分数（越小越相似）

---

## 点云滤波关键函数

### PointCloudFilter::voxelFilter()

**文件**: `include/fast_lio2_slam/data_preprocessor/point_cloud_filter.h`

**功能**: Voxel网格滤波降采样

**算法原理**:
```
流程:
1. 创建三维voxel网格
2. 计算每个voxel内所有点的重心
3. 用重心点代替voxel内所有点

效果:
- 降低点云密度
- 保持点云几何特征
```

**参数**:
- `cloud`: 输入点云
- `voxel_size`: Voxel大小（米）

**返回**: `PointCloudPtr` - 滤波后点云

---

### PointCloudFilter::removeOutliers()

**文件**: `include/fast_lio2_slam/data_preprocessor/point_cloud_filter.h`

**功能**: 统计滤波去除离群点

**算法原理**:
```
统计离群点滤波:
1. 对每个点查找k个最近邻
2. 计算平均距离
3. 假设距离分布为高斯
4. 去除距离超过阈值(mean + std_factor*std)的点
```

**参数**:
- `cloud`: 输入点云
- `k`: 最近邻数量
- `std_factor`: 标准差倍数阈值

**返回**: `PointCloudPtr` - 滤波后点云

---

### PointCloudFilter::extractFeatures()

**文件**: `include/fast_lio2_slam/data_preprocessor/point_cloud_filter.h`

**功能**: 从点云提取平面点和边缘点特征

**算法原理**:
```
特征提取:
1. 计算每个点的曲率:
   - curv = Σ ||p - p_neighbors|| / (k * ||p - p_center||)

2. 分类:
   - 平面点: 曲率小
   - 边缘点: 曲率大

3. 选取策略:
   - 每个扫描线选取固定数量特征点
   - 避免选取遮挡边界点
```

**参数**:
- `cloud`: 输入点云
- `plane_points`: 输出平面点
- `edge_points`: 输出边缘点

---

## 性能优化建议

### IEKF状态估计

1. **迭代次数**: 建议3-5次，过多会增加延迟
2. **点数限制**: 每帧选取1000-2000个特征点
3. **并行计算**: 特征提取和残差计算可并行

### 地图管理

1. **子地图大小**: 建议50m，平衡内存和查询效率
2. **Voxel滤波**: 保存时使用0.2m，可视化时使用0.5m
3. **内存限制**: 设置最大点数防止内存溢出

### IMU处理

1. **预积分窗口**: 建议200ms到500ms
2. **偏置更新**: 每次闭环后更新偏置估计
3. **静止初始化**: 确保前2-3秒保持静止

---

## 常见问题解答

### Q1: 为什么定位漂移？
可能原因：
- IMU偏置估计不准确
- 特征点数量不足
- 地图更新不及时
- 未启用闭环检测

解决方案：
- 确保静止初始化
- 增加特征点数量
- 启用闭环检测和后端优化

### Q2: 为什么地图质量差？
可能原因：
- Voxel滤波参数不当
- 点云去畸变未启用
- 子地图划分不合理

解决方案：
- 调整voxel_size参数
- 启用IMU去畸变
- 检查子地图配置

### Q3: 为什么处理延迟高？
可能原因：
- 迭代次数过多
- 地图查询效率低
- 点云未充分滤波

解决方案：
- 减少迭代次数到3-5次
- 使用局部地图而非全地图
- 添加voxel滤波降采样

---

## 参考资料

- [FAST-LIO2: Fast Direct LiDAR-Inertial Odometry](https://arxiv.org/abs/2107.06829)
- [Scan Context: Egocentric Spatial Descriptor](https://arxiv.org/abs/1807.02958)
- [iKD-Tree: Incremental KD-Tree](https://github.com/hku-mars/ikd-Tree)