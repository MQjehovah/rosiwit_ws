#!/bin/bash
# 覆盖路径规划功能验证脚本

set -e

# 设置 ROS2 环境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "============================================"
echo "ROS2 覆盖路径规划功能验证"
echo "============================================"

# 1. 启动 coverage_planner_node (后台运行)
echo "[步骤1] 启动 coverage_planner_node..."
ros2 run ros2_coverage_planner coverage_planner_node &
NODE_PID=$!
echo "节点 PID: $NODE_PID"

# 等待节点启动
sleep 3

# 2. 检查节点是否运行
echo "[步骤2] 检查节点状态..."
ros2 node list | grep coverage_planner_node || echo "节点未找到"
ros2 service list | grep plan_coverage || echo "服务未找到"

# 3. 运行地图发布脚本
echo "[步骤3] 发布测试地图..."
python3 ~/ros2_ws/src/ros2_coverage_planner/test/test_map_publisher.py &
MAP_PID=$!

# 等待地图发布
sleep 2

# 4. 调用触发服务执行覆盖规划
echo "[步骤4] 触发覆盖路径规划..."
ros2 service call /plan_coverage std_srvs/srv/Trigger "{}" 2>&1

# 等待规划完成
sleep 5

# 5. 检查路径话题
echo "[步骤5] 检查规划路径..."
ros2 topic echo /coverage_path --once 2>&1 || echo "路径未发布"

# 6. 检查节点日志
echo "[步骤6] 节点日志输出..."
ros2 topic echo /rosout --once 2>&1

# 清理
echo "============================================"
echo "清理进程..."
kill $NODE_PID 2>/dev/null || true
kill $MAP_PID 2>/dev/null || true

echo "============================================"
echo "验证完成"
echo "============================================"