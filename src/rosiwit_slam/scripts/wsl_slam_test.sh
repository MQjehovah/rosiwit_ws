#!/bin/bash
# rosiwit_slam WSL端到端建图测试脚本

echo "========== WSL建图测试 =========="
source /opt/ros/humble/setup.bash
cd /mnt/e/ai/agent/workspace/projects/rosiwit_ws
source install/setup.bash

# 创建测试目录
TEST_DIR="test_output_$(date +%Y%m%d_%H%M%S)"
mkdir -p $TEST_DIR

echo "[1] 可执行文件检查..."
ls -la install/rosiwit_slam/lib/rosiwit_slam/rosiwit_slam

echo ""
echo "[2] 节点启动测试..."
timeout 5 ros2 run rosiwit_slam rosiwit_slam --ros-args -p use_sim_time:=false > $TEST_DIR/startup.log 2>&1 || true
grep "initialized successfully" $TEST_DIR/startup.log && echo "节点启动成功" || echo "节点启动失败"

echo ""
echo "[3] 端到端建图测试..."
# 启动SLAM节点
ros2 run rosiwit_slam rosiwit_slam --ros-args -p use_sim_time:=false > $TEST_DIR/slam.log 2>&1 &
SLAM_PID=$!
sleep 2

# 启动数据生成
python3 src/rosiwit_slam/scripts/generate_simulated_data.py > $TEST_DIR/data.log 2>&1 &
DATA_PID=$!
sleep 10

# 停止节点
kill $DATA_PID $SLAM_PID 2>/dev/null || true
sleep 1

echo ""
echo "[4] 性能报告..."
grep -A10 "Performance Report" $TEST_DIR/slam.log 2>/dev/null || echo "等待性能数据..."

echo ""
echo "测试结果保存在: $TEST_DIR"
echo "========== 测试完成 =========="