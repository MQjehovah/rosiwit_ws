#!/bin/bash
# 建图功能测试脚本 - 在WSL环境中运行SLAM节点进行建图验证

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== rosiwit_slam 建图功能测试 ===${NC}"

# 环境变量
WS_DIR="/mnt/e/ai/agent/workspace/projects/rosiwit_ws"
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

# 创建测试目录
TEST_DIR="${WS_DIR}/test_output"
mkdir -p "${TEST_DIR}"
mkdir -p "${TEST_DIR}/maps"

echo -e "${YELLOW}测试目录: ${TEST_DIR}${NC}"

# 检查可执行文件
if [ ! -f "${WS_DIR}/install/rosiwit_slam/lib/rosiwit_slam/rosiwit_slam" ]; then
    echo -e "${RED}错误: 可执行文件不存在${NC}"
    echo "请先运行: colcon build"
    exit 1
fi

echo -e "${GREEN}可执行文件已确认${NC}"

# 生成模拟数据
echo -e "${YELLOW}启动模拟数据生成器...${NC}"
python3 "${WS_DIR}/src/rosiwit_slam/scripts/generate_simulated_data.py" &
SIM_PID=$!

# 等待数据生成器启动
sleep 2

# 检查话题是否发布
echo -e "${YELLOW}检查ROS2话题...${NC}"
ros2 topic list | grep -E "livox|odom" || echo "话题未发现，但继续测试"

# 启动SLAM节点
echo -e "${YELLOW}启动SLAM节点 (运行10秒)...${NC}"
ros2 run rosiwit_slam rosiwit_slam &
SLAM_PID=$!

# 等待一段时间让系统处理数据
sleep 10

# 检查输出话题
echo -e "${YELLOW}检查输出话题...${NC}"
ros2 topic hz /odom_estimated &
HZ_PID=$!
sleep 3
kill $HZ_PID 2>/dev/null || true

ros2 topic hz /cloud_map &
HZ_PID2=$!
sleep 3
kill $HZ_PID2 2>/dev/null || true

# 停止节点
echo -e "${YELLOW}停止节点...${NC}"
kill $SLAM_PID 2>/dev/null || true
kill $SIM_PID 2>/dev/null || true

# 等待进程结束
wait $SLAM_PID 2>/dev/null || true
wait $SIM_PID 2>/dev/null || true

# 检查地图文件
echo -e "${YELLOW}检查地图输出...${NC}"
if [ -f "${TEST_DIR}/maps/map.pcd" ]; then
    FILE_SIZE=$(stat -c%s "${TEST_DIR}/maps/map.pcd")
    echo -e "${GREEN}地图文件已生成: ${TEST_DIR}/maps/map.pcd (${FILE_SIZE} bytes)${NC}"
else
    echo -e "${YELLOW}地图文件未在指定目录找到${NC}"
    echo "查找可能的位置..."
    find "${WS_DIR}" -name "*.pcd" -mmin -15 2>/dev/null || echo "未找到最近的PCD文件"
fi

echo -e "${GREEN}=== 测试完成 ===${NC}"