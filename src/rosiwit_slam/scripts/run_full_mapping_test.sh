#!/bin/bash
# 完整建图功能测试脚本 - 收集建图结果和性能数据
# 用法: 在WSL Ubuntu-22.04中运行

set +e  # 不因错误退出，保证清理

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  rosiwit_slam 完整建图功能测试${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""

# 记录开始时间
START_TIME=$(date +%s)
TEST_TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# 环境变量
WS_DIR="/mnt/e/ai/agent/workspace/projects/rosiwit_ws"
TEST_DIR="${WS_DIR}/test_output_${TEST_TIMESTAMP}"
LOG_DIR="${TEST_DIR}/logs"
MAP_DIR="${TEST_DIR}/maps"

# 创建测试目录
mkdir -p "${TEST_DIR}"
mkdir -p "${LOG_DIR}"
mkdir -p "${MAP_DIR}"

echo -e "${YELLOW}测试目录: ${TEST_DIR}${NC}"
echo -e "${YELLOW}测试时间: $(date)${NC}"
echo ""

# 环境设置
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

# ============================================
# 测试1: 可执行文件检查
# ============================================
echo -e "${BLUE}[测试1] 检查可执行文件...${NC}"
EXE_PATH="${WS_DIR}/install/rosiwit_slam/lib/rosiwit_slam/rosiwit_slam"
if [ -f "${EXE_PATH}" ]; then
    FILE_SIZE=$(stat -c%s "${EXE_PATH}")
    FILE_SIZE_MB=$(echo "scale=2; ${FILE_SIZE}/1048576" | bc)
    echo -e "${GREEN}  ✓ 可执行文件存在${NC}"
    echo -e "    路径: ${EXE_PATH}"
    echo -e "    大小: ${FILE_SIZE} bytes (${FILE_SIZE_MB} MB)"
    echo "可执行文件: ${FILE_SIZE} bytes" > "${TEST_DIR}/exec_info.txt"
else
    echo -e "${RED}  ✗ 可执行文件不存在${NC}"
    echo "可执行文件不存在" > "${TEST_DIR}/error.txt"
    exit 1
fi
echo ""

# ============================================
# 测试2: 节点启动测试
# ============================================
echo -e "${BLUE}[测试2] 节点启动测试...${NC}"
timeout 5 ros2 run rosiwit_slam rosiwit_slam --ros-args -p use_sim_time:=false 2>&1 | tee "${LOG_DIR}/node_startup.log" &
NODE_PID=$!
sleep 3

# 检查初始化日志
if grep -q "initialized successfully" "${LOG_DIR}/node_startup.log" 2>/dev/null; then
    echo -e "${GREEN}  ✓ 节点初始化成功${NC}"
    grep "initialized" "${LOG_DIR}/node_startup.log" | head -10 | while read line; do
        echo -e "    ${line}"
    done
elif grep -q "initialized" "${LOG_DIR}/node_startup.log" 2>/dev/null; then
    echo -e "${GREEN}  ✓ 节点已初始化${NC}"
    grep "initialized" "${LOG_DIR}/node_startup.log" | head -5 | while read line; do
        echo -e "    ${line}"
    done
else
    echo -e "${YELLOW}  ⚠ 节点启动日志检查${NC}"
    head -20 "${LOG_DIR}/node_startup.log" 2>/dev/null
fi

kill $NODE_PID 2>/dev/null
wait $NODE_PID 2>/dev/null
echo ""

# ============================================
# 测试3: 模拟数据生成和建图测试
# ============================================
echo -e "${BLUE}[测试3] 模拟数据建图测试 (运行15秒)...${NC}"

# 启动模拟数据生成器
echo -e "${YELLOW}  启动模拟数据生成器...${NC}"
python3 "${WS_DIR}/src/rosiwit_slam/scripts/generate_simulated_data.py" > "${LOG_DIR}/sim_data.log" 2>&1 &
SIM_PID=$!
sleep 2

# 检查数据生成器是否启动
if ps -p $SIM_PID > /dev/null 2>&1; then
    echo -e "${GREEN}  ✓ 模拟数据生成器启动成功 (PID: ${SIM_PID})${NC}"
else
    echo -e "${RED}  ✗ 模拟数据生成器启动失败${NC}"
    cat "${LOG_DIR}/sim_data.log" 2>/dev/null
fi

# 检查话题是否发布
echo -e "${YELLOW}  检查ROS2话题...${NC}"
sleep 1
TOPICS=$(ros2 topic list 2>/dev/null)
echo -e "  可用话题:"
echo "$TOPICS" | grep -E "livox|odom|cloud" | while read topic; do
    echo -e "    ${topic}"
done

# 启动SLAM节点
echo -e "${YELLOW}  启动SLAM节点...${NC}"
ros2 run rosiwit_slam rosiwit_slam --ros-args -p use_sim_time:=false > "${LOG_DIR}/slam_run.log" 2>&1 &
SLAM_PID=$!
sleep 2

# 记录内存使用
echo -e "${YELLOW}  记录性能指标...${NC}"
echo "时间戳,内存(KB),CPU(%)" > "${TEST_DIR}/performance.csv"

for i in {1..10}; do
    if ps -p $SLAM_PID > /dev/null 2>&1; then
        MEM=$(ps -o rss= -p $SLAM_PID 2>/dev/null | tr -d ' ')
        CPU=$(ps -o %cpu= -p $SLAM_PID 2>/dev/null | tr -d ' ')
        TIMESTAMP=$(date +%H:%M:%S)
        echo "${TIMESTAMP},${MEM},${CPU}" >> "${TEST_DIR}/performance.csv"
        echo -e "    [${TIMESTAMP}] 内存: ${MEM}KB, CPU: ${CPU}%"
    fi
    sleep 1
done

# 检查输出话题频率
echo -e "${YELLOW}  检查输出话题频率...${NC}"
timeout 3 ros2 topic hz /odom_estimated 2>&1 | tee "${LOG_DIR}/odom_hz.log" &
HZ_PID1=$!
sleep 3
kill $HZ_PID1 2>/dev/null
wait $HZ_PID1 2>/dev/null

timeout 3 ros2 topic hz /cloud_map 2>&1 | tee "${LOG_DIR}/cloud_hz.log" &
HZ_PID2=$!
sleep 3
kill $HZ_PID2 2>/dev/null
wait $HZ_PID2 2>/dev/null

# 继续运行一段时间
echo -e "${YELLOW}  继续处理数据...${NC}"
sleep 5

# ============================================
# 测试4: 检查输出和清理
# ============================================
echo ""
echo -e "${BLUE}[测试4] 检查输出结果...${NC}"

# 统计处理帧数
if [ -f "${LOG_DIR}/slam_run.log" ]; then
    FRAME_COUNT=$(grep -c "frame" "${LOG_DIR}/slam_run.log" 2>/dev/null || echo "0")
    POINT_COUNT=$(grep -c "point" "${LOG_DIR}/slam_run.log" 2>/dev/null || echo "0")
    echo -e "  日志中的关键统计:"
    echo -e "    帧相关日志: ${FRAME_COUNT} 条"
    echo -e "    点云相关日志: ${POINT_COUNT} 条"
    
    # 提取关键日志
    echo -e "  SLAM节点关键日志:"
    grep -E "received|processed|published|error|warning" "${LOG_DIR}/slam_run.log" 2>/dev/null | head -10 | while read line; do
        echo -e "    ${line}"
    done
fi

# 停止节点
echo -e "${YELLOW}  停止节点...${NC}"
kill $SLAM_PID 2>/dev/null
kill $SIM_PID 2>/dev/null
wait $SLAM_PID 2>/dev/null
wait $SIM_PID 2>/dev/null

# 检查地图文件
echo -e "${YELLOW}  检查地图文件...${NC}"
PCD_FILES=$(find "${WS_DIR}" -name "*.pcd" -mmin -5 2>/dev/null)
if [ -n "$PCD_FILES" ]; then
    echo -e "${GREEN}  ✓ 找到PCD地图文件:${NC}"
    echo "$PCD_FILES" | while read pcd; do
        SIZE=$(stat -c%s "$pcd" 2>/dev/null)
        echo -e "    ${pcd} (${SIZE} bytes)"
        # 复制到测试目录
        cp "$pcd" "${MAP_DIR}/" 2>/dev/null
    done
else
    echo -e "${YELLOW}  ⚠ 未找到PCD地图文件${NC}"
    echo -e "  说明: 节点可能需要接收到有效数据后才会生成地图"
fi

# ============================================
# 测试结果汇总
# ============================================
END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  测试完成${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo -e "${YELLOW}测试结果摘要:${NC}"
echo -e "  测试时长: ${DURATION} 秒"
echo -e "  测试目录: ${TEST_DIR}"
echo -e "  日志目录: ${LOG_DIR}"

# 汇总文件
echo ""
echo -e "${YELLOW}生成的文件:${NC}"
ls -la "${TEST_DIR}/" 2>/dev/null | grep -v "^total" | while read line; do
    echo -e "  ${line}"
done

# 性能数据汇总
if [ -f "${TEST_DIR}/performance.csv" ]; then
    echo ""
    echo -e "${YELLOW}性能数据:${NC}"
    cat "${TEST_DIR}/performance.csv"
fi

echo ""
echo -e "${GREEN}测试日志已保存到: ${LOG_DIR}${NC}"