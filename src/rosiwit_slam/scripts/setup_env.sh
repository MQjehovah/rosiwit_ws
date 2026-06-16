#!/bin/bash
# ============================================================
# FAST-LIO2 SLAM - Environment Setup Script
# ============================================================
# 用途: 配置ROS2运行环境，确保所有依赖就绪
# 用法: source scripts/setup_env.sh
# ============================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目路径
PROJECT_ROOT="/home/jmq/agent/workspace/project/fast_lio2_slam"

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  FAST-LIO2 SLAM Environment Setup${NC}"
echo -e "${BLUE}============================================${NC}"

# ==================== Step 1: Source ROS2 Humble ====================
echo -e "\n${YELLOW}[1/4] Sourcing ROS2 Humble...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓ ROS2 Humble sourced${NC}"
else
    echo -e "${RED}✗ ROS2 Humble not found at /opt/ros/humble/setup.bash${NC}"
    echo "  Please install ROS2 Humble first:"
    echo "  https://docs.ros.org/en/humble/Installation.html"
    return 1 2>/dev/null || exit 1
fi

# ==================== Step 2: Source Project Workspace ====================
echo -e "\n${YELLOW}[2/4] Sourcing Project Workspace...${NC}"
if [ -f "${PROJECT_ROOT}/install/setup.bash" ]; then
    source "${PROJECT_ROOT}/install/setup.bash"
    echo -e "${GREEN}✓ Project workspace sourced${NC}"
else
    echo -e "${RED}✗ Project not built. Run 'colcon build' first.${NC}"
    return 1 2>/dev/null || exit 1
fi

# ==================== Step 3: Set Environment Variables ====================
echo -e "\n${YELLOW}[3/4] Setting Environment Variables...${NC}"

# 项目路径
export FAST_LIO2_ROOT="${PROJECT_ROOT}"
export FAST_LIO2_CONFIG="${PROJECT_ROOT}/config"

# 仿真时间模式（用于rosbag回放）
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

# 日志级别
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1

echo -e "${GREEN}✓ Environment variables set:${NC}"
echo "  FAST_LIO2_ROOT=${FAST_LIO2_ROOT}"
echo "  FAST_LIO2_CONFIG=${FAST_LIO2_CONFIG}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# ==================== Step 4: Verify Environment ====================
echo -e "\n${YELLOW}[4/4] Verifying Environment...${NC}"

# 检查ROS2命令
if command -v ros2 &> /dev/null; then
    ROS2_VERSION=$(ros2 --version 2>/dev/null || echo "unknown")
    echo -e "${GREEN}✓ ros2 command available${NC}"
fi

# 检查节点可执行文件
if [ -f "${PROJECT_ROOT}/install/fast_lio2_slam/lib/fast_lio2_slam/fast_lio2_slam" ]; then
    echo -e "${GREEN}✓ fast_lio2_slam node found${NC}"
else
    echo -e "${YELLOW}! fast_lio2_slam node not found, may need rebuild${NC}"
fi

# 检查配置文件
if [ -f "${PROJECT_ROOT}/config/default.yaml" ]; then
    echo -e "${GREEN}✓ default.yaml config found${NC}"
fi

# 检查数据集
DATASET_PATH="${PROJECT_ROOT}/datasets/Trayectory1"
if [ -d "${DATASET_PATH}" ]; then
    echo -e "${GREEN}✓ Dataset directory found: ${DATASET_PATH}${NC}"
else
    echo -e "${YELLOW}! Dataset directory not found at ${DATASET_PATH}${NC}"
    echo "  Please ensure datasets are placed in the correct location"
fi

# ==================== Done ====================
echo -e "\n${GREEN}============================================${NC}"
echo -e "${GREEN}  Environment Setup Complete!${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "Quick Start:"
echo "  1. Run mapping test:"
echo "     cd \${FAST_LIO2_ROOT} && ./run_mapping_test.sh"
echo ""
echo "  2. Manual launch:"
echo "     ros2 launch fast_lio2_slam fast_lio2.launch.py"
echo ""
echo "  3. Play rosbag:"
echo "     ros2 bag play datasets/Trayectory1/rosbag2_2024_05_23-15_43_25_0.db3 --clock"
echo ""