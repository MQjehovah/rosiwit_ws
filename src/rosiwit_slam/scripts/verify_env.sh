#!/bin/bash
# ============================================================
# FAST-LIO2 SLAM - Environment Verification Script
# ============================================================
# 用途: 验证ROS2环境和项目依赖是否正确配置
# 用法: ./scripts/verify_env.sh
# ============================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PASS=0
FAIL=0
WARN=0

check_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
    ((PASS++))
}

check_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((FAIL++))
}

check_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
    ((WARN++))
}

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  FAST-LIO2 SLAM Environment Verification${NC}"
echo -e "${BLUE}============================================${NC}"

# ==================== ROS2 Core ====================
echo -e "\n${BLUE}=== ROS2 Core ===${NC}"

# 检查ROS2安装
if [ -f "/opt/ros/humble/setup.bash" ]; then
    check_pass "ROS2 Humble installed"
else
    check_fail "ROS2 Humble not found"
fi

# 检查ros2命令
if command -v ros2 &> /dev/null; then
    check_pass "ros2 command available"
else
    check_fail "ros2 command not found"
fi

# 检查colcon
if command -v colcon &> /dev/null; then
    check_pass "colcon build tool available"
else
    check_warn "colcon not found (install: sudo apt install python3-colcon-common-extensions)"
fi

# ==================== System Dependencies ====================
echo -e "\n${BLUE}=== System Dependencies ===${NC}"

# Eigen3
if dpkg -l | grep -q libeigen3-dev; then
    check_pass "Eigen3 installed"
else
    check_fail "Eigen3 not installed (apt install libeigen3-dev)"
fi

# PCL
if dpkg -l | grep -q libpcl-dev; then
    check_pass "PCL installed"
else
    check_fail "PCL not installed (apt install libpcl-dev)"
fi

# yaml-cpp
if dpkg -l | grep -q libyaml-cpp-dev; then
    check_pass "yaml-cpp installed"
else
    check_fail "yaml-cpp not installed (apt install libyaml-cpp-dev)"
fi

# ==================== ROS2 Packages ====================
echo -e "\n${BLUE}=== ROS2 Packages ===${NC}"

ROS_PKGS=(
    "ros-humble-rclcpp"
    "ros-humble-sensor-msgs"
    "ros-humble-nav-msgs"
    "ros-humble-geometry-msgs"
    "ros-humble-tf2-ros"
    "ros-humble-std-srvs"
    "ros-humble-pcl-conversions"
    "ros-humble-pcl-ros"
    "ros-humble-rosbag2"
)

for pkg in "${ROS_PKGS[@]}"; do
    if dpkg -l | grep -q "${pkg}"; then
        check_pass "${pkg}"
    else
        check_warn "${pkg} not found"
    fi
done

# ==================== Project Build ====================
echo -e "\n${BLUE}=== Project Build ===${NC}"

PROJECT_ROOT="/home/jmq/agent/workspace/project/fast_lio2_slam"

# 检查build目录
if [ -d "${PROJECT_ROOT}/build" ]; then
    check_pass "build directory exists"
else
    check_fail "build directory not found"
fi

# 检查install目录
if [ -d "${PROJECT_ROOT}/install" ]; then
    check_pass "install directory exists"
else
    check_fail "install directory not found"
fi

# 检查setup.bash
if [ -f "${PROJECT_ROOT}/install/setup.bash" ]; then
    check_pass "install/setup.bash exists"
else
    check_fail "install/setup.bash not found"
fi

# 检查可执行节点
if [ -f "${PROJECT_ROOT}/install/fast_lio2_slam/lib/fast_lio2_slam/fast_lio2_slam" ] || \
   [ -L "${PROJECT_ROOT}/install/fast_lio2_slam/lib/fast_lio2_slam/fast_lio2_slam" ]; then
    check_pass "fast_lio2_slam executable found"
else
    check_fail "fast_lio2_slam executable not found"
fi

# ==================== Configuration Files ====================
echo -e "\n${BLUE}=== Configuration Files ===${NC}"

CONFIG_FILES=(
    "${PROJECT_ROOT}/config/default.yaml"
    "${PROJECT_ROOT}/config/ouster_os1.yaml"
    "${PROJECT_ROOT}/launch/fast_lio2.launch.py"
)

for file in "${CONFIG_FILES[@]}"; do
    if [ -f "${file}" ]; then
        check_pass "$(basename ${file})"
    else
        check_fail "$(basename ${file}) not found"
    fi
done

# ==================== Dataset ====================
echo -e "\n${BLUE}=== Dataset ===${NC}"

DATASET_PATH="${PROJECT_ROOT}/datasets/Trayectory1"
if [ -d "${DATASET_PATH}" ]; then
    check_pass "Dataset directory exists"
    
    # 检查rosbag文件
    ROsbag_FILE="${DATASET_PATH}/rosbag2_2024_05_23-15_43_25_0.db3"
    if [ -f "${ROsbag_FILE}" ]; then
        SIZE=$(du -h "${ROsbag_FILE}" | cut -f1)
        check_pass "Rosbag file found (${SIZE})"
    else
        check_warn "Rosbag file not found"
    fi
    
    # 检查baseline文件
    BASELINE_FILE="${DATASET_PATH}/LiDAR_baseline.csv"
    if [ -f "${BASELINE_FILE}" ]; then
        check_pass "Baseline file found"
    else
        check_warn "Baseline file not found"
    fi
else
    check_warn "Dataset directory not found at ${DATASET_PATH}"
fi

# ==================== Environment Variables ====================
echo -e "\n${BLUE}=== Environment Variables ===${NC}"

# ROS_DISTRO
if [ -n "${ROS_DISTRO}" ]; then
    check_pass "ROS_DISTRO=${ROS_DISTRO}"
else
    check_warn "ROS_DISTRO not set (source /opt/ros/humble/setup.bash)"
fi

# ROS_VERSION
if [ -n "${ROS_VERSION}" ]; then
    check_pass "ROS_VERSION=${ROS_VERSION}"
else
    check_warn "ROS_VERSION not set"
fi

# ==================== Summary ====================
echo -e "\n${BLUE}============================================${NC}"
echo -e "${BLUE}  Summary${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "${GREEN}Passed: ${PASS}${NC}"
echo -e "${RED}Failed: ${FAIL}${NC}"
echo -e "${YELLOW}Warnings: ${WARN}${NC}"

if [ ${FAIL} -gt 0 ]; then
    echo -e "\n${RED}Environment has issues that need to be resolved!${NC}"
    exit 1
elif [ ${WARN} -gt 0 ]; then
    echo -e "\n${YELLOW}Environment is ready with some warnings.${NC}"
    exit 0
else
    echo -e "\n${GREEN}Environment is fully configured and ready!${NC}"
    exit 0
fi