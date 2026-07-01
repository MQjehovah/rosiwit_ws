#!/bin/bash
# fast_lio2_slam 编译脚本
# 
# 使用方法:
#   ./build.sh [--clean] [--test]
#
# 参数:
#   --clean: 清理之前的编译结果
#   --test:  编译后运行测试

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

echo_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

echo_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 解析参数
CLEAN_BUILD=false
RUN_TESTS=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            CLEAN_BUILD=true
            shift ;;
        --test)
            RUN_TESTS=true
            shift ;;
        *)
            echo_error "Unknown option: $1"
            exit 1 ;;
    esac
done

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo_warn "ROS2 environment not detected. Please source ROS2 first:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  OR"
    echo "  source /opt/ros/iron/setup.bash"
    exit 1
fi

echo_info "ROS2 distribution: $ROS_DISTRO"

# 检查依赖
echo_info "Checking dependencies..."

# 检查Eigen3
if ! dpkg -l | grep -q libeigen3-dev; then
    echo_warn "Eigen3 not found. Installing..."
    sudo apt-get install -y libeigen3-dev
fi

# 检查PCL
if ! dpkg -l | grep -q libpcl-dev; then
    echo_warn "PCL not found. Installing..."
    sudo apt-get install -y libpcl-dev
fi

# 检查Sophus
if ! dpkg -l | grep -q libdw-dev; then
    echo_warn "Sophus dependencies not found. Installing..."
    sudo apt-get install -y libdw-dev
fi

# 检查yaml-cpp
if ! dpkg -l | grep -q libyaml-cpp-dev; then
    echo_warn "yaml-cpp not found. Installing..."
    sudo apt-get install -y libyaml-cpp-dev
fi

# 检查ROS2依赖
echo_info "Installing ROS2 dependencies..."
rosdep install --from-paths "$PROJECT_DIR" --ignore-src -y || true

# 清理编译结果
if [ "$CLEAN_BUILD" = true ]; then
    echo_info "Cleaning build artifacts..."
    cd "$PROJECT_DIR"
    rm -rf build install log
fi

# 编译
echo_info "Building fast_lio2_slam..."
cd "$PROJECT_DIR"

# 创建workspace目录结构
mkdir -p build install log

# 使用colcon编译
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cmake-args -DCMAKE_CXX_STANDARD=17

if [ $? -eq 0 ]; then
    echo_info "Build successful!"
    
    # 显示编译结果
    echo_info "Build artifacts:"
    ls -la install/fast_lio2_slam/lib/fast_lio2_slam/ 2>/dev/null || echo "No executables found"
    
    # 运行测试
    if [ "$RUN_TESTS" = true ]; then
        echo_info "Running tests..."
        colcon test --packages-select fast_lio2_slam
        colcon test-result --verbose
    fi
    
    echo_info "To use the package, source the setup file:"
    echo "  source $PROJECT_DIR/install/setup.bash"
else
    echo_error "Build failed!"
    exit 1
fi