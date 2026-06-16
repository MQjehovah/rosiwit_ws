#!/bin/bash
# fast_lio2_slam 独立测试编译脚本 (无ROS2依赖)
#
# 使用方法:
#   ./test_build.sh [--clean] [--run]
#
# 参数:
#   --clean: 清理之前的编译结果
#   --run:   编译后运行测试

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/test/build"

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
        --run)
            RUN_TESTS=true
            shift ;;
        *)
            echo_error "Unknown option: $1"
            exit 1 ;;
    esac
done

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

# 检查GTest
if ! dpkg -l | grep -q libgtest-dev; then
    echo_warn "GTest not found. Installing..."
    sudo apt-get install -y libgtest-dev googletest
fi

# 清理编译结果
if [ "$CLEAN_BUILD" = true ]; then
    echo_info "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# 创建编译目录
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# 运行CMake
echo_info "Running CMake configuration..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17

# 编译
echo_info "Building test targets..."
make -j$(nproc)

# 运行测试
if [ "$RUN_TESTS" = true ]; then
    echo_info "Running tests..."
    ctest --output-on-failure
fi

echo_info "Build completed successfully!"
echo_info "Test executables are in: $BUILD_DIR"

# 显示测试可执行文件
ls -la *.cpp 2>/dev/null || ls -la test_* 2>/dev/null || true