#!/bin/bash
# rosiwit_slam 真实数据建图运行脚本
# 使用真实 ROS bag 数据包运行 SLAM 建图流程
# 适配 NTU-VIRAL 数据集 (Ouster OS1-16)

set -e

# ==================== 颜色输出 ====================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ==================== 默认参数 ====================
WS_DIR="${WS_DIR:-/mnt/e/ai/agent/workspace/projects/rosiwit_ws}"
DATA_DIR="${DATA_DIR:-~/rosiwit_data}"
CONFIG_FILE="${CONFIG_FILE:-ouster_os1_16.yaml}"
BAG_FILE="${BAG_FILE:-}"
OUTPUT_DIR="${OUTPUT_DIR:-./real_data_maps}"
PLAY_RATE="${PLAY_RATE:-1.0}"
MAX_DURATION="${MAX_DURATION:-300}"  # 最大运行时长（秒）
USE_CLOCK="${USE_CLOCK:-true}"
LOG_LEVEL="${LOG_LEVEL:-INFO}"

# ==================== Topic 重映射 ====================
# NTU-VIRAL 数据集的 topic 映射
REMAP_LIDAR="${REMAP_LIDAR:-/os1_cloud_node/points:=/ouster/points}"
REMAP_IMU="${REMAP_IMU:-/os1_cloud_node/imu:=/ouster/imu}"

# ==================== 函数定义 ====================

print_header() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  rosiwit_slam 真实数据建图测试${NC}"
    echo -e "${GREEN}========================================${NC}"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

check_ros2_env() {
    print_info "检查 ROS2 环境..."
    
    if [ -z "$ROS_DISTRO" ]; then
        print_warning "ROS2 环境未加载，尝试自动加载..."
        source /opt/ros/humble/setup.bash || {
            print_error "无法加载 ROS2 Humble 环境"
            return 1
        }
    fi
    
    print_success "ROS2 环境: $ROS_DISTRO"
    
    # 加载工作空间
    if [ -d "${WS_DIR}/install" ]; then
        source "${WS_DIR}/install/setup.bash" || {
            print_warning "无法加载工作空间环境"
        }
        print_success "工作空间环境已加载"
    fi
    
    return 0
}

check_executable() {
    print_info "检查 SLAM 可执行文件..."
    
    SLAM_EXEC="${WS_DIR}/install/rosiwit_slam/lib/rosiwit_slam/rosiwit_slam"
    
    if [ ! -f "$SLAM_EXEC" ]; then
        print_error "SLAM 可执行文件不存在: $SLAM_EXEC"
        print_info "请先编译: cd ${WS_DIR} && colcon build --packages-select rosiwit_slam"
        return 1
    fi
    
    print_success "可执行文件已确认"
    return 0
}

check_bag_file() {
    print_info "检查 bag 文件..."
    
    if [ -z "$BAG_FILE" ]; then
        print_warning "未指定 bag 文件，尝试自动查找..."
        
        # 查找可能的 bag 文件
        if [ -d "$DATA_DIR" ]; then
            BAG_FILE=$(find "$DATA_DIR" -name "*.bag" -o -name "*.db3" | head -n 1)
        fi
        
        if [ -z "$BAG_FILE" ]; then
            print_error "未找到 bag 文件"
            print_info "请指定 bag 文件路径: BAG_FILE=~/rosiwit_data/eee_01.bag $0"
            print_info "或下载 NTU-VIRAL 数据集: python3 scripts/fetch_ntu_viral.py --sequence eee_01"
            return 1
        fi
        
        print_info "自动选择 bag 文件: $BAG_FILE"
    fi
    
    if [ ! -e "$BAG_FILE" ]; then
        print_error "bag 文件不存在: $BAG_FILE"
        return 1
    fi
    
    # 检查文件类型
    if [[ "$BAG_FILE" == *.db3 ]] || [ -d "$BAG_FILE" ]; then
        print_success "ROS2 bag 文件: $BAG_FILE"
        BAG_TYPE="ros2"
    elif [[ "$BAG_FILE" == *.bag ]]; then
        print_warning "ROS1 bag 文件，需要转换"
        print_info "运行转换: python3 scripts/convert_bag.py --input $BAG_FILE"
        BAG_TYPE="ros1"
        return 1
    else
        print_error "未知的 bag 文件格式"
        return 1
    fi
    
    return 0
}

check_config_file() {
    print_info "检查配置文件..."
    
    CONFIG_PATH="${WS_DIR}/src/rosiwit_slam/config/${CONFIG_FILE}"
    
    if [ ! -f "$CONFIG_PATH" ]; then
        print_warning "配置文件不存在: $CONFIG_PATH"
        print_info "使用默认配置: default.yaml"
        CONFIG_FILE="default.yaml"
        CONFIG_PATH="${WS_DIR}/src/rosiwit_slam/config/default.yaml"
        
        if [ ! -f "$CONFIG_PATH" ]; then
            print_error "默认配置文件也不存在"
            return 1
        fi
    fi
    
    print_success "配置文件: $CONFIG_FILE"
    return 0
}

check_disk_space() {
    print_info "检查磁盘空间..."
    
    OUTPUT_DIR_PATH=$(realpath -m "$OUTPUT_DIR" 2>/dev/null || echo "$OUTPUT_DIR")
    
    # 获取可用空间（GB）
    if command -v df &> /dev/null; then
        AVAILABLE_GB=$(df "$OUTPUT_DIR_PATH" 2>/dev/null | awk 'NR==2 {print int($4/1024/1024)}')
        
        if [ -n "$AVAILABLE_GB" ] && [ "$AVAILABLE_GB" -lt 5 ]; then
            print_warning "磁盘空间不足 ($AVAILABLE_GB GB)，建议清理或更换输出目录"
        else
            print_success "可用磁盘空间: $AVAILABLE_GB GB"
        fi
    fi
    
    return 0
}

create_output_dir() {
    print_info "创建输出目录..."
    
    mkdir -p "${OUTPUT_DIR}/maps"
    mkdir -p "${OUTPUT_DIR}/logs"
    mkdir -p "${OUTPUT_DIR}/stats"
    mkdir -p "${OUTPUT_DIR}/trajectory"
    
    print_success "输出目录: $OUTPUT_DIR"
}

get_bag_info() {
    print_info "获取 bag 文件信息..."
    
    if [ "$BAG_TYPE" == "ros2" ]; then
        ros2 bag info "$BAG_FILE" 2>/dev/null || {
            print_warning "无法获取 bag 信息"
        }
    fi
}

start_slam_node() {
    print_info "启动 SLAM 节点..."
    
    # 构建参数
    USE_SIM_TIME="true"  # 使用 bag 时间
    
    ros2 run rosiwit_slam rosiwit_slam \
        --ros-args \
        -p config_file:="${WS_DIR}/src/rosiwit_slam/config/${CONFIG_FILE}" \
        -p use_sim_time:=${USE_SIM_TIME} \
        -p log_level:=${LOG_LEVEL} \
        > "${OUTPUT_DIR}/logs/slam_output.log" 2>&1 &
    
    SLAM_PID=$!
    
    print_success "SLAM 节点已启动 (PID: $SLAM_PID)"
    
    # 等待节点启动
    sleep 3
    
    # 检查节点是否运行
    if ! ps -p $SLAM_PID > /dev/null 2>&1; then
        print_error "SLAM 节点启动失败"
        print_info "查看日志: cat ${OUTPUT_DIR}/logs/slam_output.log"
        return 1
    fi
    
    return 0
}

play_bag() {
    print_info "播放 bag 文件..."
    
    # 构建播放命令
    PLAY_CMD="ros2 bag play ${BAG_FILE}"
    
    # 添加时钟参数
    if [ "$USE_CLOCK" == "true" ]; then
        PLAY_CMD="${PLAY_CMD} --clock"
    fi
    
    # 添加播放速率
    PLAY_CMD="${PLAY_CMD} --rate ${PLAY_RATE}"
    
    # 添加 topic 重映射
    PLAY_CMD="${PLAY_CMD} --remap ${REMAP_LIDAR} --remap ${REMAP_IMU}"
    
    print_info "播放命令: $PLAY_CMD"
    
    # 记录开始时间
    START_TIME=$(date +%s)
    
    # 执行播放（后台）
    eval $PLAY_CMD > "${OUTPUT_DIR}/logs/bag_play.log" 2>&1 &
    PLAY_PID=$!
    
    print_success "bag 播放已启动 (PID: $PLAY_PID)"
    
    # 监控播放进度
    while ps -p $PLAY_PID > /dev/null 2>&1; do
        CURRENT_TIME=$(date +%s)
        ELAPSED=$((CURRENT_TIME - START_TIME))
        
        if [ $ELAPSED -gt $MAX_DURATION ]; then
            print_warning "达到最大运行时长 ($MAX_DURATION 秒)，停止播放"
            kill $PLAY_PID 2>/dev/null || true
            break
        fi
        
        # 每10秒输出进度
        if [ $((ELAPSED % 10)) -eq 0 ]; then
            print_info "运行时长: ${ELAPSED} 秒"
        fi
        
        sleep 1
    done
    
    print_success "bag 播放完成"
}

monitor_slam() {
    print_info "监控 SLAM 状态..."
    
    # 检查话题
    sleep 2
    
    ros2 topic hz /odom_estimated --window 10 > "${OUTPUT_DIR}/stats/odom_hz.txt" 2>&1 &
    HZ_PID=$!
    
    ros2 topic hz /cloud_map --window 10 > "${OUTPUT_DIR}/stats/map_hz.txt" 2>&1 &
    MAP_HZ_PID=$!
    
    # 等待一段时间收集数据
    sleep 10
    
    # 停止话题监控
    kill $HZ_PID 2>/dev/null || true
    kill $MAP_HZ_PID 2>/dev/null || true
    
    # 输出统计信息
    if [ -f "${OUTPUT_DIR}/stats/odom_hz.txt" ]; then
        cat "${OUTPUT_DIR}/stats/odom_hz.txt" | grep "average rate" || true
    fi
}

save_results() {
    print_info "保存建图结果..."
    
    # 等待 SLAM 处理完成
    sleep 5
    
    # 调用地图保存服务
    ros2 service call /save_pcd std_msgs/String "{data: '${OUTPUT_DIR}/maps/map.pcd'}" \
        > "${OUTPUT_DIR}/logs/save_map.log" 2>&1 || {
        print_warning "地图保存服务调用失败"
    }
    
    # 检查地图文件
    if [ -f "${OUTPUT_DIR}/maps/map.pcd" ]; then
        MAP_SIZE=$(stat -c%s "${OUTPUT_DIR}/maps/map.pcd" 2>/dev/null || stat -f%z "${OUTPUT_DIR}/maps/map.pcd" 2>/dev/null || echo "unknown")
        print_success "地图文件已保存: ${OUTPUT_DIR}/maps/map.pcd (${MAP_SIZE} bytes)"
    else
        print_warning "地图文件未生成"
        print_info "检查日志: cat ${OUTPUT_DIR}/logs/save_map.log"
    fi
    
    # 复制日志
    if [ -f "${OUTPUT_DIR}/logs/slam_output.log" ]; then
        print_success "SLAM 日志已保存"
    fi
}

stop_processes() {
    print_info "停止所有进程..."
    
    # 停止 SLAM 节点
    if [ -n "$SLAM_PID" ] && ps -p $SLAM_PID > /dev/null 2>&1; then
        kill $SLAM_PID 2>/dev/null || true
        wait $SLAM_PID 2>/dev/null || true
    fi
    
    # 停止播放进程
    if [ -n "$PLAY_PID" ] && ps -p $PLAY_PID > /dev/null 2>&1; then
        kill $PLAY_PID 2>/dev/null || true
        wait $PLAY_PID 2>/dev/null || true
    fi
    
    print_success "所有进程已停止"
}

generate_stats_report() {
    print_info "生成统计报告..."
    
    python3 "${WS_DIR}/src/rosiwit_slam/scripts/generate_stats_report.py" \
        --input "${OUTPUT_DIR}/logs" \
        --output "${OUTPUT_DIR}/stats/stats_report.txt" \
        --format text || {
        print_warning "统计报告生成失败"
        print_info "请确保 generate_stats_report.py 已存在"
    }
    
    if [ -f "${OUTPUT_DIR}/stats/stats_report.txt" ]; then
        print_success "统计报告: ${OUTPUT_DIR}/stats/stats_report.txt"
    fi
}

print_final_summary() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  建图测试完成${NC}"
    echo -e "${GREEN}========================================${NC}"
    
    echo -e "${BLUE}输出文件:${NC}"
    echo "  - 地图文件: ${OUTPUT_DIR}/maps/"
    echo "  - 轨迹文件: ${OUTPUT_DIR}/trajectory/"
    echo "  - 日志文件: ${OUTPUT_DIR}/logs/"
    echo "  - 统计报告: ${OUTPUT_DIR}/stats/"
    
    echo -e "${BLUE}查看命令:${NC}"
    echo "  - 查看日志: cat ${OUTPUT_DIR}/logs/slam_output.log"
    echo "  - 查看报告: cat ${OUTPUT_DIR}/stats/stats_report.txt"
    echo "  - 查看地图: pcl_viewer ${OUTPUT_DIR}/maps/map.pcd"
    
    echo -e "${GREEN}========================================${NC}"
}

cleanup_on_error() {
    print_error "发生错误，清理进程..."
    stop_processes
    exit 1
}

# ==================== 主流程 ====================

main() {
    # 设置错误处理
    trap cleanup_on_error ERR
    
    print_header
    
    # 1. 检查环境
    check_ros2_env || exit 1
    check_executable || exit 1
    check_bag_file || exit 1
    check_config_file || exit 1
    check_disk_space
    
    # 2. 获取 bag 信息
    get_bag_info
    
    # 3. 创建输出目录
    create_output_dir
    
    # 4. 启动 SLAM 节点
    start_slam_node || exit 1
    
    # 5. 播放 bag 文件
    play_bag
    
    # 6. 监控 SLAM 状态
    monitor_slam
    
    # 7. 保存结果
    save_results
    
    # 8. 停止进程
    stop_processes
    
    # 9. 生成统计报告
    generate_stats_report
    
    # 10. 输出最终总结
    print_final_summary
    
    exit 0
}

# ==================== 参数解析 ====================

usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --bag FILE          bag 文件路径 (必需)"
    echo "  --config FILE       配置文件名 (默认: ouster_os1_16.yaml)"
    echo "  --output DIR        输出目录 (默认: ./real_data_maps)"
    echo "  --rate RATE         播放速率 (默认: 1.0)"
    echo "  --duration SECONDS  最大运行时长 (默认: 300)"
    echo "  --help              显示帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 --bag ~/rosiwit_data/eee_01_ros2"
    echo "  $0 --bag ~/rosiwit_data/sbs_01 --config ouster_os1_16.yaml --rate 0.5"
    echo ""
    echo "环境变量:"
    echo "  WS_DIR              工作空间目录"
    echo "  DATA_DIR            数据目录"
    echo "  BAG_FILE            bag 文件路径"
    echo "  CONFIG_FILE         配置文件名"
    echo "  OUTPUT_DIR          输出目录"
    echo "  PLAY_RATE           播放速率"
    echo "  MAX_DURATION        最大运行时长"
}

# 解析命令行参数
while [ $# -gt 0 ]; do
    case "$1" in
        --bag)
            BAG_FILE="$2"
            shift 2
            ;;
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --rate)
            PLAY_RATE="$2"
            shift 2
            ;;
        --duration)
            MAX_DURATION="$2"
            shift 2
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            print_error "未知参数: $1"
            usage
            exit 1
            ;;
    esac
done

# 执行主流程
main