#!/bin/bash
# ============================================================
# rosiwit_simulator - Docker Run Script (ROS2 Humble)
# ============================================================
# Usage:
#   ./run.sh                    - Run 3D LiDAR simulation
#   ./run.sh --slam             - Run full stack (simulator + SLAM)
#   ./run.sh --devel            - Run development environment
#   ./run.sh --2d               - Run 2D LiDAR simulation
#   ./run.sh --stop             - Stop all containers
# ============================================================

set -euo pipefail

# ============================================================
# Configuration
# ============================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
IMAGE_NAME="rosiwit-simulator"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()  { echo -e "${BLUE}[INFO]${NC}  $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# ============================================================
# Parse Arguments
# ============================================================
MODE="simulator"
STOP=false

for arg in "$@"; do
    case $arg in
        --slam)      MODE="full" ;;
        --devel)     MODE="devel" ;;
        --2d)        MODE="2d" ;;
        --stop)      STOP=true ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --slam     Run full stack (simulator + SLAM, ROS2 native)"
            echo "  --devel    Run development environment with source mount"
            echo "  --2d       Run 2D LiDAR simulation"
            echo "  --stop     Stop all rosiwit containers"
            echo "  -h, --help Show this help"
            exit 0
            ;;
        *)
            log_error "Unknown argument: $arg"
            exit 1
            ;;
    esac
done

# ============================================================
# Stop Mode
# ============================================================
if [ "$STOP" = true ]; then
    log_info "Stopping all rosiwit containers..."
    docker-compose -f "${COMPOSE_FILE}" down --remove-orphans
    log_ok "All containers stopped"
    exit 0
fi

# ============================================================
# Pre-flight Checks
# ============================================================

# Check Docker
if ! command -v docker &> /dev/null; then
    log_error "Docker not found. Please install Docker first."
    exit 1
fi

# Check X11 display
if [ -z "${DISPLAY:-}" ]; then
    log_warn "DISPLAY not set. GUI applications (Gazebo, RViz) may not work."
    log_warn "Set DISPLAY=:0 or run with --headless"
fi

# Allow X11 connections from Docker
if [ -n "${DISPLAY:-}" ]; then
    xhost +local:docker 2>/dev/null || log_warn "xhost command failed (may need manual X auth)"
fi

# ============================================================
# Run
# ============================================================
case $MODE in
    simulator)
        log_info "Starting 3D LiDAR simulation (ROS2 Humble)..."
        log_info "  Simulator only mode"
        docker-compose -f "${COMPOSE_FILE}" up -d simulator
        log_ok "Simulator started"
        log_info "View logs: docker-compose -f ${COMPOSE_FILE} logs -f simulator"
        ;;

    full)
        log_info "Starting full SLAM stack (ROS2 Humble)..."
        log_info "  Simulator + SLAM (no ros1_bridge needed)"
        docker-compose -f "${COMPOSE_FILE}" up -d simulator slam slam-viz
        log_ok "Full stack started"
        log_info "View logs: docker-compose -f ${COMPOSE_FILE} logs -f"
        ;;

    devel)
        log_info "Starting development environment (ROS2 Humble)..."
        log_info "  Source mounted for live editing"
        docker-compose -f "${COMPOSE_FILE}" up -d simulator-devel
        log_ok "Development environment started"
        log_info "Enter container: docker exec -it rosiwit-simulator-devel bash"
        log_info "Build inside:    colcon build --packages-select simulator"
        log_info "Run inside:      ros2 launch simulator simulator_gazebo_3d.launch.py"
        ;;

    2d)
        log_info "Starting 2D LiDAR simulation (ROS2 Humble)..."
        docker run --rm -it \
            --name rosiwit-simulator-2d \
            --network host \
            --ipc host \
            -e DISPLAY=${DISPLAY:-:0} \
            -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} \
            -e GAZEBO_MODEL_PATH=/colcon_ws/src/rosiwit_simulator/models \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v /dev/shm:/dev/shm \
            ${IMAGE_NAME}:2.0.0 \
            /bin/bash -c "
                source /opt/ros/humble/setup.bash &&
                source /colcon_ws/install/setup.bash &&
                ros2 launch simulator simulator_gazebo.launch.py
            "
        ;;
esac

echo ""
log_ok "Done!"
