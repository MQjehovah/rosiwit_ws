#!/bin/bash
# ============================================================
# rosiwit_slam - Docker Run Script
# Supports: SLAM standalone, SLAM+Simulator, Development mode
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --slam          Run SLAM node only (standalone)"
    echo "  --sim           Run Simulator + SLAM stack"
    echo "  --demo          Run full demo (sim + SLAM + auto-motion)"
    echo "  --devel         Run development environment"
    echo "  --stop          Stop all containers"
    echo "  --logs          Follow container logs"
    echo "  -h, --help      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 --slam               # Run SLAM standalone"
    echo "  $0 --sim                # Run Simulator + SLAM"
    echo "  $0 --demo               # Run full demo"
    echo "  $0 --devel              # Start dev environment"
    echo "  $0 --stop               # Stop all"
}

ACTION=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --slam)
            ACTION="slam"
            shift
            ;;
        --sim)
            ACTION="sim"
            shift
            ;;
        --demo)
            ACTION="demo"
            shift
            ;;
        --devel)
            ACTION="devel"
            shift
            ;;
        --stop)
            ACTION="stop"
            shift
            ;;
        --logs)
            ACTION="logs"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Default action
if [ -z "$ACTION" ]; then
    ACTION="slam"
fi

case $ACTION in
    slam)
        echo "🚀 Starting SLAM node (standalone)..."
        docker compose -f "$COMPOSE_FILE" up -d slam
        echo "✅ SLAM node started. Use '$0 --logs' to follow."
        ;;
    sim)
        echo "🚀 Starting Simulator + SLAM stack..."
        docker compose -f "$COMPOSE_FILE" up -d simulator slam
        echo "✅ Simulator + SLAM started. Use '$0 --logs' to follow."
        ;;
    demo)
        echo "🚀 Starting full demo (Simulator + SLAM + Auto-motion)..."
        docker compose -f "$COMPOSE_FILE" up -d demo
        echo "✅ Full demo started. Use '$0 --logs' to follow."
        ;;
    devel)
        echo "🔧 Starting development environment..."
        docker compose -f "$COMPOSE_FILE" run --rm slam-devel
        ;;
    stop)
        echo "🛑 Stopping all containers..."
        docker compose -f "$COMPOSE_FILE" down
        echo "✅ All containers stopped."
        ;;
    logs)
        docker compose -f "$COMPOSE_FILE" logs -f
        ;;
esac
