#!/bin/bash
# ============================================================
# rosiwit_simulator - Docker Build Script (ROS2 Humble)
# ============================================================
# Usage:
#   ./build.sh              - Build runtime image
#   ./build.sh --devel      - Build development image
#   ./build.sh --no-cache   - Rebuild without cache
#   ./build.sh --push       - Build and push to registry
# ============================================================

set -euo pipefail

# ============================================================
# Configuration
# ============================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${DOCKER_IMAGE:-rosiwit-simulator}"
IMAGE_TAG="${DOCKER_TAG:-2.0.0}"
REGISTRY="${DOCKER_REGISTRY:-}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ============================================================
# Helper Functions
# ============================================================
log_info()  { echo -e "${BLUE}[INFO]${NC}  $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# ============================================================
# Parse Arguments
# ============================================================
BUILD_TARGET="runtime"
BUILD_CACHE="--no-cache"
PUSH_IMAGE=false

for arg in "$@"; do
    case $arg in
        --devel)
            BUILD_TARGET="builder"
            ;;
        --cache)
            BUILD_CACHE=""
            ;;
        --push)
            PUSH_IMAGE=true
            ;;
        --tag=*)
            IMAGE_TAG="${arg#*=}"
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --devel        Build development image (with build tools)"
            echo "  --cache        Use Docker build cache"
            echo "  --push         Push image to registry after build"
            echo "  --tag=TAG      Specify image tag (default: 2.0.0)"
            echo "  -h, --help     Show this help"
            exit 0
            ;;
        *)
            log_error "Unknown argument: $arg"
            exit 1
            ;;
    esac
done

# ============================================================
# Pre-build Checks
# ============================================================
log_info "rosiwit_simulator Docker Build (ROS2 Humble)"
log_info "============================================="

# Check Docker
if ! command -v docker &> /dev/null; then
    log_error "Docker not found. Please install Docker first."
    exit 1
fi

# Check Dockerfile
if [ ! -f "${SCRIPT_DIR}/Dockerfile" ]; then
    log_error "Dockerfile not found at ${SCRIPT_DIR}/Dockerfile"
    exit 1
fi

# Check source code
if [ ! -f "${PROJECT_DIR}/CMakeLists.txt" ]; then
    log_error "CMakeLists.txt not found at ${PROJECT_DIR}"
    exit 1
fi

if [ ! -f "${PROJECT_DIR}/package.xml" ]; then
    log_error "package.xml not found at ${PROJECT_DIR}"
    exit 1
fi

log_ok "Pre-build checks passed"
log_info "  Source:     ${PROJECT_DIR}"
log_info "  Dockerfile: ${SCRIPT_DIR}/Dockerfile"
log_info "  Target:     ${BUILD_TARGET}"
log_info "  Tag:        ${IMAGE_NAME}:${IMAGE_TAG}"

# ============================================================
# Build
# ============================================================
log_info "Building Docker image..."

BUILD_ARGS=(
    -f "${SCRIPT_DIR}/Dockerfile"
    --target "${BUILD_TARGET}"
    -t "${IMAGE_NAME}:${IMAGE_TAG}"
    -t "${IMAGE_NAME}:latest"
    --label "ros.distro=humble"
    --label "ros.version=2"
    --label "git.ref=$(cd "${PROJECT_DIR}" && git rev-parse --short HEAD 2>/dev/null || echo 'unknown')"
    --label "build.date=$(date -u +'%Y-%m-%dT%H:%M:%SZ')"
)

if [ -n "${BUILD_CACHE}" ]; then
    BUILD_ARGS+=("${BUILD_CACHE}")
fi

# Build context is the project root (parent of docker/)
docker build "${BUILD_ARGS[@]}" "${PROJECT_DIR}"

BUILD_EXIT=$?
if [ $BUILD_EXIT -ne 0 ]; then
    log_error "Docker build failed with exit code ${BUILD_EXIT}"
    exit $BUILD_EXIT
fi

log_ok "Build completed: ${IMAGE_NAME}:${IMAGE_TAG}"

# Show image size
IMAGE_SIZE=$(docker images "${IMAGE_NAME}:${IMAGE_TAG}" --format "{{.Size}}")
log_info "Image size: ${IMAGE_SIZE}"

# ============================================================
# Push (optional)
# ============================================================
if [ "${PUSH_IMAGE}" = true ]; then
    if [ -z "${REGISTRY}" ]; then
        log_warn "No registry configured. Set DOCKER_REGISTRY environment variable."
        log_warn "Skipping push."
        exit 0
    fi

    log_info "Pushing to registry: ${REGISTRY}"
    docker tag "${IMAGE_NAME}:${IMAGE_TAG}" "${REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}"
    docker push "${REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}"

    if [ "${IMAGE_TAG}" != "latest" ]; then
        docker tag "${IMAGE_NAME}:${IMAGE_TAG}" "${REGISTRY}/${IMAGE_NAME}:latest"
        docker push "${REGISTRY}/${IMAGE_NAME}:latest"
    fi

    log_ok "Push completed: ${REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}"
fi

echo ""
log_ok "Done! Run with: docker run --rm -it ${IMAGE_NAME}:${IMAGE_TAG}"
