#!/bin/bash
# ============================================================
# rosiwit_slam - Docker Build Script
# Supports: production, development, and cache modes
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Defaults
IMAGE_NAME="rosiwit-slam"
TAG="${TAG:-humble-2.0}"
DEVEL=false
CACHE=""
PUSH=false
VERBOSE=""

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --devel       Build development image (Dockerfile.devel)"
    echo "  --cache       Use Docker build cache (no --no-cache)"
    echo "  --push        Push image to registry after build"
    echo "  --tag TAG     Set image tag (default: humble-2.0)"
    echo "  --verbose     Verbose Docker build output"
    echo "  -h, --help    Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                          # Build production image"
    echo "  $0 --devel                  # Build development image"
    echo "  $0 --tag v2.0.0             # Build with custom tag"
    echo "  $0 --devel --cache          # Build dev image with cache"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --devel)
            DEVEL=true
            TAG="devel-2.0"
            shift
            ;;
        --cache)
            CACHE=""
            shift
            ;;
        --push)
            PUSH=true
            shift
            ;;
        --tag)
            TAG="$2"
            shift 2
            ;;
        --verbose)
            VERBOSE="--progress=plain"
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

# Select Dockerfile
if [ "$DEVEL" = true ]; then
    DOCKERFILE="docker/Dockerfile.devel"
    IMAGE_NAME="rosiwit-slam"
else
    DOCKERFILE="docker/Dockerfile"
    IMAGE_NAME="rosiwit-slam"
fi

FULL_IMAGE="${IMAGE_NAME}:${TAG}"

echo "============================================"
echo "Building: ${FULL_IMAGE}"
echo "Dockerfile: ${DOCKERFILE}"
echo "Context: ${PROJECT_DIR}"
echo "============================================"

# Build
BUILD_ARGS=(
    -f "${DOCKERFILE}"
    -t "${FULL_IMAGE}"
)

# Disable cache by default for reproducible builds
if [ -z "${CACHE}" ]; then
    BUILD_ARGS+=(--no-cache)
fi

if [ -n "${VERBOSE}" ]; then
    BUILD_ARGS+=("${VERBOSE}")
fi

BUILD_ARGS+=( "${PROJECT_DIR}" )

docker build "${BUILD_ARGS[@]}"

echo ""
echo "✅ Build successful: ${FULL_IMAGE}"
echo ""

# Push if requested
if [ "$PUSH" = true ]; then
    echo "Pushing ${FULL_IMAGE}..."
    docker push "${FULL_IMAGE}"
    echo "✅ Pushed: ${FULL_IMAGE}"
fi
