#!/bin/bash

# Docker build script for the deploy base image

set -e

# Configuration
IMAGE_NAME="nvgear/ros-2"
TAG="${1:-latest}"
PLATFORM="${2:-$(dpkg --print-architecture)}"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DOCKERFILE="$SCRIPT_DIR/Dockerfile.deploy.base"

case "$PLATFORM" in
    amd64|linux/amd64)
        DOCKER_PLATFORM="linux/amd64"
        ;;
    arm64|linux/arm64|aarch64)
        DOCKER_PLATFORM="linux/arm64"
        ;;
    *)
        echo "Unsupported platform: $PLATFORM"
        echo "Use one of: amd64, arm64, linux/amd64, linux/arm64"
        exit 1
        ;;
esac

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Building Docker image: ${IMAGE_NAME}:${TAG}${NC}"
echo -e "${YELLOW}Target platform: ${DOCKER_PLATFORM}${NC}"

if [ "$DOCKER_PLATFORM" = "linux/arm64" ]; then
    echo -e "${YELLOW}Using native docker build for local ARM64 builds.${NC}"
    docker build --network host --file "${DOCKERFILE}"         --tag "${IMAGE_NAME}:${TAG}"         .
else
    echo -e "${YELLOW}Setting up buildx builder...${NC}"
    docker buildx use multiarch-builder 2>/dev/null || {
        echo -e "${YELLOW}Creating multiarch builder...${NC}"
        docker buildx create --name multiarch-builder --use --bootstrap
    }

    echo -e "${YELLOW}Supported platforms:${NC}"
    docker buildx inspect --bootstrap | grep Platforms

    echo -e "${GREEN}Starting buildx build...${NC}"
    docker buildx build --network host --platform "${DOCKER_PLATFORM}"         --file "${DOCKERFILE}"         --tag "${IMAGE_NAME}:${TAG}"         --load         .
fi

echo -e "${GREEN}Build completed successfully!${NC}"
echo -e "${GREEN}Image: ${IMAGE_NAME}:${TAG}${NC}"
echo -e "${GREEN}Platform: ${DOCKER_PLATFORM}${NC}"
