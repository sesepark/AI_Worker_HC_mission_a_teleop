#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
CONTAINER_NAME="humanoid_challenge"
IMAGE_NAME="${HUMANOID_CHALLENGE_IMAGE:-humanoid_challenge:jazzy}"

MAIN_PC_ROS_PACKAGES=(
    perception_part_detector
    monitor_ocr
    perception_2d_to_pcd
    perception_2d_to_pcd_wrist
    task_management
    mission
)

ALL_ROS_PACKAGES=(
    "${MAIN_PC_ROS_PACKAGES[@]}"
    ai_worker_manipulation
)

show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  start     Build image if needed, start container, build GPD, and build ROS packages"
    echo "  build     Build/rebuild the Docker image"
    echo "  pull      Pull HUMANOID_CHALLENGE_IMAGE from a registry"
    echo "  gpd       Build/rebuild mounted GPD source inside the container"
    echo "  colcon    Build main-PC ROS packages inside the container"
    echo "  colcon-all"
    echo "            Build all ROS packages, including ai_worker_manipulation"
    echo "  enter     Enter the running container"
    echo "  stop      Stop and remove the container"
    echo "  restart   Stop, start, and rebuild ROS packages"
    echo "  logs      Follow container logs"
    echo "  help      Show this help message"
}

compose() {
    COMPOSE_BAKE=false docker compose -f "${COMPOSE_FILE}" "$@"
}

setup_x11() {
    if [ -n "${DISPLAY:-}" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker >/dev/null 2>&1 || true
        xhost +local:root >/dev/null 2>&1 || true
    else
        echo "Warning: DISPLAY is not set. GUI tools such as rqt_image_view will not work."
    fi
}

prepare_workspace() {
    mkdir -p "${SCRIPT_DIR}/workspace"
    mkdir -p "${PROJECT_DIR}/perception_part_detector/weights"

    if [ ! -f "${PROJECT_DIR}/perception_part_detector/weights/best.pt" ]; then
        echo "Warning: perception_part_detector/weights/best.pt is missing."
        echo "         detector_node will build, but YOLO startup needs that model file."
    fi

    if [ ! -f "${PROJECT_DIR}/monitor_ocr/best.pt" ]; then
        echo "Warning: monitor_ocr/best.pt is missing."
        echo "         monitor_ocr will build, but YOLO-assisted OCR startup needs that model file."
    fi
}

build_image() {
    echo "Building humanoid_challenge Docker image..."
    compose build
}

pull_image() {
    echo "Pulling Docker image ${IMAGE_NAME}..."
    docker pull "${IMAGE_NAME}"
}

ensure_image() {
    if docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
        echo "Docker image ${IMAGE_NAME} already exists. Skipping image build."
        return
    fi

    if [ "${IMAGE_NAME}" != "humanoid_challenge:jazzy" ]; then
        pull_image
        return
    fi

    echo "Docker image ${IMAGE_NAME} is missing. Building it now..."
    build_image
}

is_running() {
    docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

start_container() {
    setup_x11
    prepare_workspace

    echo "Starting humanoid_challenge container..."
    ensure_image
    compose up -d
    build_gpd
    build_workspace_main
}

build_gpd() {
    if ! is_running; then
        echo "Container is not running. Starting it first..."
        setup_x11
        prepare_workspace
        ensure_image
        compose up -d
    fi

    echo "Building GPD from mounted source..."
    docker exec "${CONTAINER_NAME}" bash -lc "
        set -e
        export GPD_DIR=/ws/src/humanoid_challenge/gpd
        test -d \"\${GPD_DIR}\" || { echo \"Missing GPD source: \${GPD_DIR}\" >&2; exit 1; }
        cmake -S \"\${GPD_DIR}\" -B \"\${GPD_DIR}/build\" \
            -DCMAKE_BUILD_TYPE=Release \
            -DUSE_OPENVINO=OFF \
            -DUSE_CAFFE=OFF \
            -DUSE_OPENCV=OFF \
            -DBUILD_DATA_GENERATION=OFF
        cmake --build \"\${GPD_DIR}/build\" --parallel \"\$(nproc)\" --target gpd_detect_grasps
        ln -sf \"\${GPD_DIR}/build/detect_grasps\" /usr/local/bin/detect_grasps
    "
}

build_workspace() {
    local packages=("$@")

    if ! is_running; then
        echo "Container is not running. Starting it first..."
        setup_x11
        prepare_workspace
        ensure_image
        compose up -d
    fi

    echo "Building ROS packages in /ws..."
    docker exec "${CONTAINER_NAME}" bash -lc "
        set -e
        source /opt/ros/jazzy/setup.bash
        cd /ws
        colcon build --symlink-install \
            --base-paths /ws/src/humanoid_challenge \
            --packages-up-to ${packages[*]}
    "
}

build_workspace_main() {
    build_workspace "${MAIN_PC_ROS_PACKAGES[@]}"
}

build_workspace_all() {
    build_gpd
    build_workspace "${ALL_ROS_PACKAGES[@]}"
}

enter_container() {
    setup_x11

    if ! is_running; then
        echo "Error: Container is not running. Run '$0 start' first."
        exit 1
    fi

    docker exec -it "${CONTAINER_NAME}" bash -lc "
        cd /ws
        source /opt/ros/jazzy/setup.bash
        if [ -f /ws/install/setup.bash ]; then
            source /ws/install/setup.bash
        fi
        exec bash
    "
}

stop_container() {
    if ! docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
        echo "Container is not present."
        return
    fi

    echo "Stopping humanoid_challenge container..."
    compose down
}

case "${1:-help}" in
    start)
        start_container
        ;;
    build)
        prepare_workspace
        build_image
        ;;
    pull)
        pull_image
        ;;
    gpd)
        build_gpd
        ;;
    colcon)
        prepare_workspace
        build_workspace_main
        ;;
    colcon-all)
        prepare_workspace
        build_workspace_all
        ;;
    enter)
        enter_container
        ;;
    stop)
        stop_container
        ;;
    restart)
        stop_container
        start_container
        ;;
    logs)
        compose logs -f
        ;;
    help|-h|--help)
        show_help
        ;;
    *)
        echo "Error: Unknown command: $1"
        show_help
        exit 1
        ;;
esac
