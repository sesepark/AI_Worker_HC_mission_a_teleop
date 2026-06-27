#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
CONTAINER_NAME="humanoid_challenge_teleop_final"
IMAGE_NAME="${HUMANOID_CHALLENGE_IMAGE:-shpark1104/humanoid_challenge:jazzy}"

show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  start     Pull the fixed image if needed and start the container"
    echo "  pull      Pull the fixed Docker image"
    echo "  enter     Enter the running container"
    echo "  stop      Stop and remove the container"
    echo "  restart   Stop and start the container"
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
    mkdir -p "${PROJECT_DIR}/perception/model"

    if [ ! -f "${PROJECT_DIR}/perception/model/part_detector_best.pt" ]; then
        echo "Warning: perception/model/part_detector_best.pt is missing."
        echo "         detector_node will build, but YOLO startup needs that model file."
    fi

    if [ ! -f "${PROJECT_DIR}/perception/model/monitor_ocr_best.pt" ]; then
        echo "Warning: perception/model/monitor_ocr_best.pt is missing."
        echo "         monitor_ocr_node will build, but YOLO-assisted OCR startup needs that model file."
    fi

    if [ ! -f "${PROJECT_DIR}/perception/model/tray_occupancy_best.pt" ]; then
        echo "Warning: perception/model/tray_occupancy_best.pt is missing."
        echo "         tray_manage_node will build, but tray YOLO startup needs that model file."
    fi
}

pull_image() {
    echo "Pulling Docker image ${IMAGE_NAME}..."
    docker pull "${IMAGE_NAME}"
}

ensure_image() {
    if docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
        echo "Docker image ${IMAGE_NAME} already exists."
        return
    fi

    pull_image
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
    pull)
        pull_image
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
