#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
#  perception 패키지 monitor OCR 배포 스크립트 (어느 컴퓨터에서든 실행 가능)
#
#  사전 조건:
#    - 로봇과 같은 네트워크에 연결되어 있어야 함
#    - docker 컨테이너 'ai_worker' 가 로봇에서 실행 중이어야 함
#
#  사용법:
#    ROBOT=robotis@<robot-host-or-ip> ROBOT_PW=<password> bash monitor_ocr_deploy.sh
#    bash monitor_ocr_deploy.sh robotis@<로봇IP>         # IP 직접 지정
#    bash monitor_ocr_deploy.sh robotis@<로봇IP> <비번>  # 주소 + 비번 지정
# ══════════════════════════════════════════════════════════════════════════════

set -e

# ── 설정 ──────────────────────────────────────────────────────────────────────
ROBOT="${1:-${ROBOT:-robotis@robot.local}}"
ROBOT_PW="${2:-${ROBOT_PW:-}}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
PERCEPTION_DIR="$PROJECT_DIR/perception"

if [ -z "$ROBOT_PW" ]; then
    echo "[오류] ROBOT_PW가 비어 있습니다."
    echo "  사용 예: ROBOT=robotis@<robot-host-or-ip> ROBOT_PW=<password> bash monitor_ocr_deploy.sh"
    echo "  또는:    bash monitor_ocr_deploy.sh robotis@<robot-host-or-ip> <password>"
    exit 1
fi

echo "══════════════════════════════════════════"
echo "  Monitor OCR 배포"
echo "  로봇 : $ROBOT"
echo "  소스 : $SCRIPT_DIR"
echo "══════════════════════════════════════════"
echo ""

# ── 0. sshpass 확인 ───────────────────────────────────────────────────────────
if ! command -v sshpass &>/dev/null; then
    echo "[0/4] sshpass 설치 중..."
    sudo apt-get install -y sshpass -q
    echo "      완료"
fi

SSH="sshpass -p $ROBOT_PW ssh -o StrictHostKeyChecking=no $ROBOT"
SCP="sshpass -p $ROBOT_PW scp -o StrictHostKeyChecking=no"

# ── 연결 테스트 ───────────────────────────────────────────────────────────────
echo "[*] 로봇 연결 확인 중..."
if ! $SSH "echo ok" &>/dev/null; then
    echo ""
    echo "[오류] 로봇에 연결할 수 없습니다."
    echo "  - WiFi가 로봇과 같은 네트워크인지 확인하세요"
    echo "  - 주소가 맞는지 확인: $ROBOT"
    echo "  - 직접 IP 지정: bash monitor_ocr_deploy.sh robotis@<IP주소> <비밀번호>"
    exit 1
fi
echo "      연결 성공"
echo ""

# ── 1. 코드 복사 ──────────────────────────────────────────────────────────────
echo "[1/3] 코드 복사 중..."
$SSH "mkdir -p ~/ai_worker"
$SCP -r "$PERCEPTION_DIR" "${ROBOT}:~/ai_worker/"
$SCP -r "$PROJECT_DIR/mission_interfaces" "${ROBOT}:~/ai_worker/"
echo "      완료"

# ── 2. ROS2 워크스페이스로 복사 + 빌드 ───────────────────────────────────────
echo "[2/3] ROS2 패키지 빌드 중..."
$SSH "docker exec ai_worker bash -c \"
    cp -r ~/ai_worker/perception /root/ros2_ws/src/ 2>/dev/null || true
    cp -r ~/ai_worker/mission_interfaces /root/ros2_ws/src/ 2>/dev/null || true
    source /opt/ros/jazzy/setup.bash &&
    cd /root/ros2_ws &&
    colcon build --packages-up-to perception 2>&1 | tail -5
\""
echo "      완료"

# ── 3. 완료 메시지 ────────────────────────────────────────────────────────────
echo "[3/3] 배포 완료!"
echo ""
echo "══════════════════════════════════════════"
echo "  다음 단계: OCR 노드 실행"
echo ""
echo "  1) 로봇 SSH 접속:"
echo "     ssh $ROBOT  (비번: $ROBOT_PW)"
echo ""
echo "  2) bringup 실행 (터미널 1):"
echo "     docker exec -it ai_worker bash"
echo "     ros2 launch ffw_bringup ffw_sg2_ai.launch.py"
echo ""
echo "  3) OCR 노드 실행 (터미널 2):"
echo "     bash ~/ai_worker/perception/tools/run_monitor_ocr.sh"
echo ""
echo "  4) 결과 확인 (노트북 또는 터미널 3):"
echo "     ros2 service call /mission_a/task_list mission_interfaces/srv/GetTaskList \"{timeout_sec: 20.0, frame_count: 3}\""
echo "══════════════════════════════════════════"
