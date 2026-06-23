#!/usr/bin/env bash
# Mission A 통합 시연 재현 스크립트 (nav=stub) — integration/mission-a
#
# 컨테이너 humanoid_challenge 안에서 실행:
#   cd ~/AI_Worker_HC/humanoid_challenge && ./docker/container.sh enter
#   /ws/src/humanoid_challenge/scripts/run_integration_demo.sh [stage]
# stage: build | s0 | s1 | g5 | all (기본 all)
#
# 모든 런타임은 nav_mode:=stub (또는 sim). nav_mode:=service 는 범위 밖(콜드 디스커버리 미해결).
set -u
STAGE="${1:-all}"
DOMAIN="${ROS_DOMAIN_ID:-90}"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash 2>/dev/null || true

cleanup() {
  pkill -9 -f "[l]ib/mission/mission_a" 2>/dev/null || true
  pkill -9 -f "[l]ib/mission/mock_"      2>/dev/null || true
  pkill -9 -f "[t]ray_manage_node"       2>/dev/null || true
  pkill -9 -f "[t]ask_management"        2>/dev/null || true
  sleep 2
}

do_build() {
  echo "### build (--packages-up-to mission)"
  ( cd /ws && colcon build --symlink-install --packages-up-to mission ) || exit 1
  source /ws/install/setup.bash
}

# 단계0: sim 무회귀 (SimDriver, 신규 액션/서비스 우회) — 기대 DONE 적재 3
do_s0() {
  echo "### 단계0 (sim) — 기대: DONE 적재 3"
  cleanup
  timeout 25 ros2 launch mission mission_a.launch.py \
      sim_mode:=true use_mocks:=false use_task_list_service:=true
  cleanup
}

# 단계1: nav=stub + mock 3종 — 기대 5사이클 DONE 적재 5
do_s1() {
  echo "### 단계1 (nav=stub, mock 3종) — 기대: DONE 적재 5"
  cleanup
  timeout 45 ros2 launch mission mission_a.launch.py    # nav_mode 기본 stub
  cleanup
}

# G5: 실 perception(tray_manage_node) task_list + mock manip/wrist + nav=stub — 기대 DONE 적재 5
do_g5() {
  echo "### G5 통합 시연 (실 perception task_list + mock) — 기대: DONE 적재 5 (task_list=실노드)"
  cleanup
  timeout 55 ros2 launch mission integration_demo.launch.py   # nav=stub, 실 task_list
  cleanup
}

# 주입 시험(선택): C2 드롭/C3 무효/플랩 (오선언 0 확인)
do_inject() {
  echo "### 주입 시험: C2 드롭(release 전) — 기대: 적재 0, RECOVERY"
  cleanup
  timeout 30 ros2 launch mission mission_a.launch.py \
      use_place_pose_check:=true place_pose_flap:=true \
      drop_during_move:=true drop_after_attach_sec:=0.5
  cleanup
  echo "### 주입 시험: C3 무효 — 기대: 적재 0(릴리스 안함)"
  cleanup
  timeout 28 ros2 launch mission mission_a.launch.py \
      use_place_pose_check:=true place_pose_invalid:=true
  cleanup
}

case "$STAGE" in
  build)   do_build ;;
  s0)      do_s0 ;;
  s1)      do_s1 ;;
  g5)      do_g5 ;;
  inject)  do_inject ;;
  all)     do_s0; do_s1; do_g5 ;;
  *) echo "usage: $0 [build|s0|s1|g5|inject|all]"; exit 1 ;;
esac
echo "### done."
