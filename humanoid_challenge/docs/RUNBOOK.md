# Mission A 실행 런북 (RUNBOOK)

> 흩어진 실행/검증/셋업/트러블슈팅 절차를 한 곳에서 찾도록 모은 **허브**. 복사-실행용 명령 정본.
> 구조·상태머신 이해는 [MISSION_A_STATE_MACHINE_MANUAL.md](MISSION_A_STATE_MACHINE_MANUAL.md), 계약은 [INTERFACES.md](INTERFACES.md).
> **실로봇 전체 사이클 절차의 정본은 [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md) §6.2**(근본원인 FIX 포함). 본 런북 §3은 그 운용 요약이다.
> 인용 경로는 `humanoid_challenge/` 루트 기준.

---

## 1. 빌드 & 환경

### 1.1 빌드
```bash
# humanoid_challenge 컨테이너
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install --packages-up-to mission
colcon build --packages-select perception manipulation
source /ws/install/setup.bash
```
근거: `scripts/run_integration_demo.sh:30-34`, [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md):64. 신규 머신 최초 구축은 [MANIPULATION_SETUP_NEW_MACHINE.md](MANIPULATION_SETUP_NEW_MACHINE.md)(TRAC-IK 설치 포함), perception 모델/venv는 [PERCEPTION_LOCAL_SETUP.md](PERCEPTION_LOCAL_SETUP.md).

### 1.2 환경변수
| 구성 | DOMAIN | LOCALHOST_ONLY | DISCOVERY_RANGE |
|---|---|---|---|
| 헤드리스 단일 PC | `90`(기본) | `1` | — |
| 실 통합(크로스-PC) | `30` | `0` | `SUBNET` |
```bash
# 실 통합 (양쪽 PC 동일)
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```
근거: `scripts/run_integration_demo.sh:12-15`, `mission/launch/mission_a_real.launch.py:13`.

---

## 2. 헤드리스 / Mock 실행 (로봇 없이 전 구간·게이트 검증)

원클릭 스크립트(`scripts/run_integration_demo.sh`):
```bash
/ws/src/humanoid_challenge/scripts/run_integration_demo.sh [build|s0|s1|g5|inject|all]
```

| 단계 | 명령(직접 실행 시) | 기대 | 근거 |
|---|---|---|---|
| **S0** sim 무회귀 | `ros2 launch mission mission_a.launch.py sim_mode:=true use_mocks:=false use_task_list_service:=true` | DONE 적재 3 (~25s) | `run_integration_demo.sh:36-43` |
| **S1** mock 통합(nav=stub) | `ros2 launch mission mission_a.launch.py` | DONE 적재 5, RECOVERY 0 (~45s) | `:45-51` |
| **G5** 실 task_list + mock | `ros2 launch mission integration_demo.launch.py` | DONE 적재 5 (task_list=실노드) (~55s) | `:53-59` |
| **주입** C2 드롭 | `ros2 launch mission mission_a.launch.py use_place_pose_check:=true place_pose_flap:=true drop_during_move:=true drop_after_attach_sec:=0.5` | 적재 0, RECOVERY | `:62-68` |
| **주입** C3 무효 | `ros2 launch mission mission_a.launch.py use_place_pose_check:=true place_pose_invalid:=true` | 적재 0(릴리스 안함) | `:69-73` |

> nav 서비스 단계화 검증: `ros2 launch mission mission_a.launch.py nav_mode:=service` + `mock_navigation_a`(Service). (`mission_a.launch.py:9-10`)
> 단계별 시나리오·기대값 상세는 [MISSION_A_DEMO_VERIFICATION.md](MISSION_A_DEMO_VERIFICATION.md)(§2 로컬 mock 데모) 참조.

---

## 3. 실로봇 운용 절차 (Phase 2 Real)

> **정본**: [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md) §6.2 표(A~E) + 근본원인(colorizer/TF/FIX). 로봇 관점 체크리스트는 [REAL_ROBOT_TEST_PROCEDURE.md](REAL_ROBOT_TEST_PROCEDURE.md). 아래는 운용 요약.

**필수 사전설정 3종**
1. **depth colorizer off** → `colorizer.enable1:=false colorizer.enable2:=false`(`16UC1` cross-PC 스트림). ([RESULT](MISSION_A_PHASE2_RESULT.md):81-83)
2. **static TF 유지** → `publish_camera_tf:=true`(기본; `camera_r_link→camera_right_link` identity 브리지). (`perception/launch/perception_live.launch.py:51-52`)
3. **카메라 단일소유** → bringup이 `camera_right`를 올리므로 별도 카메라 launch 금지. ([RESULT](MISSION_A_PHASE2_RESULT.md):87)

| STEP | 위치 | 명령 | 확인 |
|---|---|---|---|
| **A** bringup+MoveIt | 로봇 PC | `ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py colorizer.enable1:=false colorizer.enable2:=false tf_publish_rate1:=10.0 tf_publish_rate2:=10.0` ; `ros2 launch ffw_moveit_config moveit.launch.py` | `move_group`/`/joint_states` 보임; depth `--field encoding` → `16UC1` |
| **B** manip 서버 | 메인/ai_worker | `ros2 launch manipulation mission_a_manip.launch.py` | `ready (real MoveIt)`; `ros2 action list \| grep move_to_scan_pose`; `/manipulator_state` → `IDLE` |
| **C** perception+FSM | 메인 PC | `ros2 launch mission mission_a_real.launch.py mock_monitor_ocr:=true use_place_pose_check:=true` | `/detections`·`target_one_pose` 유입; FSM `INIT→A1_MONITOR→A2_SCAN_POSE→A2_SCAN→A3_PICK` |
| **D** 사이클 | (자동) | 부품 5개를 scan FOV(트레이)에 배치 | C2 래치→A3_PLACE C3 게이트→차감, `VERIFY→DONE 적재 N` |
| **E** 다사이클 | (반복) | manip 서버 재기동 없이 `mission_a_real` 연속 실행 | 매회 풀사이클 DONE |

> ⚠️ **STEP B부터 실 팔이 동작**. e-stop 준비. ([RESULT](MISSION_A_PHASE2_RESULT.md):99)

**nav 서비스 실연동**(옵션): 로봇 PC `ros2 launch mission move_base_lateral.launch.py` → 메인 PC `... mission_a_real.launch.py nav_mode:=service`. 콜드 첫 호출은 무이동(`distance_mm:=0`)으로 검증 가능(`mission/launch/move_base_lateral.launch.py:12-14`).

---

## 4. 검증 체크리스트

| 대상 | 문서 | 내용 |
|---|---|---|
| 통합 스모크 | [04_INTEGRATION_VERIFICATION_GUIDE.md](04_INTEGRATION_VERIFICATION_GUIDE.md) | Build / Executables / Minimal Topic Pipeline / Robot-Free Smoke / Conflict Checks |
| perception 노드별 | [PERCEPTION_NODE_VERIFICATION.md](PERCEPTION_NODE_VERIFICATION.md) | monitor_ocr / detector / tray_manage / wrist_planner 단위 확인 |
| 데모·시나리오 | [MISSION_A_DEMO_VERIFICATION.md](MISSION_A_DEMO_VERIFICATION.md) | 두 환경 정의, mock 데모, 실로봇 데모, 미구현 맵, SDR §8 매핑 |

빠른 토픽 점검:
```bash
ros2 topic echo /manipulator_state --once          # IDLE
ros2 topic echo /perception/wrist/target_one_pose --once   # base_link 3D
ros2 topic echo /perception/place_pose_valid --once        # {"valid": true}
ros2 action list | grep move_to_scan_pose
```

---

## 5. 셋업 & 트러블슈팅

| 주제 | 문서 |
|---|---|
| 신규 머신 구축(클론·컨테이너·TRAC-IK·빌드) | [MANIPULATION_SETUP_NEW_MACHINE.md](MANIPULATION_SETUP_NEW_MACHINE.md) |
| perception 로컬 셋업(모델·venv·실행순서) | [PERCEPTION_LOCAL_SETUP.md](PERCEPTION_LOCAL_SETUP.md) |
| 크로스 컨테이너/머신 통신 문제 | [MANIPULATION_ROS2_COMMUNICATION_ISSUES.md](MANIPULATION_ROS2_COMMUNICATION_ISSUES.md) |

자주 겪는 이슈(요약 — 상세는 위 문서):
- **manip↔mock 충돌**: `mission_a_manipulation_server`와 `mock_manipulation_a` 동시 기동 금지(둘 다 scan action/attach 제공). (`manipulation/launch/mission_a_manip.launch.py:7-8`)
- **depth 0프레임**: colorizer가 켜져 있으면 cross-PC 미전송. `colorizer.enable*:=false` 확인.
- **wrist target 미발행**: static TF 끊김. `publish_camera_tf:=true` 또는 bringup TF 확인.
- **nav 콜드 디스커버리**: 첫 `MoveBaseLateral` 호출 매칭 지연 → `nav_service_wait_sec`(기본 10s) 내 재시도, 상태 timeout=30s로 유계(`mission/mission/mission_a.py:150-154, 424-431`).

---

*튜닝(RGB-D sync_slop 등)·근본원인(FIX-1~6) 상세는 [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md) §6.2.*
