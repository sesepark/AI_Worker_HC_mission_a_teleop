# Mission A 3-브랜치 통합 결과 (integration/mission-a)

> system(`feature/mission-a-v2.3`) · perception(`feature/mission-a-perception-new`) ·
> manipulation(`feature/mission-a`) 3팀 작업을 `integration/mission-a` 로 통합 + nav=stub mock 시연.
> 게이트 G0~G6 절차 수행. **원본 3브랜치 무변경.** 모든 런타임 검증은 nav=stub(또는 sim).

## 0. 요약
- 3브랜치 머지 완료(conflict 0, 그래스프 래치 보존), `colcon build --packages-up-to mission` **RC=0**.
- 인터페이스 계약 정합: `/perception/task_list` 를 **`mission_interfaces/GetTaskList.Response`** 타입드 토픽으로 통일(FSM·실 perception·mock 일치).
- 무회귀(G4): 단계0(sim) DONE 적재 3, 단계1(nav=stub) **DONE 적재 5 ×3 안정**.
- 통합 시연(G5): **실 perception `tray_manage_node` 의 task_list 를 FSM 이 수신** → 5사이클 DONE 적재 5(manip/wrist=mock, nav=stub).

## 1. 머지 내역
베이스 = `origin/feature/mission-a`(manipulation). 순서: perception-new → v2.3.
| 커밋 | 내용 |
|---|---|
| `d69e1a6` | merge(perception-new) — **충돌 0**(Automatic merge) |
| `6e89d33` | merge(v2.3) — 충돌 1건(mission_a.py) 해소 |
| `5ed1716` | fix(G3): mock_perception_a → GetTaskList.Response 발행 |
| `b19441a` | fix(G4): mock_manipulation_a task 미러 구독 GetTaskList.Response |
| `a9ee638` | feat(G5): integration_demo.launch.py + mock_perception_a pub_task_list 토글 |

## 2. 충돌 분류·해소표
| 파일 | 충돌 | 해소 방침 | 결과 |
|---|---|---|---|
| `mission/mission/mission_a.py` | content(v2.3 FSM 전면개편 ↔ perception-new 타입드 토픽) | **v2.3 정본** + perception-new 타입드 토픽 계약 재적용(4곳) | `/perception/task_list`=GetTaskList.Response, `_on_task_list(msg)`, `last_task_list_response`; `import json`은 place_pose_valid용 유지 |
| `mission_interfaces`(srv/action/CMakeLists/package.xml) | 없음(auto) | **union** | 6 인터페이스 유지(+ MoveBaseLateral.srv, MoveToScanPose.action) |
| `mock_*_a.py`·`sim_driver.py`·`launch`·`setup.py` | 없음(auto) | v2.3 정본 | nav=Service mock, /detach_cmd-reactive sim_driver, setup union |
| `task_list.py`·`perception/**` | 없음(auto) | perception-new 존중 | |
| (R5) `mock_manipulation_a.py` 그래스프 래치 | — | `_pending_attach`/보류 처리 **보존 확인** | 유지됨 |

## 3. 인터페이스 계약 대조표 (FSM 기대 ↔ 실제)
| 계약 | FSM 기대 | perception-new 실노드 | mock | 일치/해소 |
|---|---|---|---|---|
| `/perception/task_list` | `GetTaskList.Response`(topic) | `tray_manage_node` 발행 ✓ | mock_perception_a (G3에서 타입드로 수정) | **일치** |
| `/perception/get_task_list` (srv) | `task_list_service_name` 기본 `/mission_a/task_list` | `/perception/get_task_list` | SimDriver=`/mission_a/task_list` | 이름 불일치 — **topic 경로가 기본**이라 G4/G5 무영향. 서비스 경로 사용 시 `task_list_service_name:=/perception/get_task_list` 파라미터/ remap |
| `/perception/wrist/target_one_pose` | `PoseStamped` | `wrist_task_grasp_planner_node` ✓(단, /detections=카메라 필요) | mock_perception_a | 일치(실검출은 카메라 필요 → 시연은 mock) |
| `/detections` | `perception/PartDetectionArray` | perception/msg/PartDetectionArray ✓ | — | 일치 |
| `/perception/place_pose_valid` (C3) | String JSON(guard) | **미제공** | mock 제공 | `use_place_pose_check:=false`(기본) → 무회귀 |
| `/manipulator_state`·`/attached_object`·`/attach_cmd`·`/detach_cmd` | manipulation 계약 | **미제공**(독립테스트만) | mock_manipulation_a | mock 대체 |
| `move_to_scan_pose`(action)·`move_base_lateral`(srv) | mission_interfaces | mock 제공 | mock | 일치 |

> 코드 변경 없이 remap만으로 해소한 항목: 없음(불일치는 타입드 토픽 정합이 필요해 mock 측 최소 코드 수정).
> 향후 서비스 경로 사용 시 remap: `task_list_service_name:=/perception/get_task_list`.

## 4. G4 무회귀 결과 (nav=stub)
- 단계0 (sim, `sim_mode:=true use_mocks:=false use_task_list_service:=true`): **DONE 적재 3**, 차감 A3_PLACE, RECOVERY 0.
- 단계1 (nav=stub, mock 3종, 기본 launch): **DONE 적재 5 ×3 반복 안정** (scan 5, nav stub 9, detach 5, RECOVERY 0).
- 게이트 안전(오선언 0) 주입 시험: C2 드롭(release 전) → 적재 0·RECOVERY, C3 무효/플랩 → 적재 0(릴리스 안함). [`scripts/run_integration_demo.sh inject`]

## 5. G5 통합 시연 결과 (실 perception + mock manip + nav=stub)
- 실행: `ros2 launch mission integration_demo.launch.py` (한 launch 그룹 = 동일 기동 윈도우로 컨테이너 DDS 디스커버리 안정).
- 구성: 실 `tray_manage_node`(mock OCR, tray detection off → 카메라/모델 불필요) + mission_a(nav=stub) + mock_manipulation_a + mock_navigation_a + mock_perception_a(`pub_task_list:=false` → wrist/place만).
- 결과: FSM A1_MONITOR 이 **실 perception task_list 수신** = `TaskList(dome_nut:1, flange_nut:1, gear_ring:1, hex_nut:1, spacer_ring:1, 총 5)` (mock_perception_a 기본값과 다른 = 실 tray_manage_node 출처 확인) → A2_SCAN_POSE→A2_SCAN→A3_PICK(mock)→A3_MOVE_TO_TRAY(stub)→A3_PLACE(mock)→… **5사이클 DONE 적재 5**.

**실제 perception 도달 범위 / mock 대체 지점**
| 단계 | 출처 |
|---|---|
| task_list (A1_MONITOR) | **실 perception `tray_manage_node`** ✓ |
| wrist target (A2_SCAN) | mock_perception_a (실검출은 카메라→detector 필요) |
| scan pose (A2_SCAN_POSE) | mock_manipulation_a (실 manip action server 미전환) |
| pick/place (A3_PICK/A3_PLACE) | mock_manipulation_a (실 manip FSM 계약 미제공) |
| 측방 이동 (A3_MOVE_TO_TRAY/RETURN) | nav=stub (실 navigation 미제공, service 범위 밖) |

> 별도 기동(다른 launch)한 실노드는 본 컨테이너에서 mission_a 가 디스커버 못하는 사례 관측 →
> **한 launch 그룹(integration_demo.launch.py)으로 기동하면 정상 디스커버리**. 실로봇/실 bringup에선 무관.

## 6. manipulation 독립 테스트 보존 & 실행법
manipulation 패키지(`humanoid_challenge/manipulation`)는 **FSM 계약(action/attach) 미연동, 독립 테스트 파일로 실행**(REPORT §1-C 확정). 통합 빌드는 `--packages-up-to mission` 으로 한정되어 manipulation 은 빌드 대상에서 자연 제외(humanoid_challenge 단독 빌드 불가 의존 때문).
- **의존(로봇 필요)**: `pymoveit2`, `moveit_msgs`, 코드 import `ai_worker_manipulation`·`moveit_msgs` → `ai_worker` 컨테이너(bringup + MoveIt) + TRAC-IK 필요. 로봇 없는 통합 검증엔 불가 → FSM 검증은 mock.
- **테스트 파일**(`manipulation/manipulation/tests/`): `test_gripper, test_move_to_capture_pose, test_pick, test_pick_with_perception, test_place, test_zone_a, test_zone_b(_pick/_place), test_dual_box/pick/place/home, test_home, test_lift, test_move_to_pose, test_workspace_scan, test_compute_capture_pose, test_pick_no_selector` — `setup.py` console_scripts 로 등록(`ros2 run manipulation test_*`).
- **실행 절차**(로봇 환경): `ai_worker` 컨테이너에서 bringup+MoveIt 기동 → manipulation 빌드 → `ros2 run manipulation test_pick` 등. (첨부 `REAL_ROBOT_TEST_PROCEDURE.md` 기준 + 위 실제 노드명 반영. 첨부 md 3종은 과거 스냅샷이라 코드베이스가 정본.)

## 7. 다음 단계 (이번 범위 밖)
- manipulation **ROS2 action server 전환** + FSM↔manip 런타임 완전 연동(`move_to_scan_pose`/`/attach_cmd`·`/detach_cmd`·`/attached_object`/`/manipulator_state`).
- `nav_mode:=service` 콜드 디스커버리 해결(별도 트랙: DDS/RMW, 또는 서비스 클라이언트 wait 전략).
- live perception 단일 노드화 + 실 카메라/detector 로 wrist target 실검출, C3 `/perception/place_pose_valid` 실제 제공.
- 서비스 경로 사용 시 `task_list_service_name` remap 적용.

## 8. 재현
컨테이너에서: `/ws/src/humanoid_challenge/scripts/run_integration_demo.sh all`
(개별: `... s0` | `s1` | `g5` | `inject`). 모두 nav=stub.
