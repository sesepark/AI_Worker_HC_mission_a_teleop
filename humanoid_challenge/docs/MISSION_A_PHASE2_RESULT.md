# Mission A Phase 2 결과 — mock→실 런타임 전환 (integration/mission-a)

> Phase 1(mock 3종 + nav=stub, DONE 적재 5)을 실 런타임으로 **drop-in 전환**. T1(manipulation 실 서버),
> T2(live perception 단일 launch + 실 wrist + C3), T3(task_list 서비스 remap). **nav 은 전 구간 stub 유지**.
> 작업 브랜치 `integration/mission-a`(원본 3브랜치 무변경). 지시 정본: `MISSION_A_PHASE2_{PROMPT,CONTEXT}.md`.

## 0. 요약 — 무엇이 실로 전환됐나
| 작업 | 전환 | 상태 |
|---|---|---|
| **T3** task_list 서비스 경로 | mock 서비스 → `/perception/get_task_list`(실 tray_manage_node) remap | ✅ humanoid_challenge 헤드리스 검증 완료 |
| **T1** manipulation | mock_manipulation_a → 실 `mission_a_manipulation_server`(MoveIt) | 🟡 코드·빌드·계약 검증 완료. **실 MoveIt 모션 검증(G-T1)은 사용자 로봇에서**(ai_worker, 안전상 본 세션 미실행) |
| **T2** perception | mock wrist/place → `perception_live`(실 detector→wrist) + `place_pose_valid_node`(C3) | 🟡 헤드리스 검증(빌드/노드기동/place_pose_valid/static TF). **실 wrist 검출(카메라)·C3 실정렬은 사용자 로봇에서** |
| nav | — | ⏸ **stub 유지(범위 밖)**. TODO: nav 서비스 콜드 디스커버리 |

> **검증 환경 한계(정직)**: 본 세션은 `humanoid_challenge` 컨테이너(로봇/카메라 無)에서 수행. ai_worker 의
> `/move_group`+컨트롤러가 가동 중이라 **픽/플레이스 모션 실행은 실 로봇을 움직일 수 있어** 사용자 지시로
> **실모션 미실행**(코드+구조+계약+헤드리스 검증만). 통과 위장 없음.

## 1. 확정된 계약 표 (소스 확정, §4.2 7항목)
1. `/manipulator_state`·`/attached_object`·`/attach_cmd`·`/detach_cmd` = **`std_msgs/String`**(mock_manipulation_a:46–55). 의미: manipulator_state "IDLE", attached_object class/"", attach_cmd "pick", detach_cmd class.
2. 픽/플레이스 트리거 = **별도 action/srv 아님**. A2_SCAN_POSE=`move_to_scan_pose` action / A3_PICK=`/attach_cmd`→`/attached_object` 대기 / A3_PLACE=C1∧C2∧C3 게이트→`/detach_cmd`→`/attached_object==""`→차감.
3. `MoveToScanPose.action`: Goal `string preset_id` / Result `bool success, string message` / FB `float32 progress`.
4. `/perception/place_pose_valid` JSON: FSM은 **`valid`(bool) 키만** 사용(+신선도≤1s, 디바운스 0.3s, 플랩 차단).
5. 그래스프 래치: `/attached_object`=class 는 **파지 확정 시점에만** 래치(손실 시 "") → C2 게이트(오선언 0).
6. `GetTaskList.srv`: Req `float32 timeout_sec, uint16 frame_count` / Resp `bool success, string message, bool screen_detected, bool all_counts_recognized, uint16 frames_used, TaskItem[] parts{string name,int32 count}`.
7. place primitive: `PlaceSkill.place(pose,arm,...)→PlaceResult.SUCCESS`(gripper.open=release; planning-scene detach는 호출자). PickSkill.pick→PickResult.SUCCESS(=GraspSkill.assess_stable, 그래스프 관측).

## 2. 변경 파일
| 경로 | 신규/수정 | 요지 |
|---|---|---|
| `manipulation/manipulation/mission_a_manipulation_server.py` | 신규 | T1 실 서버(mock drop-in, 검증 primitive 재사용, 래치 보존, MTE+Reentrant) |
| `manipulation/launch/mission_a_manip.launch.py` | 신규 | ai_worker 실 서버 기동 |
| `manipulation/setup.py`·`package.xml` | 수정 | console_script + launch 설치, mission/mission_interfaces/std_msgs dep |
| `perception/perception_nodes/place_validity/place_pose_valid_node.py` (+`__init__.py`) | 신규 | C3 `/perception/place_pose_valid` 실 발행(TF 기반 valid + 폴백 + 주입) |
| `perception/launch/perception_live.launch.py` | 신규 | detector+tray+wrist(§4.3)+static TF+place_pose_valid 단일 launch |
| `perception/CMakeLists.txt` | 수정 | place_pose_valid_node 설치 |
| `mission/launch/mission_a.launch.py` | 수정 | T3 `task_list_service_name`·`task_list_topic` arg |
| `mission/launch/integration_demo.launch.py` | 수정 | T3 서비스 args 패스스루 |
| `mission/launch/mission_a_real.launch.py` | 신규 | Phase 2 실 통합(perception_live + mission_a, nav=stub) |
| `docs/MISSION_A_PHASE2_RESULT.md` | 신규 | 본 보고서 |

> **FSM(`mission_a.py`) 외부 계약 무변경**(drop-in 원칙). mock 3종도 무변경(계약 정본 보존).

## 3. 설계 요점 (drop-in·불변식)
- **T1**: 실 서버가 mock 외부 계약(토픽/액션 이름·타입·시맨틱) 그대로 노출(노드명만 `mission_a_manipulation_server`). 내부=검증 primitive(`PickSkill`/`PlaceSkill`/`MoveItClient`/`build_mission_a_grasp_pose`, CAPTURE_JOINTS/CARRY_Z/PLACE_*) 재사용, **새 로직 없음**. **래치 보존**: `/attached_object`=class 는 `PickResult.SUCCESS`(=assess_stable) 시점에만 → C2 오선언 0. 동시성 MTE+ReentrantCallbackGroup, `_busy` 직렬화.
- **T2**: perception_live 단일 launch(동일 윈도우 디스커버리). place_pose_valid_node 가 C3 실 공급(JSON `valid` 키 = FSM 파서 일치), 무효/플랩 주입으로 게이트 검증. 실 wrist target=detector→wrist_planner(§4.3 params).
- **T3**: FSM 코드 무변경(서비스 경로 이미 구현). launch arg remap만. **topic 기본 경로 무영향**.

## 4. 게이트별 검증
- **G-T3 ✅ (humanoid_challenge, 로봇無)**: `integration_demo.launch.py use_task_list_service:=true task_list_service_name:=/perception/get_task_list task_list_topic:=/perception/_unused_tl` → `[A1_MONITOR] task_list service 요청: /perception/get_task_list` → `task_list 확정 5 parts`(토픽 차단, 서비스 단독 수신). **무회귀**: `run_integration_demo.sh s0`=DONE 적재 3, `s1`=DONE 적재 5, RECOVERY 0.
- **G-T1 🟡 (코드/구조)**: ament_python 빌드 RC=0, `mission_a_manipulation_server` console_script+launch 설치, 구문 OK, 외부계약 자가검토 일치(pub/sub/action 표 §1과 동일). **실 MoveIt 런타임(action list/send_goal/전 사이클/C2 드롭)은 미실행** — 사유: manipulation/mission/mission_interfaces 가 ai_worker 미빌드 + 픽/플레이스 모션=실 로봇 위험. → §6 절차로 사용자 수행.
- **G-T2 🟡 (헤드리스)**: perception 빌드 RC=0, perception_live 5노드 기동(static TF `camera_r_link→camera_right_link` 게시 확인), `place_pose_valid_node` echo: default→`{"valid":true}`, force_invalid→`{"valid":false}`, flap→토글(타입 `std_msgs/msg/String`). tray task_list 헤드리스 OK(G-T3). **실 wrist 검출(카메라)·C3 실 정렬 검증은 미실행** → §6 절차로 사용자.
- **G-FINAL 🟡**: `mission_a_real.launch.py` 파싱·구성 확인(perception_live+mission_a 기동, mission_a=INIT에서 실 manip 대기). **실 manip(ai_worker)+카메라 부재로 전 사이클 미실행**. 실 perception→mission task_list 연동은 Phase1 G5(`integration_demo`)에서 입증(real tray → A1_MONITOR→…→DONE 적재 5, mock manip). → §6 절차로 G-FINAL 수행.

## 5. 남은 mock/stub & TODO
- **nav = stub 유지**(범위 밖). `nav_mode:=service`/`move_base_lateral` 실연동 미수행.
- **TODO(다음 트랙)**: ① nav 서비스 **콜드 디스커버리** 해결(DDS/RMW 또는 서비스 클라 wait 전략). ② 실 로봇에서 G-T1/G-T2/G-FINAL 완주 검증(§6). ③ place_pose_valid 기하 정렬 로직 고도화(현 TF 근방판정 + 폴백). ④ carry/place pose·planning-scene attach 다사이클 정책 실로봇 튜닝.

## 6. 재현 명령 (전체)
### 6.1 헤드리스(humanoid_challenge, 로봇無) — 본 세션 검증분
```bash
docker exec -it humanoid_challenge bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
cd /ws && colcon build --symlink-install --packages-up-to mission && colcon build --packages-select perception manipulation
# G-T3 서비스 경로 + 무회귀
/ws/src/humanoid_challenge/scripts/run_integration_demo.sh s0   # DONE 적재 3
/ws/src/humanoid_challenge/scripts/run_integration_demo.sh s1   # DONE 적재 5
# place_pose_valid(C3) 확인
ros2 run perception place_pose_valid_node --ros-args -p force_invalid:=true   # {"valid": false}
```
### 6.2 실 로봇(ai_worker + humanoid_challenge, 동일 DDS 도메인) — G-T1/G-T2/G-FINAL
```bash
# 공통 env(양 컨테이너): ROS_DOMAIN_ID=30; ROS_LOCALHOST_ONLY=0; ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
# [ai_worker] 로봇/MoveIt + 실 manip 서버
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
ros2 launch ffw_moveit_config moveit.launch.py
# (manipulation/mission/mission_interfaces 를 ai_worker ws 에 빌드 후)
ros2 launch manipulation mission_a_manip.launch.py            # 실 manip 서버(T1)
ros2 action list | grep move_to_scan_pose                     # G-T1: 실 노드 노출
# [로봇] wrist 카메라 bringup
ros2 launch ffw_bringup ffw_sg2_ai.launch.py colorizer.enable1:=false colorizer.enable2:=false
# [humanoid_challenge] 실 perception + mission_a(nav=stub)
ros2 launch mission mission_a_real.launch.py mock_monitor_ocr:=true        # G-FINAL
#   C3 활성: ... use_place_pose_check:=true   (perception_live place_pose_valid 제공)
# 게이트 안전: C2 드롭/C3 무효·플랩 시 적재 0 (실 환경 확인)
```

## 7. 수용/무회귀 요약
- ✅ G-T3(서비스 remap + topic 무회귀), 헤드리스 mock 경로 무회귀(s0=3, s1=5, RECOVERY 0), place_pose_valid C3 계약.
- 🟡 G-T1/G-T2/G-FINAL: 코드·계약·빌드·헤드리스 검증 완료, **실 로봇 모션/카메라 검증은 §6.2 로 사용자 수행**(안전 결정에 따름).
- FSM·mock 외부 계약 불변, 그래스프 래치 보존, 원본 3브랜치 무변경.
