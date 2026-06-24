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
### 6.2 실 로봇 전체 사이클 실행 매뉴얼 (G-T1/G-T2/G-FINAL)

> **운용 아키텍처 (중요)**: **로봇 PC**(`ffw-...`)는 **bringup + MoveIt 등 로봇 구동에 필요한 것만** 실행한다.
> **메인 PC**(`humanoid_challenge` 환경, 본 레포 빌드)가 **모든 응용 패키지** — manip 서버(`mission_a_manip`),
> perception, mission FSM(`mission_a_real`) — 를 실행하고, **같은 네트워크의 ROS(domain 30) 통신**으로 로봇
> move_group/controller 를 원격 제어한다. 즉 manip 서버도 로봇이 아니라 **메인 PC에서 실행**한다(메인 PC에
> pymoveit2 + manipulation/mission/mission_interfaces 빌드 필요). 따라서 manip 서버의 MoveItClient 는 move_group·
> /joint_states 를 **크로스-PC**로 받으며, FIX-1(단일 executor)·FIX-2(joint_state 신선도 가드)가 이 경로에 직접 작용.

> **핵심 근본원인 2가지(반드시 반영)**:
> 1. **colorizer 끄기** — bringup 기본은 depth 를 **colorized `rgb8`** 로 발행 → (a) metric 아님(3D 투영 불가),
>    (b) 용량 커서 **네트워크로 안 넘어옴**(메인 PC에서 depth 0프레임). `colorizer.enable*:=false` 로 끄면
>    depth=`16UC1` 가 되고 **그제서야 cross-PC 스트림됨**(~4.4Hz). `tf_publish_rate*:=10.0` 로 TF 부하도 완화.
> 2. **perception_live 의 static TF 필수** — 로봇 TF(`base_link→…→camera_r_link`)와 realsense
>    (`camera_right_link→camera_right_color_optical_frame`)가 **끊겨 있어**, perception_live 의
>    `camera_r_link→camera_right_link`(identity) 브리지가 둘을 잇는다 → `publish_camera_tf:=true` 유지(기본).
> ⚠️ 카메라 단일 소유(FIX-4): bringup 이 camera_right 를 올리므로 **별도 카메라 launch 금지**(USB 충돌).

**공통 env (양쪽 PC 동일)**: `ROS_DOMAIN_ID=30`, `ROS_LOCALHOST_ONLY=0`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`

| 순서 | 위치 | 명령 | 직후 확인(메인 PC) — **공유할 결과** |
|---|---|---|---|
| **A. bringup+MoveIt** | 로봇 PC | `ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py colorizer.enable1:=false colorizer.enable2:=false tf_publish_rate1:=10.0 tf_publish_rate2:=10.0`  그리고  `ros2 launch ffw_moveit_config moveit.launch.py` | `ros2 node list \| grep -E "move_group\|controller_manager"` (보여야 함), `ros2 topic hz /joint_states`, `ros2 topic echo /camera_right/camera_right/depth/image_rect_raw --field encoding --once` → **반드시 `16UC1`** |
| **B. manip 서버** | **메인 PC** | `ros2 launch manipulation mission_a_manip.launch.py` | `씬 초기화 완료` → `ready (real MoveIt)` 로그, `ros2 action list \| grep move_to_scan_pose`, `ros2 topic echo /manipulator_state --once` → **`IDLE`** (FIX-1) |
| **C. perception+FSM** | **메인 PC** | `ros2 launch mission mission_a_real.launch.py mock_monitor_ocr:=true use_place_pose_check:=true` | `ros2 topic echo /detections --once`(부품 검출), `ros2 topic echo /perception/wrist/target_one_pose --once`(base_link 3D), FSM 상태천이 `INIT→A1_MONITOR→A2_SCAN_POSE→A2_SCAN→A3_PICK` |
| **D. 사이클 관찰** | (자동) | 부품을 **scan 자세 wrist FOV**(트레이)에 5개 배치 | `A3_PICK` 파지 성공(C2 래치 `/attached_object`), `A3_PLACE` C3 게이트→`/detach_cmd`→잔여 차감, `VERIFY→DONE 적재 N`. FIX-2(`-10` 무)/FIX-3(`error_code=-N (이름)`) |
| **E. 다사이클** | (반복) | manip 서버 **재기동 없이** mission_a_real 연속 3회 | 매회 풀사이클 DONE, `move_to_scan_pose success=True` (FIX-1/2 회귀) |

> 안전: B~E 부터 실 팔이 움직임(scan/pick/place). e-stop 준비. 이상 거동(반복 `-10`, 엉뚱 타겟, 충돌 위험) 시 즉시 중단.
> 튜닝: depth/color 가 네트워크로 ~4Hz throttle. RGB-D sync 흔들리면 `config/wrist_projection/params.yaml` 의
> `sync_slop` 0.10→0.2~0.3. PTP 가 `non-zero start velocity(INVALID_ROBOT_STATE)` 내면 로봇 정지 후(zero-vel) 시도.
> FIX-6(인지): Dynamixel `BULK_READ_FAIL`/실시간 오버런은 벤더(로봇 HW/드라이버) 영역 — baud/USB latency/전원/제어
> 루프 부하 점검. 버스 불안정 시 joint_states 깜빡임→FIX-2 가드 발동, PTP start-state 거부로 pick 불안정.

## 7. 수용/무회귀 요약
- ✅ G-T3(서비스 remap + topic 무회귀), 헤드리스 mock 경로 무회귀(s0=3, s1=5, RECOVERY 0), place_pose_valid C3 계약.
- 🟡 G-T1/G-T2/G-FINAL: 코드·계약·빌드·헤드리스 검증 완료, **실 로봇 모션/카메라 검증은 §6.2 로 사용자 수행**(안전 결정에 따름).
- FSM·mock 외부 계약 불변, 그래스프 래치 보존, 원본 3브랜치 무변경.
