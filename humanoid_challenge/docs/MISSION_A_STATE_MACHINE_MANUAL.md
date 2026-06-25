# Mission A 상태머신 실행 매뉴얼 (STATE MACHINE MANUAL)

> **대상 브랜치**: `integration/mission-a` · **범위**: Mission A FSM(`mission_a`)의 구조·실행 절차·계약 정본.
> **검증 방식**: 모든 주장은 소스 `파일:라인` 근거. 인용 경로는 `humanoid_challenge/` 루트 기준 상대경로
> (예: `mission/mission/mission_a.py:511`). 근거 미확정 항목은 "확인필요"로 표기.
> **정본 교차**: 실로봇 전체 사이클 절차·근본원인(FIX-1~6)은 [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md) §6.2가 정본이며 본 매뉴얼은 이를 링크한다(중복 금지).
> **시리즈**: 실행 명령 모음 → [RUNBOOK.md](RUNBOOK.md) · 계약 레퍼런스 → [INTERFACES.md](INTERFACES.md) · 시나리오 → [SCENARIOS.md](SCENARIOS.md) · 문서 색인 → [README.md](README.md).

---

## 1. 개요 & 아키텍처

### 1.1 설계 골격
Mission A는 **단일 FSM 노드(`mission_a`, `MissionA(Node)`)가 오케스트레이터**이고, manipulation·navigation·perception은 **토픽/액션/서비스 계약**으로만 연결된 외부 모듈이다. FSM은 자신이 직접 모터를 돌리지 않고, "스캔 포즈 형성", "파지", "베이스 이동", "검출 결과 수신"을 **계약 호출/구독**으로 위임한다. (`mission/mission/mission_a.py:1-18` 모듈 docstring, `:109` 클래스 정의)

핵심 설계 원칙(코드 docstring 근거):
- **DDS 위생**: ActionClient는 scan 1개뿐. v2.2의 2번째 액션이 통합 디스커버리 병목(EDP 굶주림)을 유발했으므로 베이스 이동은 **Action이 아닌 Service(`MoveBaseLateral.srv`)** 로 단순화. (`mission_a.py:8-9, 14-15, 166`)
- **드롭인(drop-in)**: 실 manipulation 서버(`mission_a_manipulation_server`)가 mock(`mock_manipulation_a`)과 **동일한 외부 계약**(토픽/액션 이름·타입·시맨틱)을 노출 → FSM 코드 무변경으로 mock↔실 교체. ([MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md) §3)
- **차감 게이트 이관**: 잔여 수량 차감을 VERIFY가 아니라 **A3_PLACE**에서, C1∧C2∧C3 게이트 통과 후에만 수행. (`mission_a.py:11-12, 611-648`)

### 1.2 2-PC(논리 3구성) 토폴로지
운용 아키텍처는 [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md) §6.2, `mission/launch/mission_a_real.launch.py:2-20` 주석이 정본.

| 위치 | 실행 대상 | 역할 | 근거 |
|---|---|---|---|
| **로봇 PC** (`ffw-...`) | `ffw_sg2_follower_ai` bringup + `moveit` (+ 옵션 `move_base_lateral`) | 하드웨어 구동, 카메라(`camera_right`), `move_group`/`/joint_states`, `/cmd_vel`·`/odom` 제공 | `mission_a_real.launch.py:10-18`, `move_base_lateral.launch.py:4-8` |
| **메인 PC** (`humanoid_challenge`) | perception_live + mission FSM(`mission_a_real`) | 실 perception 파이프라인 + FSM 오케스트레이션 | `mission_a_real.launch.py:49-61` |
| **manip 서버** (`ai_worker` 컨테이너) | `mission_a_manip` (실 MoveIt 클라이언트) | 픽/플레이스 primitive, `move_group`을 **크로스-PC** 원격 제어 | `mission_a_real.launch.py:9-13`, `manipulation/launch/mission_a_manip.launch.py:2-8` |

> 주의: manip 서버는 "로봇이 아니라 메인/ai_worker 측"에서 실행되어 `move_group`·`/joint_states`를 크로스-PC로 수신한다. ([MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md):74-78) 단일 PC 헤드리스 검증에서는 mock 3종으로 대체.

### 1.3 데이터 흐름 (정상 1사이클)
```
                      task_list(GetTaskList.Response)
 tray_manage_node ─────────────────────────────► mission_a (FSM)
 (perception_live)                                  │  A1_MONITOR 에서 목표 확정
                                                    │
 detector_node ─► /detections ─► wrist_task_grasp_planner_node
                                   │ /perception/wrist/target_one_pose (PoseStamped, base_link)
                                   ▼
                              mission_a ──(MoveToScanPose.action)──► manip 서버  [A2_SCAN_POSE]
                              mission_a ──(/attach_cmd "pick")──────► manip 서버  [A3_PICK]
                              manip 서버 ─(/attached_object=class)──► mission_a   (파지 래치, C2)
                              mission_a ──(MoveBaseLateral.srv left)─► nav        [A3_MOVE_TO_TRAY]
 place_pose_valid_node ─(/perception/place_pose_valid)─► mission_a               [A3_PLACE C3]
                              mission_a ──(/detach_cmd class)───────► manip 서버  [A3_PLACE]
                              manip 서버 ─(/attached_object="")────► mission_a   (해제 확정 → 차감)
                              mission_a ──(MoveBaseLateral.srv right)─► nav       [A3_RETURN_TO_BOX]
                                   └──► VERIFY ──► (잔여>0 ? 루프 : DONE)
```

---

## 2. 상태머신 명세

### 2.1 상태 다이어그램
12개 상태(`State(Enum)`, `mission/mission/mission_a.py:58-70`). 10Hz 타이머가 `_tick`으로 현재 상태 핸들러를 디스패치(`:241, :449-464`).

```
 INIT ──(servers_ok ∧ IDLE | timeout 60s)──► A1_MONITOR
                                               │
                  ┌──(task_list 잔여 0)────────┘
                  ▼                            └──(task_list 잔여>0)──► A2_SCAN_POSE ◄──────────────┐
                VERIFY ◄──(task_list 비어있음)── A2_SCAN ◄─(scan success)─┘                          │
                  │  │                            │                                                 │
   (잔여>0)────────┘  └──(잔여 0)──► DONE          └─(target base_link)─► A3_PICK                     │
   ▼                                                                       │(attached)              │
 A3_RETURN_TO_BOX ◄──────────────────────────────────────────┐            ▼                        │
   │(arrived right 675mm) ──► A2_SCAN_POSE                    │     A3_MOVE_TO_TRAY ─(arrived left)─►│
   │                                                          │            │                  A3_PLACE
   └──(fail|timeout)──► RECOVERY                              │            └─(C2 drop)─► RECOVERY  │
                                                              │                                    │
 RECOVERY ──(recovery_count<3 ⇒ ++)──► A2_SCAN_POSE           └────────(C1∧C2∧C3 통과 → detach → 해제확정 → 차감)─► VERIFY
        └──(recovery_count≥3)──► MANUAL_WAIT (정지, TODO Phase2)
```
> 모든 실패/타임아웃 전이는 RECOVERY로 수렴한다(상태표 참조). RECOVERY 재진입 지점은 항상 `A2_SCAN_POSE`.

### 2.2 상태 표
타임아웃: 정적 맵 `STATE_TIMEOUT`(`:74-81`) + `__init__`에서 파라미터 합성(`:158-162`). 판정은 `_timed_out()`(`:257-259`).

| 상태 | 진입 동작(`_on_enter`) | 수행 동작 | 정상 전이(가드) | 실패·예외 → RECOVERY | 타임아웃 |
|---|---|---|---|---|---|
| **INIT** `:498-509` | — | `/active_mission='A'` 발행; scan 서버 준비+manip IDLE 확인 | `servers_ok ∧ last_manipulator_state=='IDLE'` → A1_MONITOR (`:501-505`) | (없음) | 60s 시 A1_MONITOR **강행**(`:506-509`) |
| **A1_MONITOR** `:511-523` | — | task_list 확정 대기; `use_task_list_service`면 서비스 요청(`:522-523`) | 잔여>0 → A2_SCAN_POSE(`:514-517`); 잔여==0(비어있지 않음) → VERIFY(`:518-520`) | (없음) | **핸들러가 `_timed_out()` 미호출 → 무한 대기**(불일치 §8) |
| **A2_SCAN_POSE** `:525-556` | `_scan.reset()`(`:477`) | `MoveToScanPose` goal 송신·결과 폴링; sim 우회(`:527-528`) | `_scan.done ∧ result.success` → A2_SCAN(`:547-550`) | goal 거부/예외/실패(`:551-553`); 서버 미준비 timeout(`:536-538`) | 30s(`scan_pose_timeout_sec`) → RECOVERY(`:554-556`) |
| **A2_SCAN** `:558-579` | cycle++, target/attached/pick_class 리셋(`:478-483`) | `target_one_pose` 수신·frame 검증·consume | `last_target_pose≠None ∧ frame=='base_link'` → A3_PICK(`:559-571`); task_list 비면 → VERIFY(`:573-575`) | frame≠base_link면 폐기·재대기(`:561-565`) | 90s → RECOVERY(`:577-579`) |
| **A3_PICK** `:581-590` | `/attach_cmd='pick'` 발행(`:487`) | 파지 반응 대기(manip이 `/attached_object`에 class 보고) | `last_attached_object`(비어있지 않음) → A3_MOVE_TO_TRAY(`:583-587`) | (없음) | 45s → RECOVERY(`:588-590`) |
| **A3_MOVE_TO_TRAY** `:592-609` | `_move_tray.reset()`(`:489`) | 이동 중 C2 드롭 감시; `_nav_step(left)` | `st=='arrived'` → A3_PLACE(`:600-603`) | C2 드롭 `attached==''`(`:594-597`); `st=='failed'`(`:604-606`) | 30s → RECOVERY(`:607-609`) |
| **A3_PLACE** `:611-648` | `_release_issued=False`(`:491`) | **Phase1**: C1∧C2∧C3 게이트→`/detach_cmd` 발행. **Phase2**: 해제확정(`attached==''`)→차감 | Phase2 `attached==''` → 차감+`placed++` → VERIFY(`:636-645`) | C1 class 없음(`:615-618`); C2 드롭(`:619-623`); C3 무효+timeout(`:624-629`); Phase2 timeout(`:646-648`) | 45s |
| **A3_RETURN_TO_BOX** `:663-675` | `_return.reset()`(`:493`) | `_nav_step(right)`(트레이→박스) | `st=='arrived'` → A2_SCAN_POSE(`:666-669`) | `st=='failed'`(`:670-672`) | 30s → RECOVERY(`:673-675`) |
| **VERIFY** `:650-661` | — | 잔여 라우팅(차감은 A3_PLACE에서 끝남) | 잔여>0 → A3_RETURN_TO_BOX(`:656-658`); 잔여0 → DONE(`:659-661`) | (없음) | 20s(맵 존재, 핸들러 미참조) |
| **DONE** `:677-679` | — | `적재 N` 로그 + `timer.cancel()`(종료) | (종단) | — | — |
| **RECOVERY** `:681-691` | — | 재시도 카운트 판정 | `recovery_count<3` ⇒ `++` → A2_SCAN_POSE(`:682-688`) | — | — |
| **MANUAL_WAIT** `:693-695` | — | `pass`(운용자 개입 대기) | **미구현**(TODO Phase2, `:694`) | — | — |

> **잔여 루프**: 차감은 A3_PLACE Phase2(`:637 task_list.decrement`)에서만. VERIFY가 잔여>0이면 A3_RETURN_TO_BOX→A2_SCAN_POSE로 루프(`:657-669`).
> **RECOVERY 예산**: `MAX_RECOVERY_RETRY=3`(`:52`), 미션 전체 누적(사이클 간 리셋 없음 — §8).

### 2.3 비동기 래치 패턴
액션/서비스 호출은 **상태 진입 시 1회 송신, 매 틱 폴링**. `AsyncLatch`(`:87-104`)는 콜백에서 `result`를 먼저 채우고 `done`을 마지막에 set(원자성). scan=`_scan`, 좌이동=`_move_tray`, 우이동=`_return`. nav 폴링은 `_nav_step`(`:410-444`)이 `'arrived'|'pending'|'failed'` 반환.

---

## 3. 실행 순서 & 기동 런북

> 복사-실행용 명령 모음의 정본은 [RUNBOOK.md](RUNBOOK.md). 본 절은 "무엇을/어디서/어떤 순서·게이팅으로"의 개요.

### 3.1 환경변수
| 구성 | `ROS_DOMAIN_ID` | `ROS_LOCALHOST_ONLY` | `ROS_AUTOMATIC_DISCOVERY_RANGE` | 근거 |
|---|---|---|---|---|
| 헤드리스 단일 PC(s0/s1/g5) | 90(기본) | 1 | (미설정) | `scripts/run_integration_demo.sh:12-15` |
| 실 통합(크로스-PC) | 30 | 0 | SUBNET | `mission_a_real.launch.py:13`, [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md):89 |

### 3.2 4개 기동 시나리오
모두 `mission/launch/`의 단일 launch로 게이팅된다.

| # | 시나리오 | 명령 | 구성 노드 | 선행조건/게이팅 | 기대 |
|---|---|---|---|---|---|
| **S0** | sim 무회귀 | `ros2 launch mission mission_a.launch.py sim_mode:=true use_mocks:=false use_task_list_service:=true` | `mission_a`(SimDriver 입력) | 없음(신규 액션/서비스 우회, `:527-528`) | DONE 적재 3 (`run_integration_demo.sh:36-43`) |
| **S1** | mock 통합(nav=stub) | `ros2 launch mission mission_a.launch.py` | `mission_a` + mock 3종(`use_mocks` 기본 true) | mock이 계약 제공 | DONE 적재 5, RECOVERY 0 (`:45-51`) |
| **G5** | 실 perception task_list + mock | `ros2 launch mission integration_demo.launch.py` | `tray_manage_node`(실) + mission_a + mock manip/nav/wrist | `mock_pub_task_list:=false`로 실 task_list만 주입(`integration_demo.launch.py:47-57`) | DONE 적재 5 (`:53-59`) |
| **Real** | Phase 2 실 통합 | `ros2 launch mission mission_a_real.launch.py [nav_mode:=service] [use_place_pose_check:=true]` | perception_live + mission_a(`use_mocks:=false`) | 로봇 PC bringup/MoveIt + ai_worker manip 서버 선행, depth=16UC1, static TF, 카메라 단일소유 | 실 사이클(정본 절차=[RESULT §6.2](MISSION_A_PHASE2_RESULT.md)) |

### 3.3 단계별 게이팅(Real 기준 요약)
정본 절차·확인 명령은 [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md) §6.2 표(A~E)와 [RUNBOOK.md](RUNBOOK.md).
1. **로봇 PC**: bringup(`colorizer.enable*:=false`, `tf_publish_rate*:=10.0`) + MoveIt → depth `16UC1` 확인.
2. **ai_worker**: `mission_a_manip.launch.py` → `/manipulator_state=IDLE`, `move_to_scan_pose` 노출.
3. **메인 PC**: `mission_a_real.launch.py` → perception_live + FSM. FSM은 `INIT`에서 manip IDLE+scan 서버 준비를 게이트(`:501-505`) 후 진행.
4. **nav 서비스 사용 시**: 로봇 PC에서 `move_base_lateral.launch.py` 별도 기동 후 메인 PC `nav_mode:=service`.

---

## 4. 실행파일 ↔ 코드 맵

| 실행파일 | 패키지 | Entry point / 설치 | 소스 | 역할 |
|---|---|---|---|---|
| `mission_a` | mission | `mission.mission_a:main` (`mission/setup.py:26`) | `mission/mission/mission_a.py` | **FSM 오케스트레이터**(본 매뉴얼 대상) |
| `mock_manipulation_a` | mission | `mission.mock_manipulation_a:main` (`setup.py:27`) | `mission/mission/mock_manipulation_a.py` | manip 계약 mock(scan action, attach/detach, `/attached_object`·`/manipulator_state`) |
| `mock_navigation_a` | mission | `mission.mock_navigation_a:main` (`setup.py:28`) | `mission/mission/mock_navigation_a.py` | `MoveBaseLateral.srv` mock 서버 |
| `mock_perception_a` | mission | `mission.mock_perception_a:main` (`setup.py:29`) | `mission/mission/mock_perception_a.py` | task_list / wrist target / place_pose_valid mock 발행 |
| `move_base_lateral` | mission | `mission.move_base_lateral_node:main` (`setup.py:30`) | `mission/mission/move_base_lateral_node.py` | **실** `MoveBaseLateral.srv` 서버(/cmd_vel·/odom 폐루프, 로봇 PC) |
| `mission_a_manipulation_server` | manipulation | `manipulation.mission_a_manipulation_server:main` (`manipulation/setup.py:29`) | `manipulation/manipulation/mission_a_manipulation_server.py` | **실** manip 서버(MoveIt 픽/플레이스, mock drop-in) |
| `detector_node`(part_detector) | perception | `install(PROGRAMS ... RENAME detector_node)` (`perception/CMakeLists.txt:21-25`) | `perception/perception_nodes/part_detector/detector_node.py` | YOLO 부품 검출 → `/detections` |
| `tray_manage_node` | perception | `CMakeLists.txt:36-40` | `perception/perception_nodes/management/tray_manage_node.py` | OCR→task_list 발행 + `/perception/get_task_list` 서비스 |
| `wrist_task_grasp_planner_node` | perception | `CMakeLists.txt:31-35` | `perception/perception_nodes/wrist_projection/wrist_task_grasp_planner_node.py` | 검출+depth → `target_one_pose`(base_link) |
| `place_pose_valid_node` | perception | `CMakeLists.txt:61-65` | `perception/perception_nodes/place_validity/place_pose_valid_node.py` | C3 `/perception/place_pose_valid` 발행 |
| `monitor_ocr_node`, `monitor_ocr_viewer`, `peg_detector_node`, `nut_detector_node`, `head_pipe_top_centers_node` | perception | `CMakeLists.txt:26-30, 41-60` | `perception/perception_nodes/...` | OCR/보조 검출(코어 흐름 외) |
| `static_transform_publisher` | tf2_ros(벤더) | `perception_live.launch.py:94-102` | — | `camera_r_link→camera_right_link`(identity) TF 브리지 |

> `mission_interfaces`는 실행파일 없음 — 인터페이스 생성 전용(`mission_interfaces/CMakeLists.txt:9-17`).
> 빌드: `colcon build --symlink-install --packages-up-to mission`(+ `--packages-select perception manipulation`). (`run_integration_demo.sh:30-34`, [RESULT §6.1](MISSION_A_PHASE2_RESULT.md):64)

---

## 5. 계약/인터페이스

> 통합 레퍼런스: [INTERFACES.md](INTERFACES.md). 본 절은 FSM 시점의 생산자/소비자 매핑.

### 5.1 토픽 (모두 FSM 기준)
| 토픽 | 타입 | FSM | 상대 노드 | 근거 |
|---|---|---|---|---|
| `/active_mission` | std_msgs/String | **pub** (`:203, :499`) | (모니터링) | — |
| `/attach_cmd` | std_msgs/String | **pub** "pick" (`:204, :487`) | manip **sub**(`mission_a_manipulation_server.py:64-65`) | A3_PICK 트리거 |
| `/detach_cmd` | std_msgs/String | **pub** class (`:205, :630`) | manip **sub**(`server.py:66-67`) | A3_PLACE 해제 |
| `/manipulator_state` | std_msgs/String | **sub** (`:176-178`) | manip **pub** "IDLE"/"BUSY"(`server.py:63`) | INIT 게이트 |
| `/attached_object` | std_msgs/String | **sub** (`:185-187`) | manip **pub** class/""(`server.py:62`) | C2 파지 래치 |
| `/detections` | perception/PartDetectionArray | **sub** (`:179-181`, 저장만/미사용 §8) | detector **pub** | — |
| `/perception/wrist/target_one_pose` | geometry_msgs/PoseStamped | **sub** (`:182-184`) | wrist_planner **pub** | A2_SCAN 입력(base_link) |
| `/perception/task_list` | mission_interfaces/GetTaskList.**Response** | **sub** (`:190-191`) | tray_manage **pub** | A1_MONITOR 목표 |
| `/perception/place_pose_valid` | std_msgs/String(JSON) | **sub** (`:194-196`) | place_pose_valid **pub** | C3 게이트 |

> 특이점: `/perception/task_list`는 서비스 응답 타입(`GetTaskList.Response`)을 **토픽 메시지로** 사용한다(`:190-191`, `mission_a.py:17` 주석).

### 5.2 액션 / 서비스 (FSM = 클라이언트)
| 이름 | 타입 | 기본 이름(파라미터) | FSM | 서버 | 근거 |
|---|---|---|---|---|---|
| scan action | mission_interfaces/MoveToScanPose | `move_to_scan_pose`(`scan_action_name`, `:155-156`) | ActionClient(`:170-171`) | manip ActionServer(`server.py:74-75`) | A2_SCAN_POSE |
| nav service | mission_interfaces/MoveBaseLateral | `move_base_lateral`(`nav_service_name`, `:148-149`) | client(`:172-173`) | `move_base_lateral`/mock_nav | A3_MOVE_TO_TRAY / RETURN |
| task_list service | mission_interfaces/GetTaskList | `/mission_a/task_list`(`task_list_service_name`, `:116-117`) | client(`:199-200`) | tray_manage(`/perception/get_task_list`로 remap) | A1_MONITOR(옵션) |

### 5.3 인터페이스 필드 정의(`mission_interfaces/`)
- **MoveToScanPose.action**: Goal `string preset_id` / Result `bool success, string message` / Feedback `float32 progress`.
- **MoveBaseLateral.srv**: Req `string direction("left"|"right"), float32 distance_mm` / Resp `bool arrived, float32 lateral_error_mm, string message`.
- **GetTaskList.srv**: Req `float32 timeout_sec, uint16 frame_count` / Resp `bool success, string message, bool screen_detected, bool all_counts_recognized, uint16 frames_used, TaskItem[] parts`.
- **TaskItem.msg**: `string name, int32 count`.

---

## 6. perception→manipulation 핸드오프 & C1/C2/C3 게이트

### 6.1 핸드오프 추적(검출 → 파지)
1. **검출/투영**: `detector_node`(`/detections`) → `wrist_task_grasp_planner_node`가 depth로 3D 투영 → `/perception/wrist/target_one_pose`(PoseStamped, `base_link`).
2. **FSM 수신**: 콜백 `_on_target_pose`가 `last_target_pose` 저장(`:268-269`). A2_SCAN이 `frame_id=='base_link'` 검증 후 `current_target_pose`로 consume(`:559-567`) → **A3_PICK**(`:571`).
3. **파지 트리거**: A3_PICK 진입 시 `/attach_cmd='pick'` 발행(`_on_enter`, `:487`).
4. **manip 반응**: 서버가 `PickSkill` 수행, **성공 시에만** `/attached_object=class` 발행(`server.py:164`); 실패 시 미발행(`server.py:174`).
5. **파지 확정**: FSM `_on_attached_object`가 `last_attached_object` 저장(`:271-272`). A3_PICK이 이를 보면 `current_pick_class`로 래치 후 전이(`:583-587`).

### 6.2 C1 / C2 / C3 게이트 (A3_PLACE Phase1, `:614-634`)
| 게이트 | 의미 | 판정 코드 | 미통과 시 |
|---|---|---|---|
| **C1** | pick class 존재 | `if not self.current_pick_class` (`:615-618`) | RECOVERY |
| **C2** | 파지 유지(드롭 아님) | `if self.last_attached_object == ''` (`:619-623`; 이동 중에도 `:594-597`) | RECOVERY(무차감) |
| **C3** | place 위치 유효(옵션) | `use_place_pose_check ∧ _place_pose_valid_now()` (`:624-629`) | 유효 전 release 금지·대기, timeout 시 RECOVERY |

**C3 판정 세부**(`_place_pose_valid_now`, `:290-301`): ① 최신 스냅샷 `valid==True`, ② 신선도 `now-recv_time ≤ 1.0s`(`:296`), ③ 디바운스 `now - _place_valid_since ≥ place_pose_valid_debounce_sec`(기본 0.3s, `:301`). `valid`가 False로 떨어지면 `_place_valid_since` 리셋(플랩 차단, `:285-286`). 콜백은 `/perception/place_pose_valid` JSON에서 `valid` 키만 사용(`:274-288`).

**차감(Phase2)**: 게이트 통과 후 `/detach_cmd` 발행→`_release_issued=True`(`:630-631`), 다음 틱부터 `/attached_object==''`(해제 확정) 수신 시 `task_list.decrement` + `placed_count++`(`:636-645`).

---

## 7. 선행조건·환경·안전경계

### 7.1 실 통합 필수 선행조건 (정본: [RESULT §6.2](MISSION_A_PHASE2_RESULT.md))
1. **colorizer off** — bringup 기본 depth는 colorized `rgb8`(metric 아님 + 대용량으로 크로스-PC 미전송). `colorizer.enable*:=false`로 `16UC1` 전환해야 cross-PC 스트림(~4.4Hz). ([RESULT](MISSION_A_PHASE2_RESULT.md):81-83)
2. **static TF 브리지 필수** — 로봇 TF(`base_link→…→camera_r_link`)와 realsense(`camera_right_link→…`)가 끊겨 있어 `perception_live`의 `camera_r_link→camera_right_link`(identity)가 잇는다. `publish_camera_tf:=true` 유지(`perception_live.launch.py:51-52, 94-102`).
3. **카메라 단일 소유(FIX-4)** — bringup이 `camera_right`를 올리므로 별도 카메라 launch 금지(USB 충돌). ([RESULT](MISSION_A_PHASE2_RESULT.md):87)
4. **manip 서버 전제** — `ffw bringup + MoveIt + TRAC-IK`로 `move_group`/`/joint_states` 가용 후 `mission_a_manip`. (`mission_a_manip.launch.py:4-5`)
5. **INIT 게이트** — FSM은 scan 서버 준비 + `/manipulator_state=IDLE`까지 대기(60s 후 강행, `:501-509`).

### 7.2 안전경계
- A3_PLACE/PICK 단계부터 **실 팔이 동작**한다. e-stop 준비. ([RESULT](MISSION_A_PHASE2_RESULT.md):99)
- 실 베이스 측방 이동(`move_base_lateral`)은 `max_duration_sec=12`(< FSM `base_move_timeout_sec=30`), `wait_for_odom_sec`로 odom 신선도 가드, 콜드 첫 호출은 `distance_mm:=0` 무이동 경로로 검증 가능(`move_base_lateral.launch.py:12-14, 30-36`).
- 코드 분석/문서는 본 매뉴얼 범위. **실 로봇 모션 실행은 사용자(운용자) 감독 영역.**

---

## 8. 알려진 제약 / TODO & 코드-문서 불일치

소스 재검증으로 확정한 항목(추측 아님):

| # | 항목 | 근거 | 영향/비고 |
|---|---|---|---|
| 1 | **A1_MONITOR 무한 대기** | `STATE_TIMEOUT`에 90s 존재(`:76`)하나 `_run_a1_monitor`(`:511-523`)가 `_timed_out()`을 호출하지 않음 | perception가 task_list 미발행 시 FSM이 무기한 정지. (TIMEOUT_A1_MONITOR 상수는 사실상 미사용 경로) |
| 2 | **recovery_count 미리셋** | init `:209`, 증가 `:683`, 리셋 코드 없음 | 재시도 예산이 사이클별이 아닌 **미션 누적**. 초반 드롭이 누적되면 후반 사이클 예산 감소 |
| 3 | **MANUAL_WAIT 탈출 미구현** | `_run_manual_wait` = `pass` + TODO(`:693-695`) | 재시도 3회 초과 시 노드 정지(운용자 재기동 필요). Phase2 과제 |
| 4 | **`rescan_each_cycle` 미사용** | 선언 `:146-147`, 다른 참조 없음 | 파라미터 토글이 동작에 영향 없음(항상 A3_RETURN_TO_BOX→A2_SCAN_POSE 루프) |
| 5 | **`GRASP_ASSESSMENT_ENABLED` 미사용** | 정의 `:42`, 사용처는 기동 로그뿐(`:246`) | 파지 품질 사후 게이트 미통합(Hand-Eye Calib 후 활성 예정 주석) |
| 6 | **`/detections` 구독 미사용** | sub `:179-181`, 콜백 저장만(`:265-266`), 로직 참조 없음 | FSM은 wrist target만 사용. 향후 직접 검출 매핑 대비 |
| 7 | **C3 기본 비활성** | `use_place_pose_check` 기본 False(`:142-143`, `mission_a.launch.py:37`) | 기본 구성에서 C3는 건너뜀(단계적 통합 의도). 실 운용 시 `use_place_pose_check:=true` |
| 8 | **nav=stub 기본** | `nav_mode` 기본 'stub'(`:132`), 실 콜드 디스커버리 TODO | 베이스 이동은 기본적으로 즉시 성공(외부 호출 없음). 실 연동은 `nav_mode:=service` + 로봇 PC `move_base_lateral` |

### 정본 문서 관련 정정(프롬프트 가정 ≠ 레포 현실)
- `MISSION_A_PHASE2_STATUS.md`·`MISSION_A_PHASE2_HANDOFF.md`는 **레포에 존재하지 않는다**.
- 실제 정본은 [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md) 하나이며, "FIX-1~6"은 별도 HANDOFF가 아니라 **RESULT.md §6.2 본문에 인라인**으로 기술되어 있다(FIX-1 단일 executor, FIX-2 joint_state 신선도 가드, FIX-3 error_code, FIX-4 카메라 단일소유, FIX-6 Dynamixel 버스). 

### 빌드 위생(코드 외, 참고)
워킹트리에 미정리 `*.bak`이 있다: `manipulation/setup.py.bak`, `manipulation/setup.py.bak2`, `scripts/run_integration_demo.sh.bak`. 본 문서화 범위 밖이며 빌드에 영향 없음(정리는 별도 판단).
