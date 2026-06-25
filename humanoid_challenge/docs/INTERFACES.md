# Mission A 인터페이스 계약 (INTERFACES)

> Mission A 통합의 토픽·액션·서비스·메시지 계약을 **소스 근거로 일원화**한 레퍼런스.
> 인용 경로는 `humanoid_challenge/` 루트 기준. FSM 시점의 동작 흐름은 [MISSION_A_STATE_MACHINE_MANUAL.md](MISSION_A_STATE_MACHINE_MANUAL.md), 실 통합 확정 계약 7항목은 [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md) §1이 정본.
> **perception 노드 I/O 상세**(OCR JSON 스키마 등)는 보호문서 [PERCEPTION_INTERFACE.md](PERCEPTION_INTERFACE.md)를 참조.

---

## 1. 토픽 계약 (생산자/소비자)

| 토픽 | 타입 | 생산자(pub) | 소비자(sub) | 의미 |
|---|---|---|---|---|
| `/active_mission` | std_msgs/String | mission_a (`mission/mission/mission_a.py:203`) | (모니터링) | 현재 미션 ID = "A" |
| `/attach_cmd` | std_msgs/String | mission_a (`mission_a.py:204`) | manip 서버 (`manipulation/manipulation/mission_a_manipulation_server.py:64`), mock_manipulation_a | "pick" — A3_PICK 파지 트리거 |
| `/detach_cmd` | std_msgs/String | mission_a (`mission_a.py:205`) | manip 서버 (`server.py:66`), mock_manipulation_a | class — A3_PLACE 해제 트리거 |
| `/manipulator_state` | std_msgs/String | manip 서버 (`server.py:63`), mock_manipulation_a | mission_a (`mission_a.py:176`) | "IDLE" \| "BUSY" — INIT 게이트 |
| `/attached_object` | std_msgs/String | manip 서버 (`server.py:62`), mock_manipulation_a | mission_a (`mission_a.py:185`) | 파지 class \| "" — **C2 래치** |
| `/detections` | perception/PartDetectionArray | detector_node | mission_a (`mission_a.py:179`, 미사용), wrist_planner | YOLO 부품 검출 배열 |
| `/perception/wrist/target_one_pose` | geometry_msgs/PoseStamped | wrist_task_grasp_planner_node, mock_perception_a | mission_a (`mission_a.py:182`), manip 서버 (`server.py:71`) | 파지 타겟(base_link 3D) |
| `/perception/task_list` | mission_interfaces/**GetTaskList.Response** | tray_manage_node, mock_perception_a | mission_a (`mission_a.py:190`), manip 서버 (`server.py:68`) | 목표 부품/수량(타입드 토픽) |
| `/perception/place_pose_valid` | std_msgs/String (JSON) | place_pose_valid_node, mock_perception_a | mission_a (`mission_a.py:194`) | **C3** place 유효성 |

> **타입 주의**: `/perception/task_list`는 서비스 응답 타입 `GetTaskList.Response`를 토픽 메시지로 사용한다(`mission_a.py:190-191`, `mission_a.py:17` 주석). perception-new 타입드 토픽 계약.

## 2. 액션 / 서비스 계약 (FSM = 클라이언트)

| 이름 | 타입 | 기본 이름(FSM 파라미터) | 클라이언트 | 서버 |
|---|---|---|---|---|
| scan action | mission_interfaces/MoveToScanPose | `move_to_scan_pose` (`scan_action_name`, `mission_a.py:155`) | mission_a ActionClient (`mission_a.py:170`) | manip ActionServer (`server.py:74`), mock_manipulation_a |
| nav service | mission_interfaces/MoveBaseLateral | `move_base_lateral` (`nav_service_name`, `mission_a.py:148`) | mission_a client (`mission_a.py:172`) | move_base_lateral_node, mock_navigation_a |
| task_list service | mission_interfaces/GetTaskList | `/mission_a/task_list` (`task_list_service_name`, `mission_a.py:116`) | mission_a client (`mission_a.py:199`) | tray_manage_node `/perception/get_task_list`(remap) |

## 3. 인터페이스 정의 (`mission_interfaces/`)

근거: `mission_interfaces/CMakeLists.txt:9-17` + 각 정의 파일.

**MoveToScanPose.action**
```
# Goal
string preset_id          # "" = manipulation 기본 스캔 포즈 preset
---
# Result
bool success
string message
---
# Feedback
float32 progress          # 0.0..1.0 (mock 은 0 가능)
```

**MoveBaseLateral.srv**
```
# Request
string direction          # "left" | "right"
float32 distance_mm       # 예: 675.0
---
# Response
bool arrived
float32 lateral_error_mm  # 부호 있는 도착 잔차
string message
```

**GetTaskList.srv**
```
# Request
float32 timeout_sec
uint16 frame_count
---
# Response
bool success
string message
bool screen_detected
bool all_counts_recognized
uint16 frames_used
mission_interfaces/TaskItem[] parts
```

**GetTrayDetections.srv** (보조)
```
# Request: float32 timeout_sec, uint16 frame_count
# Response: bool success, string message, bool tray_detected, uint16 frames_used,
#           Detection2D[] trays, Detection2D[] parts
```

**TaskItem.msg**: `string name` / `int32 count`

**Detection2D.msg**: `std_msgs/Header header`, `int32 class_id`, `string class_name`, `float32 confidence`, `int32[] bbox`, `string source`, `float32 center_x`, `float32 center_y`, `float32[] mask_x`, `float32[] mask_y`

**perception 메시지** (`perception/CMakeLists.txt:13-17`): `PartDetection.msg`, `PartDetectionArray.msg`(`Header header`, `PartDetection[] detections`).

## 4. 불변식 / 스키마

### 4.1 그래스프 래치 (C2 보호)
`/attached_object=class`는 **`PickResult.SUCCESS`(=`GraspSkill.assess_stable`, 파지 관측) 시점에만** 래치된다. 파지 실패 시 미발행(`server.py:16, 164, 174`). 손실 시 `""`로 복귀 → FSM C2 게이트가 오선언을 0으로 막음(`mission_a.py:594-597, 619-623`).

### 4.2 C3 place_pose_valid JSON
`/perception/place_pose_valid` 본문은 JSON. FSM은 **`valid`(bool) 키만** 사용(`mission_a.py:274-288`). 판정 조건(`_place_pose_valid_now`, `mission_a.py:290-301`):
- `valid == true`
- 신선도: `now - recv_time ≤ 1.0s`
- 디바운스: `now - _place_valid_since ≥ place_pose_valid_debounce_sec`(기본 0.3s)
- `valid`가 false로 떨어지면 `_place_valid_since` 리셋(플랩 차단).

### 4.3 픽/플레이스 primitive (manip 내부)
`PickSkill.pick → PickResult.SUCCESS`(=`GraspSkill.assess_stable`), `PlaceSkill.place(pose, arm, …) → PlaceResult.SUCCESS`(gripper.open = release; planning-scene detach는 호출자). (정본 [RESULT](MISSION_A_PHASE2_RESULT.md) §1.7)

## 5. 부품 class ↔ 한글명

`mission/mission/task_list.py:34-39` (`CLASS_TO_PART_NAME`):

| class | 한글 | class | 한글 |
|---|---|---|---|
| `flange_nut` | 플랜지 너트 | `hex_nut` | 육각 너트 |
| `gear_ring` | 기어 링 | `dome_nut` | 돔 너트 |
| `spacer_ring` | 스페이서 링 | | |

> OCR 별칭 브리지: tray_manage는 "dom nut"(e 없음)을 발행 → `domnut → dome_nut` 매핑이 핵심(`task_list.py:16-30`).

---

*확정 계약 7항목(실 통합)의 정본 표는 [MISSION_A_PHASE2_RESULT.md](MISSION_A_PHASE2_RESULT.md) §1. perception 노드별 I/O 상세는 [PERCEPTION_INTERFACE.md](PERCEPTION_INTERFACE.md).*
