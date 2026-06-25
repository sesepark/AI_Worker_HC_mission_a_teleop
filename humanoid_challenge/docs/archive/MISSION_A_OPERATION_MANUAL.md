> ⚠️ **아카이브됨 (2026-06-25 재구조화)** — 이 운용 매뉴얼은 불완전(FSM 흐름에 A3_MOVE_TO_TRAY/RETURN/VERIFY/RECOVERY/MANUAL_WAIT 누락)하여 완전판 [../MISSION_A_STATE_MACHINE_MANUAL.md](../MISSION_A_STATE_MACHINE_MANUAL.md)로 **대체**되었습니다. 운용 STEP은 [../RUNBOOK.md](../RUNBOOK.md), 계약 §5는 [../INTERFACES.md](../INTERFACES.md)에 흡수됨. (원 파일명에 공백 결함이 있어 이동 시 교정.) 참고용으로 보존.

# Mission A 통합 시스템 — 구성 및 실행 매뉴얼

Mission A 통합 시스템(perception → manipulation → mission FSM)의 운용 구조, 인터페이스 계약,
실행 절차를 정리한 매뉴얼입니다. 부품(너트·링) 인식 → 파지 → 적재 전체 사이클을 구동합니다.

---

## 1. 시스템 아키텍처

운용은 **로봇 PC**와 **메인 PC** 2대로 구성되며, 동일 ROS 네트워크에서 통신합니다.

| PC | 실행 대상 | 역할 |
|---|---|---|
| **로봇 PC** | 로봇 bringup + MoveIt | 하드웨어 제어, 카메라(camera_right), `move_group`/`/joint_states` 제공 |
| **메인 PC** | 모든 응용 패키지 | manipulation 서버 + perception + mission FSM 실행 |

메인 PC가 로봇 PC의 `move_group`을 같은 네트워크 ROS 통신(domain 30)으로 **원격 제어**합니다.
즉, `mission_a_manip.launch.py`·`mission_a_real.launch.py` 등 응용 패키지는 **모두 메인 PC에서 실행**합니다.

**공통 환경변수 (양쪽 PC 동일하게 설정):**

```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

---

## 2. 동작 흐름 (FSM)

```
INIT → A1_MONITOR → A2_SCAN_POSE → A2_SCAN → A3_PICK → A3_PLACE → (반복) → DONE
```

| 상태 | 동작 |
|---|---|
| **A1_MONITOR** | task_list 확정 — 작업 대상 부품 종류/수량 결정 |
| **A2_SCAN_POSE** | 스캔 자세(CAPTURE_JOINTS)로 이동 (`move_to_scan_pose` action) |
| **A2_SCAN** | wrist 카메라로 부품 검출 → 3D target(base_link) 수신 |
| **A3_PICK** | 파지 실행 → 성공 시 `/attached_object`=class 래치 (게이트 C2) |
| **A3_PLACE** | C1∧C2∧C3 게이트 통과 → place → `/detach_cmd` → 잔여 수량 차감 |
| **DONE** | 모든 부품 적재 완료 |

대상 부품 5종: `dome_nut`, `flange_nut`, `gear_ring`, `hex_nut`, `spacer_ring`

---

## 3. 사전 필수 설정

실행 전 반드시 확인해야 하는 3가지 설정입니다.

### 3.1 Depth colorizer 비활성화 (필수)

bringup 기본값은 depth 를 colorized `rgb8` 로 발행합니다. 이 경우 (a) metric 깊이가 아니라 3D 투영이
불가하고, (b) 용량이 커서 cross‑PC 로 전송되지 않습니다. bringup 시 colorizer 를 끄면 depth 가
`16UC1` 로 발행되어 메인 PC 로 스트림됩니다(~4.4Hz).

```bash
colorizer.enable1:=false colorizer.enable2:=false
```

### 3.2 Static TF — perception_live 가 발행 (기본 유지)

로봇 TF 트리(`base_link → … → camera_r_link`)와 realsense TF(`camera_right_link → …`)가 분리되어
있습니다. `perception_live` 가 발행하는 `camera_r_link → camera_right_link`(identity) static TF 가
둘을 연결합니다. `publish_camera_tf:=true`(기본값) 를 유지하십시오. (없으면 wrist 노드가 target 미발행.)

### 3.3 카메라 단일 소유

`ffw_sg2_follower_ai` 가 camera_right 를 기동하므로, **별도 카메라 launch 를 동시에 실행하지 마십시오.**
(중복 기동 시 camera_right USB 충돌 `RS2_USB_STATUS_BUSY` 발생.)

---

## 4. 실행 절차

각 STEP 을 순서대로 실행하고, 표시된 확인 항목으로 정상 동작을 검증한 뒤 다음 단계로 진행합니다.

### STEP A — 로봇 PC: bringup + MoveIt

```bash
# [로봇 PC]
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py \
     colorizer.enable1:=false colorizer.enable2:=false \
     tf_publish_rate1:=10.0 tf_publish_rate2:=10.0

ros2 launch ffw_moveit_config moveit.launch.py
```

**확인 (메인 PC 에서):**
- `move_group`, `/joint_states`, camera 토픽이 메인 PC 에서 보임
- depth encoding 이 `16UC1`:

```bash
ros2 topic echo /camera_right/camera_right/depth/image_rect_raw --field encoding --once   # → 16UC1
```

### STEP B — 메인 PC: manipulation 서버

```bash
# [메인 PC]
ros2 launch manipulation mission_a_manip.launch.py
```

**확인 (정상 기동 신호, 아래 순서로 로그 출력):**
- `move_group action servers ready` → `Joint states ready`
- `씬 초기화 완료: 모든 collision objects 제거됨`
- `mission_a_manipulation_server ready (real MoveIt)`
- `/move_to_scan_pose` action 노출 및 `/manipulator_state` = IDLE 발행

```bash
ros2 action list | grep move_to_scan_pose       # → /move_to_scan_pose
ros2 topic echo /manipulator_state --once        # → IDLE
```

> ⚠️ **안전**: 이 단계부터 실제 팔이 동작할 수 있습니다. e‑stop 을 준비하십시오.

### STEP C — 메인 PC: perception + mission FSM

```bash
# [메인 PC]
ros2 launch mission mission_a_real.launch.py \
     mock_monitor_ocr:=true use_place_pose_check:=true
```

**확인:**
- 부품 검출 유입 (5종, base_link 3D target 발행)
- FSM 상태천이: `INIT → A1_MONITOR → A2_SCAN_POSE → A2_SCAN → A3_PICK`

```bash
ros2 topic echo /detections --once                          # 부품 검출
ros2 topic echo /perception/wrist/target_one_pose --once    # base_link 3D target
```

> manip 서버가 `/manipulator_state`=IDLE 을 발행하면 FSM 이 `INIT → A1_MONITOR` 로 자동 진행합니다.

### STEP D — 사이클 실행

부품 5개를 **스캔 자세에서 wrist 카메라 FOV(트레이)** 에 배치합니다.

| 단계 | 동작 |
|---|---|
| A3_PICK | 파지 → `/attached_object`=class 래치 (C2) |
| A3_PLACE | C1∧C2∧C3 게이트 통과 → `/detach_cmd` → `/attached_object`="" → 잔여 차감 |
| DONE | 적재 N (모든 부품 처리 완료) |

### STEP E — 다사이클

manip 서버 재기동 없이 `mission_a_real` 을 연속 실행하여 사이클을 반복합니다.

---

## 5. 인터페이스 계약

### 5.1 Manipulation 토픽 (모두 `std_msgs/String`)

| 토픽 | 의미 |
|---|---|
| `/manipulator_state` | manipulator 상태 ("IDLE" 등) |
| `/attached_object` | 파지된 부품 class, 미파지 시 "" |
| `/attach_cmd` | 파지 트리거 ("pick") |
| `/detach_cmd` | 해제 대상 부품 class |

### 5.2 픽/플레이스 트리거 방식

- **A2_SCAN_POSE** = `move_to_scan_pose` action
- **A3_PICK** = `/attach_cmd` 발행 → `/attached_object` 대기
- **A3_PLACE** = C1∧C2∧C3 게이트 → `/detach_cmd` → `/attached_object`=="" → 차감

### 5.3 액션·서비스 정의

- **`MoveToScanPose.action`**: Goal `string preset_id` / Result `bool success, string message` / Feedback `float32 progress`
- **`GetTaskList.srv`**: Req `float32 timeout_sec, uint16 frame_count` / Resp `bool success, string message, bool screen_detected, bool all_counts_recognized, uint16 frames_used, TaskItem[] parts{string name, int32 count}`

### 5.4 C3 place 유효성

`/perception/place_pose_valid` (JSON) 의 **`valid`(bool) 키**를 사용합니다. 신선도 ≤1s, 디바운스 0.3s
(플랩 차단) 적용.

### 5.5 그래스프 래치

`/attached_object`=class 는 **파지가 확정된 시점에만** 래치되며, 손실 시 "" 로 복귀합니다. → C2 게이트로
오선언을 방지합니다.

### 5.6 Pick/Place primitive

`PickSkill.pick → PickResult.SUCCESS` (그래스프 관측 = `GraspSkill.assess_stable`),
`PlaceSkill.place(pose, arm, …) → PlaceResult.SUCCESS` (gripper.open = release).

---

## 6. 시스템 구성 요소

| 구성 요소 | 역할 |
|---|---|
| `manipulation/mission_a_manipulation_server.py` | 실 manipulation 서버 (MoveIt 기반 픽/플레이스, 래치 보존) |
| `manipulation/launch/mission_a_manip.launch.py` | manipulation 서버 기동 |
| `perception/.../place_pose_valid_node.py` | C3 place 유효성 판정 (`/perception/place_pose_valid` 발행) |
| `perception/launch/perception_live.launch.py` | detector + tray + wrist + static TF + place_pose_valid 통합 launch |
| `mission/launch/mission_a_real.launch.py` | perception + mission FSM 통합 실행 |
| `mission/launch/mission_a.launch.py` | task_list 서비스/토픽 경로 설정 |

---

## 7. 안전 게이트

place 동작은 **C1∧C2∧C3 게이트를 모두 통과한 경우에만** 실행됩니다.

| 게이트 | 조건 |
|---|---|
| **C2 (파지 래치)** | `/attached_object`=class 가 파지 확정 시점에만 래치 → 오선언 방지 |
| **C3 (place 유효성)** | `/perception/place_pose_valid` 의 `valid` 키 기반 (신선도 ≤1s, 디바운스 0.3s) |

게이트 미통과 시 적재가 진행되지 않습니다(잘못된 파지/배치 보호).

---

## 8. 튜닝 참고

- depth/color 가 네트워크로 ~4Hz 로 throttle 됩니다. RGB‑D sync 가 흔들릴 경우
  `config/wrist_projection/params.yaml` 의 `sync_slop` 을 `0.10 → 0.2~0.3` 으로 상향하십시오.
