# 미션 A 시나리오 코드 계획
> **최종 업데이트**: 2026-06-22 (Perception runtime을 Mission A 5개 노드로 정리)
> **목표**: 시험 기간 전 미션 A end-to-end 자율 동작 완료  
> **배점**: 원격 40점 / 자율 60점  
> **상태 범례**: ✅ 완료 | 🔄 진행 중 | ⬜ 개발 예정 | ⚠️ 블로커  
> **연관 문서**: [PERCEPTION_INTERFACE.md](./PERCEPTION_INTERFACE.md) (토픽/노드 인터페이스) · [PERCEPTION_LOCAL_SETUP.md](./PERCEPTION_LOCAL_SETUP.md) (실행 검증·런북)

> **🟢 Perception 구동 현황 (2026-06-22)**: runtime은 `monitor_ocr_node` /
> `monitor_ocr_viewer` / `detector_node` / `tray_manage_node` / `wrist_task_grasp_planner_node`
> 5개 실행 노드만 유지한다. 삭제된 head/wrist projection·pointcloud·grasp PCD 노드는 현재 Mission A
> 통합 경로에서 launch되지 않는다.
> mission_a 가 구독할 최종 target 토픽은 **`/perception/wrist/target_one_pose`** (planner 출력)로 확정.
>
> **🆕 tray management 통합**: `tray_manage_node`가 OCR 결과와 wrist image를 받아
> **`/perception/task_list`**(`GetTaskList_Response`)와 `/perception/tray_roi`를 발행.
> mission_a 가 이를 구독하면 task list를 공용 typed 인터페이스로 받을 수 있음.
> (mission_a 코드 반영 완료 — sim 검증 통과. [PERCEPTION_INTERFACE.md](./PERCEPTION_INTERFACE.md) §10~11 참고)

---

## 레포 구조

> **2026-05-30 업데이트** — Step 1에서 팀별 코드 통합 완료 (방식 A: 팀 레포 클론 후 복사).

```
~/AI_Worker_HC/                                # snu-shape/AI_Worker_HC.git (main)
├── ai_worker/                                 # ROBOTIS 공식 — 수정 금지
├── humanoid_challenge/                        # 휴머노이드 챌린지 Mission A 팀 개발 패키지
│   ├── perception/                           # OCR / detector / tray task / wrist target
│   ├── manipulation/                # MoveIt client, GPD, grasp_filter
│   ├── mission/                               # System 팀 미션 (ament_python 패키지 `mission`)
│   │   ├── package.xml / setup.py             # P0 패키지화 (2026-05-30)
│   │   ├── mission/{mission_a,task_list,sim_driver}.py  # FSM + 잔여수량 + sim
│   │   └── README.md
│   └── docs/                                  # 설계 문서
│       ├── MISSION_A_SCENARIO_PLAN.md         # (본 문서)
│       └── PERCEPTION_INTERFACE.md
├── physical_ai_tools/                         # ROBOTIS 공식 — 수정 금지
├── robotis_applications/                      # ROBOTIS Vuer 등 기존 robotis_applications 패키지
│   ├── robotis_applications/                  # 스캐폴드 메타 패키지 (ROBOTIS)
│   ├── robotis_vuer/                          # ROBOTIS Vuer 도구
│   └── docker/                                # ROBOTIS 도커 컨피그
└── README.md
```

### 팀별 소스 레포 (개발자 워크스페이스)

| 팀 | 원본 레포 | 로컬 클론 위치 | 비고 |
|----|----------|---------------|------|
| Perception | `hublemon/Humanoid-Challenge-Perception` | `~/perception-ws/` | `~/robotis_ros2_ws/`는 root 소유라 우회 |
| Manipulation | `chlgkals07/ai-worker-ws` | `~/ai-worker-ws/` | 기대 경로와 동일 |

> **동기화 정책**: 팀별 코드 업데이트는 각자 working 워크스페이스에서 작업 후 PR. AI_Worker_HC `humanoid_challenge/` 아래 신규 패키지들은 검증된 시점 스냅샷으로 사용. 동기화 자동화는 후속 작업 (REPO_INTEGRATION_GUIDE.md TBD).

---

## State Machine 전체 구조

```
[*] → INIT
INIT → A1_MONITOR       : /active_mission="A" 발행
A1_MONITOR → A2_SCAN    : /monitor_ocr/result 수신 AND screen_detected=true
A1_MONITOR → A1_MONITOR : 실패 (재시도) / 10초 후 fallback OK
A2_SCAN → A3_PICK       : /perception/wrist/target_one_pose 수신
A3_PICK → A3_PLACE      : /attached_object non-empty
A3_PICK → RECOVERY      : timeout (파지 실패)
A3_PLACE → VERIFY       : /attached_object=""
A3_PLACE → RECOVERY     : timeout (place 실패)
VERIFY → A2_SCAN        : 잔여 수량 > 0
VERIFY → DONE           : 잔여 수량 = 0
RECOVERY → A3_PICK      : 재시도 (max 3회)
RECOVERY → MANUAL_WAIT  : 재시도 초과
MANUAL_WAIT → A3_PICK   : 운용자 재개 신호
DONE → [*]
```

---

## Step별 로직 상세

### Step 1 — INIT (환경 초기화)

**목적**: MoveIt planning scene 등록, CM 노드 확인, 미션 시작 선언

**로직**
```python
1. /active_mission = "A" 발행  →  CM이 Zone A planning scene 자동 등록
2. /manipulator_state = IDLE 수신 대기
3. head/wrist camera timestamp 유효 확인
4. 조건 충족 시 → A1_MONITOR 진입
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/manipulator_state` | IDLE 확인 후 다음 Step 진입 |

**발행 토픽**
| 토픽 | 내용 |
|------|------|
| `/active_mission` | `"A"` 발행 |

**Timeout**: 초기화 전체 목표 30초 / hard 1분

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `competition_manager_node.py` | Manipulation | ✅ 완료 |
| `environment.py` → `setup_zone_a()` | Manipulation | ✅ 완료 |
| `mission_a.py` (INIT 상태) | System | 🔄 stub 생성 (2026-05-30) |

**요청사항**
- Manipulation: `competition_manager` → 시스템 팀 관리 노드로 이전
- Manipulation: `/active_zone` → `/active_mission` 명칭 변경
- Manipulation: `/cm_state` → `/manipulator_state` 명칭 변경

---

### Step 2 — A1_MONITOR (지령 모니터 인식)

**목적**: 지령 모니터 OCR 파싱 → task_list 생성 → OK 사인 출력

**로직**
```python
1. monitor_ocr_node 실행 확인
2. /monitor_ocr/result 구독 → screen_detected=true + parts 파싱 성공 시
   → task_list 내부 저장
   → 모니터에 OK 사인 출력
   → A2_SCAN 진입
3. 파싱 실패 시: 재시도
   → 10초 경과 후 fallback: 강제 OK 출력 후 진입 (점수 10점 확보)
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/monitor_ocr/result` | JSON: parts 배열 + screen_detected |

**발행 토픽**
| 토픽 | 내용 |
|------|------|
| OK 사인 | 모니터 출력 (구현 방식 TBD) |

**Timeout**: 목표 1분 / hard 1분 30초  
- 모니터 탐색·crop: 30초  
- OCR 파싱: 20초  
- OK sign 출력: 5~10초

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `monitor_ocr_node.py` | Perception | ⚠️ ocr_venv 의존성 블로커 (노드 로직 정상) + 실모니터 YOLO 전환 중 |
| `mission_a.py` (A1_MONITOR 상태) | System | 🔄 stub 생성 (2026-05-30) |

> ⚠️ **monitor_ocr 블로커**: ocr_venv 의존성 누락으로 미실행. 해소법은 [PERCEPTION_LOCAL_SETUP.md](./PERCEPTION_LOCAL_SETUP.md) "함정 ②". 해소 전까지 A1_MONITOR는 **10초 fallback OK 경로**로 진행(점수 10점 확보) 가능.

**요청사항**
- Perception: `/monitor_ocr/result` → Action 구조 변환 검토
- Perception: 코드 AI_Worker_HC 레포 브랜치에 업로드

---

### Step 3 — A2_SCAN (부품 상자 스캔)

**목적**: 노란 상자 ROI 검출 → 부품 탐지·분류 → target 선정

**로직**
```python
1. detector_node 구동 확인
2. /detections 구독 → 노란 상자 ROI 내 부품 검출
3. task_list 기준 현재 target 부품 class 확인
   (planner가 /monitor_ocr/result를 직접 구독해 task 필터링까지 수행)
4. target class 부품 중 grasp score 최상 후보 1개 → planner가 선정
5. /perception/wrist/target_one_pose 수신 (wrist_task_grasp_planner_node 출력)
6. /detector_debug_image overlay 모니터 표시
7. → A3_PICK 진입
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/detections` | PartDetectionArray (5종 부품 bbox/class/confidence) |
| **`/perception/wrist/target_one_pose`** | task+grasp score 기준 최종 선정 부품 3D pose (base_link) — planner 출력 |

> ⚠️ 기존 `/target_pose`는 폐기. per-detection 중심점은 `/perception/wrist/target_pose`, 최종 1개는 `/perception/wrist/target_one_pose`.
> planner가 `/monitor_ocr/result` task list를 직접 구독하므로, mission_a는 target_one_pose 한 토픽만 보면 됨.

**발행 토픽**
| 토픽 | 내용 |
|------|------|
| (내부 target 선정 결과) | grasp_filter로 전달 |

**Timeout**: 목표 1분 / hard 1분 30초  
- 노란 box ROI 인식: 20초  
- 부품 검출·분류: 40초

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `detector_node.py` | Perception | ✅ 로봇 검증 (2026-05-30) |
| `wrist_task_grasp_planner_node.py` | Perception | ✅ 로봇 검증 — `/perception/wrist/target_one_pose` 발행 |
| `point_cloud_transformer_node.py` | Manipulation | ✅ 완료 |
| `mission_a.py` (A2_SCAN 상태) | System | 🔄 stub 생성 + 구독 토픽 정정 완료 (2026-05-30) |

**요청사항**
- Perception: command target 부품 `/detections`에서 별도 표시 필드 추가
- Perception: `/camera_left/points_base`, `/camera_right/points_base` 토픽명 확정
- Manipulation: 전달받은 grasp 후보 개수에 따라 grasp 물체 결정 로직

---

### Step 4 — (A3_PICK 전처리) Grasp Pose 계산

**목적**: PCD → GPD → grasp candidate 필터링 → best pose 1개 선택

**로직**
```python
1. /camera_right/points_base (PCD) → gpd_dual_view_node → /gpd/grasp_poses
2. /target_pose + /gpd/grasp_poses → grasp_filter.py
   → IK 가능 여부 확인
   → MoveIt 충돌 여부 확인
   → 작업 반경 내 여부 확인
3. best_grasp_pose 1개 선택 → bin_pick.py로 전달
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/camera_left/points_base` | base_link 기준 left PCD |
| `/camera_right/points_base` | base_link 기준 right PCD |
| `/gpd/grasp_poses` | PoseArray (GPD 출력) |
| `/target_pose` | IK seed로 활용 |

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `gpd_dual_view_node.py` | Manipulation | ✅ 완료 |
| `grasp_filter.py` | Manipulation | 🔄 개발 중 (~5.30) |
| `tf_transformer.py` | Manipulation | ✅ 완료 |

**Timeout**: Pick & Place 1회당 45초

> ⚠️ **Hand-Eye Calibration 미완료 시**: `/target_pose` 좌표 오차로 pick 실패  
> → 우회: `/attach_cmd`로 수동 attach 후 이후 Step 테스트 진행  
> → `GRASP_ASSESSMENT_ENABLED = False` (현재)

---

### Step 5 — A3_PICK (Pick 실행)

**목적**: grasp_filter best pose → 실제 파지 수행 → 파지 성공 확인

**로직**
```python
1. moveit_client.move_to_pose(pre_grasp)   # OMPL 충돌 회피
2. moveit_client.cartesian_move(grasp_pose) # Pilz LIN 직선 하강
3. gripper.close()  →  CM: GRASPING → ATTACHED 자동 전환
4. /attached_object non-empty 대기 (파지 확인)
5. 성공 → A3_PLACE 진입
6. 실패 (timeout) → RECOVERY
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/attached_object` | non-empty → 파지 성공 |
| `/manipulator_state` | GRASPING → ATTACHED 전환 모니터링 |

**발행 토픽**
| 토픽 | 내용 |
|------|------|
| `/attach_cmd` | 수동 attach (캘리브레이션 전 우회용) |

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `bin_pick.py` | Manipulation | ⬜ 개발 예정 |
| `moveit_client.py` | Manipulation | ✅ 완료 |
| `gripper_controller.py` | Manipulation | ✅ 완료 |
| `competition_manager_node.py` | System | ✅ 완료 |

**요청사항**
- Manipulation: `bin_pick.py` 내부 함수 → **Action 서버**로 감싸기 (success/fail/timeout 반환)

---

### Step 6 — A3_PLACE (트레이 검출 및 Place 실행)

**목적**: 파란 트레이 검출 → place pose 계산 → 부품 내려놓기

**로직**
```python
1. /tray_region 수신 (Perception 트레이 검출 노드)
2. 빈 슬롯 place pose 계산
3. moveit_client.move_to_pose(pre_place)    # OMPL
4. moveit_client.cartesian_move(place_pose) # Pilz LIN
5. gripper.open()  →  CM: RELEASING → IDLE 자동 전환
6. /attached_object = "" 대기 (내려놓기 확인)
7. 성공 → VERIFY 진입
8. 실패 (timeout) → RECOVERY
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| `/tray_region` | 파란 트레이 3D 위치 + 적재 가능 영역 (base_link 기준) |
| `/attached_object` | "" → place 완료 확인 |
| `/manipulator_state` | RELEASING → IDLE 전환 확인 |

**발행 토픽**
| 토픽 | 내용 |
|------|------|
| `/detach_cmd` | 수동 detach (필요 시) |

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `tray_manage_node` | Perception | ✅ 구현 — `/perception/task_list`, `/perception/tray_roi` |
| 트레이 base_link place 좌표 발행 | Perception | ⬜ 미구현 (현재 tray_roi 는 2D bbox) |
| `tray_place.py` | Manipulation | ⬜ 개발 예정 |
| `moveit_client.py` | Manipulation | ✅ 완료 |

**요청사항**
- Perception: wrist camera로 적재 가능 내부 영역 보정 로직 포함 개발
- Perception: `/tray_region` 메시지 타입 Manipulation과 합의 필요

---

### Step 7 — VERIFY (적재 검증 및 반복/완료 판단)

**목적**: 적재 성공 확인 → task_list 수량 차감 → 반복 or 완료

**로직**
```python
1. release + retreat 완료 후 tray ROI 재관측
2. 적재 object class == current target type 확인
3. 성공 시: remaining_quantity -= 1
   - remaining_quantity > 0 → A2_SCAN 복귀 (Step 3)
   - remaining_quantity = 0 → DONE
4. 실패 시 (wrong/no/multi-placement): RECOVERY → Step 3 재시도
```

**구독 토픽**
| 토픽 | 내용 |
|------|------|
| **`/perception/task_list`** | `GetTaskList_Response` typed task list. `total==0` → DONE |

> ✅ **구현됨**: mission_a 가 `/perception/task_list` 를 구독해 VERIFY 에서 잔여 감소를 확인.
> place 전 잔여(baseline) 대비 줄면 적재 검증 성공 → 잔여>0 A2_SCAN / 잔여0 DONE.
> tray_manage_node 미가동 시엔 service fallback 또는 레거시 자체 차감(성공 가정) 경로를 사용.

**분기 로직**
```
잔여 수량 > 0  →  A2_SCAN (Step 3) 복귀
잔여 수량 = 0  →  DONE
파지/적재 실패 →  RECOVERY → Step 3 재시도
```

**Timeout**
- 적재 성공 판정: 20초
- task state update: 즉시 (1~2초)
- 최종 완료 판정: 10~20초

**실행 코드**
| 코드 | 팀 | 상태 |
|------|----|------|
| `mission_a.py` (VERIFY 상태) | System | 🔄 stub 생성 (2026-05-30) |
| `tray_manage_node` (`/perception/task_list`) | Perception | ✅ 구현 |
| `mission_a` VERIFY 트레이 차감 연동 | System | ✅ 반영 (sim 검증) |

---

## Blackboard 키 (전 팀 공통 합의 필요)

> ⚠️ **2026-05-30 정정** — Task 1 Perception 코드 분석 결과 기존 토픽명 일부가 실제 발행명과 달라 정정. 자세한 내용은 [PERCEPTION_INTERFACE.md](./PERCEPTION_INTERFACE.md) 상단 "토픽명 정정 이력" 참조.

| 키 | 방향 | 타입 | 설명 | 합의 상태 |
|----|------|------|------|-----------|
| `/active_mission` | mission_a → CM | `std_msgs/String` | 현재 활성 미션 ("A"~"D") | ⬜ 미합의 |
| `/manipulator_state` | CM → mission_a | `std_msgs/String` | IDLE / GRASPING / ATTACHED / RELEASING | ⬜ 미합의 |
| `/monitor_ocr/result` | Perception → mission_a | `std_msgs/String` (JSON) | OCR 결과 | ✅ 확정 |
| `/detections` | Perception → mission_a | `PartDetectionArray` | 부품 검출 배열 | ✅ 확정 |
| **`/perception/wrist/target_one_pose`** | Perception → mission_a | `PoseStamped` | task list 반영 최종 target 1개 (wrist_task_grasp_planner_node) | 🔄 mission_a.py 적용 필요 |
| **`/perception/task_list`** | Perception(tray_manage_node) → mission_a | `mission_interfaces/srv/GetTaskList_Response` | canonical 부품명(공백), OCR frame metadata, mission_complete 상태. A1/VERIFY 소스 | ✅ 발행 구현 (mission_a 반영) |
| `/perception/tray_roi` | Perception(tray_manage_node) → downstream | `sensor_msgs/RegionOfInterest` | 최신 tray bbox | ✅ 발행 구현 |
| `/tray_region` (A3_PLACE place 좌표) | Perception → tray_place | TBD | 파란 트레이 **base_link 3D 위치/적재영역** — tray_contents 는 2D 카운트만이라 별도 필요 | ⬜ 미합의 |
| `/attached_object` | CM → mission_a | `std_msgs/String` | 파지 물체명 (""=없음) | ✅ 확정 |
| `/gpd/grasp_poses` | GPD → grasp_filter | `PoseArray` | grasp pose 후보 | ✅ 확정 |
| `/attach_cmd` | mission_a → CM | `std_msgs/String` | 수동 attach | ✅ 확정 |
| `/detach_cmd` | mission_a → CM | `std_msgs/String` | 수동 detach | ✅ 확정 |

**삭제된 잘못된 키** (이전 문서 표기 → 실제 발행 토픽으로 대체)
- ~~`/target_pose`~~ → `/perception/wrist/target_one_pose` (mission_a.py 구독) 또는 `/perception/wrist/target_pose` (per-detection)
- ~~`/camera_right/points_base`~~ → `/perception/wrist/mask_cloud` 또는 `/perception/wrist/target_pcd/<class>`

---

## 단기 Action Items

| # | 할 일 | 담당 | 기한 | 상태 |
|---|-------|------|------|------|
| 1 | Hand-Eye Calibration 착수 | Manipulation (조서영) | 이번 주 | ⬜ |
| 2 | TF tree `camera_right_depth_optical_frame`→`camera_r_link` launch 등록 | Perception | 이번 주 | 🔄 |
| 3 | Blackboard 키 이름 전 팀 합의 문서화 | System | 이번 주 | ⬜ |
| 4 | `grasp_filter.py` 완성 | Manipulation | 5.30 | 🔄 |
| 5 | `/perception/wrist/target_one_pose` base_link 발행 로봇 검증 | Perception | 5.31 | ✅ (2026-05-30) |
| 6 | `mission_a.py` State Machine 뼈대 생성 | System | 5.31 | ✅ (2026-05-30) |
| 7 | pick-place dummy 루프 1회 end-to-end 테스트 | Manipulation + System | 6.1 | ⬜ |
| 8 | `/tray_region` 발행 구현 및 인터페이스 합의 | Perception | 6.1 | ⬜ |
| 9 | `monitor_ocr` ocr_venv 의존성 해소 + `/monitor_ocr/result` 로봇 발행 검증 | Perception | 5.31 | ⚠️ 블로커 |
| 10 | `mission_a.py` A1~A3 실로직 1차 구현 (아래 "초안 작성 계획") | System | 6.1 | ⬜ |

---

## 전제 조건 / 블로커

| 블로커 | 영향 범위 | 우회 방법 | 상태 |
|--------|----------|-----------|------|
| Hand-Eye Calibration (목표 오차 5mm↓) | Step 4~6 전체 | `/attach_cmd` 수동 attach | ⬜ 미착수 |
| TF tree static transform 등록 | Step 3 3D 변환 | left wrist / ZED head만 임시 운용 | 🔄 |
| Blackboard 키 이름 전 팀 합의 | `mission_a.py` 작성 시작 불가 | — | ⬜ 미착수 |
| `bin_pick.py` 개발 완료 | Step 5 | 수동 attach로 Step 6~7만 테스트 | ⬜ |

---

## mission_a.py 초안 작성 계획

> **전략**: 외부 블로커(monitor_ocr venv, Manipulation Action 서버, Hand-Eye Calib)에 막히지 않도록
> **mission_a 내부 로직을 먼저 완성**하고, 외부 의존부는 stub/fallback/fake-publisher로 분리해 단독 테스트 가능하게 만든다.

### Perception 코드 분석 → mission_a 반영 매핑

mission_a 는 **추측이 아니라 Perception 실제 소스 코드를 분석**해 나온 토픽명·메시지 구조·의미를
그대로 입력 인터페이스로 삼았다 (분석 출처: [PERCEPTION_INTERFACE.md](./PERCEPTION_INTERFACE.md)).

| Perception 코드 분석 결과 | mission_a 코드 반영 |
|---------------------------|---------------------|
| `wrist_task_grasp_planner_node` 가 task 필터 + grasp score top-1 을 **`/perception/wrist/target_one_pose`**(`PoseStamped`, base_link) 로 발행 | A2_SCAN 이 이 토픽 구독 + `frame_id=='base_link'` 검증 + consume-once ([mission_a.py](../mission/mission/mission_a.py) `_on_target_pose`/`_run_a2_scan`) |
| 구 `/target_pose` 는 실제 발행 안 됨 (per-detection 은 `/perception/wrist/target_pose`) | 구 `/target_pose` 구독 **제거** |
| `tray_manage_node` 가 **`/perception/task_list`**(`GetTaskList_Response`, canonical 부품명) 발행 | `/perception/task_list` 구독 + perception-owned 모드(`_on_task_list`). canonical→class_name 매핑 `CANONICAL_TO_CLASS`('dom nut'→`dome_nut` 등) 신설 |
| `tray_manage_node` 가 OCR 기반 task list와 tray ROI를 발행 | VERIFY 가 자체 차감이 아니라 **잔여 감소 관측**(baseline 대비)으로 적재 검증 |
| `monitor_ocr` `/monitor_ocr/result` JSON(`parts[{name,count}]`, `latest_screen_detected`) + "OCR 실패 시 10초 강제 OK" 정책 | management 미가동 시 이 JSON 직접 파싱(폴백) + `FALLBACK_OK_DELAY=10` 강제 OK |
| 부품 5종 class_name + 한국어/canonical 표기 차이 | `task_list.py` 매핑 테이블(`PART_NAME_TO_CLASS`, `CANONICAL_TO_CLASS`) |
| `detector_node` `/detections` = `PartDetectionArray` | 구독 (메시지 패키지 미빌드 시 import 가드) |

### 연동 검증 상태 (정직한 현황)

| 항목 | 상태 |
|------|------|
| 인터페이스 정합성 — 토픽명/타입/JSON 구조를 perception **실제 소스에서 추출** | ✅ (추측 아님) |
| `--sim` 검증 — perception 메시지 포맷을 모사한 fake publisher 로 전 루프 통과 | ✅ |
| **라이브 end-to-end** — 실제 perception 스택 + mission_a **동시 구동 검증** | ⬜ **미완** |

> ⚠️ **요약**: mission_a 는 "실제로 작동하는 perception 코드"의 **인터페이스 분석에 기반**해 작성됐고,
> 그 인터페이스를 모사한 sim 으로는 통과했다. 그러나 **두 스택을 같이 띄운 라이브 연동은 아직 검증 전**이다.
> 미검증 블로커: ① `monitor_ocr` ocr_venv (→`/perception/task_list` 라이브 미발행) ②
> 트레이 모델 `tray_occupancy_best.pt` 미배치 ③ Manipulation Action(A3_PICK/PLACE) 미연동.
> → 라이브 검증은 monitor_ocr venv 해소 후 "perception 풀스택 + `ros2 run mission mission_a`" 동시 구동으로 진행.

### 사전 정비 (P0 — 외부 의존 0) ✅ **완료 (2026-05-30)**
| # | 작업 | 대상 | 상태 |
|---|------|------|------|
| P0-1 | `mission/` ament_python 패키지화 (`package.xml`/`setup.py`/`entry_point`) | `ros2 run mission mission_a` | ✅ |
| P0-2 | `/perception/wrist/target_one_pose` 구독 정정 | A2_SCAN | ✅ |
| P0-3 | `task_list` 자료구조 — `{class_name: remaining}` + 빌드/차감/완료 | `mission/task_list.py` | ✅ (단위 테스트 통과) |
| P0-4 | 한국어 부품명 ↔ class_name 매핑 (5종, 공백 정규화) | `PART_NAME_TO_CLASS` | ✅ |
| P0-5 | state별 timeout (`STATE_TIMEOUT` + `_timed_out()`) | 전 state | ✅ |
| P0-6 | `--sim` fake publisher 모드 (`mission/sim_driver.py`) | 테스트 | ✅ |

> **🟢 P0 sim 검증 로그 (2026-05-30, 격리 도메인 99/localhost-only)**: `INIT→A1_MONITOR`(IDLE 주입) →
> task_list `{flange_nut:1, hex_nut:2}`(총 3) 빌드 → **3회 pick-place 루프**
> (hex_nut 2→1→0, flange_nut 1→0 클래스별 정확 차감·재진입) → `잔여 0 -> DONE` 도달.
> 실행: `ros2 run mission mission_a --ros-args -p sim_mode:=true` (mission/README "sim 모드").
> 추가 구현: A1 폴백(10s 강제 OK), A2 `frame_id==base_link` 검사, consume-once 시맨틱, RECOVERY 재시도(max3).

### Phase 1 — Perception 입력 처리 (perception 라이브로 테스트 가능)
| # | 작업 | state | 상태 |
|---|------|-------|------|
| 1-0 | **`/perception/task_list` 구독 → task_list 소유권 perception 이양** | A1/VERIFY | ✅ 구현 (sim 검증) |
| 1-1 | `/monitor_ocr/result` JSON → `task_list` 빌드 (management 미가동 시 폴백) | A1_MONITOR | ✅ 폴백 구현 (라이브 검증은 monitor_ocr venv 해소 후) |
| 1-2 | 10초 fallback OK 경로 (OCR 실패 시 강제 진입, 점수 10점) | A1_MONITOR | ✅ 구현 |
| 1-3 | `/perception/wrist/target_one_pose` 수신 → `current_target` + frame_id 검사 | A2_SCAN | ✅ 구현 (planner 검증됨, 라이브 대기) |
| 1-4 | target 없을 때 재스캔/타임아웃 → RECOVERY | A2_SCAN | ✅ 구현 |
| 1-5 | VERIFY 트레이 차감 검증 (`/perception/task_list` 잔여 감소) | VERIFY | ✅ 구현 (sim 검증) |

> **🟢 Phase 1 진행 기록 (2026-05-31)**: tray management 추가 반영. mission_a 가 `/perception/task_list`
> 를 구독해 perception-owned 경로로 동작 — sim 에서 A1 task_list 확정 → 3회 루프 각 VERIFY 가
> 트레이 차감(잔여 3→2→1→0) 검증 → DONE 까지 통과. **남은 라이브 검증**: monitor_ocr venv 해소 후
> 실제 OCR→tray_manage_node→task_list, 그리고 planner 실 target 연동.

### Phase 2 — Manipulation 연동 (Action 서버 준비 후, 그 전엔 수동 fallback)
| # | 작업 | state | 의존 |
|---|------|-------|------|
| 2-1 | `bin_pick` Action 클라이언트 호출 (goal=target pose) | A3_PICK | ⬜ `bin_pick.py` Action화 대기 |
| 2-2 | Calib 전 우회: `/attach_cmd` 발행 + `/attached_object` 폴링 | A3_PICK | `GRASP_ASSESSMENT_ENABLED=False` |
| 2-3 | `tray_place` Action 클라이언트 호출 | A3_PLACE | ⬜ `tray_place.py` + `/tray_region` 대기 |
| 2-4 | Calib 전 우회: `/detach_cmd` 발행 | A3_PLACE | |
| 2-5 | `/manipulator_state` GRASPING→ATTACHED / RELEASING→IDLE 모니터 | A3_PICK/PLACE | ⬜ CM 토픽명 합의(`/manipulator_state`) |

### Phase 3 — 반복/완료/복구 루프
| # | 작업 | state | 의존 |
|---|------|-------|------|
| 3-1 | VERIFY: 적재 성공 판정 → `task_list[target] -= 1` | VERIFY | tray 재스캔(추후) 전엔 무조건 성공 가정 |
| 3-2 | 잔여 합계>0 → A2_SCAN, =0 → DONE 분기 | VERIFY | |
| 3-3 | RECOVERY: 최대 3회 재시도 → 초과 시 MANUAL_WAIT | RECOVERY | `MAX_RECOVERY_RETRY` |
| 3-4 | `/tray_region` 수신 시 VERIFY 재스캔 로직으로 대체 | VERIFY | ⬜ tray_region_node 대기 |

### 권장 진행 순서 & 마일스톤
1. **P0 전체 + Phase1(fake)** → `--sim` 으로 INIT→A1→A2→A3→VERIFY→DONE 전이 1회 통과 (외부 의존 0, ~당일)
2. **Phase1 라이브** → perception 스택 띄우고 실제 `target_one_pose`로 A2_SCAN 통과 (monitor_ocr는 fake 유지)
3. **Phase2 수동 fallback** → `/attach_cmd`·`/detach_cmd`로 pick/place 자리만 통과 (Calib 전)
4. **Phase2 Action 연동** → bin_pick/tray_place Action 준비되면 교체
5. **Phase3 + monitor_ocr 해소 + tray_region** → 진짜 end-to-end

### 합의 선행 필요 (코딩 전 확정)
- CM 토픽명: `/active_mission`, `/manipulator_state`, `/attached_object` (Manipulation과 ⬜ 미합의)
- `bin_pick` / `tray_place` Action 인터페이스 (goal/result/feedback 스키마)
- `/tray_region` 메시지 타입
