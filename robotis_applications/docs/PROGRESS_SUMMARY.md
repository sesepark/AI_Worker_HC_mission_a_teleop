# 진행 상황 요약 (Mission A / Perception 통합)

> **최종 업데이트**: 2026-05-31
> **범위**: System 팀 관점의 Perception 통합 · 로컬 구동 검증 · mission_a 구현
> **연관 문서**: [MISSION_A_SCENARIO_PLAN.md](./MISSION_A_SCENARIO_PLAN.md) ·
> [PERCEPTION_INTERFACE.md](./PERCEPTION_INTERFACE.md) · [PERCEPTION_LOCAL_SETUP.md](./PERCEPTION_LOCAL_SETUP.md) ·
> [mission/README.md](../mission/README.md)

---

## 1. 레포 / 워크스페이스 구조

| 구분 | 경로 | 역할 |
|------|------|------|
| 소스 진실(git) | `~/AI_Worker_HC/` | 통합 monorepo (ai_worker / physical_ai_tools / robotis_applications) |
| 실행 워크스페이스 | `~/robotis_ros2_ws` (도커 `/ws`) | colcon 빌드·노드 실행. AI_Worker_HC 바깥, rsync 사본 |
| Perception upstream | [hublemon/Humanoid-Challenge-Perception](https://github.com/hublemon/Humanoid-Challenge-Perception) | 브랜치 **`demo/senario_A`** (최신), `fix/wrist-task-grasp-stability`(planner) |

- 도커: 사용자 `docker` 그룹 → `sudo` 불필요. 이미지 `ros2_jazzy_robotis_perception:latest` (직접 빌드).
- 로봇 `ffw-SNPR48A1087.local` (SSH pw `root`) — bringup 이 카메라/TF/ZED 발행.

---

## 2. Perception 로컬 구동 검증 (2026-05-30)

환경 구축(이미지·venv·colcon) + 실로봇 bringup 으로 구동.

| 노드 | 상태 |
|------|------|
| `detector` / `projection` / `wrist_projection` / `wrist_pointcloud` / `wrist_grasp_pcd` / `wrist_task_grasp_planner` | ✅ 검증 |
| `monitor_ocr` | ⚠️ 블로커 — 노드 로직 정상, `ocr_venv` 의존성 누락 (재생성 필요) |

**두 함정** (상세: PERCEPTION_LOCAL_SETUP "함정 ①②"):
1. `ros2 run` shebang 이 시스템 python → venv 노드는 `prefix=`(launch) 또는 venv python 직접 실행.
2. `ocr_venv` 가 오염된 시스템 python 위에서 생성돼 의존성 누락 → 깨끗이 재생성.

> 신규 launch(`monitor_ocr.launch.py`, `task_management.launch.py`)는 `prefix="/ws/<venv>/bin/python3"`
> 로 함정 ① 을 우회한다.

---

## 3. Perception 신규 반영 — `task_management` (upstream `demo/senario_A`, 2026-05-31)

트레이 검출 + 태스크 리스트 관리 패키지. 19개 파일 로컬 반영 완료.

```
detector(/detections) ─┐
ZED RGB ───────────────┴→ tray_occupancy_node ──/perception/tray_contents──┐
monitor_ocr(/monitor_ocr/result) ────────────────────────────────────────┴→ management_node
                                                                                   │
                                                          /perception/task_list (잔여=OCR목표−트레이관측)
                                                                                   │
                                                                              mission_a (A1 / VERIFY)
```

| 노드 | 입력 | 출력 |
|------|------|------|
| `tray_occupancy_node` | `/detections`, ZED RGB (+별도 트레이 YOLO `tray_model_path`) | `/perception/tray_contents` (트레이 내 부품 카운트) |
| `management_node` | `/monitor_ocr/result`, `/perception/tray_contents` | **`/perception/task_list`** (canonical 부품명, 잔여) |

- canonical 표기는 공백/소문자: `flange nut / gear ring / spacer ring / hex nut / **dom nut**`
  (detector class_name `flange_nut`/`dome_nut` 와 다름 → `mission/task_list.py` 변환).
- 미해결: 트레이 **base_link place 좌표**(A3_PLACE용)는 아직 없음 (tray_contents 는 2D 카운트만).

---

## 4. mission_a 구현 (System, 2026-05-31)

`robotis_applications/mission/` 를 ament_python 패키지 `mission` 으로 구성.

| 모듈 | 내용 |
|------|------|
| `mission/mission_a.py` | FSM (INIT→A1→A2→A3_PICK→A3_PLACE→VERIFY→DONE/RECOVERY/MANUAL_WAIT) |
| `mission/task_list.py` | OCR(한국어)·canonical 부품명 → `{class_name: 잔여}`, 빌드/차감/완료 (단위 테스트 통과) |
| `mission/sim_driver.py` | `--sim` fake 토픽 주입 — task_management 파이프라인 모사 |

**구현된 로직**
- task 소스: `/perception/task_list`(있으면 우선) → OCR 직접파싱 폴백 → 10초 강제 OK 폴백.
- A2_SCAN: `/perception/wrist/target_one_pose`(planner) 수신, `frame_id==base_link` 검사, consume-once.
- VERIFY: perception-owned 면 **트레이 차감(잔여 감소)** 으로 적재 검증, 아니면 자체 차감(성공 가정).
- state 별 timeout, RECOVERY 재시도(max 3) → MANUAL_WAIT.

**검증 (sim, 격리 도메인 99)**
```
INIT→A1(/perception/task_list 총3 확정)
→ 루프1 flange_nut(잔여3→2) → 루프2 hex_nut(2→1) → 루프3 hex_nut(1→0)
→ VERIFY 잔여0 → DONE   ✅ 전 전이·트레이 차감 검증 통과
```
실행: `ros2 run mission mission_a --ros-args -p sim_mode:=true` (격리: `ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1`).

---

## 5. 확정된 인터페이스 (mission_a 기준)

| 토픽 | 방향 | 용도 |
|------|------|------|
| `/perception/task_list` | Perception → mission_a | 잔여 task (A1/VERIFY source of truth) |
| `/perception/wrist/target_one_pose` | Perception → mission_a | 최종 grasp target 1개 (planner) |
| `/monitor_ocr/result` | Perception → mission_a | OCR 목표 (폴백 경로) |
| `/detections` | Perception → mission_a | 부품 검출 배열 |
| `/active_mission` `/attach_cmd` `/detach_cmd` | mission_a → CM/Manip | 미션 선언·수동 attach/detach |
| `/manipulator_state` `/attached_object` | CM/Manip → mission_a | 상태·파지물체 |

---

## 6. 남은 작업 (다음 단계)

- [ ] **monitor_ocr `ocr_venv` 재생성** → 실 OCR→management_node→`/perception/task_list` 라이브 검증
- [ ] **트레이 YOLO 모델 `tray_best.pt` 배치** (launch `tray_model_path`)
- [ ] **A3_PLACE place 좌표**: 트레이 base_link 3D 위치/적재영역 인터페이스 Perception+Manipulation 협의
- [ ] **Phase 2 Manipulation 연동**: `bin_pick`/`tray_place` Action (전엔 `/attach_cmd`·`/detach_cmd` 수동 우회)
- [ ] CM 토픽명(`/active_mission`,`/manipulator_state`,`/attached_object`) 전 팀 합의
- [ ] planner 가 `/perception/task_list`(잔여) 를 반영하도록 협의 (현재 `/monitor_ocr/result` 만 구독 — 적재분 재선정 가능성)
- [ ] `build/`·`install/`·`log/` `.gitignore` 추가
