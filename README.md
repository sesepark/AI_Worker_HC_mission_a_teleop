# AI_Worker_HC — 휴머노이드 챌린지 (Mission A)

ROBOTIS AI Worker 기반 휴머노이드 챌린지 통합 레포. Perception / Manipulation / System(미션) 팀 코드를
한 워크스페이스로 통합해 **Mission A**(지령 인식 → 부품 집기 → 트레이 적재)를 자율 수행하는 것이 목표.

> 📄 상세 진행상황: [humanoid_challenge/docs/PROGRESS_SUMMARY.md](humanoid_challenge/docs/PROGRESS_SUMMARY.md)

---

## 레포 구조

```
AI_Worker_HC/
├── ai_worker/              # ROBOTIS 공식 (로봇 bringup 등) — 수정 금지
├── humanoid_challenge/     # 휴머노이드 챌린지 Mission A 팀 개발 패키지
│   ├── monitor_ocr/                      # 지령 모니터 OCR
│   ├── perception_part_detector/         # YOLO 부품 검출
│   ├── perception_2d_to_pcd(_wrist)/     # 2D→3D pose/PCD
│   ├── task_management/                  # 트레이 검출 + 잔여 task 관리
│   ├── ai_worker_manipulation/           # MoveIt/GPD/pick-place primitives
│   ├── mission/                          # System 팀 — Mission A 상태기계
│   └── docs/                             # 설계·인터페이스·실행 문서
├── physical_ai_tools/      # ROBOTIS 공식 — 수정 금지
└── robotis_applications/   # ROBOTIS 공식 (Vuer 등) — 수정 금지
```

> ⚠️ **실행 워크스페이스는 별도**: 도커 컨테이너는 `~/robotis_ros2_ws`(= `/ws`)를 마운트해 빌드·실행한다.
> `~/AI_Worker_HC`가 소스 진실이고, `~/robotis_ros2_ws/src`는 rsync 사본(git 아님).

---

## 이번 세션 진행 내용 (2026-05-30 ~ 05-31)

### 1) Perception 로컬 구동 검증 ✅
- 도커 이미지(`ros2_jazzy_robotis_perception`) 빌드 + venv + colcon + 실로봇 bringup 으로 구동.
- **`monitor_ocr` 제외 전 노드 정상 작동 확인** (detector / projection / wrist 파이프라인 / grasp planner).
- `monitor_ocr` 는 노드 로직은 정상이나 `ocr_venv` 의존성 누락 블로커 (재생성 필요).
- 두 함정 정리: ① `ros2 run` 의 venv shebang 문제 ② ocr_venv 의존성 오염 → [PERCEPTION_LOCAL_SETUP.md](humanoid_challenge/docs/PERCEPTION_LOCAL_SETUP.md)

### 2) Perception 신규 `task_management` 반영 ✅ (upstream `demo/senario_A`)
트레이 비전으로 적재 진행을 자동 추적하는 파이프라인:
```
detector(/detections) + ZED RGB → tray_occupancy_node → /perception/tray_contents
                                  /monitor_ocr/result  → management_node → /perception/task_list
                                                          (잔여 = OCR목표 − 트레이관측)
```
→ `mission_a` 가 `/perception/task_list` 만 보면 OCR 파싱·자체 차감 불필요, 트레이 비전이 자동 검증.

### 3) `mission_a` 구현 ✅ (System)
- `humanoid_challenge/mission/` 를 ament_python 패키지 `mission` 으로 구성 → `ros2 run mission mission_a`.
- FSM: `INIT→A1_MONITOR→A2_SCAN→A3_PICK→A3_PLACE→VERIFY→DONE` (+RECOVERY/MANUAL_WAIT).
- `/perception/task_list` 우선 소비(없으면 OCR 폴백), `/perception/wrist/target_one_pose` 구독,
  state timeout, A1 10초 폴백, VERIFY 트레이 차감 검증.
- **`--sim` 모드로 전체 루프 검증 통과** (트레이 차감 3→2→1→0 → DONE).
  ```bash
  export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1
  ros2 run mission mission_a --ros-args -p sim_mode:=true
  ```

---

## 문서 안내

| 문서 | 내용 |
|------|------|
| [docs/PROGRESS_SUMMARY.md](humanoid_challenge/docs/PROGRESS_SUMMARY.md) | **전체 진행상황 요약 (먼저 읽기)** |
| [docs/MISSION_A_SCENARIO_PLAN.md](humanoid_challenge/docs/MISSION_A_SCENARIO_PLAN.md) | 미션 A 시나리오·상태기계·mission_a 작성 계획 |
| [docs/PERCEPTION_INTERFACE.md](humanoid_challenge/docs/PERCEPTION_INTERFACE.md) | Perception 노드·토픽 인터페이스 |
| [docs/PERCEPTION_LOCAL_SETUP.md](humanoid_challenge/docs/PERCEPTION_LOCAL_SETUP.md) | 로컬 도커 실행 셋업·런북·트러블슈팅 |
| [mission/README.md](humanoid_challenge/mission/README.md) | mission 패키지 빌드·실행 |

---

## 남은 작업
- [ ] `monitor_ocr` ocr_venv 재생성 → 실 OCR → `/perception/task_list` 라이브 검증
- [ ] 트레이 YOLO 모델 `tray_best.pt` 배치
- [ ] A3_PLACE용 트레이 base_link place 좌표 인터페이스 협의 (현재 tray_contents 는 2D 카운트만)
- [ ] Phase 2 Manipulation 연동 (`bin_pick`/`tray_place` Action)
- [ ] CM 토픽명(`/active_mission`, `/manipulator_state`, `/attached_object`) 전 팀 합의

---

## 참고 — 카메라 시리얼
```
camera 1 : 335122271636
camera 2 : 335122270229
```
