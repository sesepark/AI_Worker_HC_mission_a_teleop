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
│   ├── perception/                       # OCR / YOLO / 2D→3D / tray occupancy 통합 perception 패키지
│   ├── mission_interfaces/               # Mission-Perception 서비스/메시지 인터페이스
│   ├── manipulation/                     # MoveIt/GPD/pick-place primitives
│   ├── mission/                          # System 팀 — Mission A 상태기계
│   └── docs/                             # 설계·인터페이스·실행 문서
├── physical_ai_tools/      # ROBOTIS 공식 — 수정 금지
└── robotis_applications/   # ROBOTIS 공식 (Vuer 등) — 수정 금지
```

> ⚠️ **실행 워크스페이스는 별도**: 도커 컨테이너는 `~/robotis_ros2_ws`(= `/ws`)를 마운트해 빌드·실행한다.
> `~/AI_Worker_HC`가 소스 진실이고, `~/robotis_ros2_ws/src`는 rsync 사본(git 아님).

---

## 최근 정리 내용

### 1) Perception 패키지 통합
- `monitor_ocr`, 부품 YOLO, head/wrist 2D→3D projection, tray occupancy primitive를 `perception` 단일 ROS 패키지로 통합.
- 내부 기능은 `perception_nodes/{monitor_ocr,part_detector,head_projection,wrist_projection,tray_occupancy}` 아래에 둔다.
- task list와 tray detection은 토픽 누적 로직 대신 `mission_interfaces` 서비스로 mission 쪽에서 요청한다.

### 2) Mission 서비스 연동
```
/mission_a/task_list       # monitor OCR 결과 요청
/mission_a/tray_detections # tray/part primitive detection 요청
```
→ perception은 primitive detection/OCR을 제공하고, 파싱·정규화·수량 관리는 mission 쪽에서 수행한다.

### 3) `mission_a` 구현
- `humanoid_challenge/mission/` 를 ament_python 패키지 `mission` 으로 구성 → `ros2 run mission mission_a`.
- FSM: `INIT→A1_MONITOR→A2_SCAN→A3_PICK→A3_PLACE→VERIFY→DONE` (+RECOVERY/MANUAL_WAIT).
- `/mission_a/task_list` 서비스로 OCR 지령을 받고, `/perception/wrist/target_one_pose`를 구독한다.
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

## 학습 모델 파일 위치

도커 이미지는 학습된 `.pt` 모델 파일을 포함하지 않는다. 컨테이너 실행 전 로컬 소스 트리에 아래처럼 배치한다.

| 파일 | 다운로드 |
|------|----------|
| `humanoid_challenge/perception/model/part_detector_best.pt` | [Drive](https://drive.google.com/file/d/17BepvzEurXIQbh3F9X3SQDCB8iaqLkWC/view) |
| `humanoid_challenge/perception/model/monitor_ocr_best.pt` | [Drive](https://drive.google.com/file/d/14H48riKH3KkKxky2yrCMufPfiGz6gfa0/view) |
| `humanoid_challenge/perception/model/tray_occupancy_best.pt` (파랑 트레이) | [Drive: blue_tray_yolo](https://drive.google.com/drive/folders/1MzRzf27wtmPqp8-KqR9iH_AnrLsgaPOU?usp=sharing) |

tray 모델은 `TRAY_MODEL_PATH` 환경 변수나 `tray_model_path` launch argument로 다른 경로를 지정할 수 있다.

---

## 남은 작업
- [ ] 실제 헤드 카메라 입력으로 monitor OCR 라이브 검증
- [ ] 트레이 YOLO 모델 `tray_occupancy_best.pt` 배치
- [ ] A3_PLACE용 트레이 base_link place 좌표 인터페이스 협의 (현재 tray_contents 는 2D 카운트만)
- [ ] Phase 2 Manipulation 연동 (`bin_pick`/`tray_place` Action)
- [ ] CM 토픽명(`/active_mission`, `/manipulator_state`, `/attached_object`) 전 팀 합의

---

## 참고 — 카메라 시리얼
```
camera 1 : 335122271636
camera 2 : 335122270229
```
