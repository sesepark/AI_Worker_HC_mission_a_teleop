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
- `monitor_ocr`, 부품 YOLO, head/wrist 2D->3D projection, wrist grasp target planner, tray task management를 `perception` 단일 ROS 패키지로 통합.
- 내부 Python 모듈은 `perception_nodes/{monitor_ocr,part_detector,head_projection,wrist_projection,management}` 구조를 유지한다.
- 메시지는 통합 패키지의 `perception.msg.PartDetectionArray`를 사용하고, 기존 독립 패키지명(`perception_part_detector`, `perception_2d_to_pcd_wrist`, `task_management`) import는 사용하지 않는다.

### 2) robotis_ros2_ws perception 변경분 반영

이 브랜치는 `hublemon/Humanoid-Challenge-Perception`의 `backup/current-work-20260622` 작업을 `snu-shape/AI_Worker_HC`의 `feature/mission-a` 구조에 맞춰 이식한 것이다. 별도 ROS 패키지를 추가하지 않고, `humanoid_challenge/perception` 단일 패키지 안에서 실행 파일과 launch만 연결했다.

#### Task management 경로
- 새 실행 노드: `perception_nodes/management/tray_manage_node.py`
- 설치 실행명: `ros2 run perception tray_manage_node`
- launch 진입점:
  ```bash
  ros2 launch perception task_management.launch.py
  ```
- 호환 launch:
  ```bash
  ros2 launch perception tray_occupancy.launch.py
  ```
  기존 이름을 쓰는 스크립트가 있어도 같은 `tray_manage_node`가 실행되도록 연결했다.
- 입력:
  - `/monitor_ocr/result` (`std_msgs/String`, OCR JSON)
  - `/camera_right/camera_right/color/image_rect_raw` (`sensor_msgs/Image`, tray YOLO 입력)
- 출력:
  - `/perception/task_list` (`mission_interfaces/srv/GetTaskList_Response`, typed task list state)
  - `/perception/get_task_list` (`mission_interfaces/srv/GetTaskList`, latest task list service)
  - `/perception/tray_roi` (`sensor_msgs/RegionOfInterest`, 최신 tray bbox)
- `GetTaskList` 변환 규칙:
  - `mission_complete` -> `success`
  - `source` -> `message` (`source` dict를 JSON string으로 직렬화)
  - `ocr_latest_screen_detected` -> `all_counts_recognized`
  - `ocr_frames_used` -> `frames_used`
  - `parts` -> `mission_interfaces/TaskItem[] parts`
- 기존 String JSON `/perception/task_list`는 제거했고, `/perception/task_list`는 typed `GetTaskList_Response` topic으로 발행한다.
- `GetTaskList` service 이름은 기본적으로 `/perception/get_task_list`이며, System FSM의 service fallback에 직접 연결하려면 `task_list_service_name:=/mission_a/task_list`로 override할 수 있다.
- `mission_a`의 service fallback은 이 매핑을 반영해, `success=false`라도 `parts`가 있으면 task list를 반영한다. 여기서 `success`는 RPC 성공 여부가 아니라 `mission_complete` 상태다.
- 모델 기본 경로:
  - `humanoid_challenge/perception/model/tray_occupancy_best.pt`
  - `TRAY_MODEL_PATH` 환경 변수 또는 `tray_model_path:=...` launch argument로 override 가능.
- 이전 `management_node`, `tray_contents_node`, `tray_occupancy_node` 구현 파일은 제거했다. 잘못된 예전 파이프라인이 같이 떠서 `/perception/task_list`를 중복 발행하는 것을 막기 위한 정리다.

#### Wrist grasp target 경로
- 실행 노드: `perception_nodes/wrist_projection/wrist_task_grasp_planner_node.py`
- 설치 실행명: `ros2 run perception wrist_task_grasp_planner_node`
- launch 진입점:
  ```bash
  ros2 launch perception wrist_task_grasp_planner.launch.py
  ```
- `wrist_all.launch.py`도 현재는 최종 target planner만 실행한다.
- 입력:
  - `/detections` (`perception/msg/PartDetectionArray`)
  - wrist RGB-D image/camera_info
  - `/perception/task_list` (`mission_interfaces/srv/GetTaskList_Response`)
- 출력:
  - `/perception/wrist/target_one_pose` (`geometry_msgs/PoseStamped`, base_link 기준 최종 target)
  - `/perception/wrist/target_one_detection` (`std_msgs/String`, 선택된 detection/bbox/score JSON)
- scoring은 기존 화면 중심/마스크 품질 중심 로직 대신 `confidence + arm_reference proximity` 기준으로 바뀌었다.
- 관련 파라미터는 `humanoid_challenge/perception/config/wrist_projection/params.yaml`의 `wrist_task_grasp_planner_node` 섹션에 있다.
  - `out_target_detection_topic`
  - `arm_reference_frame`
  - `arm_reference_xyz`
  - `max_arm_distance_m`
  - `weight_confidence`
  - `weight_arm_proximity`
- `wrist_projection_node`, `wrist_pointcloud_node`, `wrist_grasp_pcd_node`와 각 개별 launch는 유지한다. 필요 시 기존 projection/pointcloud/PCD 디버깅용으로 단독 실행할 수 있다.

#### System 팀 영향
- `mission_a`가 구독하는 task input 이름은 유지하되, 타입은 `std_msgs/String` JSON에서 `mission_interfaces/srv/GetTaskList_Response`로 변경했다.
  - task input: `/perception/task_list`
  - pick target: `/perception/wrist/target_one_pose`
- 따라서 System 쪽 FSM은 topic 이름은 그대로 쓰되, subscriber 타입만 `GetTaskList.Response` 계약에 맞추면 된다.
- 변경된 것은 `/perception/task_list`를 만드는 내부 구현이다. 예전 `tray_contents_node + management_node` 조합 대신 `tray_manage_node`가 OCR 결과와 tray image를 받아 task list를 직접 발행한다.
- `use_task_list_service` 기반 OCR 서비스 fallback 코드는 남아 있으므로, topic pipeline이 준비되지 않은 상황에서도 기존 서비스 테스트 경로를 막지 않는다.

#### 보조 수집 도구
다음 스크립트를 `humanoid_challenge/perception/tools/`에 추가했다.
- `save_right_wrist_base_pose.py`: right wrist camera frame의 base 기준 TF와 joint state를 JSON/CSV로 저장
- `save_wrist_target_pairs.py`: `/perception/wrist/target_one_pose`와 `/perception/wrist/target_one_detection`을 pair로 저장
- `save_zed_rgb_100.py`: ZED RGB image topic에서 fixed count image를 저장

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
- [ ] A3_PLACE용 트레이 base_link place 좌표 인터페이스 협의 (현재 perception은 `/perception/tray_roi` 2D ROI까지만 제공)
- [ ] Phase 2 Manipulation 연동 (`bin_pick`/`tray_place` Action)
- [ ] CM 토픽명(`/active_mission`, `/manipulator_state`, `/attached_object`) 전 팀 합의

---

## 참고 — 카메라 시리얼
```
camera 1 : 335122271636
camera 2 : 335122270229
```
