# 04 · Perception 통합 검증 가이드 (INTEGRATION VERIFICATION GUIDE)

> 대상: 공용 레포 `AI_Worker_HC` (branch `feature/mission-a`), 단일 `perception` 패키지 + `mission`.
> 목적: Perception 팀 검증본(demo/senario_A)을 공용 레포로 이전한 **Phase 2 통합 결과**를,
> 사용자가 **직접 따라 실행하며** 통합된 패키지 기능 작동을 확인하는 절차.
> 작성 기준: 실제 통합 파일(읽기 전용)에서 노드명·토픽·경로·파라미터를 재확인하여 근거 `파일:라인`을 명시.
> 컨테이너 마운트: repo → `/ws/src/humanoid_challenge`, colcon 워크스페이스 루트 `/ws`.

---

## (A) 통합 작업현황 요약

### A-1. 변경 12건

| # | 경로 | 신규/수정 | 변경요지 | 근거 |
|---|---|---|---|---|
| 1 | `perception/perception_nodes/name_utils.py` | 신규 | `CANONICAL_PARTS`=`["flange nut","gear ring","spacer ring","hex nut","dom nut"]`, `canonical_part_name` 등 | `name_utils.py:6-12,61-67` |
| 2 | `perception/perception_nodes/management/management_node.py` (+`__init__.py`) | 신규 | 데모 이식, import만 `perception_nodes.name_utils`. `/monitor_ocr/result`+`/perception/tray_contents`→`/perception/task_list` | `management_node.py:12,36-38` |
| 3 | `perception/perception_nodes/tray_occupancy/tray_contents_node.py` | 신규 | 데모 연속발행 로직. `perception.msg`/`perception_nodes.name_utils`, 모델기본값 `perception/model/tray_occupancy_best.pt`, 노드명 `tray_contents_node` | `tray_contents_node.py:17-18,24,59,66,114` |
| 4 | `perception/perception_nodes/monitor_ocr/monitor_ocr_topic_node.py` | 신규 | 공용 OCR 파이프라인 재사용해 `/monitor_ocr/result` 발행 (서비스 노드 무수정) | `monitor_ocr_topic_node.py:35-41,83` |
| 5 | `perception/CMakeLists.txt` | 수정 | 3개 신규 노드 `install(PROGRAMS … RENAME …)` | `CMakeLists.txt:71-90` |
| 6 | `perception/launch/task_management.launch.py` | 신규 | `tray_contents_node`(prefix `/ws/yolo_venv`)+`management_node` | `task_management.launch.py:44-46,89` |
| 7 | `mission/mission/task_list.py` | 수정 | 영문 canonical 키 추가(★`domnut→dome_nut`) | `task_list.py:14-31` |
| 8 | `mission/mission/mission_a.py` | 수정 | `/perception/task_list` 구독+`_on_task_list`, A1 토픽구동, `use_task_list_service` 가드, VERIFY 하이브리드 | `mission_a.py` (아래 D 참조) |
| 9 | `perception/config/wrist_projection/params.yaml` | 수정 | `class_alias_json`에 영문 별칭 추가(★`"dom nut":"dome_nut"`), 한국어 보존 | `params.yaml:162-164` |
| 10 | `docker/Dockerfile.amd64` | 수정 | `ros-${ROS_DISTRO}-image-transport-plugins` | `Dockerfile.amd64:73` |
| 11 | `docker/docker-compose.yml` | 수정 | WSLg env(`WAYLAND_DISPLAY`/`XDG_RUNTIME_DIR`/`QT_QPA_PLATFORM=xcb`)+`/mnt/wslg` 마운트 | `docker-compose.yml:13,17-18,24` |
| 12 | `.gitignore` (repo 루트) | 수정 | `paddlex_cache/`, `*.zip`, `*.whl` | `.gitignore` |

### A-2. 통합 후 노드 / 토픽 파이프라인 맵

신규 **토픽 노드**와 기존 **서비스 노드**가 **병행** 존재한다(서비스 노드 무수정).

```
[검증된 토픽 파이프라인 — 신규]                         [기존 서비스 — 무수정 병행]
                                                       monitor_ocr_node  (srv /mission_a/task_list)
(로봇) 카메라 image + /tf                                tray_occupancy_node (srv /mission_a/tray_detections)
   │
   ├─► detector_node ─► /detections (PartDetectionArray)
   │                          │
 /monitor_ocr/result ◄── monitor_ocr_topic_node(실) │  또는  fake OCR(topic pub)
   │                          ▼
   │                   tray_contents_node ─► /perception/tray_contents (String JSON)
   │                          │
   ▼                          ▼
 management_node ◄────────────┘   (remaining = ocr − tray)
   │
   ▼
 /perception/task_list (String JSON, canonical 영문 "flange nut".."dom nut")
   │                                   │
   ├─► wrist_task_grasp_planner_node ──┤  (task_topic 구독)
   │        │                          │
   │        ▼                          ▼
   │   /perception/wrist/target_one_pose (PoseStamped, base_link)  ← 출력계약 불변
   │
   └─► mission_a (_on_task_list 구독 → A1_MONITOR → A2_SCAN …)
```

- `management_node` 구독 `/monitor_ocr/result`,`/perception/tray_contents` → 발행 `/perception/task_list` (`management_node.py:36-38`).
- `tray_contents_node` 발행 `/perception/tray_contents`, 구독 `/detections`+image (`tray_contents_node.py:114-122`).
- `monitor_ocr_topic_node` 발행 `/monitor_ocr/result` (`monitor_ocr_topic_node.py:83`).
- 기존 서비스: `monitor_ocr_node` srv `/mission_a/task_list` (`monitor_ocr_node.py:46,90-95`), `tray_occupancy_node` srv `/mission_a/tray_detections` (`tray_occupancy_node.py:67,129-134`) — **삭제하지 않고 유지**.
- 출력계약 `/perception/wrist/target_one_pose` (PoseStamped, `base_link`) — wrist planner 무수정 (`wrist_task_grasp_planner_node.py:66,968`).

---

## (B) 검증 개요

검증을 두 범주로 나눈다.

| 범주 | 로봇/실물 | 누가 실행 | 무엇을 확인 |
|---|---|---|---|
| **범주 1** (C) | 불필요 | 사용자 직접 재현 | 빌드/설치/토픽 발행/이름 정합 등 **패키지 기능 통합** |
| **범주 1-2** (D) | 불필요 | 사용자 직접 재현 | **mission_a 상태머신** 토픽 구동 (A1→A2) |
| **범주 2** (E) | 필요 | 사용자가 로봇 연결 후 실행 | 실 detector/tray/planner, target_one_pose 등 |

### 공통 전제 (컨테이너 진입)

```bash
# [호스트] 공용 컨테이너 시작 + 진입
cd ~/AI_Worker_HC/humanoid_challenge
./docker/container.sh start     # 이미지 pull(필요시) + compose up -d  (container.sh:76-83)
./docker/container.sh enter     # docker exec, /ws에서 ROS+install source  (container.sh:85-101)
# 추가 터미널마다 다시:  ./docker/container.sh enter
```

- 컨테이너명 `humanoid_challenge`, 이미지 `shpark1104/humanoid_challenge:jazzy` (`container.sh:8-9`).
- 진입 시 자동으로 `cd /ws`, `source /opt/ros/jazzy/setup.bash`, (있으면) `source /ws/install/setup.bash` 수행 (`container.sh:93-100`).
- 환경 사실(확정·재구성 금지): `yolo_venv`/`ocr_venv` 분리·`--system-site-packages`·`numpy<2`·`COLCON_IGNORE`. 노드 shebang/launch prefix는 `/ws/yolo_venv`·`/ws/ocr_venv` 경로에 의존.
- **각 추가 터미널**에서 빌드 후에는 `source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash`를 먼저 실행한다.

> 범주 1·1-2는 모델 `.pt`·venv·카메라 없이 동작한다(management_node·mission_a는 순수 `rclpy`/`std_msgs`). 모델·카메라가 필요한 노드(detector/tray_contents/monitor_ocr_topic 실행)는 범주 2에서 다룬다.

---

## (C) 범주 1 — 로봇 불필요: 패키지 기능 통합 검증

> 터미널 순서대로 진행. 각 단계는 **명령 / 확인 / 의미 / 실제 환경 사용** 4요소.

### C-0. 빌드 (터미널 1)

- **명령**
  ```bash
  cd /ws
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install --packages-up-to perception
  source /ws/install/setup.bash
  ```
- **확인**: `Starting >>> mission_interfaces` → `Finished`, `Starting >>> perception` → `Finished`, 마지막 `Summary: 2 packages finished`, 오류 0.
- **의미**: 단일 `perception` 패키지가 의존(`mission_interfaces`)과 함께 빌드됨 = msg 생성(`PartDetection`/`PartDetectionArray`)·신규 노드 설치·`perception_nodes` 파이썬 패키지 설치가 정합. (`CMakeLists.txt:13-19,71-90`)
- **실제 사용**: 실로봇/실파이프라인에서도 동일 빌드 명령을 사용. (`perception`가 인터페이스 패키지이므로 다운스트림보다 먼저 빌드)

### C-1. 실행파일 노출 확인 (터미널 1)

- **명령**
  ```bash
  ros2 pkg executables perception
  ```
- **확인**: 아래 13개가 모두 보여야 하며, **신규 3개**가 포함:
  ```
  perception detector_node
  perception management_node            ← 신규
  perception monitor_ocr_node
  perception monitor_ocr_topic_node     ← 신규
  perception tray_contents_node         ← 신규
  perception tray_occupancy_node
  perception wrist_task_grasp_planner_node
  perception head_projection_node / head_pointcloud_node / head_grasp_pcd_node
  perception wrist_projection_node / wrist_pointcloud_node / wrist_grasp_pcd_node
  ```
- **의미**: `CMakeLists.txt:71-90`의 `install(PROGRAMS … RENAME …)`가 정상 설치됨 = 신규 노드가 `ros2 run perception <노드>`로 호출 가능. 기존 서비스 노드(`monitor_ocr_node`,`tray_occupancy_node`)도 그대로 병행 노출.
- **실제 사용**: launch 파일들이 `executable='management_node'` 등 이 이름으로 노드를 띄운다 (`task_management.launch.py:44,89`).

### C-2. management_node 기동 (터미널 2)

- **명령**
  ```bash
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  ros2 run perception management_node
  ```
- **확인**: `ManagementNode ready. ocr_result_topic=/monitor_ocr/result, tray_contents_topic=/perception/tray_contents, task_list_topic=/perception/task_list` 로그.
- **의미**: 신규 management_node가 정상 기동, 구독/발행 토픽이 `management_node.py:19-21,40-43`대로 설정됨.
- **실제 사용**: 실파이프라인에서는 `ros2 launch perception task_management.launch.py`로 tray_contents_node와 함께 기동된다.

### C-3. fake OCR 발행 (터미널 3)

- **명령** (★부품 5종 전부 포함 — `require_complete_ocr=True`라 누락 시 무시됨, `management_node.py:23,49-56`)
  ```bash
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  ros2 topic pub /monitor_ocr/result std_msgs/msg/String \
  "{data: '{\"frames_used\":10,\"parts\":[{\"name\":\"플랜지 너트\",\"count\":1},{\"name\":\"기어 링\",\"count\":2},{\"name\":\"스페이서 링\",\"count\":1},{\"name\":\"육각 너트\",\"count\":4},{\"name\":\"돔 너트\",\"count\":2}],\"latest_screen_detected\":true}'}" -r 1
  ```
- **확인**: 터미널 3에 `publishing #N` 반복 출력. 터미널 2(management) 경고(`Invalid OCR JSON`/`Incomplete OCR result ignored`)가 **없어야** 정상.
- **의미**: management가 한국어 부품명을 `canonical_part_name`으로 정규화(`management_node.py:80`, `name_utils.py:61-67`)해 수용. (한국어 "돔 너트" → canonical "dom nut")
- **실제 사용**: 실환경에서는 이 토픽을 `monitor_ocr_topic_node`(실 OCR, 범주 2)가 채운다 — fake와 **동일 토픽/형상**.

### C-4. /perception/task_list 출력 확인 (터미널 4)

- **명령**
  ```bash
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  ros2 topic echo --once /perception/task_list
  ```
- **확인**: 다음 형상의 JSON String:
  ```
  data: '{"parts": [{"name": "flange nut", "count": 1}, {"name": "gear ring", "count": 2},
          {"name": "spacer ring", "count": 1}, {"name": "hex nut", "count": 4},
          {"name": "dom nut", "count": 2}], "source": {...}, "ocr_frames_used": 10, ...}'
  ```
- **의미**: ① management 토픽 발행 **복원** 확인, ② canonical **영문** 이름으로 정규화, ③ `remaining = ocr − tray` 계산(`management_node.py:100-105`). tray 입력이 없어도 `publish_on_empty_tray=True`(`management_node.py:22,97`)로 `remaining=ocr`.
- **실제 사용**: 이 토픽이 wrist planner(`task_topic`)와 mission_a(`_on_task_list`)의 입력. 다운스트림이 그대로 소비.

### C-5. (선택) fake tray로 차감 확인 (터미널 5)

- **명령** (트레이에 flange nut 1개가 적재됐다고 가정 → 잔량 1−1=0)
  ```bash
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  ros2 topic pub /perception/tray_contents std_msgs/msg/String \
  "{data: '{\"parts\":[{\"name\":\"flange nut\",\"count\":1}],\"tray_count\":1,\"tray_detections\":[]}'}" -r 1
  ```
- **확인**: 터미널 4의 `/perception/task_list`에서 `flange nut`의 `count`가 `1`→`0`으로 줄어든다.
- **의미**: management의 `remaining = max(ocr − tray, 0)` 차감 로직(`management_node.py:100-105`)이 실제로 동작.
- **실제 사용**: 실환경에서는 `tray_contents_node`(범주 2)가 트레이 YOLO로 이 토픽을 채워 적재 진행분을 차감.

### C-6. 토픽 연결(파이프라인 배선) 확인 (터미널 4/5)

- **명령**
  ```bash
  ros2 topic info /monitor_ocr/result
  ros2 topic info /perception/tray_contents
  ros2 topic info /perception/task_list
  ```
- **확인**: `/monitor_ocr/result` → Publisher(fake/실), Subscriber ≥1(management). `/perception/task_list` → Publisher 1(management), Subscriber 수(echo/planner/mission_a 실행 시 증가). 타입 모두 `std_msgs/msg/String`.
- **의미**: 토픽명·타입·발행/구독 연결이 통합 설계대로 배선됨.
- **실제 사용**: 노드 누락/오타로 파이프라인이 끊기지 않았는지 빠르게 진단하는 표준 방법.

---

## (D) 범주 1-2 — 로봇 불필요: mission_a 상태머신 통합 검증

mission_a를 **토픽 모드(기본)**로 구동해 `/perception/task_list` 수신 → `A1_MONITOR`(total>0) → `A2_SCAN` 진입까지 확인한다. 이후 단계(A3_PICK 등)는 manipulation/perception-3D(로봇)가 필요하므로 **막히는 것이 정상/예상**이다.

### D-0. mission 빌드 (터미널 1)

- **명령**
  ```bash
  cd /ws
  colcon build --symlink-install --packages-select mission
  source /ws/install/setup.bash
  ```
- **확인**: `Finished <<< mission`, 오류 0. (선행으로 C-0의 perception 빌드 필요 — mission이 `perception.msg`/`mission_interfaces` 의존)
- **의미**: mission 패키지가 통합된 `perception.msg` import로 빌드됨 (`mission_a.py:23` `from perception.msg import PartDetectionArray`).
- **실제 사용**: 동일.

### D-1. mission_a 기동 (터미널 2)

- **명령**
  ```bash
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  ros2 run mission mission_a
  ```
  (실행파일명 근거 `mission/setup.py:22`)
- **확인**: `mission_a started in state=INIT (sim_mode=False, …)` 로그.
- **의미**: mission_a 기동. 기본은 **토픽 모드** — `use_task_list_service=False`(`mission_a.py` 파라미터), `/perception/task_list` 구독(`_on_task_list`).
- **실제 사용**: 동일하게 토픽 모드로 구동.

### D-2. 입력 주입 — IDLE + task_list (터미널 3, 4)

mission_a의 INIT→A1 전이는 `/manipulator_state == 'IDLE'` 또는 타임아웃(60s)에서 발생한다. 빠른 확인을 위해 IDLE을 주입한다.

- **명령** (터미널 3 — manipulator IDLE)
  ```bash
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  ros2 topic pub /manipulator_state std_msgs/msg/String "{data: 'IDLE'}" -r 2
  ```
- **명령** (터미널 4 — task_list 입력) — 두 방식 중 택1:
  - (a) **풀 체인**: (C)의 management_node + fake OCR를 그대로 켜 두면 `/perception/task_list`가 자동 채워진다(권장, 통합 전체 경로 확인).
  - (b) **단축**: task_list를 직접 발행
    ```bash
    ros2 topic pub /perception/task_list std_msgs/msg/String \
    "{data: '{\"parts\":[{\"name\":\"flange nut\",\"count\":1},{\"name\":\"gear ring\",\"count\":2},{\"name\":\"spacer ring\",\"count\":1},{\"name\":\"hex nut\",\"count\":4},{\"name\":\"dom nut\",\"count\":2}]}'}" -r 1
    ```
- **확인**(터미널 2 mission_a 로그, 순서대로):
  ```
  [INIT] manipulator IDLE 확인 -> A1_MONITOR
  [state] INIT -> A1_MONITOR
  [A1_MONITOR] task_list 확정: TaskList(dome_nut:2, flange_nut:1, gear_ring:2, hex_nut:4, spacer_ring:1) (총 10) -> A2_SCAN
  [state] A1_MONITOR -> A2_SCAN
  ```
- **의미**: ① mission_a가 서비스 호출 없이 **토픽 구독**으로 task_list 구성, ② **영문 canonical + dom→dome 매핑**이 동작(로그에 `dome_nut:2` 존재 = `'dom nut'`→`'domnut'`→`dome_nut`, `task_list.py:14-31`), ③ `total_remaining>0` → `A1→A2` 전이.
- **실제 사용**: 실환경에서는 management_node가 `/perception/task_list`를 채우고 mission_a가 같은 방식으로 소비.

### D-3. 이후 상태(예상되는 정지점)

- **확인**: A2_SCAN 진입 후 `/perception/wrist/target_one_pose`(wrist planner, 로봇 RGB-D 필요)가 없으므로 `[A2_SCAN] target 미수신 timeout -> RECOVERY`로 진행하다 재시도 한도 후 `MANUAL_WAIT`로 정지.
- **의미**: **정상/예상.** perception 소유 경계(A1→A2 + task_list 구성)는 확인됨. 이후 grasp/manipulation은 로봇·미완성 manipulation 액션 영역.
- **실제 사용**: 실로봇에서는 wrist planner의 target_one_pose가 들어와 A2→A3로 진행.

### D-4. 모드/완료판정 참고

- **`--sim` vs 토픽 모드**: `sim_driver`는 `GetTaskList` **서비스 서버**를 제공한다. `--sim`(또는 sim 사용) 시 task_list를 받으려면 **`-p use_task_list_service:=true`** 필요. 기본(토픽 모드)은 서비스 호출 안 함 — `mission_a.py`의 A1에서 `use_task_list_service` 가드.
- **VERIFY 하이브리드**: 적재 완료 판정은 mission_a **자체 차감(`task_list.decrement`)**이 구동축(트레이 비전 없이도 완료). `/perception/task_list` 잔량은 교차확인·로그용이며 VERIFY 로그에 `topic_remaining=…`로 표시. **순수 토픽 판정**은 `verify_use_topic_remaining:=true`로 켤 수 있으나, 실 트레이 검출이 0이면 management의 `remaining`이 줄지 않아 완료가 안 되므로 **실 트레이 검증 후** 사용. (decision #1-A 하이브리드)

---

## (E) 범주 2 — 로봇/실물 필요: 실행·확인 방법

> 사용자가 로봇(AI Worker)에 연결 후 따라 하는 절차. 각 단계 4요소(명령/확인/의미/실제사용).
> 카메라 토픽·TF·모델이 필요하므로 **로봇 bringup이 선행**되어야 한다.

### E-0. 모델 `.pt` 3종 배치 (빌드 전, 호스트)

- **명령** (호스트에서 `perception/model/`에 배치 — Git 미포함)
  ```bash
  cd ~/AI_Worker_HC/humanoid_challenge/perception/model
  gdown "https://drive.google.com/uc?id=17BepvzEurXIQbh3F9X3SQDCB8iaqLkWC" -O part_detector_best.pt
  gdown "https://drive.google.com/uc?id=14H48riKH3KkKxky2yrCMufPfiGz6gfa0" -O monitor_ocr_best.pt
  gdown --folder "https://drive.google.com/drive/folders/1aPlhEepxsM0mS9x-DXibiJBmqr6Q6LGl" -O /tmp/tray_model
  cp /tmp/tray_model/best.pt tray_occupancy_best.pt        # ★ best.pt → tray_occupancy_best.pt 리네임
  ```
- **확인**: 컨테이너에서 `ls /ws/src/humanoid_challenge/perception/model/` → `part_detector_best.pt monitor_ocr_best.pt tray_occupancy_best.pt`. `./docker/container.sh start` 시 누락 경고가 없어야 함(`container.sh:38-56`).
- **의미**: 노드 기본 모델 경로와 일치 — detector `perception/model/part_detector_best.pt`(`detector_node.py:50`), tray `tray_occupancy_best.pt`(`tray_contents_node.py:23-28`), monitor_ocr `monitor_ocr_best.pt`(`monitor_ocr/ocr_pipeline_parts.py`의 `default_yolo_model_path`).
- **실제 사용**: 모델 없으면 detector/tray YOLO 로드 실패 → 검출 0. 빌드 **전** 배치해야 install symlink 정상.

> 배치 후 컨테이너에서 C-0 빌드를 (재)수행한다.

### E-1. 로봇 bringup (로봇 측 터미널)

- **명령** (AI Worker 컨테이너 내)
  ```bash
  # (T1) wrist 카메라 TF 보정 — 없으면 wrist planner의 base_link 변환 실패
  ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
    --frame-id camera_r_link --child-frame-id camera_right_link
  # (T2) FFW 로봇 bringup (카메라/드라이버/TF)
  ros2 launch ffw_bringup ffw_sg2_ai.launch.py \
    colorizer.enable1:=false colorizer.enable2:=false \
    tf_publish_rate1:=10.0 tf_publish_rate2:=10.0
  ```
- **확인**: `ros2 topic list`에 `/camera_right/camera_right/color/image_rect_raw`, `/zed/zed_node/rgb/image_rect_color`, `/tf`, `/tf_static` 등. (PC 컨테이너에서도 보이면 DDS 연결 OK; ROS_DOMAIN_ID=30 일치)
- **의미**: 카메라 image+TF 공급 = 다운스트림 인지의 입력 전제. static TF는 `camera_r_link→camera_right_link`(identity)로 wrist 광학프레임→base_link 변환 체인을 잇는다.
- **실제 사용**: 이 토픽들이 detector/tray/planner의 입력.

### E-2. 실 detector (PC 컨테이너, 터미널 A)

- **명령**
  ```bash
  ros2 launch perception part_detector.launch.py \
    camera_name:=wrist_right \
    image_topic:=/camera_right/camera_right/color/image_rect_raw \
    detections_topic:=/detections
  ```
  (인자 기본값 근거 `part_detector.launch.py:24-34`; 노드 shebang `#!/ws/yolo_venv/bin/python3` `detector_node.py:1`)
- **확인**: `dome_nut conf≈0.89`, `hex_nut conf≈0.86` 등 검출 로그. `ros2 topic echo /detections --once` → `PartDetectionArray`(각 detection `source_camera=wrist_right`).
- **의미**: 통합 detector가 `perception.msg`로 `/detections` 발행(`detector_node.py:24`). 모델·venv·카메라 정합 확인.
- **실제 사용**: tray_contents_node·wrist planner가 `/detections`를 구독해 부품을 처리.

### E-3. task_management (tray_contents + management) (터미널 B)

- **명령**
  ```bash
  ros2 launch perception task_management.launch.py \
    image_topic:=/camera_right/camera_right/color/image_rect_raw \
    source_camera_filter:=wrist_right
  ```
  (tray_contents_node prefix `/ws/yolo_venv` + management_node 동시기동 `task_management.launch.py:44-46,89`; tray 모델 기본 `perception/model/tray_occupancy_best.pt`)
- **확인**: `TrayContentsNode ready`, `Loading tray YOLO model: …/tray_occupancy_best.pt`, `ManagementNode ready`. `ros2 topic echo /perception/tray_contents` → `tray_count`(트레이 보이면 ≥1). `ros2 topic echo /perception/task_list` → canonical JSON.
- **확인(트레이 미검출 시)**: `tray_count: 0`이면 트레이를 시야에 두고 재시도, 또는 `-p tray_conf_threshold:=0.30`(낮춤)로 재확인.
- **의미**: 실 트레이 검출(tray_count≥1) 시 management가 `remaining=ocr−tray`로 적재 진행분을 차감. tray=0이어도 `/perception/task_list`는 정상 발행(`publish_on_empty_tray`).
- **실제 사용**: 적재가 진행되면 트레이 내 부품이 잡혀 `remaining`이 줄고 mission 완료 판정에 반영.

### E-4. wrist grasp planner (터미널 C)

- **명령** (디버그 파라미터 예시)
  ```bash
  ros2 run perception wrist_task_grasp_planner_node --ros-args \
    --params-file /ws/src/humanoid_challenge/perception/config/wrist_projection/params.yaml \
    -p temporal_smoothing_enable:=false \
    -p min_score_to_publish:=0.0 \
    -p hold_last_pose_sec:=10.0
  ```
  (파라미터명 `temporal_smoothing_enable` 근거 `wrist_task_grasp_planner_node.py:140`; params.yaml에 `class_alias_json` "dom nut"→dome_nut 포함 `params.yaml:162-164`)
- **확인**: `SELECT [flange_nut] score=… -> base_link (x, y, z) m` + 후보 랭킹 로그. `ros2 topic echo /perception/wrist/target_one_pose` → `frame_id: base_link`의 PoseStamped.
- **의미**: ① `/perception/task_list`(canonical 영문)와 detector class를 `_canonical_label`로 매칭(영문 별칭 추가로 dome nut도 매칭, `params.yaml:162-164`), ② 출력계약 `/perception/wrist/target_one_pose`(base_link) 정상 — **불변**.
- **실제 사용**: 이 PoseStamped가 manipulation(다운스트림)의 grasp 목표. (manipulation은 미완성 — 범위 밖)

### E-5. 추가 미검증분 (각각 실행·확인 방법)

| 항목 | 실행 | 확인 | 의미 |
|---|---|---|---|
| **실 monitor_ocr_topic_node** (실 OCR로 `/monitor_ocr/result`) | `/ws/ocr_venv/bin/python3 /ws/install/perception/lib/perception/monitor_ocr_topic_node --ros-args -p image_topic:=/zed/zed_node/rgb/image_rect_color` (★ocr_venv 필요) | `ros2 topic echo /monitor_ocr/result` → parts JSON. 사전 `/ws/ocr_venv/bin/python3 -c "import paddleocr"` 정상 | fake OCR 대신 실 OCR가 같은 토픽을 채움 → 모니터 화면 입력으로 OCR 동작 |
| **head 경로** (`perception_2d_to_pcd` head) | `ros2 launch perception head_all.launch.py` (ZED 토픽+TF 필요) | head 3D/pose 산출 로그·토픽 | head ZED 2D→3D 변환 |
| **wrist_pointcloud_node** | `ros2 run perception wrist_pointcloud_node` (wrist RGB-D 필요) | pointcloud 토픽 발행 | planner와 별개 단독 노드 |
| **dome nut end-to-end grasp** | E-2~E-4 진행 중 시야에 dome nut 배치 | planner 랭킹/`SELECT [dome_nut]` 출현, target_one_pose 발행 | `"dom nut"`→`dome_nut` 별칭이 실검출과 매칭되는지(통합 신규 정합) |
| **mission_a 라이브 연동** | E-1~E-4 + `ros2 run mission mission_a` | A1→A2→(target 수신 시)A3 진행 | perception 실출력으로 FSM end-to-end |

---

## (F) 검증 결과 기록용 체크리스트

### 범주 1 (로봇 불필요) — [통과/실패/비고]

| 단계 | 확인 포인트 | 결과 | 비고 |
|---|---|---|---|
| C-0 | `--packages-up-to perception` 빌드 성공 | | |
| C-1 | 신규 3노드 `ros2 pkg executables` 노출 | | |
| C-2 | `ManagementNode ready` | | |
| C-3 | fake OCR `publishing`, management 경고 없음 | | |
| C-4 | `/perception/task_list` canonical JSON(`dom nut` 포함) | | |
| C-5 | (선택) fake tray로 `remaining` 차감 | | |
| C-6 | 토픽 info 발행/구독 배선 | | |

### 범주 1-2 (mission_a) — [통과/실패/비고]

| 단계 | 확인 포인트 | 결과 | 비고 |
|---|---|---|---|
| D-0 | `mission` 빌드 성공 | | |
| D-1 | `mission_a started … state=INIT` | | |
| D-2 | `INIT→A1_MONITOR→A2_SCAN`, task_list에 `dome_nut` 매핑 | | |
| D-3 | A2 이후 target 미수신 정지(예상) | | |

### 범주 2 (로봇/실물 필요) — [실행함/미실행/비고]

| 단계 | 확인 포인트 | 결과 | 비고 |
|---|---|---|---|
| E-0 | 모델 3종 배치 | | |
| E-1 | 로봇 bringup(카메라/TF) | | |
| E-2 | detector 검출 로그(`dome_nut conf≈0.89`) | | |
| E-3 | tray_contents/management 토픽, tray_count | | |
| E-4 | planner `SELECT…→base_link`, target_one_pose | | |
| E-5 | 실 OCR / head / pointcloud / dome grasp / mission 라이브 | | |

---

## (G) 안내

> **범주 1·1-2 확인 통과 후, git commit & push는 사용자가 별도로 직접 진행 예정입니다.** (본 가이드 작성자는 git 작업을 수행하지 않습니다.)
