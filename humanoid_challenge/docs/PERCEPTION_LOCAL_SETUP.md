# Perception 로컬 실행 셋업 체크리스트
> **목적**: Perception 팀 docker 이미지 + venv 셋업 받은 후 즉시 실행까지 가는 단계별 절차
> **전제**: [PERCEPTION_INTERFACE.md](./PERCEPTION_INTERFACE.md) "실행 전 준비 사항" 표 참고

---

## 🟢 실행 검증 현황 (2026-05-30 세션 — 로컬 직접 구동)

이번 세션에서 로컬 컨테이너 + 실로봇 bringup으로 실제 구동·검증한 결과.

**환경 구축 (전부 완료)**
| 항목 | 상태 | 비고 |
|------|------|------|
| 이미지 `ros2_jazzy_robotis_perception:latest` | ✅ 빌드 (3.34GB) | `~/robotis_docker/Dockerfile` 로 빌드 (ros:jazzy 베이스) |
| `~/robotis_ros2_ws` (도커 `/ws`) | ✅ AI_Worker_HC 통합본과 동기화 | planner 노드·`best.pt` 포함 |
| `ocr_venv` | ⚠️ paddleocr 3.6 / paddlepaddle 3.3 / ultralytics 설치됨, **의존성 일부 누락** | 아래 monitor_ocr 블로커 |
| `yolo_venv` | ✅ ultralytics(→torch) | detector 용 |
| 시스템 python `open3d`/`scipy` | ⚠️ 설치 실패 흔적 (psutil 충돌) | 2d_to_pcd 계열 필요 시 `--ignore-installed psutil` 로 재설치 |
| colcon build (4 패키지) | ✅ 완료 | `/ws/install/` |
| 로봇 bringup (카메라/TF/ZED) | ✅ 토픽 발행 확인 | `/zed/.../left/image_rect_color`, `/camera_*`, `/tf` |

**노드 검증**
| 노드 | 검증 | 메모 |
|------|------|------|
| `detector_node` (head/wrist) | ✅ 정상 작동 | yolo_venv |
| `projection_node` / `wrist_projection_node` | ✅ 정상 작동 | |
| `wrist_pointcloud_node` / grasp_pcd | ✅ 정상 작동 | |
| `wrist_task_grasp_planner_node` | ✅ 정상 작동 | launch로 실행 (아래 런북) |
| `monitor_ocr_node` | ⚠️ **블로커** | 노드 자체는 YOLO+PaddleOCR init·구독까지 정상. 단 ocr_venv 의존성 누락으로 `import paddleocr` 체인 실패 |

### ⚠️ 두 가지 핵심 함정 (꼭 숙지)

**① `ros2 run` 으로 venv 노드 실행 불가 (shebang 문제)**
colcon이 설치한 실행파일의 첫 줄이 `#!/usr/bin/python3` (시스템 python)이라,
`source venv/bin/activate` 후 `ros2 run` 해도 venv가 무시되어 `ModuleNotFoundError` 발생.
→ **venv python으로 노드 파일을 직접 실행**하거나 shebang을 덮어써야 함:
```bash
# 방법 A: venv python 직접 실행
/ws/ocr_venv/bin/python /ws/install/monitor_ocr/lib/monitor_ocr/monitor_ocr_node \
  --ros-args -p parts_mode:=true
# 방법 B: 설치본 shebang을 venv python으로 1회 수정 → 이후 ros2 run/launch 정상
sed -i '1s|.*|#!/ws/ocr_venv/bin/python|' /ws/install/monitor_ocr/lib/monitor_ocr/monitor_ocr_node
```
> detector(yolo_venv)도 동일 원리 — launch가 작동했다면 yolo_venv 경로/shebang이 이미 맞춰진 상태.

**② monitor_ocr `ocr_venv` 의존성 누락 (재현된 블로커)**
ocr_venv 생성 시점에 (실패한 open3d `--break-system-packages` 설치가) 시스템 python에
`urllib3`/`requests`/`typing_extensions`/`tqdm`/`scipy` 등을 잠깐 깔아둔 상태였고,
`--system-site-packages` venv라 pip이 "이미 있음"으로 보고 **그 의존성들을 venv 안에 안 깔았음**.
컨테이너 재생성 후 시스템 패키지가 사라지자 ocr_venv에서 줄줄이 `ModuleNotFoundError`.
→ **권장 수정 (깨끗한 컨테이너에서 ocr_venv 재생성)**:
```bash
rm -rf /ws/ocr_venv
python3 -m venv --system-site-packages /ws/ocr_venv
/ws/ocr_venv/bin/pip install --upgrade pip
/ws/ocr_venv/bin/pip install paddleocr paddlepaddle "numpy<2" ultralytics opencv-python
# 시스템 오염이 없으면 pip이 전체 의존성을 venv 안에 정상 설치
```
> 빠른 우회(재다운로드 회피): `pip check` + 런타임 에러로 누락분 직접 설치 — 단 whack-a-mole 위험.

---

## 단계 0 — Perception 팀에 요청한 자산

| 항목 | 상태 | 비고 |
|------|------|------|
| Docker 이미지 `ros2_jazzy_robotis_perception:latest` (Dockerfile 또는 tar) | ⬜ 수령 대기 | |
| venv 셋업 스크립트 / requirements (ocr_venv, yolo_venv) | ⬜ 수령 대기 | |

> 자산 수령 위치 (예시): `~/Downloads/ros2_jazzy_robotis_perception.tar`, `~/Downloads/install_venvs.sh`

---

## 단계 1 — Docker 이미지 로딩

### 방식 A: docker save된 tar 파일을 받은 경우
```bash
sudo docker load -i ~/Downloads/ros2_jazzy_robotis_perception.tar
sudo docker images ros2_jazzy_robotis_perception   # 확인
```

### 방식 B: Dockerfile을 받은 경우
```bash
# Perception 팀 빌드 컨텍스트가 perception-ws 안에 있다고 가정
cd ~/perception-ws
sudo docker build -t ros2_jazzy_robotis_perception:latest -f Dockerfile .
```

### 방식 C: 사설 registry / docker pull
```bash
sudo docker pull <registry>/ros2_jazzy_robotis_perception:latest
sudo docker tag  <registry>/ros2_jazzy_robotis_perception:latest \
                 ros2_jazzy_robotis_perception:latest
```

---

## 단계 2 — `~/robotis_ros2_ws` 워크스페이스 준비

> **현재 상태 (2026-05-30 확인)**: 소유권은 이미 `jihun` 으로 해제됨. `~/robotis_ros2_ws/src/` 에
> perception 4개 패키지 복사본이 있으나 **stale** — AI_Worker_HC 통합본과 비교 시 다음이 빠져 있음:
> - ❌ `perception/.../wrist_task_grasp_planner_node.py` + `wrist_task_grasp_planner.launch.py` (cherry-pick된 최신 planner)
> - ❌ `perception/model/part_detector_best.pt`, `perception/model/monitor_ocr_best.pt` (모델 가중치)
> - ⚠️ `params.yaml`, `setup.py`, `wrist_all.launch.py` 가 구버전
>
> **→ 실행 전 반드시 아래 re-sync 필요.** (`save_image.py` 잔여 파일도 정리)

```bash
# stale src 를 AI_Worker_HC 통합본으로 재동기화 (planner + .pt 포함)
rsync -a --delete \
  ~/AI_Worker_HC/humanoid_challenge/monitor_ocr \
  ~/AI_Worker_HC/humanoid_challenge/perception \
  ~/AI_Worker_HC/humanoid_challenge/perception \
  ~/AI_Worker_HC/humanoid_challenge/perception \
  ~/AI_Worker_HC/humanoid_challenge/tray_occupancy \
  ~/robotis_ros2_ws/src/
rm -f ~/robotis_ros2_ws/save_image.py

# 재동기화 확인 (아래 3개 모두 존재해야 함)
ls ~/robotis_ros2_ws/src/perception/perception/wrist_task_grasp_planner_node.py
ls ~/robotis_ros2_ws/src/perception/model/part_detector_best.pt
ls ~/robotis_ros2_ws/src/perception/model/monitor_ocr_best.pt
```

> ⚠️ re-sync 후에는 **단계 5 colcon 재빌드 필수** (planner 노드가 새로 install 되어야 함).

<details><summary>참고: 최초 셋업(소유권 root)일 때의 원래 절차</summary>

```bash
# 1. 소유권 해제
sudo chown -R $USER:$USER ~/robotis_ros2_ws
chmod u+rwx ~/robotis_ros2_ws

# 2. 무관한 파일 정리 (선택)
rm -f ~/robotis_ros2_ws/save_image.py

# 3. perception src 복사 (AI_Worker_HC 통합본 사용 권장 — planner + .pt 포함)
mkdir -p ~/robotis_ros2_ws/src
cp -r \
  ~/AI_Worker_HC/humanoid_challenge/monitor_ocr \
  ~/AI_Worker_HC/humanoid_challenge/perception \
  ~/AI_Worker_HC/humanoid_challenge/perception \
  ~/AI_Worker_HC/humanoid_challenge/perception \
  ~/AI_Worker_HC/humanoid_challenge/tray_occupancy \
  ~/robotis_ros2_ws/src/

# 4. 확인
ls ~/robotis_ros2_ws/src/
# 기대 결과:
#   monitor_ocr  perception  perception  perception  tray_occupancy

# 5. .pt 파일 동기화 확인 (.gitignore라 cp -r에 따라옴)
ls ~/robotis_ros2_ws/src/perception/model/part_detector_best.pt
ls ~/robotis_ros2_ws/src/perception/model/monitor_ocr_best.pt
```

`~/robotis_ppm_captures` 디렉토리도 root 소유면 동일하게 해제:
```bash
ls -la ~/robotis_ppm_captures/   # owner 확인
sudo chown -R $USER:$USER ~/robotis_ppm_captures
```

</details>

---

## 단계 3 — Docker 컨테이너 진입

```bash
xhost +local:root

sudo docker run -it --rm \
  --name ros2_jazzy_robotis \
  --network host \
  --ipc host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/robotis_ros2_ws:/ws \
  -v ~/robotis_ppm_captures:/captures \
  ros2_jazzy_robotis_perception:latest \
  bash
```

추가 터미널 진입:
```bash
sudo docker exec -it ros2_jazzy_robotis bash
```

---

## 단계 4 — venv 셋업 (Perception 팀이 준 스크립트 또는 수동)

### Perception 팀 install 스크립트 받은 경우
```bash
# 컨테이너 안
cd /ws
bash /captures/install_venvs.sh   # 또는 받은 경로
```

### 수동 셋업 (확정 의존성 — 2026-05-30 코드 import + deploy.sh 기준)

> **출처**: upstream `monitor_ocr/deploy.sh`(PaddleOCR `numpy<2` 명시) + 4개 패키지 실제 import 전수 추출.
> 레포에 `requirements.txt`/`install_venvs.sh` 는 없음 — 아래가 코드 기준 권위 목록.

⚠️ **반드시 `--system-site-packages`** : venv 안에서 ROS의 `cv_bridge`, `rclpy`, `message_filters`, `tf2_ros` 를
import 해야 하므로 시스템 site-packages 가 보여야 함. 일반 venv 면 `import cv_bridge` 부터 실패.

```bash
# 컨테이너 안
cd /ws

# ocr_venv  (monitor_ocr: paddleocr, paddlepaddle, numpy<2, ultralytics, cv2)
python3 -m venv --system-site-packages ocr_venv
source ocr_venv/bin/activate
pip install --upgrade pip
pip install paddleocr paddlepaddle "numpy<2" ultralytics opencv-python
deactivate

# yolo_venv  (perception: ultralytics→torch 자동, cv2, numpy)
python3 -m venv --system-site-packages yolo_venv
source yolo_venv/bin/activate
pip install --upgrade pip
pip install ultralytics opencv-python numpy
deactivate
```

**⚠️ 시스템 python 추가 의존성 (2d_to_pcd / 2d_to_pcd_wrist 용 — 이미지에 미포함)**
`projection/pointcloud/grasp_pcd/planner` 노드는 venv 없이 시스템 python 으로 도는데
`open3d`, `scipy` 를 import 한다. 이미지 Dockerfile 엔 둘 다 없으므로 컨테이너에서 추가 설치:
```bash
pip install --break-system-packages open3d scipy
# (PEP 668 externally-managed 환경이라 --break-system-packages 필요)
```

> 미확정 1건: detector 의 torch 가 **CPU vs CUDA** 빌드인지(ultralytics 기본은 환경에 맞춰 설치).
> GPU 사용 시 Perception 팀의 torch 설치 인덱스 확인 권장.

---

## 단계 5 — colcon 빌드

```bash
# 컨테이너 안
cd /ws
source /opt/ros/jazzy/setup.bash

colcon build --packages-select \
  perception \
  monitor_ocr \
  perception \
  perception

source /ws/install/setup.bash

# 확인
ros2 pkg list | grep -E "monitor_ocr|perception"
ls /ws/install/perception/share/perception/model/part_detector_best.pt
ls /ws/install/monitor_ocr/share/perception/model/monitor_ocr_best.pt
```

빌드 실패 시 자주 보는 에러:
- `perception messages not built` → `perception` 가 다른 노드보다 먼저 빌드되어야 함. `--packages-select` 순서대로 강제: `colcon build --packages-up-to perception`
- `ament_python` 비표준 레이아웃 (`detector_node.py` 가 루트에 있음) → `py_modules` 가 setup.py에 잡혀 있어야 함

---

## 단계 6 — 로봇 측 bringup (별도 머신, SSH)

> **컨테이너 A — bringup**
```bash
ssh robotis@ffw-SNPR48A1087.local
cd ~/ai_worker
./docker/container.sh enter

ros2 launch ffw_bringup ffw_sg2_ai.launch.py \
  colorizer.enable1:=false colorizer.enable2:=false \
  tf_publish_rate1:=10.0 tf_publish_rate2:=10.0
```

> **컨테이너 B — TF bridge** (`camera_r_link ↔ camera_right_link` 이름 불일치 해소)
```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.0 \
  --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id camera_r_link \
  --child-frame-id camera_right_link
```

TF 연결 확인 (값이 계속 출력되면 OK):
```bash
ros2 run tf2_ros tf2_echo base_link camera_right_color_optical_frame
```

---

## 단계 7 — 노드 실행 (로컬 컨테이너)

[PERCEPTION_INTERFACE.md "실행 가이드"](./PERCEPTION_INTERFACE.md#실행-가이드--로컬-도커-컨테이너) 섹션 참고. 요약:

각 터미널 공통 환경:
```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

| 터미널 | 추가 venv | 실행 |
|--------|----------|------|
| T1 (OCR) | `source /ws/ocr_venv/bin/activate` | `ros2 run perception monitor_ocr_node --ros-args -p parts_mode:=true` |
| T2 (Detector) | `source /ws/yolo_venv/bin/activate` | `ros2 launch perception part_detector.launch.py camera_name:=wrist_right` |
| T3 (Projection) | (system) | `ros2 launch perception wrist_projection.launch.py` |
| T4 (Pointcloud) | (system) | `ros2 launch perception wrist_pointcloud.launch.py` |
| T5 (Grasp PCD) | (system) | `ros2 launch perception wrist_grasp_pcd.launch.py` |
| T6 (Planner) ⭐ | (system) | `ros2 run perception wrist_task_grasp_planner_node --ros-args --params-file /ws/src/perception/config/wrist_projection/params.yaml` |

---

## 단계 8 — 동작 검증 (별도 터미널)

```bash
# 컨테이너 안
ros2 topic list -t | grep -E "detect|perception|monitor_ocr"

# 핵심 토픽 1회 출력
ros2 topic echo --once /monitor_ocr/result
ros2 topic echo --once /detections
ros2 topic echo --once /perception/wrist/target_one_pose

# 주기
ros2 topic hz /perception/wrist/mask_cloud

# rqt overlay
export QT_X11_NO_MITSHM=1
ros2 run rqt_image_view rqt_image_view
# dropdown → /detector_debug_image
```

End-to-end OK 기준:
- `/monitor_ocr/result` JSON에 `latest_screen_detected: true` + `parts` 배열
- `/detections` PartDetectionArray 발행 (≥1Hz)
- `/perception/wrist/target_one_pose` PoseStamped 발행 + `frame_id: base_link`
- 로그에 `[head] class_name -> base_link (x, y, z) m conf=...` 패턴

---

## 트러블슈팅 자주 보는 이슈

| 증상 | 원인 | 조치 |
|------|------|------|
| `Unable to find image 'ros2_jazzy_robotis_perception:latest'` | 이미지 미설치 | 단계 1 |
| `source /ws/ocr_venv/bin/activate: No such file` | venv 미생성 | 단계 4 |
| `Package 'monitor_ocr' not found` | colcon 빌드 안 됨 | 단계 5 |
| `cannot import name 'PartDetectionArray'` | 메시지 패키지 빌드 순서 | `colcon build --packages-up-to perception` |
| `TF base_link → camera_right_color_optical_frame failed` | TF bridge 미실행 또는 base_link 미발행 | 단계 6 컨테이너 B 확인 |
| `ros2 topic list`에 로봇 토픽 안 보임 | DDS daemon 캐시 | `ros2 daemon stop && ros2 daemon start` |
| best.pt 못 찾음 | colcon install share 위치 차이 | `--ros-args -p model_path:=/ws/src/perception/model/part_detector_best.pt` 로 명시 |
| `ModuleNotFoundError` (paddleocr/ultralytics 등) | venv shebang 무시 / ocr_venv 의존성 누락 | 위 "두 가지 핵심 함정" ①② |

---

## 부록 A — 노드별 정식 실행 런북 (Perception 팀 가이드 통합)

> 각 노드는 **별도 터미널**에서 실행. 모든 터미널 공통:
> `source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash` +
> `export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`
> 추가 터미널 진입: `docker exec -it ros2_jazzy_robotis bash` (sudo 불필요 — docker 그룹)

### A-0. 로봇 측 (별도 머신, SSH) — 유지
```bash
# 컨테이너 A — bringup
ssh robotis@ffw-SNPR48A1087.local        # 비번: root
cd ~/ai_worker && ./docker/container.sh enter
ros2 launch ffw_bringup ffw_sg2_ai.launch.py \
  colorizer.enable1:=false colorizer.enable2:=false \
  tf_publish_rate1:=10.0 tf_publish_rate2:=10.0

# 컨테이너 B — TF bridge (camera_r_link ↔ camera_right_link 이름 불일치 해소)
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.0 --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id camera_r_link --child-frame-id camera_right_link
# 확인 (값이 계속 출력되면 TF 연결됨)
ros2 run tf2_ros tf2_echo base_link camera_right_color_optical_frame
```

### A-1. monitor_ocr (ocr_venv) ⚠️ 블로커 — 위 함정 ②  해결 후
```bash
source /ws/ocr_venv/bin/activate
which python    # → /ws/ocr_venv/bin/python 확인
# ros2 run 이 shebang 때문에 실패하면 venv python 직접 실행 (함정 ①)
/ws/ocr_venv/bin/python /ws/install/monitor_ocr/lib/monitor_ocr/monitor_ocr_node \
  --ros-args -p parts_mode:=true
# launch 도 제공됨: ros2 launch perception monitor_ocr.launch.py
# overlay viewer (새 터미널):
/ws/ocr_venv/bin/python /ws/install/monitor_ocr/lib/monitor_ocr/monitor_ocr_viewer \
  --ros-args -p image_topic:=/zed/zed_node/rgb/image_rect_color
```
발행: `/monitor_ocr/result`(JSON), `/parts`, `/part_counts`, `/recognized`

### A-2. detector (yolo_venv) ✅
```bash
source /ws/yolo_venv/bin/activate
ros2 daemon stop && ros2 daemon start     # rqt 토픽 인식 안 될 때
ros2 launch perception part_detector.launch.py camera_name:=head        # head
ros2 launch perception part_detector.launch.py camera_name:=wrist_left  # wrist L
ros2 launch perception part_detector.launch.py camera_name:=wrist_right # wrist R
# 정상 로그: "Loading model from ..." → "PartDetectorNode ready."
# 확인: ros2 topic echo --once /detections   /   rqt → /detector_debug_image
```

### A-3. head ZED 파이프라인 (2d_to_pcd) ✅
```bash
# 터미널1: detector(head) + 터미널2:
ros2 launch perception head_projection.launch.py
# (옵션) ros2 run perception pointcloud_node / grasp_pcd_node
# 확인:
ros2 topic hz /perception/head/rgb /perception/head/depth /perception/head/camera_info
ros2 topic echo --once /perception/head/target_pose
# 정상 로그: [head] class_name -> base_link (x, y, z) m conf=...
```
발행: `/perception/head/target_pose`, `/perception/head/mask_cloud`, `/perception/head/target_pcd/<class>`

### A-4. wrist 파이프라인 (2d_to_pcd_wrist) ✅
```bash
# detector(wrist_right)가 먼저 돌고 있어야 함
ros2 launch perception wrist_projection.launch.py    # wrist_projection_node
ros2 launch perception wrist_pointcloud.launch.py    # wrist_pointcloud_node
# 최종 target 1개 (planner) — launch + params_file
ros2 launch perception wrist_task_grasp_planner.launch.py \
  params_file:=/ws/src/perception/config/wrist_projection/params.yaml
```
발행: `/perception/wrist/target_pose`(per-det), `/perception/wrist/mask_cloud`,
`/perception/wrist/target_pcd/<class>`, **`/perception/wrist/target_one_pose`**(planner, mission_a 입력)

### A-5. rqt 이미지 뷰 (X11)
```bash
export QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
ros2 run rqt_image_view rqt_image_view    # 좌상단 dropdown → /detector_debug_image
```

### A-6. tray_occupancy — 트레이 검출 + 잔여 task list (신규 `demo/senario_A`)
```bash
# OCR 결과와 wrist image가 준비되어 있어야 함. 트레이 YOLO 모델 별도 필요.
ros2 launch perception task_management.launch.py \
  tray_model_path:=/ws/src/humanoid_challenge/perception/model/tray_occupancy_best.pt
# 확인
ros2 topic echo --once /perception/tray_roi        # 최신 tray bbox
ros2 topic echo --once /perception/task_list       # GetTaskList_Response typed task list
```
- `tray_manage_node` 는 yolo_venv prefix로 실행된다.

> **💡 launch `prefix=` 가 venv shebang 문제(함정 ①)를 해결**: 신규 launch들(`monitor_ocr.launch.py`,
> `tray_occupancy.launch.py`)은 `prefix="/ws/ocr_venv/bin/python3"` 처럼 venv python을 직접 지정해
> 노드를 띄운다. 그래서 **launch로 실행하면 `ros2 run` shebang 문제가 안 생긴다.**
> ```bash
> ros2 launch perception monitor_ocr.launch.py   # ocr_venv prefix 내장 — 함정 ① 우회
> ```
