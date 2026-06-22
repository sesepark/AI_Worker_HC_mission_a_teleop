# Perception 팀 코드 인터페이스
> **최종 업데이트**: 2026-05-30 (upstream `demo/senario_A` 반영 — 신규 `tray_occupancy` 패키지: 트레이 검출 + 태스크 리스트 관리, 연결 launch)
> **기준 레포**: `~/AI_Worker_HC/humanoid_challenge/` (통합본)
> **Perception 코드 upstream (진실의 원천)**: [hublemon/Humanoid-Challenge-Perception](https://github.com/hublemon/Humanoid-Challenge-Perception) — **브랜치 `fix/wrist-task-grasp-stability`** (⚠️ `main` 아님)
>   - upstream은 `src/` 하위에 패키지 배치. 우리 통합본은 Perception 패키지들을 `humanoid_challenge/` 아래에 배치한 것.
>   - **2026-05-30 전수 대조 결과: 64개 파일 모두 blob 해시 일치** (`.pt` 가중치 제외). 핵심 추가분 `wrist_task_grasp_planner_node.py` 포함 완전 동기화됨.
> **참조 원본**:
> - Notion: [Perception 4주차](https://www.notion.so/87a7502383c283ed8ad501586ae02aa7)
>   - 하위: [perception (head)](https://www.notion.so/2f87502383c283089e578106278d0d0c), [perception](https://www.notion.so/3ef7502383c28274a42b81864b5b5c43), [메시지 양식 확인 wrist 용](https://www.notion.so/4e87502383c28221b91201ce413843f0)
> - Slack: Perception 팀 채널 도커 컨테이너 진입·노드 실행 가이드
> **상태 범례**: ✅ 검증 완료 | 🔄 진행 중 | ⬜ 개발 예정 | ⚠️ 이슈

---

## 📣 Perception 팀 공식 완료 현황 (2026-05-30 팀 보고)

퍼셉션 팀이 직접 보고한 완료/진행 상태. 아래 "노드별 상세"(레포 정적 분석 기준 9개)와
**기능 단위 6개 노드**로 매핑된다.

| # | 기능 (팀 표현) | 완료 상태 | 매핑되는 레포 노드 | 비고 |
|---|----------------|-----------|--------------------|------|
| ① | 모니터 OCR 추출 (지령 인식 → 종류별 대상·태스크 리스트 생성) | ✅ 완료 | `monitor_ocr_node` | 실모니터 파싱은 YOLO bbox 전환 진행 중 |
| ② | 부품 종류 판정 (바운딩 박스) | ✅ 완료 | `detector_node` | YOLO11s-seg 5종. **confidence: wrist 단독 0.75~0.80, head 기준 0.8 이상.** head↔wrist 동일 개체 대응 완료 |
| ③ | 그래스프 타겟 플래닝 및 3D 좌표 전달 | ✅ 완료 | `projection_node` / `wrist_projection_node` | per-detection base_link 중심 좌표 발행 |
| ④ | 전체 장면 포인트 클라우드 발행 | ✅ 완료 | `pointcloud_node` / `wrist_pointcloud_node` | base_link 월드 좌표 변환 후 Manipulation 전달 완료, **Slack 검증 확인** |
| ⑤ | 탑 1 그래스프 캔디데이트 중심 좌표 발행 | ✅ 완료 | `wrist_task_grasp_planner_node` | **최대 5개 후보 → score 랭킹 → top-1 중심 3D 좌표 별도 토픽** (`/perception/wrist/target_one_pose`) |
| ⑥ | 트레이 인식 및 task list 노드 | ✅ **구현됨** | `tray_manage_node` | OCR 결과 + wrist image → typed `/perception/task_list`와 `/perception/tray_roi` 발행 |

> **객체별 정제 PCD**(`grasp_pcd_node` / `wrist_grasp_pcd_node`)는 팀 기능 뷰에서 ④(PCD 생성·전달)에 포함.
> 레포에는 head/wrist 분리 + 정제 PCD 노드가 따로 있어 정적 분석 기준 노드 수가 9개로 더 많다.

**핵심 추가 정보 (이번 보고로 확정)**
- detector confidence 실측: **wrist 단독 0.75~0.80 / head 0.8 이상** — 미션 임계값 설정 시 wrist는 0.75 근처로 낮게 잡아야 누락 없음.
- head ↔ wrist **동일 개체 대응(cross-camera)** 완료 → planner `cross camera` score 항(0.03)이 실제 동작.
- PCD가 이미 **base_link 월드 좌표로 변환되어 Manipulation에 전달·Slack 검증 완료** → GPD 입력은 좌표 변환 불필요, 토픽명만 정정하면 됨.
- ⑥ 트레이/task list 노드 **완성** → `mission_a` `VERIFY`/A1_MONITOR가 **`/perception/task_list`(`GetTaskList_Response`)** 를 구독해 공용 typed task list를 사용. (§11 참고)

---

## ⚠️ 실행 전 준비 사항 (Task 2 정적 검증으로 발견)

`~/AI_Worker_HC/humanoid_challenge/` 의 Perception 패키지만으로는 실행 불가. 추가로 필요한 자산:

| 항목 | 상태 | 출처 / 우회 |
|------|------|-------------|
| YOLO 모델 `perception/model/part_detector_best.pt` | ✅ 배치 완료 (2026-05-30) | [Google Drive: part_detector best.pt](https://drive.google.com/file/d/17BepvzEurXIQbh3F9X3SQDCB8iaqLkWC/view) |
| YOLO 모델 `perception/model/monitor_ocr_best.pt` | ✅ 배치 완료 (2026-05-30) | [Google Drive: monitor_ocr best.pt](https://drive.google.com/file/d/14H48riKH3KkKxky2yrCMufPfiGz6gfa0/view) |
| YOLO 모델 `perception/model/tray_occupancy_best.pt` (파랑 트레이) | ✅ 배치+로드 검증 (2026-06-01, segment 모델 class `blue_tray`) | [Google Drive: blue_tray_yolo](https://drive.google.com/drive/folders/1MzRzf27wtmPqp8-KqR9iH_AnrLsgaPOU?usp=sharing) |
| `wrist_task_grasp_planner_node` 코드 | ✅ feature 브랜치에서 cherry-pick 완료 (2026-05-30) | `fix/wrist-task-grasp-stability` 브랜치의 5개 파일 (planner_node.py, planner.launch.py, wrist_all.launch.py, setup.py, params.yaml) 반영 |
| Docker 이미지 `ros2_jazzy_robotis_perception:latest` | ❌ 로컬 미존재, Dockerfile 미포함 | Perception 팀 docker registry / 빌드 스크립트 별도 보유 |
| `~/robotis_ros2_ws` 워크스페이스 | ❌ root 소유, 비어 있음 | `sudo chown -R $USER:$USER ~/robotis_ros2_ws` 후 perception src 복사 |
| 호스트 ROS 2 jazzy | ❌ `/opt/ros/` 없음 | 도커 안에서만 실행 가능 (정상) |
| AI worker 로봇 (`ffw-SNPR48A1087.local`) | ✅ ping 4.7ms 도달 | 로봇 켜져 있고 LAN 접근 가능 |
| Head ZED / Wrist RealSense | ✅ (로봇 측 USB) | 호스트엔 미연결, ROS 2 토픽으로 수신 |

### 모델 파일 배치 방법

`.pt` 파일은 용량 크고 `.gitignore`로 제외되므로 git에는 안 들어감. 다음 경로에 직접 배치:

```bash
# 1. Perception 팀에서 모델 받기 (Google Drive)
#    - part_detector_best.pt:
#      https://drive.google.com/file/d/17BepvzEurXIQbh3F9X3SQDCB8iaqLkWC/view
#    - monitor_ocr_best.pt:
#      https://drive.google.com/file/d/14H48riKH3KkKxky2yrCMufPfiGz6gfa0/view
#    - tray_occupancy_best.pt:
#      https://drive.google.com/drive/folders/1MzRzf27wtmPqp8-KqR9iH_AnrLsgaPOU?usp=sharing

# 2. 정위치로 복사
PERC=~/AI_Worker_HC/humanoid_challenge
mkdir -p $PERC/perception/model
cp ~/Downloads/part_detector_best.pt   $PERC/perception/model/part_detector_best.pt
cp ~/Downloads/monitor_ocr_best.pt     $PERC/perception/model/monitor_ocr_best.pt
cp ~/Downloads/tray_occupancy_best.pt  $PERC/perception/model/tray_occupancy_best.pt
```

**확인**: perception 패키지 share 디렉토리 구조
- `detector_node` 기본 경로: `<pkg_share>/perception/model/part_detector_best.pt`
  - 소스 트리: `humanoid_challenge/perception/model/part_detector_best.pt`
  - 오버라이드: `--ros-args -p model_path:=/path/to/best.pt`
- `monitor_ocr_node` 기본 경로: `<pkg_share>/perception/model/monitor_ocr_best.pt`
  - 소스 트리: `humanoid_challenge/perception/model/monitor_ocr_best.pt`
  - 오버라이드: `--ros-args -p yolo_model_path:=/path/to/best.pt`

**정적 검증 결과** (2026-05-30):
- ✅ 38개 Python 파일 모두 문법 OK
- ✅ 4개 `package.xml` 모두 valid XML
- ✅ 9개 launch.py 모두 `LaunchDescription` 정상 호출
- ✅ wrist `package.xml` 의존성 완전 (cv_bridge, message_filters, tf2_*, scipy, numpy, opencv 등)
- ⚠️ `detector_node.py` 가 패키지 ROOT 위치 (모듈 디렉토리 안이 아님) — setup.py가 `py_modules`로 처리하는 비표준 ament_python 구조

---

## ⚠️ System 팀 주의 — 토픽명 정정 이력

| 이전 문서 표기 | 실제 발행 토픽 | 영향 |
|----------------|----------------|------|
| `/target_pose` | **`/perception/wrist/target_pose`** (per-detection 중심점, wrist_projection_node) | `mission_a.py` A2_SCAN 구독 토픽 수정 필요 |
| `/target_pose` (Manipulation 입력) | **`/perception/wrist/target_one_pose`** (task list 필터링 후 최종 1개, wrist_task_grasp_planner_node) | grasp_filter 입력 토픽 재검토 필요 |
| `/camera_right/points_base` | **`/perception/wrist/mask_cloud`** (전체 mask 합본) / **`/perception/wrist/target_pcd/<class>`** (객체별) | GPD 입력 토픽명 정정 필요 |
| `/detections` | 동일 — 정확함 | — |
| `/monitor_ocr/result` | 동일 — 정확함 | — |

→ Step 2 `mission_a.py` 실로직 진입 전 위 토픽명 일괄 적용 + Manipulation 팀 확인 필요.

---

## 노드 목록

레포 정적 분석 기준 9개 (기능 단위로는 6개 — 위 "공식 완료 현황" 참고) + 신규 트레이 노드 1개.
미션 A에서 직접 쓰는 핵심 5개는 ★ 표시.

> **🟢 로컬 구동 검증 (2026-05-30 세션)**: 로컬 컨테이너 + 실로봇 bringup으로 **monitor_ocr 제외 전 노드 정상 작동 확인**.
> 환경 구축·검증 상세·실행 런북은 [PERCEPTION_LOCAL_SETUP.md](./PERCEPTION_LOCAL_SETUP.md) "실행 검증 현황" + "부록 A 런북" 참고.

| 노드 | 패키지 | 미션 A 역할 | 상태 (로컬 검증) |
|------|--------|-------------|------|
| ★ `monitor_ocr_node` | `monitor_ocr` | A.1 지령 모니터 OCR 파싱 | ⚠️ **블로커** — 노드 init·구독 정상이나 ocr_venv 의존성 누락 (SETUP 함정 ②). 실모니터 YOLO bbox 전환도 진행 중 |
| ★ `detector_node` | `perception` | A.2 부품 검출·분류 (head + wrist) | ✅ 로봇 검증 (conf: wrist 0.75~0.80 / head 0.8+, cross-camera 대응) |
| `projection_node` | `perception` | head 2D→3D center pose | ✅ 로봇 검증 |
| `pointcloud_node` | `perception` | head 전체 mask PointCloud2 | ✅ 로봇 검증 |
| `grasp_pcd_node` | `perception` | head 객체별 정제 PCD | ✅ 로봇 검증 |
| ★ `wrist_projection_node` | `perception` | wrist 2D→3D center pose (per-detection) | ✅ 로봇 검증 |
| ★ `wrist_pointcloud_node` | `perception` | wrist 전체 mask PointCloud2 | ✅ 로봇 검증 (Manipulation 전달·Slack 확인) |
| ★ `wrist_grasp_pcd_node` | `perception` | wrist 객체별 정제 PCD (GPD 입력 후보) | ✅ 로봇 검증 |
| ★ `wrist_task_grasp_planner_node` | `perception` | task list 반영 최종 target 1개 (최대 5후보→top-1) | ✅ 로봇 검증 — `wrist_task_grasp_planner.launch.py params_file:=...` 로 실행 |
| `tray_manage_node` | `perception` | OCR 결과 + wrist image → `/perception/task_list`, `/perception/tray_roi` | ✅ 구현 |

**내부 파이프라인**
```
detector_node → wrist_projection_node       (per-detection 중심점)
              → wrist_pointcloud_node       (장면 전체 mask cloud)
              → wrist_grasp_pcd_node        (객체별 grasp용 정제 PCD)
              → wrist_task_grasp_planner_node (task list로 최종 1개 선택)
```

---

## 노드별 상세

### 1. `monitor_ocr_node` — A.1 지령 파싱

**패키지**: `monitor_ocr`

**실행**
```bash
ros2 run perception monitor_ocr_node --ros-args -p parts_mode:=true
```

**Subscribe**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/zed/zed_node/left/image_rect_color` | `sensor_msgs/msg/Image` | Head ZED 카메라 |

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/monitor_ocr/result` | `std_msgs/msg/String` (JSON) | OCR 파싱 결과 |

**`/monitor_ocr/result` JSON 구조**
```json
{
  "frames_used": 10,
  "parts": [
    {"name": "플랜지 너트", "count": 1},
    {"name": "기어 링",    "count": 2},
    {"name": "스페이서 링","count": 1},
    {"name": "육각 너트",  "count": 4},
    {"name": "돔 너트",    "count": 2}
  ],
  "latest_elapsed_ms": 1234.5,
  "latest_screen_detected": true
}
```

**System 팀 요청사항**
- `/monitor_ocr/result` → ROS 2 **Action** 구조로 감싸기 (OCR 처리 시간 고려)
- `/command_valid` 는 내부(internal)에서만 사용
- OCR 실패 시 fallback: 10초 딜레이 후 강제 OK 출력 (점수 10점 확보)

**현재 이슈**
- OpenCV 방식 실모니터 파싱 실패 → YOLO 기반 모니터 bbox 검출로 전환 중

**디버그 viewer** — OCR overlay 보기
```bash
/ws/ocr_venv/bin/python /ws/install/monitor_ocr/lib/monitor_ocr/monitor_ocr_viewer \
  --ros-args -p image_topic:=/zed/zed_node/rgb/image_rect_color
```

---

### 2. `detector_node` — A.2 부품 검출·분류

**패키지**: `perception`

**실행** (launch 권장 — camera_name 파라미터로 head/wrist_left/wrist_right 분기)
```bash
ros2 launch perception part_detector.launch.py camera_name:=head
ros2 launch perception part_detector.launch.py camera_name:=wrist_left
ros2 launch perception part_detector.launch.py camera_name:=wrist_right
```

**Subscribe**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/zed/zed_node/left/image_rect_color` | `sensor_msgs/msg/Image` | Head ZED |
| `/camera_left/camera_left/color/image_rect_raw` | `sensor_msgs/msg/Image` | Wrist Left |
| `/camera_right/camera_right/color/image_rect_raw` | `sensor_msgs/msg/Image` | Wrist Right |

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/detections` | `perception/msg/PartDetectionArray` | 부품 detection 배열 |
| `/detector_debug_image` | `sensor_msgs/msg/Image` | bbox/mask overlay |

**`PartDetectionArray`**
```
std_msgs/Header header
PartDetection[] detections
```

**`PartDetection`**
```
int32   class_id
string  class_name        # flange_nut / gear_ring / spacer_ring / hex_nut / dome_nut
float32 confidence
int32[] bbox              # [x1, y1, x2, y2]
string  source_camera     # head / wrist_left / wrist_right
float32 center_x
float32 center_y
float32[] mask_x          # segmentation polygon
float32[] mask_y
```

**5종 부품 class**
| class_id | class_name | 한국어 |
|----------|-----------|--------|
| 0 | `flange_nut`  | 플랜지 너트 |
| 1 | `gear_ring`   | 기어 링 |
| 2 | `spacer_ring` | 스페이서 링 |
| 3 | `hex_nut`     | 육각 너트 |
| 4 | `dome_nut`    | 돔 너트 |

**모델**: YOLO11s-seg, mAP50 = 0.993

**System 팀 요청사항**
- command target 부품 별도 표시 기능
- confidence threshold 이하 부품 별도 표시
- wrist cam 기준 추가 학습 반영 후 재검증

---

### 3. `projection_node` (head) — A.4 2D→3D 중심점

**패키지**: `perception`
**내부 노드 이름**: `projection_2d_to_pcd`

**실행**
```bash
ros2 run perception projection_node
# 또는: ros2 launch perception head_projection.launch.py
```

**Subscribe**
| 토픽 | 타입 |
|------|------|
| `/zed/zed_node/rgb/image_rect_color` | `sensor_msgs/msg/Image` |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/msg/Image` |
| `/zed/zed_node/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/detections` | `PartDetectionArray` |
| `/tf`, `/tf_static` | (내부 구독) |

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/perception/head/target_pose` | `geometry_msgs/msg/PoseStamped` | base_link 기준 3D 중심점 (per-detection) |
| `/perception/head/rgb` | `sensor_msgs/msg/Image` | 동기화 RGB |
| `/perception/head/depth` | `sensor_msgs/msg/Image` | 동기화 Depth |
| `/perception/head/camera_info` | `sensor_msgs/msg/CameraInfo` | 동기화 CameraInfo |

**동작 메모**
- `source_camera == "head"` detection만 처리
- mask 있으면 mask 내부 depth median, 없으면 center 주변 window(5px) median
- orientation은 항상 단위 쿼터니언 (`w=1.0`)

---

### 4. `pointcloud_node` (head) — A.5 장면 전체 PCD

**패키지**: `perception`
**내부 노드 이름**: `mask_to_pointcloud`

**실행**
```bash
ros2 run perception pointcloud_node
```

**Subscribe**: projection_node와 동일

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/perception/head/mask_cloud` | `sensor_msgs/msg/PointCloud2` | 탐지된 모든 mask 합본 XYZRGB cloud (base_link) |

---

### 5. `grasp_pcd_node` (head) — A.6 객체별 grasp용 PCD

**패키지**: `perception`

**실행**
```bash
ros2 run perception grasp_pcd_node
```

**Subscribe**: projection_node와 동일

**Publish** (클래스마다 토픽 동적 생성)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/perception/head/target_pcd/<class_name>` | `sensor_msgs/msg/PointCloud2` | 정제된 객체별 국소 PCD |

예: `/perception/head/target_pcd/hex_nut`, `/perception/head/target_pcd/flange_nut`

**처리 파이프라인**
1. 2D mask 기반 3D 역투영 (벡터화 NumPy)
2. 범위 필터 → Mask Erosion → SOR (Statistical Outlier Removal)
3. TF로 base_link 변환 후 객체별 발행

**주요 파라미터** (`config/params.yaml`)
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `base_frame` | `base_link` | 출력 기준 |
| `depth_scale` | `0.001` | 16-bit mm → m |
| `min_depth_m` / `max_depth_m` | `0.1` / `5.0` | 유효 depth 범위 |
| `mask_erosion_px` | `2` | 경계 노이즈 제거 |
| `sor_k_neighbors` | `20` | SOR 이웃 수 |
| `camera_name` | `head` | detection source 필터 |

---

### 6. `wrist_projection_node` — B.4 wrist 2D→3D 중심점

**패키지**: `perception`

**실행**
```bash
ros2 run perception wrist_projection_node \
  --ros-args --params-file /ws/src/perception/config/wrist_projection/params.yaml
# 또는: ros2 launch perception wrist_projection.launch.py
```

**Subscribe** ⚠️ head와 달리 RGB Info + Depth Info 둘 다 필요 (RGB↔Depth 비정렬)
| 토픽 | 타입 |
|------|------|
| `/camera_right/camera_right/color/image_rect_raw` | `sensor_msgs/msg/Image` |
| `/camera_right/camera_right/depth/image_rect_raw` | `sensor_msgs/msg/Image` |
| `/camera_right/camera_right/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera_right/camera_right/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/detections` | `PartDetectionArray` |
| `/tf`, `/tf_static` | (내부 구독) |

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| **`/perception/wrist/target_pose`** | `geometry_msgs/msg/PoseStamped` | base_link 기준 3D 중심점 (per-detection) |
| `/perception/wrist/rgb` | `sensor_msgs/msg/Image` | 동기화 RGB |
| `/perception/wrist/depth` | `sensor_msgs/msg/Image` | 동기화 Depth |

**중심점 산출 로직** (head 방식 ≠ wrist 방식 — 중요)
```
depth 픽셀 + Z   --K_depth^-1-->  depth frame 3D 점
                 --[R|t]-------->  color frame 3D 점   (depth→color extrinsics)
                 --K_rgb-------->  RGB 이미지 평면 (u, v) 픽셀
RGB mask 내부 (u, v) 점만 선택 → 그 3D 점들의 coordinate-wise median = 중심
```
- bbox center 픽셀의 depth를 직접 읽지 않음 (RGB/depth 해상도·frame 다름)
- median이라 outlier(배경 픽셀, mask 경계 ghost)에 robust
- depth→color extrinsics는 TF에서 `camera_right_color_optical_frame ← camera_right_depth_optical_frame` lookup, 실패 시 params.yaml fallback
- orientation 미계산, 단위 쿼터니언 고정

**Wrist Depth intrinsics (확인된 값, 480x270)**
```
fx = fy = 246.20
cx = 240.82
cy = 134.63
frame_id: camera_right_depth_optical_frame
```

**Depth→Color extrinsics** (TF 실패 시 fallback)
```
rotation ≈ I_3
translation ≈ [-9.7e-06, 1.0e-05, 1.0e-05]  # 두 센서 물리적으로 매우 가까움
```

**전제 조건**
- TF tree: `camera_right_color_optical_frame → base_link` 경로 필요
- 과거 `camera_r_link ↔ camera_right_link` 이름 불일치로 끊긴 적 있음
- 검증: `ros2 run tf2_ros tf2_echo base_link camera_right_color_optical_frame` 으로 확인

**관측된 에러**
```
[WARN] TF camera_right_color_optical_frame → base_link failed:
       "base_link" passed to lookupTransform argument target_frame does not exist.
```
→ AI worker 컨테이너 B에서 `static_transform_publisher`로 우회 (아래 실행 가이드 참고)

---

### 7. `wrist_pointcloud_node` — B.5 wrist 장면 전체 PCD

**패키지**: `perception`
**내부 노드 이름**: `wrist_mask_to_pointcloud`

**실행**
```bash
ros2 run perception wrist_pointcloud_node \
  --ros-args --params-file /ws/src/perception/config/wrist_projection/params.yaml
# 또는: ros2 launch perception wrist_pointcloud.launch.py
```

**Subscribe**: wrist_projection_node와 동일 (RGB + Depth + 두 CameraInfo + /detections + TF)

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| **`/perception/wrist/mask_cloud`** | `sensor_msgs/msg/PointCloud2` | 탐지된 모든 mask 합본 XYZRGB cloud (base_link) |

> ⚠️ 이전 문서의 `/camera_right/points_base`는 잘못된 이름. Manipulation GPD 입력 토픽 확인 필요.

---

### 8. `wrist_grasp_pcd_node` — B.6 wrist 객체별 grasp용 PCD

**패키지**: `perception`

**실행**
```bash
ros2 run perception wrist_grasp_pcd_node \
  --ros-args --params-file /ws/src/perception/config/wrist_projection/params.yaml
```

**Subscribe**: wrist_projection_node와 동일

**Publish** (클래스마다 토픽 동적 생성)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/perception/wrist/target_pcd/<class_name>` | `sensor_msgs/msg/PointCloud2` | 정제된 객체별 국소 PCD (base_link) |

**처리 파이프라인**: head 버전과 동일 (depth→color→RGB mask filter → erosion → SOR → TF)

**주요 파라미터**
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `base_frame` | `base_link` | 출력 기준 |
| `depth_scale` | `0.001` | 16-bit mm → m |
| `min_depth_m` / `max_depth_m` | `0.1` / **`3.0`** | wrist는 근접 작업이라 head보다 짧음 |
| `mask_erosion_px` | `2` | 경계 노이즈 제거 |
| `sor_k_neighbors` | `20` | SOR 이웃 수 |
| `camera_name` | `wrist_right` | detection source 필터 |
| `use_tf_for_extrinsics` | `true` | depth→color TF lookup, 실패 시 params fallback |

---

### 9. `wrist_task_grasp_planner_node` — B.7 task list 반영 최종 target 1개 ⭐

**패키지**: `perception`

**용도**: `/monitor_ocr/result`의 task list를 기준으로 현재 필요한 부품만 필터링 + wrist detection 후보를 점수화 + 가장 집기 좋은 후보 1개의 중심 좌표를 publish.

> **System 팀**: `mission_a.py` A2_SCAN/A3_PICK이 실질적으로 구독해야 할 토픽은 **이 노드의 출력**.

**실행**
```bash
ros2 run perception wrist_task_grasp_planner_node \
  --ros-args --params-file /ws/src/perception/config/wrist_projection/params.yaml
```

**Subscribe**
| 토픽 | 타입 |
|------|------|
| `/monitor_ocr/result` | `std_msgs/msg/String` (JSON) |
| `/detections` | `PartDetectionArray` |
| `/camera_right/camera_right/color/image_rect_raw` | `sensor_msgs/msg/Image` |
| `/camera_right/camera_right/depth/image_rect_raw` | `sensor_msgs/msg/Image` |
| `/camera_right/camera_right/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera_right/camera_right/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/tf`, `/tf_static` | (내부 구독) |

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| **`/perception/wrist/target_one_pose`** | `geometry_msgs/msg/PoseStamped` | task + grasp score 기준 선택된 부품 1개의 base_link 중심 좌표 |

**전체 흐름**
```
/monitor_ocr/result 수신
    ↓
task list (한국어 부품명) → detector class_name 변환
    ↓
/detections 수신 → task 일치 + source_camera == "wrist_right" 후보만 선택
    ↓
후보별 grasp score 계산 (8개 metric weighted sum)
    ↓
top-1 후보 선택 → mask 내부 wrist depth point의 median = 중심
    ↓
camera_right_color_optical_frame → base_link TF 변환
    ↓
/perception/wrist/target_one_pose publish
```

**Grasp score = weighted sum (default 가중치)**
| 항목 | 가중치 | 의미 |
|------|--------|------|
| confidence | 0.25 | detector confidence |
| mask quality | 0.20 | mask 안 valid depth point 풍부도 |
| occlusion | 0.15 | depth z 분포 안정성 (IQR 기반) |
| bbox size | 0.10 | ideal ≈ 4% 화면 면적 |
| screen center | 0.10 | 화면 중앙 근접도 |
| overlap | 0.10 | 다른 bbox와 IoU (낮을수록 ↑) |
| boundary | 0.07 | tray ROI 경계와의 거리 |
| cross camera | 0.03 | head에서도 같은 class 보이면 bonus |

**Stable target 안정화 로직** (최근 추가, params.yaml)
```yaml
temporal_smoothing_enable: true
temporal_window_sec: 0.8      # 최근 0.8초 후보 누적
temporal_min_observations: 2  # 최소 2번 관측되어야 publish
temporal_position_gate_m: 0.06  # 같은 class라도 6cm 이내만 같은 target으로 묶음
republish_last_pose_hz: 2.0   # echo --once 타이밍 보호용 재발행
hold_last_pose_sec: 2.0
```
- "Waiting for stable target observation..." 로그 = 안정화 대기 중
- 흔들리면 `temporal_min_observations: 3` 권장, 너무 안 나오면 `1`

**현재 한계** (manipulation에 넘기기 전 보완 필요)
- orientation 미계산 — 단위 쿼터니언 고정
- gripper 접근 방향 / grasp width / surface normal / collision-free 검증 없음
- **결론**: `/perception/wrist/target_one_pose`는 **pre-grasp target point**로 사용. 최종 grasp pose는 `/perception/wrist/target_pcd/<class>`와 함께 manipulation에서 계산.

**디버그 로그 예시**
```
[INFO] Active task classes: dome_nut:2, flange_nut:1, gear_ring:2, hex_nut:4, spacer_ring:1
[INFO] Candidate ranking:
  #1 gear_ring  score=0.860 conf=0.93 mask=0.87 occ=0.81 size=0.99 center=0.75 ...
  #2 hex_nut    score=0.849 conf=0.94 mask=0.86 occ=0.80 size=0.99 center=0.88 ...
```

---

### 10~11. `tray_occupancy` 패키지 — 트레이 검출 + 태스크 리스트 관리 ✅ (`demo/senario_A`)

> **신규 패키지** (upstream `demo/senario_A`, 커밋 `eb595b1f`). 트레이 비전으로 적재 진행을
> 자동 추적해 **잔여 task list** 를 발행. mission_a 의 OCR 파싱·자체 차감을 대체한다.
> 두 노드를 잇는 launch: `tray_occupancy.launch.py`.

#### 10. `tray_occupancy_node` — 파란 트레이 검출 + 부품 카운트

**실행** (별도 트레이 YOLO 모델 필요, yolo_venv prefix)
```bash
ros2 launch perception tray_occupancy.launch.py \
  tray_model_path:=/ws/src/humanoid_challenge/perception/model/tray_occupancy_best.pt
```

**Subscribe**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/detections` | `PartDetectionArray` | 부품 bbox (트레이 내부 판정용) |
| `/zed/zed_node/rgb/image_rect_color` | `Image` | 트레이 YOLO 입력 |

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/perception/tray_roi` | `sensor_msgs/RegionOfInterest` | 최신 tray bbox |

트레이 YOLO 모델(`tray_model_path`)로 파란 트레이 bbox를 검출하고 안정화된 최신 ROI를 발행한다.

#### 11. `tray_manage_node` — OCR task list + tray ROI

**Subscribe**
| 토픽 | 타입 |
|------|------|
| `/monitor_ocr/result` | `String`(JSON) — OCR 목표 수량 |
| wrist image topic | `sensor_msgs/Image` — tray YOLO 입력 |

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| **`/perception/task_list`** | `mission_interfaces/srv/GetTaskList_Response` | typed task list state |

**`/perception/task_list` message**
```yaml
success: false
message: '{"ocr_topic":"/monitor_ocr/result","timeout_sec":30.0,"frame_count":10}'
screen_detected: true
all_counts_recognized: true
frames_used: 10
parts:
  - {name: "flange nut", count: 1}
  - {name: "gear ring", count: 0}
  - {name: "spacer ring", count: 0}
  - {name: "hex nut", count: 2}
  - {name: "dom nut", count: 0}
```
- `require_complete_ocr=true` 면 OCR 에 5종이 다 안 잡힌 프레임은 무시(이전 목표 유지) → 안정적.
- `/perception/get_task_list` service도 같은 response 계약을 반환한다.

**미션 A 영향 (중요)**
- `mission_a` 는 OCR 파싱·한국어 매핑을 직접 할 필요 없이 `/perception/task_list` 한 토픽으로:
  - A1_MONITOR: 첫 `/perception/task_list` 수신 → 목표 확정
  - VERIFY: 최신 `/perception/task_list` 의 `total==0` 이면 DONE, 아니면 A2_SCAN 복귀
- 단 canonical `name`(공백 표기) ↔ detector class_name 변환 테이블 필요 (`mission/task_list.py` 에 추가).
- place pose(A3_PLACE)용 트레이 3D 위치/적재영역은 아직 별도 협의 필요(트레이 PCD/centroid).

---

## 전체 토픽 흐름

```
[Head ZED] ──────────────────────────────────→ monitor_ocr_node
                                                       │
                                                /monitor_ocr/result
                                                       │
                                  ┌────────────────────┴────────────────────┐
                                  ↓                                         ↓
                          mission_a.py (A1)                  wrist_task_grasp_planner_node
                                                                              │
[Head ZED]   ──┐                                                              │
[Wrist Left] ──┤                                                              │
[Wrist Right]──┴──→ detector_node ──→ /detections ─────────────────────────┐  │
                                  └──→ /detector_debug_image (UI)          │  │
                                                                            ↓  ↓
                                              ┌─────────────────────────────────┐
                                              │ wrist_projection_node           │ /perception/wrist/target_pose (per-det)
                                              │ wrist_pointcloud_node           │ /perception/wrist/mask_cloud
                                              │ wrist_grasp_pcd_node            │ /perception/wrist/target_pcd/<class>
                                              │ wrist_task_grasp_planner_node   │ /perception/wrist/target_one_pose ★★
                                              └─────────────────────────────────┘
                                              (head 버전: /perception/head/* 대응)
                                                                              │
                                                            mission_a.py (A2_SCAN/A3_PICK)
                                                            + Manipulation grasp_filter / GPD
```

---

## 실행 가이드 — 로컬 도커 컨테이너

> 이 섹션은 Perception 팀 Slack 가이드를 재정리한 것. 이후 다른 팀원이 동일 환경을 빠르게 띄울 수 있게 한 곳에 통합.

### 0. 도커 컨테이너 진입

호스트에서:
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

추가 터미널에서 같은 컨테이너 진입:
```bash
sudo docker exec -it ros2_jazzy_robotis bash
```

### 1. 컨테이너 내 공통 환경

```bash
cd /ws

# 노드별로 활성화할 venv가 다름
# - monitor_ocr  : /ws/ocr_venv
# - detector     : /ws/yolo_venv
# - 2d_to_pcd*   : venv 없이 시스템 python

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

ROS 2 daemon 캐시 문제 시:
```bash
ros2 daemon stop
ros2 daemon start
```

### 2. AI worker 측 (별도 머신, 유지)

**컨테이너 A — bringup**
```bash
ssh robotis@ffw-SNPR48A1087.local
cd ~/ai_worker
./docker/container.sh enter
ros2 launch ffw_bringup ffw_sg2_ai.launch.py \
  colorizer.enable1:=false colorizer.enable2:=false \
  tf_publish_rate1:=10.0 tf_publish_rate2:=10.0
```

**컨테이너 B — TF bridge** (`camera_r_link` ↔ `camera_right_link` 이름 불일치 우회)
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

### 3. 노드별 실행 순서 (로컬 컨테이너)

| 순서 | 노드 | venv | 명령어 |
|------|------|------|--------|
| 1 | `monitor_ocr_node` | ocr_venv | `ros2 run perception monitor_ocr_node --ros-args -p parts_mode:=true` |
| 2 | `detector_node` (head/wrist) | yolo_venv | `ros2 launch perception part_detector.launch.py camera_name:=wrist_right` |
| 3a | `wrist_projection_node` | (system) | `ros2 launch perception wrist_projection.launch.py` |
| 3b | `wrist_pointcloud_node` | (system) | `ros2 launch perception wrist_pointcloud.launch.py` |
| 3c | `wrist_grasp_pcd_node` | (system) | `ros2 launch perception wrist_grasp_pcd.launch.py` |
| 3d | `wrist_task_grasp_planner_node` | (system) | `ros2 run perception wrist_task_grasp_planner_node --ros-args --params-file /ws/src/perception/config/wrist_projection/params.yaml` |

> ⚠️ `wrist_pointcloud_node`와 `wrist_grasp_pcd_node`는 사용자가 둘 다 필요한지 미션 의도에 맞게 결정. 단일 부품 grasp만 필요하면 `wrist_grasp_pcd_node` + `wrist_task_grasp_planner_node` 조합으로 충분.

### 4. 동작 확인

```bash
# 토픽 목록 (detector 관련만)
ros2 topic list -t | grep -E "detector|perception|monitor_ocr"

# detection 1회 확인
ros2 topic echo --once /detections

# 최종 target 1회 확인
ros2 topic echo --once /perception/wrist/target_one_pose

# 발행 주기 확인
ros2 topic hz /perception/wrist/mask_cloud
ros2 topic hz /perception/wrist/target_pose

# rqt overlay
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
ros2 run rqt_image_view rqt_image_view
# 좌상단 dropdown → /detector_debug_image
```

### 5. Head 카메라 변형 — A.4/A.5/A.6

3D 파이프라인을 head ZED 기준으로 돌릴 때:
```bash
ros2 launch perception part_detector.launch.py camera_name:=head
ros2 launch perception head_projection.launch.py
# (옵션) ros2 run perception pointcloud_node
# (옵션) ros2 run perception grasp_pcd_node
```

발행 토픽:
```
/perception/head/target_pose         (per-detection 중심점)
/perception/head/mask_cloud          (장면 전체 cloud)
/perception/head/target_pcd/<class>  (객체별 정제 PCD)
/perception/head/rgb, /depth, /camera_info  (동기화 신호)
```

확인:
```bash
ros2 topic hz /perception/head/rgb
ros2 topic hz /perception/head/depth
ros2 topic hz /perception/head/camera_info
ros2 topic echo --once /perception/head/target_pose
```

정상 로그:
```
[head] class_name -> base_link (x, y, z) m conf=...
```

---

## 메시지 양식 확인 (실측, wrist)

**Wrist depth camera info** (`ros2 topic echo /camera_right/camera_right/depth/camera_info --once`)
```
frame_id: camera_right_depth_optical_frame
height: 270
width: 480
k: fx=246.20, fy=246.20, cx=240.82, cy=134.63
distortion_model: plumb_bob   # d = [0,0,0,0,0]
```

**Wrist extrinsics depth_to_color** (`ros2 topic echo /camera_right/camera_right/extrinsics/depth_to_color --once`)
```
rotation ≈ I_3
  [0.99999, -0.00159, -0.00311,
   0.00159,  0.99999,  0.00044,
   0.00311, -0.00044,  0.99999]
translation ≈ [-9.7e-06, 1.0e-05, 1.0e-05]   # 거의 0
```

**YOLO detector wrist 입력 토픽** (`detector_node.py` DEFAULT_IMAGE_TOPICS 확인)
```
/camera_left/camera_left/color/image_rect_raw
/camera_right/camera_right/color/image_rect_raw
/zed/zed_node/rgb/image_rect_color
```

---

## 블로커 및 TODO

| 항목 | 담당 | 기한 | 상태 |
|------|------|------|------|
| `mission_a.py` 구독 토픽 정정 (`/target_pose` → `/perception/wrist/target_one_pose`) | System | 5.31 | ✅ (2026-05-30 적용) |
| Manipulation GPD 입력 토픽 정정 (`/camera_right/points_base` → `/perception/wrist/mask_cloud` 또는 `/perception/wrist/target_pcd/<class>`) | Manipulation | 5.31 | ⬜ |
| `tray_manage_node` 발행 `/perception/task_list` mission_a 연동 (VERIFY/A1) | System | 6.1 | ✅ |
| 트레이 base_link place 좌표(A3_PLACE용) 인터페이스 협의 — 현재 `/perception/tray_roi` 는 2D bbox | Perception+Manipulation | 6.1 | ⬜ |
| 트레이 YOLO 모델 `tray_occupancy_best.pt` 배치 ([Drive](https://drive.google.com/drive/folders/1MzRzf27wtmPqp8-KqR9iH_AnrLsgaPOU?usp=sharing)) → `perception/model/tray_occupancy_best.pt` | System | 6.1 | ✅ (2026-06-01 배치·로드 검증) |
| TF tree `camera_right_color_optical_frame → base_link` 확인 / launch 등록 | Perception | 이번 주 | 🔄 |
| `monitor_ocr_node` YOLO 전환 + 실모니터 검증 | Perception | 이번 주 | 🔄 |
| wrist cam 기준 추가 학습 반영 후 `detector_node` 재검증 | Perception | 이번 주 | ⬜ |
| `/monitor_ocr/result` Action 구조 변환 | Perception | 협의 후 | ⬜ |
| command target 부품 별도 표시 기능 | Perception | 협의 후 | ⬜ |
| `wrist_task_grasp_planner_node` stable 로직 튜닝 (`temporal_min_observations`) | Perception | 검증 후 | 🔄 |
| `detection_inference` 다수결 stable 로직 (Notion 메모 사항) | Perception | 협의 후 | ⬜ |

---

## Claude Code 세션 시작 시 확인 명령어

```bash
# 패키지 위치
find ~/AI_Worker_HC/humanoid_challenge -maxdepth 2 -name "package.xml" 2>/dev/null

# 토픽 발행 확인 (로봇 연결 시)
ros2 topic echo --once /monitor_ocr/result
ros2 topic echo --once /detections
ros2 topic echo --once /perception/wrist/target_pose
ros2 topic echo --once /perception/wrist/target_one_pose
ros2 topic hz /perception/wrist/mask_cloud

# TF 연결
ros2 run tf2_ros tf2_echo base_link camera_right_color_optical_frame
```
