# Perception 팀 코드 인터페이스
> **최종 업데이트**: 2026-05-30 (Task 1 — Notion·Slack 출처와 대조하여 토픽명·노드 목록 정정)
> **기준 레포**: `~/AI_Worker_HC/robotis_applications/perception/` (snu-shape/AI_Worker_HC `main`)
> **참조 원본**:
> - Notion: [Perception 4주차](https://www.notion.so/87a7502383c283ed8ad501586ae02aa7)
>   - 하위: [perception_2d_to_pcd (head)](https://www.notion.so/2f87502383c283089e578106278d0d0c), [perception_2d_to_pcd_wrist](https://www.notion.so/3ef7502383c28274a42b81864b5b5c43), [메시지 양식 확인 wrist 용](https://www.notion.so/4e87502383c28221b91201ce413843f0)
> - Slack: Perception 팀 채널 도커 컨테이너 진입·노드 실행 가이드
> **상태 범례**: ✅ 검증 완료 | 🔄 진행 중 | ⬜ 개발 예정 | ⚠️ 이슈

---

## ⚠️ 실행 전 준비 사항 (Task 2 정적 검증으로 발견)

`~/AI_Worker_HC/robotis_applications/perception/` 의 코드만으로는 실행 불가. 추가로 필요한 자산:

| 항목 | 상태 | 출처 / 우회 |
|------|------|-------------|
| YOLO 모델 `perception_part_detector/weights/best.pt` | ✅ 배치 완료 (2026-05-30) | [Google Drive: part_detector best.pt](https://drive.google.com/file/d/17BepvzEurXIQbh3F9X3SQDCB8iaqLkWC/view) |
| YOLO 모델 `monitor_ocr/best.pt` | ✅ 배치 완료 (2026-05-30) | [Google Drive: monitor_ocr best.pt](https://drive.google.com/file/d/14H48riKH3KkKxky2yrCMufPfiGz6gfa0/view) |
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

# 2. 정위치로 복사 (파일명 best.pt로 통일)
PERC=~/AI_Worker_HC/robotis_applications/perception
mkdir -p $PERC/perception_part_detector/weights
cp ~/Downloads/part_detector_best.pt $PERC/perception_part_detector/weights/best.pt
cp ~/Downloads/monitor_ocr_best.pt   $PERC/monitor_ocr/best.pt
```

**확인**: ament_python share 디렉토리 구조 (setup.py 기준)
- `detector_node` 기본 경로: `<pkg_share>/perception_part_detector/weights/best.pt`
  - 소스 트리: `perception/perception_part_detector/weights/best.pt`
  - 오버라이드: `--ros-args -p model_path:=/path/to/best.pt`
- `monitor_ocr_node` 기본 경로: `<pkg_share>/monitor_ocr/best.pt`
  - 소스 트리: `perception/monitor_ocr/best.pt`
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

총 9개 (기존 4개 → 5개 추가 발견). 미션 A에서 직접 쓰는 핵심 5개는 ★ 표시.

| 노드 | 패키지 | 미션 A 역할 | 상태 |
|------|--------|-------------|------|
| ★ `monitor_ocr_node` | `monitor_ocr` | A.1 지령 모니터 OCR 파싱 | 🔄 YOLO 전환 중 |
| ★ `detector_node` | `perception_part_detector` | A.2 부품 검출·분류 (head + wrist) | 🔄 학습완료, 로봇 검증 필요 |
| `projection_node` | `perception_2d_to_pcd` | head 2D→3D center pose | ✅ |
| `pointcloud_node` | `perception_2d_to_pcd` | head 전체 mask PointCloud2 | ✅ |
| `grasp_pcd_node` | `perception_2d_to_pcd` | head 객체별 정제 PCD | ✅ |
| ★ `wrist_projection_node` | `perception_2d_to_pcd_wrist` | wrist 2D→3D center pose (per-detection) | ✅ |
| ★ `wrist_pointcloud_node` | `perception_2d_to_pcd_wrist` | wrist 전체 mask PointCloud2 | ✅ |
| ★ `wrist_grasp_pcd_node` | `perception_2d_to_pcd_wrist` | wrist 객체별 정제 PCD (GPD 입력 후보) | ✅ |
| ★ `wrist_task_grasp_planner_node` | `perception_2d_to_pcd_wrist` | task list 반영 최종 target 1개 선택 | 🔄 stable 로직 추가 중 |

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
ros2 run monitor_ocr monitor_ocr_node --ros-args -p parts_mode:=true
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

**패키지**: `perception_part_detector`

**실행** (launch 권장 — camera_name 파라미터로 head/wrist_left/wrist_right 분기)
```bash
ros2 launch perception_part_detector detector.launch.py camera_name:=head
ros2 launch perception_part_detector detector.launch.py camera_name:=wrist_left
ros2 launch perception_part_detector detector.launch.py camera_name:=wrist_right
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
| `/detections` | `perception_part_detector/msg/PartDetectionArray` | 부품 detection 배열 |
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

**패키지**: `perception_2d_to_pcd`
**내부 노드 이름**: `projection_2d_to_pcd`

**실행**
```bash
ros2 run perception_2d_to_pcd projection_node
# 또는: ros2 launch perception_2d_to_pcd projection.launch.py
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

**패키지**: `perception_2d_to_pcd`
**내부 노드 이름**: `mask_to_pointcloud`

**실행**
```bash
ros2 run perception_2d_to_pcd pointcloud_node
```

**Subscribe**: projection_node와 동일

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/perception/head/mask_cloud` | `sensor_msgs/msg/PointCloud2` | 탐지된 모든 mask 합본 XYZRGB cloud (base_link) |

---

### 5. `grasp_pcd_node` (head) — A.6 객체별 grasp용 PCD

**패키지**: `perception_2d_to_pcd`

**실행**
```bash
ros2 run perception_2d_to_pcd grasp_pcd_node
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

**패키지**: `perception_2d_to_pcd_wrist`

**실행**
```bash
ros2 run perception_2d_to_pcd_wrist wrist_projection_node \
  --ros-args --params-file /ws/src/perception_2d_to_pcd_wrist/config/params.yaml
# 또는: ros2 launch perception_2d_to_pcd_wrist wrist_projection.launch.py
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

**패키지**: `perception_2d_to_pcd_wrist`
**내부 노드 이름**: `wrist_mask_to_pointcloud`

**실행**
```bash
ros2 run perception_2d_to_pcd_wrist wrist_pointcloud_node \
  --ros-args --params-file /ws/src/perception_2d_to_pcd_wrist/config/params.yaml
# 또는: ros2 launch perception_2d_to_pcd_wrist wrist_pointcloud.launch.py
```

**Subscribe**: wrist_projection_node와 동일 (RGB + Depth + 두 CameraInfo + /detections + TF)

**Publish**
| 토픽 | 타입 | 설명 |
|------|------|------|
| **`/perception/wrist/mask_cloud`** | `sensor_msgs/msg/PointCloud2` | 탐지된 모든 mask 합본 XYZRGB cloud (base_link) |

> ⚠️ 이전 문서의 `/camera_right/points_base`는 잘못된 이름. Manipulation GPD 입력 토픽 확인 필요.

---

### 8. `wrist_grasp_pcd_node` — B.6 wrist 객체별 grasp용 PCD

**패키지**: `perception_2d_to_pcd_wrist`

**실행**
```bash
ros2 run perception_2d_to_pcd_wrist wrist_grasp_pcd_node \
  --ros-args --params-file /ws/src/perception_2d_to_pcd_wrist/config/params.yaml
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

**패키지**: `perception_2d_to_pcd_wrist`

**용도**: `/monitor_ocr/result`의 task list를 기준으로 현재 필요한 부품만 필터링 + wrist detection 후보를 점수화 + 가장 집기 좋은 후보 1개의 중심 좌표를 publish.

> **System 팀**: `mission_a.py` A2_SCAN/A3_PICK이 실질적으로 구독해야 할 토픽은 **이 노드의 출력**.

**실행**
```bash
ros2 run perception_2d_to_pcd_wrist wrist_task_grasp_planner_node \
  --ros-args --params-file /ws/src/perception_2d_to_pcd_wrist/config/params.yaml
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
| 1 | `monitor_ocr_node` | ocr_venv | `ros2 run monitor_ocr monitor_ocr_node --ros-args -p parts_mode:=true` |
| 2 | `detector_node` (head/wrist) | yolo_venv | `ros2 launch perception_part_detector detector.launch.py camera_name:=wrist_right` |
| 3a | `wrist_projection_node` | (system) | `ros2 launch perception_2d_to_pcd_wrist wrist_projection.launch.py` |
| 3b | `wrist_pointcloud_node` | (system) | `ros2 launch perception_2d_to_pcd_wrist wrist_pointcloud.launch.py` |
| 3c | `wrist_grasp_pcd_node` | (system) | `ros2 launch perception_2d_to_pcd_wrist wrist_grasp_pcd.launch.py` |
| 3d | `wrist_task_grasp_planner_node` | (system) | `ros2 run perception_2d_to_pcd_wrist wrist_task_grasp_planner_node --ros-args --params-file /ws/src/perception_2d_to_pcd_wrist/config/params.yaml` |

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
ros2 launch perception_part_detector detector.launch.py camera_name:=head
ros2 launch perception_2d_to_pcd projection.launch.py
# (옵션) ros2 run perception_2d_to_pcd pointcloud_node
# (옵션) ros2 run perception_2d_to_pcd grasp_pcd_node
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
| `mission_a.py` 구독 토픽 정정 (`/target_pose` → `/perception/wrist/target_one_pose`) | System | 5.31 | ⬜ |
| Manipulation GPD 입력 토픽 정정 (`/camera_right/points_base` → `/perception/wrist/mask_cloud` 또는 `/perception/wrist/target_pcd/<class>`) | Manipulation | 5.31 | ⬜ |
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
find ~/AI_Worker_HC/robotis_applications/perception -name "package.xml" 2>/dev/null

# 토픽 발행 확인 (로봇 연결 시)
ros2 topic echo --once /monitor_ocr/result
ros2 topic echo --once /detections
ros2 topic echo --once /perception/wrist/target_pose
ros2 topic echo --once /perception/wrist/target_one_pose
ros2 topic hz /perception/wrist/mask_cloud

# TF 연결
ros2 run tf2_ros tf2_echo base_link camera_right_color_optical_frame
```
