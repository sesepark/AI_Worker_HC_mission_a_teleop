# Perception 노드 실행·검증 런북 (공용 레포)

공용 레포 `AI_Worker_HC`에서 perception 노드들이 정상 작동하는지 확인하는 절차.
컨테이너 진입 → 빌드 → 노드별 실행/검증 순서로 정리한다.

> 모델 가중치 3개는 `perception/model/`에 배치되어 있어야 한다
> (`part_detector_best.pt`, `monitor_ocr_best.pt`, `tray_occupancy_best.pt`).

---

## STEP 0 — 컨테이너 start / enter

```bash
cd /home/jihun/AI_Worker_HC/humanoid_challenge/docker
./container.sh start    # 이미지 pull(최초 1회) + compose up -d
./container.sh enter     # /ws 로 진입, ROS + install 자동 source
```

- 컨테이너 안 소스 위치: `/ws/src/humanoid_challenge`
- 작업 루트: `/ws` (`COLCON_WS=/ws`)
- `ROS_DOMAIN_ID=30`, host 네트워크 → 로봇 카메라 토픽 그대로 수신
- 내장 venv: `/ws/yolo_venv` (detector, tray), `/ws/ocr_venv` (monitor_ocr)

## STEP 1 — 빌드 (컨테이너 내부)

`perception`은 `mission_interfaces`(서비스 타입)에 의존하므로 의존성까지 함께 빌드한다.

```bash
cd /ws
colcon build --symlink-install --packages-up-to perception
source install/setup.bash
```

코드 변경 후 클린 빌드가 필요하면:

```bash
cd /ws
rm -rf build/perception install/perception
colcon build --symlink-install --packages-up-to perception
source install/setup.bash
```

## STEP 2 — 로봇 토픽 수신 확인

```bash
ros2 topic list | grep -E "zed|camera_right|camera_left"
```

카메라 토픽이 안 보이면 로봇/ZED 노드가 안 켜진 것 → 노드 검증 전에 먼저 해결.

> ⚠️ 도메인 ID 주의: `humanoid_challenge` 컨테이너는 `ROS_DOMAIN_ID=30`이지만
> `ai_worker` 컨테이너(카메라/로봇 토픽 발행원)는 기본값(0)일 수 있다.
> 토픽이 안 보이면 두 컨테이너의 `ROS_DOMAIN_ID`를 30으로 맞춘다.

---

## STEP 2.5 — 로봇 PC bringup + TF 브리지 (3D 노드 전제조건)

카메라/TF는 **별도 robot PC**의 bringup에서 나온다. 2D 노드는 기본 bringup만으로 충분하지만,
3D 노드(head projection/pcd, wrist grasp planner)는 추가로 **ZED depth**와 **wrist 카메라 TF**가 필요하다.

### (A) 로봇 PC — 카메라 + depth + TF 통합 bringup

robot PC의 ai_worker 컨테이너에서 실행 (벤더 config 미수정, override yaml로 depth만 켬).
번들 bringup은 ZED를 `depth_mode: NONE`으로 띄우므로, 카메라를 분리해 override로 다시 올린다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash     # 로봇 워크스페이스 경로에 맞게
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0

cat > /tmp/zed_depth_override.yaml <<'YAML'
/**:
    ros__parameters:
        depth:
            depth_mode: 'PERFORMANCE'
YAML

cat > /tmp/perception_robot_bringup.launch.py <<'PY'
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    d = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')
    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(d, 'ffw_sg2_follower_ai.launch.py')),
        launch_arguments={'launch_cameras': 'false'}.items())
    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(d, 'camera_zed.launch.py')),
        launch_arguments={'camera_model': 'zedm',
                          'ros_params_override_path': '/tmp/zed_depth_override.yaml'}.items())
    rs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(d, 'camera_realsense.launch.py')),
        launch_arguments={'tf_publish_rate1': '10.0', 'tf_publish_rate2': '10.0',
                          'colorizer.enable1': 'false', 'colorizer.enable2': 'false'}.items())
    return LaunchDescription([
        follower,
        TimerAction(period=8.0,  actions=[zed]),
        TimerAction(period=12.0, actions=[rs]),
    ])
PY

ros2 launch /tmp/perception_robot_bringup.launch.py
```

- `depth_mode`: **`PERFORMANCE` 권장**. `NEURAL`은 GPU/인터넷(첫 모델 최적화) 의존이라 환경에 따라 crash-loop → ZED가 rgb조차 발행 못 함.
- 기체가 sg2가 아니면 `ffw_sg2_follower_ai.launch.py`만 해당 모델로 교체.
- 이 터미널은 **포그라운드 유지** (닫으면 전부 내려감). 에러로 프롬프트 복귀하면 실패 — 출력 확인.
- `ffw_bringup` 미발견(`PackageNotFoundError`)이면 소싱 누락 → 위 `source` 4줄 먼저 실행.

### (B) perception 로컬 컨테이너 — wrist TF 브리지

robot URDF는 wrist D405를 `camera_r_*`로, RealSense 드라이버는 `camera_right_*`로 발행해 트리가 끊긴다.
동일 물리 카메라이므로 identity static transform으로 잇는다 (perception 컨테이너에서 실행).

```bash
ros2 run tf2_ros static_transform_publisher --frame-id camera_r_link --child-frame-id camera_right_link &
ros2 run tf2_ros static_transform_publisher --frame-id camera_l_link --child-frame-id camera_left_link &
```

확인:

```bash
ros2 run tf2_ros tf2_echo base_link camera_right_color_optical_frame   # Translation 나오면 OK
ros2 topic info /zed/zed_node/depth/depth_registered | grep "Publisher count"  # 1 이면 depth ON
```

---

## STEP 3 — 노드별 실행 / 검증

### ① detector_node (yolo_venv)

```bash
# Head
ros2 launch perception part_detector.launch.py camera_name:=head \
  image_topic:=/zed/zed_node/rgb/image_rect_color

# Wrist right
ros2 launch perception part_detector.launch.py camera_name:=wrist_right \
  image_topic:=/camera_right/camera_right/color/image_rect_raw
```

확인:

```bash
ros2 topic echo /detections --once          # PartDetectionArray 출력되면 OK
ros2 run rqt_image_view rqt_image_view /detector_debug_image
```

### ② monitor_ocr_node (ocr_venv = PaddleOCR)

```bash
ros2 launch perception monitor_ocr.launch.py \
  image_topic:=/zed/zed_node/rgb/image_rect_color
```

확인 (서비스형):

```bash
ros2 service list | grep /mission_a/task_list
```

### ③ tray_occupancy_node (yolo_venv, 서비스형)

```bash
ros2 launch perception tray_occupancy.launch.py camera_name:=head \
  image_topic:=/zed/zed_node/rgb/image_rect_color
```

확인:

```bash
ros2 service list | grep /mission_a/tray_detections
```

### ④ head projection / pointcloud / grasp_pcd (※ ZED depth 필요 — STEP 2.5-A)

detector(head)가 떠 있어야 `/detections`가 들어온다. depth가 켜져 있으면 RGB+depth+camera_info
동기화가 성공해 `/perception/head/*`가 발행된다.

```bash
ros2 launch perception part_detector.launch.py camera_name:=head \
  image_topic:=/zed/zed_node/rgb/image_rect_color   # (다른 터미널) 입력원
ros2 launch perception head_all.launch.py
```

확인:

```bash
ros2 topic hz /perception/head/rgb        # rate 잡히면 동기화 성공 (depth OFF면 "No synchronized depth" 경고만 뜸)
ros2 topic info /perception/head/mask_cloud | grep "Publisher count"
# /perception/head/target_pose 는 head 시야에 부품이 있을 때 출력
```

### ⑤ wrist_task_grasp_planner_node (※ wrist TF 브리지 필요 — STEP 2.5-B)

wrist detector + RealSense depth + base_link TF(브리지)가 전제. `/perception/task_list`는
typed `GetTaskList_Response` 메시지를 fake로 주입해 검증할 수 있다.

```bash
ros2 launch perception part_detector.launch.py camera_name:=wrist_right \
  image_topic:=/camera_right/camera_right/color/image_rect_raw   # (다른 터미널) 입력원
ros2 launch perception wrist_task_grasp_planner.launch.py
```

다른 터미널에서 가짜 task_list 주입:

```bash
ros2 topic pub /perception/task_list mission_interfaces/srv/GetTaskList_Response \
  "{success: false, message: '{\"ocr_topic\":\"manual\",\"timeout_sec\":30.0,\"frame_count\":1}', screen_detected: true, all_counts_recognized: true, frames_used: 1, parts: [{name: 'gear ring', count: 2}]}" -r 1

ros2 topic echo /perception/wrist/target_one_pose
```

- 정상이면 로그가 "No synchronized RGB-D" → detection 처리 단계로 넘어가고, task 주입 시
  active class가 `ALL → ['gear_ring']`로 바뀐다. 후보/포즈는 wrist 시야에 부품이 있을 때 출력.
- TF 브리지가 없으면 `target_one_pose`가 base_link 변환에서 막힌다(STEP 2.5-B 먼저).

---

## 컨테이너 종료

```bash
cd /home/jihun/AI_Worker_HC/humanoid_challenge/docker
./container.sh stop
```
