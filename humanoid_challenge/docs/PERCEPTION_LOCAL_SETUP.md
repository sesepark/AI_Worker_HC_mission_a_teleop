# Perception Local Setup

실행은 ROS 2 Jazzy 컨테이너 안의 `/ws` 기준이다.

## Build

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mission_interfaces perception mission
source install/setup.bash
```

## Required Models

| File | 기본 위치 |
| --- | --- |
| part detector | `/ws/src/AI_Worker_HC/humanoid_challenge/perception/model/part_detector_best.pt` |
| monitor OCR | `/ws/src/AI_Worker_HC/humanoid_challenge/perception/model/monitor_ocr_best.pt` |
| tray detector | `/ws/src/AI_Worker_HC/humanoid_challenge/perception/model/tray_occupancy_best.pt` |

다른 경로를 쓰려면 각 launch argument의 `model_path`, `yolo_model_path`, `tray_model_path`를 override한다.

## Run Order

Manipulation팀 mock target 테스트는 아래 한 줄로 실행한다.

```bash
ros2 launch perception manipulation_mock.launch.py
```

이 launch는 `detector_node`, `tray_manage_node(mock_monitor_ocr:=true, enable_tray_detection:=false)`,
`wrist_task_grasp_planner_node(weight_arm_proximity:=0.0, temporal_smoothing_enable:=false)`를
같이 실행한다.

전체 OCR 포함 경로를 분리해서 띄울 때는 아래 순서를 사용한다.

```bash
ros2 launch perception monitor_ocr.launch.py
ros2 launch perception part_detector.launch.py camera_name:=wrist_right
ros2 launch perception task_management.launch.py
ros2 launch perception wrist_task_grasp_planner.launch.py
```

OCR 디버그:

```bash
ros2 run perception monitor_ocr_viewer
```

## Topics

```bash
ros2 topic echo --once /monitor_ocr/result
ros2 topic echo --once /detections
ros2 topic echo --once /perception/task_list
ros2 topic echo --once /perception/tray_roi
ros2 topic echo --once /perception/wrist/target_one_pose
```

## Notes

- OCR 노드는 `/ws/ocr_venv/bin/python3` prefix로 실행한다.
- YOLO 기반 노드는 `/ws/yolo_venv/bin/python3` prefix로 실행한다.
- `wrist_all.launch.py`는 현재 `wrist_task_grasp_planner_node`만 실행하는 alias다.
