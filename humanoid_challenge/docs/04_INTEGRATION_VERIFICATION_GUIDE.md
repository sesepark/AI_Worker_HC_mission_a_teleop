# Integration Verification Guide

현재 perception runtime은 Mission A에 필요한 5개 실행 노드만 유지한다.

| 실행 노드 | 역할 |
| --- | --- |
| `monitor_ocr_node` | OCR 결과 topic 발행 + legacy task service |
| `monitor_ocr_viewer` | OCR 디버그 viewer |
| `detector_node` | 부품 YOLO detection |
| `tray_manage_node` | OCR task list 변환 + tray ROI |
| `wrist_task_grasp_planner_node` | 최종 grasp pose 1개 선택 |

## Build

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mission_interfaces perception mission
source install/setup.bash
```

## Executables

```bash
ros2 pkg executables perception
```

확인 대상:

```text
perception detector_node
perception monitor_ocr_node
perception monitor_ocr_viewer
perception tray_manage_node
perception wrist_task_grasp_planner_node
```

## Minimal Topic Pipeline

Terminal A:

```bash
ros2 launch perception monitor_ocr.launch.py
```

Terminal B:

```bash
ros2 launch perception part_detector.launch.py camera_name:=wrist_right
```

Terminal C:

```bash
ros2 launch perception task_management.launch.py
```

Terminal D:

```bash
ros2 launch perception wrist_task_grasp_planner.launch.py
```

확인:

```bash
ros2 topic info /monitor_ocr/result
ros2 topic info /detections
ros2 topic info /perception/task_list
ros2 topic info /perception/tray_roi
ros2 topic info /perception/wrist/target_one_pose
```

## Robot-Free Smoke Test

OCR 없이 task list를 직접 주입해 `mission_a`와 wrist planner의 typed topic 연결을 확인할 수 있다.

```bash
ros2 topic pub /perception/task_list mission_interfaces/srv/GetTaskList_Response \
"{success: false, message: '{\"ocr_topic\":\"manual\"}', screen_detected: true, all_counts_recognized: true, frames_used: 1, parts: [{name: 'flange nut', count: 1}, {name: 'gear ring', count: 2}]}" -r 1
```

`tray_manage_node` service 확인:

```bash
ros2 service call /perception/get_task_list mission_interfaces/srv/GetTaskList \
"{timeout_sec: 30.0, frame_count: 1}"
```

## Conflict Checks

```bash
git status --short
git diff --check
rg -n "^(<<<<<<<|=======|>>>>>>>)" .
```
