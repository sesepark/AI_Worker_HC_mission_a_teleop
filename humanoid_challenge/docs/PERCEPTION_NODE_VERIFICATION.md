# Perception Node Verification

이 문서는 현재 남아 있는 perception 실행 노드만 대상으로 한다.

## monitor_ocr_node

```bash
ros2 launch perception monitor_ocr.launch.py
```

확인:

```bash
ros2 topic echo --once /monitor_ocr/result
ros2 topic echo --once /monitor_ocr/recognized
ros2 service call /mission_a/task_list mission_interfaces/srv/GetTaskList \
"{timeout_sec: 20.0, frame_count: 3}"
```

Viewer:

```bash
ros2 run perception monitor_ocr_viewer
```

## detector_node

```bash
ros2 launch perception part_detector.launch.py camera_name:=wrist_right
```

확인:

```bash
ros2 topic echo --once /detections
ros2 topic echo --once /detector_debug_image
```

## tray_manage_node

```bash
ros2 launch perception task_management.launch.py
```

OCR 없이 빠르게 확인하려면:

```bash
ros2 launch perception task_management.launch.py mock_monitor_ocr:=true
```

확인:

```bash
ros2 topic echo --once /perception/task_list
ros2 topic echo --once /perception/tray_roi
ros2 service call /perception/get_task_list mission_interfaces/srv/GetTaskList \
"{timeout_sec: 30.0, frame_count: 1}"
```

## wrist_task_grasp_planner_node

```bash
ros2 launch perception wrist_task_grasp_planner.launch.py
```

Alias:

```bash
ros2 launch perception wrist_all.launch.py
```

Fake task list:

```bash
ros2 topic pub /perception/task_list mission_interfaces/srv/GetTaskList_Response \
"{success: false, message: '{\"ocr_topic\":\"manual\"}', screen_detected: true, all_counts_recognized: true, frames_used: 1, parts: [{name: 'gear ring', count: 2}]}" -r 1
```

확인:

```bash
ros2 topic echo /perception/wrist/target_one_pose
ros2 topic echo /perception/wrist/target_one_detection
```
