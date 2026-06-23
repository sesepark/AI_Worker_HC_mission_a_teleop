# Perception Interface

현재 `humanoid_challenge/perception`에는 Mission A 핵심 흐름에 필요한 실행 노드만 남긴다.

```text
monitor_ocr_node
  -> /monitor_ocr/result
  -> tray_manage_node
  -> /perception/task_list
  -> wrist_task_grasp_planner_node
  -> /perception/wrist/target_one_pose

detector_node
  -> /detections
  -> wrist_task_grasp_planner_node
```

디버그용으로 `monitor_ocr_viewer`만 추가 유지한다.

## Nodes

| 실행명 | Launch | 역할 |
| --- | --- | --- |
| `monitor_ocr_node` | `ros2 launch perception monitor_ocr.launch.py` | 모니터 OCR로 목표 부품 수량 추출 |
| `monitor_ocr_viewer` | `ros2 run perception monitor_ocr_viewer` | OCR 디버그 overlay viewer |
| `detector_node` | `ros2 launch perception part_detector.launch.py` | YOLO 부품 detection |
| `tray_manage_node` | `ros2 launch perception task_management.launch.py` | OCR task list 변환 + tray ROI 발행 |
| `wrist_task_grasp_planner_node` | `ros2 launch perception wrist_task_grasp_planner.launch.py` | task 대상 중 최종 grasp pose 1개 선택 |

`wrist_all.launch.py`는 `wrist_task_grasp_planner_node`만 띄우는 alias다.
`manipulation_mock.launch.py`는 manipulation팀 target 테스트용 bundle이며
`detector_node`, `tray_manage_node(mock_monitor_ocr:=true, enable_tray_detection:=false)`,
`wrist_task_grasp_planner_node(weight_arm_proximity:=0.0, temporal_smoothing_enable:=false)`를
같이 실행한다.

## Topic Contract

### `/monitor_ocr/result`

Publisher: `monitor_ocr_node`

Type: `std_msgs/String`

JSON:

```json
{
  "frames_used": 10,
  "parts": [
    {"name": "플랜지 너트", "count": 1},
    {"name": "기어 링", "count": 2},
    {"name": "스페이서 링", "count": 1},
    {"name": "육각 너트", "count": 4},
    {"name": "돔 너트", "count": 2}
  ],
  "latest_elapsed_ms": 1234.5,
  "latest_screen_detected": true
}
```

### `/detections`

Publisher: `detector_node`

Type: `perception/msg/PartDetectionArray`

### `/perception/task_list`

Publisher: `tray_manage_node`

Type: `mission_interfaces/srv/GetTaskList_Response`

Fields:

```text
bool success
string message
bool screen_detected
bool all_counts_recognized
uint16 frames_used
mission_interfaces/TaskItem[] parts
```

Mapping:

| OCR/task payload | `GetTaskList_Response` |
| --- | --- |
| `mission_complete` | `success` |
| `source` | `message` JSON string |
| `ocr_latest_screen_detected` | `screen_detected`, `all_counts_recognized` |
| `ocr_frames_used` | `frames_used` |
| `parts` | `parts` |

### `/perception/tray_roi`

Publisher: `tray_manage_node`

Type: `sensor_msgs/RegionOfInterest`

Latest blue-tray ROI. If no stable tray is detected, width/height may be zero.

### `/perception/wrist/target_one_pose`

Publisher: `wrist_task_grasp_planner_node`

Type: `geometry_msgs/PoseStamped`

Frame: `base_link`

### `/perception/wrist/target_one_detection`

Publisher: `wrist_task_grasp_planner_node`

Type: `std_msgs/String`

Selected detection/debug JSON paired with `target_one_pose`.

## Service Contract

### `/perception/get_task_list`

Server: `tray_manage_node`

Type: `mission_interfaces/srv/GetTaskList`

Returns the latest task list using the same response contract as `/perception/task_list`.

### `/mission_a/task_list`

Server: `monitor_ocr_node`

Type: `mission_interfaces/srv/GetTaskList`

Legacy OCR service fallback. The topic pipeline should normally use `/monitor_ocr/result` -> `tray_manage_node`.

## Removed Legacy Nodes

The following executable nodes and launch files were intentionally removed from `humanoid_challenge/perception`:

| Removed | Replacement |
| --- | --- |
| `monitor_ocr_topic_node` | `monitor_ocr_node` now publishes `/monitor_ocr/result` directly |
| head projection/pointcloud/grasp PCD nodes | not part of current Mission A perception runtime |
| wrist projection/pointcloud/grasp PCD nodes | `wrist_task_grasp_planner_node` performs the required final target selection |
| legacy tray occupancy/management nodes | `tray_manage_node` |
| perception tools scripts | out of runtime scope |
