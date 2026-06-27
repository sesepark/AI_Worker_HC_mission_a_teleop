# Mission B System Integration

This document is the navigation-side contract for Mission B.

The system team only needs to run one navigation coordinator node and publish
String actions. Navigation publishes String events back to the system.

## Package

ROS 2 package:

```text
ffw_mission_b_nav
```

Main executable:

```text
sg2_mission_b_system_nav
```

Recommended launch file:

```text
mission_b_system_nav.launch.py
```

Recommended real robot parameter file:

```text
config/mission_b_real_robot.yaml
```

The safe-A coordinator keeps the same system action topic and navigation event
topic as the standard coordinator. It changes only the B-to-A return behavior:
near A, it detects three conveyor legs from `/scan`, aligns to their center, and
publishes debug markers on `/mission_b/debug/conveyor_legs`.

## Runtime Assumption

Robot PC runs hardware bringup and navigation SLAM/Nav2.

User PC or Docker runs this package.

All machines must use:

```bash
export ROS_DOMAIN_ID=30
```

Real robot nodes must use:

```text
use_sim_time: false
```

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ffw_mission_b_nav
source install/setup.bash
```

## Run

```bash
ros2 launch ffw_mission_b_nav mission_b_system_nav.launch.py
```

Equivalent direct command:

```bash
ros2 run ffw_mission_b_nav sg2_mission_b_system_nav --ros-args \
  --params-file $(ros2 pkg prefix ffw_mission_b_nav)/share/ffw_mission_b_nav/config/mission_b_real_robot.yaml
```

## System To Navigation

Topic:

```text
/mission_b/system/action
```

Type:

```text
std_msgs/msg/String
```

Accepted values:

| Action | Meaning |
|---|---|
| `A_TO_B` | Move from current A pose to the B stop line and run LiDAR alignment |
| `APPROACH_B` | Move a short fixed distance forward from the B stop line and save B pose |
| `B_TO_A` | Return from B to A using reverse segment motion |
| `STOP` | Stop the active navigation route subprocess |

Supported aliases:

| Alias | Normalized action |
|---|---|
| `ACTION_A_TO_B`, `START_A_TO_B`, `MOVE_A_TO_B` | `A_TO_B` |
| `MOVE_FORWARD`, `FORWARD_TO_TABLE`, `FINAL_APPROACH_B` | `APPROACH_B` |
| `ACTION_B_TO_A`, `RETURN_TO_A`, `MOVE_B_TO_A` | `B_TO_A` |

## Navigation To System

Topic:

```text
/mission_b/nav/event
```

Type:

```text
std_msgs/msg/String
```

Events:

| Event | Meaning |
|---|---|
| `READY` | Coordinator is running and waiting for a system action |
| `A_TO_B_ACCEPTED` | `A_TO_B` was accepted and route subprocess started |
| `REACHED_B_STOP_LINE` | Robot reached the B stop line and LiDAR alignment finished |
| `APPROACH_B_ACCEPTED` | `APPROACH_B` was accepted |
| `REACHED_B_PLACE_POSE` | Robot moved closer to the table, saved B pose, and updated marker |
| `B_TO_A_ACCEPTED` | `B_TO_A` was accepted |
| `REACHED_A` | Robot returned to A and A-side alignment finished |
| `REJECTED:<action>:<reason>` | Action was rejected because the coordinator was in the wrong state |
| `FAILED:<reason>` | Navigation failed |
| `STOPPED` | `STOP` action was accepted |

State topic:

```text
/mission_b/system_nav/state
```

Type:

```text
std_msgs/msg/String
```

The value is formatted as:

```text
<system_state>;nav=<route_state>;failure=<failure_reason>
```

## Mission Sequence

```text
System publishes A_TO_B
-> Navigation stores current pose as A
-> Navigation moves backward, right, forward
-> Navigation runs LiDAR alignment at the B stop line
-> Navigation publishes REACHED_B_STOP_LINE

System publishes APPROACH_B
-> Navigation moves b_approach_forward_distance forward
-> Navigation stores current pose as B
-> Navigation publishes REACHED_B_PLACE_POSE

Manipulator places the box

System publishes B_TO_A
-> Navigation returns by reverse segments
-> Navigation runs A-side LiDAR alignment
-> Navigation stores current pose as A
-> Navigation publishes REACHED_A
```

## Tested Real Robot Parameters

These safe-A values were tested on the real robot:

```yaml
backward_distance: 0.70
right_distance: 3.80
forward_distance: 0.30
lateral_speed: 0.20
forward_speed: 0.12
desired_front_distance: 0.70
b_approach_forward_distance: 0.06
b_approach_speed: 0.12
return_mode: reverse_segments
enable_return_final_trim: true
enable_return_a_lidar_alignment: true
return_a_desired_front_distance: 0.30
return_a_align_timeout_sec: 30.0
return_a_alignment_mode: legs
return_a_leg_min_spacing: 1.00
return_a_roi_y_abs: 2.00
cmd_vel_topic: /cmd_vel
```

Parameter meaning:

| Parameter | Meaning |
|---|---|
| `desired_front_distance` | B stop-line LiDAR alignment target distance |
| `b_approach_forward_distance` | Extra fixed forward motion after `APPROACH_B` |
| `return_a_alignment_mode` | A-side alignment mode. `legs` detects three conveyor legs from `/scan` |
| `return_a_desired_front_distance` | Final front distance from the three-leg conveyor center |
| `return_a_leg_min_spacing` | Minimum spacing between selected conveyor leg candidates |
| `return_a_roi_y_abs` | A-side LiDAR ROI half-width for detecting conveyor legs |

## Manual Test Commands

Watch events:

```bash
ros2 topic echo /mission_b/nav/event
```

Watch state:

```bash
ros2 topic echo /mission_b/system_nav/state
```

Send actions:

```bash
ros2 topic pub --once /mission_b/system/action std_msgs/msg/String "{data: A_TO_B}"
ros2 topic pub --once /mission_b/system/action std_msgs/msg/String "{data: APPROACH_B}"
ros2 topic pub --once /mission_b/system/action std_msgs/msg/String "{data: B_TO_A}"
```

Stop:

```bash
ros2 topic pub --once /mission_b/system/action std_msgs/msg/String "{data: STOP}"
```

## Marker Topics

RViz markers:

```text
/mission_b/primitive/waypoints
```

Pose topics:

```text
/mission_b/primitive/a_pose
/mission_b/primitive/b_pose
```

## Notes For System Team

Do not publish `APPROACH_B` before `REACHED_B_STOP_LINE`.

Do not publish `B_TO_A` before `REACHED_B_PLACE_POSE` and manipulation place
completion.

`A_TO_B` captures the current odom pose as A every cycle.

`APPROACH_B` captures the final place pose as B every cycle.

The return motion is `reverse_segments`, not diagonal shortest-path motion.
