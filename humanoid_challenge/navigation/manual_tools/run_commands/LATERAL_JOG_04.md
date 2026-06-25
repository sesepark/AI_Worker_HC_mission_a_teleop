# SG2 0.4m Lateral Jog

SG2를 왼쪽 또는 오른쪽으로 `0.4 m` 횡이동시키는 실행 명령입니다. `ffw_manual_tools`의 `sg2_lateral_jog` 노드를 사용하며, `/odom`을 기준으로 목표 거리까지 이동한 뒤 정지합니다.

## 준비

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ffw_manual_tools
source install/setup.bash
export ROS_DOMAIN_ID=30
```

실행 전에는 Mission manager, Nav2 goal, teleop처럼 `/cmd_vel`을 publish하는 다른 노드를 멈춥니다.

## 왼쪽으로 0.4m 이동

```bash
ros2 run ffw_manual_tools sg2_lateral_jog --ros-args \
  -p direction:=left \
  -p distance:=0.4 \
  -p speed:=0.12 \
  -p cmd_vel_topic:=/cmd_vel
```

## 오른쪽으로 0.4m 이동

```bash
ros2 run ffw_manual_tools sg2_lateral_jog --ros-args \
  -p direction:=right \
  -p distance:=0.4 \
  -p speed:=0.12 \
  -p cmd_vel_topic:=/cmd_vel
```

## 강제 정지

움직임을 즉시 멈춰야 하면 아래 명령을 보냅니다.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 로그 확인

정상 종료 시 아래처럼 `/odom` 기준 실제 횡이동량이 출력됩니다.

```text
Lateral jog done: reason=odom_distance_reached, odom_left_delta=...
```

`odom_left_delta`가 양수면 왼쪽, 음수면 오른쪽 이동입니다.
