# SG2 Mobile Movement Commands

SG2를 키보드로 전후진, 좌우 횡이동, 회전시키는 `ffw_manual_tools`의 `sg2_mobile_teleop` 실행 명령입니다.

## 준비

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ffw_manual_tools
source install/setup.bash
export ROS_DOMAIN_ID=30
```

## 실행

기본 실행:

```bash
ros2 run ffw_manual_tools sg2_mobile_teleop --ros-args \
  -p cmd_vel_topic:=/cmd_vel \
  -p linear_speed:=0.10 \
  -p lateral_speed:=0.10 \
  -p angular_speed:=0.25
```

실제 로봇에서 `0.10 m/s`가 약하면 아래처럼 `0.12 m/s`로 실행합니다.

```bash
ros2 run ffw_manual_tools sg2_mobile_teleop --ros-args \
  -p cmd_vel_topic:=/cmd_vel \
  -p linear_speed:=0.12 \
  -p lateral_speed:=0.12 \
  -p angular_speed:=0.25
```

## 키 조작

```text
W/S : 전진 / 후진
A/D : 왼쪽 횡이동 / 오른쪽 횡이동
Q/E : 왼쪽 회전 / 오른쪽 회전
+/- : 속도 증가 / 감소
Space : 정지
H : 도움말
Ctrl+C : 종료
```

## 주의

- Mission manager, Nav2 goal, 다른 teleop처럼 `/cmd_vel`을 publish하는 노드와 동시에 쓰지 않습니다.
- 실제 로봇에서는 사람이 바로 정지할 수 있는 상태에서 낮은 속도로 먼저 확인합니다.
- 급할 때는 `Space`를 누르거나 `Ctrl+C`로 종료합니다.
