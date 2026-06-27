# 자율 → 원격 텔레오퍼레이션 전환 절차

Mission A 자율 운용이 끝난 뒤, 카메라/bringup/MoveIt을 재시작하지 않고 원격 운용 feedback과
Leader/operator teleop만 붙이는 절차입니다.

## 목표 구조

| 위치 | 유지/추가 |
|---|---|
| robot PC | 기존 follower bringup, camera, lidar, MoveIt 유지 |
| robot PC | 자율 종료 후 `ffw_sg2_robot_teleop_attach.launch.py`만 추가 실행 |
| main PC | 자율 mission 종료 후 `ffw_sg2_operator_leader.launch.py` 실행 |
| 공통 | 카메라 재시작 없음, 자율팀 launch 명령어 변경 없음 |

전환 시 `/cmd_vel` publisher가 자율 노드에서 `teleop_cmd_vel_mux`로 바뀌었는지 반드시 확인합니다.

## 최초 1회 설치/빌드

기존 `~/ros2_ws`를 그대로 쓰는 방식입니다. `~/ros2_ws/src` 안에 원본 `ai_worker`와
별도 teleop 레포가 동시에 있으면 package name이 중복되므로 하나만 둡니다.

robot PC:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ffw_teleop
source install/setup.bash
```

main PC:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  ffw_teleop ffw_bringup ffw_joystick_controller
source install/setup.bash
```

빌드 후 확인:

```bash
ros2 pkg prefix ffw_teleop
ros2 pkg prefix ffw_joystick_controller
```

출력이 방금 빌드한 `~/ros2_ws/install/...`을 가리켜야 합니다. Docker 안에서는 보통
workspace가 `/root/ros2_ws`입니다.

## 자율 운용 중 유지할 것

robot PC에서 아래 launch는 끄지 않습니다.

```bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py ...
ros2 launch ffw_moveit_config moveit.launch.py
```

유지 대상:

```text
follower bringup
ZED
right wrist RealSense
lidar
/joint_states
/robot_description
/scan
/odom
MoveIt
controller manager
```

## 자율 종료 시 끄는 것

자율 미션 완료 후 아래 launch를 종료합니다.

```text
Ctrl-C: mission_a_real.launch.py
Ctrl-C: mission_a_manip.launch.py        # 별도 실행한 경우
Ctrl-C: move_base_lateral.launch.py      # nav_mode:=service를 쓴 경우
```

끄지 않는 것:

```text
robot PC bringup
camera
lidar
MoveIt
robot_state_publisher/controller core runtime
```

Ctrl-C 후 zero twist를 한 번 보냅니다.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 원격 시작 전 확인

main PC 또는 robot PC에서 확인합니다.

```bash
ros2 topic info /cmd_vel -v
ros2 node list | grep -E "mission_a|mission_a_manipulation_server|move_base_lateral|mock|perception"
ros2 topic hz /zed/zed_node/left/image_rect_color
ros2 topic hz /camera_right/camera_right/color/image_rect_raw
ros2 topic hz /camera_right/camera_right/depth/image_rect_raw
```

원칙:

```text
/cmd_vel에 자율 publisher가 남아 있으면 원격 시작 금지
mission_a_real / mission_a_manipulation_server / move_base_lateral이 남아 있으면 먼저 종료
ZED는 29Hz 이상 목표
wrist color/depth 토픽이 살아 있어야 함
```

## 원격 시작

robot PC 새 터미널:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch ffw_teleop ffw_sg2_robot_teleop_attach.launch.py
```

이 launch는 follower, ZED, RealSense를 새로 켜지 않습니다. 기존 raw topic에 붙어서
원격용 compressed/status topic만 만듭니다.

main PC 새 터미널:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch ffw_teleop ffw_sg2_operator_leader.launch.py \
  leader_controller_config:=ffw_lg2_leader_ai_hardware_controller_teleop.yaml \
  head_mux_output_topic:=/leader/joystick_controller_left/joint_trajectory \
  start_cmd_vel_mux:=true \
  start_head_trajectory_mux:=true
```

## 원격 성공 확인

```bash
ros2 topic info /cmd_vel -v
ros2 topic hz /teleop/zed/depth_assist/compressed
ros2 topic hz /teleop/wrist_right/depth_assist/compressed
ros2 topic hz /camera_right/camera_right/color/image_rect_raw
```

성공 기준:

```text
/cmd_vel publisher는 teleop_cmd_vel_mux 하나
/teleop/zed/depth_assist/compressed 28Hz 이상
/teleop/wrist_right/depth_assist/compressed 28Hz 이상
OpenCV viewer에 STATUS / BANDWIDTH / ZED / R WRIST / R COLOR 표시
R COLOR 90도 회전
drive control 창에서 base drive, ZED/head control 동작
RViz에서 robot model과 /scan 표시
```

ZED 입력은 자율 운용과 충돌을 피하기 위해 기존 VGA급 raw topic을 그대로 사용합니다. 실사용 가능한
화질인지는 실제 operator viewer에서 확인해야 합니다.

## Docker 이름

Docker 이름은 실행 혼동 방지용입니다.

```text
ai_worker              -> ai_worker_teleop_final
humanoid_challenge     -> humanoid_challenge_teleop_final
```

`./container.sh start`, `./container.sh enter`를 쓰면 Docker 이름을 직접 몰라도 됩니다.
직접 들어가야 하면:

```bash
docker exec -it ai_worker_teleop_final bash
docker exec -it humanoid_challenge_teleop_final bash
```
