# Manual SG2 Tools

수동 테스트용 SG2 이동 도구입니다.

이 폴더는 시스템 팀 통합용 `system_team_handoff`와 분리되어 있습니다. 수동 조작이나 간단한 이동 테스트가 필요할 때만 빌드해서 사용합니다.

## 포함 노드

```text
ffw_manual_tools sg2_lateral_jog
ffw_manual_tools sg2_mobile_teleop
```

## 빌드

`ffw_manual_tools` 폴더를 ROS 2 workspace의 `src` 안에 넣고 빌드합니다.

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ffw_manual_tools
source install/setup.bash
export ROS_DOMAIN_ID=30
```

## 실행 문서

```text
manual_tools/run_commands/LATERAL_JOG_04.md
manual_tools/run_commands/SG2_MOBILE.md
```

## 주의

Mission navigation, Nav2 goal, 다른 teleop처럼 `/cmd_vel`을 publish하는 노드와 동시에 사용하지 않습니다.
