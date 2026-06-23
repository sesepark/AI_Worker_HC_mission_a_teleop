#!/usr/bin/env python3
"""Mission A Phase 2 실 통합 launch (humanoid_challenge 측) — 실 perception + mission_a + nav=stub.

Phase 1 `integration_demo.launch.py`(mock 위주)의 Phase 2 대응본. 한 launch 그룹(동일 기동 윈도우,
컨테이너 DDS 디스커버리 안정 — G5 검증 패턴)으로:
  · 실 perception (`perception_live.launch.py`: detector+tray_manage+wrist_planner+static TF+place_pose_valid)
  · mission_a (FSM, use_mocks=false, nav_mode=stub, task_list=topic 경로)

**실 manipulation 서버는 ai_worker 컨테이너에서 별도 기동**(동일 DDS 도메인):
  ai_worker$  ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py     # bringup
  ai_worker$  ros2 launch ffw_moveit_config moveit.launch.py            # MoveIt
  ai_worker$  ros2 launch manipulation mission_a_manip.launch.py        # 실 manip 서버(T1)
  (공통 env: ROS_DOMAIN_ID=30, ROS_LOCALHOST_ONLY=0, ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET — CONTEXT §7.1)

nav 은 전 구간 stub(nav_mode=stub). move_base_lateral 실연동은 범위 밖(TODO).
mock 전용 회귀 검증은 기존 `mission_a.launch.py use_mocks:=true` / `integration_demo.launch.py` 사용.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    mission_pkg = get_package_share_directory('mission')
    perception_pkg = get_package_share_directory('perception')
    mission_launch = os.path.join(mission_pkg, 'launch', 'mission_a.launch.py')
    perception_live = os.path.join(perception_pkg, 'launch', 'perception_live.launch.py')

    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('use_place_pose_check', default_value='false',
                              description='C3 게이트(실 place_pose_valid 준비 후 true)'),
        DeclareLaunchArgument('mock_monitor_ocr', default_value='true',
                              description='실 perception: 모니터 OCR mock(카메라 없이 task_list)'),
    ]

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_live),
        launch_arguments={'mock_monitor_ocr': lc('mock_monitor_ocr')}.items(),
    )
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mission_launch),
        launch_arguments={
            'use_mocks': 'false',           # 실 manip(ai_worker)·실 perception 사용 → mock 미기동
            'nav_mode': 'stub',
            'use_place_pose_check': lc('use_place_pose_check'),
        }.items(),
    )
    return LaunchDescription(args + [perception, mission])
