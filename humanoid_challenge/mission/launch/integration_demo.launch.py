#!/usr/bin/env python3
"""Mission A 통합 시연 launch (G5) — 실 perception task_list + mock manip/nav(stub).

한 launch 그룹에서 다음을 함께 기동(동일 기동 윈도우 → 같은 컨테이너 DDS 에서 디스커버리 안정):
  · 실 perception `tray_manage_node` (mock OCR, tray detection off) → /perception/task_list
    (mission_interfaces/GetTaskList.Response) + /perception/get_task_list 서비스.
  · mission_a(nav_mode=stub) + mock_manipulation_a + mock_navigation_a +
    mock_perception_a(pub_task_list:=false → wrist target/place_pose_valid 만, task_list 은 실노드).

→ FSM 이 **실 perception 의 task_list** 를 받아 A1_MONITOR→A2_SCAN_POSE→A2_SCAN→A3_PICK(mock)
   →A3_MOVE_TO_TRAY(stub)→A3_PLACE(mock)→… 흐르는지 확인.

manipulation 실노드는 FSM 계약(scan action/attach 등) 미제공이라 mock 대체. nav=stub(범위 밖 service 우회).
wrist target 은 카메라/detector 필요 → mock 대체(실검출은 로봇/캘리브 후).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    perception_launch = os.path.join(
        get_package_share_directory('perception'), 'launch', 'task_management.launch.py')
    mission_launch = os.path.join(
        get_package_share_directory('mission'), 'launch', 'mission_a.launch.py')

    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('nav_mode', default_value='stub'),
        # T3: 서비스 경로 검증용 패스스루(기본 topic 경로 무영향).
        DeclareLaunchArgument('use_task_list_service', default_value='false'),
        DeclareLaunchArgument('task_list_service_name', default_value='/mission_a/task_list'),
        DeclareLaunchArgument('task_list_topic', default_value='/perception/task_list'),
        # 실 perception task_list (mock OCR, tray detection off — 카메라/모델 불필요)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={
                'mock_monitor_ocr': 'true',
                'enable_tray_detection': 'false',
                'require_complete_ocr': 'false',
            }.items(),
        ),
        # mission_a + mocks (task_list 은 실노드 사용 → mock_pub_task_list:=false)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            launch_arguments={
                'nav_mode': lc('nav_mode'),
                'mock_pub_task_list': 'false',
                'use_task_list_service': lc('use_task_list_service'),
                'task_list_service_name': lc('task_list_service_name'),
                'task_list_topic': lc('task_list_topic'),
            }.items(),
        ),
    ])
