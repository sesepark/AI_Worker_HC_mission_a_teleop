#!/usr/bin/env python3
"""Mission A launch — FSM + (옵션) Mission A mock 3종.

검증 구성:
  · 단계0 무회귀(sim):  ros2 launch mission mission_a.launch.py sim_mode:=true use_mocks:=false use_task_list_service:=true
       → mission_a 단독(SimDriver 입력 주입, 신규 액션/서비스 우회).
  · 단계1 (nav=stub):   ros2 launch mission mission_a.launch.py
       → mission_a(nav_mode=stub) + mock 3종. 로봇/실서비스 없이 전 구간·게이트 검증.
  · 단계2 (nav=service): ros2 launch mission mission_a.launch.py nav_mode:=service
       → mission_a(nav_mode=service) + mock_navigation_a(Service) 실연동.

주입/토글 인자: nav_mode, use_place_pose_check, base_shift_mm,
  fail_arrive (nav), drop_during_move (manip), place_pose_invalid/flap (perception).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('sim_mode', default_value='false'),
        DeclareLaunchArgument('use_mocks', default_value='true',
                              description='mock 3종 기동 여부(sim 무회귀 시 false)'),
        DeclareLaunchArgument('use_task_list_service', default_value='false',
                              description='true 면 서비스 경로(GetTaskList) 병행. (기본 topic 경로)'),
        DeclareLaunchArgument('task_list_service_name', default_value='/mission_a/task_list',
                              description='서비스 경로명. 실 perception 연동 시 /perception/get_task_list 로 remap'),
        DeclareLaunchArgument('task_list_topic', default_value='/perception/task_list',
                              description='task_list 토픽명. 서비스 경로 단독 검증 시 미사용 토픽으로 redirect 가능'),
        DeclareLaunchArgument('nav_mode', default_value='stub',
                              description='{stub|service} — 기본 stub(단계1), service(단계2)'),
        DeclareLaunchArgument('use_place_pose_check', default_value='false'),
        DeclareLaunchArgument('rescan_each_cycle', default_value='true'),
        DeclareLaunchArgument('base_shift_mm', default_value='675.0'),
        DeclareLaunchArgument('scan_pose_preset_id', default_value=''),
        DeclareLaunchArgument('place_pose_valid_debounce_sec', default_value='0.3'),
        # nav mock (service)
        DeclareLaunchArgument('travel_sec', default_value='1.0'),
        DeclareLaunchArgument('fail_arrive', default_value='false'),
        DeclareLaunchArgument('lateral_error_mm', default_value='2.0'),
        # manip mock
        DeclareLaunchArgument('drop_during_move', default_value='false'),
        DeclareLaunchArgument('drop_after_attach_sec', default_value='0.5'),
        # perception mock
        DeclareLaunchArgument('place_pose_invalid', default_value='false'),
        DeclareLaunchArgument('place_pose_flap', default_value='false'),
        DeclareLaunchArgument('parts_json', default_value=''),
        DeclareLaunchArgument('mock_pub_task_list', default_value='true',
                              description='실 perception(tray_manage_node) task_list 사용 시 false'),
    ]

    lc = LaunchConfiguration
    use_mocks = lc('use_mocks')

    mission_a = Node(
        package='mission', executable='mission_a', name='mission_a',
        output='screen',
        parameters=[{
            'sim_mode': lc('sim_mode'),
            'use_task_list_service': lc('use_task_list_service'),
            'task_list_service_name': lc('task_list_service_name'),
            'task_list_topic': lc('task_list_topic'),
            'nav_mode': lc('nav_mode'),
            'use_place_pose_check': lc('use_place_pose_check'),
            'rescan_each_cycle': lc('rescan_each_cycle'),
            'base_shift_mm': lc('base_shift_mm'),
            'scan_pose_preset_id': lc('scan_pose_preset_id'),
            'place_pose_valid_debounce_sec': lc('place_pose_valid_debounce_sec'),
        }],
    )

    mock_manip = Node(
        package='mission', executable='mock_manipulation_a', name='mock_manipulation_a',
        output='screen', condition=IfCondition(use_mocks),
        parameters=[{
            'drop_during_move': lc('drop_during_move'),
            'drop_after_attach_sec': lc('drop_after_attach_sec'),
        }],
    )

    mock_nav = Node(
        package='mission', executable='mock_navigation_a', name='mock_navigation_a',
        output='screen', condition=IfCondition(use_mocks),
        parameters=[{
            'travel_sec': lc('travel_sec'),
            'fail_arrive': lc('fail_arrive'),
            'lateral_error_mm': lc('lateral_error_mm'),
        }],
    )

    mock_perc = Node(
        package='mission', executable='mock_perception_a', name='mock_perception_a',
        output='screen', condition=IfCondition(use_mocks),
        parameters=[{
            'place_pose_invalid': lc('place_pose_invalid'),
            'place_pose_flap': lc('place_pose_flap'),
            'parts_json': lc('parts_json'),
            'pub_task_list': lc('mock_pub_task_list'),
        }],
    )

    return LaunchDescription(args + [mission_a, mock_manip, mock_nav, mock_perc])
