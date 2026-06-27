#!/usr/bin/env python3
"""Mission C launch — FSM(mission_c) + peg 중심 공급원 + (옵션) mock 3종.

peg 중심(place 타깃) 공급원은 `pipe_source` 로 토글:
  · pipe_source:=preset (기본) — perception `pipe_centers_preset_pub`(사전 측정값, 학습 전 임시).
  · pipe_source:=model         — 실 `head_pipe_top_centers_node`(학습 모델). 학습 완료 시 이걸로 교체.
다운스트림(FSM/manip)은 동일 토픽(/perception/head/pipe_top_centers)이라 무변경.

검증 구성(로봇 無):
  ros2 launch mission mission_c.launch.py
    → mission_c(nav_mode=stub) + mock 3종 + preset peg. 전 구간 헤드리스 사이클(삽입 N) 검증.
  ros2 launch mission mission_c.launch.py nav_mode:=service
    → 실 MoveBaseLateral 연동(실 nav 서버는 로봇 PC 별도 기동).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('sim_mode', default_value='false'),
        DeclareLaunchArgument('use_mocks', default_value='true',
                              description='mock 3종(manip/perception/nav) 기동'),
        DeclareLaunchArgument('nav_mode', default_value='stub',
                              description='{stub|service} — service 시 실 MoveBaseLateral(로봇 PC)'),
        DeclareLaunchArgument('nav_service_wait_sec', default_value='10.0'),
        DeclareLaunchArgument('base_shift_mm', default_value='675.0'),
        DeclareLaunchArgument('arm_mode', default_value='right',
                              description='{right|left|auto} — 현 단계 우완 단일팔(기본). '
                                          '실 manip 서버 arm_mode 와 일치시킬 것'),
        DeclareLaunchArgument('use_place_pose_check', default_value='false',
                              description='C3 peg 삽입 위치 유효성 게이트'),
        DeclareLaunchArgument('insert_dry_run', default_value='false',
                              description='C3_INSERT dry-run(임의/안전 위치 release로 항상 통과). '
                                          '실 manip 서버 insert_dry_run 과 함께 사용'),
        DeclareLaunchArgument('insert_dry_run_grace_sec', default_value='2.0',
                              description='dry-run 해제 강제확정 grace(초)'),
        DeclareLaunchArgument('scan_pose_preset_id', default_value=''),
        # 미션 C 베이스 시퀀스(옵션, 기본 OFF=기존 동작)
        DeclareLaunchArgument('base_seq_enable', default_value='false',
                              description='true=너트 A/B 정렬 + pipe3 기준 place 시퀀스'),
        DeclareLaunchArgument('use_monitor_ocr', default_value='true',
                              description='false=OCR task_list 미사용, pick_positions 기반 수동 task_list 사용'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='legacy. base_seq 너트 pick은 항상 perception/camera 사용'),
        DeclareLaunchArgument('use_pipe_camera', default_value='false',
                              description='false=manual pipe3 hardcoded place, true=perception pipe center place'),
        DeclareLaunchArgument('nut_pitch_mm', default_value='150.0',
                              description='legacy. base_seq pick A/B 모드에서는 미사용'),
        DeclareLaunchArgument('place_forward_mm', default_value='100.0',
                              description='legacy. 현재 base_seq place에서는 미사용'),
        DeclareLaunchArgument('pick_positions', default_value='',
                              description="픽 순서를 너트 위치(1~5,왼→오)로 지정 예:'4-5-3-1'. "
                                          '설정 시 pick_order 대체'),
        # peg 중심 공급원 토글
        DeclareLaunchArgument('pipe_source', default_value='preset',
                              description='{preset|model} — peg 중심 공급(학습 전 preset)'),
        # preset peg 좌표(실측, R4). 학습 모델 사용 시 무시.
        DeclareLaunchArgument('pipe_x', default_value='0.40'),
        DeclareLaunchArgument('pipe_z', default_value='0.90'),
        # nav mock
        DeclareLaunchArgument('travel_sec', default_value='1.0'),
        DeclareLaunchArgument('fail_arrive', default_value='false'),
        # manip/perception mock
        DeclareLaunchArgument('drop_during_move', default_value='false'),
        DeclareLaunchArgument('place_pose_invalid', default_value='false'),
        DeclareLaunchArgument('parts_json', default_value=''),
    ]

    use_mocks = lc('use_mocks')
    is_preset = IfCondition(PythonExpression(["'", lc('pipe_source'), "' == 'preset'"]))

    mission_c = Node(
        package='mission', executable='mission_c', name='mission_c', output='screen',
        parameters=[{
            'sim_mode': lc('sim_mode'),
            'nav_mode': lc('nav_mode'),
            'nav_service_wait_sec': lc('nav_service_wait_sec'),
            'base_shift_mm': lc('base_shift_mm'),
            'arm_mode': lc('arm_mode'),
            'use_place_pose_check': lc('use_place_pose_check'),
            'insert_dry_run': lc('insert_dry_run'),
            'insert_dry_run_grace_sec': lc('insert_dry_run_grace_sec'),
            'scan_pose_preset_id': lc('scan_pose_preset_id'),
            'base_seq_enable': lc('base_seq_enable'),
            'use_monitor_ocr': lc('use_monitor_ocr'),
            'use_camera': lc('use_camera'),
            'use_pipe_camera': lc('use_pipe_camera'),
            'nut_pitch_mm': lc('nut_pitch_mm'),
            'place_forward_mm': lc('place_forward_mm'),
            'pick_positions': lc('pick_positions'),
        }],
    )

    # peg 중심: preset(임시) — 학습 완료 시 pipe_source:=model 로 실 노드 교체.
    preset_pegs = Node(
        package='perception', executable='pipe_centers_preset_pub',
        name='pipe_centers_preset_pub', output='screen', condition=is_preset,
        parameters=[{'pipe_x': lc('pipe_x'), 'pipe_z': lc('pipe_z')}],
    )

    mock_manip = Node(
        package='mission', executable='mock_manipulation_a', name='mock_manipulation_a',
        output='screen', condition=IfCondition(use_mocks),
        parameters=[{'drop_during_move': lc('drop_during_move')}],
    )
    mock_nav = Node(
        package='mission', executable='mock_navigation_a', name='mock_navigation_a',
        output='screen', condition=IfCondition(use_mocks),
        parameters=[{'travel_sec': lc('travel_sec'), 'fail_arrive': lc('fail_arrive')}],
    )
    mock_perc = Node(
        package='mission', executable='mock_perception_a', name='mock_perception_a',
        output='screen', condition=IfCondition(use_mocks),
        parameters=[{
            'place_pose_invalid': lc('place_pose_invalid'),
            'parts_json': ParameterValue(lc('parts_json'), value_type=str),
            'pub_task_list': lc('use_monitor_ocr'),
        }],
    )

    return LaunchDescription(args + [mission_c, preset_pegs, mock_manip, mock_nav, mock_perc])
