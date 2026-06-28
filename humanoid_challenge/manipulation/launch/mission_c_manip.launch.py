#!/usr/bin/env python3
"""Mission C 실 manipulation 서버 launch — ai_worker 컨테이너.

전제: ffw bringup(ffw_sg2_follower_ai.launch.py) + MoveIt(moveit.launch.py) + TRAC-IK 가 선행 기동되어
move_group / controllers / /joint_states 가 가용해야 한다.

이 launch 는 실 mission_c manipulation 서버(peg 삽입)만 띄운다.
mock_manipulation_a / mission_a_manipulation_server 와는 **동시 기동 금지**
(모두 move_to_scan_pose / attach 계약 제공 → 충돌). mock/실 선택은 통합 launch 레벨에서.

arm_mode(기본 right): 현 단계 우완 단일팔. main PC 의 mission_c FSM(arm_mode)과 **일치**시킬 것.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('arm_mode', default_value='right',
                              description='{right|left|auto} — 우완 단일팔(기본). FSM arm_mode 와 일치'),
        DeclareLaunchArgument('insert_dry_run', default_value='false',
                              description='정밀 peg 삽입 생략, 제자리 release(항상 성공). '
                                          'FSM insert_dry_run 과 함께 사용(전 사이클 시험)'),
        DeclareLaunchArgument('collision_table_only', default_value='true',
                              description='planning scene 에 벤치만 등록(볼트/peg 충돌 제외). '
                                          'grasp 직하강이 볼트/peg 실린더와 충돌하는 모순 회피(픽 가능). '
                                          'false=기존 setup_zone_c(볼트/peg 포함)'),
        DeclareLaunchArgument('pick_dry_run', default_value='false',
                              description='실제 파지 생략, 즉시 성공 처리(베이스 이동 시퀀스 검증용). '
                                          'grasp/reach 분리'),
        DeclareLaunchArgument('two_stage_pick', default_value='true',
                              description='test_pick_c operational primitive: two-stage capture 후 pick'),
        DeclareLaunchArgument('pick_capture_z', default_value='1.050'),
        DeclareLaunchArgument('pick_capture_settle', default_value='2.0'),
        DeclareLaunchArgument('pick_perception_timeout', default_value='100.0'),
        DeclareLaunchArgument('use_pipe_camera', default_value='false',
                              description='place_mode 토픽 미수신 시 fallback. true=camera, false=manual'),
        DeclareLaunchArgument('manual_pipe_x', default_value='0.40'),
        DeclareLaunchArgument('manual_pipe_y', default_value='-0.335'),
        DeclareLaunchArgument('manual_pipe_z', default_value='0.90'),
        DeclareLaunchArgument('manual_place_y_offset', default_value='0.0'),
        DeclareLaunchArgument('manual_gripper_open', default_value='0.5'),
        DeclareLaunchArgument('camera_place_y_offset', default_value='-0.030'),
        DeclareLaunchArgument('camera_gripper_open', default_value='0.0'),
        Node(
            package='manipulation',
            executable='mission_c_manipulation_server',
            name='mission_c_manipulation_server',
            output='screen',
            parameters=[{
                'arm_mode': LaunchConfiguration('arm_mode'),
                'insert_dry_run': LaunchConfiguration('insert_dry_run'),
                'collision_table_only': LaunchConfiguration('collision_table_only'),
                'pick_dry_run': LaunchConfiguration('pick_dry_run'),
                'two_stage_pick': LaunchConfiguration('two_stage_pick'),
                'pick_capture_z': LaunchConfiguration('pick_capture_z'),
                'pick_capture_settle': LaunchConfiguration('pick_capture_settle'),
                'pick_perception_timeout': LaunchConfiguration('pick_perception_timeout'),
                'use_pipe_camera': LaunchConfiguration('use_pipe_camera'),
                'manual_pipe_x': LaunchConfiguration('manual_pipe_x'),
                'manual_pipe_y': LaunchConfiguration('manual_pipe_y'),
                'manual_pipe_z': LaunchConfiguration('manual_pipe_z'),
                'manual_place_y_offset': LaunchConfiguration('manual_place_y_offset'),
                'manual_gripper_open': LaunchConfiguration('manual_gripper_open'),
                'camera_place_y_offset': LaunchConfiguration('camera_place_y_offset'),
                'camera_gripper_open': LaunchConfiguration('camera_gripper_open'),
            }],
        ),
    ])
