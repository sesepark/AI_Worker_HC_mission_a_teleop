#!/usr/bin/env python3
"""실 MoveBaseLateral nav 서버 단독 launch.

이 노드는 /cmd_vel(Twist) 발행 + /odom(Odometry) 구독으로 SG2 swerve 베이스를 측방 strafe 한다.
따라서 **/cmd_vel·/odom 하드웨어가 있는 로봇 PC에서 단독 기동**한다(FSM 은 desktop → cross-PC service).

  robot PC$  ros2 launch mission move_base_lateral.launch.py
  desktop$   ros2 launch mission mission_a.launch.py use_mocks:=false nav_mode:=service

기존 mock/stub 회귀(mission_a.launch.py)에는 영향 없음(별도 launch).

안전: 콜드 첫 호출 검증은 무이동 경로로 가능 —
  ros2 service call /move_base_lateral mission_interfaces/srv/MoveBaseLateral "{direction: left, distance_mm: 0.0}"
실 이동은 저속·E-stop 준비 하에 사용자가 트리거.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('service_name', default_value='move_base_lateral'),
        DeclareLaunchArgument('speed', default_value='0.12',
                              description='측방 strafe 속도 [m/s]'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('rate_hz', default_value='20.0'),
        DeclareLaunchArgument('max_duration_sec', default_value='12.0',
                              description='이동 상한 [s] — FSM base_move_timeout_sec(30) 보다 작게'),
        DeclareLaunchArgument('wrong_direction_tolerance', default_value='0.05'),
        DeclareLaunchArgument('use_odom_stop', default_value='true',
                              description='true=odom 폐루프 정지, false=개루프(벤치용)'),
        DeclareLaunchArgument('wait_for_odom_sec', default_value='3.0',
                              description='이동 전 odom 신선도 대기(미수신 시 무이동 실패)'),
        DeclareLaunchArgument('fail_inject', default_value='false',
                              description='true 면 강제 arrived=false(→ FSM RECOVERY 검증)'),
    ]

    lc = LaunchConfiguration
    node = Node(
        package='mission', executable='move_base_lateral', name='move_base_lateral',
        output='screen',
        parameters=[{
            'service_name': lc('service_name'),
            'speed': lc('speed'),
            'cmd_vel_topic': lc('cmd_vel_topic'),
            'odom_topic': lc('odom_topic'),
            'rate_hz': lc('rate_hz'),
            'max_duration_sec': lc('max_duration_sec'),
            'wrong_direction_tolerance': lc('wrong_direction_tolerance'),
            'use_odom_stop': lc('use_odom_stop'),
            'wait_for_odom_sec': lc('wait_for_odom_sec'),
            'fail_inject': lc('fail_inject'),
        }],
    )

    return LaunchDescription(args + [node])
