"""Ⓑ-3 단독 실행 — 지정 위치 안착 + 왕복 이동.

순서: APPROACH_B(책상 접근) → dual_place(안착) → 완료 신호 → B_TO_A(A 복귀).
선행조건: ffw_mission_b_nav 코디네이터 상태 WAITING_APPROACH_B_ACTION
          (= Ⓑ-2 의 REACHED_B_STOP_LINE 이후) + 로봇 bringup/MoveIt/manipulation.
검증:    /mission_b/monitor → delivery_text="안착 완료", /mission_b/nav/event → REACHED_A

  ros2 launch mission mission_b_b3.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('place_cmd',
                              default_value='ros2 run manipulation test_dual_place'),
        Node(
            package='mission', executable='mission_b', name='mission_b', output='screen',
            parameters=[{
                'stage': 'b3',
                'place_cmd': LaunchConfiguration('place_cmd'),
            }],
        ),
    ])
