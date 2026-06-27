"""Ⓑ-1 단독 실행 — 박스 파지 + 출발 선언 (manipulation dual_pick).

선행조건: 로봇 bringup + MoveIt + manipulation 패키지 빌드(실행).
검증:    ros2 topic echo /mission_b/monitor  → departure_text="출발 가능"

  ros2 launch mission mission_b_b1.launch.py
  # 드라이런(무로봇): mock_nav_b 와 함께, pick_cmd:='true'
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pick_cmd',
                              default_value='ros2 run manipulation test_dual_pick'),
        Node(
            package='mission', executable='mission_b', name='mission_b', output='screen',
            parameters=[{
                'stage': 'b1',
                'pick_cmd': LaunchConfiguration('pick_cmd'),
            }],
        ),
    ])
