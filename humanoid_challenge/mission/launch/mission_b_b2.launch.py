"""Ⓑ-2 단독 실행 — 정지선 도착 (nav A_TO_B: 후진→우횡이동→전진+LiDAR 정렬).

선행조건: ffw_mission_b_nav 코디네이터 구동 + 상태 IDLE(또는 STOPPED).
검증:    ros2 topic echo /mission_b/nav/event  → REACHED_B_STOP_LINE
         ros2 topic echo /mission_b/monitor    → stopline_text="정지선 도착"

  ros2 launch mission mission_b_b2.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stop_line_dwell_sec', default_value='1.5'),
        Node(
            package='mission', executable='mission_b', name='mission_b', output='screen',
            parameters=[{
                'stage': 'b2',
                'stop_line_dwell_sec': ParameterValue(
                    LaunchConfiguration('stop_line_dwell_sec'), value_type=float),
            }],
        ),
    ])
