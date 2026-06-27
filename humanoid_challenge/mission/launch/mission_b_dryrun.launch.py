"""Mission B 무로봇 드라이런 — mock_nav_b + FSM(stage=all, manipulation stub).

로봇/MoveIt 없이 FSM 로직·신호 흐름·nav action 시퀀스를 검증한다. 실제 이동/파지는 없음.
manipulation 은 셸 no-op('true')로 대체, nav 는 mock_nav_b 로 대체.

  ros2 launch mission mission_b_dryrun.launch.py
  ros2 topic echo /mission_b/monitor
  ros2 topic echo /mission_b/system/action
  ros2 topic echo /mission_b/nav/event
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('auto_chain', default_value='true'),
        DeclareLaunchArgument('max_boxes', default_value='4'),
        DeclareLaunchArgument('leg_sec', default_value='2.0'),
        Node(
            package='mission', executable='mock_nav_b', name='mock_nav_b', output='screen',
            parameters=[{
                'leg_sec': ParameterValue(LaunchConfiguration('leg_sec'), value_type=float),
            }],
        ),
        Node(
            package='mission', executable='mission_b', name='mission_b', output='screen',
            parameters=[{
                'stage': 'all',
                'auto_chain': ParameterValue(
                    LaunchConfiguration('auto_chain'), value_type=bool),
                'max_boxes': ParameterValue(
                    LaunchConfiguration('max_boxes'), value_type=int),
                'pick_cmd': 'true',
                'place_cmd': 'true',
                'stop_line_dwell_sec': 1.0,
                'manip_timeout': 30.0,
            }],
        ),
    ])
