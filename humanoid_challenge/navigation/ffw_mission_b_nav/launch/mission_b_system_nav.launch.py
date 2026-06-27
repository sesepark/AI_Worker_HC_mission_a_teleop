#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_config = PathJoinSubstitution([
        FindPackageShare('ffw_mission_b_nav'),
        'config',
        'mission_b_real_robot.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Mission B system navigation parameter file.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock. Keep false on the real robot.'),
        Node(
            package='ffw_mission_b_nav',
            executable='sg2_mission_b_system_nav',
            name='sg2_mission_b_system_nav',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
