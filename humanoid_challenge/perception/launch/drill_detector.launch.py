import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('perception')
    part_name = 'drill'

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'part_detector', f'{part_name}_params.yaml'),
            description='Path to drill detector parameters',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=os.path.join(pkg_share, 'model', f'{part_name}_best.pt'),
            description='Path to drill detector YOLO weights',
        ),
        Node(
            package='perception',
            executable='detector',
            name=f'{part_name}_detector',
            parameters=[
                LaunchConfiguration('params_file'),
                {'model_path': LaunchConfiguration('model_path')},
            ],
            remappings=[],
            output='screen',
        ),
    ])
