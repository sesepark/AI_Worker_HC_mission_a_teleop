"""Operator-side RViz launch for SG2 teleoperation practice."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def make_rviz_node(context, *args, **kwargs):
    del args, kwargs
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    rviz_gl_mode = LaunchConfiguration('rviz_gl_mode').perform(context).strip().lower()
    env = {
        'QT_QPA_PLATFORM': 'xcb',
        'QT_X11_NO_MITSHM': '1',
        'LIBGL_DRI3_DISABLE': '1',
    }
    if rviz_gl_mode in ('software', 'llvmpipe', 'mesa'):
        env.update({
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'QT_OPENGL': 'software',
            'GALLIUM_DRIVER': 'llvmpipe',
            'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
        })

    return [Node(
        package='rviz2',
        executable='rviz2',
        name='teleop_operator_rviz',
        arguments=['-d', rviz_config],
        output='screen',
        emulate_tty=True,
        additional_env=env,
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_gl_mode', default_value='native'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'rviz',
                'teleop_operator.rviz',
            ]),
        ),
        OpaqueFunction(function=make_rviz_node),
    ])
