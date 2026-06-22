from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ocr_python", default_value="/ws/ocr_venv/bin/python3"),
        DeclareLaunchArgument("image_topic", default_value="/zed/zed_node/rgb/image_rect_color"),
        DeclareLaunchArgument("result_topic", default_value="/monitor_ocr/result"),
        DeclareLaunchArgument("parts_topic", default_value="/monitor_ocr/parts"),
        DeclareLaunchArgument("part_counts_topic", default_value="/monitor_ocr/part_counts"),
        DeclareLaunchArgument("recognized_topic", default_value="/monitor_ocr/recognized"),
        DeclareLaunchArgument("process_interval", default_value="2.0"),
        DeclareLaunchArgument("task_list_service_name", default_value="/mission_a/task_list"),
        DeclareLaunchArgument("task_list_service_timeout_sec", default_value="20.0"),
        DeclareLaunchArgument("task_list_service_frame_count", default_value="3"),

        Node(
            package="perception",
            executable="monitor_ocr_node",
            name="monitor_ocr_node",
            prefix=LaunchConfiguration("ocr_python"),
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "result_topic": LaunchConfiguration("result_topic"),
                "parts_topic": LaunchConfiguration("parts_topic"),
                "part_counts_topic": LaunchConfiguration("part_counts_topic"),
                "recognized_topic": LaunchConfiguration("recognized_topic"),
                "process_interval": ParameterValue(
                    LaunchConfiguration("process_interval"),
                    value_type=float,
                ),
                "task_list_service_name": LaunchConfiguration("task_list_service_name"),
                "task_list_service_timeout_sec": ParameterValue(
                    LaunchConfiguration("task_list_service_timeout_sec"),
                    value_type=float,
                ),
                "task_list_service_frame_count": ParameterValue(
                    LaunchConfiguration("task_list_service_frame_count"),
                    value_type=int,
                ),
            }],
            output="screen",
        ),
    ])
