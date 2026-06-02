import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_tray_model = os.environ.get(
        "TRAY_MODEL_PATH",
        os.path.join(
            get_package_share_directory("perception"),
            "model",
            "tray_occupancy_best.pt",
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument("detections_topic", default_value="/detections"),
        DeclareLaunchArgument("image_topic", default_value="/zed/zed_node/rgb/image_rect_color"),
        DeclareLaunchArgument("tray_detection_service_name", default_value="/mission_a/tray_detections"),
        DeclareLaunchArgument("tray_detection_service_timeout_sec", default_value="3.0"),
        DeclareLaunchArgument("tray_detection_service_frame_count", default_value="1"),
        DeclareLaunchArgument("tray_model_path", default_value=default_tray_model),
        DeclareLaunchArgument("tray_conf_threshold", default_value="0.50"),
        DeclareLaunchArgument("tray_iou_threshold", default_value="0.35"),
        DeclareLaunchArgument("tray_imgsz", default_value="640"),
        DeclareLaunchArgument("tray_max_age_sec", default_value="1.0"),
        DeclareLaunchArgument("tray_process_interval_sec", default_value="0.10"),
        DeclareLaunchArgument("part_min_confidence", default_value="0.30"),
        DeclareLaunchArgument("bbox_margin_px", default_value="0.0"),
        DeclareLaunchArgument("source_camera_filter", default_value=""),
        DeclareLaunchArgument("tray_python", default_value="/ws/yolo_venv/bin/python3"),

        Node(
            package="perception",
            executable="tray_occupancy_node",
            name="tray_occupancy_node",
            prefix=LaunchConfiguration("tray_python"),
            parameters=[{
                "detections_topic": LaunchConfiguration("detections_topic"),
                "image_topic": LaunchConfiguration("image_topic"),
                "tray_detection_service_name": LaunchConfiguration("tray_detection_service_name"),
                "tray_detection_service_timeout_sec": ParameterValue(
                    LaunchConfiguration("tray_detection_service_timeout_sec"),
                    value_type=float,
                ),
                "tray_detection_service_frame_count": ParameterValue(
                    LaunchConfiguration("tray_detection_service_frame_count"),
                    value_type=int,
                ),
                "tray_model_path": LaunchConfiguration("tray_model_path"),
                "tray_conf_threshold": ParameterValue(
                    LaunchConfiguration("tray_conf_threshold"),
                    value_type=float,
                ),
                "tray_iou_threshold": ParameterValue(
                    LaunchConfiguration("tray_iou_threshold"),
                    value_type=float,
                ),
                "tray_imgsz": ParameterValue(LaunchConfiguration("tray_imgsz"), value_type=int),
                "tray_max_age_sec": ParameterValue(
                    LaunchConfiguration("tray_max_age_sec"),
                    value_type=float,
                ),
                "tray_process_interval_sec": ParameterValue(
                    LaunchConfiguration("tray_process_interval_sec"),
                    value_type=float,
                ),
                "part_min_confidence": ParameterValue(
                    LaunchConfiguration("part_min_confidence"),
                    value_type=float,
                ),
                "bbox_margin_px": ParameterValue(LaunchConfiguration("bbox_margin_px"), value_type=float),
                "source_camera_filter": LaunchConfiguration("source_camera_filter"),
            }],
            output="screen",
        ),
    ])
