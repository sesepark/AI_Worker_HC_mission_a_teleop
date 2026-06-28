#!/usr/bin/env python3
"""Attach teleoperation feedback to an already-running SG2 bringup.

This launch intentionally does not start follower bringup, ZED, or RealSense.
It only converts existing robot-side raw topics into teleoperation feedback
topics for the main PC operator UI.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_zed_depth_assist = LaunchConfiguration('start_zed_depth_assist')
    start_right_wrist_depth_assist = LaunchConfiguration('start_right_wrist_depth_assist')
    start_alignment_monitor = LaunchConfiguration('start_alignment_monitor')
    start_bandwidth_monitor = LaunchConfiguration('start_bandwidth_monitor')
    dxl_current_topic = LaunchConfiguration('dxl_current_topic')
    dxl_current_calibration_s = LaunchConfiguration('dxl_current_calibration_s')
    dxl_current_baseline_floor = LaunchConfiguration('dxl_current_baseline_floor')
    dxl_current_caution_ratio = LaunchConfiguration('dxl_current_caution_ratio')
    dxl_current_warn_ratio = LaunchConfiguration('dxl_current_warn_ratio')
    dxl_current_stale_timeout_s = LaunchConfiguration('dxl_current_stale_timeout_s')

    zed_depth_assist = Node(
        package='ffw_teleop',
        executable='zed_depth_assist',
        name='zed_depth_assist',
        output='screen',
        condition=IfCondition(start_zed_depth_assist),
        parameters=[{
            'depth_topic': LaunchConfiguration('zed_depth_topic'),
            'base_image_topic': LaunchConfiguration('zed_base_image_topic'),
            'camera_info_topic': LaunchConfiguration('zed_camera_info_topic'),
            'assist_topic': LaunchConfiguration('zed_assist_topic'),
            'metrics_topic': LaunchConfiguration('zed_metrics_topic'),
            'stream_stats_topic': LaunchConfiguration('stream_stats_topic'),
            'camera_perf_topic': LaunchConfiguration('camera_perf_topic'),
            'stream_stats_name': 'zed',
            'assist_mode': LaunchConfiguration('zed_assist_mode'),
            'publish_fps': LaunchConfiguration('zed_assist_fps'),
            'jpeg_quality': LaunchConfiguration('zed_assist_jpeg_quality'),
            'min_depth_m': LaunchConfiguration('zed_min_depth_m'),
            'max_depth_m': LaunchConfiguration('zed_max_depth_m'),
            'base_image_timeout_s': LaunchConfiguration('zed_base_image_timeout_s'),
            'camera_optical_frame': LaunchConfiguration('zed_camera_optical_frame'),
            'use_latest_tf': LaunchConfiguration('zed_use_latest_tf'),
            'tf_lookup_timeout_s': LaunchConfiguration('zed_tf_lookup_timeout_s'),
            'left_hand_frame': LaunchConfiguration('zed_left_hand_frame'),
            'right_hand_frame': LaunchConfiguration('zed_right_hand_frame'),
            'enable_near_hand_objects': 'false',
        }],
    )

    right_wrist_depth_assist = Node(
        package='ffw_teleop',
        executable='right_wrist_depth_overlay',
        name='right_wrist_depth_overlay',
        output='screen',
        condition=IfCondition(start_right_wrist_depth_assist),
        parameters=[{
            'depth_topic': LaunchConfiguration('right_wrist_depth_topic'),
            'base_image_topic': LaunchConfiguration('right_wrist_color_rect_topic'),
            'base_image_fallback_topics': ['/camera_right/camera_right/color/image_raw'],
            'overlay_topic': '/teleop/wrist_right/depth_overlay',
            'compressed_topic': '/teleop/wrist_right/depth_overlay/compressed',
            'assist_topic': LaunchConfiguration('right_wrist_assist_topic'),
            'color_compressed_topic': '/teleop/wrist_right/color/compressed',
            'base_compressed_topic': '/teleop/wrist_right/color/compressed',
            'center_distance_topic': LaunchConfiguration('right_wrist_center_distance_topic'),
            'metrics_topic': LaunchConfiguration('right_wrist_metrics_topic'),
            'stream_stats_topic': LaunchConfiguration('stream_stats_topic'),
            'camera_perf_topic': LaunchConfiguration('camera_perf_topic'),
            'stream_stats_name': 'wrist_right',
            'side': 'right',
            'feedback_visual_mode': 'assist',
            'subscribe_base_image': 'false',
            'publish_raw_overlay': 'false',
            'publish_base_compressed': 'false',
            'publish_metrics': 'true',
            'publish_fps': LaunchConfiguration('right_wrist_assist_fps'),
            'depth_scale': LaunchConfiguration('right_wrist_depth_scale'),
            'min_depth_m': LaunchConfiguration('right_wrist_min_depth_m'),
            'max_depth_m': LaunchConfiguration('right_wrist_max_depth_m'),
            'roi_size_px': '32',
            'jpeg_quality': LaunchConfiguration('right_wrist_jpeg_quality'),
            'colormap': 'VIRIDIS',
            'base_alpha': '0.70',
            'depth_alpha': '0.30',
            'base_image_timeout_s': '0.5',
            'show_depth_contours': 'true',
            'contour_near_depth_m': '0.45',
            'contour_min_area_px': '30.0',
            'invalid_depth_mode': 'base_only',
            'assist_component_margin_m': '0.08',
            'assist_offset_threshold_px': '24',
            'assist_target_depth_m': '0.30',
            'assist_depth_tolerance_m': '0.04',
            'view_preset': 'driver_90',
            'view_rotate_deg': '90.0',
            'view_flip_horizontal': 'false',
            'view_flip_vertical': 'false',
            'gripper_target_offset_x_px': '0',
            'gripper_target_offset_y_px': '96',
            'band_red_max_m': '0.06',
            'band_green_min_m': '0.06',
            'band_green_max_m': '0.13',
            'band_orange_min_m': '0.13',
            'band_orange_max_m': '0.20',
            'band_alpha': '0.45',
            'band_min_area_px': '20.0',
        }],
    )

    alignment_status = Node(
        package='ffw_teleop',
        executable='teleop_alignment_status',
        name='teleop_alignment_status',
        output='screen',
        condition=IfCondition(start_alignment_monitor),
        parameters=[{
            'joystick_mode_topic': LaunchConfiguration('joystick_mode_topic'),
            'head_target_topic': LaunchConfiguration('head_target_topic'),
            'joint_state_topic': LaunchConfiguration('joint_state_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'right_center_distance_topic': LaunchConfiguration(
                'right_wrist_center_distance_topic'),
            'right_depth_metrics_topic': LaunchConfiguration('right_wrist_metrics_topic'),
            'status_panel_topic': LaunchConfiguration('status_panel_topic'),
            'status_panel_jpeg_quality': LaunchConfiguration('status_panel_jpeg_quality'),
            'dxl_current_topic': dxl_current_topic,
            'dxl_current_calibration_s': dxl_current_calibration_s,
            'dxl_current_baseline_floor': dxl_current_baseline_floor,
            'dxl_current_caution_ratio': dxl_current_caution_ratio,
            'dxl_current_warn_ratio': dxl_current_warn_ratio,
            'dxl_current_stale_timeout_s': dxl_current_stale_timeout_s,
            'record_practice_events': 'false',
        }],
    )

    bandwidth_monitor = Node(
        package='ffw_teleop',
        executable='teleop_bandwidth_monitor',
        name='teleop_bandwidth_monitor',
        output='screen',
        condition=IfCondition(start_bandwidth_monitor),
        parameters=[{
            'stream_stats_topic': LaunchConfiguration('stream_stats_topic'),
            'camera_perf_topic': LaunchConfiguration('camera_perf_topic'),
            'monitor_topic': LaunchConfiguration('bandwidth_monitor_topic'),
            'panel_topic': LaunchConfiguration('bandwidth_panel_topic'),
            'available_mbps': LaunchConfiguration('bandwidth_available_mbps'),
            'publish_hz': LaunchConfiguration('bandwidth_monitor_publish_hz'),
            'target_fps': '30.0',
            'usb_available_mbps': LaunchConfiguration('bandwidth_usb_available_mbps'),
            'usb_wrist_left_depth_profile': '',
            'usb_wrist_right_depth_profile': '480,270,30',
            'usb_wrist_left_color_profile': '',
            'usb_wrist_right_color_profile': '424,240,30',
            'usb_wrist_left_depth_enabled': 'false',
            'usb_wrist_right_depth_enabled': 'true',
            'usb_wrist_left_color_enabled': 'false',
            'usb_wrist_right_color_enabled': 'true',
            'wrist_right_color_topic': LaunchConfiguration('right_wrist_color_raw_topic'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_zed_depth_assist', default_value='true'),
        DeclareLaunchArgument('start_right_wrist_depth_assist', default_value='true'),
        DeclareLaunchArgument('start_alignment_monitor', default_value='true'),
        DeclareLaunchArgument('start_bandwidth_monitor', default_value='true'),
        DeclareLaunchArgument('zed_depth_topic', default_value='/zed/zed_node/depth/depth_registered'),
        DeclareLaunchArgument('zed_base_image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('zed_camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument('zed_assist_topic', default_value='/teleop/zed/depth_assist/compressed'),
        DeclareLaunchArgument('zed_metrics_topic', default_value='/teleop/zed/depth_metrics'),
        DeclareLaunchArgument('zed_assist_mode', default_value='tf_header'),
        DeclareLaunchArgument('zed_assist_fps', default_value='30.0'),
        DeclareLaunchArgument('zed_assist_jpeg_quality', default_value='75'),
        DeclareLaunchArgument('zed_min_depth_m', default_value='0.15'),
        DeclareLaunchArgument('zed_max_depth_m', default_value='4.0'),
        DeclareLaunchArgument('zed_base_image_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('zed_camera_optical_frame', default_value=''),
        DeclareLaunchArgument('zed_use_latest_tf', default_value='true'),
        DeclareLaunchArgument('zed_tf_lookup_timeout_s', default_value='0.005'),
        DeclareLaunchArgument('zed_left_hand_frame', default_value='end_effector_l_link'),
        DeclareLaunchArgument('zed_right_hand_frame', default_value='end_effector_r_link'),
        DeclareLaunchArgument(
            'right_wrist_depth_topic',
            default_value='/camera_right/camera_right/depth/image_rect_raw'),
        DeclareLaunchArgument(
            'right_wrist_color_rect_topic',
            default_value='/camera_right/camera_right/color/image_rect_raw'),
        DeclareLaunchArgument(
            'right_wrist_color_raw_topic',
            default_value='/camera_right/camera_right/color/image_rect_raw'),
        DeclareLaunchArgument(
            'right_wrist_assist_topic',
            default_value='/teleop/wrist_right/depth_assist/compressed'),
        DeclareLaunchArgument(
            'right_wrist_center_distance_topic',
            default_value='/teleop/wrist_right/center_distance_m'),
        DeclareLaunchArgument(
            'right_wrist_metrics_topic',
            default_value='/teleop/wrist_right/depth_metrics'),
        DeclareLaunchArgument('right_wrist_assist_fps', default_value='30.0'),
        DeclareLaunchArgument('right_wrist_jpeg_quality', default_value='70'),
        DeclareLaunchArgument('right_wrist_depth_scale', default_value='0.001'),
        DeclareLaunchArgument('right_wrist_min_depth_m', default_value='0.03'),
        DeclareLaunchArgument('right_wrist_max_depth_m', default_value='0.70'),
        DeclareLaunchArgument('stream_stats_topic', default_value='/teleop/stream_stats'),
        DeclareLaunchArgument('camera_perf_topic', default_value='/teleop/camera_perf'),
        DeclareLaunchArgument('joystick_mode_topic', default_value='/leader/joystick_controller_right/joystick_mode'),
        DeclareLaunchArgument('head_target_topic', default_value='/leader/joystick_controller_left/joint_trajectory'),
        DeclareLaunchArgument('joint_state_topic', default_value='/joint_states'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('status_panel_topic', default_value='/teleop/operator_status/compressed'),
        DeclareLaunchArgument('status_panel_jpeg_quality', default_value='95'),
        DeclareLaunchArgument('dxl_current_topic', default_value='/dynamic_joint_states'),
        DeclareLaunchArgument('dxl_current_calibration_s', default_value='3.0'),
        DeclareLaunchArgument('dxl_current_baseline_floor', default_value='50.0'),
        DeclareLaunchArgument('dxl_current_caution_ratio', default_value='1.6'),
        DeclareLaunchArgument('dxl_current_warn_ratio', default_value='2.2'),
        DeclareLaunchArgument('dxl_current_stale_timeout_s', default_value='1.0'),
        DeclareLaunchArgument('bandwidth_monitor_topic', default_value='/teleop/bandwidth_monitor'),
        DeclareLaunchArgument('bandwidth_panel_topic', default_value='/teleop/bandwidth_monitor/compressed'),
        DeclareLaunchArgument('bandwidth_available_mbps', default_value='350.0'),
        DeclareLaunchArgument('bandwidth_usb_available_mbps', default_value='320.0'),
        DeclareLaunchArgument('bandwidth_monitor_publish_hz', default_value='2.0'),
        zed_depth_assist,
        right_wrist_depth_assist,
        alignment_status,
        bandwidth_monitor,
    ])
