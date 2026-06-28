#!/usr/bin/env python3
"""perception_live — Mission A live perception 단일 launch (T2).

분리 기동되던 perception 런타임을 한 launch 그룹으로 통합(동일 기동 윈도우 → 컨테이너 DDS
디스커버리 안정, CONTEXT §1.2). 포함:
  · part_detector(detector_node, camera_name=wrist_right) → /detections
  · tray_manage_node → /perception/task_list (GetTaskList.Response) + /perception/get_task_list(srv)
  · wrist_task_grasp_planner_node → /perception/wrist/target_one_pose (실 검출, §4.3 파라미터 기본 노출)
  · static TF: camera_r_link → camera_right_link (identity; 현재 수동 게시 → launch 통합)
  · place_pose_valid_node → /perception/place_pose_valid (C3, FSM valid 키)

전제: wrist 카메라 bringup(ffw_sg2_ai.launch.py) 가 선행되어 /camera_right/... 이미지·camera_info,
base_link↔camera_right_link TF 가 가용해야 실 wrist target 이 산출된다(로봇/카메라 영역).
mock_monitor_ocr:=true 면 모니터 OCR 없이 task_list 공급(카메라 없이 task_list 부분만 헤드리스 가능).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('perception')
    detector_config = os.path.join(pkg, 'config', 'part_detector', 'params.yaml')
    detector_model = os.path.join(pkg, 'model', 'part_detector_best.pt')
    tray_model = os.environ.get(
        'TRAY_MODEL_PATH', os.path.join(pkg, 'model', 'tray_occupancy_best.pt'))
    wrist_params = os.path.join(pkg, 'config', 'wrist_projection', 'params.yaml')

    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('camera_name', default_value='wrist_right'),
        DeclareLaunchArgument('detections_topic', default_value='/detections'),
        DeclareLaunchArgument('task_list_topic', default_value='/perception/task_list'),
        DeclareLaunchArgument('task_list_service_name', default_value='/perception/get_task_list'),
        DeclareLaunchArgument('mock_monitor_ocr', default_value='true',
                              description='true=모니터 OCR mock(카메라 없이 task_list), false=실 OCR'),
        DeclareLaunchArgument('enable_tray_detection', default_value='false'),
        DeclareLaunchArgument('require_complete_ocr', default_value='false'),
        DeclareLaunchArgument('yolo_python', default_value='/ws/yolo_venv/bin/python3'),
        # wrist planner (§4.3 기본값 노출)
        DeclareLaunchArgument('arm_reference_frame', default_value='camera_right_link'),
        # mission-a 87bcf99 wrist-select 정합: 즉시 select(min_obs=1) + jitter 허용 gate(0.10)
        #   + 짧은 window(1.0s). 2회 안정 게이트가 지터/저속검출로 안 차던 문제 해결.
        DeclareLaunchArgument('temporal_window_sec', default_value='1.0'),
        DeclareLaunchArgument('temporal_min_observations', default_value='1'),
        DeclareLaunchArgument('temporal_position_gate_m', default_value='0.10'),
        # static TF (camera_r_link → camera_right_link). 실 bringup 이 이미 게시하면 publish_camera_tf:=false
        DeclareLaunchArgument('publish_camera_tf', default_value='true'),
        # C3 place_pose_valid 주입(검증용)
        DeclareLaunchArgument('place_force_invalid', default_value='false'),
        DeclareLaunchArgument('place_flap', default_value='false'),
        DeclareLaunchArgument('place_default_valid', default_value='true'),
    ]

    detector = Node(
        package='perception', executable='detector_node', name='part_detector',
        parameters=[detector_config, {
            'camera_name': lc('camera_name'),
            'detections_topic': lc('detections_topic'),
            'model_path': detector_model,
        }],
        output='screen',
    )
    tray = Node(
        package='perception', executable='tray_manage_node', name='tray_manage_node',
        prefix=lc('yolo_python'),
        parameters=[{
            'task_list_topic': lc('task_list_topic'),
            'task_list_service_name': lc('task_list_service_name'),
            'tray_model_path': tray_model,
            'enable_tray_detection': ParameterValue(lc('enable_tray_detection'), value_type=bool),
            'require_complete_ocr': ParameterValue(lc('require_complete_ocr'), value_type=bool),
            'mock_monitor_ocr': ParameterValue(lc('mock_monitor_ocr'), value_type=bool),
        }],
        output='screen',
    )
    wrist = Node(
        package='perception', executable='wrist_task_grasp_planner_node',
        name='wrist_task_grasp_planner_node', prefix=lc('yolo_python'),
        parameters=[wrist_params, {
            'detections_topic': lc('detections_topic'),
            'task_topic': lc('task_list_topic'),
            'arm_reference_frame': lc('arm_reference_frame'),
            'temporal_window_sec': ParameterValue(lc('temporal_window_sec'), value_type=float),
            'temporal_min_observations': ParameterValue(lc('temporal_min_observations'), value_type=int),
            'temporal_position_gate_m': ParameterValue(lc('temporal_position_gate_m'), value_type=float),
        }],
        output='screen',
    )
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_right_static_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'camera_r_link', '--child-frame-id', 'camera_right_link'],
        condition=IfCondition(lc('publish_camera_tf')),
        output='screen',
    )
    place_valid = Node(
        package='perception', executable='place_pose_valid_node', name='place_pose_valid_node',
        parameters=[{
            'force_invalid': ParameterValue(lc('place_force_invalid'), value_type=bool),
            'flap': ParameterValue(lc('place_flap'), value_type=bool),
            'default_valid': ParameterValue(lc('place_default_valid'), value_type=bool),
        }],
        output='screen',
    )

    return LaunchDescription(args + [detector, tray, wrist, static_tf, place_valid])
