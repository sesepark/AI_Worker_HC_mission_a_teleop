#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Wonho Yun

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='sh5',
        description='VR model to run: hx5, sg2, or sh5',
    )
    view_only_mode_arg = DeclareLaunchArgument(
        'view_only_mode',
        default_value='true',
        description='SG2 only: block normal VR robot command publishing',
    )
    enable_vr_head_tracking_arg = DeclareLaunchArgument(
        'enable_vr_head_tracking',
        default_value='false',
        description='SG2 only: publish robot head joints from VR headset orientation',
    )
    enable_leader_control_arg = DeclareLaunchArgument(
        'enable_leader_control',
        default_value='false',
        description='SG2 only: marker that real manipulation is handled by Leader',
    )
    enable_vr_robot_control_arg = DeclareLaunchArgument(
        'enable_vr_robot_control',
        default_value='false',
        description='SG2 only: explicitly allow original VR controller robot commands',
    )
    enable_vr_image_arg = DeclareLaunchArgument(
        'enable_vr_image',
        default_value='false',
        description='SG2 only: stream compressed camera topics as the Vuer VR background',
    )
    vr_image_left_topic_arg = DeclareLaunchArgument(
        'vr_image_left_topic',
        default_value='/zed/zed_node/left/image_rect_color/compressed',
        description='SG2 only: compressed image topic for the left VR background layer',
    )
    vr_image_right_topic_arg = DeclareLaunchArgument(
        'vr_image_right_topic',
        default_value='/zed/zed_node/right/image_rect_color/compressed',
        description='SG2 only: compressed image topic for the right VR background layer',
    )
    vr_image_fps_arg = DeclareLaunchArgument(
        'vr_image_fps',
        default_value='15.0',
        description='SG2 only: maximum camera background update rate in Hz',
    )
    vr_head_tracking_hz_arg = DeclareLaunchArgument(
        'vr_head_tracking_hz',
        default_value='10.0',
        description='SG2 only: maximum VR head tracking command rate in Hz',
    )
    vr_head_tracking_deadband_rad_arg = DeclareLaunchArgument(
        'vr_head_tracking_deadband_rad',
        default_value='0.01',
        description='SG2 only: headset deadband before head joint movement',
    )
    vr_head_tracking_smoothing_alpha_arg = DeclareLaunchArgument(
        'vr_head_tracking_smoothing_alpha',
        default_value='0.25',
        description='SG2 only: low-pass smoothing alpha for VR head tracking',
    )
    vr_head_tracking_max_delta_arg = DeclareLaunchArgument(
        'vr_head_tracking_max_delta_per_update',
        default_value='0.03',
        description='SG2 only: max head joint step per VR tracking update in rad',
    )
    vr_head_tracking_pitch_scale_arg = DeclareLaunchArgument(
        'vr_head_tracking_pitch_scale',
        default_value='1.0',
        description='SG2 only: scale from headset pitch to head_joint1',
    )
    vr_head_tracking_yaw_scale_arg = DeclareLaunchArgument(
        'vr_head_tracking_yaw_scale',
        default_value='-1.0',
        description='SG2 only: scale from headset right-yaw to head_joint2',
    )
    vr_head_joint1_min_arg = DeclareLaunchArgument(
        'vr_head_joint1_min',
        default_value='-0.20',
        description='SG2 only: conservative head_joint1 lower command limit',
    )
    vr_head_joint1_max_arg = DeclareLaunchArgument(
        'vr_head_joint1_max',
        default_value='0.50',
        description='SG2 only: conservative head_joint1 upper command limit',
    )
    vr_head_joint2_min_arg = DeclareLaunchArgument(
        'vr_head_joint2_min',
        default_value='-0.28',
        description='SG2 only: conservative head_joint2 lower command limit',
    )
    vr_head_joint2_max_arg = DeclareLaunchArgument(
        'vr_head_joint2_max',
        default_value='0.28',
        description='SG2 only: conservative head_joint2 upper command limit',
    )

    model = LaunchConfiguration('model')
    view_only_mode = LaunchConfiguration('view_only_mode')
    enable_vr_head_tracking = LaunchConfiguration('enable_vr_head_tracking')
    enable_leader_control = LaunchConfiguration('enable_leader_control')
    enable_vr_robot_control = LaunchConfiguration('enable_vr_robot_control')
    enable_vr_image = LaunchConfiguration('enable_vr_image')
    vr_image_left_topic = LaunchConfiguration('vr_image_left_topic')
    vr_image_right_topic = LaunchConfiguration('vr_image_right_topic')
    vr_image_fps = LaunchConfiguration('vr_image_fps')
    vr_head_tracking_hz = LaunchConfiguration('vr_head_tracking_hz')
    vr_head_tracking_deadband_rad = LaunchConfiguration('vr_head_tracking_deadband_rad')
    vr_head_tracking_smoothing_alpha = LaunchConfiguration(
        'vr_head_tracking_smoothing_alpha'
    )
    vr_head_tracking_max_delta = LaunchConfiguration(
        'vr_head_tracking_max_delta_per_update'
    )
    vr_head_tracking_pitch_scale = LaunchConfiguration('vr_head_tracking_pitch_scale')
    vr_head_tracking_yaw_scale = LaunchConfiguration('vr_head_tracking_yaw_scale')
    vr_head_joint1_min = LaunchConfiguration('vr_head_joint1_min')
    vr_head_joint1_max = LaunchConfiguration('vr_head_joint1_max')
    vr_head_joint2_min = LaunchConfiguration('vr_head_joint2_min')
    vr_head_joint2_max = LaunchConfiguration('vr_head_joint2_max')
    sg2_node = Node(
        package='robotis_vuer',
        executable='vr_publisher_sg2',
        name='vr_publisher_sg2',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'view_only_mode': ParameterValue(view_only_mode, value_type=bool),
            'enable_vr_head_tracking': ParameterValue(
                enable_vr_head_tracking, value_type=bool
            ),
            'enable_leader_control': ParameterValue(
                enable_leader_control, value_type=bool
            ),
            'enable_vr_robot_control': ParameterValue(
                enable_vr_robot_control, value_type=bool
            ),
            'enable_vr_image': ParameterValue(enable_vr_image, value_type=bool),
            'vr_image_left_topic': vr_image_left_topic,
            'vr_image_right_topic': vr_image_right_topic,
            'vr_image_fps': ParameterValue(vr_image_fps, value_type=float),
            'vr_head_tracking_hz': ParameterValue(
                vr_head_tracking_hz, value_type=float
            ),
            'vr_head_tracking_deadband_rad': ParameterValue(
                vr_head_tracking_deadband_rad, value_type=float
            ),
            'vr_head_tracking_smoothing_alpha': ParameterValue(
                vr_head_tracking_smoothing_alpha, value_type=float
            ),
            'vr_head_tracking_max_delta_per_update': ParameterValue(
                vr_head_tracking_max_delta, value_type=float
            ),
            'vr_head_tracking_pitch_scale': ParameterValue(
                vr_head_tracking_pitch_scale, value_type=float
            ),
            'vr_head_tracking_yaw_scale': ParameterValue(
                vr_head_tracking_yaw_scale, value_type=float
            ),
            'vr_head_joint1_min': ParameterValue(vr_head_joint1_min, value_type=float),
            'vr_head_joint1_max': ParameterValue(vr_head_joint1_max, value_type=float),
            'vr_head_joint2_min': ParameterValue(vr_head_joint2_min, value_type=float),
            'vr_head_joint2_max': ParameterValue(vr_head_joint2_max, value_type=float),
        }],
        condition=IfCondition(
            PythonExpression(["'true' if '", model, "' == 'sg2' else 'false'"])
        ),
    )
    sh5_node = Node(
        package='robotis_vuer',
        executable='vr_publisher_sh5',
        name='vr_publisher_sh5',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression(["'true' if '", model, "' == 'sh5' else 'false'"])
        ),
    )
    hx5_node = Node(
        package='robotis_vuer',
        executable='vr_publisher_hx5',
        name='vr_publisher_hx5',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression(["'true' if '", model, "' == 'hx5' else 'false'"])
        ),
    )

    return LaunchDescription([
        model_arg,
        view_only_mode_arg,
        enable_vr_head_tracking_arg,
        enable_leader_control_arg,
        enable_vr_robot_control_arg,
        enable_vr_image_arg,
        vr_image_left_topic_arg,
        vr_image_right_topic_arg,
        vr_image_fps_arg,
        vr_head_tracking_hz_arg,
        vr_head_tracking_deadband_rad_arg,
        vr_head_tracking_smoothing_alpha_arg,
        vr_head_tracking_max_delta_arg,
        vr_head_tracking_pitch_scale_arg,
        vr_head_tracking_yaw_scale_arg,
        vr_head_joint1_min_arg,
        vr_head_joint1_max_arg,
        vr_head_joint2_min_arg,
        vr_head_joint2_max_arg,
        sg2_node,
        sh5_node,
        hx5_node,
    ])
