#!/usr/bin/env python3
"""Operator-side SG2 teleoperation launch.

Runs the RViz operator view and the LG2 leader controller on the main PC.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    description_file = LaunchConfiguration('description_file')
    leader_controller_config = LaunchConfiguration('leader_controller_config')
    leader_left_port = LaunchConfiguration('leader_left_port')
    leader_right_port = LaunchConfiguration('leader_right_port')
    start_rviz = LaunchConfiguration('start_rviz')
    start_leader = LaunchConfiguration('start_leader')
    start_mission_control = LaunchConfiguration('start_mission_control')
    start_operator_image_viewer = LaunchConfiguration('start_operator_image_viewer')
    start_operator_drive_panel = LaunchConfiguration('start_operator_drive_panel')
    start_operator_layout = LaunchConfiguration('start_operator_layout')
    start_cmd_vel_mux = LaunchConfiguration('start_cmd_vel_mux')
    start_head_trajectory_mux = LaunchConfiguration('start_head_trajectory_mux')
    joystick_cmd_vel_topic = LaunchConfiguration('joystick_cmd_vel_topic')
    keyboard_cmd_vel_topic = LaunchConfiguration('keyboard_cmd_vel_topic')
    keyboard_enabled_topic = LaunchConfiguration('keyboard_enabled_topic')
    leader_head_cmd_topic = LaunchConfiguration('leader_head_cmd_topic')
    monitor_head_cmd_topic = LaunchConfiguration('monitor_head_cmd_topic')
    head_enabled_topic = LaunchConfiguration('head_enabled_topic')
    head_mux_output_topic = LaunchConfiguration('head_mux_output_topic')
    head_mux_status_topic = LaunchConfiguration('head_mux_status_topic')
    head_pan_step_deg = LaunchConfiguration('head_pan_step_deg')
    head_tilt_step_deg = LaunchConfiguration('head_tilt_step_deg')
    mission_keyboard_drive_enabled = LaunchConfiguration('mission_keyboard_drive_enabled')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    cmd_vel_mux_status_topic = LaunchConfiguration('cmd_vel_mux_status_topic')
    keyboard_linear_x_mps = LaunchConfiguration('keyboard_linear_x_mps')
    keyboard_linear_y_mps = LaunchConfiguration('keyboard_linear_y_mps')
    keyboard_angular_z_radps = LaunchConfiguration('keyboard_angular_z_radps')
    default_drive_speed_scale = LaunchConfiguration('default_drive_speed_scale')
    boost_drive_speed_scale = LaunchConfiguration('boost_drive_speed_scale')
    keyboard_publish_hz = LaunchConfiguration('keyboard_publish_hz')
    keyboard_key_timeout_s = LaunchConfiguration('keyboard_key_timeout_s')
    mouse_hold_timeout_s = LaunchConfiguration('mouse_hold_timeout_s')
    mouse_max_hold_s = LaunchConfiguration('mouse_max_hold_s')
    keyboard_stale_timeout_s = LaunchConfiguration('keyboard_stale_timeout_s')
    joystick_stale_timeout_s = LaunchConfiguration('joystick_stale_timeout_s')
    operator_ok_topic = LaunchConfiguration('operator_ok_topic')
    ok_overlay_duration_s = LaunchConfiguration('ok_overlay_duration_s')
    operator_image_viewer_layout_store_path = LaunchConfiguration(
        'operator_image_viewer_layout_store_path')
    operator_image_viewer_canvas_width = LaunchConfiguration(
        'operator_image_viewer_canvas_width')
    operator_image_viewer_canvas_height = LaunchConfiguration(
        'operator_image_viewer_canvas_height')
    operator_image_viewer_auto_canvas_size = LaunchConfiguration(
        'operator_image_viewer_auto_canvas_size')
    operator_image_viewer_follow_window_size = LaunchConfiguration(
        'operator_image_viewer_follow_window_size')
    operator_image_viewer_show_toolbar = LaunchConfiguration(
        'operator_image_viewer_show_toolbar')
    camera_perf_topic = LaunchConfiguration('camera_perf_topic')
    mission_profiles_config = LaunchConfiguration('mission_profiles_config')
    operator_screen_layout_config = LaunchConfiguration('operator_screen_layout_config')
    operator_layout_action = LaunchConfiguration('operator_layout_action')
    operator_layout_store_path = LaunchConfiguration('operator_layout_store_path')
    leader_namespace = 'leader'
    leader_robot_description_topic = '/leader/robot_description'

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'),
        'config',
        'ffw_lg2_leader',
        leader_controller_config,
    ])

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('ffw_description'),
                'urdf',
                'ffw_lg2_leader',
                description_file,
            ]),
            ' ',
            'leader_left_port:=',
            leader_left_port,
            ' ',
            'leader_right_port:=',
            leader_right_port,
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    leader_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=leader_namespace,
        output='both',
        parameters=[
            robot_description,
            {
                'frame_prefix': 'leader_',
                'use_robot_description_topic': False,
            },
        ],
        remappings=[
            ('robot_description', leader_robot_description_topic),
        ],
        condition=IfCondition(start_leader),
    )

    leader_control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=leader_namespace,
        parameters=[robot_description, robot_controllers],
        remappings=[
            ('~/robot_description', leader_robot_description_topic),
            ('robot_description', leader_robot_description_topic),
            ('/cmd_vel', joystick_cmd_vel_topic),
            ('cmd_vel', joystick_cmd_vel_topic),
        ],
        output='both',
        condition=IfCondition(start_leader),
    )

    leader_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=leader_namespace,
        arguments=[
            'joint_trajectory_command_broadcaster',
            'spring_actuator_controller_left',
            'spring_actuator_controller_right',
            'joystick_controller',
            'joint_state_broadcaster',
            '--controller-manager',
            '/leader/controller_manager',
        ],
        parameters=[robot_description],
        condition=IfCondition(start_leader),
    )

    mission_control = Node(
        package='ffw_teleop',
        executable='mission_mode_manager',
        name='mission_mode_manager',
        output='screen',
        parameters=[{
            'profiles_config': mission_profiles_config,
            'keyboard_drive_ui_enabled': mission_keyboard_drive_enabled,
            'keyboard_cmd_vel_topic': keyboard_cmd_vel_topic,
            'keyboard_enabled_topic': keyboard_enabled_topic,
            'keyboard_linear_x_mps': keyboard_linear_x_mps,
            'keyboard_linear_y_mps': keyboard_linear_y_mps,
            'keyboard_angular_z_radps': keyboard_angular_z_radps,
            'keyboard_publish_hz': keyboard_publish_hz,
            'keyboard_key_timeout_s': keyboard_key_timeout_s,
            'operator_ok_topic': operator_ok_topic,
            'ok_overlay_duration_s': ok_overlay_duration_s,
        }],
        condition=IfCondition(start_mission_control),
    )

    cmd_vel_mux = Node(
        package='ffw_teleop',
        executable='teleop_cmd_vel_mux',
        name='teleop_cmd_vel_mux',
        output='screen',
        parameters=[{
            'joystick_cmd_vel_topic': joystick_cmd_vel_topic,
            'keyboard_cmd_vel_topic': keyboard_cmd_vel_topic,
            'keyboard_enabled_topic': keyboard_enabled_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'status_topic': cmd_vel_mux_status_topic,
            'publish_hz': keyboard_publish_hz,
            'keyboard_stale_timeout_s': keyboard_stale_timeout_s,
            'joystick_stale_timeout_s': joystick_stale_timeout_s,
        }],
        condition=IfCondition(start_cmd_vel_mux),
    )

    head_trajectory_mux = Node(
        package='ffw_teleop',
        executable='teleop_head_trajectory_mux',
        name='teleop_head_trajectory_mux',
        output='screen',
        parameters=[{
            'leader_head_topic': leader_head_cmd_topic,
            'monitor_head_topic': monitor_head_cmd_topic,
            'monitor_enabled_topic': head_enabled_topic,
            'output_head_topic': head_mux_output_topic,
            'status_topic': head_mux_status_topic,
            'publish_hz': keyboard_publish_hz,
        }],
        condition=IfCondition(start_head_trajectory_mux),
    )

    operator_image_viewer = Node(
        package='ffw_teleop',
        executable='operator_image_viewer',
        name='operator_image_viewer',
        output='screen',
        parameters=[{
            'window_title': 'Teleop Image Viewer',
            'tile_width': 640,
            'tile_height': 360,
            'columns': 2,
            'canvas_width': operator_image_viewer_canvas_width,
            'canvas_height': operator_image_viewer_canvas_height,
            'auto_canvas_size': operator_image_viewer_auto_canvas_size,
            'follow_window_size': operator_image_viewer_follow_window_size,
            'layout_store_path': operator_image_viewer_layout_store_path,
            'show_toolbar': operator_image_viewer_show_toolbar,
            'camera_perf_topic': camera_perf_topic,
            'window_x': 1520,
            'window_y': 40,
        }],
        condition=IfCondition(start_operator_image_viewer),
    )

    operator_drive_panel = Node(
        package='ffw_teleop',
        executable='operator_drive_panel',
        name='operator_drive_panel',
        output='screen',
        parameters=[{
            'window_title': 'Teleop Drive Control',
            'window_width': 560,
            'window_height': 980,
            'window_x': 80,
            'window_y': 360,
            'keyboard_cmd_vel_topic': keyboard_cmd_vel_topic,
            'keyboard_enabled_topic': keyboard_enabled_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'cmd_vel_mux_status_topic': cmd_vel_mux_status_topic,
            'monitor_head_cmd_topic': monitor_head_cmd_topic,
            'head_enabled_topic': head_enabled_topic,
            'head_mux_status_topic': head_mux_status_topic,
            'joint_state_topic': '/joint_states',
            'head_pan_step_deg': head_pan_step_deg,
            'head_tilt_step_deg': head_tilt_step_deg,
            'keyboard_linear_x_mps': keyboard_linear_x_mps,
            'keyboard_linear_y_mps': keyboard_linear_y_mps,
            'keyboard_angular_z_radps': keyboard_angular_z_radps,
            'default_drive_speed_scale': default_drive_speed_scale,
            'boost_drive_speed_scale': boost_drive_speed_scale,
            'click_jog_duration_s': keyboard_key_timeout_s,
            'mouse_hold_timeout_s': mouse_hold_timeout_s,
            'mouse_max_hold_s': mouse_max_hold_s,
            'operator_ok_topic': operator_ok_topic,
            'ok_overlay_duration_s': ok_overlay_duration_s,
            'layout_command_topic': '/teleop/operator_layout/command',
            'layout_status_topic': '/teleop/operator_layout/status',
        }],
        condition=IfCondition(start_operator_drive_panel),
    )

    operator_layout = Node(
        package='ffw_teleop',
        executable='operator_layout_manager',
        name='operator_layout_manager',
        output='screen',
        parameters=[{
            'layout_config': operator_screen_layout_config,
            'action': operator_layout_action,
            'layout_store_path': operator_layout_store_path,
            'initial_delay_s': 3.0,
            'retry_count': 24,
            'retry_interval_s': 0.5,
        }],
        condition=IfCondition(start_operator_layout),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_lg2_leader.urdf.xacro'),
        DeclareLaunchArgument(
            'leader_controller_config',
            default_value='ffw_lg2_leader_ai_hardware_controller_teleop.yaml'),
        DeclareLaunchArgument('leader_left_port', default_value='/dev/left_leader'),
        DeclareLaunchArgument('leader_right_port', default_value='/dev/right_leader'),
        DeclareLaunchArgument('start_leader', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_gl_mode', default_value='native'),
        DeclareLaunchArgument('start_mission_control', default_value='false'),
        DeclareLaunchArgument('start_operator_image_viewer', default_value='true'),
        DeclareLaunchArgument('start_operator_drive_panel', default_value='true'),
        DeclareLaunchArgument('mission_keyboard_drive_enabled', default_value='false'),
        DeclareLaunchArgument(
            'start_cmd_vel_mux',
            default_value='true'),
        DeclareLaunchArgument('start_head_trajectory_mux', default_value='true'),
        DeclareLaunchArgument('joystick_cmd_vel_topic', default_value='/teleop/joystick_cmd_vel'),
        DeclareLaunchArgument('keyboard_cmd_vel_topic', default_value='/teleop/keyboard_cmd_vel'),
        DeclareLaunchArgument(
            'keyboard_enabled_topic', default_value='/teleop/keyboard_drive/enabled'),
        DeclareLaunchArgument('leader_head_cmd_topic', default_value='/teleop/leader_head_cmd'),
        DeclareLaunchArgument('monitor_head_cmd_topic', default_value='/teleop/monitor_head_cmd'),
        DeclareLaunchArgument('head_enabled_topic', default_value='/teleop/head_drive/enabled'),
        DeclareLaunchArgument(
            'head_mux_output_topic',
            default_value='/leader/joystick_controller_left/joint_trajectory'),
        DeclareLaunchArgument('head_mux_status_topic', default_value='/teleop/head_mux/status'),
        DeclareLaunchArgument('head_pan_step_deg', default_value='3.0'),
        DeclareLaunchArgument('head_tilt_step_deg', default_value='3.0'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'cmd_vel_mux_status_topic', default_value='/teleop/cmd_vel_mux/status'),
        DeclareLaunchArgument('keyboard_linear_x_mps', default_value='0.24'),
        DeclareLaunchArgument('keyboard_linear_y_mps', default_value='0.24'),
        DeclareLaunchArgument('keyboard_angular_z_radps', default_value='0.40'),
        DeclareLaunchArgument('default_drive_speed_scale', default_value='0.5'),
        DeclareLaunchArgument('boost_drive_speed_scale', default_value='1.0'),
        DeclareLaunchArgument('keyboard_publish_hz', default_value='30.0'),
        DeclareLaunchArgument('keyboard_key_timeout_s', default_value='0.35'),
        DeclareLaunchArgument('mouse_hold_timeout_s', default_value='0.75'),
        DeclareLaunchArgument('mouse_max_hold_s', default_value='8.0'),
        DeclareLaunchArgument('keyboard_stale_timeout_s', default_value='0.20'),
        DeclareLaunchArgument('joystick_stale_timeout_s', default_value='0.30'),
        DeclareLaunchArgument('operator_ok_topic', default_value='/teleop/operator_ok'),
        DeclareLaunchArgument('ok_overlay_duration_s', default_value='3.0'),
        DeclareLaunchArgument(
            'operator_image_viewer_layout_store_path',
            default_value='~/.config/ffw_teleop/operator_image_viewer_layout.json'),
        DeclareLaunchArgument('operator_image_viewer_canvas_width', default_value='0'),
        DeclareLaunchArgument('operator_image_viewer_canvas_height', default_value='0'),
        DeclareLaunchArgument('operator_image_viewer_auto_canvas_size', default_value='true'),
        DeclareLaunchArgument('operator_image_viewer_follow_window_size', default_value='true'),
        DeclareLaunchArgument('operator_image_viewer_show_toolbar', default_value='true'),
        DeclareLaunchArgument('camera_perf_topic', default_value='/teleop/camera_perf'),
        DeclareLaunchArgument('start_operator_layout', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'rviz',
                'teleop_operator.rviz',
            ])),
        DeclareLaunchArgument(
            'mission_profiles_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'config',
                'mission_profiles.yaml',
            ])),
        DeclareLaunchArgument(
            'operator_screen_layout_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'config',
                'operator_screen_layout.yaml',
            ])),
        DeclareLaunchArgument('operator_layout_action', default_value='server'),
        DeclareLaunchArgument(
            'operator_layout_store_path',
            default_value='~/.config/ffw_teleop/operator_screen_layout.json'),
        LogInfo(msg=['LG2 Leader controller config: ', leader_controller_config],
                condition=IfCondition(start_leader)),
        LogInfo(
            msg=[
                'LG2 Leader base route: joystick_controller -> ',
                joystick_cmd_vel_topic,
                ' -> teleop_cmd_vel_mux -> ',
                cmd_vel_topic,
            ],
            condition=IfCondition(start_cmd_vel_mux),
        ),
        LogInfo(msg=['LG2 Leader robot_description topic: ', leader_robot_description_topic],
                condition=IfCondition(start_leader)),
        LogInfo(msg=['LG2 Leader left port: ', leader_left_port],
                condition=IfCondition(start_leader)),
        LogInfo(msg=['LG2 Leader right port: ', leader_right_port],
                condition=IfCondition(start_leader)),
        leader_robot_state_publisher,
        leader_control,
        leader_spawner,
        OpaqueFunction(function=make_rviz_node),
        TimerAction(period=1.0, actions=[cmd_vel_mux]),
        TimerAction(period=1.2, actions=[head_trajectory_mux]),
        TimerAction(period=1.5, actions=[mission_control]),
        TimerAction(period=2.5, actions=[operator_image_viewer]),
        TimerAction(period=3.0, actions=[operator_drive_panel]),
        TimerAction(period=4.0, actions=[operator_layout]),
    ])
