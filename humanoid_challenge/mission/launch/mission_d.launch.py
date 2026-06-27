#!/usr/bin/env python3
"""Mission D launch 파일.

기본값은 Mission D System과 test_env gate를 함께 띄운다. mock smoke 검증에서는
use_test_env=false로 끄고, 실제 로봇 통합에서는 use_mocks=false로 두고 동일한
topic/service/action 계약을 제공하는 팀별 노드를 별도로 실행한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('use_mocks', default_value='true'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mission'), 'config', 'mission_d.yaml',
            ]),
            description='Mission D parameter YAML file'),
        DeclareLaunchArgument(
            'mock_navigation_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mission'), 'config', 'mock_navigation_d.yaml',
            ]),
            description='Navigation-team mock parameter YAML file'),
        DeclareLaunchArgument(
            'mock_perception_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mission'), 'config', 'mock_perception_d.yaml',
            ]),
            description='Perception-team mock parameter YAML file'),
        DeclareLaunchArgument(
            'mock_manipulation_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mission'), 'config', 'mock_manipulation_d.yaml',
            ]),
            description='Manipulation-team mock parameter YAML file'),
        DeclareLaunchArgument('use_test_env', default_value='true'),
        DeclareLaunchArgument('test_env_required', default_value='true'),
        DeclareLaunchArgument('test_env_package', default_value='manipulation'),
        DeclareLaunchArgument('test_env_executable', default_value='test_env'),
        DeclareLaunchArgument(
            'setup_environment_service_name', default_value='/mission_d/setup_environment'),
        DeclareLaunchArgument('test_env_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('nav_service_name', default_value='move_base_relative'),
        DeclareLaunchArgument('nav_service_wait_sec', default_value='10.0'),
        DeclareLaunchArgument('manipulation_action_name', default_value='/mission_d/manipulation'),
        DeclareLaunchArgument('mock_scenario', default_value='normal'),
        DeclareLaunchArgument('confidence', default_value='0.90'),
        DeclareLaunchArgument('plan_success', default_value='true'),
        DeclareLaunchArgument('travel_sec', default_value='1.0'),
        DeclareLaunchArgument('fail_arrive', default_value='false'),
        DeclareLaunchArgument('fail_within_expected_range', default_value='false'),
        DeclareLaunchArgument('fail_on_call_indices_json', default_value='[]'),
        DeclareLaunchArgument('fail_on_path_ids_json', default_value='[]'),
        DeclareLaunchArgument('actual_translation_scale', default_value='1.0'),
        DeclareLaunchArgument('actual_yaw_scale', default_value='1.0'),
        DeclareLaunchArgument('action_sec', default_value='1.0'),
        DeclareLaunchArgument('fail_wheel_attempts', default_value='0'),
        DeclareLaunchArgument('fail_bolt_detection_attempts', default_value='0'),
        DeclareLaunchArgument('fail_drill_detection_attempts', default_value='0'),
        DeclareLaunchArgument('fail_fixture_detection_attempts', default_value='0'),
        DeclareLaunchArgument('fail_wheel_grasp_attempts', default_value='0'),
        DeclareLaunchArgument('fail_bolt_grasp_attempts', default_value='0'),
        DeclareLaunchArgument('fail_drill_grasp_attempts', default_value='0'),
        DeclareLaunchArgument('fail_bolt_insert_attempts', default_value='0'),
        DeclareLaunchArgument('fail_fasten_attempts', default_value='0'),
    ]

    lc = LaunchConfiguration
    use_mocks = lc('use_mocks')
    use_test_env = lc('use_test_env')
    config_file = lc('config_file')
    mock_navigation_config_file = lc('mock_navigation_config_file')
    mock_perception_config_file = lc('mock_perception_config_file')
    mock_manipulation_config_file = lc('mock_manipulation_config_file')

    mission_d = Node(
        package='mission',
        executable='mission_d',
        name='mission_d',
        output='screen',
        parameters=[config_file, {
            'nav_service_name': lc('nav_service_name'),
            'nav_service_wait_sec': lc('nav_service_wait_sec'),
            'manipulation_action_name': lc('manipulation_action_name'),
            'use_test_env': lc('use_test_env'),
            'test_env_required': lc('test_env_required'),
            'setup_environment_service_name': lc('setup_environment_service_name'),
            'test_env_timeout_sec': lc('test_env_timeout_sec'),
        }],
    )

    test_env = Node(
        package=lc('test_env_package'),
        executable=lc('test_env_executable'),
        name='mission_d_test_env',
        output='screen',
        condition=IfCondition(use_test_env),
        parameters=[{
            'setup_environment_service_name': lc('setup_environment_service_name'),
        }],
    )

    mock_perception = Node(
        package='mission',
        executable='mock_perception_d',
        name='mock_perception_d',
        output='screen',
        condition=IfCondition(use_mocks),
        parameters=[mock_perception_config_file, {
            'scenario': lc('mock_scenario'),
            'confidence': lc('confidence'),
            'plan_success': lc('plan_success'),
            'fail_wheel_attempts': lc('fail_wheel_attempts'),
            'fail_bolt_detection_attempts': lc('fail_bolt_detection_attempts'),
            'fail_drill_detection_attempts': lc('fail_drill_detection_attempts'),
            'fail_fixture_detection_attempts': lc('fail_fixture_detection_attempts'),
        }],
    )

    mock_navigation = Node(
        package='mission',
        executable='mock_navigation_d',
        name='mock_navigation_d',
        output='screen',
        condition=IfCondition(use_mocks),
        parameters=[mock_navigation_config_file, {
            'nav_service_name': lc('nav_service_name'),
            'scenario': lc('mock_scenario'),
            'travel_sec': lc('travel_sec'),
            'fail_arrive': lc('fail_arrive'),
            'fail_within_expected_range': lc('fail_within_expected_range'),
            'fail_on_call_indices_json': ParameterValue(
                lc('fail_on_call_indices_json'), value_type=str),
            'fail_on_path_ids_json': ParameterValue(
                lc('fail_on_path_ids_json'), value_type=str),
            'actual_translation_scale': lc('actual_translation_scale'),
            'actual_yaw_scale': lc('actual_yaw_scale'),
        }],
    )

    mock_manipulation = Node(
        package='mission',
        executable='mock_manipulation_d',
        name='mock_manipulation_d',
        output='screen',
        condition=IfCondition(use_mocks),
        parameters=[mock_manipulation_config_file, {
            'manipulation_action_name': lc('manipulation_action_name'),
            'scenario': lc('mock_scenario'),
            'action_sec': lc('action_sec'),
            'fail_wheel_grasp_attempts': lc('fail_wheel_grasp_attempts'),
            'fail_bolt_grasp_attempts': lc('fail_bolt_grasp_attempts'),
            'fail_drill_grasp_attempts': lc('fail_drill_grasp_attempts'),
            'fail_bolt_insert_attempts': lc('fail_bolt_insert_attempts'),
            'fail_fasten_attempts': lc('fail_fasten_attempts'),
        }],
    )

    return LaunchDescription(args + [
        test_env,
        mission_d,
        mock_perception,
        mock_navigation,
        mock_manipulation,
    ])
