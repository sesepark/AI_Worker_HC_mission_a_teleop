"""Mission B 통합 실행 (stage=all) — Ⓑ-1 → Ⓑ-2 → Ⓑ-3 연속.

기본은 무인 연속 실행(auto_chain:=true) — 단계 경계(B1→B2, B2→B3, 다음 박스)를
조종자 확인 없이 자동 진행한다. 단계마다 확인을 받으려면 auto_chain:=false 로 주고:
    ros2 topic pub --once /mission_b/operator_event std_msgs/msg/String "{data: proceed}"
로 각 경계를 진행한다.

선행조건(실로봇): bringup + MoveIt + manipulation 빌드.
  include_nav_coordinator:=true(기본) 면 ffw_mission_b_nav 코디네이터도 함께 기동하며,
  nav 거리/LiDAR 를 아래 nav_* 인자로 덮어쓸 수 있다(미지정 시 실로봇 검증값=yaml).
  (로봇 PC 의 bringup/MoveIt 은 별도 — docs/MISSION_B_INTEGRATION_RUN.md 참고)

  # 실전(실거리·LiDAR on, 박스 4개, 무인 자동 진행)
  ros2 launch mission mission_b.launch.py
  # 단계마다 조종자 확인 받기
  ros2 launch mission mission_b.launch.py auto_chain:=false
  # 2단계 간이(짧은 거리·LiDAR off·박스 없이) — 한 명령
  ros2 launch mission mission_b.launch.py max_boxes:=1 \
      nav_backward_distance:=0.2 nav_right_distance:=0.5 nav_forward_distance:=0.1 \
      nav_enable_lidar:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    include_nav = LaunchConfiguration('include_nav_coordinator')

    # LaunchConfiguration 은 문자열로 치환되므로, 비-문자열 파라미터는 value_type 으로
    # 명시 캐스팅해야 노드 declare_parameter 의 타입과 일치한다(미적용 시 startup 예외).
    def pv(name, t):
        return ParameterValue(LaunchConfiguration(name), value_type=t)

    # 벤더링한 nav 패키지의 실로봇 검증값 yaml — nav_* 인자로 그 위를 덮어쓴다.
    nav_config = PathJoinSubstitution([
        FindPackageShare('ffw_mission_b_nav'),
        'config', 'mission_b_real_robot.yaml',
    ])

    # nav 코디네이터를 직접 Node 로 기동(패키지 코드 무변경, 파라미터만 레이어링).
    nav_node = Node(
        package='ffw_mission_b_nav', executable='sg2_mission_b_system_nav',
        name='sg2_mission_b_system_nav', output='screen',
        condition=IfCondition(include_nav),
        parameters=[
            nav_config,
            {
                'backward_distance': pv('nav_backward_distance', float),
                'right_distance': pv('nav_right_distance', float),
                'forward_distance': pv('nav_forward_distance', float),
                # 이동 속도(직진/횡)는 기본값을 yaml(0.12/0.20)보다 약간 올린다.
                # 책상 접근(b_approach)·LiDAR 정렬 최소속도는 정밀도 위해 yaml 유지.
                'linear_speed': pv('nav_linear_speed', float),
                'lateral_speed': pv('nav_lateral_speed', float),
                'forward_speed': pv('nav_forward_speed', float),
                # 간이 테스트는 LiDAR 정렬을 한 번에 끈다(테이블 불필요).
                'enable_lidar_alignment': pv('nav_enable_lidar', bool),
                'enable_return_a_lidar_alignment': pv('nav_enable_lidar', bool),
                'use_sim_time': False,
            },
        ],
    )

    fsm = Node(
        package='mission', executable='mission_b', name='mission_b', output='screen',
        parameters=[{
            'stage': 'all',
            'auto_chain': pv('auto_chain', bool),
            'proceed_event': LaunchConfiguration('proceed_event'),
            'max_boxes': pv('max_boxes', int),
            'pick_cmd': LaunchConfiguration('pick_cmd'),
            'place_cmd': LaunchConfiguration('place_cmd'),
            'stop_line_dwell_sec': pv('stop_line_dwell_sec', float),
        }],
    )

    # 조종자 시각 모니터 창(PyQt5). DISPLAY 가 있어야 뜬다(헤드리스면 show_monitor:=false).
    monitor = Node(
        package='mission', executable='mission_b_monitor', name='mission_b_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_monitor')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('include_nav_coordinator', default_value='true'),
        DeclareLaunchArgument('auto_chain', default_value='true'),
        DeclareLaunchArgument('proceed_event', default_value='proceed'),
        DeclareLaunchArgument('max_boxes', default_value='4'),
        DeclareLaunchArgument('pick_cmd',
                              default_value='ros2 run manipulation test_dual_pick'),
        DeclareLaunchArgument('place_cmd',
                              default_value='ros2 run manipulation test_dual_place'),
        DeclareLaunchArgument('stop_line_dwell_sec', default_value='1.5'),
        # --- nav 거리/LiDAR (기본값 = 실로봇 검증값; 간이 단계에서 덮어쓰기) ---
        DeclareLaunchArgument('nav_backward_distance', default_value='1.00'),
        DeclareLaunchArgument('nav_right_distance', default_value='3.80'),
        DeclareLaunchArgument('nav_forward_distance', default_value='0.30'),
        DeclareLaunchArgument('nav_enable_lidar', default_value='true'),
        # --- nav 이동 속도(m/s). yaml 기본(0.12/0.20/0.12)보다 약간 상향 ---
        DeclareLaunchArgument('nav_linear_speed', default_value='0.18'),
        DeclareLaunchArgument('nav_lateral_speed', default_value='0.28'),
        DeclareLaunchArgument('nav_forward_speed', default_value='0.18'),
        DeclareLaunchArgument('show_monitor', default_value='true'),
        nav_node,
        fsm,
        monitor,
    ])
