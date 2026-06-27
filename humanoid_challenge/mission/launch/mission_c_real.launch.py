#!/usr/bin/env python3
"""Mission C 실 통합 launch (humanoid_challenge 측) — 실 perception + mission_c + peg 공급원.

`mission_a_real.launch.py` 의 C 대응본. 한 launch 그룹(동일 기동 윈도우, 컨테이너 DDS 디스커버리 안정)으로:
  · 실 perception (`perception_live.launch.py`: detector+tray_manage+wrist_planner+static TF+place_pose_valid)
    → pick 타깃(/perception/wrist/target_one_pose) + task_list 공급.
  · peg 공급원 (`pipe_source`): preset(사전측정, 학습 전) | model(실 head_pipe, 학습 후) — mission_c.launch.py 내부.
  · mission_c (FSM, use_mocks=false, nav_mode 단계화).

**실 manipulation 서버(dual-arm + peg 삽입)는 ai_worker 컨테이너에서 별도 기동**(동일 DDS 도메인):
  ai_worker$  ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py     # bringup
  ai_worker$  ros2 launch ffw_moveit_config moveit.launch.py            # MoveIt
  ai_worker$  ros2 launch manipulation mission_c_manip.launch.py        # 실 C manip 서버(dual-arm)
  (공통 env: ROS_DOMAIN_ID=30, ROS_LOCALHOST_ONLY=0, ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET)
  dry-run 시험: 본 launch 의 insert_dry_run:=true (FSM) + ai_worker mission_c_manip.launch.py
    insert_dry_run:=true (실 manip 서버) 를 **양쪽 함께** 설정해 정밀 삽입 없이 전 사이클 루프 검증.

nav: `nav_mode`(기본 stub). **실 MoveBaseLateral(nav_mode:=service)** 시 실 nav 서버는 /cmd_vel·/odom 가
있는 **로봇 PC에서 별도 기동**: robot PC$ ros2 launch mission move_base_lateral.launch.py
FSM 은 C3_MOVE_TO_PEG/C3_RETURN 에서 좌/우 측방 이동(MoveBaseLateral, A 와 동일 재사용).

peg 좌표(학습 전 preset, 실측 R4): pipe_x/pipe_z 인자로 보정 가능. 학습 완료 시 `pipe_source:=model`.
실 삽입 모션은 사용자 감독(저속·E-stop).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:
    mission_pkg = get_package_share_directory('mission')
    perception_pkg = get_package_share_directory('perception')
    mission_c_launch = os.path.join(mission_pkg, 'launch', 'mission_c.launch.py')
    perception_live = os.path.join(perception_pkg, 'launch', 'perception_live.launch.py')
    monitor_ocr_a = os.path.join(perception_pkg, 'launch', 'monitor_ocr_a.launch.py')

    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('use_place_pose_check', default_value='false',
                              description='C3 peg 삽입 위치 유효성 게이트(실 준비 후 true)'),
        DeclareLaunchArgument('insert_dry_run', default_value='false',
                              description='FSM C3_INSERT dry-run(항상 통과). 실 삽입 stall 우회 시험용. '
                                          'ai_worker mission_c_manip 의 insert_dry_run 과 함께 설정'),
        DeclareLaunchArgument('insert_dry_run_grace_sec', default_value='2.0',
                              description='dry-run 해제 강제확정 grace(초)'),
        DeclareLaunchArgument('use_monitor_ocr', default_value='true',
                              description='false=OCR task_list 미사용, pick_positions 기반 수동 task_list 사용'),
        DeclareLaunchArgument('mock_monitor_ocr', default_value='true',
                              description='true=OCR 노드 대신 mock task_list. false=monitor_ocr_a 노드 사용'),
        DeclareLaunchArgument('nav_mode', default_value='stub',
                              description='{stub|service} — service 시 실 move_base_lateral(로봇 PC 별도 기동)'),
        DeclareLaunchArgument('nav_service_wait_sec', default_value='10.0'),
        DeclareLaunchArgument('base_shift_mm', default_value='675.0'),
        DeclareLaunchArgument('arm_mode', default_value='right',
                              description='{right|left|auto} — 현 단계 우완 단일팔(기본). '
                                          'ai_worker mission_c_manip 의 arm_mode 와 일치시킬 것'),
        DeclareLaunchArgument('pipe_source', default_value='preset',
                              description='{preset|model} — peg 중심 공급(학습 전 preset)'),
        DeclareLaunchArgument('pipe_x', default_value='0.40'),
        DeclareLaunchArgument('pipe_z', default_value='0.90'),
        # base_seq: 카메라 픽 + 너트 A/B 정렬 / pipe3 기준 place 로 베이스 측방 이동.
        #   검증용: insert_dry_run:=true 면 place 는 dry(베이스 정렬까지만 확인).
        #   nav_mode:=service + 로봇 PC move_base_lateral 서버 필수(베이스 실제 이동).
        DeclareLaunchArgument('base_seq_enable', default_value='false',
                              description='너트 A/B 정렬 픽 + pipe3 기준 place 시퀀스. '
                                          'true 면 nav_mode:=service 필수'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='legacy. base_seq 너트 pick은 항상 perception/camera 사용'),
        DeclareLaunchArgument('use_pipe_camera', default_value='false',
                              description='false=manual pipe3 hardcoded place, true=perception pipe center place'),
        DeclareLaunchArgument('nut_pitch_mm', default_value='150.0',
                              description='legacy. base_seq pick A/B 모드에서는 미사용'),
        DeclareLaunchArgument('place_forward_mm', default_value='100.0',
                              description='legacy. 현재 base_seq place에서는 미사용'),
        DeclareLaunchArgument('pick_positions', default_value='',
                              description="픽 순서를 너트 위치(1~5,왼→오)로 지정 예:'4-5-3-1'"),
    ]

    effective_mock_monitor_ocr = PythonExpression([
        "'true' if '", lc('use_monitor_ocr'), "' == 'true' and '",
        lc('mock_monitor_ocr'), "' == 'true' else 'false'",
    ])
    use_real_monitor_ocr = IfCondition(PythonExpression([
        "'", lc('use_monitor_ocr'), "' == 'true' and '",
        lc('mock_monitor_ocr'), "' == 'false'",
    ]))

    monitor_ocr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(monitor_ocr_a),
        condition=use_real_monitor_ocr,
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_live),
        launch_arguments={'mock_monitor_ocr': effective_mock_monitor_ocr}.items(),
    )
    # mission_c.launch.py 를 use_mocks=false 로 포함 → FSM + peg 공급원(preset/model). mock 미기동.
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mission_c_launch),
        launch_arguments={
            'use_mocks': 'false',           # 실 manip(ai_worker)·실 perception 사용 → mock 미기동
            'nav_mode': lc('nav_mode'),
            'nav_service_wait_sec': lc('nav_service_wait_sec'),
            'base_shift_mm': lc('base_shift_mm'),
            'arm_mode': lc('arm_mode'),
            'use_place_pose_check': lc('use_place_pose_check'),
            'insert_dry_run': lc('insert_dry_run'),
            'insert_dry_run_grace_sec': lc('insert_dry_run_grace_sec'),
            'pipe_source': lc('pipe_source'),
            'pipe_x': lc('pipe_x'),
            'pipe_z': lc('pipe_z'),
            'base_seq_enable': lc('base_seq_enable'),
            'use_monitor_ocr': lc('use_monitor_ocr'),
            'use_camera': lc('use_camera'),
            'use_pipe_camera': lc('use_pipe_camera'),
            'nut_pitch_mm': lc('nut_pitch_mm'),
            'place_forward_mm': lc('place_forward_mm'),
            'pick_positions': lc('pick_positions'),
        }.items(),
    )
    return LaunchDescription(args + [monitor_ocr, perception, mission])
