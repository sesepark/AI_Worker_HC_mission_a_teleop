#!/usr/bin/env python3
"""Mission A 실 manipulation 서버 launch (T1) — ai_worker 컨테이너.

전제: ffw bringup(ffw_sg2_follower_ai.launch.py) + MoveIt(moveit.launch.py) + TRAC-IK 가 선행 기동되어
move_group / controllers / /joint_states 가 가용해야 한다(CONTEXT §7.3).

이 launch 는 실 manipulation 서버 노드만 띄운다. mock_manipulation_a 와는 **동시 기동 금지**
(둘 다 move_to_scan_pose / attach 계약을 제공 → 충돌). mock/실 선택은 통합 launch 레벨에서.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='manipulation',
            executable='mission_a_manipulation_server',
            name='mission_a_manipulation_server',
            output='screen',
        ),
    ])
