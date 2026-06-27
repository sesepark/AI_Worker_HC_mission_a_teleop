"""
Mission C capture pose 이동 테스트.
y 좌표 기준으로 팔을 선택해 capture pose로 이동.

y >= 0 → 왼팔, y < 0 → 오른팔

TODO: 실물 로봇 텔레옵으로 joint 값 확인 후 갱신

실행:
  ros2 run manipulation test_capture_to_pick_c
"""

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient
from manipulation.robot_interface.planning_scene import setup_zone_c_table, remove_zone_c_table
from manipulation.skill_primitives.mission_c_arm_selector import select_arm
from manipulation.tests.MissionC.test_pick_C import CENTER_Y

CAPTURE_JOINTS_R = [-0.514537, -1.079939,  0.611448, -2.036518, -2.695534,  1.082374, -1.580207]
CAPTURE_JOINTS_L = [-0.514537,  1.079939, -0.611448, -2.036518,  2.695534,  1.082374,  1.580207]


def main():
    rclpy.init()
    node   = Node('test_capture_to_pick_c')
    log    = node.get_logger()
    client = MoveItClient(node)

    setup_zone_c_table(client)

    # ── place 후 위치 복귀 ───────────────────────────────────────────
    # pseudo: nav_y_offset = test_place_C.get_last_nav_offset()
    #         if nav_y_offset != 0.0:
    #             direction = 'right' if nav_y_offset < 0 else 'left'
    #             # ↓ 여기서 navigation으로 |nav_y_offset|만큼 반대 방향 이동
    #             log.info(f'[capture_pose] {direction}으로 복귀 이동 완료')
    # ─────────────────────────────────────────────────────────────────

    arm            = select_arm(CENTER_Y)
    capture_joints = CAPTURE_JOINTS_L if arm.value == 'left' else CAPTURE_JOINTS_R
    log.info(f'[capture_pose] 선택된 팔: {arm.value} (CENTER_Y={CENTER_Y})')
    log.info(f'[capture_pose] joints={[f"{v:.3f}" for v in capture_joints]}')

    r = client.move_to_joints(capture_joints, arm=arm, velocity=0.2, acceleration=0.2)
    log.info(f'[capture_pose] 결과: {r.value}')

    remove_zone_c_table(client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
