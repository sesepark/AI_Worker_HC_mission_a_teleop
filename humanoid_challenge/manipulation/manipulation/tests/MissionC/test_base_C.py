"""
Mission C base 자세 테스트.
오른팔 → 왼팔 순서로 capture pose로 이동.

실행:
  ros2 run manipulation test_base_c
"""

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient, Arm, MoveResult
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import setup_zone_c_table, remove_zone_c_table

# test_capture_to_pick_C.py 와 동일한 joint 값
CAPTURE_JOINTS_R = [-0.514537, -1.079939,  0.611448, -2.036518, -2.695534,  1.082374, -1.580207]
CAPTURE_JOINTS_L = [-0.514537,  1.079939, -0.611448, -2.036518,  2.695534,  1.082374,  1.580207]


def main():
    rclpy.init()
    node   = Node('test_base_c')
    log    = node.get_logger()
    client  = MoveItClient(node)
    gripper = GripperInterface(node)

    setup_zone_c_table(client)

    log.info('[base_c] Step 0: 양쪽 그리퍼 열기')
    gripper.open_to('right', 0.5)
    gripper.wait_until_executed()
    gripper.open_to('left', 0.5)
    gripper.wait_until_executed()

    log.info('[base_c] Step 1: 오른팔 capture pose 이동')
    r = client.move_to_joints(CAPTURE_JOINTS_R, arm=Arm.RIGHT, velocity=0.2, acceleration=0.2)
    log.info(f'[base_c] 오른팔 결과: {r.value}')
    if r != MoveResult.SUCCEEDED:
        log.error('[base_c] 오른팔 실패 — 종료')
        node.destroy_node()
        rclpy.shutdown()
        return

    log.info('[base_c] Step 2: 왼팔 capture pose 이동')
    r = client.move_to_joints(CAPTURE_JOINTS_L, arm=Arm.LEFT, velocity=0.2, acceleration=0.2)
    log.info(f'[base_c] 왼팔 결과: {r.value}')

    remove_zone_c_table(client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
