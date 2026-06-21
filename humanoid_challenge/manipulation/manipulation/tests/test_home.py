"""
홈 포지션 이동 테스트.

실행:
  ros2 run manipulation test_home
"""

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient, Arm

# ── 파라미터 ──────────────────────────────────
ARM = 'both'   # 'left' | 'right' | 'both'
# ──────────────────────────────────────────────


def main():
    rclpy.init()
    node = Node('test_home')
    log  = node.get_logger()

    client = MoveItClient(node)

    log.info(f'[test_home] home 이동 — arm={ARM}')

    if ARM in ('right', 'both'):
        result = client.move_to_home(Arm.RIGHT)
        log.info(f'[test_home] right: {result.value}')

    if ARM in ('left', 'both'):
        result = client.move_to_home(Arm.LEFT)
        log.info(f'[test_home] left: {result.value}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
