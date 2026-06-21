"""
그리퍼 인터랙티브 테스트 도구.
키 입력으로 그리퍼를 직접 열고 닫을 수 있습니다.

  o  → open
  c  → close
  t  → toggle
  l  → left side
  r  → right side
  b  → both
  q  → 종료

실행:
  ros2 run manipulation test_gripper
"""

import sys
import time
import rclpy
from rclpy.node import Node

from manipulation.robot_interface.gripper_controller import GripperInterface


def _getch():
    import tty, termios
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node    = Node('test_gripper')
    log     = node.get_logger()
    gripper = GripperInterface(node)

    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=1.0)

    side = 'right'
    log.info(f'[test_gripper] 시작 — side={side}')
    log.info('[test_gripper] o=open  c=close  t=toggle  l=left  r=right  b=both  q=quit')

    while True:
        key = _getch()

        if key == 'q':
            log.info('[test_gripper] 종료')
            break
        elif key == 'r':
            side = 'right'
            log.info(f'[test_gripper] side → {side}')
        elif key == 'l':
            side = 'left'
            log.info(f'[test_gripper] side → {side}')
        elif key == 'b':
            side = 'both'
            log.info(f'[test_gripper] side → {side}')
        elif key == 'o':
            log.info(f'[test_gripper] open ({side})')
            gripper.open(side)
            gripper.wait_until_executed()
            gripper.wait_motion()
            pos = gripper._command.get_position('left' if side == 'both' else side)
            log.info(f'[test_gripper] position: {pos}')
        elif key == 'c':
            log.info(f'[test_gripper] close ({side})')
            gripper.close(side)
            gripper.wait_until_executed()
            gripper.wait_motion()
            pos = gripper._command.get_position('left' if side == 'both' else side)
            log.info(f'[test_gripper] position: {pos}')
        elif key == 't':
            log.info(f'[test_gripper] toggle ({side})')
            gripper.toggle(side)
            gripper.wait_until_executed()
            gripper.wait_motion()
            pos = gripper._command.get_position('left' if side == 'both' else side)
            log.info(f'[test_gripper] position: {pos}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
