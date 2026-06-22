"""
리프트 인터랙티브 테스트 도구.
키 입력으로 리프트를 올리고 내릴 수 있습니다.

  w  → 위로 (STEP만큼)
  s  → 아래로 (STEP만큼)
  h  → 홈 (0.0, 최상단)
  p  → 현재 위치 출력
  q  → 종료

실행:
  ros2 run manipulation test_lift
"""

import sys
import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient

STEP     = 0.05   # 한 번 누를 때 이동 거리 (m)
MIN_POS  = -0.30  # 리프트 하한 (m)
MAX_POS  =  0.0   # 리프트 상한 (m, 0.0 = 최상단)


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
    node   = Node('test_lift')
    log    = node.get_logger()
    client = MoveItClient(node)

    current = 0.0
    log.info(f'[test_lift] 시작 — pos={current:.3f} m')
    log.info('[test_lift] w=up  s=down  h=home  p=print  q=quit')

    while True:
        key = _getch()

        if key == 'q':
            log.info('[test_lift] 종료')
            break

        elif key == 'h':
            target = MAX_POS
            log.info(f'[test_lift] home → {target:.3f} m')
            result = client.move_lift(target)
            if result.value == 'succeeded':
                current = target
            log.info(f'[test_lift] 결과: {result.value}  pos={current:.3f} m')

        elif key == 'w':
            target = min(current + STEP, MAX_POS)
            log.info(f'[test_lift] up → {target:.3f} m')
            result = client.move_lift(target)
            if result.value == 'succeeded':
                current = target
            log.info(f'[test_lift] 결과: {result.value}  pos={current:.3f} m')

        elif key == 's':
            target = max(current - STEP, MIN_POS)
            log.info(f'[test_lift] down → {target:.3f} m')
            result = client.move_lift(target)
            if result.value == 'succeeded':
                current = target
            log.info(f'[test_lift] 결과: {result.value}  pos={current:.3f} m')

        elif key == 'p':
            log.info(f'[test_lift] 현재 위치: {current:.3f} m')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()