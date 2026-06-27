"""
Zone C 환경 시각화 테스트 (test_zone_a 의 C 대응본).
MoveIt Planning Scene에 Zone C collision object(벤치 + peg 4개 + bolt 4개 + 버튼)를 등록하고
/competition_markers 토픽으로 RViz 마커를 퍼블리시한다.

실행:
  ros2 run manipulation test_zone_c
"""

import time
import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient
from manipulation.robot_interface.planning_scene import (
    setup_zone_c,
    clear_all_objects,
    EnvironmentVisualizer,
)

_MARKER_PUBLISH_HZ = 2.0


def main():
    rclpy.init()
    node = Node('test_zone_c')
    log = node.get_logger()

    log.info('[test_zone_c] 시작')
    client = MoveItClient(node)

    log.info('[test_zone_c] Planning Scene 초기화')
    clear_all_objects(client)
    time.sleep(0.5)

    log.info('[test_zone_c] Zone C collision objects 등록 중...')
    setup_zone_c(client)
    time.sleep(0.5)

    viz = EnvironmentVisualizer(node)

    log.info('[test_zone_c] RViz 마커 퍼블리시 시작 (Ctrl+C로 종료)')
    try:
        while rclpy.ok():
            viz.publish_zone('C')
            rclpy.spin_once(node, timeout_sec=1.0 / _MARKER_PUBLISH_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        log.info('[test_zone_c] 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
