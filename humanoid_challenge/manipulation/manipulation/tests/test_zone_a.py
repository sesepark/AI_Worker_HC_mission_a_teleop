"""
Zone A 환경 시각화 테스트.
MoveIt Planning Scene에 Zone A collision object를 등록하고
/competition_markers 토픽으로 RViz 마커를 퍼블리시한다.

실행:
  ros2 run manipulation test_zone_a
"""

import time
import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient
from manipulation.robot_interface.planning_scene import (
    setup_zone_a,
    clear_all_objects,
    EnvironmentVisualizer,
)

_MARKER_PUBLISH_HZ = 2.0


def main():
    rclpy.init()
    node = Node('test_zone_a')
    log = node.get_logger()

    log.info('[test_zone_a] 시작')
    client = MoveItClient(node)

    log.info('[test_zone_a] Planning Scene 초기화')
    clear_all_objects(client)
    time.sleep(0.5)

    log.info('[test_zone_a] Zone A collision objects 등록 중...')
    setup_zone_a(client)
    time.sleep(0.5)

    viz = EnvironmentVisualizer(node)

    log.info('[test_zone_a] RViz 마커 퍼블리시 시작 (Ctrl+C로 종료)')
    try:
        while rclpy.ok():
            viz.publish_zone('A')
            rclpy.spin_once(node, timeout_sec=1.0 / _MARKER_PUBLISH_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        log.info('[test_zone_a] 종료 — collision objects 제거 중')
        clear_all_objects(client)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
