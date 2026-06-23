#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from ai_worker_manipulation.robot_interface.planning_scene_b import (
    setup_zone_b,
    clear_all_objects,
    EnvironmentVisualizer,
)

_MARKER_PUBLISH_HZ = 2.0


def main(args=None):
    rclpy.init(args=args)

    node = Node("test_zone_b")
    log = node.get_logger()

    try:
        log.info("[test_zone_b] 시작")

        client = MoveItClient(node)

        log.info("[test_zone_b] Planning Scene 초기화")
        clear_all_objects(client)
        time.sleep(0.5)

        log.info("[test_zone_b] Zone B collision objects 등록 중...")
        setup_zone_b(client)
        time.sleep(0.5)

        viz = EnvironmentVisualizer(node)

        log.info("[test_zone_b] /competition_markers publish 시작")
        log.info("[test_zone_b] RViz에서 Add -> By topic -> /competition_markers -> MarkerArray 추가")
        log.info("[test_zone_b] 종료하려면 Ctrl+C")

        while rclpy.ok():
            viz.publish_zone("B")
            rclpy.spin_once(node, timeout_sec=1.0 / _MARKER_PUBLISH_HZ)

    except KeyboardInterrupt:
        log.info("[test_zone_b] KeyboardInterrupt")

    finally:
        log.info("[test_zone_b] 종료")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()