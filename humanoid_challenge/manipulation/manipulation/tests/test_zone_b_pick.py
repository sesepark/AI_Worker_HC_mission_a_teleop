#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient
from manipulation.robot_interface.planning_scene_b_pick import (
    setup_zone_b,
    clear_all_objects,
    EnvironmentVisualizer,
)

_MARKER_PUBLISH_HZ = 2.0


def main(args=None):
    rclpy.init(args=args)

    node = Node("test_zone_b_pick")
    log = node.get_logger()

    try:
        log.info("[test_zone_b_pick] 시작")

        client = MoveItClient(node)

        log.info("[test_zone_b_pick] Planning Scene 초기화")
        clear_all_objects(client)
        time.sleep(0.5)

        log.info("[test_zone_b_pick] Zone B pick collision objects 등록 중...")
        log.info("[test_zone_b_pick] zone_b_box는 시각화만 하고 collision object로 등록하지 않음")
        setup_zone_b(client)
        time.sleep(0.5)

        viz = EnvironmentVisualizer(node)
        viz.publish_zone("B")

        log.info("[test_zone_b_pick] /competition_markers publish 시작")
        log.info("[test_zone_b_pick] RViz에서 Add -> By topic -> /competition_markers -> MarkerArray 추가")
        log.info("[test_zone_b_pick] 종료하려면 Ctrl+C")

        while rclpy.ok():
            viz.publish_zone("B")
            rclpy.spin_once(node, timeout_sec=1.0 / _MARKER_PUBLISH_HZ)

    except KeyboardInterrupt:
        log.info("[test_zone_b_pick] KeyboardInterrupt")

    finally:
        log.info("[test_zone_b_pick] 종료")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
