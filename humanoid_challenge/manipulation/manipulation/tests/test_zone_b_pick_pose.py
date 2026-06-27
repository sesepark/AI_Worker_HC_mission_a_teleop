#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient
from manipulation.robot_interface.planning_scene_b_pick_pose import (
    ZONE_B_PICK_POSE_POINT_POSITION,
    clear_all_objects,
    EnvironmentVisualizer,
    setup_zone_b,
)


_MARKER_PUBLISH_HZ = 2.0


def main(args=None):
    rclpy.init(args=args)

    node = Node("test_zone_b_pick_pose")
    log = node.get_logger()

    try:
        log.info("[test_zone_b_pick_pose] 시작")

        client = MoveItClient(node)

        log.info("[test_zone_b_pick_pose] Planning Scene 초기화")
        clear_all_objects(client)
        time.sleep(0.5)

        log.info("[test_zone_b_pick_pose] Zone B pick collision objects 등록 중...")
        setup_zone_b(client)
        time.sleep(0.5)

        viz = EnvironmentVisualizer(node)
        viz.publish_zone("B")

        log.info(
            "[test_zone_b_pick_pose] pick pose point "
            f"x={ZONE_B_PICK_POSE_POINT_POSITION[0]:.3f}, "
            f"y={ZONE_B_PICK_POSE_POINT_POSITION[1]:.3f}, "
            f"z={ZONE_B_PICK_POSE_POINT_POSITION[2]:.3f}"
        )
        log.info("[test_zone_b_pick_pose] /competition_markers publish 시작")
        log.info("[test_zone_b_pick_pose] RViz에서 Add -> By topic -> /competition_markers -> MarkerArray 추가")
        log.info("[test_zone_b_pick_pose] 종료하려면 Ctrl+C")

        while rclpy.ok():
            viz.publish_zone("B")
            rclpy.spin_once(node, timeout_sec=1.0 / _MARKER_PUBLISH_HZ)

    except KeyboardInterrupt:
        log.info("[test_zone_b_pick_pose] KeyboardInterrupt")

    finally:
        log.info("[test_zone_b_pick_pose] 종료")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
