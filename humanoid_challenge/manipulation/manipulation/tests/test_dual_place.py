from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_dual_client import (
    LiftTrajectoryClient,
    SymmetricDualArmClient,
    copy_pose_with_z_offset,
    pose_to_str,
    waypoint_to_pose,
)
from manipulation.skill_primitives.planning_filter_dual import PlanningFilterDual


# =============================================================================
# 파라미터
# =============================================================================

SEGMENT_DURATION_SEC = 4.0
TIMEOUT_SEC = 30.0

LIFT_HOME = 0.0
LIFT_PLACE_Z = -0.25
LIFT_MOVE_DURATION_SEC = 5.0
LIFT_TIMEOUT_SEC = 15.0

PLACE_WAYPOINT = {
    "name": "wp1_place",
    "position": [0.7, 0.35, 1.2],
    "rpy_deg": [-90.0, 0.0, 90.0],
}

# =============================================================================


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("test_dual_place")
    log = node.get_logger()

    client = SymmetricDualArmClient(node, manage_executor=True)
    lift = LiftTrajectoryClient(node)
    planning_filter = PlanningFilterDual(client, log=log)

    try:
        log.info("test_dual_place uses file-defined LEFT waypoint with PlanningFilterDual")
        client.wait_until_ready(timeout_sec=30.0)
        time.sleep(0.2)
        client.log_joint_summary("Robot state before test_dual_place")

        place_name, raw_place_pose = waypoint_to_pose(PLACE_WAYPOINT)
        place_pose = copy_pose_with_z_offset(raw_place_pose, LIFT_PLACE_Z)

        log.info("==== DUAL PLACE SEQUENCE ====")
        log.info(f"Step 1: keep arms still, move lift to absolute {LIFT_PLACE_Z:.3f}")
        log.info(f"Step 2: move arms to {place_name}: {pose_to_str(place_pose)}")
        log.info(f"Step 3: move lift back to {LIFT_HOME:.3f}")
        log.info(f"Raw place waypoint: {pose_to_str(raw_place_pose)}")
        log.info(f"Applied lift z offset: {LIFT_PLACE_Z:.3f}")

        if not lift.move_to(
            LIFT_PLACE_Z,
            duration_sec=LIFT_MOVE_DURATION_SEC,
            timeout_sec=LIFT_TIMEOUT_SEC,
        ):
            raise RuntimeError("failed to move lift to absolute place height")

        left_traj, right_traj, _ = planning_filter.plan_left_waypoint_then_mirror(
            place_pose,
            name=place_name,
            segment_duration_sec=SEGMENT_DURATION_SEC,
            timeout_sec=TIMEOUT_SEC,
        )
        result = client.execute_both(left_traj, right_traj, timeout_sec=TIMEOUT_SEC)
        log.info(f"{place_name} result: {result}")
        if not result.both_succeeded:
            lift.move_to(
                LIFT_HOME,
                duration_sec=LIFT_MOVE_DURATION_SEC,
                timeout_sec=LIFT_TIMEOUT_SEC,
            )
            raise RuntimeError(f"{place_name} execution failed: {result}")

        if not lift.move_to(
            LIFT_HOME,
            duration_sec=LIFT_MOVE_DURATION_SEC,
            timeout_sec=LIFT_TIMEOUT_SEC,
        ):
            raise RuntimeError("failed to move lift home after place")

        client.log_joint_summary("Robot state after test_dual_place")

    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
