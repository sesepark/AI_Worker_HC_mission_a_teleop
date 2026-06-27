from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_dual_client import (
    LIFT_JOINT_NAME,
    LiftTrajectoryClient,
    SymmetricDualArmClient,
    copy_pose_with_z_offset,
    pose_to_str,
    resolve_waypoint_poses,
)
from manipulation.robot_interface.planning_scene_b_pick import (
    add_zone_b_box_collision,
    remove_zone_b_box_collision,
)
from manipulation.skill_primitives.planning_filter_dual import PlanningFilterDual


# =============================================================================
# 파라미터
# =============================================================================

SEGMENT_DURATION_SEC = 4.0
TIMEOUT_SEC = 30.0

LIFT_HOME = 0.0
LIFT_DROP = 0.45
LIFT_DOWN = LIFT_HOME - LIFT_DROP
WAIT_AFTER_WP1_SEC = 2.0
LIFT_WAIT_AFTER_WP2_SEC = 4.0
LIFT_MOVE_DURATION_SEC = 10.0
LIFT_TIMEOUT_SEC = 15.0
LIFT_MIN_MOVE_DURATION_SEC = 0.5
PLANNING_SCENE_SETTLE_SEC = 0.5

LEFT_WAYPOINTS = [
    {
        "name": "wp1",
        "position": [0.6, 0.35, 1.2],
        "rpy_deg": [-90.0, 0.0, 90.0],
    },
    {
        "name": "wp2",
        "position": [0.6, 0.246, 1.2],
        "rpy_deg": [-90.0, 0.0, 90.0],
    },
]

# =============================================================================


def get_lift_position(client: SymmetricDualArmClient) -> float | None:
    state = client.current_robot_state()
    names = list(state.joint_state.name)
    positions = list(state.joint_state.position)
    try:
        return float(positions[names.index(LIFT_JOINT_NAME)])
    except ValueError:
        return None


def lift_duration_at_nominal_speed(
    client: SymmetricDualArmClient,
    target: float,
    log,
) -> float:
    current = get_lift_position(client)
    if current is None:
        log.warn(
            f"[lift] {LIFT_JOINT_NAME} not found in joint_states; "
            f"using default duration {LIFT_MOVE_DURATION_SEC:.1f}s"
        )
        return LIFT_MOVE_DURATION_SEC

    nominal_speed = abs(LIFT_DROP) / LIFT_MOVE_DURATION_SEC
    distance = abs(float(target) - current)
    duration = max(LIFT_MIN_MOVE_DURATION_SEC, distance / nominal_speed)
    log.info(
        f"[lift] current={current:.3f}, target={float(target):.3f}, "
        f"distance={distance:.3f}, duration={duration:.2f}s "
        f"(speed={nominal_speed:.3f}m/s)"
    )
    return duration


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("test_dual_pick_old")
    log = node.get_logger()

    client = SymmetricDualArmClient(node, manage_executor=True)
    lift = LiftTrajectoryClient(node)
    planning_filter = PlanningFilterDual(client, log=log)
    box_collision_enabled = False

    try:
        log.info("test_dual_pick_old uses file-defined LEFT waypoints with PlanningFilterDual")
        client.wait_until_ready(timeout_sec=30.0)
        time.sleep(0.2)
        client.log_joint_summary("Robot state before test_dual_pick_old")

        resolved_waypoints = resolve_waypoint_poses(LEFT_WAYPOINTS)
        wp1_name, wp1_pose = resolved_waypoints[0]
        wp2_name, wp2_pose_raw = resolved_waypoints[1]
        wp2_pose = copy_pose_with_z_offset(wp2_pose_raw, -LIFT_DROP)

        log.info("==== DUAL PICK SEQUENCE ====")
        log.info(f"Step 0: ensure lift at {LIFT_HOME:.3f}")
        log.info(f"Step 1: move arms to {wp1_name}: {pose_to_str(wp1_pose)}")
        log.info(f"Step 2: move lift down to {LIFT_DOWN:.3f}")
        log.info(f"Step 3: move arms to {wp2_name}: {pose_to_str(wp2_pose)}")
        log.info(f"Step 4: move lift back to {LIFT_HOME:.3f}")

        if not lift.move_to(
            LIFT_HOME,
            duration_sec=lift_duration_at_nominal_speed(client, LIFT_HOME, log),
            timeout_sec=LIFT_TIMEOUT_SEC,
        ):
            raise RuntimeError("failed to move lift to home before wp1")

        log.info("[scene] wp1 접근 계획: zone_b_box를 collision object로 사용")
        add_zone_b_box_collision(client)
        box_collision_enabled = True
        time.sleep(PLANNING_SCENE_SETTLE_SEC)

        left_traj_1, right_traj_1, _ = planning_filter.plan_left_waypoint_then_mirror(
            wp1_pose,
            name=wp1_name,
            segment_duration_sec=SEGMENT_DURATION_SEC,
            timeout_sec=TIMEOUT_SEC,
        )
        result = client.execute_both(left_traj_1, right_traj_1, timeout_sec=TIMEOUT_SEC)
        log.info(f"{wp1_name} result: {result}")
        if not result.both_succeeded:
            raise RuntimeError(f"{wp1_name} execution failed: {result}")

        log.info(f"{wp1_name} reached; waiting {WAIT_AFTER_WP1_SEC:.1f}s before lift motion")
        time.sleep(WAIT_AFTER_WP1_SEC)

        if not lift.move_to(
            LIFT_DOWN,
            duration_sec=LIFT_MOVE_DURATION_SEC,
            timeout_sec=LIFT_TIMEOUT_SEC,
        ):
            raise RuntimeError("failed to move lift down before wp2")

        log.info("[scene] wp2 접근 계획: lift down 이후 zone_b_box collision object 제거")
        remove_zone_b_box_collision(client)
        box_collision_enabled = False
        time.sleep(PLANNING_SCENE_SETTLE_SEC)

        left_traj_2, right_traj_2, _ = planning_filter.plan_left_waypoint_then_mirror(
            wp2_pose,
            name=wp2_name,
            segment_duration_sec=SEGMENT_DURATION_SEC,
            timeout_sec=TIMEOUT_SEC,
        )
        result = client.execute_both(left_traj_2, right_traj_2, timeout_sec=TIMEOUT_SEC)
        log.info(f"{wp2_name} result: {result}")
        if not result.both_succeeded:
            lift.move_to(
                LIFT_HOME,
                duration_sec=LIFT_MOVE_DURATION_SEC,
                timeout_sec=LIFT_TIMEOUT_SEC,
            )
            raise RuntimeError(f"{wp2_name} execution failed: {result}")

        log.info(f"{wp2_name} reached; waiting {LIFT_WAIT_AFTER_WP2_SEC:.1f}s")
        time.sleep(LIFT_WAIT_AFTER_WP2_SEC)

        if not lift.move_to(
            LIFT_HOME,
            duration_sec=LIFT_MOVE_DURATION_SEC,
            timeout_sec=LIFT_TIMEOUT_SEC,
        ):
            raise RuntimeError("failed to move lift home after wp2")

        client.log_joint_summary("Robot state after test_dual_pick_old")

    finally:
        if box_collision_enabled:
            try:
                remove_zone_b_box_collision(client)
                time.sleep(PLANNING_SCENE_SETTLE_SEC)
            except Exception as exc:
                log.warn(f"[scene] failed to remove zone_b_box during cleanup: {exc}")
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
