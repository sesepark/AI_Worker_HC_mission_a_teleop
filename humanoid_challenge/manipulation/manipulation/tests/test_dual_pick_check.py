from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from manipulation.robot_interface.moveit_dual_client import (
    LIFT_JOINT_NAME,
    LiftTrajectoryClient,
    SymmetricDualArmClient,
    copy_pose_with_z_offset,
    pose_to_str,
    resolve_waypoint_poses,
    trajectory_duration_seconds,
)
from manipulation.skill_primitives.planning_filter_dual import PlanningFilterDual


# =============================================================================
# Parameters
# =============================================================================

WP1_SEGMENT_DURATION_SEC = 4.0
SCAN_SEGMENT_DURATION_SEC = 2.0
TIMEOUT_SEC = 30.0

LIFT_HOME = 0.0
LIFT_DROP = 0.45
LIFT_DOWN = LIFT_HOME - LIFT_DROP
WAIT_AFTER_WP1_SEC = 2.0
LIFT_MOVE_DURATION_SEC = 10.0
LIFT_TIMEOUT_SEC = 15.0
LIFT_MIN_MOVE_DURATION_SEC = 0.5

SCAN_Y_START_MM = 252
SCAN_Y_END_MM = 220
SCAN_Y_STEP_MM = -1

LEFT_WAYPOINTS = [
    {
        "name": "wp1",
        "position": [0.7, 0.35, 1.2],
        "rpy_deg": [-90.0, 0.0, 90.0],
    },
    {
        "name": "wp2",
        "position": [0.7, 0.22, 1.2],
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


def iter_scan_y_values() -> list[float]:
    if SCAN_Y_STEP_MM >= 0:
        raise ValueError("SCAN_Y_STEP_MM must be negative for this descending scan")
    return [
        mm / 1000.0
        for mm in range(SCAN_Y_START_MM, SCAN_Y_END_MM - 1, SCAN_Y_STEP_MM)
    ]


def make_wp2_scan_pose(raw_wp2_pose: Pose, y: float) -> Pose:
    pose = copy_pose_with_z_offset(raw_wp2_pose, -LIFT_DROP)
    pose.position.y = float(y)
    return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("test_dual_pick_check")
    log = node.get_logger()

    client = SymmetricDualArmClient(node, manage_executor=True)
    lift = LiftTrajectoryClient(node)
    planning_filter = PlanningFilterDual(client, log=log)

    should_return_lift_home = False

    try:
        log.info("test_dual_pick_check scans wp2 y with PlanningFilterDual")
        client.wait_until_ready(timeout_sec=30.0)
        time.sleep(0.2)
        client.log_joint_summary("Robot state before test_dual_pick_check")

        resolved_waypoints = resolve_waypoint_poses(LEFT_WAYPOINTS)
        wp1_name, wp1_pose = resolved_waypoints[0]
        wp2_name, wp2_pose_raw = resolved_waypoints[1]
        y_values = iter_scan_y_values()

        log.info("==== DUAL PICK CHECK SEQUENCE ====")
        log.info(f"Step 0: ensure lift at {LIFT_HOME:.3f}")
        log.info(f"Step 1: move arms to {wp1_name}: {pose_to_str(wp1_pose)}")
        log.info(f"Step 2: move lift down to {LIFT_DOWN:.3f}")
        log.info(
            f"Step 3: scan {wp2_name} y from {y_values[0]:.3f} "
            f"to {y_values[-1]:.3f} by 0.001 m"
        )
        log.info(f"Each scan motion duration: {SCAN_SEGMENT_DURATION_SEC:.1f}s")

        if not lift.move_to(
            LIFT_HOME,
            duration_sec=lift_duration_at_nominal_speed(client, LIFT_HOME, log),
            timeout_sec=LIFT_TIMEOUT_SEC,
        ):
            raise RuntimeError("failed to move lift to home before wp1")

        left_traj_1, right_traj_1, _ = planning_filter.plan_left_waypoint_then_mirror(
            wp1_pose,
            name=wp1_name,
            segment_duration_sec=WP1_SEGMENT_DURATION_SEC,
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
            raise RuntimeError("failed to move lift down before wp2 scan")
        should_return_lift_home = True

        last_success_y: float | None = None
        first_failed_y: float | None = None
        failure_reason = ""
        success_count = 0

        for index, y in enumerate(y_values, start=1):
            scan_pose = make_wp2_scan_pose(wp2_pose_raw, y)
            scan_name = f"{wp2_name}_y_{y:.3f}"
            log.info(
                f"[scan {index}/{len(y_values)}] trying y={y:.3f} "
                f"pose={pose_to_str(scan_pose)}"
            )

            try:
                left_traj, right_traj, _ = planning_filter.plan_left_waypoint_then_mirror(
                    scan_pose,
                    name=scan_name,
                    segment_duration_sec=SCAN_SEGMENT_DURATION_SEC,
                    timeout_sec=TIMEOUT_SEC,
                    skip_current_pose_check=True,
                )
            except Exception as exc:
                first_failed_y = y
                failure_reason = f"planning failed: {exc}"
                log.error(f"[scan] FAIL y={y:.3f} | {failure_reason}")
                break

            planned_duration = trajectory_duration_seconds(left_traj)
            log.info(f"[scan] planned controller duration y={y:.3f}: {planned_duration:.3f}s")

            result = client.execute_both(left_traj, right_traj, timeout_sec=TIMEOUT_SEC)
            log.info(f"{scan_name} result: {result}")
            if not result.both_succeeded:
                first_failed_y = y
                failure_reason = f"execution failed: {result}"
                log.error(f"[scan] FAIL y={y:.3f} | {failure_reason}")
                break

            last_success_y = y
            success_count += 1
            log.info(
                f"[scan] SUCCESS y={y:.3f} | "
                f"last_success_y={last_success_y:.3f} "
                f"success_count={success_count}/{len(y_values)}"
            )

        log.info("==== DUAL PICK CHECK RESULT ====")
        if last_success_y is None:
            log.warn("[scan] no wp2 y value succeeded")
        else:
            log.info(
                f"[scan] last successful y={last_success_y:.3f} "
                f"success_count={success_count}/{len(y_values)}"
            )

        if first_failed_y is None:
            log.info(f"[scan] all y values succeeded through {y_values[-1]:.3f}")
        else:
            log.warn(f"[scan] first failed y={first_failed_y:.3f}")
            log.warn(f"[scan] failure reason: {failure_reason}")

        client.log_joint_summary("Robot state after test_dual_pick_check scan")

    finally:
        if should_return_lift_home:
            try:
                if not lift.move_to(
                    LIFT_HOME,
                    duration_sec=LIFT_MOVE_DURATION_SEC,
                    timeout_sec=LIFT_TIMEOUT_SEC,
                ):
                    log.error("failed to move lift home after wp2 scan")
            except Exception as exc:
                log.error(f"exception while returning lift home: {exc}")

        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
