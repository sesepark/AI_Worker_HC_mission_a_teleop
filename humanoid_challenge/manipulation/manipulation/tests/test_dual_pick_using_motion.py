from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from manipulation.robot_interface.moveit_dual_client import (
    ARM_L_JOINTS,
    ARM_R_JOINTS,
    Arm,
    LiftTrajectoryClient,
    SymmetricDualArmClient,
    trajectory_duration_seconds,
)
from manipulation.robot_interface.dual_motion_io import (
    SEGMENT_LIFT_DOWN_TO_WP2,
    SEGMENT_Z0_TO_WP1,
    get_segment,
    get_segment_trajectories,
    latest_pick_motion_path,
    load_motion_file,
    max_trajectory_start_error,
    resolve_pick_motion_path,
)
from manipulation.tests.test_dual_pick_old import (
    LIFT_DOWN,
    LIFT_HOME,
    LIFT_MOVE_DURATION_SEC,
    LIFT_TIMEOUT_SEC,
    LIFT_WAIT_AFTER_WP2_SEC,
    TIMEOUT_SEC,
    WAIT_AFTER_WP1_SEC,
    lift_duration_at_nominal_speed,
)


DEFAULT_START_TOLERANCE_RAD = 0.15


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description="Replay test_dual_pick arm motions from a saved joint-motion JSON file."
    )
    parser.add_argument(
        "--motion-file",
        type=Path,
        default=None,
        help="Motion JSON produced by test_dual_motion_save. If omitted, uses the newest saved file.",
    )
    parser.add_argument(
        "--allow-start-mismatch",
        action="store_true",
        help="Replay even if current joints do not match the first saved point.",
    )
    parser.add_argument(
        "--start-tolerance-rad",
        type=float,
        default=DEFAULT_START_TOLERANCE_RAD,
        help="Maximum allowed per-joint start mismatch before replay is blocked.",
    )
    parser.add_argument(
        "--no-final-lift-home",
        action="store_true",
        help="Do not move the lift back to z=0 after wp2.",
    )
    parser.add_argument("--execute-timeout-sec", type=float, default=TIMEOUT_SEC)
    return parser.parse_known_args(args=args)


def validate_trajectory_pair(left_traj: JointTrajectory, right_traj: JointTrajectory) -> None:
    if left_traj.joint_names != ARM_L_JOINTS:
        raise ValueError(f"left trajectory joint_names must be {ARM_L_JOINTS}, got {left_traj.joint_names}")
    if right_traj.joint_names != ARM_R_JOINTS:
        raise ValueError(f"right trajectory joint_names must be {ARM_R_JOINTS}, got {right_traj.joint_names}")
    if not left_traj.points or not right_traj.points:
        raise ValueError("saved motion contains an empty trajectory")


def check_segment_start(
    *,
    client: SymmetricDualArmClient,
    log,
    name: str,
    left_traj: JointTrajectory,
    right_traj: JointTrajectory,
    tolerance_rad: float,
    allow_mismatch: bool,
) -> None:
    left_current = client.get_joints(Arm.LEFT)
    right_current = client.get_joints(Arm.RIGHT)
    if left_current is None or right_current is None:
        raise RuntimeError("joint states unavailable")

    left_error = max_trajectory_start_error(left_current, left_traj)
    right_error = max_trajectory_start_error(right_current, right_traj)
    max_error = max(left_error, right_error)
    log.info(
        f"[{name}] start error: left={left_error:.4f} rad, "
        f"right={right_error:.4f} rad, max={max_error:.4f} rad"
    )
    if max_error <= float(tolerance_rad):
        return

    message = (
        f"[{name}] current joints do not match saved motion start "
        f"(max_error={max_error:.4f} rad > tolerance={float(tolerance_rad):.4f} rad)"
    )
    if allow_mismatch:
        log.warn(message)
        return
    raise RuntimeError(message)


def log_segment_info(log, payload: dict, name: str, left_traj: JointTrajectory) -> None:
    segment = get_segment(payload, name)
    log.info(
        f"[{name}] points={len(left_traj.points)} "
        f"duration={trajectory_duration_seconds(left_traj):.3f}s "
        f"lift_start={float(segment.get('lift_position_start', 0.0)):.3f} "
        f"lift_end={float(segment.get('lift_position_end', 0.0)):.3f}"
    )


def main(args=None) -> None:
    ns, ros_args = parse_args(args=args)
    rclpy.init(args=ros_args)
    node = Node("test_dual_pick_using_motion")
    log = node.get_logger()

    client = SymmetricDualArmClient(node, manage_executor=True)
    lift = LiftTrajectoryClient(node)

    exit_code = 1
    lift_is_down = False

    try:
        motion_file = (
            resolve_pick_motion_path(Path(ns.motion_file))
            if ns.motion_file is not None
            else latest_pick_motion_path()
        )
        payload = load_motion_file(motion_file)
        left_traj_1, right_traj_1 = get_segment_trajectories(payload, SEGMENT_Z0_TO_WP1)
        left_traj_2, right_traj_2 = get_segment_trajectories(payload, SEGMENT_LIFT_DOWN_TO_WP2)
        validate_trajectory_pair(left_traj_1, right_traj_1)
        validate_trajectory_pair(left_traj_2, right_traj_2)

        log.info(f"test_dual_pick_using_motion loaded {motion_file}")
        log_segment_info(log, payload, SEGMENT_Z0_TO_WP1, left_traj_1)
        log_segment_info(log, payload, SEGMENT_LIFT_DOWN_TO_WP2, left_traj_2)

        client.wait_until_ready(timeout_sec=30.0)
        time.sleep(0.2)
        client.log_joint_summary("Robot state before test_dual_pick_using_motion")

        log.info("==== DUAL PICK REPLAY SEQUENCE ====")
        log.info(f"Step 0: ensure lift at {LIFT_HOME:.3f}")
        log.info(f"Step 1: replay saved {SEGMENT_Z0_TO_WP1}")
        log.info(f"Step 2: move lift down to {LIFT_DOWN:.3f}")
        log.info(f"Step 3: replay saved {SEGMENT_LIFT_DOWN_TO_WP2}")
        log.info(f"Step 4: move lift back to {LIFT_HOME:.3f}")

        if not lift.move_to(
            LIFT_HOME,
            duration_sec=lift_duration_at_nominal_speed(client, LIFT_HOME, log),
            timeout_sec=LIFT_TIMEOUT_SEC,
        ):
            raise RuntimeError("failed to move lift to home before replay")

        check_segment_start(
            client=client,
            log=log,
            name=SEGMENT_Z0_TO_WP1,
            left_traj=left_traj_1,
            right_traj=right_traj_1,
            tolerance_rad=float(ns.start_tolerance_rad),
            allow_mismatch=bool(ns.allow_start_mismatch),
        )
        result = client.execute_both(
            left_traj_1,
            right_traj_1,
            timeout_sec=float(ns.execute_timeout_sec),
        )
        log.info(f"{SEGMENT_Z0_TO_WP1} result: {result}")
        if not result.both_succeeded:
            raise RuntimeError(f"{SEGMENT_Z0_TO_WP1} execution failed: {result}")

        log.info(f"{SEGMENT_Z0_TO_WP1} reached; waiting {WAIT_AFTER_WP1_SEC:.1f}s before lift motion")
        time.sleep(WAIT_AFTER_WP1_SEC)

        if not lift.move_to(
            LIFT_DOWN,
            duration_sec=LIFT_MOVE_DURATION_SEC,
            timeout_sec=LIFT_TIMEOUT_SEC,
        ):
            raise RuntimeError("failed to move lift down before wp2 replay")
        lift_is_down = True

        check_segment_start(
            client=client,
            log=log,
            name=SEGMENT_LIFT_DOWN_TO_WP2,
            left_traj=left_traj_2,
            right_traj=right_traj_2,
            tolerance_rad=float(ns.start_tolerance_rad),
            allow_mismatch=bool(ns.allow_start_mismatch),
        )
        result = client.execute_both(
            left_traj_2,
            right_traj_2,
            timeout_sec=float(ns.execute_timeout_sec),
        )
        log.info(f"{SEGMENT_LIFT_DOWN_TO_WP2} result: {result}")
        if not result.both_succeeded:
            raise RuntimeError(f"{SEGMENT_LIFT_DOWN_TO_WP2} execution failed: {result}")

        log.info(f"{SEGMENT_LIFT_DOWN_TO_WP2} reached; waiting {LIFT_WAIT_AFTER_WP2_SEC:.1f}s")
        time.sleep(LIFT_WAIT_AFTER_WP2_SEC)

        if not ns.no_final_lift_home:
            if not lift.move_to(
                LIFT_HOME,
                duration_sec=LIFT_MOVE_DURATION_SEC,
                timeout_sec=LIFT_TIMEOUT_SEC,
            ):
                raise RuntimeError("failed to move lift home after replay")
            lift_is_down = False

        client.log_joint_summary("Robot state after test_dual_pick_using_motion")
        exit_code = 0

    except Exception as exc:
        log.error(f"test_dual_pick_using_motion failed: {exc}")
        if lift_is_down and not ns.no_final_lift_home:
            try:
                lift.move_to(
                    LIFT_HOME,
                    duration_sec=LIFT_MOVE_DURATION_SEC,
                    timeout_sec=LIFT_TIMEOUT_SEC,
                )
            except Exception as lift_exc:
                log.error(f"exception while returning lift home: {lift_exc}")
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
