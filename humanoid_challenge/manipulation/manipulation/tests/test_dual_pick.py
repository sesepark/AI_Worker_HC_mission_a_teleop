from __future__ import annotations

import argparse
import copy
import math
import sys
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from manipulation.robot_interface.moveit_dual_client import (
    ARM_L_JOINTS,
    ARM_R_JOINTS,
    Arm,
    LIFT_JOINT_NAME,
    LiftTrajectoryClient,
    SymmetricDualArmClient,
    duration_from_seconds,
    mirror_left_joints_to_right,
    mirror_left_trajectory_to_right,
    retime_trajectory,
)
from manipulation.robot_interface.planning_scene_b_pick import (
    add_zone_b_box_collision,
    remove_zone_b_box_collision,
)


# =============================================================================
# USER CONFIG
# =============================================================================

# LEFT arm waypoint2 joint values in ARM_L_JOINTS order, degree.
# Rounded to integer degrees from:
# -36.527, 2.687, -0.861, -57.786, 92.732, 0.705, 4.310
LEFT_WAYPOINT2_JOINTS_DEG: list[float] = [
    -37.0,  # arm_l_joint1
    3.0,    # arm_l_joint2
    -1.0,   # arm_l_joint3
    -58.0,  # arm_l_joint4
    93.0,   # arm_l_joint5
    1.0,    # arm_l_joint6
    4.0,    # arm_l_joint7
]

# waypoint1 is copied from waypoint2, except arm_l_joint2/3 are replaced by these.
LEFT_WAYPOINT1_JOINT2_DEG = 24.0
LEFT_WAYPOINT1_JOINT3_DEG = 7.0


# =============================================================================
# Parameters
# =============================================================================

SEGMENT_DURATION_SEC = 4.0
JOINT2_DURATION_SEC = 4.0
JOINT3_DURATION_SEC = 4.0
JOINT_PAUSE_SEC = 1.0
JOINT_MOVE_POINTS = 25
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

MOVEIT_VELOCITY = 0.05
MOVEIT_ACCELERATION = 0.05

# =============================================================================


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description=(
            "Dual pick variant: MoveIt plans to wp1 with zone_b_box as an obstacle, "
            "then lift drops and joint2 moves first, waits, then joint3 moves to wp2."
        )
    )
    parser.add_argument(
        "--wp2-left-joints-deg",
        "--wp2-left-joints",
        type=float,
        nargs=7,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="LEFT arm waypoint2 joint values in degrees. Overrides LEFT_WAYPOINT2_JOINTS_DEG.",
    )
    parser.add_argument(
        "--wp1-left-joint2-deg",
        type=float,
        default=None,
        help="LEFT arm joint2 value for waypoint1 in degrees. Overrides LEFT_WAYPOINT1_JOINT2_DEG.",
    )
    parser.add_argument(
        "--wp1-left-joint3-deg",
        "--wp1-left-joint3",
        type=float,
        default=None,
        help="LEFT arm joint3 value for waypoint1 in degrees. Overrides LEFT_WAYPOINT1_JOINT3_DEG.",
    )
    parser.add_argument("--segment-duration-sec", type=float, default=SEGMENT_DURATION_SEC)
    parser.add_argument("--joint2-duration-sec", type=float, default=JOINT2_DURATION_SEC)
    parser.add_argument("--joint3-duration-sec", type=float, default=JOINT3_DURATION_SEC)
    parser.add_argument("--joint-pause-sec", type=float, default=JOINT_PAUSE_SEC)
    parser.add_argument("--joint-points", type=int, default=JOINT_MOVE_POINTS)
    return parser.parse_known_args(args=args)


def format_joints(joints: list[float], *, precision: int = 4) -> str:
    return "[" + ", ".join(f"{v:.{precision}f}" for v in joints) + "]"


def degrees_to_radians(values: list[float]) -> list[float]:
    return [math.radians(float(value)) for value in values]


def radians_to_degrees(values: list[float]) -> list[float]:
    return [math.degrees(float(value)) for value in values]


def resolved_left_waypoints(ns: argparse.Namespace) -> tuple[list[float], list[float]]:
    raw_wp2_deg = ns.wp2_left_joints_deg
    if raw_wp2_deg is None:
        raw_wp2_deg = LEFT_WAYPOINT2_JOINTS_DEG
    if len(raw_wp2_deg) != 7:
        raise ValueError(
            "Set LEFT_WAYPOINT2_JOINTS_DEG in test_dual_pick.py, "
            "or pass --wp2-left-joints-deg J1 J2 J3 J4 J5 J6 J7."
        )

    wp1_joint2_deg = ns.wp1_left_joint2_deg
    if wp1_joint2_deg is None:
        wp1_joint2_deg = LEFT_WAYPOINT1_JOINT2_DEG
    wp1_joint3_deg = ns.wp1_left_joint3_deg
    if wp1_joint3_deg is None:
        wp1_joint3_deg = LEFT_WAYPOINT1_JOINT3_DEG
    if wp1_joint2_deg is None:
        raise ValueError(
            "Set LEFT_WAYPOINT1_JOINT2_DEG in test_dual_pick.py, "
            "or pass --wp1-left-joint2-deg VALUE."
        )
    if wp1_joint3_deg is None:
        raise ValueError(
            "Set LEFT_WAYPOINT1_JOINT3_DEG in test_dual_pick.py, "
            "or pass --wp1-left-joint3-deg VALUE."
        )

    wp2 = degrees_to_radians([float(value) for value in raw_wp2_deg])
    wp1 = list(wp2)
    wp1[1] = math.radians(float(wp1_joint2_deg))
    wp1[2] = math.radians(float(wp1_joint3_deg))
    return wp1, wp2


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


def with_explicit_start(traj: JointTrajectory, start_joints: list[float]) -> JointTrajectory:
    out = copy.deepcopy(traj)
    if not out.points:
        raise RuntimeError("trajectory is empty")

    if len(out.points) == 1:
        final_point = copy.deepcopy(out.points[0])
        start_point = copy.deepcopy(out.points[0])
        start_point.positions = list(start_joints)
        start_point.time_from_start = duration_from_seconds(0.0)
        out.points = [start_point, final_point]
    else:
        out.points[0].positions = list(start_joints)
    return out


def plan_wp1_with_moveit_then_mirror(
    client: SymmetricDualArmClient,
    left_wp1: list[float],
    *,
    duration_sec: float,
    timeout_sec: float,
) -> tuple[JointTrajectory, JointTrajectory]:
    left_start = client.get_joints(Arm.LEFT)
    right_start = client.get_joints(Arm.RIGHT)
    if left_start is None or right_start is None:
        raise RuntimeError("joint states unavailable")

    left_traj = client.plan_to_joints(
        Arm.LEFT,
        left_wp1,
        velocity=MOVEIT_VELOCITY,
        acceleration=MOVEIT_ACCELERATION,
        timeout_sec=timeout_sec,
    )
    left_traj = with_explicit_start(left_traj, left_start)
    left_traj = retime_trajectory(left_traj, duration_sec)

    right_traj = mirror_left_trajectory_to_right(left_traj)
    if right_traj.points:
        right_traj.points[0].positions = list(right_start)
    return left_traj, right_traj


def smoothstep(alpha: float) -> float:
    return 3.0 * alpha * alpha - 2.0 * alpha * alpha * alpha


def append_interpolated_segment(
    traj: JointTrajectory,
    start: list[float],
    target: list[float],
    *,
    start_time_sec: float,
    duration_sec: float,
    points: int,
    skip_first: bool,
) -> None:
    if len(start) != len(target) or len(start) != len(traj.joint_names):
        raise ValueError("trajectory joint count mismatch")
    if duration_sec <= 0.0:
        raise ValueError("duration_sec must be positive")
    if points < 2:
        raise ValueError("points must be >= 2")

    for i in range(points):
        if skip_first and i == 0:
            continue
        alpha = i / float(points - 1)
        s = smoothstep(alpha)
        point = JointTrajectoryPoint()
        point.positions = [start[j] + s * (target[j] - start[j]) for j in range(len(start))]
        point.time_from_start = duration_from_seconds(start_time_sec + duration_sec * alpha)
        traj.points.append(point)


def append_hold_point(
    traj: JointTrajectory,
    joints: list[float],
    *,
    time_sec: float,
) -> None:
    point = JointTrajectoryPoint()
    point.positions = list(joints)
    point.time_from_start = duration_from_seconds(time_sec)
    traj.points.append(point)


def build_two_joint_sequence(
    joint_names: list[str],
    current: list[float],
    after_joint2: list[float],
    final: list[float],
    *,
    joint2_duration_sec: float,
    pause_sec: float,
    joint3_duration_sec: float,
    points: int,
) -> JointTrajectory:
    if pause_sec < 0.0:
        raise ValueError("pause_sec must be non-negative")

    traj = JointTrajectory()
    traj.joint_names = list(joint_names)

    append_interpolated_segment(
        traj,
        current,
        after_joint2,
        start_time_sec=0.0,
        duration_sec=joint2_duration_sec,
        points=points,
        skip_first=False,
    )

    joint3_start_time = joint2_duration_sec + pause_sec
    if pause_sec > 0.0:
        append_hold_point(traj, after_joint2, time_sec=joint3_start_time)

    append_interpolated_segment(
        traj,
        after_joint2,
        final,
        start_time_sec=joint3_start_time,
        duration_sec=joint3_duration_sec,
        points=points,
        skip_first=True,
    )
    return traj


def build_joint2_then_joint3_to_wp2(
    client: SymmetricDualArmClient,
    left_wp2: list[float],
    *,
    joint2_duration_sec: float,
    pause_sec: float,
    joint3_duration_sec: float,
    points: int,
) -> tuple[JointTrajectory, JointTrajectory]:
    left_current = client.get_joints(Arm.LEFT)
    right_current = client.get_joints(Arm.RIGHT)
    if left_current is None or right_current is None:
        raise RuntimeError("joint states unavailable")

    left_after_joint2 = list(left_current)
    left_after_joint2[1] = float(left_wp2[1])
    left_target = list(left_after_joint2)
    left_target[2] = float(left_wp2[2])

    mirrored_wp2 = mirror_left_joints_to_right(left_wp2)
    right_after_joint2 = list(right_current)
    right_after_joint2[1] = float(mirrored_wp2[1])
    right_target = list(right_after_joint2)
    right_target[2] = float(mirrored_wp2[2])

    left_traj = build_two_joint_sequence(
        ARM_L_JOINTS,
        left_current,
        left_after_joint2,
        left_target,
        joint2_duration_sec=joint2_duration_sec,
        pause_sec=pause_sec,
        joint3_duration_sec=joint3_duration_sec,
        points=points,
    )
    right_traj = build_two_joint_sequence(
        ARM_R_JOINTS,
        right_current,
        right_after_joint2,
        right_target,
        joint2_duration_sec=joint2_duration_sec,
        pause_sec=pause_sec,
        joint3_duration_sec=joint3_duration_sec,
        points=points,
    )
    return left_traj, right_traj


def run(ns: argparse.Namespace, ros_args: list[str]) -> int:
    rclpy.init(args=ros_args)
    node = Node("test_dual_pick")
    log = node.get_logger()

    client = SymmetricDualArmClient(node, manage_executor=True)
    lift = LiftTrajectoryClient(node)
    box_collision_enabled = False

    try:
        left_wp1, left_wp2 = resolved_left_waypoints(ns)
        right_wp1 = mirror_left_joints_to_right(left_wp1)
        right_wp2 = mirror_left_joints_to_right(left_wp2)

        log.info("test_dual_pick uses joint-defined wp1/wp2")
        log.info(f"LEFT wp1 [deg] = {format_joints(radians_to_degrees(left_wp1), precision=1)}")
        log.info(f"LEFT wp2 [deg] = {format_joints(radians_to_degrees(left_wp2), precision=1)}")
        log.info(f"LEFT wp1 [rad] = {format_joints(left_wp1)}")
        log.info(f"LEFT wp2 [rad] = {format_joints(left_wp2)}")
        log.info(f"RIGHT wp1 mirrored [deg] = {format_joints(radians_to_degrees(right_wp1), precision=1)}")
        log.info(f"RIGHT wp2 mirrored [deg] = {format_joints(radians_to_degrees(right_wp2), precision=1)}")
        log.info(f"RIGHT wp1 mirrored [rad] = {format_joints(right_wp1)}")
        log.info(f"RIGHT wp2 mirrored [rad] = {format_joints(right_wp2)}")

        client.wait_until_ready(timeout_sec=30.0)
        time.sleep(0.2)
        client.log_joint_summary("Robot state before test_dual_pick")

        log.info("==== DUAL PICK JOINT SEQUENCE ====")
        log.info(f"Step 0: ensure lift at {LIFT_HOME:.3f}")
        log.info("Step 1: add zone_b_box collision and MoveIt-plan to wp1")
        log.info(f"Step 2: move lift down to {LIFT_DOWN:.3f}")
        log.info("Step 3: remove zone_b_box collision, move joint2, wait, then move joint3 to wp2")
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

        left_traj_1, right_traj_1 = plan_wp1_with_moveit_then_mirror(
            client,
            left_wp1,
            duration_sec=float(ns.segment_duration_sec),
            timeout_sec=TIMEOUT_SEC,
        )
        result = client.execute_both(left_traj_1, right_traj_1, timeout_sec=TIMEOUT_SEC)
        log.info(f"wp1 result: {result}")
        if not result.both_succeeded:
            raise RuntimeError(f"wp1 execution failed: {result}")

        log.info(f"wp1 reached; waiting {WAIT_AFTER_WP1_SEC:.1f}s before lift motion")
        time.sleep(WAIT_AFTER_WP1_SEC)

        if not lift.move_to(
            LIFT_DOWN,
            duration_sec=LIFT_MOVE_DURATION_SEC,
            timeout_sec=LIFT_TIMEOUT_SEC,
        ):
            raise RuntimeError("failed to move lift down before wp2")

        log.info("[scene] wp2 joint2/joint3 이동 전 zone_b_box collision object 제거")
        remove_zone_b_box_collision(client)
        box_collision_enabled = False
        time.sleep(PLANNING_SCENE_SETTLE_SEC)

        left_traj_2, right_traj_2 = build_joint2_then_joint3_to_wp2(
            client,
            left_wp2,
            joint2_duration_sec=float(ns.joint2_duration_sec),
            pause_sec=float(ns.joint_pause_sec),
            joint3_duration_sec=float(ns.joint3_duration_sec),
            points=int(ns.joint_points),
        )
        result = client.execute_both(left_traj_2, right_traj_2, timeout_sec=TIMEOUT_SEC)
        log.info(f"wp2 joint2-then-joint3 result: {result}")
        if not result.both_succeeded:
            lift.move_to(
                LIFT_HOME,
                duration_sec=LIFT_MOVE_DURATION_SEC,
                timeout_sec=LIFT_TIMEOUT_SEC,
            )
            raise RuntimeError(f"wp2 joint2-then-joint3 execution failed: {result}")

        log.info(f"wp2 reached; waiting {LIFT_WAIT_AFTER_WP2_SEC:.1f}s")
        time.sleep(LIFT_WAIT_AFTER_WP2_SEC)

        if not lift.move_to(
            LIFT_HOME,
            duration_sec=LIFT_MOVE_DURATION_SEC,
            timeout_sec=LIFT_TIMEOUT_SEC,
        ):
            raise RuntimeError("failed to move lift home after wp2")

        client.log_joint_summary("Robot state after test_dual_pick")
        return 0

    except Exception as exc:
        log.error(f"test_dual_pick failed: {exc}")
        return 1

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


def main(args=None) -> None:
    ns, ros_args = parse_args(args=args)
    sys.exit(run(ns, ros_args))


if __name__ == "__main__":
    main()
