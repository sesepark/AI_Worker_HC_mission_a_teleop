#!/usr/bin/env python3
"""
test_dual_home.py

Move both arms to a manually defined FULL-EXTENDED pose.

This script does NOT use build_home_trajectories().
Instead, it sends both arms directly to the joint targets below:

    LEFT_FULL_EXTENDED
    RIGHT_FULL_EXTENDED

The trajectories are sent directly to:

    /arm_l_controller/follow_joint_trajectory
    /arm_r_controller/follow_joint_trajectory
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ai_worker_manipulation.robot_interface.moveit_dual import (
    ARM_L_JOINTS,
    ARM_R_JOINTS,
    Arm,
    SymmetricDualArmClient,
    duration_from_seconds,
)


# =============================================================================
# USER CONFIG
# =============================================================================

# 여기 값을 "완전히 팔이 펴진 자세" joint 값으로 바꾸면 됩니다.
# 아래 0.0 값은 기본 예시입니다.
# 로봇에서 실제로 완전히 펴지는 값이 다를 수 있으니,
# RViz에서 원하는 자세를 만든 뒤 /joint_states 값을 읽어서 넣는 것이 가장 정확합니다.

LEFT_FULL_EXTENDED = [
    0.0,  # arm_l_joint1
    0.0,  # arm_l_joint2
    0.0,  # arm_l_joint3
    0.0,  # arm_l_joint4
    0.0,  # arm_l_joint5
    0.0,  # arm_l_joint6
    0.0,  # arm_l_joint7
]

RIGHT_FULL_EXTENDED = [
    0.0,  # arm_r_joint1
    0.0,  # arm_r_joint2
    0.0,  # arm_r_joint3
    0.0,  # arm_r_joint4
    0.0,  # arm_r_joint5
    0.0,  # arm_r_joint6
    0.0,  # arm_r_joint7
]

DEFAULT_DURATION_SEC = 5.0
DEFAULT_POINTS = 30
DEFAULT_TIMEOUT_SEC = 30.0

# =============================================================================
# END USER CONFIG
# =============================================================================


def smoothstep(alpha: float) -> float:
    return 3.0 * alpha * alpha - 2.0 * alpha * alpha * alpha


def build_joint_target_trajectory(
    joint_names: list[str],
    start_joints: list[float],
    target_joints: list[float],
    duration_sec: float,
    points: int,
) -> JointTrajectory:
    if len(start_joints) != len(joint_names):
        raise ValueError(
            f"start_joints length mismatch: expected {len(joint_names)}, "
            f"got {len(start_joints)}"
        )

    if len(target_joints) != len(joint_names):
        raise ValueError(
            f"target_joints length mismatch: expected {len(joint_names)}, "
            f"got {len(target_joints)}"
        )

    if duration_sec <= 0.0:
        raise ValueError("duration_sec must be positive")

    if points < 2:
        raise ValueError("points must be >= 2")

    traj = JointTrajectory()
    traj.joint_names = list(joint_names)

    for i in range(points):
        alpha = i / float(points - 1)
        s = smoothstep(alpha)

        point = JointTrajectoryPoint()
        point.positions = [
            float(start_joints[j]) + s * (float(target_joints[j]) - float(start_joints[j]))
            for j in range(len(joint_names))
        ]
        point.time_from_start = duration_from_seconds(alpha * duration_sec)

        traj.points.append(point)

    return traj


def round_list(values: list[float] | None, digits: int = 4) -> list[float] | None:
    if values is None:
        return None
    return [round(float(v), digits) for v in values]


def print_joint_summary(
    node: Node,
    client: SymmetricDualArmClient,
    title: str,
) -> None:
    log = node.get_logger()
    log.info(f"==== {title} ====")

    left = client.get_joints(Arm.LEFT)
    right = client.get_joints(Arm.RIGHT)

    if left is None:
        log.error("left joint state unavailable")
    else:
        log.info(f"[left]  {dict(zip(ARM_L_JOINTS, round_list(left)))}")

    if right is None:
        log.error("right joint state unavailable")
    else:
        log.info(f"[right] {dict(zip(ARM_R_JOINTS, round_list(right)))}")


def describe_controller_result(result) -> str:
    parts = []

    result_value = getattr(result, "result", None)
    if result_value is not None:
        value = getattr(result_value, "value", result_value)
        parts.append(f"result={value}")

    status = getattr(result, "status", None)
    if status is not None:
        parts.append(f"status={status}")

    message = getattr(result, "message", None)
    if message:
        parts.append(f"message={message}")

    if not parts:
        return repr(result)

    return ", ".join(parts)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Move both arms to manually defined full-extended joint targets."
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_SEC,
        help=f"trajectory duration in seconds, default: {DEFAULT_DURATION_SEC}",
    )

    parser.add_argument(
        "--points",
        type=int,
        default=DEFAULT_POINTS,
        help=f"number of trajectory points, default: {DEFAULT_POINTS}",
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_SEC,
        help=f"controller action timeout in seconds, default: {DEFAULT_TIMEOUT_SEC}",
    )

    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="build trajectories but do not execute controller goals",
    )

    ns = parser.parse_args(args=args)

    if ns.duration <= 0.0:
        raise ValueError("--duration must be positive")

    if ns.points < 2:
        raise ValueError("--points must be >= 2")

    if ns.timeout <= 0.0:
        raise ValueError("--timeout must be positive")

    rclpy.init(args=args)

    node = Node("test_dual_home")
    log = node.get_logger()
    client = SymmetricDualArmClient(node, manage_executor=True)

    exit_code = 1

    try:
        log.info("==== test_dual_home: move both arms to FULL_EXTENDED pose ====")
        log.info(
            f"duration={ns.duration:.2f}s, "
            f"points={ns.points}, "
            f"timeout={ns.timeout:.2f}s, "
            f"plan_only={ns.plan_only}"
        )

        log.info(f"LEFT_FULL_EXTENDED : {[round(v, 4) for v in LEFT_FULL_EXTENDED]}")
        log.info(f"RIGHT_FULL_EXTENDED: {[round(v, 4) for v in RIGHT_FULL_EXTENDED]}")

        client.wait_until_ready(timeout_sec=10.0)
        time.sleep(0.2)

        print_joint_summary(node, client, "Robot state before test_dual_home")

        left_start = client.get_joints(Arm.LEFT)
        right_start = client.get_joints(Arm.RIGHT)

        if left_start is None:
            raise RuntimeError("left joint states unavailable")

        if right_start is None:
            raise RuntimeError("right joint states unavailable")

        left_traj = build_joint_target_trajectory(
            joint_names=list(ARM_L_JOINTS),
            start_joints=list(left_start),
            target_joints=LEFT_FULL_EXTENDED,
            duration_sec=ns.duration,
            points=ns.points,
        )

        right_traj = build_joint_target_trajectory(
            joint_names=list(ARM_R_JOINTS),
            start_joints=list(right_start),
            target_joints=RIGHT_FULL_EXTENDED,
            duration_sec=ns.duration,
            points=ns.points,
        )

        log.info(f"left trajectory points : {len(left_traj.points)}")
        log.info(f"right trajectory points: {len(right_traj.points)}")

        if ns.plan_only:
            log.info("plan-only requested; not executing controller goals")
            exit_code = 0
        else:
            log.info("sending FULL_EXTENDED trajectories to both arm controllers")
            result = client.execute_both(
                left_traj,
                right_traj,
                timeout_sec=ns.timeout,
            )

            log.info(f"Result: {result}")
            log.info(f"Left:  {describe_controller_result(result.left)}")
            log.info(f"Right: {describe_controller_result(result.right)}")

            exit_code = 0 if result.both_succeeded else 1

        time.sleep(0.5)
        print_joint_summary(node, client, "Robot state after test_dual_home")

    except Exception as exc:
        log.error(f"test_dual_home failed: {exc}")
        log.error(traceback.format_exc())
        exit_code = 1

    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()