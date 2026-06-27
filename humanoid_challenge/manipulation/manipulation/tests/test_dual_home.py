from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_dual_client import (
    LEFT_ATTENTION_TARGET,
    LiftTrajectoryClient,
    RIGHT_ATTENTION_TARGET,
    SymmetricDualArmClient,
)


# =============================================================================
# 파라미터
# =============================================================================

ARM_MOVE_DURATION_SEC = 5.0
ARM_TRAJECTORY_POINTS = 30
TIMEOUT_SEC = 30.0

LIFT_HOME = 0.0
LIFT_MOVE_DURATION_SEC = 10.0
LIFT_TIMEOUT_SEC = 15.0

# =============================================================================


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("test_dual_home")
    log = node.get_logger()

    client = SymmetricDualArmClient(node, manage_executor=True)
    lift = LiftTrajectoryClient(node)

    try:
        log.info("test_dual_home moves to the full-extended attention pose")
        client.wait_until_ready(timeout_sec=30.0)
        time.sleep(0.2)
        client.log_joint_summary("Robot state before test_dual_home")

        log.info("==== DUAL HOME SEQUENCE ====")
        log.info(f"Step 0: ensure lift at {LIFT_HOME:.3f}")
        log.info(
            "Step 1: move both arms to the attention joint target "
            f"over {ARM_MOVE_DURATION_SEC:.1f}s"
        )
        log.info(f"LEFT_ATTENTION_TARGET : {[round(v, 4) for v in LEFT_ATTENTION_TARGET]}")
        log.info(f"RIGHT_ATTENTION_TARGET: {[round(v, 4) for v in RIGHT_ATTENTION_TARGET]}")

        if not lift.move_to(
            LIFT_HOME,
            duration_sec=LIFT_MOVE_DURATION_SEC,
            timeout_sec=LIFT_TIMEOUT_SEC,
        ):
            raise RuntimeError("failed to move lift to home before dual home")

        left_traj, right_traj = client.build_attention_trajectories(
            duration_sec=ARM_MOVE_DURATION_SEC,
            points=ARM_TRAJECTORY_POINTS,
        )
        log.info(f"left trajectory points : {len(left_traj.points)}")
        log.info(f"right trajectory points: {len(right_traj.points)}")

        result = client.execute_both(left_traj, right_traj, timeout_sec=TIMEOUT_SEC)
        log.info(f"dual_home result: {result}")
        if not result.both_succeeded:
            raise RuntimeError(f"dual_home execution failed: {result}")

        client.log_joint_summary("Robot state after test_dual_home")

    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
