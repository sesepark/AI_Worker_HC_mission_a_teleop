#!/usr/bin/env python3
"""
moveit_dual.py

Dual-arm helper for FFW arms when the goal is symmetric motion.

Key idea
--------
Do not send two simultaneous goals to MoveIt's single /move_action server.
Instead:
  1) filter fixed, file-defined waypoint poses with the same policy as
     task_pose_selector.py; no GPD data or candidate selection is used (workspace, orientation envelope, pre-grasp,
     reachable checks, planner cascade),
  2) generate the fixed LEFT-arm waypoint path with pymoveit2.MoveIt2.plan_async(),
     matching moveit_client.py's pose-planning flow,
  3) mirror it for the other arm when symmetric motion is required,
  4) send the final JointTrajectory goals directly to
     /arm_l_controller/follow_joint_trajectory and
     /arm_r_controller/follow_joint_trajectory together.

This file intentionally supports a mirrored joint trajectory workflow because a
box-carrying task requires the two hands to move as a coupled pair, not as two
independent MoveIt pose plans.
"""

from __future__ import annotations

import copy
import math
import os
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from pymoveit2 import MoveIt2
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Arm(Enum):
    LEFT = "left"
    RIGHT = "right"


class MotionResult(Enum):
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    REJECTED = "rejected"
    TIMEOUT = "timeout"
    INVALID = "invalid"


@dataclass
class ArmExecutionResult:
    result: MotionResult
    status: Optional[int] = None
    message: str = ""
    started_at: Optional[float] = None
    finished_at: Optional[float] = None


@dataclass
class DualArmExecutionResult:
    left: ArmExecutionResult
    right: ArmExecutionResult

    @property
    def both_succeeded(self) -> bool:
        return self.left.result == MotionResult.SUCCEEDED and self.right.result == MotionResult.SUCCEEDED

    @property
    def dispatch_skew_sec(self) -> Optional[float]:
        if self.left.started_at is None or self.right.started_at is None:
            return None
        return abs(self.left.started_at - self.right.started_at)

    @property
    def finish_skew_sec(self) -> Optional[float]:
        if self.left.finished_at is None or self.right.finished_at is None:
            return None
        return abs(self.left.finished_at - self.right.finished_at)

    def __repr__(self) -> str:
        dispatch = "" if self.dispatch_skew_sec is None else f", dispatch_skew={self.dispatch_skew_sec:.3f}s"
        finish = "" if self.finish_skew_sec is None else f", finish_skew={self.finish_skew_sec:.3f}s"
        return (
            "DualArmExecutionResult("
            f"left={self.left.result.value}, right={self.right.result.value}"
            f"{dispatch}{finish})"
        )


ARM_L_JOINTS = [
    "arm_l_joint1", "arm_l_joint2", "arm_l_joint3",
    "arm_l_joint4", "arm_l_joint5", "arm_l_joint6", "arm_l_joint7",
]
ARM_R_JOINTS = [
    "arm_r_joint1", "arm_r_joint2", "arm_r_joint3",
    "arm_r_joint4", "arm_r_joint5", "arm_r_joint6", "arm_r_joint7",
]
ARM_JOINTS = {Arm.LEFT: ARM_L_JOINTS, Arm.RIGHT: ARM_R_JOINTS}
ARM_CONTROLLER_ACTION = {
    Arm.LEFT: "/arm_l_controller/follow_joint_trajectory",
    Arm.RIGHT: "/arm_r_controller/follow_joint_trajectory",
}
ARM_GROUP = {Arm.LEFT: "arm_l", Arm.RIGHT: "arm_r"}
ARM_EEF = {Arm.LEFT: "end_effector_l_link", Arm.RIGHT: "end_effector_r_link"}
BASE_LINK = "base_link"

# Empirical symmetric reference from your current hardcoded home pose.
# Mirroring is applied to DELTAS around this pair, not by raw sign flipping.
LEFT_SYMMETRY_REF = [-0.587, 0.046, 0.090, 0.912, -0.095, -0.315, 0.048]
RIGHT_SYMMETRY_REF = [-0.594, -0.021, -0.040, 0.914, 0.056, -0.310, -0.022]

# From observed SG2 behavior: joint1,4,6 are same-direction; joint2,3,5,7 mirror sign.
MIRROR_SIGNS_L_TO_R = [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0]

# Same planning defaults used by moveit_client.py.
DEFAULT_PLANNING_TIME = float(os.environ.get("MOVEIT_PLANNING_TIME", "5.0"))
DEFAULT_PLANNING_ATTEMPTS = int(os.environ.get("MOVEIT_PLANNING_ATTEMPTS", "5"))
POSE_TOL_POSITION = float(os.environ.get("MOVEIT_POSE_TOL_POSITION", "0.001"))
POSE_TOL_ORIENTATION = float(os.environ.get("MOVEIT_POSE_TOL_ORIENTATION", "0.01"))

# MoveIt errors that mean planning/IK failed before controller execution.
PLANNING_ERROR_CODES = {
    MoveItErrorCodes.FAILURE,
    MoveItErrorCodes.NO_IK_SOLUTION,
    MoveItErrorCodes.PLANNING_FAILED,
    MoveItErrorCodes.INVALID_MOTION_PLAN,
    MoveItErrorCodes.GOAL_IN_COLLISION,
    MoveItErrorCodes.GOAL_STATE_INVALID,
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED,
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS,
    MoveItErrorCodes.START_STATE_INVALID,
    MoveItErrorCodes.START_STATE_IN_COLLISION,
}

# TaskPoseSelector-style path-generation defaults. The dual-arm client keeps
# its existing mirror + simultaneous-controller execution strategy. It does NOT
# fetch GPD data or choose among perception candidates here; the caller passes
# fixed waypoint poses, and each waypoint is filtered/planned through the
# task_pose_selector.py policy:
# workspace envelope, orientation envelope, target/prepose IK checks, planner
# cascade and path-quality scoring.
TASK_PRE_GRASP_OFFSET = float(os.environ.get("TASK_PRE_GRASP_OFFSET", "0.15"))
TASK_WORKSPACE_BOUNDS = {
    "x": (0.03, 0.40),
    "y": (-0.55, 0.55),
    "z": (0.50, 1.10),
}
TASK_ORIENTATION_FILTER = {
    "enabled": True,
    "max_abs_roll_deg": 70.0,
    "max_abs_pitch_deg": 70.0,
    "reject_topdown_pitch_deg": 80.0,
}
TASK_PATH_QUALITY = {
    "max_joint_path_length": 3.5,
    "max_joint_step": 1.0,
}
TASK_SCORING = {
    "joint_path_weight": 1.0,
    "max_joint_step_weight": 2.0,
    "planning_time_weight": 0.05,
}
TASK_PLANNER_SPECS = [
    ("pilz_industrial_motion_planner", "PTP"),
    ("stomp", "STOMP"),
    ("ompl", "RRTstar"),
    ("ompl", "LBKPIECE"),
]



# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------


def duration_from_seconds(seconds: float) -> Duration:
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


def copy_point_time(point: JointTrajectoryPoint) -> Duration:
    return Duration(sec=point.time_from_start.sec, nanosec=point.time_from_start.nanosec)


def copy_pose(pose: Pose) -> Pose:
    copied = Pose()
    copied.position.x = float(pose.position.x)
    copied.position.y = float(pose.position.y)
    copied.position.z = float(pose.position.z)
    copied.orientation.x = float(pose.orientation.x)
    copied.orientation.y = float(pose.orientation.y)
    copied.orientation.z = float(pose.orientation.z)
    copied.orientation.w = float(pose.orientation.w)
    return copied


def pose_to_str(pose: Pose) -> str:
    p = pose.position
    q = pose.orientation
    return (
        f"pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), "
        f"quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})"
    )




def pose_position_distance(a: Pose, b: Pose) -> float:
    return math.sqrt(
        (float(a.position.x) - float(b.position.x)) ** 2
        + (float(a.position.y) - float(b.position.y)) ** 2
        + (float(a.position.z) - float(b.position.z)) ** 2
    )


def quaternion_angle_distance(a: Pose, b: Pose) -> float:
    aq = a.orientation
    bq = b.orientation
    an = math.sqrt(aq.x * aq.x + aq.y * aq.y + aq.z * aq.z + aq.w * aq.w)
    bn = math.sqrt(bq.x * bq.x + bq.y * bq.y + bq.z * bq.z + bq.w * bq.w)
    if an < 1.0e-9 or bn < 1.0e-9:
        return math.inf
    dot = abs((aq.x / an) * (bq.x / bn) + (aq.y / an) * (bq.y / bn) + (aq.z / an) * (bq.z / bn) + (aq.w / an) * (bq.w / bn))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def make_single_point_trajectory(joint_names: list[str], joints: list[float], *, time_sec: float = 0.0) -> JointTrajectory:
    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    point = JointTrajectoryPoint()
    point.positions = list(joints)
    point.time_from_start = duration_from_seconds(float(time_sec))
    traj.points.append(point)
    return traj

def quaternion_to_rpy_deg(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Return roll, pitch, yaw in degrees from quaternion.

    This avoids adding scipy as a runtime dependency in moveit_dual.py while
    matching task_pose_selector.py's use of xyz Euler angles for the roll/pitch
    orientation envelope.
    """
    # Normalize first to avoid domain errors from slightly non-unit quaternions.
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1.0e-9:
        raise ValueError("quaternion norm is zero")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def make_task_pre_pose(target_pose: Pose, offset: float = TASK_PRE_GRASP_OFFSET) -> Pose:
    """Return the same pre-grasp pose policy used by task_pose_selector.py.

    task_pose_selector.py delegates this to pick_skill.pre_grasp_of(). Import it
    lazily so moveit_dual.py does not create package import cycles at startup.
    If the helper is not available, fall back to a simple upward Z offset.
    """
    try:
        from ai_worker_manipulation.skill_primitives.pick_skill import pre_grasp_of

        return pre_grasp_of(target_pose, offset=float(offset))
    except Exception:
        pre = copy_pose(target_pose)
        pre.position.z = float(pre.position.z) + float(offset)
        return pre


def task_workspace_ok(pose: Pose) -> bool:
    p = pose.position
    bounds = TASK_WORKSPACE_BOUNDS
    return (
        bounds["x"][0] <= p.x <= bounds["x"][1]
        and bounds["y"][0] <= p.y <= bounds["y"][1]
        and bounds["z"][0] <= p.z <= bounds["z"][1]
    )


def task_orientation_ok(pose: Pose) -> bool:
    """Return True if pose passes task_pose_selector.py's orientation envelope."""
    filt = TASK_ORIENTATION_FILTER
    if not filt.get("enabled", True):
        return True
    q = pose.orientation
    try:
        roll, pitch, _yaw = quaternion_to_rpy_deg(q.x, q.y, q.z, q.w)
    except ValueError:
        return False

    max_roll = float(filt["max_abs_roll_deg"])
    max_pitch = float(filt["max_abs_pitch_deg"])
    reject_topdown = float(filt["reject_topdown_pitch_deg"])
    return (
        abs(roll) <= max_roll
        and abs(pitch) <= max_pitch
        and abs(pitch) < reject_topdown
    )


def mirror_left_joints_to_right(
    left_joints: list[float],
    *,
    left_ref: list[float] = LEFT_SYMMETRY_REF,
    right_ref: list[float] = RIGHT_SYMMETRY_REF,
    signs: list[float] = MIRROR_SIGNS_L_TO_R,
) -> list[float]:
    """Mirror a left-arm joint state into right-arm joint space.

    This uses calibrated reference poses:
        right = right_ref + sign * (left - left_ref)

    This is safer than raw sign flipping because your two home poses are not
    numerically identical around zero.
    """
    if len(left_joints) != 7:
        raise ValueError(f"left_joints must have length 7, got {len(left_joints)}")
    return [right_ref[i] + signs[i] * (left_joints[i] - left_ref[i]) for i in range(7)]


def mirror_left_trajectory_to_right(left_traj: JointTrajectory) -> JointTrajectory:
    right_traj = JointTrajectory()
    right_traj.joint_names = list(ARM_R_JOINTS)

    for lp in left_traj.points:
        rp = JointTrajectoryPoint()
        rp.positions = mirror_left_joints_to_right(list(lp.positions))
        if lp.velocities:
            rp.velocities = [MIRROR_SIGNS_L_TO_R[i] * lp.velocities[i] for i in range(7)]
        if lp.accelerations:
            rp.accelerations = [MIRROR_SIGNS_L_TO_R[i] * lp.accelerations[i] for i in range(7)]
        rp.time_from_start = copy_point_time(lp)
        right_traj.points.append(rp)
    return right_traj


def make_interpolated_trajectory(
    joint_names: list[str],
    start: list[float],
    target: list[float],
    *,
    duration_sec: float = 4.0,
    points: int = 25,
) -> JointTrajectory:
    if len(start) != len(target) or len(start) != len(joint_names):
        raise ValueError("joint_names, start, and target must have matching lengths")
    if points < 2:
        raise ValueError("points must be >= 2")

    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    for i in range(points):
        s = i / float(points - 1)
        # smoothstep profile: zero velocity at endpoints when sampled as positions.
        a = 3.0 * s * s - 2.0 * s * s * s
        p = JointTrajectoryPoint()
        p.positions = [start[j] + a * (target[j] - start[j]) for j in range(len(start))]
        p.time_from_start = duration_from_seconds(duration_sec * s)
        traj.points.append(p)
    return traj


def retime_trajectory(traj: JointTrajectory, duration_sec: float) -> JointTrajectory:
    """Return a copy of traj with point times scaled to duration_sec."""
    out = copy.deepcopy(traj)
    if not out.points:
        return out
    last = out.points[-1].time_from_start.sec + out.points[-1].time_from_start.nanosec * 1e-9
    if last <= 1e-9:
        # If source has no useful time, distribute uniformly.
        n = len(out.points)
        for i, p in enumerate(out.points):
            p.time_from_start = duration_from_seconds(duration_sec * i / max(n - 1, 1))
        return out
    scale = duration_sec / last
    for p in out.points:
        old = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
        p.time_from_start = duration_from_seconds(old * scale)
    return out


def trajectory_duration_seconds(traj: JointTrajectory) -> float:
    """Return the final time_from_start of a trajectory in seconds."""
    if not traj.points:
        return 0.0
    last = traj.points[-1].time_from_start
    return float(last.sec) + float(last.nanosec) * 1e-9


def point_time_seconds(point: JointTrajectoryPoint) -> float:
    return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9


def append_trajectory_segment(
    target: JointTrajectory,
    segment: JointTrajectory,
    *,
    time_offset_sec: float,
    skip_first_point: bool,
) -> None:
    """Append segment into target while preserving relative timing.

    MoveIt returns each planned segment with its own time_from_start beginning
    near 0. For a multi-waypoint test we need one continuous controller
    trajectory, so each segment is shifted by the accumulated duration.
    """
    if target.joint_names != segment.joint_names:
        raise ValueError(
            f"cannot append trajectory with different joints: "
            f"target={target.joint_names}, segment={segment.joint_names}"
        )
    for index, src in enumerate(segment.points):
        if skip_first_point and index == 0:
            continue
        dst = copy.deepcopy(src)
        dst.time_from_start = duration_from_seconds(time_offset_sec + point_time_seconds(src))
        target.points.append(dst)


def strip_to_arm_joints(traj: JointTrajectory, arm: Arm) -> JointTrajectory:
    """Reorder/filter a MoveIt trajectory so it exactly matches the arm controller joints."""
    desired = ARM_JOINTS[arm]
    idx = []
    for name in desired:
        if name not in traj.joint_names:
            raise ValueError(f"trajectory is missing joint {name}; names={traj.joint_names}")
        idx.append(traj.joint_names.index(name))

    out = JointTrajectory()
    out.joint_names = list(desired)
    for src in traj.points:
        dst = JointTrajectoryPoint()
        dst.positions = [src.positions[i] for i in idx]
        if src.velocities:
            dst.velocities = [src.velocities[i] for i in idx]
        if src.accelerations:
            dst.accelerations = [src.accelerations[i] for i in idx]
        dst.time_from_start = copy_point_time(src)
        out.points.append(dst)
    return out


# ---------------------------------------------------------------------------
# Main client
# ---------------------------------------------------------------------------


class SymmetricDualArmClient:
    """Direct controller executor + optional MoveIt plan-only helper."""

    def __init__(self, node: Node, *, manage_executor: bool = True) -> None:
        self.node = node
        self.log = node.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        self._joint_state: Optional[JointState] = None
        self._joint_lock = threading.Lock()
        self._js_sub = node.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            20,
            callback_group=self.callback_group,
        )

        self._action_clients = {
            Arm.LEFT: ActionClient(
                node,
                FollowJointTrajectory,
                ARM_CONTROLLER_ACTION[Arm.LEFT],
                callback_group=self.callback_group,
            ),
            Arm.RIGHT: ActionClient(
                node,
                FollowJointTrajectory,
                ARM_CONTROLLER_ACTION[Arm.RIGHT],
                callback_group=self.callback_group,
            ),
        }

        # MoveIt path generation follows moveit_client.py: use pymoveit2.MoveIt2
        # and plan_async(pose=...) instead of manually calling IK then planning
        # to joint constraints. These handles are only used for planning; final
        # execution still goes directly to both arm controllers in execute_both().
        self._moveit2 = {
            Arm.LEFT: MoveIt2(
                node=self.node,
                joint_names=ARM_L_JOINTS,
                base_link_name=BASE_LINK,
                end_effector_name=ARM_EEF[Arm.LEFT],
                group_name=ARM_GROUP[Arm.LEFT],
                callback_group=self.callback_group,
                use_move_group_action=True,
            ),
            Arm.RIGHT: MoveIt2(
                node=self.node,
                joint_names=ARM_R_JOINTS,
                base_link_name=BASE_LINK,
                end_effector_name=ARM_EEF[Arm.RIGHT],
                group_name=ARM_GROUP[Arm.RIGHT],
                callback_group=self.callback_group,
                use_move_group_action=True,
            ),
        }
        self._moveit_locks = {Arm.LEFT: threading.Lock(), Arm.RIGHT: threading.Lock()}

        # Kept for backwards-compatible helper methods below, but the main
        # pose path now uses MoveIt2.plan_async() like moveit_client.py.
        self._ik_client = node.create_client(
            GetPositionIK,
            "/compute_ik",
            callback_group=self.callback_group,
        )
        self._plan_client = node.create_client(
            GetMotionPlan,
            "/plan_kinematic_path",
            callback_group=self.callback_group,
        )

        self._executor: Optional[MultiThreadedExecutor] = None
        self._executor_thread: Optional[threading.Thread] = None
        if manage_executor:
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(node)
            self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._executor_thread.start()
            self.log.info("SymmetricDualArmClient internal executor started.")

    def destroy(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._executor_thread is not None:
            self._executor_thread.join(timeout=5.0)

    def _on_joint_state(self, msg: JointState) -> None:
        with self._joint_lock:
            self._joint_state = msg

    def wait_until_ready(self, timeout_sec: float = 10.0) -> None:
        deadline = time.time() + timeout_sec
        for arm, client in self._action_clients.items():
            self.log.info(f"Waiting for {ARM_CONTROLLER_ACTION[arm]} ...")
            while not client.wait_for_server(timeout_sec=0.5):
                if time.time() > deadline:
                    raise RuntimeError(f"Timed out waiting for {ARM_CONTROLLER_ACTION[arm]}")
                self.log.warn(f"{ARM_CONTROLLER_ACTION[arm]} not available yet")

        self.log.info("Waiting for /joint_states ...")
        while True:
            if self._joint_state is not None:
                break
            if time.time() > deadline:
                raise RuntimeError("Timed out waiting for /joint_states")
            time.sleep(0.05)
        self.log.info("Dual arm controller clients and joint states are ready.")

    def wait_for_moveit_services(self, timeout_sec: float = 10.0) -> None:
        deadline = time.time() + timeout_sec
        self.log.info("Waiting for MoveIt services: /compute_ik, /plan_kinematic_path ...")
        while not self._ik_client.wait_for_service(timeout_sec=0.5):
            if time.time() > deadline:
                raise RuntimeError("Timed out waiting for /compute_ik")
        while not self._plan_client.wait_for_service(timeout_sec=0.5):
            if time.time() > deadline:
                raise RuntimeError("Timed out waiting for /plan_kinematic_path")
        self.log.info("MoveIt services are ready.")

    def wait_for_moveit_servers(self, timeout_sec: float = 10.0) -> None:
        """Wait for MoveIt's move_group action clients used by pymoveit2."""
        deadline = time.time() + timeout_sec
        for arm, moveit2 in self._moveit2.items():
            self.log.info(f"Waiting for move_group action server [{ARM_GROUP[arm]}] ...")
            client = moveit2._MoveIt2__move_action_client
            while not client.wait_for_server(timeout_sec=0.5):
                if time.time() > deadline:
                    raise RuntimeError(f"Timed out waiting for move_group action server [{ARM_GROUP[arm]}]")
                self.log.warn(f"move_group action server [{ARM_GROUP[arm]}] not available yet")
            while not client.server_is_ready():
                if time.time() > deadline:
                    raise RuntimeError(f"move_group action server [{ARM_GROUP[arm]}] did not become ready")
                time.sleep(0.05)
        self.log.info("MoveIt move_group action servers are ready.")

    def _moveit(self, arm: Arm) -> MoveIt2:
        return self._moveit2[arm]

    def _configure_moveit(
        self,
        moveit2: MoveIt2,
        *,
        velocity: float,
        acceleration: float,
        pipeline: str,
        planner: str,
    ) -> None:
        # pymoveit2 exposes this typo as a public attribute, as in moveit_client.py.
        moveit2.motion_suceeded = False
        moveit2.pipeline_id = pipeline
        moveit2.planner_id = planner
        moveit2.max_velocity = float(velocity)
        moveit2.max_acceleration = float(acceleration)
        moveit2.allowed_planning_time = DEFAULT_PLANNING_TIME
        moveit2.num_planning_attempts = DEFAULT_PLANNING_ATTEMPTS

    @staticmethod
    def _classify_error_code(error_code: int) -> MotionResult:
        if error_code == MoveItErrorCodes.SUCCESS:
            return MotionResult.SUCCEEDED
        if error_code in PLANNING_ERROR_CODES:
            return MotionResult.INVALID
        return MotionResult.FAILED

    @staticmethod
    def _trajectory_metrics(trajectory: JointTrajectory) -> dict:
        points = list(getattr(trajectory, "points", []))
        metrics = {
            "point_count": len(points),
            "joint_path_length": 0.0,
            "max_joint_step": 0.0,
            "planned_duration_s": 0.0,
        }
        if not points:
            return metrics

        previous = None
        for point in points:
            current = list(point.positions)
            if previous is not None and current and len(current) == len(previous):
                step = math.sqrt(sum((a - b) ** 2 for a, b in zip(current, previous)))
                metrics["joint_path_length"] += step
                metrics["max_joint_step"] = max(metrics["max_joint_step"], step)
            previous = current

        final_time = points[-1].time_from_start
        metrics["planned_duration_s"] = float(final_time.sec) + float(final_time.nanosec) / 1e9
        metrics["joint_path_length"] = round(metrics["joint_path_length"], 6)
        metrics["max_joint_step"] = round(metrics["max_joint_step"], 6)
        metrics["planned_duration_s"] = round(metrics["planned_duration_s"], 6)
        return metrics

    def _log_pose(self, label: str, arm: Arm, pose: Pose, velocity: float, acceleration: float) -> None:
        p = pose.position
        o = pose.orientation
        self.log.info(
            f"[{label}] [{arm.value}] "
            f"pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
            f"quat=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f}) "
            f"| vel={velocity} acc={acceleration} "
            f"tol_pos={POSE_TOL_POSITION} tol_ori={POSE_TOL_ORIENTATION}"
        )

    def _wait_future(self, future, timeout_sec: float):
        deadline = time.time() + timeout_sec
        while not future.done():
            if time.time() > deadline:
                raise TimeoutError("future timeout")
            time.sleep(0.01)
        return future.result()

    def get_joints(self, arm: Arm) -> Optional[list[float]]:
        names = ARM_JOINTS[arm]
        with self._joint_lock:
            js = copy.deepcopy(self._joint_state)
        if js is None:
            return None
        name_to_pos = dict(zip(js.name, js.position))
        if not all(name in name_to_pos for name in names):
            missing = [n for n in names if n not in name_to_pos]
            self.log.error(f"Missing joints in /joint_states for {arm.value}: {missing}")
            return None
        return [float(name_to_pos[n]) for n in names]

    def current_robot_state(self) -> RobotState:
        with self._joint_lock:
            js = copy.deepcopy(self._joint_state)
        if js is None:
            raise RuntimeError("joint_states unavailable")
        state = RobotState()
        state.joint_state = js
        return state

    def get_pose(self, arm: Arm = Arm.LEFT, timeout_sec: float = 5.0) -> Optional[Pose]:
        """Return current end-effector pose through MoveIt FK.

        This is used to avoid asking MoveIt to plan a zero-distance segment
        when the first file-defined waypoint is already the current EE pose.
        Some MoveIt configurations return a generic planning failure for that
        no-op request, even though the robot is already at the goal.
        """
        moveit2 = self._moveit(arm)
        future = moveit2.compute_fk_async()
        if future is None:
            self.log.warn(f"[get_pose] [{arm.value}] compute_fk_async returned None")
            return None
        try:
            result = self._wait_future(future, timeout_sec=timeout_sec)
            fk = moveit2.get_compute_fk_result(future)
        except Exception as exc:
            self.log.warn(f"[get_pose] [{arm.value}] FK failed: {exc}")
            return None
        if fk is None:
            self.log.warn(f"[get_pose] [{arm.value}] FK returned no pose")
            return None
        return fk.pose

    def _is_current_pose_goal(
        self,
        pose: Pose,
        arm: Arm,
        *,
        position_tolerance: float = 0.01,
        orientation_tolerance: float = 0.10,
    ) -> bool:
        current = self.get_pose(arm)
        if current is None:
            return False
        pos_err = pose_position_distance(current, pose)
        ori_err = quaternion_angle_distance(current, pose)
        self.log.info(
            f"[waypoint-plan] current-pose check [{arm.value}] "
            f"pos_err={pos_err:.4f}m ori_err={ori_err:.4f}rad "
            f"target={pose_to_str(pose)} current={pose_to_str(current)}"
        )
        return pos_err <= position_tolerance and ori_err <= orientation_tolerance

    # ------------------------------------------------------------------
    # Direct controller execution
    # ------------------------------------------------------------------

    def execute_both(self, left_traj: JointTrajectory, right_traj: JointTrajectory, timeout_sec: float = 30.0) -> DualArmExecutionResult:
        if left_traj.joint_names != ARM_L_JOINTS:
            raise ValueError(f"left trajectory joint_names must be {ARM_L_JOINTS}, got {left_traj.joint_names}")
        if right_traj.joint_names != ARM_R_JOINTS:
            raise ValueError(f"right trajectory joint_names must be {ARM_R_JOINTS}, got {right_traj.joint_names}")
        if not left_traj.points or not right_traj.points:
            raise ValueError("both trajectories must have at least one point")

        barrier = threading.Barrier(2)
        results: dict[Arm, ArmExecutionResult] = {}

        def worker(arm: Arm, traj: JointTrajectory) -> None:
            client = self._action_clients[arm]
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            try:
                barrier.wait(timeout=5.0)
                started = time.time()
                send_future = client.send_goal_async(goal)
                goal_handle = self._wait_future(send_future, timeout_sec=5.0)
                if not goal_handle.accepted:
                    results[arm] = ArmExecutionResult(MotionResult.REJECTED, message="goal rejected", started_at=started, finished_at=time.time())
                    return
                result_future = goal_handle.get_result_async()
                wrapped = self._wait_future(result_future, timeout_sec=timeout_sec)
                status = int(wrapped.status)
                if status == GoalStatus.STATUS_SUCCEEDED:
                    motion_result = MotionResult.SUCCEEDED
                else:
                    motion_result = MotionResult.FAILED
                message = getattr(wrapped.result, "error_string", "")
                results[arm] = ArmExecutionResult(motion_result, status=status, message=message, started_at=started, finished_at=time.time())
            except TimeoutError as exc:
                results[arm] = ArmExecutionResult(MotionResult.TIMEOUT, message=str(exc), started_at=time.time(), finished_at=time.time())
            except Exception as exc:
                results[arm] = ArmExecutionResult(MotionResult.FAILED, message=repr(exc), started_at=time.time(), finished_at=time.time())

        self.log.info("[dual/direct] sending both FollowJointTrajectory goals together")
        t_left = threading.Thread(target=worker, args=(Arm.LEFT, left_traj), daemon=True)
        t_right = threading.Thread(target=worker, args=(Arm.RIGHT, right_traj), daemon=True)
        t_left.start()
        t_right.start()
        t_left.join(timeout_sec + 10.0)
        t_right.join(timeout_sec + 10.0)

        left = results.get(Arm.LEFT, ArmExecutionResult(MotionResult.TIMEOUT, message="left thread did not return"))
        right = results.get(Arm.RIGHT, ArmExecutionResult(MotionResult.TIMEOUT, message="right thread did not return"))
        result = DualArmExecutionResult(left=left, right=right)
        self.log.info(f"[dual/direct] {result}")
        return result

    def build_home_trajectories(self, duration_sec: float = 4.0, points: int = 25) -> tuple[JointTrajectory, JointTrajectory]:
        left_current = self.get_joints(Arm.LEFT)
        right_current = self.get_joints(Arm.RIGHT)
        if left_current is None or right_current is None:
            raise RuntimeError("joint states unavailable")
        left_traj = make_interpolated_trajectory(ARM_L_JOINTS, left_current, LEFT_SYMMETRY_REF, duration_sec=duration_sec, points=points)
        right_traj = make_interpolated_trajectory(ARM_R_JOINTS, right_current, RIGHT_SYMMETRY_REF, duration_sec=duration_sec, points=points)
        return left_traj, right_traj

    def build_symmetric_trajectories_from_left_target(
        self,
        left_target: list[float],
        *,
        duration_sec: float = 4.0,
        points: int = 25,
    ) -> tuple[JointTrajectory, JointTrajectory]:
        """Build a mirrored pair from a left-arm target.

        The left arm interpolates from its current state to left_target.
        The right arm follows the calibrated mirror of the left path.
        The first right point is forced to current right joints to avoid an abrupt jump if
        the robot is not exactly symmetric at the start.
        """
        left_current = self.get_joints(Arm.LEFT)
        right_current = self.get_joints(Arm.RIGHT)
        if left_current is None or right_current is None:
            raise RuntimeError("joint states unavailable")
        left_traj = make_interpolated_trajectory(ARM_L_JOINTS, left_current, left_target, duration_sec=duration_sec, points=points)
        right_traj = mirror_left_trajectory_to_right(left_traj)
        if right_traj.points:
            right_traj.points[0].positions = list(right_current)
        return left_traj, right_traj

    # ------------------------------------------------------------------
    # MoveIt plan helpers
    # ------------------------------------------------------------------

    def plan_to_pose_details(
        self,
        pose: Pose,
        arm: Arm = Arm.LEFT,
        *,
        velocity: float = 0.05,
        acceleration: float = 0.05,
        timeout_sec: float = 10.0,
        pipeline: str = "ompl",
        planner: str = "RRTConnect",
        start_joint_state: Optional[list[float]] = None,
        cartesian: bool = False,
        cartesian_max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.999,
    ) -> dict:
        """Plan to a Cartesian pose using moveit_client.py's MoveIt2.plan_async flow.

        This method intentionally does not execute the motion. The returned
        JointTrajectory can be mirrored and then passed to execute_both().
        """
        self.wait_for_moveit_servers(timeout_sec=timeout_sec)
        label = "plan_cartesian" if cartesian else "plan_to_pose"
        self._log_pose(label, arm, pose, velocity, acceleration)

        with self._moveit_locks[arm]:
            moveit2 = self._moveit(arm)
            self._configure_moveit(
                moveit2,
                velocity=velocity,
                acceleration=acceleration,
                pipeline=pipeline,
                planner=planner,
            )
            future = moveit2.plan_async(
                pose=pose,
                tolerance_position=POSE_TOL_POSITION,
                tolerance_orientation=POSE_TOL_ORIENTATION,
                start_joint_state=start_joint_state,
                cartesian=cartesian,
                max_step=cartesian_max_step,
            )
            if future is None:
                self.log.error(f"[{label}] [{arm.value}] plan_async returned None")
                return {"result": MotionResult.INVALID, "elapsed_s": 0.0, "trajectory": None}

            start = time.time()
            while not future.done():
                if time.time() - start > timeout_sec:
                    elapsed = round(time.time() - start, 3)
                    self.log.error(f"[{label}] [{arm.value}] TIMEOUT after {elapsed:.1f}s")
                    return {"result": MotionResult.TIMEOUT, "elapsed_s": elapsed, "trajectory": None}
                time.sleep(0.05)

            response = future.result()
            elapsed = time.time() - start
            if response is None:
                self.log.error(f"[{label}] [{arm.value}] empty planning response")
                return {"result": MotionResult.INVALID, "elapsed_s": round(elapsed, 3), "trajectory": None}

            details: dict = {"elapsed_s": round(elapsed, 3)}
            if cartesian:
                result = self._classify_error_code(response.error_code.val)
                details["cartesian_fraction"] = round(float(response.fraction), 6)
                if response.fraction < cartesian_fraction_threshold:
                    result = MotionResult.INVALID
                raw_traj = response.solution.joint_trajectory
                error_code = response.error_code.val
            else:
                motion_response = response.motion_plan_response
                result = self._classify_error_code(motion_response.error_code.val)
                raw_traj = motion_response.trajectory.joint_trajectory
                error_code = motion_response.error_code.val

            if result != MotionResult.SUCCEEDED:
                raw_points = len(getattr(raw_traj, "points", []) or [])
                raw_joints = list(getattr(raw_traj, "joint_names", []) or [])
                details["result"] = result
                details["trajectory"] = None
                details["error_code"] = error_code
                if cartesian:
                    extra = f" | fraction={details['cartesian_fraction']:.3f}"
                else:
                    extra = ""
                self.log.error(
                    f"[{label}] [{arm.value}] {result.value.upper()} in {elapsed:.2f}s "
                    f"| error_code={error_code} | raw_points={raw_points} "
                    f"raw_joints={raw_joints}{extra}"
                )
                return details

            try:
                traj = strip_to_arm_joints(raw_traj, arm)
            except Exception as exc:
                self.log.error(f"[{label}] [{arm.value}] invalid trajectory from MoveIt: {exc}")
                return {
                    "result": MotionResult.INVALID,
                    "elapsed_s": round(elapsed, 3),
                    "trajectory": None,
                    "error_code": error_code,
                }

            details.update(self._trajectory_metrics(traj))
            details["result"] = result
            details["trajectory"] = traj
            details["error_code"] = error_code

            if result == MotionResult.SUCCEEDED and traj.points:
                extra = ""
                if cartesian:
                    extra = f" | fraction={details['cartesian_fraction']:.3f}"
                self.log.info(
                    f"[{label}] [{arm.value}] SUCCEEDED in {elapsed:.2f}s "
                    f"| points={details['point_count']} "
                    f"| joint_path={details['joint_path_length']:.3f}{extra}"
                )
            else:
                extra = ""
                if cartesian:
                    extra = f" | fraction={details['cartesian_fraction']:.3f}"
                self.log.error(
                    f"[{label}] [{arm.value}] {result.value.upper()} in {elapsed:.2f}s "
                    f"| error_code={error_code}{extra}"
                )
            return details

    def plan_to_pose(
        self,
        pose: Pose,
        arm: Arm = Arm.LEFT,
        *,
        velocity: float = 0.05,
        acceleration: float = 0.05,
        timeout_sec: float = 10.0,
        pipeline: str = "ompl",
        planner: str = "RRTConnect",
        start_joint_state: Optional[list[float]] = None,
        cartesian: bool = False,
        cartesian_max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.999,
    ) -> JointTrajectory:
        """Return a planned arm trajectory, or raise if planning fails."""
        details = self.plan_to_pose_details(
            pose=pose,
            arm=arm,
            velocity=velocity,
            acceleration=acceleration,
            timeout_sec=timeout_sec,
            pipeline=pipeline,
            planner=planner,
            start_joint_state=start_joint_state,
            cartesian=cartesian,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        if details["result"] != MotionResult.SUCCEEDED or details["trajectory"] is None:
            raise RuntimeError(
                f"planning failed for {arm.value}: "
                f"result={details['result'].value}, error_code={details.get('error_code')}"
            )
        if not details["trajectory"].points:
            raise RuntimeError(f"MoveIt returned empty trajectory for {arm.value}")
        return details["trajectory"]

    def _task_path_quality_ok(self, details: dict) -> bool:
        joint_path = float(details.get("joint_path_length", 0.0) or 0.0)
        max_step = float(details.get("max_joint_step", 0.0) or 0.0)
        return (
            joint_path <= float(TASK_PATH_QUALITY["max_joint_path_length"])
            and max_step <= float(TASK_PATH_QUALITY["max_joint_step"])
        )

    def _task_path_score(self, details: dict) -> float:
        weights = TASK_SCORING
        return (
            float(weights["joint_path_weight"]) * float(details.get("joint_path_length", 0.0) or 0.0)
            + float(weights["max_joint_step_weight"]) * float(details.get("max_joint_step", 0.0) or 0.0)
            + float(weights["planning_time_weight"]) * float(details.get("elapsed_s", 0.0) or 0.0)
        )

    def _append_planned_segment(
        self,
        combined: JointTrajectory,
        segment: JointTrajectory,
        *,
        start_joint_state: list[float],
        time_offset: float,
        duration_sec: Optional[float],
    ) -> tuple[float, list[float]]:
        segment = copy.deepcopy(segment)
        if not segment.points:
            raise RuntimeError("cannot append an empty trajectory segment")

        # Make the boundary explicit and deterministic. When appending after an
        # existing segment the first point is skipped, but setting it still keeps
        # the segment internally continuous before retiming.
        segment.points[0].positions = list(start_joint_state)

        if duration_sec is not None:
            segment = retime_trajectory(segment, duration_sec)

        append_trajectory_segment(
            combined,
            segment,
            time_offset_sec=time_offset,
            skip_first_point=bool(combined.points),
        )
        new_offset = time_offset + trajectory_duration_seconds(segment)
        new_start = list(segment.points[-1].positions)
        return new_offset, new_start

    def _check_left_pose_reachable(self, pose: Pose, *, start_joint_state: list[float], timeout_sec: float) -> bool:
        """Use the existing MoveIt2 IK helper with an explicit LEFT seed when possible."""
        with self._moveit_locks[Arm.LEFT]:
            moveit2 = self._moveit(Arm.LEFT)
            position = (pose.position.x, pose.position.y, pose.position.z)
            quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            try:
                future = moveit2.compute_ik_async(position, quat, start_joint_state=start_joint_state)
            except TypeError:
                # Older pymoveit2 versions may not accept start_joint_state.
                future = moveit2.compute_ik_async(position, quat)
            if future is None:
                return False
            try:
                deadline = time.time() + timeout_sec
                while not future.done():
                    if time.time() > deadline:
                        return False
                    time.sleep(0.05)
                result = moveit2.get_compute_ik_result(future)
                return result is not None and len(getattr(result, "name", [])) > 0
            except Exception:
                return False

    def _validate_task_pose_pair(
        self,
        target_pose: Pose,
        pre_pose: Pose,
        *,
        waypoint_index: int,
        start_joint_state: list[float],
        timeout_sec: float,
    ) -> None:
        """Apply task_pose_selector.py fast filters to one fixed LEFT waypoint."""
        if not task_workspace_ok(target_pose):
            raise RuntimeError(
                f"task target pose is outside workspace bounds at index={waypoint_index}: "
                f"{pose_to_str(target_pose)}"
            )
        if not task_orientation_ok(target_pose):
            roll, pitch, yaw = quaternion_to_rpy_deg(
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            )
            raise RuntimeError(
                f"task target pose rejected by orientation envelope at index={waypoint_index}: "
                f"rpy=({roll:.1f}, {pitch:.1f}, {yaw:.1f}) {pose_to_str(target_pose)}"
            )
        if not task_workspace_ok(pre_pose):
            raise RuntimeError(
                f"task prepose is outside workspace bounds at index={waypoint_index}: "
                f"{pose_to_str(pre_pose)}"
            )
        if not task_orientation_ok(pre_pose):
            roll, pitch, yaw = quaternion_to_rpy_deg(
                pre_pose.orientation.x,
                pre_pose.orientation.y,
                pre_pose.orientation.z,
                pre_pose.orientation.w,
            )
            raise RuntimeError(
                f"task prepose rejected by orientation envelope at index={waypoint_index}: "
                f"rpy=({roll:.1f}, {pitch:.1f}, {yaw:.1f}) {pose_to_str(pre_pose)}"
            )

        # TaskPoseSelector checks both target and prepose reachability before it
        # spends time scoring planners. Preserve that behavior for the planned
        # LEFT path that will later be mirrored to RIGHT.
        if not self._check_left_pose_reachable(target_pose, start_joint_state=start_joint_state, timeout_sec=timeout_sec):
            raise RuntimeError(
                f"task target IK failed for LEFT at index={waypoint_index}: {pose_to_str(target_pose)}"
            )
        if not self._check_left_pose_reachable(pre_pose, start_joint_state=start_joint_state, timeout_sec=timeout_sec):
            raise RuntimeError(
                f"task prepose IK failed for LEFT at index={waypoint_index}: {pose_to_str(pre_pose)}"
            )

    def _plan_task_global_segment(
        self,
        pose: Pose,
        *,
        start_joint_state: list[float],
        velocity: float,
        acceleration: float,
        timeout_sec: float,
        enforce_path_quality: bool,
    ) -> tuple[JointTrajectory, str, str, dict]:
        """Plan one global segment using task_pose_selector.py's planner cascade.

        Unlike the top-down center shortcut, TaskPoseSelector scores all valid
        planner results and chooses the lowest score. Do the same here.
        """
        best: tuple[JointTrajectory, str, str, dict, float] | None = None
        last_details: dict | None = None
        for pipeline, planner in TASK_PLANNER_SPECS:
            self.log.info(f"[task/path] trying planner {pipeline}/{planner} for prepose {pose_to_str(pose)}")
            details = self.plan_to_pose_details(
                pose=pose,
                arm=Arm.LEFT,
                velocity=velocity,
                acceleration=acceleration,
                timeout_sec=timeout_sec,
                pipeline=pipeline,
                planner=planner,
                start_joint_state=start_joint_state,
            )
            last_details = details
            if details.get("result") != MotionResult.SUCCEEDED or details.get("trajectory") is None:
                self.log.warn(
                    f"[task/path] planner rejected {pipeline}/{planner}: "
                    f"result={details.get('result')}, error_code={details.get('error_code')}"
                )
                continue
            if enforce_path_quality and not self._task_path_quality_ok(details):
                self.log.warn(
                    f"[task/path] planner rejected by path quality {pipeline}/{planner}: "
                    f"joint_path={details.get('joint_path_length')} "
                    f"max_step={details.get('max_joint_step')}"
                )
                continue
            score = self._task_path_score(details)
            self.log.info(
                f"[task/path] planner accepted {pipeline}/{planner} "
                f"score={score:.3f} points={details.get('point_count')} "
                f"joint_path={details.get('joint_path_length')} max_step={details.get('max_joint_step')}"
            )
            if best is None or score < best[4]:
                best = (details["trajectory"], pipeline, planner, details, score)

        if best is not None:
            trajectory, pipeline, planner, details, score = best
            self.log.info(f"[task/path] selected planner {pipeline}/{planner} score={score:.3f}")
            return trajectory, pipeline, planner, details

        error_code = None if last_details is None else last_details.get("error_code")
        raise RuntimeError(
            f"task global planning failed for prepose {pose_to_str(pose)} "
            f"with planners={TASK_PLANNER_SPECS}, last_error_code={error_code}"
        )

    def _plan_task_local_segment(
        self,
        pose: Pose,
        *,
        start_joint_state: list[float],
        velocity: float,
        acceleration: float,
        timeout_sec: float,
        fallback_pipeline: str,
        fallback_planner: str,
        cartesian_fraction_threshold: float,
    ) -> tuple[JointTrajectory, str, str, dict]:
        """Plan pre-grasp -> target using TaskPoseSelector's preferred local mode.

        task_pose_selector.py sets local_modes.preferred to cartesian and
        fallback to lift. This dual-arm path uses cartesian first; if it fails,
        it falls back to the selected global planner so the controller can still
        receive a complete joint trajectory.
        """
        self.log.info(f"[task/local] trying cartesian segment to target {pose_to_str(pose)}")
        details = self.plan_to_pose_details(
            pose=pose,
            arm=Arm.LEFT,
            velocity=velocity,
            acceleration=acceleration,
            timeout_sec=timeout_sec,
            pipeline="ompl",
            planner="RRTConnect",
            start_joint_state=start_joint_state,
            cartesian=True,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        if details.get("result") == MotionResult.SUCCEEDED and details.get("trajectory") is not None:
            return details["trajectory"], "cartesian", "compute_cartesian_path", details

        self.log.warn(
            f"[task/local] cartesian failed; falling back to {fallback_pipeline}/{fallback_planner} "
            f"for target pose"
        )
        details = self.plan_to_pose_details(
            pose=pose,
            arm=Arm.LEFT,
            velocity=velocity,
            acceleration=acceleration,
            timeout_sec=timeout_sec,
            pipeline=fallback_pipeline,
            planner=fallback_planner,
            start_joint_state=start_joint_state,
        )
        if details.get("result") == MotionResult.SUCCEEDED and details.get("trajectory") is not None:
            return details["trajectory"], fallback_pipeline, fallback_planner, details

        raise RuntimeError(
            f"task local target planning failed for {pose_to_str(pose)}: "
            f"result={details.get('result')}, error_code={details.get('error_code')}"
        )

    def plan_left_waypoints_then_mirror(
        self,
        left_poses: list[Pose],
        *,
        velocity: float = 0.05,
        acceleration: float = 0.05,
        segment_duration_sec: Optional[float] = None,
        timeout_sec: float = 10.0,
        pipeline: str = "ompl",
        planner: str = "RRTConnect",
        cartesian: bool = False,
        cartesian_max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.999,
        use_task_selector: bool = True,
        include_task_target: bool = True,
        pre_grasp_offset: float = TASK_PRE_GRASP_OFFSET,
        enforce_task_path_quality: bool = False,
    ) -> tuple[JointTrajectory, JointTrajectory, list[list[float]]]:
        """Plan LEFT-arm waypoints, then mirror the resulting path to RIGHT.

        The dual-arm strategy is unchanged from the original moveit_dual.py:
        only the LEFT path is planned, the RIGHT path is generated by the
        calibrated mirror mapping, and both final JointTrajectory goals are sent
        to the two arm controllers together by execute_both().

        When use_task_selector=True, each fixed input waypoint is handled with the same
        filtering/planning policy as task_pose_selector.py. This method does not
        call GPD and does not select one pose from multiple candidates:
          1. keep the input pose position and orientation exactly as provided,
          2. reject poses outside the workspace/orientation envelope,
          3. build pre_grasp_of(target_pose),
          4. check target and prepose IK reachability,
          5. plan to the prepose using the task planner cascade and score,
          6. optionally add a local cartesian prepose -> target segment.
        """
        poses = list(left_poses)
        if not poses:
            raise ValueError("left_poses must contain at least one pose")

        left_start = self.get_joints(Arm.LEFT)
        right_start = self.get_joints(Arm.RIGHT)
        if left_start is None or right_start is None:
            raise RuntimeError("joint states unavailable")

        combined_left = JointTrajectory()
        combined_left.joint_names = list(ARM_L_JOINTS)

        start_joint_state = list(left_start)
        time_offset = 0.0
        waypoint_joints: list[list[float]] = []

        for index, raw_pose in enumerate(poses):
            self.log.info(f"[waypoint-plan] planning LEFT waypoint {index + 1}/{len(poses)}")

            if use_task_selector:
                target_pose = copy_pose(raw_pose)
                pre_pose = make_task_pre_pose(target_pose, offset=pre_grasp_offset)
                self.log.info(
                    f"[task/path] target={pose_to_str(target_pose)} | pre={pose_to_str(pre_pose)}"
                )
                self._validate_task_pose_pair(
                    target_pose,
                    pre_pose,
                    waypoint_index=index,
                    start_joint_state=start_joint_state,
                    timeout_sec=min(float(timeout_sec), 5.0),
                )

                per_waypoint_duration = segment_duration_sec
                if per_waypoint_duration is not None and include_task_target:
                    global_duration = float(per_waypoint_duration) * 0.5
                    local_duration = float(per_waypoint_duration) * 0.5
                else:
                    global_duration = per_waypoint_duration
                    local_duration = per_waypoint_duration

                global_segment, selected_pipeline, selected_planner, _global_details = self._plan_task_global_segment(
                    pre_pose,
                    start_joint_state=start_joint_state,
                    velocity=velocity,
                    acceleration=acceleration,
                    timeout_sec=timeout_sec,
                    enforce_path_quality=enforce_task_path_quality,
                )
                time_offset, start_joint_state = self._append_planned_segment(
                    combined_left,
                    global_segment,
                    start_joint_state=start_joint_state,
                    time_offset=time_offset,
                    duration_sec=global_duration,
                )

                if include_task_target:
                    local_segment, local_pipeline, local_planner, _local_details = self._plan_task_local_segment(
                        target_pose,
                        start_joint_state=start_joint_state,
                        velocity=velocity,
                        acceleration=acceleration,
                        timeout_sec=timeout_sec,
                        fallback_pipeline=selected_pipeline,
                        fallback_planner=selected_planner,
                        cartesian_fraction_threshold=cartesian_fraction_threshold,
                    )
                    time_offset, start_joint_state = self._append_planned_segment(
                        combined_left,
                        local_segment,
                        start_joint_state=start_joint_state,
                        time_offset=time_offset,
                        duration_sec=local_duration,
                    )
                    self.log.info(
                        f"[task/path] waypoint {index + 1} used global "
                        f"{selected_pipeline}/{selected_planner} + local {local_pipeline}/{local_planner}"
                    )
                else:
                    self.log.info(
                        f"[task/path] waypoint {index + 1} used global "
                        f"{selected_pipeline}/{selected_planner}; target segment disabled"
                    )

                waypoint_joints.append(list(start_joint_state))
                self.log.info(
                    f"[waypoint-plan] LEFT waypoint {index + 1} planned via task selector "
                    f"| total_points={len(combined_left.points)} "
                    f"| total_duration={time_offset:.3f}s"
                )
                continue

            # Direct fixed-waypoint mode. Do not ask MoveIt to plan a
            # zero-distance segment to the current FK pose: some MoveIt setups
            # return a generic planning failure for that no-op request. Instead,
            # seed the combined trajectory with the current joints and continue
            # to the next waypoint. This is especially useful when the first
            # waypoint is copied from tf2_echo.
            if not combined_left.points and self._is_current_pose_goal(raw_pose, Arm.LEFT):
                self.log.info(
                    f"[waypoint-plan] LEFT waypoint {index + 1} is already the current EE pose; "
                    "using current joint state as the first trajectory point"
                )
                combined_left.points.extend(
                    make_single_point_trajectory(ARM_L_JOINTS, start_joint_state, time_sec=0.0).points
                )
                waypoint_joints.append(list(start_joint_state))
                continue

            # For the first actual plan, let MoveIt use its live current state.
            # Passing a hand-built 7-value start_joint_state into pymoveit2 can
            # make a current-pose or near-current-pose request fail on some
            # Jazzy/pymoveit2 combinations. Once a planned segment exists, chain
            # later waypoints from the previous segment's final joints.
            planning_start_state = None if len(combined_left.points) <= 1 else start_joint_state

            details = self.plan_to_pose_details(
                pose=raw_pose,
                arm=Arm.LEFT,
                velocity=velocity,
                acceleration=acceleration,
                timeout_sec=timeout_sec,
                pipeline=pipeline,
                planner=planner,
                start_joint_state=planning_start_state,
                cartesian=cartesian,
                cartesian_max_step=cartesian_max_step,
                cartesian_fraction_threshold=cartesian_fraction_threshold,
            )
            if details["result"] != MotionResult.SUCCEEDED or details["trajectory"] is None:
                raise RuntimeError(
                    f"LEFT waypoint planning failed at index={index}, "
                    f"result={details['result'].value}, error_code={details.get('error_code')}, "
                    f"pose={pose_to_str(raw_pose)}"
                )

            segment = copy.deepcopy(details["trajectory"])
            time_offset, start_joint_state = self._append_planned_segment(
                combined_left,
                segment,
                start_joint_state=start_joint_state,
                time_offset=time_offset,
                duration_sec=segment_duration_sec,
            )
            waypoint_joints.append(list(start_joint_state))
            self.log.info(
                f"[waypoint-plan] LEFT waypoint {index + 1} planned directly "
                f"| total_points={len(combined_left.points)} "
                f"| total_duration={time_offset:.3f}s"
            )

        if not combined_left.points:
            raise RuntimeError("combined LEFT trajectory is empty")

        right_traj = mirror_left_trajectory_to_right(combined_left)
        if right_traj.points:
            # Preserve the existing controller-side safety behavior: force the
            # first right point to the actual current right-arm state so the
            # controller does not jump if the robot is not perfectly symmetric.
            right_traj.points[0].positions = list(right_start)

        self.log.info(
            f"[waypoint-plan] combined LEFT points={len(combined_left.points)}, "
            f"RIGHT points={len(right_traj.points)}, "
            f"duration={trajectory_duration_seconds(combined_left):.3f}s"
        )
        return combined_left, right_traj, waypoint_joints

    def move_left_waypoints_then_mirror(
        self,
        left_poses: list[Pose],
        *,
        velocity: float = 0.05,
        acceleration: float = 0.05,
        segment_duration_sec: Optional[float] = None,
        plan_timeout_sec: float = 10.0,
        execute_timeout_sec: float = 30.0,
        pipeline: str = "ompl",
        planner: str = "RRTConnect",
        cartesian: bool = False,
        cartesian_max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.999,
        use_task_selector: bool = True,
        include_task_target: bool = True,
        pre_grasp_offset: float = TASK_PRE_GRASP_OFFSET,
        enforce_task_path_quality: bool = False,
    ) -> DualArmExecutionResult:
        """Plan multiple left pose waypoints, mirror them, and execute both arms."""
        left_traj, right_traj, _ = self.plan_left_waypoints_then_mirror(
            left_poses,
            velocity=velocity,
            acceleration=acceleration,
            segment_duration_sec=segment_duration_sec,
            timeout_sec=plan_timeout_sec,
            pipeline=pipeline,
            planner=planner,
            cartesian=cartesian,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
            use_task_selector=use_task_selector,
            include_task_target=include_task_target,
            pre_grasp_offset=pre_grasp_offset,
            enforce_task_path_quality=enforce_task_path_quality,
        )
        return self.execute_both(left_traj, right_traj, timeout_sec=execute_timeout_sec)

    def compute_ik(self, arm: Arm, pose: Pose, timeout_sec: float = 5.0) -> list[float]:
        self.wait_for_moveit_services(timeout_sec=timeout_sec)
        req = GetPositionIK.Request()
        req.ik_request.group_name = ARM_GROUP[arm]
        req.ik_request.robot_state = self.current_robot_state()
        req.ik_request.avoid_collisions = True
        req.ik_request.ik_link_name = ARM_EEF[arm]
        ps = PoseStamped()
        ps.header.frame_id = BASE_LINK
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.pose = pose
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout = duration_from_seconds(timeout_sec)
        # ROS 2 Jazzy moveit_msgs/PositionIKRequest has no `attempts` field.
        # Some older examples mention it, so keep this guarded for portability.
        if hasattr(req.ik_request, "attempts"):
            req.ik_request.attempts = 5
        future = self._ik_client.call_async(req)
        resp = self._wait_future(future, timeout_sec=timeout_sec + 2.0)
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"IK failed for {arm.value}, error_code={resp.error_code.val}")
        name_to_pos = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
        return [float(name_to_pos[name]) for name in ARM_JOINTS[arm]]

    def plan_to_joints(self, arm: Arm, target_joints: list[float], *, velocity: float = 0.05, acceleration: float = 0.05, timeout_sec: float = 10.0) -> JointTrajectory:
        self.wait_for_moveit_services(timeout_sec=timeout_sec)
        if len(target_joints) != 7:
            raise ValueError("target_joints must have length 7")

        req = GetMotionPlan.Request()
        mpr = req.motion_plan_request
        mpr.group_name = ARM_GROUP[arm]
        mpr.num_planning_attempts = 5
        mpr.allowed_planning_time = 5.0
        mpr.max_velocity_scaling_factor = float(velocity)
        mpr.max_acceleration_scaling_factor = float(acceleration)
        mpr.start_state = self.current_robot_state()
        mpr.goal_constraints = [self._joint_goal_constraints(arm, target_joints)]

        future = self._plan_client.call_async(req)
        resp = self._wait_future(future, timeout_sec=timeout_sec + 2.0)
        if resp.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"planning failed for {arm.value}, error_code={resp.motion_plan_response.error_code.val}")
        traj = strip_to_arm_joints(resp.motion_plan_response.trajectory.joint_trajectory, arm)
        if not traj.points:
            raise RuntimeError(f"MoveIt returned empty trajectory for {arm.value}")
        return traj

    def plan_left_pose_then_mirror(
        self,
        left_pose: Pose,
        *,
        velocity: float = 0.05,
        acceleration: float = 0.05,
        duration_sec: Optional[float] = None,
        timeout_sec: float = 10.0,
        pipeline: str = "ompl",
        planner: str = "RRTConnect",
        cartesian: bool = False,
        cartesian_max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.999,
    ) -> tuple[JointTrajectory, JointTrajectory, list[float], list[float]]:
        """Plan the left arm to a Cartesian pose with MoveIt2, then mirror it.

        Path generation follows moveit_client.py's plan_async(pose=...) style.
        The returned pair is still meant to be executed by execute_both(), so the
        two arms move together through the direct FollowJointTrajectory clients.

        Returns:
            left_traj, right_traj, left_final_joints, right_mirror_final_joints
        """
        left_traj = self.plan_to_pose(
            pose=left_pose,
            arm=Arm.LEFT,
            velocity=velocity,
            acceleration=acceleration,
            timeout_sec=timeout_sec,
            pipeline=pipeline,
            planner=planner,
            cartesian=cartesian,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        if duration_sec is not None:
            left_traj = retime_trajectory(left_traj, duration_sec)

        right_traj = mirror_left_trajectory_to_right(left_traj)
        right_current = self.get_joints(Arm.RIGHT)
        if right_current is not None and right_traj.points:
            # Preserve moveit_dual.py's controller-side safety behavior: avoid
            # an initial right-arm jump if the robot is not exactly symmetric.
            right_traj.points[0].positions = list(right_current)

        left_final_joints = list(left_traj.points[-1].positions)
        right_mirror_final_joints = mirror_left_joints_to_right(left_final_joints)
        self.log.info(f"[symmetric/plan] left final    : {[round(v, 4) for v in left_final_joints]}")
        self.log.info(f"[symmetric/plan] right mirror  : {[round(v, 4) for v in right_mirror_final_joints]}")
        return left_traj, right_traj, left_final_joints, right_mirror_final_joints

    def move_left_pose_then_mirror(
        self,
        left_pose: Pose,
        *,
        velocity: float = 0.05,
        acceleration: float = 0.05,
        duration_sec: Optional[float] = None,
        plan_timeout_sec: float = 10.0,
        execute_timeout_sec: float = 30.0,
        pipeline: str = "ompl",
        planner: str = "RRTConnect",
        cartesian: bool = False,
        cartesian_max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.999,
    ) -> DualArmExecutionResult:
        """Plan with MoveIt2, mirror the path, and execute both arms together."""
        left_traj, right_traj, _, _ = self.plan_left_pose_then_mirror(
            left_pose,
            velocity=velocity,
            acceleration=acceleration,
            duration_sec=duration_sec,
            timeout_sec=plan_timeout_sec,
            pipeline=pipeline,
            planner=planner,
            cartesian=cartesian,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        return self.execute_both(left_traj, right_traj, timeout_sec=execute_timeout_sec)

    @staticmethod
    def _joint_goal_constraints(arm: Arm, target_joints: list[float]) -> Constraints:
        constraints = Constraints()
        for name, pos in zip(ARM_JOINTS[arm], target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints
