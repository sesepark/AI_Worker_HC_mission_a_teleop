from __future__ import annotations

import argparse
import math
import os
import random
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState


LEFT_JOINTS = [
    'arm_l_joint1',
    'arm_l_joint2',
    'arm_l_joint3',
    'arm_l_joint4',
    'arm_l_joint5',
    'arm_l_joint6',
    'arm_l_joint7',
]

GROUP_NAME = 'arm_l'
BASE_LINK = 'base_link'
EE_LINK = 'end_effector_l_link'

DEFAULT_TARGET_POSITION = [0.75, 0.246, 1.2]
DEFAULT_TARGET_RPY_DEG = [-90.0, 0.0, 90.0]
DEFAULT_FORCE_Y_N = 10.0
DEFAULT_LEFT_ARM_EFFORT_LIMITS_NM = [26.0, 26.0, 26.0, 14.6, 14.6, 14.6, 5.1]
DEFAULT_SCORE_WEIGHTS = [1.0, 1.25, 1.0, 1.0, 1.0, 1.0, 1.0]

DEFAULT_HOME = [0.0, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_SEED_LIMITS = [(-math.pi, math.pi)] * 7
PLACEHOLDER_EFFORT_LIMIT_NM = 500.0


@dataclass
class CandidateScore:
    joints: list[float]
    tau_nm: list[float]
    ratios: list[float]
    weighted_ratios: list[float]
    score_max: float
    score_l2: float
    worst_joint_index: int
    worst_joint_name: str
    ee_y_sensitivity: list[float]


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert roll, pitch, yaw in radians to quaternion x, y, z, w."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def make_pose(position: Iterable[float], rpy_deg: Iterable[float]) -> Pose:
    pos = list(position)
    rpy = [math.radians(v) for v in rpy_deg]
    qx, qy, qz, qw = quaternion_from_rpy(rpy[0], rpy[1], rpy[2])

    pose = Pose()
    pose.position.x = float(pos[0])
    pose.position.y = float(pos[1])
    pose.position.z = float(pos[2])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def duration_from_seconds(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def joint_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum(wrap_to_pi(x - y) ** 2 for x, y in zip(a, b)))


def format_float_list(values: Iterable[float], *, precision: int = 4) -> str:
    return '[' + ', '.join(f'{float(v):.{precision}f}' for v in values) + ']'


def radians_to_degrees(values: Iterable[float]) -> list[float]:
    return [math.degrees(float(v)) for v in values]


def default_urdf_candidates() -> list[Path]:
    candidates: list[Path] = []

    for env_name in ("ROBOT_URDF_PATH", "ROBOT_DESCRIPTION_PATH"):
        if os.environ.get(env_name):
            candidates.append(Path(os.environ[env_name]).expanduser())

    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("ffw_description"))
        candidates.extend(
            [
                share / "urdf" / "ffw_sg2_rev1_follower" / "ffw_sg2_follower.urdf",
                share / "urdf" / "ffw_sg2_rev1_follower" / "ffw_sg2_follower.urdf.xacro",
            ]
        )
    except Exception:
        pass

    source_root = Path(__file__).resolve()
    for parent in source_root.parents:
        candidates.extend(
            [
                parent / "ai_worker" / "ffw_description" / "urdf" / "ffw_sg2_rev1_follower" / "ffw_sg2_follower.urdf",
                parent / "ffw_description" / "urdf" / "ffw_sg2_rev1_follower" / "ffw_sg2_follower.urdf",
            ]
        )

    unique: list[Path] = []
    seen: set[Path] = set()
    for candidate in candidates:
        resolved = candidate.expanduser()
        if resolved not in seen:
            unique.append(resolved)
            seen.add(resolved)
    return unique


def load_effort_limits_from_urdf(path: Path) -> list[float]:
    tree = ET.parse(path)
    root = tree.getroot()
    effort_by_joint: dict[str, float] = {}

    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        if name not in LEFT_JOINTS:
            continue
        limit = joint.find("limit")
        if limit is None or "effort" not in limit.attrib:
            continue
        effort_by_joint[name] = float(limit.attrib["effort"])

    missing = [name for name in LEFT_JOINTS if name not in effort_by_joint]
    if missing:
        raise ValueError(f"URDF is missing effort limits for joints: {missing}")
    return [effort_by_joint[name] for name in LEFT_JOINTS]


def resolve_effort_limits(args: argparse.Namespace, log) -> list[float] | None:
    if args.effort_limits is not None:
        return [float(v) for v in args.effort_limits]

    if args.no_auto_effort_limits:
        return None

    candidates = [args.urdf_path.expanduser()] if args.urdf_path is not None else default_urdf_candidates()
    for candidate in candidates:
        if not candidate.exists() or candidate.suffix == ".xacro":
            continue
        try:
            limits = load_effort_limits_from_urdf(candidate)
        except Exception as exc:
            log.warn(f"Could not read effort limits from {candidate}: {exc}")
            continue

        if all(limit >= PLACEHOLDER_EFFORT_LIMIT_NM for limit in limits):
            log.warn(
                f"URDF effort limits from {candidate} look like placeholders: "
                + "[" + ", ".join(f"{v:.1f}" for v in limits) + "]. "
                "Not using them for normalized scoring."
            )
            continue

        log.info(f"Loaded effort limits from {candidate}")
        return limits

    log.info(
        "Using default left-arm effort limits [Nm]: "
        + "[" + ", ".join(f"{v:.3f}" for v in DEFAULT_LEFT_ARM_EFFORT_LIMITS_NM) + "]"
    )
    return list(DEFAULT_LEFT_ARM_EFFORT_LIMITS_NM)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Find a left-arm posture that minimizes torque concentration for +Y EE force.'
    )
    parser.add_argument('--target-position', type=float, nargs=3, default=DEFAULT_TARGET_POSITION)
    parser.add_argument('--target-rpy-deg', type=float, nargs=3, default=DEFAULT_TARGET_RPY_DEG)
    parser.add_argument('--force-y', type=float, default=DEFAULT_FORCE_Y_N,
                        help='External force applied to the left EE in Y of base_link [N]. Default is +10N.')
    parser.add_argument('--samples', type=int, default=60,
                        help='Number of IK seed samples to try.')
    parser.add_argument('--random-seed', type=int, default=7)
    parser.add_argument('--ik-timeout', type=float, default=0.25,
                        help='IK timeout per candidate [s].')
    parser.add_argument('--fk-timeout', type=float, default=1.0,
                        help='FK service timeout per call [s].')
    parser.add_argument('--fd-eps', type=float, default=1e-4,
                        help='Finite-difference step for numerical Jacobian [rad].')
    parser.add_argument('--unique-threshold', type=float, default=1e-3,
                        help='Minimum joint-space distance for unique IK candidates.')
    parser.add_argument('--effort-limits', type=float, nargs=7, default=None,
                        help='Left arm joint effort limits [Nm]. Overrides the default SG2 left-arm values.')
    parser.add_argument('--score-weights', type=float, nargs=7, default=DEFAULT_SCORE_WEIGHTS,
                        help='Per-joint score weights. Default slightly penalizes arm_l_joint2.')
    parser.add_argument('--urdf-path', type=Path, default=None,
                        help='URDF file to read effort limits from when --effort-limits is omitted.')
    parser.add_argument('--no-auto-effort-limits', action='store_true',
                        help='Do not use default or URDF effort limits; score by raw |tau| instead.')
    parser.add_argument('--allow-collision-ik', action='store_true',
                        help='Allow IK solutions that are in collision. Useful only for diagnosing IK failures.')
    parser.add_argument('--execute', action='store_true',
                        help='Move the left arm to the selected joint posture.')
    parser.add_argument('--velocity', type=float, default=0.1,
                        help='Execution velocity scaling.')
    parser.add_argument('--acceleration', type=float, default=0.1,
                        help='Execution acceleration scaling.')
    parser.add_argument('--planning-time', type=float, default=5.0)
    parser.add_argument('--planner', type=str, default='RRTConnect')
    parser.add_argument('--pipeline', type=str, default='ompl')
    return parser.parse_args()


class LeftArmTorquePoseSearch:
    def __init__(self, node: Node, args: argparse.Namespace):
        self.node = node
        self.log = node.get_logger()
        self.args = args
        self.current_left_joints: list[float] | None = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.node.create_subscription(JointState, 'joint_states', self._joint_state_cb, qos)

        self.ik_client = node.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = node.create_client(GetPositionFK, '/compute_fk')

    def _joint_state_cb(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(name in name_to_pos for name in LEFT_JOINTS):
            self.current_left_joints = [float(name_to_pos[name]) for name in LEFT_JOINTS]

    def wait_ready(self) -> None:
        self.log.info('Waiting for /compute_ik and /compute_fk services...')
        while rclpy.ok() and not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.log.warn('/compute_ik not available yet')
        while rclpy.ok() and not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.log.warn('/compute_fk not available yet')

        self.log.info('Waiting for left-arm joint_states...')
        start = time.time()
        while rclpy.ok() and self.current_left_joints is None:
            if time.time() - start > 10.0:
                raise RuntimeError('Timed out waiting for left-arm joint_states')
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.log.info('MoveIt services and joint_states are ready.')

    def solve_ik(self, target_pose: Pose, seed_joints: list[float]) -> list[float] | None:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = BASE_LINK
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.pose = target_pose

        robot_state = RobotState()
        robot_state.joint_state.name = list(LEFT_JOINTS)
        robot_state.joint_state.position = [float(v) for v in seed_joints]

        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.ik_link_name = EE_LINK
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.robot_state = robot_state
        req.ik_request.timeout = duration_from_seconds(self.args.ik_timeout)
        req.ik_request.avoid_collisions = not bool(self.args.allow_collision_ik)

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.args.ik_timeout + 0.5)
        if not future.done():
            return None

        response = future.result()
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        js = response.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))
        if not all(name in name_to_pos for name in LEFT_JOINTS):
            return None

        return [float(name_to_pos[name]) for name in LEFT_JOINTS]

    def compute_fk_position(self, joints: list[float]) -> np.ndarray | None:
        robot_state = RobotState()
        robot_state.joint_state.name = list(LEFT_JOINTS)
        robot_state.joint_state.position = [float(v) for v in joints]

        req = GetPositionFK.Request()
        req.header.frame_id = BASE_LINK
        req.fk_link_names = [EE_LINK]
        req.robot_state = robot_state

        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.args.fk_timeout)
        if not future.done():
            return None

        response = future.result()
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        if not response.pose_stamped:
            return None

        p = response.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z], dtype=float)

    def compute_y_sensitivity(self, joints: list[float]) -> list[float] | None:
        """Return dy_ee / dq_i for each left-arm joint using central differences."""
        eps = float(self.args.fd_eps)
        sensitivities: list[float] = []

        for i in range(7):
            q_plus = list(joints)
            q_minus = list(joints)
            q_plus[i] += eps
            q_minus[i] -= eps

            p_plus = self.compute_fk_position(q_plus)
            p_minus = self.compute_fk_position(q_minus)
            if p_plus is None or p_minus is None:
                return None

            dy_dqi = float((p_plus[1] - p_minus[1]) / (2.0 * eps))
            sensitivities.append(dy_dqi)

        return sensitivities

    def score_candidate(self, joints: list[float]) -> CandidateScore | None:
        y_sensitivity = self.compute_y_sensitivity(joints)
        if y_sensitivity is None:
            return None

        # For force-only Y wrench: tau_i = Fy * dy/dq_i
        tau = [float(self.args.force_y * s) for s in y_sensitivity]

        if self.args.effort_limits is None:
            ratios = [abs(v) for v in tau]
        else:
            ratios = [abs(t) / float(limit) for t, limit in zip(tau, self.args.effort_limits)]

        weighted_ratios = [
            float(ratio) * float(weight)
            for ratio, weight in zip(ratios, self.args.score_weights)
        ]

        score_max = float(max(weighted_ratios))
        score_l2 = float(math.sqrt(sum(r * r for r in weighted_ratios)))
        worst_index = int(max(range(7), key=lambda idx: weighted_ratios[idx]))

        return CandidateScore(
            joints=[float(v) for v in joints],
            tau_nm=tau,
            ratios=[float(v) for v in ratios],
            weighted_ratios=weighted_ratios,
            score_max=score_max,
            score_l2=score_l2,
            worst_joint_index=worst_index,
            worst_joint_name=LEFT_JOINTS[worst_index],
            ee_y_sensitivity=y_sensitivity,
        )

    def make_seed(self, sample_index: int) -> list[float]:
        current = self.current_left_joints or DEFAULT_HOME

        if sample_index == 0:
            return list(current)
        if sample_index == 1:
            return list(DEFAULT_HOME)

        seed: list[float] = []
        if sample_index % 2 == 0:
            # Local sampling around current posture.
            for q, (lo, hi) in zip(current, DEFAULT_SEED_LIMITS):
                value = q + random.gauss(0.0, 0.8)
                seed.append(float(min(max(value, lo), hi)))
        else:
            # Global sampling.
            for lo, hi in DEFAULT_SEED_LIMITS:
                seed.append(float(random.uniform(lo, hi)))

        return seed

    def generate_ik_candidates(self, target_pose: Pose) -> list[list[float]]:
        candidates: list[list[float]] = []

        for k in range(max(1, int(self.args.samples))):
            seed = self.make_seed(k)
            q = self.solve_ik(target_pose, seed)
            if q is None:
                continue

            if any(joint_distance(q, existing) < self.args.unique_threshold for existing in candidates):
                continue

            candidates.append(q)
            self.log.info(
                f'IK candidate {len(candidates):02d}: '
                f'rad={format_float_list(q, precision=4)}, '
                f'deg={format_float_list(radians_to_degrees(q), precision=2)}'
            )

        return candidates

    def search(self) -> CandidateScore | None:
        random.seed(int(self.args.random_seed))
        np.random.seed(int(self.args.random_seed))

        target_pose = make_pose(self.args.target_position, self.args.target_rpy_deg)
        p = target_pose.position
        o = target_pose.orientation
        self.log.info(
            'Target left EE pose: '
            f'position=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), '
            f'quat=({o.x:.4f}, {o.y:.4f}, {o.z:.4f}, {o.w:.4f})'
        )
        self.log.info(f'Assumed external force: Y = {self.args.force_y:.3f} N in {BASE_LINK}')
        self.log.info(f'IK collision checking: avoid_collisions={not bool(self.args.allow_collision_ik)}')

        if self.args.effort_limits is None:
            self.log.warn(
                'No --effort-limits provided. score_max is raw max |tau| [Nm], '
                'not normalized by motor limits.'
            )
        else:
            self.log.info(
                'Effort limits [Nm]: '
                + '[' + ', '.join(f'{v:.3f}' for v in self.args.effort_limits) + ']'
            )
        self.log.info(
            'Score weights: '
            + '[' + ', '.join(f'{v:.3f}' for v in self.args.score_weights) + ']'
        )

        candidates = self.generate_ik_candidates(target_pose)
        if not candidates:
            self.log.error('No IK candidates found for the target pose.')
            self.log.error(
                'Hints: for the mirrored left-arm pose from test_dual_pick, try '
                '--target-position 0.6 0.246 1.2 --target-rpy-deg -90 0 90. '
                'If x=0.75 fails, it may be near/outside the reachable workspace; try x=0.60~0.65. '
                'Use --allow-collision-ik once to check whether collision filtering is the blocker.'
            )
            return None

        self.log.info(f'Evaluating {len(candidates)} IK candidates...')
        best: CandidateScore | None = None

        for idx, q in enumerate(candidates, start=1):
            score = self.score_candidate(q)
            if score is None:
                self.log.warn(f'Candidate {idx:02d}: FK/Jacobian failed; skipped.')
                continue

            self.log.info(
                f'Candidate {idx:02d}: score_max={score.score_max:.6f}, '
                f'score_l2={score.score_l2:.6f}, '
                f'weighted_worst={score.worst_joint_name}, '
                f'tau=[' + ', '.join(f'{v:.3f}' for v in score.tau_nm) + ']'
            )

            if best is None:
                best = score
            elif score.score_max < best.score_max - 1e-9:
                best = score
            elif abs(score.score_max - best.score_max) <= 1e-9 and score.score_l2 < best.score_l2:
                best = score

        if best is None:
            self.log.error('All candidates failed during score evaluation.')
            return None

        self.print_best(best)
        return best

    def print_best(self, best: CandidateScore) -> None:
        self.log.info('================ BEST LEFT-ARM POSTURE ================')
        self.log.info(f'joints [rad] = {format_float_list(best.joints, precision=6)}')
        self.log.info(f'joints [deg] = {format_float_list(radians_to_degrees(best.joints), precision=3)}')
        self.log.info(f'tau [Nm]     = {format_float_list(best.tau_nm, precision=6)}')
        if self.args.effort_limits is None:
            self.log.info(f'ratio/raw    = {format_float_list(best.ratios, precision=6)}')
        else:
            self.log.info(f'ratio        = {format_float_list(best.ratios, precision=6)}')
        self.log.info(f'weighted     = {format_float_list(best.weighted_ratios, precision=6)}')
        self.log.info(f'score_max    = {best.score_max:.6f}')
        self.log.info(f'score_l2     = {best.score_l2:.6f}')
        self.log.info(f'worst_joint  = {best.worst_joint_name}')
        self.log.info(f'dy/dq        = {format_float_list(best.ee_y_sensitivity, precision=6)}')
        if self.args.effort_limits is not None:
            max_ratio_index = int(max(range(7), key=lambda idx: best.ratios[idx]))
            max_ratio = best.ratios[max_ratio_index]
            max_ratio_joint = LEFT_JOINTS[max_ratio_index]
            if max_ratio > 1.0:
                self.log.warn(
                    f'Best posture still exceeds at least one effort limit: '
                    f'max ratio={max_ratio:.6f} at {max_ratio_joint}'
                )
            else:
                self.log.info(
                    f'Best posture is within effort limits: '
                    f'max ratio={max_ratio:.6f} at {max_ratio_joint}'
                )
        self.log.info('========================================================')

    def execute(self, joints: list[float]) -> bool:
        try:
            from pymoveit2 import MoveIt2
            from pymoveit2.moveit2 import MoveIt2State
        except Exception as exc:
            self.log.error(f'Cannot import pymoveit2 for execution: {exc}')
            return False

        from rclpy.callback_groups import ReentrantCallbackGroup

        cb_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(
            node=self.node,
            joint_names=LEFT_JOINTS,
            base_link_name=BASE_LINK,
            end_effector_name=EE_LINK,
            group_name=GROUP_NAME,
            callback_group=cb_group,
            use_move_group_action=True,
        )

        # Match the QoS pattern from the user's current MoveIt client.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.node.create_subscription(
            JointState,
            'joint_states',
            moveit2._MoveIt2__joint_state_callback,
            qos,
            callback_group=cb_group,
        )

        self.log.info('Waiting for move_group action server [arm_l]...')
        while rclpy.ok() and not moveit2._MoveIt2__move_action_client.wait_for_server(timeout_sec=1.0):
            self.log.warn('[arm_l] move_group action server not available yet')

        start = time.time()
        while rclpy.ok() and moveit2.joint_state is None:
            if time.time() - start > 10.0:
                self.log.error('Timed out waiting for pymoveit2 joint_state')
                return False
            rclpy.spin_once(self.node, timeout_sec=0.1)

        moveit2.motion_suceeded = False
        moveit2.pipeline_id = self.args.pipeline
        moveit2.planner_id = self.args.planner
        moveit2.max_velocity = float(self.args.velocity)
        moveit2.max_acceleration = float(self.args.acceleration)
        moveit2.allowed_planning_time = float(self.args.planning_time)
        moveit2.num_planning_attempts = 5

        self.log.info('Executing selected left-arm joint posture...')
        moveit2.move_to_configuration(joints)

        start = time.time()
        while rclpy.ok() and moveit2.query_state() == MoveIt2State.IDLE:
            if time.time() - start > 2.0:
                self.log.error('MoveIt goal never left IDLE')
                return False
            rclpy.spin_once(self.node, timeout_sec=0.02)

        while rclpy.ok() and moveit2.query_state() != MoveIt2State.IDLE:
            if time.time() - start > 30.0:
                self.log.error('Execution timeout')
                return False
            rclpy.spin_once(self.node, timeout_sec=0.05)

        if moveit2.motion_suceeded:
            self.log.info('Execution succeeded.')
            return True

        error = moveit2.get_last_execution_error_code()
        error_val = error.val if error is not None else MoveItErrorCodes.UNDEFINED
        self.log.error(f'Execution failed. MoveIt error_code={error_val}')
        return False


def main() -> None:
    args = parse_args()

    if args.effort_limits is not None and any(v <= 0.0 for v in args.effort_limits):
        raise ValueError('All --effort-limits values must be positive.')
    if any(v <= 0.0 for v in args.score_weights):
        raise ValueError('All --score-weights values must be positive.')

    rclpy.init()
    node = Node('left_arm_torque_pose')

    try:
        args.effort_limits = resolve_effort_limits(args, node.get_logger())
        if args.effort_limits is not None and any(v <= 0.0 for v in args.effort_limits):
            raise ValueError('All resolved effort limit values must be positive.')

        searcher = LeftArmTorquePoseSearch(node, args)
        searcher.wait_ready()
        best = searcher.search()

        if best is not None and args.execute:
            searcher.execute(best.joints)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
