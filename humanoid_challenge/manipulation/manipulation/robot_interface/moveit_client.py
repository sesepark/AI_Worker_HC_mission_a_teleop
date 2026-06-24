import os
import math
import time
import threading
from enum import Enum
from collections.abc import Iterable

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from pymoveit2 import MoveIt2
from pymoveit2.moveit2 import MoveIt2State
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose


class MoveResult(Enum):
    SUCCEEDED = 'succeeded'
    FAILED    = 'failed'
    INVALID   = 'invalid'   # goal rejected before execution (IK, planning, bad pose)
    TIMEOUT   = 'timeout'   # executor never got a result back in time


class Arm(Enum):
    RIGHT = 'right'
    LEFT  = 'left'


_ARM_R_JOINTS = [
    'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
    'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
]
_ARM_L_JOINTS = [
    'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
    'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
]
_ARM_JOINTS = {Arm.RIGHT: _ARM_R_JOINTS, Arm.LEFT: _ARM_L_JOINTS}

# MoveIt error codes that indicate planning/IK failure, not execution failure.
# Used to classify a non-success result as INVALID vs FAILED.
_PLANNING_ERROR_CODES = {
    MoveItErrorCodes.FAILURE,            # 99999 — generic planning timeout / no path found
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

# Reverse lookup int -> name, built from the installed moveit_msgs constants (no hardcoding).
# e.g. -10 -> 'START_STATE_IN_COLLISION', -31 -> 'NO_IK_SOLUTION', -1 -> 'PLANNING_FAILED'.
_ERROR_CODE_NAMES = {
    v: k for k, v in vars(MoveItErrorCodes).items()
    if k.isupper() and isinstance(v, int)
}


def _error_name(val: int) -> str:
    return _ERROR_CODE_NAMES.get(val, 'UNKNOWN')


_JOINT_STATES_TIMEOUT  = 10.0
_DEFAULT_PLANNING_TIME = float(os.environ.get('MOVEIT_PLANNING_TIME', '5.0'))
_DEFAULT_PLANNING_ATTEMPTS = int(os.environ.get('MOVEIT_PLANNING_ATTEMPTS', '5'))
_POSE_TOL_POSITION = float(os.environ.get('MOVEIT_POSE_TOL_POSITION', '0.001'))
_POSE_TOL_ORIENTATION = float(os.environ.get('MOVEIT_POSE_TOL_ORIENTATION', '0.01'))
_IK_TIMEOUT = 5.0


class MoveItClient:

    def __init__(self, node: Node, *, manage_executor: bool = True):
        """MoveIt2 wrapper.

        manage_executor=True (default, standalone/test usage): this client owns a
        MultiThreadedExecutor in a daemon thread spinning `node`, and __init__ blocks
        until move_group servers + joint states are ready.
        manage_executor=False: the CALLER owns the (single) executor that spins `node`.
        __init__ does NOT create an executor and does NOT block — the caller must spin
        the node and then call `wait_until_ready()`. Use this when `node` is added to an
        external executor (e.g. a long-running server) to avoid a node being spun by two
        executors (which corrupts the action-client wait set: rcl_action action_client.c:659).
        """
        self._node      = node
        self._log       = node.get_logger()
        self._destroyed = False
        self._manage_executor = manage_executor
        self._executor = None
        self._executor_thread = None

        # ReentrantCallbackGroup allows action feedback and result callbacks
        # to fire concurrently inside the MultiThreadedExecutor.
        self._cb_group = ReentrantCallbackGroup()

        self._moveit_r = MoveIt2(
            node=self._node,
            joint_names=_ARM_R_JOINTS,
            base_link_name='base_link',
            end_effector_name='end_effector_r_link',
            group_name='arm_r',
            callback_group=self._cb_group,
            use_move_group_action=True,
        )
        self._moveit_l = MoveIt2(
            node=self._node,
            joint_names=_ARM_L_JOINTS,
            base_link_name='base_link',
            end_effector_name='end_effector_l_link',
            group_name='arm_l',
            callback_group=self._cb_group,
            use_move_group_action=True,
        )
        self._moveit_lift = MoveIt2(
            node=self._node,
            joint_names=['lift_joint'],
            base_link_name='base_link',
            end_effector_name='lift_link',
            group_name='lift',
            callback_group=self._cb_group,
            use_move_group_action=True,
        )

        # pymoveit2 subscribes to /joint_states with VOLATILE QoS by default.
        # bringup publishes with TRANSIENT_LOCAL, which is incompatible.
        # Override by adding subscriptions with matching QoS that feed pymoveit2's callback.
        from sensor_msgs.msg import JointState
        _js_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for mv in (self._moveit_r, self._moveit_l, self._moveit_lift):
            self._node.create_subscription(
                JointState,
                'joint_states',
                mv._MoveIt2__joint_state_callback,
                _js_qos,
                callback_group=self._cb_group,
            )

        # Per-arm locks prevent concurrent callers from corrupting shared mutable state
        # (pipeline_id, max_velocity, etc.) on the same MoveIt2 instance.
        self._lock_r    = threading.Lock()
        self._lock_l    = threading.Lock()
        self._lock_lift = threading.Lock()

        # Executor runs in a daemon thread — all ROS2 callbacks (action results,
        # joint state updates, FK responses) are handled automatically without
        # the main thread ever calling spin_once.
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin,
            daemon=True,
        )
        self._executor_thread.start()

        self._wait_for_servers()
        self._wait_for_joint_states()

    @property
    def node(self) -> Node:
        """ROS node owned by this client."""
        return self._node

    @property
    def moveit2_r(self) -> MoveIt2:
        """Right-arm MoveIt2 handle for planning-scene helpers."""
        return self._moveit_r

    @property
    def moveit2_l(self) -> MoveIt2:
        """Left-arm MoveIt2 handle for planning-scene helpers."""
        return self._moveit_l

    # ------------------------------------------------------------------
    # Startup helpers
    # ------------------------------------------------------------------

    def _wait_for_servers(self) -> None:
        """Block until all MoveGroup action servers (arms + lift) are reachable."""
        _all = [('arm_r', self._moveit_r), ('arm_l', self._moveit_l), ('lift', self._moveit_lift)]
        for label, moveit in _all:
            self._log.info(f'Waiting for move_group action server [{label}]...')
            while not moveit._MoveIt2__move_action_client.wait_for_server(timeout_sec=1.0):
                self._log.warn(f'[{label}] move_group not available, retrying...')

        deadline = time.time() + 10.0
        for label, moveit in _all:
            client = moveit._MoveIt2__move_action_client
            while not client.server_is_ready():
                if time.time() > deadline:
                    raise RuntimeError(
                        f'[{label}] server_is_ready() never stabilised after wait_for_server() succeeded'
                    )
                self._log.warn(f'[{label}] server_is_ready() False — waiting for DDS to stabilise...')
                time.sleep(0.1)

        self._log.info('move_group action servers ready.')

    def _wait_for_joint_states(self) -> None:
        """Block until joint states arrive for both arms, or raise after timeout."""
        self._log.info('Waiting for joint states...')
        start = time.time()
        while self._moveit_r.joint_state is None or self._moveit_l.joint_state is None:
            if time.time() - start > _JOINT_STATES_TIMEOUT:
                raise RuntimeError(
                    f'Joint states not received within {_JOINT_STATES_TIMEOUT}s. '
                    'Check that /joint_states is publishing and QoS is compatible.'
                )
            time.sleep(0.05)
        self._log.info('Joint states ready.')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _moveit(self, arm: Arm) -> MoveIt2:
        return self._moveit_r if arm == Arm.RIGHT else self._moveit_l

    def _lock(self, arm: Arm) -> threading.Lock:
        return self._lock_r if arm == Arm.RIGHT else self._lock_l

    def _guard(self) -> None:
        if self._destroyed:
            raise RuntimeError('MoveItClient has been destroyed — create a new instance.')

    def _log_pose(self, label: str, arm: Arm, pose: Pose,
                  vel: float, acc: float, tol_pos: float, tol_ori: float) -> None:
        p = pose.position
        o = pose.orientation
        self._log.info(
            f'[{label}] [{arm.value}] '
            f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
            f'quat=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f}) '
            f'| vel={vel} acc={acc} tol_pos={tol_pos} tol_ori={tol_ori}'
        )

    def _configure(self, moveit2: MoveIt2, vel: float, acc: float,
                   pipeline: str = 'ompl', planner: str = 'RRTConnect') -> None:
        moveit2.motion_suceeded       = False  # typo is in pymoveit2's public API
        moveit2.pipeline_id           = pipeline
        moveit2.planner_id            = planner
        moveit2.max_velocity          = vel
        moveit2.max_acceleration      = acc
        moveit2.allowed_planning_time = _DEFAULT_PLANNING_TIME
        moveit2.num_planning_attempts = _DEFAULT_PLANNING_ATTEMPTS

    def _wait(self, moveit2: MoveIt2, label: str, arm: Arm, timeout: float) -> MoveResult:
        """Poll query_state() until done or timeout. Executor thread drives all callbacks.

        query_state() is the public API — it returns MoveIt2State.REQUESTING,
        EXECUTING, or IDLE. We cannot use wait_until_executed() because it calls
        rclpy.spin_once() internally, which conflicts with our executor thread.

        Two-phase wait: first confirm the goal left IDLE (guards against a race where
        the background executor processes a rejection callback before this loop's first
        iteration, making query_state() appear IDLE before we ever observed REQUESTING).
        """
        start = time.time()

        # Phase 1: wait until the goal is in-flight (non-IDLE).
        # Timeout of 2s here covers slow DDS goal acceptance; if we never leave IDLE
        # the action server dropped the goal entirely.
        _ACCEPT_TIMEOUT = 2.0
        while moveit2.query_state() == MoveIt2State.IDLE:
            if time.time() - start > _ACCEPT_TIMEOUT:
                self._log.error(
                    f'[{label}] [{arm.value}] goal never left IDLE — '
                    'action server may have dropped the request'
                )
                return MoveResult.INVALID
            time.sleep(0.01)

        # Phase 2: wait for completion (return to IDLE).
        while moveit2.query_state() != MoveIt2State.IDLE:
            elapsed = time.time() - start
            if elapsed > timeout:
                self._log.error(
                    f'[{label}] [{arm.value}] TIMEOUT after {elapsed:.1f}s '
                    f'| state={moveit2.query_state().name}'
                )
                return MoveResult.TIMEOUT
            time.sleep(0.05)

        elapsed = time.time() - start

        if moveit2.motion_suceeded:  # typo is in pymoveit2's public API
            self._log.info(f'[{label}] [{arm.value}] SUCCEEDED in {elapsed:.2f}s')
            return MoveResult.SUCCEEDED

        error     = moveit2.get_last_execution_error_code()
        error_val = error.val if error is not None else MoveItErrorCodes.UNDEFINED

        if error_val in _PLANNING_ERROR_CODES:
            self._log.error(
                f'[{label}] [{arm.value}] INVALID in {elapsed:.2f}s '
                f'| error_code={error_val} — planning/IK failure (unreachable pose or no IK solution)'
            )
            return MoveResult.INVALID

        self._log.error(
            f'[{label}] [{arm.value}] FAILED in {elapsed:.2f}s '
            f'| error_code={error_val} — execution error (joint limits, collision, controller)'
        )
        return MoveResult.FAILED

    def _classify_error_code(self, error_code: int) -> MoveResult:
        if error_code == MoveItErrorCodes.SUCCESS:
            return MoveResult.SUCCEEDED
        if error_code in _PLANNING_ERROR_CODES:
            return MoveResult.INVALID
        return MoveResult.FAILED

    def _trajectory_metrics(self, trajectory) -> dict:
        points = list(getattr(trajectory, 'points', []))
        metrics = {
            'point_count': len(points),
            'joint_path_length': 0.0,
            'max_joint_step': 0.0,
            'planned_duration_s': 0.0,
        }
        if not points:
            return metrics

        previous = None
        for point in points:
            current = list(point.positions)
            if previous is not None and current and len(current) == len(previous):
                step = math.sqrt(sum((a - b) ** 2 for a, b in zip(current, previous)))
                metrics['joint_path_length'] += step
                metrics['max_joint_step'] = max(metrics['max_joint_step'], step)
            previous = current

        final_time = points[-1].time_from_start
        metrics['planned_duration_s'] = float(final_time.sec) + float(final_time.nanosec) / 1e9
        metrics['joint_path_length'] = round(metrics['joint_path_length'], 6)
        metrics['max_joint_step'] = round(metrics['max_joint_step'], 6)
        metrics['planned_duration_s'] = round(metrics['planned_duration_s'], 6)
        return metrics

    # ------------------------------------------------------------------
    # Move functions
    # ------------------------------------------------------------------

    def move_to_pose(
        self,
        pose: Pose,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 30.0,
        pipeline: str = 'ompl',
        planner: str = 'RRTConnect',
        fallback_planners: Iterable[str] | None = None,
    ) -> MoveResult:
        """Free-space motion to a Cartesian pose. Defaults to OMPL RRTConnect.

        fallback_planners can be supplied by production callers that want a
        deliberate planner cascade (for example RRTConnect -> LBKPIECE). The
        experiment harness passes an empty tuple so planner comparisons stay clean.
        """
        self._guard()
        tol_pos, tol_ori = _POSE_TOL_POSITION, _POSE_TOL_ORIENTATION
        self._log_pose('move_to_pose', arm, pose, velocity, acceleration, tol_pos, tol_ori)

        with self._lock(arm):
            moveit2 = self._moveit(arm)
            self._configure(moveit2, velocity, acceleration, pipeline, planner)
            moveit2.move_to_pose(
                pose=pose,
                tolerance_position=tol_pos,
                tolerance_orientation=tol_ori,
            )
            result = self._wait(moveit2, 'move_to_pose', arm, timeout)

        fallback_planners = tuple(fallback_planners or ())
        if result == MoveResult.INVALID and fallback_planners:
            next_planner = fallback_planners[0]
            remaining = fallback_planners[1:]
            self._log.warn(
                f'[move_to_pose] [{arm.value}] {planner} failed — retrying with {next_planner}'
            )
            return self.move_to_pose(
                pose, arm=arm, velocity=velocity, acceleration=acceleration,
                timeout=timeout, pipeline=pipeline, planner=next_planner,
                fallback_planners=remaining,
            )

        return result

    def plan_to_pose(
        self,
        pose: Pose,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 30.0,
        pipeline: str = 'ompl',
        planner: str = 'RRTConnect',
        start_joint_state: list[float] | None = None,
    ) -> MoveResult:
        """Plan to a Cartesian pose without executing the trajectory."""
        return self.plan_to_pose_details(
            pose=pose,
            arm=arm,
            velocity=velocity,
            acceleration=acceleration,
            timeout=timeout,
            pipeline=pipeline,
            planner=planner,
            start_joint_state=start_joint_state,
        )['result']

    def plan_to_pose_details(
        self,
        pose: Pose,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 30.0,
        pipeline: str = 'ompl',
        planner: str = 'RRTConnect',
        start_joint_state: list[float] | None = None,
        cartesian: bool = False,
        cartesian_max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.999,
    ) -> dict:
        """Plan to a Cartesian pose without executing, returning quality metrics."""
        self._guard()
        tol_pos, tol_ori = _POSE_TOL_POSITION, _POSE_TOL_ORIENTATION
        label = 'plan_cartesian' if cartesian else 'plan_to_pose'
        self._log_pose(label, arm, pose, velocity, acceleration, tol_pos, tol_ori)

        with self._lock(arm):
            moveit2 = self._moveit(arm)
            self._configure(moveit2, velocity, acceleration, pipeline, planner)
            future = moveit2.plan_async(
                pose=pose,
                tolerance_position=tol_pos,
                tolerance_orientation=tol_ori,
                start_joint_state=start_joint_state,
                cartesian=cartesian,
                max_step=cartesian_max_step,
            )
            if future is None:
                self._log.error(f'[{label}] [{arm.value}] plan_async returned None')
                return {'result': MoveResult.INVALID, 'elapsed_s': 0.0}

            start = time.time()
            while not future.done():
                if time.time() - start > timeout:
                    self._log.error(
                        f'[{label}] [{arm.value}] TIMEOUT after {timeout:.1f}s'
                    )
                    return {'result': MoveResult.TIMEOUT, 'elapsed_s': round(time.time() - start, 3)}
                time.sleep(0.05)

            response = future.result()
            elapsed = time.time() - start
            if response is None:
                self._log.error(f'[{label}] [{arm.value}] empty planning response')
                return {'result': MoveResult.INVALID, 'elapsed_s': round(elapsed, 3)}

            details = {'elapsed_s': round(elapsed, 3)}

            if cartesian:
                result = self._classify_error_code(response.error_code.val)
                details['cartesian_fraction'] = round(float(response.fraction), 6)
                if response.fraction < cartesian_fraction_threshold:
                    result = MoveResult.INVALID
                trajectory = response.solution.joint_trajectory
                details.update(self._trajectory_metrics(trajectory))
                details['result'] = result
                if result == MoveResult.SUCCEEDED:
                    self._log.info(
                        f'[{label}] [{arm.value}] SUCCEEDED in {elapsed:.2f}s '
                        f'| fraction={response.fraction:.3f} '
                        f'| points={details["point_count"]} '
                        f'| joint_path={details["joint_path_length"]:.3f}'
                    )
                else:
                    self._log.error(
                        f'[{label}] [{arm.value}] {result.value.upper()} in {elapsed:.2f}s '
                        f'| fraction={response.fraction:.3f} '
                        f'| error_code={response.error_code.val}'
                    )
                return details

            motion_response = response.motion_plan_response
            result = self._classify_error_code(motion_response.error_code.val)
            trajectory = motion_response.trajectory.joint_trajectory
            details.update(self._trajectory_metrics(trajectory))
            details['result'] = result
            if result == MoveResult.SUCCEEDED:
                self._log.info(
                    f'[{label}] [{arm.value}] SUCCEEDED in {elapsed:.2f}s '
                    f'| points={details["point_count"]} '
                    f'| joint_path={details["joint_path_length"]:.3f}'
                )
            else:
                self._log.error(
                    f'[{label}] [{arm.value}] {result.value.upper()} in {elapsed:.2f}s '
                    f'| error_code={motion_response.error_code.val}'
                )
            return details

    def move_to_joints(
        self,
        joint_positions: list[float],
        arm: Arm = Arm.RIGHT,import os
import math
import time
import threading
from enum import Enum
from collections.abc import Iterable

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from pymoveit2 import MoveIt2
from pymoveit2.moveit2 import MoveIt2State
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose


class MoveResult(Enum):
    SUCCEEDED = 'succeeded'
    FAILED    = 'failed'
    INVALID   = 'invalid'   # goal rejected before execution (IK, planning, bad pose)
    TIMEOUT   = 'timeout'   # executor never got a result back in time


class Arm(Enum):
    RIGHT = 'right'
    LEFT  = 'left'


_ARM_R_JOINTS = [
    'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
    'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
]
_ARM_L_JOINTS = [
    'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
    'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
]
_ARM_JOINTS = {Arm.RIGHT: _ARM_R_JOINTS, Arm.LEFT: _ARM_L_JOINTS}

# MoveIt error codes that indicate planning/IK failure, not execution failure.
# Used to classify a non-success result as INVALID vs FAILED.
_PLANNING_ERROR_CODES = {
    MoveItErrorCodes.FAILURE,            # 99999 — generic planning timeout / no path found
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

# Reverse lookup int -> name, built from the installed moveit_msgs constants (no hardcoding).
# e.g. -10 -> 'START_STATE_IN_COLLISION', -31 -> 'NO_IK_SOLUTION', -1 -> 'PLANNING_FAILED'.
_ERROR_CODE_NAMES = {
    v: k for k, v in vars(MoveItErrorCodes).items()
    if k.isupper() and isinstance(v, int)
}


def _error_name(val: int) -> str:
    return _ERROR_CODE_NAMES.get(val, 'UNKNOWN')


_JOINT_STATES_TIMEOUT  = 10.0
_DEFAULT_PLANNING_TIME = float(os.environ.get('MOVEIT_PLANNING_TIME', '5.0'))
_DEFAULT_PLANNING_ATTEMPTS = int(os.environ.get('MOVEIT_PLANNING_ATTEMPTS', '5'))
_POSE_TOL_POSITION = float(os.environ.get('MOVEIT_POSE_TOL_POSITION', '0.001'))
_POSE_TOL_ORIENTATION = float(os.environ.get('MOVEIT_POSE_TOL_ORIENTATION', '0.01'))
_IK_TIMEOUT = 5.0


class MoveItClient:

    def __init__(self, node: Node, *, manage_executor: bool = True):
        """MoveIt2 wrapper.

        manage_executor=True (default, standalone/test usage): this client owns a
        MultiThreadedExecutor in a daemon thread spinning `node`, and __init__ blocks
        until move_group servers + joint states are ready.
        manage_executor=False: the CALLER owns the (single) executor that spins `node`.
        __init__ does NOT create an executor and does NOT block — the caller must spin
        the node and then call `wait_until_ready()`. Use this when `node` is added to an
        external executor (e.g. a long-running server) to avoid a node being spun by two
        executors (which corrupts the action-client wait set: rcl_action action_client.c:659).
        """
        self._node      = node
        self._log       = node.get_logger()
        self._destroyed = False
        self._manage_executor = manage_executor
        self._executor = None
        self._executor_thread = None

        # ReentrantCallbackGroup allows action feedback and result callbacks
        # to fire concurrently inside the MultiThreadedExecutor.
        self._cb_group = ReentrantCallbackGroup()

        self._moveit_r = MoveIt2(
            node=self._node,
            joint_names=_ARM_R_JOINTS,
            base_link_name='base_link',
            end_effector_name='end_effector_r_link',
            group_name='arm_r',
            callback_group=self._cb_group,
            use_move_group_action=True,
        )
        self._moveit_l = MoveIt2(
            node=self._node,
            joint_names=_ARM_L_JOINTS,
            base_link_name='base_link',
            end_effector_name='end_effector_l_link',
            group_name='arm_l',
            callback_group=self._cb_group,
            use_move_group_action=True,
        )
        self._moveit_lift = MoveIt2(
            node=self._node,
            joint_names=['lift_joint'],
            base_link_name='base_link',
            end_effector_name='lift_link',
            group_name='lift',
            callback_group=self._cb_group,
            use_move_group_action=True,
        )

        # pymoveit2 subscribes to /joint_states with VOLATILE QoS by default.
        # bringup publishes with TRANSIENT_LOCAL, which is incompatible.
        # Override by adding subscriptions with matching QoS that feed pymoveit2's callback.
        from sensor_msgs.msg import JointState
        _js_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for mv in (self._moveit_r, self._moveit_l, self._moveit_lift):
            self._node.create_subscription(
                JointState,
                'joint_states',
                mv._MoveIt2__joint_state_callback,
                _js_qos,
                callback_group=self._cb_group,
            )

        # Per-arm locks prevent concurrent callers from corrupting shared mutable state
        # (pipeline_id, max_velocity, etc.) on the same MoveIt2 instance.
        self._lock_r    = threading.Lock()
        self._lock_l    = threading.Lock()
        self._lock_lift = threading.Lock()

        # Executor runs in a daemon thread — all ROS2 callbacks (action results,
        # joint state updates, FK responses) are handled automatically without
        # the main thread ever calling spin_once.
        # When manage_executor=False the caller owns the single executor that spins
        # this node, so we neither create our own nor block here (see wait_until_ready()).
        if self._manage_executor:
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._executor_thread = threading.Thread(
                target=self._executor.spin,
                daemon=True,
            )
            self._executor_thread.start()

            self.wait_until_ready()

    @property
    def node(self) -> Node:
        """ROS node owned by this client."""
        return self._node

    @property
    def moveit2_r(self) -> MoveIt2:
        """Right-arm MoveIt2 handle for planning-scene helpers."""
        return self._moveit_r

    @property
    def moveit2_l(self) -> MoveIt2:
        """Left-arm MoveIt2 handle for planning-scene helpers."""
        return self._moveit_l

    # ------------------------------------------------------------------
    # Startup helpers
    # ------------------------------------------------------------------

    def wait_until_ready(self) -> None:
        """Block until move_group servers + joint states are ready.

        Called automatically when manage_executor=True. With manage_executor=False the
        caller must call this AFTER it has started spinning the node (otherwise the
        blocking waits below never receive messages).
        """
        self._wait_for_servers()
        self._wait_for_joint_states()

    def _wait_for_servers(self) -> None:
        """Block until all MoveGroup action servers (arms + lift) are reachable."""
        _all = [('arm_r', self._moveit_r), ('arm_l', self._moveit_l), ('lift', self._moveit_lift)]
        for label, moveit in _all:
            self._log.info(f'Waiting for move_group action server [{label}]...')
            while not moveit._MoveIt2__move_action_client.wait_for_server(timeout_sec=1.0):
                self._log.warn(f'[{label}] move_group not available, retrying...')

        deadline = time.time() + 10.0
        for label, moveit in _all:
            client = moveit._MoveIt2__move_action_client
            while not client.server_is_ready():
                if time.time() > deadline:
                    raise RuntimeError(
                        f'[{label}] server_is_ready() never stabilised after wait_for_server() succeeded'
                    )
                self._log.warn(f'[{label}] server_is_ready() False — waiting for DDS to stabilise...')
                time.sleep(0.1)

        self._log.info('move_group action servers ready.')

    def _wait_for_joint_states(self) -> None:
        """Block until joint states arrive for both arms, or raise after timeout."""
        self._log.info('Waiting for joint states...')
        start = time.time()
        while self._moveit_r.joint_state is None or self._moveit_l.joint_state is None:
            if time.time() - start > _JOINT_STATES_TIMEOUT:
                raise RuntimeError(
                    f'Joint states not received within {_JOINT_STATES_TIMEOUT}s. '
                    'Check that /joint_states is publishing and QoS is compatible.'
                )
            time.sleep(0.05)
        self._log.info('Joint states ready.')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _moveit(self, arm: Arm) -> MoveIt2:
        return self._moveit_r if arm == Arm.RIGHT else self._moveit_l

    def _lock(self, arm: Arm) -> threading.Lock:
        return self._lock_r if arm == Arm.RIGHT else self._lock_l

    def _guard(self) -> None:
        if self._destroyed:
            raise RuntimeError('MoveItClient has been destroyed — create a new instance.')

    def _assert_fresh_joint_state(self, arm: Arm, max_age: float = 1.0) -> bool:
        """Ensure the cached joint_state (used as the plan start_state) is present and recent.

        pymoveit2 sets MotionPlanRequest.start_state.joint_state from self.joint_state. A
        missing/stale sample yields a garbage start state and a misleading
        START_STATE_IN_COLLISION (-10). Wait briefly for a sample; abort the plan with a
        clear message if it is missing, rather than planning from an unreliable start state.
        Freshness is best-effort: skipped for unstamped publishers / clock-domain mismatches
        so the guard never falsely blocks legitimate motion.
        """
        moveit2 = self._moveit(arm)
        deadline = time.time() + 0.5
        js = moveit2.joint_state
        while js is None and time.time() < deadline:
            time.sleep(0.02)
            js = moveit2.joint_state
        if js is None:
            self._log.error(
                f'[{arm.value}] joint_state missing — start state unreliable; aborting plan'
            )
            return False
        stamp = js.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return True  # unstamped publisher — presence is all we can verify
        age = self._node.get_clock().now().nanoseconds / 1e9 - (stamp.sec + stamp.nanosec / 1e9)
        if max_age < age < 3600.0:  # clearly stale, but not a clock-domain mismatch
            self._log.error(
                f'[{arm.value}] joint_state stale (age={age:.2f}s > {max_age}s) — '
                'start state unreliable; aborting plan'
            )
            return False
        return True

    def _log_pose(self, label: str, arm: Arm, pose: Pose,
                  vel: float, acc: float, tol_pos: float, tol_ori: float) -> None:
        p = pose.position
        o = pose.orientation
        self._log.info(
            f'[{label}] [{arm.value}] '
            f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
            f'quat=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f}) '
            f'| vel={vel} acc={acc} tol_pos={tol_pos} tol_ori={tol_ori}'
        )

    def _configure(self, moveit2: MoveIt2, vel: float, acc: float,
                   pipeline: str = 'ompl', planner: str = 'RRTConnect') -> None:
        moveit2.motion_suceeded       = False  # typo is in pymoveit2's public API
        moveit2.pipeline_id           = pipeline
        moveit2.planner_id            = planner
        moveit2.max_velocity          = vel
        moveit2.max_acceleration      = acc
        moveit2.allowed_planning_time = _DEFAULT_PLANNING_TIME
        moveit2.num_planning_attempts = _DEFAULT_PLANNING_ATTEMPTS

    def _wait(self, moveit2: MoveIt2, label: str, arm: Arm, timeout: float) -> MoveResult:
        """Poll query_state() until done or timeout. Executor thread drives all callbacks.

        query_state() is the public API — it returns MoveIt2State.REQUESTING,
        EXECUTING, or IDLE. We cannot use wait_until_executed() because it calls
        rclpy.spin_once() internally, which conflicts with our executor thread.

        Two-phase wait: first confirm the goal left IDLE (guards against a race where
        the background executor processes a rejection callback before this loop's first
        iteration, making query_state() appear IDLE before we ever observed REQUESTING).
        """
        start = time.time()

        # Phase 1: wait until the goal is in-flight (non-IDLE).
        # Timeout of 2s here covers slow DDS goal acceptance; if we never leave IDLE
        # the action server dropped the goal entirely.
        _ACCEPT_TIMEOUT = 2.0
        while moveit2.query_state() == MoveIt2State.IDLE:
            if time.time() - start > _ACCEPT_TIMEOUT:
                self._log.error(
                    f'[{label}] [{arm.value}] goal never left IDLE — '
                    'action server may have dropped the request'
                )
                return MoveResult.INVALID
            time.sleep(0.01)

        # Phase 2: wait for completion (return to IDLE).
        while moveit2.query_state() != MoveIt2State.IDLE:
            elapsed = time.time() - start
            if elapsed > timeout:
                self._log.error(
                    f'[{label}] [{arm.value}] TIMEOUT after {elapsed:.1f}s '
                    f'| state={moveit2.query_state().name}'
                )
                return MoveResult.TIMEOUT
            time.sleep(0.05)

        elapsed = time.time() - start

        if moveit2.motion_suceeded:  # typo is in pymoveit2's public API
            self._log.info(f'[{label}] [{arm.value}] SUCCEEDED in {elapsed:.2f}s')
            return MoveResult.SUCCEEDED

        error     = moveit2.get_last_execution_error_code()
        error_val = error.val if error is not None else MoveItErrorCodes.UNDEFINED

        if error_val in _PLANNING_ERROR_CODES:
            self._log.error(
                f'[{label}] [{arm.value}] INVALID in {elapsed:.2f}s '
                f'| error_code={error_val} ({_error_name(error_val)}) — planning rejected before execution'
            )
            return MoveResult.INVALID

        self._log.error(
            f'[{label}] [{arm.value}] FAILED in {elapsed:.2f}s '
            f'| error_code={error_val} ({_error_name(error_val)}) — execution error (joint limits, collision, controller)'
        )
        return MoveResult.FAILED

    def _classify_error_code(self, error_code: int) -> MoveResult:
        if error_code == MoveItErrorCodes.SUCCESS:
            return MoveResult.SUCCEEDED
        if error_code in _PLANNING_ERROR_CODES:
            return MoveResult.INVALID
        return MoveResult.FAILED

    def _trajectory_metrics(self, trajectory) -> dict:
        points = list(getattr(trajectory, 'points', []))
        metrics = {
            'point_count': len(points),
            'joint_path_length': 0.0,
            'max_joint_step': 0.0,
            'planned_duration_s': 0.0,
        }
        if not points:
            return metrics

        previous = None
        for point in points:
            current = list(point.positions)
            if previous is not None and current and len(current) == len(previous):
                step = math.sqrt(sum((a - b) ** 2 for a, b in zip(current, previous)))
                metrics['joint_path_length'] += step
                metrics['max_joint_step'] = max(metrics['max_joint_step'], step)
            previous = current

        final_time = points[-1].time_from_start
        metrics['planned_duration_s'] = float(final_time.sec) + float(final_time.nanosec) / 1e9
        metrics['joint_path_length'] = round(metrics['joint_path_length'], 6)
        metrics['max_joint_step'] = round(metrics['max_joint_step'], 6)
        metrics['planned_duration_s'] = round(metrics['planned_duration_s'], 6)
        return metrics

    # ------------------------------------------------------------------
    # Move functions
    # ------------------------------------------------------------------

    def move_to_pose(
        self,
        pose: Pose,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 30.0,
        pipeline: str = 'ompl',
        planner: str = 'RRTConnect',
        fallback_planners: Iterable[str] | None = None,
    ) -> MoveResult:
        """Free-space motion to a Cartesian pose. Defaults to OMPL RRTConnect.

        fallback_planners can be supplied by production callers that want a
        deliberate planner cascade (for example RRTConnect -> LBKPIECE). The
        experiment harness passes an empty tuple so planner comparisons stay clean.
        """
        self._guard()
        if not self._assert_fresh_joint_state(arm):
            return MoveResult.INVALID
        tol_pos, tol_ori = _POSE_TOL_POSITION, _POSE_TOL_ORIENTATION
        self._log_pose('move_to_pose', arm, pose, velocity, acceleration, tol_pos, tol_ori)

        with self._lock(arm):
            moveit2 = self._moveit(arm)
            self._configure(moveit2, velocity, acceleration, pipeline, planner)
            moveit2.move_to_pose(
                pose=pose,
                tolerance_position=tol_pos,
                tolerance_orientation=tol_ori,
            )
            result = self._wait(moveit2, 'move_to_pose', arm, timeout)

        fallback_planners = tuple(fallback_planners or ())
        if result == MoveResult.INVALID and fallback_planners:
            next_planner = fallback_planners[0]
            remaining = fallback_planners[1:]
            self._log.warn(
                f'[move_to_pose] [{arm.value}] {planner} failed — retrying with {next_planner}'
            )
            return self.move_to_pose(
                pose, arm=arm, velocity=velocity, acceleration=acceleration,
                timeout=timeout, pipeline=pipeline, planner=next_planner,
                fallback_planners=remaining,
            )

        return result

    def plan_to_pose(
        self,
        pose: Pose,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 30.0,
        pipeline: str = 'ompl',
        planner: str = 'RRTConnect',
        start_joint_state: list[float] | None = None,
    ) -> MoveResult:
        """Plan to a Cartesian pose without executing the trajectory."""
        return self.plan_to_pose_details(
            pose=pose,
            arm=arm,
            velocity=velocity,
            acceleration=acceleration,
            timeout=timeout,
            pipeline=pipeline,
            planner=planner,
            start_joint_state=start_joint_state,
        )['result']

    def plan_to_pose_details(
        self,
        pose: Pose,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 30.0,
        pipeline: str = 'ompl',
        planner: str = 'RRTConnect',
        start_joint_state: list[float] | None = None,
        cartesian: bool = False,
        cartesian_max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.999,
    ) -> dict:
        """Plan to a Cartesian pose without executing, returning quality metrics."""
        self._guard()
        tol_pos, tol_ori = _POSE_TOL_POSITION, _POSE_TOL_ORIENTATION
        label = 'plan_cartesian' if cartesian else 'plan_to_pose'
        self._log_pose(label, arm, pose, velocity, acceleration, tol_pos, tol_ori)

        with self._lock(arm):
            moveit2 = self._moveit(arm)
            self._configure(moveit2, velocity, acceleration, pipeline, planner)
            future = moveit2.plan_async(
                pose=pose,
                tolerance_position=tol_pos,
                tolerance_orientation=tol_ori,
                start_joint_state=start_joint_state,
                cartesian=cartesian,
                max_step=cartesian_max_step,
            )
            if future is None:
                self._log.error(f'[{label}] [{arm.value}] plan_async returned None')
                return {'result': MoveResult.INVALID, 'elapsed_s': 0.0}

            start = time.time()
            while not future.done():
                if time.time() - start > timeout:
                    self._log.error(
                        f'[{label}] [{arm.value}] TIMEOUT after {timeout:.1f}s'
                    )
                    return {'result': MoveResult.TIMEOUT, 'elapsed_s': round(time.time() - start, 3)}
                time.sleep(0.05)

            response = future.result()
            elapsed = time.time() - start
            if response is None:
                self._log.error(f'[{label}] [{arm.value}] empty planning response')
                return {'result': MoveResult.INVALID, 'elapsed_s': round(elapsed, 3)}

            details = {'elapsed_s': round(elapsed, 3)}

            if cartesian:
                result = self._classify_error_code(response.error_code.val)
                details['cartesian_fraction'] = round(float(response.fraction), 6)
                if response.fraction < cartesian_fraction_threshold:
                    result = MoveResult.INVALID
                trajectory = response.solution.joint_trajectory
                details.update(self._trajectory_metrics(trajectory))
                details['result'] = result
                if result == MoveResult.SUCCEEDED:
                    self._log.info(
                        f'[{label}] [{arm.value}] SUCCEEDED in {elapsed:.2f}s '
                        f'| fraction={response.fraction:.3f} '
                        f'| points={details["point_count"]} '
                        f'| joint_path={details["joint_path_length"]:.3f}'
                    )
                else:
                    self._log.error(
                        f'[{label}] [{arm.value}] {result.value.upper()} in {elapsed:.2f}s '
                        f'| fraction={response.fraction:.3f} '
                        f'| error_code={response.error_code.val} ({_error_name(response.error_code.val)})'
                    )
                return details

            motion_response = response.motion_plan_response
            result = self._classify_error_code(motion_response.error_code.val)
            trajectory = motion_response.trajectory.joint_trajectory
            details.update(self._trajectory_metrics(trajectory))
            details['result'] = result
            if result == MoveResult.SUCCEEDED:
                self._log.info(
                    f'[{label}] [{arm.value}] SUCCEEDED in {elapsed:.2f}s '
                    f'| points={details["point_count"]} '
                    f'| joint_path={details["joint_path_length"]:.3f}'
                )
            else:
                self._log.error(
                    f'[{label}] [{arm.value}] {result.value.upper()} in {elapsed:.2f}s '
                    f'| error_code={motion_response.error_code.val} ({_error_name(motion_response.error_code.val)})'
                )
            return details

    def move_to_joints(
        self,
        joint_positions: list[float],
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 30.0,
        pipeline: str = 'ompl',
        planner: str = 'RRTConnect',
    ) -> MoveResult:
        """Move to a joint configuration. joint_positions must be 7 floats in radians."""
        self._guard()
        if len(joint_positions) != 7:
            self._log.error(
                f'[move_to_joints] [{arm.value}] expected 7 joint positions, '
                f'got {len(joint_positions)}'
            )
            return MoveResult.INVALID

        if not self._assert_fresh_joint_state(arm):
            return MoveResult.INVALID

        joints_str = ', '.join(f'{j:.3f}' for j in joint_positions)
        self._log.info(
            f'[move_to_joints] [{arm.value}] '
            f'joints=[{joints_str}] | vel={velocity} acc={acceleration}'
        )

        with self._lock(arm):
            moveit2 = self._moveit(arm)
            self._configure(moveit2, velocity, acceleration, pipeline, planner)
            moveit2.move_to_configuration(joint_positions)
            return self._wait(moveit2, 'move_to_joints', arm, timeout)

    def move_lift(
        self,
        position: float,
        velocity: float = 0.2,
        acceleration: float = 0.2,
        timeout: float = 15.0,
    ) -> MoveResult:
        """Move lift_joint to position (metres). 0.0 = top, negative = down."""
        self._guard()
        self._log.info(f'[move_lift] target={position:.3f} m')

        with self._lock_lift:
            self._moveit_lift.motion_suceeded  = False
            self._moveit_lift.max_velocity     = velocity
            self._moveit_lift.max_acceleration = acceleration
            self._moveit_lift.move_to_configuration([position])

            start = time.time()
            while self._moveit_lift.query_state() == MoveIt2State.IDLE:
                if time.time() - start > 2.0:
                    self._log.error('[move_lift] goal never left IDLE')
                    return MoveResult.INVALID
                time.sleep(0.01)

            while self._moveit_lift.query_state() != MoveIt2State.IDLE:
                if time.time() - start > timeout:
                    self._log.error(f'[move_lift] TIMEOUT after {time.time()-start:.1f}s')
                    return MoveResult.TIMEOUT
                time.sleep(0.05)

            elapsed = time.time() - start
            if self._moveit_lift.motion_suceeded:
                self._log.info(f'[move_lift] SUCCEEDED in {elapsed:.2f}s')
                return MoveResult.SUCCEEDED

            self._log.error(f'[move_lift] FAILED in {elapsed:.2f}s')
            return MoveResult.FAILED

    def move_to_home(
        self,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
    ) -> MoveResult:
        home = [0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0] if arm == Arm.RIGHT else [0.0, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
        return self.move_to_joints(
            home,
            arm=arm,
            velocity=velocity,
            acceleration=acceleration,
        )

    def move_cartesian(
        self,
        pose: Pose,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 15.0,
    ) -> MoveResult:
        """Straight-line Cartesian motion using the Pilz LIN planner."""
        self._guard()
        tol_pos, tol_ori = 0.001, 0.005
        self._log_pose('move_cartesian', arm, pose, velocity, acceleration, tol_pos, tol_ori)

        with self._lock(arm):
            moveit2 = self._moveit(arm)
            self._configure(moveit2, velocity, acceleration,
                            pipeline='pilz_industrial_motion_planner', planner='LIN')
            try:
                moveit2.move_to_pose(
                    pose=pose,
                    tolerance_position=tol_pos,
                    tolerance_orientation=tol_ori,
                )
                return self._wait(moveit2, 'move_cartesian', arm, timeout)
            finally:
                # Always restore so subsequent OMPL calls are not affected.
                moveit2.pipeline_id = 'ompl'
                moveit2.planner_id  = 'RRTConnect'

    # ------------------------------------------------------------------
    # Query methods
    # ------------------------------------------------------------------

    def get_joints(self, arm: Arm = Arm.RIGHT) -> list[float] | None:
        """Return current joint positions [joint1..joint7] in radians, or None if unavailable."""
        self._guard()
        moveit2     = self._moveit(arm)
        joint_names = _ARM_JOINTS[arm]

        if moveit2.joint_state is None:
            self._log.warn(f'[get_joints] [{arm.value}] joint state not yet available')
            return None

        js          = moveit2.joint_state
        name_to_pos = dict(zip(js.name, js.position))

        if not all(n in name_to_pos for n in joint_names):
            self._log.error(
                f'[get_joints] [{arm.value}] some joints missing from /joint_states. '
                f'Expected: {joint_names} | Received: {list(js.name)}'
            )
            return None

        positions  = [name_to_pos[n] for n in joint_names]
        joints_str = ', '.join(f'{j:.3f}' for j in positions)
        self._log.debug(f'[get_joints] [{arm.value}] [{joints_str}]')
        return positions

    def get_pose(self, arm: Arm = Arm.RIGHT) -> Pose | None:
        """Return current end-effector pose via FK, or None on failure."""
        self._guard()
        moveit2 = self._moveit(arm)

        future = moveit2.compute_fk_async()
        if future is None:
            self._log.error(
                f'[get_pose] [{arm.value}] FK request failed — is move_group running?'
            )
            return None

        timeout = 5.0
        start   = time.time()
        while not future.done():
            if time.time() - start > timeout:
                self._log.error(
                    f'[get_pose] [{arm.value}] FK response timeout after {timeout}s'
                )
                return None
            time.sleep(0.05)

        result = moveit2.get_compute_fk_result(future)
        if result is None:
            self._log.error(f'[get_pose] [{arm.value}] FK returned no result')
            return None

        p = result.pose.position
        o = result.pose.orientation
        self._log.info(
            f'[get_pose] [{arm.value}] '
            f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
            f'quat=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})'
        )
        return result.pose

    def check_reachable(self, pose: Pose, arm: Arm = Arm.RIGHT) -> bool:
        """Return True if a valid IK solution exists for pose on the given arm.

        Uses compute_ik_async + manual future polling to avoid rclpy.spin_once()
        conflict with the executor thread.
        """
        self._guard()
        with self._lock(arm):
            moveit2 = self._moveit(arm)

            position = (pose.position.x, pose.position.y, pose.position.z)
            quat     = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )

            future = moveit2.compute_ik_async(position, quat)
            if future is None:
                self._log.warn(f'[check_reachable] [{arm.value}] compute_ik_async returned None')
                return False

            deadline = time.time() + _IK_TIMEOUT
            while not future.done():
                if time.time() > deadline:
                    self._log.error(f'[check_reachable] [{arm.value}] IK timeout after {_IK_TIMEOUT}s')
                    return False
                time.sleep(0.05)

            result = moveit2.get_compute_ik_result(future)
            # get_compute_ik_result returns None on IK failure; a valid JointState on success.
            # Guard against solvers that return an empty JointState instead of None.
            reachable = result is not None and len(result.name) > 0
            self._log.info(f'[check_reachable] [{arm.value}] reachable={reachable}')
            return reachable

    def solve_ik(self, pose: Pose, arm: Arm = Arm.RIGHT) -> list[float] | None:
        """Return joint values [joint1..joint7] for pose via IK, or None on failure."""
        self._guard()
        with self._lock(arm):
            moveit2 = self._moveit(arm)

            position = (pose.position.x, pose.position.y, pose.position.z)
            quat     = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )

            future = moveit2.compute_ik_async(position, quat)
            if future is None:
                self._log.warn(f'[solve_ik] [{arm.value}] compute_ik_async returned None')
                return None

            deadline = time.time() + _IK_TIMEOUT
            while not future.done():
                if time.time() > deadline:
                    self._log.error(f'[solve_ik] [{arm.value}] IK timeout after {_IK_TIMEOUT}s')
                    return None
                time.sleep(0.05)

            result = moveit2.get_compute_ik_result(future)
            if result is None or len(result.name) == 0:
                self._log.warn(f'[solve_ik] [{arm.value}] IK failed — no solution')
                return None

            name_to_pos = dict(zip(result.name, result.position))
            joint_names = _ARM_JOINTS[arm]
            if not all(n in name_to_pos for n in joint_names):
                self._log.error(f'[solve_ik] [{arm.value}] IK result missing expected joints')
                return None

            joints = [name_to_pos[n] for n in joint_names]
            joints_str = ', '.join(f'{v:.6f}' for v in joints)
            self._log.info(f'[solve_ik] [{arm.value}] [{joints_str}]')
            return joints

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy(self) -> None:
        """Shut down the executor and join the background thread.
        Caller is responsible for destroying the node and calling rclpy.shutdown()."""
        self._destroyed = True
        if self._executor is not None:
            self._executor.shutdown()
        if self._executor_thread is not None:
            self._executor_thread.join(timeout=5.0)

        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 30.0,
        pipeline: str = 'ompl',
        planner: str = 'RRTConnect',
    ) -> MoveResult:
        """Move to a joint configuration. joint_positions must be 7 floats in radians."""
        self._guard()
        if len(joint_positions) != 7:
            self._log.error(
                f'[move_to_joints] [{arm.value}] expected 7 joint positions, '
                f'got {len(joint_positions)}'
            )
            return MoveResult.INVALID

        joints_str = ', '.join(f'{j:.3f}' for j in joint_positions)
        self._log.info(
            f'[move_to_joints] [{arm.value}] '
            f'joints=[{joints_str}] | vel={velocity} acc={acceleration}'
        )

        with self._lock(arm):
            moveit2 = self._moveit(arm)
            self._configure(moveit2, velocity, acceleration, pipeline, planner)
            moveit2.move_to_configuration(joint_positions)
            return self._wait(moveit2, 'move_to_joints', arm, timeout)

    def move_lift(
        self,
        position: float,
        velocity: float = 0.2,
        acceleration: float = 0.2,
        timeout: float = 15.0,
    ) -> MoveResult:
        """Move lift_joint to position (metres). 0.0 = top, negative = down."""
        self._guard()
        self._log.info(f'[move_lift] target={position:.3f} m')

        with self._lock_lift:
            self._moveit_lift.motion_suceeded  = False
            self._moveit_lift.max_velocity     = velocity
            self._moveit_lift.max_acceleration = acceleration
            self._moveit_lift.move_to_configuration([position])

            start = time.time()
            while self._moveit_lift.query_state() == MoveIt2State.IDLE:
                if time.time() - start > 2.0:
                    self._log.error('[move_lift] goal never left IDLE')
                    return MoveResult.INVALID
                time.sleep(0.01)

            while self._moveit_lift.query_state() != MoveIt2State.IDLE:
                if time.time() - start > timeout:
                    self._log.error(f'[move_lift] TIMEOUT after {time.time()-start:.1f}s')
                    return MoveResult.TIMEOUT
                time.sleep(0.05)

            elapsed = time.time() - start
            if self._moveit_lift.motion_suceeded:
                self._log.info(f'[move_lift] SUCCEEDED in {elapsed:.2f}s')
                return MoveResult.SUCCEEDED

            self._log.error(f'[move_lift] FAILED in {elapsed:.2f}s')
            return MoveResult.FAILED

    def move_to_home(
        self,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
    ) -> MoveResult:
        home = [0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0] if arm == Arm.RIGHT else [0.0, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
        return self.move_to_joints(
            home,
            arm=arm,
            velocity=velocity,
            acceleration=acceleration,
        )

    def move_cartesian(
        self,
        pose: Pose,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 15.0,
    ) -> MoveResult:
        """Straight-line Cartesian motion using the Pilz LIN planner."""
        self._guard()
        tol_pos, tol_ori = 0.001, 0.005
        self._log_pose('move_cartesian', arm, pose, velocity, acceleration, tol_pos, tol_ori)

        with self._lock(arm):
            moveit2 = self._moveit(arm)
            self._configure(moveit2, velocity, acceleration,
                            pipeline='pilz_industrial_motion_planner', planner='LIN')
            try:
                moveit2.move_to_pose(
                    pose=pose,
                    tolerance_position=tol_pos,
                    tolerance_orientation=tol_ori,
                )
                return self._wait(moveit2, 'move_cartesian', arm, timeout)
            finally:
                # Always restore so subsequent OMPL calls are not affected.
                moveit2.pipeline_id = 'ompl'
                moveit2.planner_id  = 'RRTConnect'

    # ------------------------------------------------------------------
    # Query methods
    # ------------------------------------------------------------------

    def get_joints(self, arm: Arm = Arm.RIGHT) -> list[float] | None:
        """Return current joint positions [joint1..joint7] in radians, or None if unavailable."""
        self._guard()
        moveit2     = self._moveit(arm)
        joint_names = _ARM_JOINTS[arm]

        if moveit2.joint_state is None:
            self._log.warn(f'[get_joints] [{arm.value}] joint state not yet available')
            return None

        js          = moveit2.joint_state
        name_to_pos = dict(zip(js.name, js.position))

        if not all(n in name_to_pos for n in joint_names):
            self._log.error(
                f'[get_joints] [{arm.value}] some joints missing from /joint_states. '
                f'Expected: {joint_names} | Received: {list(js.name)}'
            )
            return None

        positions  = [name_to_pos[n] for n in joint_names]
        joints_str = ', '.join(f'{j:.3f}' for j in positions)
        self._log.debug(f'[get_joints] [{arm.value}] [{joints_str}]')
        return positions

    def get_pose(self, arm: Arm = Arm.RIGHT) -> Pose | None:
        """Return current end-effector pose via FK, or None on failure."""
        self._guard()
        moveit2 = self._moveit(arm)

        future = moveit2.compute_fk_async()
        if future is None:
            self._log.error(
                f'[get_pose] [{arm.value}] FK request failed — is move_group running?'
            )
            return None

        timeout = 5.0
        start   = time.time()
        while not future.done():
            if time.time() - start > timeout:
                self._log.error(
                    f'[get_pose] [{arm.value}] FK response timeout after {timeout}s'
                )
                return None
            time.sleep(0.05)

        result = moveit2.get_compute_fk_result(future)
        if result is None:
            self._log.error(f'[get_pose] [{arm.value}] FK returned no result')
            return None

        p = result.pose.position
        o = result.pose.orientation
        self._log.info(
            f'[get_pose] [{arm.value}] '
            f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
            f'quat=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})'
        )
        return result.pose

    def check_reachable(self, pose: Pose, arm: Arm = Arm.RIGHT) -> bool:
        """Return True if a valid IK solution exists for pose on the given arm.

        Uses compute_ik_async + manual future polling to avoid rclpy.spin_once()
        conflict with the executor thread.
        """
        self._guard()
        with self._lock(arm):
            moveit2 = self._moveit(arm)

            position = (pose.position.x, pose.position.y, pose.position.z)
            quat     = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )

            future = moveit2.compute_ik_async(position, quat)
            if future is None:
                self._log.warn(f'[check_reachable] [{arm.value}] compute_ik_async returned None')
                return False

            deadline = time.time() + _IK_TIMEOUT
            while not future.done():
                if time.time() > deadline:
                    self._log.error(f'[check_reachable] [{arm.value}] IK timeout after {_IK_TIMEOUT}s')
                    return False
                time.sleep(0.05)

            result = moveit2.get_compute_ik_result(future)
            # get_compute_ik_result returns None on IK failure; a valid JointState on success.
            # Guard against solvers that return an empty JointState instead of None.
            reachable = result is not None and len(result.name) > 0
            self._log.info(f'[check_reachable] [{arm.value}] reachable={reachable}')
            return reachable

    def solve_ik(self, pose: Pose, arm: Arm = Arm.RIGHT) -> list[float] | None:
        """Return joint values [joint1..joint7] for pose via IK, or None on failure."""
        self._guard()
        with self._lock(arm):
            moveit2 = self._moveit(arm)

            position = (pose.position.x, pose.position.y, pose.position.z)
            quat     = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )

            future = moveit2.compute_ik_async(position, quat)
            if future is None:
                self._log.warn(f'[solve_ik] [{arm.value}] compute_ik_async returned None')
                return None

            deadline = time.time() + _IK_TIMEOUT
            while not future.done():
                if time.time() > deadline:
                    self._log.error(f'[solve_ik] [{arm.value}] IK timeout after {_IK_TIMEOUT}s')
                    return None
                time.sleep(0.05)

            result = moveit2.get_compute_ik_result(future)
            if result is None or len(result.name) == 0:
                self._log.warn(f'[solve_ik] [{arm.value}] IK failed — no solution')
                return None

            name_to_pos = dict(zip(result.name, result.position))
            joint_names = _ARM_JOINTS[arm]
            if not all(n in name_to_pos for n in joint_names):
                self._log.error(f'[solve_ik] [{arm.value}] IK result missing expected joints')
                return None

            joints = [name_to_pos[n] for n in joint_names]
            joints_str = ', '.join(f'{v:.6f}' for v in joints)
            self._log.info(f'[solve_ik] [{arm.value}] [{joints_str}]')
            return joints

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy(self) -> None:
        """Shut down the executor and join the background thread.
        Caller is responsible for destroying the node and calling rclpy.shutdown()."""
        self._destroyed = True
        self._executor.shutdown()
        self._executor_thread.join(timeout=5.0)
