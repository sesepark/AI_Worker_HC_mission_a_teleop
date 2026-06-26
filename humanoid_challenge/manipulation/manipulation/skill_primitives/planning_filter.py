"""Motion planning filter: workspace/IK fast-filter + planner selection and scoring."""

import time
from collections import Counter
from dataclasses import dataclass, field

from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import Arm, MoveItClient, MoveResult


NO_VALID_MOTION_CANDIDATE = 'no_valid_motion_candidate'


@dataclass(frozen=True)
class PlannerSpec:
    pipeline: str
    planner: str


@dataclass
class MotionSelection:
    arm: Arm
    target_pose: Pose
    pre_pose: Pose
    global_pipeline: str
    global_planner: str
    score: float
    rejection_summary: dict[str, int] = field(default_factory=dict)
    planning_details: dict = field(default_factory=dict)
    elapsed_s: float = 0.0


_DEFAULT_CFG = {
    'enabled': True,
    'result_on_exhaustion': NO_VALID_MOTION_CANDIDATE,
    'plan_timeout_s': 5.0,   # per-planner timeout
    'total_budget_s': 25.0,  # plan_timeout_s * 4 + buffer
    'workspace_bounds': {
        'x': [0.03, 0.7],
        'y': [-0.6, 0.6],
        'z': [0.50, 1.4],
    },
    'path_quality': {
        'max_joint_path_length': 5.0,
        'max_joint_step': 1.0,
    },
    'global_planners': [
        {'pipeline': 'pilz_industrial_motion_planner', 'planner': 'PTP'},
        {'pipeline': 'stomp', 'planner': 'STOMP'},
        {'pipeline': 'ompl', 'planner': 'RRTstar'},
        {'pipeline': 'ompl', 'planner': 'LBKPIECE'},
    ],
}


def _merged_dict(base: dict, override: dict | None) -> dict:
    result = {}
    for key, value in base.items():
        if isinstance(value, dict):
            result[key] = _merged_dict(value, None)
        elif isinstance(value, list):
            result[key] = list(value)
        else:
            result[key] = value
    if not override:
        return result
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merged_dict(result[key], value)
        else:
            result[key] = value
    return result


def _copy_pose(pose: Pose) -> Pose:
    p = Pose()
    p.position.x = pose.position.x
    p.position.y = pose.position.y
    p.position.z = pose.position.z
    p.orientation.x = pose.orientation.x
    p.orientation.y = pose.orientation.y
    p.orientation.z = pose.orientation.z
    p.orientation.w = pose.orientation.w
    return p


def _hover_pose(pose: Pose, offset: float) -> Pose:
    h = _copy_pose(pose)
    h.position.z = pose.position.z + offset
    return h


class PlanningFilter:
    """Workspace/IK fast-filter + planner selection for a single target pose.

    Accepts a fully resolved pose (position + orientation) and returns the
    best MotionSelection (arm + planner) based on path quality scoring.
    """

    def __init__(
        self,
        moveit: MoveItClient,
        config: dict | None = None,
        log=None,
    ):
        self._moveit = moveit
        self._cfg = _merged_dict(_DEFAULT_CFG, (config or {}).get('motion_selection', {}))
        self._root_cfg = config or {}
        self._log = log
        self.last_failure_reason = ''
        self.last_rejections: Counter[str] = Counter()
        self.last_report: list[str] = []

    def select_pose(
        self,
        pose: Pose,
        arm: Arm,
        approach_height: float = 0.10,
    ) -> MotionSelection | None:
        """Select planner for a fully resolved target pose.

        Parameters
        ----------
        pose            : Target pose in base_link frame (position + orientation already final).
        arm             : Arm to use.
        approach_height : Height above pose for hover pre-pose (must match pick_skill).
        """
        self.last_failure_reason = ''
        self.last_rejections = Counter()
        self.last_report = []

        if not self._cfg.get('enabled', True):
            self._reject(arm, 'selector_disabled')
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            return None

        start = time.monotonic()
        budget = float(self._cfg['total_budget_s'])

        if not self._within_workspace(pose):
            self._reject(arm, 'target_workspace_bounds')
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            self._log_warn('[PlanningFilter] pose out of workspace')
            return None

        pre_pose = _hover_pose(pose, offset=approach_height)
        if not self._within_workspace(pre_pose):
            self._reject(arm, 'prepose_workspace_bounds')
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            self._log_warn('[PlanningFilter] hover pose out of workspace')
            return None

        if self._elapsed(start) > budget:
            self._reject(arm, 'time_budget_exceeded')
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            return None

        if not self._moveit.check_reachable(pose, arm=arm):
            self._reject(arm, 'target_ik_failed')
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            self._log_warn('[PlanningFilter] target IK failed')
            return None
        # Brief pause so the IK service's response pipeline (pymoveit2 future +
        # DDS ack) fully clears before the next compute_ik_async call.
        # Back-to-back async IK requests on the same service client can cause the
        # second future to stall indefinitely on some rmw implementations.
        time.sleep(0.3)
        if not self._moveit.check_reachable(pre_pose, arm=arm):
            self._reject(arm, 'prepose_ik_failed')
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            self._log_warn('[PlanningFilter] pre-pose IK failed')
            return None

        for planner in self._planner_specs():
            if self._elapsed(start) > budget:
                self._reject(arm, 'time_budget_exceeded')
                break

            details = self._moveit.plan_to_pose_details(
                pre_pose,
                arm=arm,
                pipeline=planner.pipeline,
                planner=planner.planner,
                timeout=float(self._cfg['plan_timeout_s']),
            )
            if details.get('result') != MoveResult.SUCCEEDED:
                self._reject(arm, f'plan_failed:{planner.planner}')
                continue
            if not self._path_quality_ok(details, planner):
                continue

            selection = MotionSelection(
                arm=arm,
                target_pose=_copy_pose(pose),
                pre_pose=_copy_pose(pre_pose),
                global_pipeline=planner.pipeline,
                global_planner=planner.planner,
                score=round(self._score(details), 6),
                planning_details=dict(details),
                elapsed_s=round(self._elapsed(start), 3),
            )
            self._log_info(
                f'[PlanningFilter] selected arm={arm.value} '
                f'planner={planner.pipeline}/{planner.planner} '
                f'score={selection.score:.3f} elapsed={selection.elapsed_s:.3f}s'
            )
            return selection

        self.last_failure_reason = self._cfg['result_on_exhaustion']
        self._log_warn(f'[PlanningFilter] no valid planner: {dict(self.last_rejections)}')
        return None

    # ------------------------------------------------------------------
    # Filters and scoring
    # ------------------------------------------------------------------

    def _within_workspace(self, pose: Pose) -> bool:
        bounds = self._cfg['workspace_bounds']
        p = pose.position
        return (
            float(bounds['x'][0]) <= p.x <= float(bounds['x'][1])
            and float(bounds['y'][0]) <= p.y <= float(bounds['y'][1])
            and float(bounds['z'][0]) <= p.z <= float(bounds['z'][1])
        )

    def _path_quality_ok(self, details: dict, planner: PlannerSpec) -> bool:
        quality = self._cfg['path_quality']
        max_path = float(quality['max_joint_path_length'])
        max_step = float(quality['max_joint_step'])
        joint_path = float(details.get('joint_path_length', 0.0) or 0.0)
        max_joint_step = float(details.get('max_joint_step', 0.0) or 0.0)
        if joint_path > max_path or max_joint_step > max_step:
            self._log_info(
                f'[PlanningFilter] {planner.planner} quality rejected: '
                f'path_length={joint_path:.2f} (max {max_path}), '
                f'max_step={max_joint_step:.2f} (max {max_step})'
            )
            return False
        return True

    def _score(self, details: dict) -> float:
        return (
            1.0 * float(details.get('joint_path_length', 0.0) or 0.0)
            + 2.0 * float(details.get('max_joint_step', 0.0) or 0.0)
            + 0.05 * float(details.get('elapsed_s', 0.0) or 0.0)
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _planner_specs(self) -> list[PlannerSpec]:
        return [
            PlannerSpec(pipeline=str(item['pipeline']), planner=str(item['planner']))
            for item in self._cfg['global_planners']
        ]

    def _elapsed(self, start: float) -> float:
        return time.monotonic() - start

    def _reject(self, arm: Arm, reason: str) -> None:
        self.last_rejections[reason] += 1
        self.last_report.append(f'{arm.value} rejected: {reason}')

    def _log_info(self, msg: str) -> None:
        if self._log is not None:
            self._log.info(msg)

    def _log_warn(self, msg: str) -> None:
        if self._log is not None:
            self._log.warn(msg)
