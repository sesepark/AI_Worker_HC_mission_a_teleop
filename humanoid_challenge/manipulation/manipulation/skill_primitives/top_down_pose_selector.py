"""Top-down pose selector for vertical grasp/place approach.

Perception publishes a single PoseStamped (base_link frame) per object.
Use select_grasp_from_center() for pick, select_place() for place.
"""

from __future__ import annotations

import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Iterable

from geometry_msgs.msg import Pose, PoseArray
from scipy.spatial.transform import Rotation

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
    local_mode: str
    score: float
    candidate_index: int
    rejection_summary: dict[str, int] = field(default_factory=dict)
    planning_details: dict = field(default_factory=dict)
    elapsed_s: float = 0.0


@dataclass
class _Candidate:
    arm: Arm
    target_pose: Pose
    pre_pose: Pose
    candidate_index: int


_DEFAULT_SELECTION_CFG = {
    'enabled': True,
    'result_on_exhaustion': NO_VALID_MOTION_CANDIDATE,
    'max_candidates': 8,
    'plan_top_n': 4,
    'normal_time_budget_s': 5.0,
    'hard_time_budget_s': 15.0,
    'plan_timeout_s': 5.0,
    'workspace_bounds': {
        'x': [0.03, 0.5],
        'y': [-0.58, 0.55],
        'z': [0.50, 1.30],
    },
    'orientation_filter': {
        'enabled': True,
        'min_pitch_deg': 70.0,
        'max_abs_roll_deg': 30.0,
    },
    'path_quality': {
        'max_joint_path_length': 3.5,
        'max_joint_step': 1.0,
    },
    'scoring': {
        'joint_path_weight': 1.0,
        'max_joint_step_weight': 2.0,
        'planning_time_weight': 0.05,
    },
    'global_planners': [
        {'pipeline': 'pilz_industrial_motion_planner', 'planner': 'PTP'},
        {'pipeline': 'stomp', 'planner': 'STOMP'},
        {'pipeline': 'ompl', 'planner': 'RRTstar'},
        {'pipeline': 'ompl', 'planner': 'LBKPIECE'},
    ],
    'local_modes': {
        'preferred': 'hover',
        'fallback': 'lift',
    },
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


def copy_pose(pose: Pose) -> Pose:
    copied = Pose()
    copied.position.x = pose.position.x
    copied.position.y = pose.position.y
    copied.position.z = pose.position.z
    copied.orientation.x = pose.orientation.x
    copied.orientation.y = pose.orientation.y
    copied.orientation.z = pose.orientation.z
    copied.orientation.w = pose.orientation.w
    return copied


def _hover_pose(pose: Pose, offset: float) -> Pose:
    """Return a pose directly above `pose` by `offset` metres (top-down only)."""
    h = copy_pose(pose)
    h.position.z = pose.position.z + offset
    return h


class TopDownPoseSelector:
    """Select valid task poses with top-down (vertical) grasp approach.

    All candidates are normalized to a pure vertical orientation before
    IK and planning checks.
    """

    def __init__(
        self,
        moveit: MoveItClient,
        config: dict | None = None,
        log=None,
    ):
        self._moveit = moveit
        self._cfg = _merged_dict(_DEFAULT_SELECTION_CFG, (config or {}).get('motion_selection', {}))
        self._root_cfg = config or {}
        self._log = log
        self.last_failure_reason = ''
        self.last_rejections: Counter[str] = Counter()
        self.last_report: list[str] = []

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def select_grasp_from_center(
        self,
        center_pose: Pose,
        fixed_arm: Arm | None = None,
        grasp_y_offset: float = -0.045,
    ) -> MotionSelection | None:
        """Select a grasp from a single center pose published by perception.

        Parameters
        ----------
        center_pose    : Object center pose in base_link frame.
                         orientation 은 center_pose 그대로 사용 (기본 identity quat).
        fixed_arm      : If specified, only try this arm; otherwise try both.
        grasp_y_offset : Y offset from object center to actual grasp point (metres).
                         Nuts are not grasped at center — offset ~3 cm to the side.
        """
        self.last_failure_reason = ''
        self.last_rejections = Counter()
        self.last_report = []

        if not self._cfg.get('enabled', True):
            self._reject(-1, fixed_arm, 'selector_disabled')
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            return None

        start = time.monotonic()
        budget = float(self._cfg['normal_time_budget_s'])

        grasp_pose = copy_pose(center_pose)
        grasp_pose.position.y += grasp_y_offset

        if not self._within_workspace(grasp_pose):
            self._reject(0, fixed_arm, 'target_workspace_bounds')
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            self._log_warn('[TopDownPoseSelector] grasp pose out of workspace')
            return None

        pre_grasp = _hover_pose(grasp_pose, offset=self._pre_offset())
        if not self._within_workspace(pre_grasp):
            self._reject(0, fixed_arm, 'prepose_workspace_bounds')
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            self._log_warn('[TopDownPoseSelector] hover pose out of workspace')
            return None

        arms = [fixed_arm] if fixed_arm is not None else self._arm_order(center_pose)
        for arm in arms:
            if self._elapsed(start) > budget:
                self._reject(0, arm, 'selection_time_budget_exceeded')
                break

            if not self._moveit.check_reachable(grasp_pose, arm=arm):
                self._reject(0, arm, 'target_ik_failed')
                continue
            if not self._moveit.check_reachable(pre_grasp, arm=arm):
                self._reject(0, arm, 'prepose_ik_failed')
                continue

            for planner in self._planner_specs():
                details = self._moveit.plan_to_pose_details(
                    pre_grasp,
                    arm=arm,
                    pipeline=planner.pipeline,
                    planner=planner.planner,
                    timeout=float(self._cfg['plan_timeout_s']),
                )
                if details.get('result') != MoveResult.SUCCEEDED:
                    self._reject(0, arm, f'plan_failed:{planner.planner}')
                    continue
                if not self._path_quality_ok_simple(details, planner):
                    continue

                score = self._score_simple(details)
                selection = MotionSelection(
                    arm=arm,
                    target_pose=copy_pose(grasp_pose),
                    pre_pose=copy_pose(pre_grasp),
                    global_pipeline=planner.pipeline,
                    global_planner=planner.planner,
                    local_mode=self._local_mode(),
                    score=round(score, 6),
                    candidate_index=0,
                    planning_details=dict(details),
                    elapsed_s=round(self._elapsed(start), 3),
                )
                self._log_info(
                    f'[TopDownPoseSelector] selected arm={arm.value} '
                    f'planner={planner.pipeline}/{planner.planner} '
                    f'score={selection.score:.3f} elapsed={selection.elapsed_s:.3f}s'
                )
                return selection

        self.last_failure_reason = self._cfg['result_on_exhaustion']
        self._log_warn(f'[TopDownPoseSelector] no valid planner found: {dict(self.last_rejections)}')
        return None

    def select_place(self, place_pose: Pose, arm: Arm) -> MotionSelection | None:
        return self._select([place_pose], place_pose=None, fixed_arm=arm)

    def select_pick(
        self,
        candidates: PoseArray | Iterable[Pose],
        place_pose: Pose | None = None,
    ) -> MotionSelection | None:
        """Legacy multi-candidate selection. Prefer select_grasp_from_center()."""
        poses = self._candidate_list(candidates)
        return self._select(poses, place_pose=place_pose, fixed_arm=None)

    # ------------------------------------------------------------------
    # Selection stages
    # ------------------------------------------------------------------

    def _select(
        self,
        poses: list[Pose],
        place_pose: Pose | None,
        fixed_arm: Arm | None,
    ) -> MotionSelection | None:
        self.last_failure_reason = ''
        self.last_rejections = Counter()
        self.last_report = []

        if not self._cfg.get('enabled', True):
            self._reject(-1, fixed_arm, 'selector_disabled')
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            return None

        start = time.monotonic()
        budget = (
            float(self._cfg['hard_time_budget_s'])
            if place_pose is not None
            else float(self._cfg['normal_time_budget_s'])
        )

        fast_candidates: list[_Candidate] = []
        max_candidates = int(self._cfg['max_candidates'])

        for index, pose in enumerate(poses[:max_candidates]):
            if self._elapsed(start) > budget:
                self._reject(index, fixed_arm, 'selection_time_budget_exceeded')
                break
            fast_candidates.extend(
                self._fast_filter_candidate(index, pose, place_pose, fixed_arm)
            )

        if not fast_candidates:
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            self._log_warn(f'[TopDownPoseSelector] no fast-filter candidate: {dict(self.last_rejections)}')
            return None

        fast_candidates.sort(key=lambda c: c.candidate_index)
        planned_candidates = fast_candidates[: int(self._cfg['plan_top_n'])]

        best: MotionSelection | None = None
        for candidate in planned_candidates:
            if self._elapsed(start) > budget:
                self._reject(candidate.candidate_index, candidate.arm, 'selection_time_budget_exceeded')
                break
            selection = self._score_candidate(candidate, start)
            if selection is None:
                continue
            if best is None or selection.score < best.score:
                best = selection

        if best is None:
            self.last_failure_reason = self._cfg['result_on_exhaustion']
            self._log_warn(f'[TopDownPoseSelector] no valid planned candidate: {dict(self.last_rejections)}')
            return None

        best.rejection_summary = dict(self.last_rejections)
        best.elapsed_s = round(self._elapsed(start), 3)
        self._log_info(
            f'[TopDownPoseSelector] selected candidate={best.candidate_index} '
            f'arm={best.arm.value} planner={best.global_pipeline}/{best.global_planner} '
            f'score={best.score:.3f} elapsed={best.elapsed_s:.3f}s'
        )
        return best

    def _fast_filter_candidate(
        self,
        index: int,
        pose: Pose,
        place_pose: Pose | None,
        fixed_arm: Arm | None,
    ) -> list[_Candidate]:
        if not self._within_workspace(pose):
            self._reject(index, fixed_arm, 'target_workspace_bounds')
            return []
        if not self._topdown_orientation_ok(pose):
            self._reject(index, fixed_arm, 'target_not_topdown')
            return []

        pre = _hover_pose(pose, offset=self._pre_offset())
        if not self._within_workspace(pre):
            self._reject(index, fixed_arm, 'prepose_workspace_bounds')
            return []

        arms = [fixed_arm] if fixed_arm is not None else self._arm_order(pose)
        accepted: list[_Candidate] = []
        for arm in arms:
            if not self._moveit.check_reachable(pose, arm=arm):
                self._reject(index, arm, 'target_ik_failed')
                continue
            if not self._moveit.check_reachable(pre, arm=arm):
                self._reject(index, arm, 'prepose_ik_failed')
                continue
            if place_pose is not None and not self._place_fast_ok(index, place_pose, arm):
                continue

            accepted.append(
                _Candidate(
                    arm=arm,
                    target_pose=copy_pose(pose),
                    pre_pose=pre,
                    candidate_index=index,
                )
            )
            self._log_info(f'[TopDownPoseSelector] candidate {index} {arm.value} passed fast filters')
        return accepted

    def _place_fast_ok(self, index: int, place_pose: Pose, arm: Arm) -> bool:
        if not self._within_workspace(place_pose):
            self._reject(index, arm, 'place_workspace_bounds')
            return False
        if not self._topdown_orientation_ok(place_pose):
            self._reject(index, arm, 'place_not_topdown')
            return False
        pre_place = _hover_pose(place_pose, offset=self._pre_offset())
        if not self._within_workspace(pre_place):
            self._reject(index, arm, 'preplace_workspace_bounds')
            return False
        if not self._moveit.check_reachable(place_pose, arm=arm):
            self._reject(index, arm, 'place_ik_failed')
            return False
        if not self._moveit.check_reachable(pre_place, arm=arm):
            self._reject(index, arm, 'preplace_ik_failed')
            return False
        return True

    def _score_candidate(self, candidate: _Candidate, start: float) -> MotionSelection | None:
        best: MotionSelection | None = None
        for planner in self._planner_specs():
            details = self._moveit.plan_to_pose_details(
                candidate.pre_pose,
                arm=candidate.arm,
                pipeline=planner.pipeline,
                planner=planner.planner,
                timeout=float(self._cfg['plan_timeout_s']),
            )
            if details.get('result') != MoveResult.SUCCEEDED:
                self._reject(candidate.candidate_index, candidate.arm, f'plan_failed:{planner.planner}')
                continue
            if not self._path_quality_ok(candidate, details, planner):
                continue

            score = self._score(candidate, details)
            selection = MotionSelection(
                arm=candidate.arm,
                target_pose=copy_pose(candidate.target_pose),
                pre_pose=copy_pose(candidate.pre_pose),
                global_pipeline=planner.pipeline,
                global_planner=planner.planner,
                local_mode=self._local_mode(),
                score=round(score, 6),
                candidate_index=candidate.candidate_index,
                planning_details=dict(details),
                elapsed_s=round(self._elapsed(start), 3),
            )
            self._log_info(
                f'[TopDownPoseSelector] candidate {candidate.candidate_index} '
                f'{candidate.arm.value} accepted {planner.pipeline}/{planner.planner} '
                f'score={selection.score:.3f}'
            )
            if best is None or selection.score < best.score:
                best = selection
        return best

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

    def _topdown_orientation_ok(self, pose: Pose) -> bool:
        filt = self._cfg['orientation_filter']
        if not filt.get('enabled', True):
            return True
        q = pose.orientation
        try:
            roll, pitch, _yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)
        except ValueError:
            return False
        return abs(roll) <= float(filt['max_abs_roll_deg']) and pitch >= float(filt['min_pitch_deg'])

    def _path_quality_ok(
        self,
        candidate: _Candidate,
        details: dict,
        planner: PlannerSpec,
    ) -> bool:
        quality = self._cfg['path_quality']
        max_path = float(quality['max_joint_path_length'])
        max_step = float(quality['max_joint_step'])
        if float(details.get('joint_path_length', 0.0) or 0.0) > max_path:
            self._reject(candidate.candidate_index, candidate.arm, f'path_too_long:{planner.planner}')
            return False
        if float(details.get('max_joint_step', 0.0) or 0.0) > max_step:
            self._reject(candidate.candidate_index, candidate.arm, f'joint_step_too_large:{planner.planner}')
            return False
        return True

    def _path_quality_ok_simple(self, details: dict, planner: PlannerSpec) -> bool:
        quality = self._cfg['path_quality']
        max_path = float(quality['max_joint_path_length'])
        max_step = float(quality['max_joint_step'])
        joint_path = float(details.get('joint_path_length', 0.0) or 0.0)
        max_joint_step = float(details.get('max_joint_step', 0.0) or 0.0)
        if joint_path > max_path or max_joint_step > max_step:
            self._log_info(
                f'[TopDownPoseSelector] {planner.planner} path quality: '
                f'path_length={joint_path:.2f} (max {max_path}), '
                f'max_step={max_joint_step:.2f} (max {max_step})'
            )
            return False
        return True

    def _score(self, candidate: _Candidate, details: dict) -> float:
        weights = self._cfg['scoring']
        return (
            float(weights['joint_path_weight']) * float(details.get('joint_path_length', 0.0) or 0.0)
            + float(weights['max_joint_step_weight']) * float(details.get('max_joint_step', 0.0) or 0.0)
            + float(weights['planning_time_weight']) * float(details.get('elapsed_s', 0.0) or 0.0)
        )

    def _score_simple(self, details: dict) -> float:
        weights = self._cfg['scoring']
        return (
            float(weights['joint_path_weight']) * float(details.get('joint_path_length', 0.0) or 0.0)
            + float(weights['max_joint_step_weight']) * float(details.get('max_joint_step', 0.0) or 0.0)
            + float(weights['planning_time_weight']) * float(details.get('elapsed_s', 0.0) or 0.0)
        )

    # ------------------------------------------------------------------
    # Normalization and helpers
    # ------------------------------------------------------------------

    def _normalize_to_topdown(self, pose: Pose, yaw_deg: float = 0.0) -> Pose:
        """Normalize pose orientation to pure top-down vertical approach.

        pitch=90° (EE x-axis pointing down), roll=0°, yaw=yaw_deg.
        Position is unchanged.
        """
        normalized = copy_pose(pose)
        rot = Rotation.from_euler('xyz', [0.0, 90.0, yaw_deg], degrees=True)
        quat = rot.as_quat()
        normalized.orientation.x = float(quat[0])
        normalized.orientation.y = float(quat[1])
        normalized.orientation.z = float(quat[2])
        normalized.orientation.w = float(quat[3])
        return normalized

    # ------------------------------------------------------------------
    # Small helpers
    # ------------------------------------------------------------------

    def _candidate_list(self, candidates: PoseArray | Iterable[Pose]) -> list[Pose]:
        if isinstance(candidates, PoseArray):
            return list(candidates.poses)
        if hasattr(candidates, 'poses'):
            return list(candidates.poses)
        return list(candidates)

    def _arm_order(self, pose: Pose) -> tuple[Arm, Arm]:
        y_threshold = float(self._root_cfg.get('y_threshold', 0.0))
        primary = Arm.LEFT if pose.position.y >= y_threshold else Arm.RIGHT
        secondary = Arm.RIGHT if primary == Arm.LEFT else Arm.LEFT
        return primary, secondary

    def _planner_specs(self) -> list[PlannerSpec]:
        return [
            PlannerSpec(pipeline=str(item['pipeline']), planner=str(item['planner']))
            for item in self._cfg['global_planners']
        ]

    def _pre_offset(self) -> float:
        return float(self._root_cfg.get('pre_grasp_offset', 0.15))

    def _local_mode(self) -> str:
        return str(self._cfg['local_modes'].get('preferred', 'hover'))

    def _elapsed(self, start: float) -> float:
        return time.monotonic() - start

    def _reject(self, index: int, arm: Arm | None, reason: str) -> None:
        self.last_rejections[reason] += 1
        arm_label = arm.value if arm is not None else '<any>'
        self.last_report.append(f'candidate {index} {arm_label} rejected: {reason}')

    def _log_info(self, msg: str) -> None:
        if self._log is not None:
            self._log.info(msg)

    def _log_warn(self, msg: str) -> None:
        if self._log is not None:
            self._log.warn(msg)
