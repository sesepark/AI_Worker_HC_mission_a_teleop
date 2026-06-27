"""Dual-arm motion planning filter: workspace/IK fast-filter + planner selection."""

import time
from collections import Counter
from dataclasses import dataclass, field

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory

from manipulation.robot_interface.moveit_dual_client import (
    Arm,
    MotionResult,
    SymmetricDualArmClient,
)


NO_VALID_MOTION_CANDIDATE = "no_valid_motion_candidate"


@dataclass(frozen=True)
class PlannerSpec:
    pipeline: str
    planner: str


@dataclass
class DualMotionSelection:
    arm: Arm
    target_pose: Pose
    plan_pose: Pose
    global_pipeline: str
    global_planner: str
    score: float
    rejection_summary: dict[str, int] = field(default_factory=dict)
    planning_details: dict = field(default_factory=dict)
    elapsed_s: float = 0.0


_DEFAULT_CFG = {
    "enabled": True,
    "result_on_exhaustion": NO_VALID_MOTION_CANDIDATE,
    "plan_timeout_s": 10.0,
    "total_budget_s": 24.0,
    "workspace_bounds": {
        "x": [0.03, 0.75],
        "y": [-0.58, 0.58],
        "z": [0.45, 1.35],
    },
    "path_quality": {
        "max_joint_path_length": 5.0,
        "max_joint_step": 1.0,
    },
    # Keep the known-good raw dual path first, then fall back to the single-arm
    # style planners for comparison/diagnosis.
    "global_planners": [
        {"pipeline": "ompl", "planner": "RRTConnect"},
        {"pipeline": "pilz_industrial_motion_planner", "planner": "PTP"},
        {"pipeline": "stomp", "planner": "STOMP"},
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


def _offset_pose_z(pose: Pose, offset: float) -> Pose:
    p = _copy_pose(pose)
    p.position.z = pose.position.z + offset
    return p


class PlanningFilterDual:
    """Workspace/IK fast-filter + planner selection for one dual waypoint.

    The input pose is a LEFT-arm waypoint in base_link. Selection plans only the
    LEFT arm; the caller then mirrors the selected LEFT trajectory to RIGHT.
    """

    def __init__(
        self,
        moveit: SymmetricDualArmClient,
        config: dict | None = None,
        log=None,
    ):
        self._moveit = moveit
        self._cfg = _merged_dict(_DEFAULT_CFG, (config or {}).get("motion_selection", {}))
        self._root_cfg = config or {}
        self._log = log
        self.last_failure_reason = ""
        self.last_rejections: Counter[str] = Counter()
        self.last_report: list[str] = []

    def select_pose(
        self,
        pose: Pose,
        arm: Arm = Arm.LEFT,
        approach_height: float = 0.0,
        start_joint_state: list[float] | None = None,
    ) -> DualMotionSelection | None:
        """Select a planner for a waypoint pose.

        approach_height defaults to 0.0 because test_dual_pick's waypoints are
        already explicit path waypoints rather than pick target poses requiring
        a hover/pre-grasp offset.
        """
        self.last_failure_reason = ""
        self.last_rejections = Counter()
        self.last_report = []

        if not self._cfg.get("enabled", True):
            self._reject(arm, "selector_disabled")
            self.last_failure_reason = self._cfg["result_on_exhaustion"]
            return None

        start = time.monotonic()
        budget = float(self._cfg["total_budget_s"])

        if not self._within_workspace(pose):
            self._reject(arm, "target_workspace_bounds")
            self.last_failure_reason = self._cfg["result_on_exhaustion"]
            self._log_warn("[PlanningFilterDual] pose out of workspace")
            return None

        plan_pose = _offset_pose_z(pose, float(approach_height))
        if not self._within_workspace(plan_pose):
            self._reject(arm, "plan_pose_workspace_bounds")
            self.last_failure_reason = self._cfg["result_on_exhaustion"]
            self._log_warn("[PlanningFilterDual] plan pose out of workspace")
            return None

        if self._elapsed(start) > budget:
            self._reject(arm, "time_budget_exceeded")
            self.last_failure_reason = self._cfg["result_on_exhaustion"]
            return None

        if not self._moveit.check_reachable(
            pose,
            arm=arm,
            start_joint_state=start_joint_state,
            timeout_sec=min(float(self._cfg["plan_timeout_s"]), 5.0),
        ):
            self._reject(arm, "target_ik_failed")
            self.last_failure_reason = self._cfg["result_on_exhaustion"]
            self._log_warn("[PlanningFilterDual] target IK failed")
            return None

        if approach_height and not self._moveit.check_reachable(
            plan_pose,
            arm=arm,
            start_joint_state=start_joint_state,
            timeout_sec=min(float(self._cfg["plan_timeout_s"]), 5.0),
        ):
            self._reject(arm, "plan_pose_ik_failed")
            self.last_failure_reason = self._cfg["result_on_exhaustion"]
            self._log_warn("[PlanningFilterDual] plan pose IK failed")
            return None

        for planner in self._planner_specs():
            if self._elapsed(start) > budget:
                self._reject(arm, "time_budget_exceeded")
                break

            details = self._moveit.plan_to_pose_details(
                plan_pose,
                arm=arm,
                pipeline=planner.pipeline,
                planner=planner.planner,
                timeout_sec=float(self._cfg["plan_timeout_s"]),
                start_joint_state=start_joint_state,
            )
            if details.get("result") != MotionResult.SUCCEEDED:
                self._reject(arm, f"plan_failed:{planner.planner}")
                continue
            if not self._path_quality_ok(details, planner):
                continue

            selection = DualMotionSelection(
                arm=arm,
                target_pose=_copy_pose(pose),
                plan_pose=_copy_pose(plan_pose),
                global_pipeline=planner.pipeline,
                global_planner=planner.planner,
                score=round(self._score(details), 6),
                planning_details=dict(details),
                elapsed_s=round(self._elapsed(start), 3),
            )
            self._log_info(
                f"[PlanningFilterDual] selected arm={arm.value} "
                f"planner={planner.pipeline}/{planner.planner} "
                f"score={selection.score:.3f} elapsed={selection.elapsed_s:.3f}s"
            )
            return selection

        self.last_failure_reason = self._cfg["result_on_exhaustion"]
        self._log_warn(f"[PlanningFilterDual] no valid planner: {dict(self.last_rejections)}")
        return None

    def plan_left_waypoint_then_mirror(
        self,
        pose: Pose,
        *,
        name: str = "waypoint",
        velocity: float = 0.05,
        acceleration: float = 0.05,
        segment_duration_sec: float | None = None,
        timeout_sec: float = 30.0,
        skip_current_pose_check: bool = False,
    ) -> tuple[JointTrajectory, JointTrajectory, list[float]]:
        """Filter/select a planner, plan LEFT waypoint, then mirror to RIGHT."""
        selection = self.select_pose(pose, arm=Arm.LEFT)
        if selection is None:
            raise RuntimeError(
                f"{name}: planning filter failed: "
                f"{self.last_failure_reason}, "
                f"rejections={dict(self.last_rejections)}"
            )

        left_traj, right_traj, left_waypoint_joints = self._moveit.plan_left_waypoints_then_mirror(
            [selection.plan_pose],
            velocity=velocity,
            acceleration=acceleration,
            segment_duration_sec=segment_duration_sec,
            timeout_sec=timeout_sec,
            pipeline=selection.global_pipeline,
            planner=selection.global_planner,
            use_task_selector=False,
            skip_current_pose_check=skip_current_pose_check,
        )

        if not left_waypoint_joints:
            raise RuntimeError(f"{name}: planner returned no left waypoint joints")

        self._log_info(
            f"[PlanningFilterDual] {name} final joints="
            f"{[round(v, 4) for v in left_waypoint_joints[-1]]}"
        )
        return left_traj, right_traj, left_waypoint_joints[-1]

    def _within_workspace(self, pose: Pose) -> bool:
        bounds = self._cfg["workspace_bounds"]
        p = pose.position
        return (
            float(bounds["x"][0]) <= p.x <= float(bounds["x"][1])
            and float(bounds["y"][0]) <= p.y <= float(bounds["y"][1])
            and float(bounds["z"][0]) <= p.z <= float(bounds["z"][1])
        )

    def _path_quality_ok(self, details: dict, planner: PlannerSpec) -> bool:
        quality = self._cfg["path_quality"]
        max_path = float(quality["max_joint_path_length"])
        max_step = float(quality["max_joint_step"])
        joint_path = float(details.get("joint_path_length", 0.0) or 0.0)
        max_joint_step = float(details.get("max_joint_step", 0.0) or 0.0)
        if joint_path > max_path or max_joint_step > max_step:
            self._log_info(
                f"[PlanningFilterDual] {planner.planner} quality rejected: "
                f"path_length={joint_path:.2f} (max {max_path}), "
                f"max_step={max_joint_step:.2f} (max {max_step})"
            )
            self._reject(Arm.LEFT, f"quality_rejected:{planner.planner}")
            return False
        return True

    def _score(self, details: dict) -> float:
        return (
            1.0 * float(details.get("joint_path_length", 0.0) or 0.0)
            + 2.0 * float(details.get("max_joint_step", 0.0) or 0.0)
            + 0.05 * float(details.get("elapsed_s", 0.0) or 0.0)
        )

    def _planner_specs(self) -> list[PlannerSpec]:
        return [
            PlannerSpec(pipeline=str(item["pipeline"]), planner=str(item["planner"]))
            for item in self._cfg["global_planners"]
        ]

    def _elapsed(self, start: float) -> float:
        return time.monotonic() - start

    def _reject(self, arm: Arm, reason: str) -> None:
        self.last_rejections[reason] += 1
        self.last_report.append(f"{arm.value} rejected: {reason}")

    def _log_info(self, msg: str) -> None:
        if self._log is not None:
            self._log.info(msg)

    def _log_warn(self, msg: str) -> None:
        if self._log is not None:
            self._log.warn(msg)
