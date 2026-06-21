# skill_primitives/pick_skill.py
#
# Executes a single pick sequence given a pre-filtered grasp pose.
# Does NOT own retry logic or perception re-query — those belong in the action server.
#
# Contract:
#   SUCCESS → arm at hover pose, object in gripper
#   FAILURE → arm at hover pose, gripper open, bin as undisturbed as possible

import random
from enum import Enum
from typing import Callable

from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import (
    MoveItClient,
    Arm,
    MoveResult,
)
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.skill_primitives.grasp_skill import GraspSkill, GraspResult


_APPROACH_HEIGHT   = 0.10
_LIFT_HOME         = 0.0
_PLANNING_RETRIES  = 3
_JITTER_RETRIES    = 3
_JITTER_STD        = 0.01


def _move_with_retry(
    move_fn: Callable[[Pose], MoveResult],
    pose: Pose,
    log,
    label: str,
    same_retries: int = _PLANNING_RETRIES,
    jitter_retries: int = _JITTER_RETRIES,
    jitter_std: float = _JITTER_STD,
) -> MoveResult:
    """Retry move_fn(pose) with same pose, then with gaussian-jittered pose."""
    result = MoveResult.FAILED

    for attempt in range(same_retries):
        result = move_fn(pose)
        if result == MoveResult.SUCCEEDED:
            return result
        log.warn(
            f'[{label}] same-pose attempt {attempt + 1}/{same_retries} → {result.value}'
        )

    for attempt in range(jitter_retries):
        jittered = Pose()
        jittered.position.x  = pose.position.x + random.gauss(0.0, jitter_std)
        jittered.position.y  = pose.position.y + random.gauss(0.0, jitter_std)
        jittered.position.z  = pose.position.z + random.gauss(0.0, jitter_std)
        jittered.orientation = pose.orientation
        result = move_fn(jittered)
        log.warn(
            f'[{label}] jitter attempt {attempt + 1}/{jitter_retries} → {result.value}'
        )
        if result == MoveResult.SUCCEEDED:
            return result

    return result


class PickResult(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'
    TIMEOUT = 'timeout'


class PickSkill:
    """
    Single pick sequence: open → hover → approach → grasp+assess → retreat.

    Two modes:
      'hover' : arm moves to hover pose, then Cartesian descent to grasp.
      'lift'  : arm moves to hover pose via IK, then lift joint descends to grasp.

    Grasp quality is assessed immediately after closing — before any retreat.
    If the grasp is unstable the gripper re-opens and the arm retracts cleanly,
    leaving the bin as undisturbed as possible for a retry.

    The caller (action server) is responsible for:
      - Providing a pre-filtered single grasp pose
      - Deciding whether and how to retry (including re-querying perception)
    """

    def __init__(
        self,
        node,
        moveit: MoveItClient,
        gripper: GripperInterface,
        grasp_skill: GraspSkill,
    ):
        self._node       = node
        self._log        = node.get_logger()
        self._moveit     = moveit
        self._gripper    = gripper
        self._grasp      = grasp_skill

    def pick(
        self,
        grasp_pose: Pose,
        arm: Arm = Arm.RIGHT,
        object_name: str = 'ETC',
        approach_height: float = _APPROACH_HEIGHT,
        lift_home: float = _LIFT_HOME,
        planning_retries: int = _PLANNING_RETRIES,
        jitter_retries: int = _JITTER_RETRIES,
        jitter_std: float = _JITTER_STD,
        global_pipeline: str = 'ompl',
        global_planner: str = 'RRTConnect',
        local_mode: str = 'hover',
    ) -> PickResult:
        side = arm.value
        self._log.info(f'[PickSkill] [{side}] starting pick — object={object_name!r} mode={local_mode!r}')

        if local_mode == 'lift':
            return self._pick_lift(
                grasp_pose, arm, object_name, approach_height, lift_home,
                planning_retries, jitter_retries, jitter_std,
                global_pipeline, global_planner,
            )

        return self._pick_hover(
            grasp_pose, arm, object_name, approach_height,
            planning_retries, jitter_retries, jitter_std,
            global_pipeline, global_planner,
        )

    def _pick_hover(
        self,
        grasp_pose: Pose,
        arm: Arm,
        object_name: str,
        approach_height: float,
        planning_retries: int,
        jitter_retries: int,
        jitter_std: float,
        global_pipeline: str = 'ompl',
        global_planner: str = 'RRTConnect',
    ) -> PickResult:
        """Move to hover above grasp, Cartesian descend, grasp, Cartesian retreat to hover."""
        side = arm.value
        self._log.info(f'[PickSkill] [{side}] hover mode')

        hover = Pose()
        hover.position.x = grasp_pose.position.x
        hover.position.y = grasp_pose.position.y
        hover.position.z = grasp_pose.position.z + approach_height
        hover.orientation = grasp_pose.orientation

        self._gripper.open(side)
        self._gripper.wait_until_executed()

        result = _move_with_retry(
            lambda p, _arm=arm: self._moveit.move_to_pose(
                p, arm=_arm, pipeline=global_pipeline, planner=global_planner,
            ),
            hover, self._log, f'PickSkill/{side}/hover',
            same_retries=planning_retries,
            jitter_retries=jitter_retries,
            jitter_std=jitter_std,
        )
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[PickSkill] [{side}] hover move failed')
            return PickResult.FAILURE

        result = self._moveit.move_cartesian(grasp_pose, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.warn(f'[PickSkill] [{side}] cartesian descent failed')
            self._moveit.move_to_pose(hover, arm=arm)
            return PickResult.FAILURE

        grasp_result = self._grasp.grasp(side, object_name=object_name)
        if grasp_result != GraspResult.SUCCESS:
            self._log.error(f'[PickSkill] [{side}] grasp FAILED — retracting')
            self._moveit.move_cartesian(hover, arm=arm)
            return PickResult.FAILURE

        retract = self._moveit.move_cartesian(hover, arm=arm)
        if retract != MoveResult.SUCCEEDED:
            self._moveit.move_to_pose(hover, arm=arm)

        self._log.info(f'[PickSkill] [{side}] pick SUCCEEDED (hover)')
        return PickResult.SUCCESS

    def _pick_lift(
        self,
        grasp_pose: Pose,
        arm: Arm,
        object_name: str,
        approach_height: float,
        lift_home: float,
        planning_retries: int,
        jitter_retries: int,
        jitter_std: float,
        global_pipeline: str = 'ompl',
        global_planner: str = 'RRTConnect',
    ) -> PickResult:
        """Move arm to hover pose via IK, then descend with lift joint to grasp."""
        side = arm.value
        self._log.info(f'[PickSkill] [{side}] lift mode')

        hover = Pose()
        hover.position.x  = grasp_pose.position.x
        hover.position.y  = grasp_pose.position.y
        hover.position.z  = grasp_pose.position.z + approach_height
        hover.orientation = grasp_pose.orientation

        self._gripper.open(side)
        self._gripper.wait_until_executed()
        self._moveit.move_lift(lift_home)

        result = _move_with_retry(
            lambda p, _arm=arm: self._moveit.move_to_pose(
                p, arm=_arm, pipeline=global_pipeline, planner=global_planner,
            ),
            hover, self._log, f'PickSkill/{side}/hover',
            same_retries=planning_retries,
            jitter_retries=jitter_retries,
            jitter_std=jitter_std,
        )
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[PickSkill] [{side}] hover move failed')
            return PickResult.FAILURE

        self._moveit.move_lift(lift_home - approach_height)

        grasp_result = self._grasp.grasp(side, object_name=object_name)
        if grasp_result != GraspResult.SUCCESS:
            self._log.error(f'[PickSkill] [{side}] grasp FAILED — ascending')
            self._moveit.move_lift(lift_home)
            return PickResult.FAILURE

        self._moveit.move_lift(lift_home)
        self._log.info(f'[PickSkill] [{side}] pick SUCCEEDED (lift)')
        return PickResult.SUCCESS
