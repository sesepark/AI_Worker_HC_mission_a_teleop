# skill_primitives/pick_skill.py
#
# Executes a single pick sequence given a pre-filtered grasp pose.
# Does NOT own retry logic or perception re-query — those belong in the action server.
#
# Contract:
#   SUCCESS → arm at hover pose, object in gripper
#   FAILURE → arm at hover pose, gripper open, bin as undisturbed as possible

from enum import Enum

from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm, MoveResult
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.skill_primitives.grasp_skill import GraspSkill, GraspResult
from manipulation.skill_primitives.planning_filter import PlanningFilter


_APPROACH_HEIGHT = 0.05
_LIFT_HOME       = 0.0


class PickResult(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'


class PickSkill:
    """Single pick sequence: open → hover → approach → grasp → retreat.

    Two modes:
      'hover' : global move to hover, Cartesian descent/retreat via Pilz LIN.
      'lift'  : global move to hover via IK, lift joint descends to grasp.

    PlanningFilter selects the planner for the global move (current → hover).
    Planning retry (PTP → STOMP fallback) is handled inside PlanningFilter.
    """

    def __init__(
        self,
        node,
        moveit: MoveItClient,
        gripper: GripperInterface,
        grasp_skill: GraspSkill,
        planning_filter: PlanningFilter,
    ):
        self._node    = node
        self._log     = node.get_logger()
        self._moveit  = moveit
        self._gripper = gripper
        self._grasp   = grasp_skill
        self._filter  = planning_filter

    def pick(
        self,
        grasp_pose: Pose,
        arm: Arm = Arm.RIGHT,
        object_name: str = 'ETC',
        approach_height: float = _APPROACH_HEIGHT,
        lift_home: float = _LIFT_HOME,
        local_mode: str = 'hover',
    ) -> PickResult:
        side = arm.value
        self._log.info(f'[PickSkill] [{side}] starting pick — object={object_name!r} mode={local_mode!r}')

        selection = self._filter.select_pose(grasp_pose, arm=arm, approach_height=approach_height)
        if selection is None:
            self._log.error(
                f'[PickSkill] [{side}] planning_filter failed: {self._filter.last_failure_reason}'
            )
            return PickResult.FAILURE

        if local_mode == 'lift':
            return self._pick_lift(
                grasp_pose, arm, object_name, approach_height, lift_home,
                selection.global_pipeline, selection.global_planner,
            )

        return self._pick_hover(
            grasp_pose, arm, object_name, approach_height,
            selection.global_pipeline, selection.global_planner,
        )

    def _pick_hover(
        self,
        grasp_pose: Pose,
        arm: Arm,
        object_name: str,
        approach_height: float,
        global_pipeline: str,
        global_planner: str,
    ) -> PickResult:
        """Global move to hover, Cartesian descent, grasp, Cartesian retreat."""
        side = arm.value

        hover = Pose()
        hover.position.x  = grasp_pose.position.x
        hover.position.y  = grasp_pose.position.y
        hover.position.z  = grasp_pose.position.z + approach_height
        hover.orientation = grasp_pose.orientation

        # executed = self._gripper.close(side)
        executed = self._gripper.open_to(side, 0.5)
        self._gripper.wait_until_executed()
        self._gripper.wait_motion()

        pos = self._gripper._command.get_position(side)
        self._log.info(f'[PickSkill] [{side}] gripper open_to(0.5): executed={executed}, pos={pos}')

        result = self._moveit.move_to_pose(
            hover, arm=arm,
            pipeline=global_pipeline,
            planner=global_planner,
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
            self._log.error(f'[PickSkill] [{side}] grasp failed — retracting')
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
        global_pipeline: str,
        global_planner: str,
    ) -> PickResult:
        """Global move to hover via IK, lift joint descends to grasp."""
        side = arm.value

        hover = Pose()
        hover.position.x  = grasp_pose.position.x
        hover.position.y  = grasp_pose.position.y
        hover.position.z  = grasp_pose.position.z + approach_height
        hover.orientation = grasp_pose.orientation

        self._gripper.open_to(side, 0.5)
        self._gripper.wait_until_executed()
        self._gripper.wait_motion()
        self._moveit.move_lift(lift_home)

        result = self._moveit.move_to_pose(
            hover, arm=arm,
            pipeline=global_pipeline,
            planner=global_planner,
        )
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[PickSkill] [{side}] hover move failed')
            return PickResult.FAILURE

        self._moveit.move_lift(lift_home - approach_height)

        grasp_result = self._grasp.grasp(side, object_name=object_name)
        if grasp_result != GraspResult.SUCCESS:
            self._log.error(f'[PickSkill] [{side}] grasp failed — ascending')
            self._moveit.move_lift(lift_home)
            return PickResult.FAILURE

        self._moveit.move_lift(lift_home)
        self._log.info(f'[PickSkill] [{side}] pick SUCCEEDED (lift)')
        return PickResult.SUCCESS
