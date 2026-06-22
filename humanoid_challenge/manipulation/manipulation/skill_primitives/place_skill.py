# skill_primitives/place_skill.py
#
# Executes a single place sequence given a target place pose.
# Does NOT own navigation or retry logic — those belong in the action server.
#
# Three modes:
#   'hover' : arm to hover above place pose → Cartesian descent → open → retreat
#   'lift'  : arm to hover → lift joint descent → open → lift retreat
#   'wheel' : robot body at tray via wheels; arm to hover →
#             lift joint descends to release_height → open → lift retreat

from enum import Enum

from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm, MoveResult
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.skill_primitives.planning_filter import PlanningFilter


_APPROACH_HEIGHT = 0.10
_LIFT_HOME       = 0.0
_RELEASE_HEIGHT  = 0.05


class PlaceResult(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'


class PlaceSkill:

    def __init__(
        self,
        node,
        moveit: MoveItClient,
        gripper: GripperInterface,
        planning_filter: PlanningFilter,
    ):
        self._node    = node
        self._log     = node.get_logger()
        self._moveit  = moveit
        self._gripper = gripper
        self._filter  = planning_filter

    def place(
        self,
        place_pose: Pose,
        arm: Arm = Arm.RIGHT,
        approach_height: float = _APPROACH_HEIGHT,
        lift_home: float = _LIFT_HOME,
        release_height: float = _RELEASE_HEIGHT,
        local_mode: str = 'hover',
    ) -> PlaceResult:
        side = arm.value
        self._log.info(f'[PlaceSkill] [{side}] starting place — mode={local_mode!r}')

        selection = self._filter.select_pose(place_pose, arm=arm, approach_height=approach_height)
        if selection is None:
            self._log.error(
                f'[PlaceSkill] [{side}] planning_filter failed: {self._filter.last_failure_reason}'
            )
            return PlaceResult.FAILURE

        if local_mode == 'lift':
            return self._place_lift(
                place_pose, arm, approach_height, lift_home,
                selection.global_pipeline, selection.global_planner,
            )

        if local_mode == 'wheel':
            return self._place_wheel(
                place_pose, arm, approach_height, lift_home, release_height,
                selection.global_pipeline, selection.global_planner,
            )

        return self._place_hover(
            place_pose, arm, approach_height,
            selection.global_pipeline, selection.global_planner,
        )

    def _place_hover(
        self,
        place_pose: Pose,
        arm: Arm,
        approach_height: float,
        global_pipeline: str,
        global_planner: str,
    ) -> PlaceResult:
        side = arm.value

        hover = Pose()
        hover.position.x  = place_pose.position.x
        hover.position.y  = place_pose.position.y
        hover.position.z  = place_pose.position.z + approach_height
        hover.orientation = place_pose.orientation

        result = self._moveit.move_to_pose(
            hover, arm=arm, pipeline=global_pipeline, planner=global_planner,
        )
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[PlaceSkill] [{side}] hover move failed')
            return PlaceResult.FAILURE

        result = self._moveit.move_cartesian(place_pose, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.warn(f'[PlaceSkill] [{side}] cartesian descent failed')
            self._moveit.move_to_pose(hover, arm=arm)
            return PlaceResult.FAILURE

        self._gripper.open(side)
        self._gripper.wait_until_executed()
        self._gripper.wait_motion()

        retract = self._moveit.move_cartesian(hover, arm=arm)
        if retract != MoveResult.SUCCEEDED:
            self._moveit.move_to_pose(hover, arm=arm)

        self._log.info(f'[PlaceSkill] [{side}] place SUCCEEDED (hover)')
        return PlaceResult.SUCCESS

    def _place_lift(
        self,
        place_pose: Pose,
        arm: Arm,
        approach_height: float,
        lift_home: float,
        global_pipeline: str,
        global_planner: str,
    ) -> PlaceResult:
        side = arm.value

        hover = Pose()
        hover.position.x  = place_pose.position.x
        hover.position.y  = place_pose.position.y
        hover.position.z  = place_pose.position.z + approach_height
        hover.orientation = place_pose.orientation

        self._moveit.move_lift(lift_home)

        result = self._moveit.move_to_pose(
            hover, arm=arm, pipeline=global_pipeline, planner=global_planner,
        )
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[PlaceSkill] [{side}] hover move failed')
            return PlaceResult.FAILURE

        self._moveit.move_lift(lift_home - approach_height)

        self._gripper.open(side)
        self._gripper.wait_until_executed()
        self._gripper.wait_motion()

        self._moveit.move_lift(lift_home)
        self._log.info(f'[PlaceSkill] [{side}] place SUCCEEDED (lift)')
        return PlaceResult.SUCCESS

    def _place_wheel(
        self,
        place_pose: Pose,
        arm: Arm,
        approach_height: float,
        lift_home: float,
        release_height: float,
        global_pipeline: str,
        global_planner: str,
    ) -> PlaceResult:
        side = arm.value

        hover = Pose()
        hover.position.x  = place_pose.position.x
        hover.position.y  = place_pose.position.y
        hover.position.z  = place_pose.position.z + approach_height
        hover.orientation = place_pose.orientation

        self._moveit.move_lift(lift_home)

        result = self._moveit.move_to_pose(
            hover, arm=arm, pipeline=global_pipeline, planner=global_planner,
        )
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[PlaceSkill] [{side}] hover move failed')
            return PlaceResult.FAILURE

        self._moveit.move_lift(lift_home - release_height)

        self._gripper.open(side)
        self._gripper.wait_until_executed()
        self._gripper.wait_motion()

        self._moveit.move_lift(lift_home)
        self._log.info(f'[PlaceSkill] [{side}] place SUCCEEDED (wheel)')
        return PlaceResult.SUCCESS
