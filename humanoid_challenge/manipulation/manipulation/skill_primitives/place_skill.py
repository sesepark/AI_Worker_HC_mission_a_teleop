# skill_primitives/place_skill.py
#
# Executes a single place sequence given a target place pose.
# Does NOT own navigation or retry logic — those belong in the action server.
#
# Three modes:
#   'hover' : arm IK to hover above place pose → Cartesian descent → open → retreat
#   'lift'  : arm IK to hover → lift joint descent to place → open → lift retreat
#   'wheel' : robot body already positioned by wheels; arm IK to hover →
#             lift joint descends to release_height (closer than approach_height) →
#             open → lift retreat

from enum import Enum

from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import (
    MoveItClient,
    Arm,
    MoveResult,
)
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.skill_primitives.pick_skill import (
    _move_with_retry,
    _APPROACH_HEIGHT,
    _LIFT_HOME,
    _PLANNING_RETRIES,
    _JITTER_RETRIES,
    _JITTER_STD,
)

_RELEASE_HEIGHT = 0.05  # lift descent for wheel mode — closer to tray than approach_height


class PlaceResult(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'


class PlaceSkill:

    def __init__(self, node, moveit: MoveItClient, gripper: GripperInterface):
        self._node    = node
        self._log     = node.get_logger()
        self._moveit  = moveit
        self._gripper = gripper

    def place(
        self,
        place_pose: Pose,
        arm: Arm = Arm.RIGHT,
        approach_height: float = _APPROACH_HEIGHT,
        lift_home: float = _LIFT_HOME,
        release_height: float = _RELEASE_HEIGHT,
        planning_retries: int = _PLANNING_RETRIES,
        jitter_retries: int = _JITTER_RETRIES,
        jitter_std: float = _JITTER_STD,
        global_pipeline: str = 'ompl',
        global_planner: str = 'RRTConnect',
        local_mode: str = 'hover',
    ) -> PlaceResult:
        side = arm.value
        self._log.info(f'[PlaceSkill] [{side}] starting place — mode={local_mode!r}')

        if local_mode == 'lift':
            return self._place_lift(
                place_pose, arm, approach_height, lift_home,
                planning_retries, jitter_retries, jitter_std,
                global_pipeline, global_planner,
            )

        if local_mode == 'wheel':
            return self._place_wheel(
                place_pose, arm, approach_height, lift_home, release_height,
                planning_retries, jitter_retries, jitter_std,
                global_pipeline, global_planner,
            )

        return self._place_hover(
            place_pose, arm, approach_height,
            planning_retries, jitter_retries, jitter_std,
            global_pipeline, global_planner,
        )

    def _place_hover(
        self,
        place_pose: Pose,
        arm: Arm,
        approach_height: float,
        planning_retries: int,
        jitter_retries: int,
        jitter_std: float,
        global_pipeline: str = 'ompl',
        global_planner: str = 'RRTConnect',
    ) -> PlaceResult:
        """Arm IK to hover above place, Cartesian descent, open gripper, Cartesian retreat."""
        side = arm.value
        self._log.info(f'[PlaceSkill] [{side}] hover mode')

        hover = Pose()
        hover.position.x = place_pose.position.x
        hover.position.y = place_pose.position.y
        hover.position.z = place_pose.position.z + approach_height
        hover.orientation = place_pose.orientation

        result = _move_with_retry(
            lambda p, _arm=arm: self._moveit.move_to_pose(
                p, arm=_arm, pipeline=global_pipeline, planner=global_planner,
            ),
            hover, self._log, f'PlaceSkill/{side}/hover',
            same_retries=planning_retries,
            jitter_retries=jitter_retries,
            jitter_std=jitter_std,
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
        planning_retries: int,
        jitter_retries: int,
        jitter_std: float,
        global_pipeline: str = 'ompl',
        global_planner: str = 'RRTConnect',
    ) -> PlaceResult:
        """Arm IK to hover, lift joint descends to place, open gripper, lift retreat."""
        side = arm.value
        self._log.info(f'[PlaceSkill] [{side}] lift mode')

        hover = Pose()
        hover.position.x  = place_pose.position.x
        hover.position.y  = place_pose.position.y
        hover.position.z  = place_pose.position.z + approach_height
        hover.orientation = place_pose.orientation

        self._moveit.move_lift(lift_home)

        result = _move_with_retry(
            lambda p, _arm=arm: self._moveit.move_to_pose(
                p, arm=_arm, pipeline=global_pipeline, planner=global_planner,
            ),
            hover, self._log, f'PlaceSkill/{side}/hover',
            same_retries=planning_retries,
            jitter_retries=jitter_retries,
            jitter_std=jitter_std,
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
        planning_retries: int,
        jitter_retries: int,
        jitter_std: float,
        global_pipeline: str = 'ompl',
        global_planner: str = 'RRTConnect',
    ) -> PlaceResult:
        """Robot body already at tray via wheels. Arm IK to hover, lift descends to
        release_height (closer to tray than approach_height), open gripper, lift retreat."""
        side = arm.value
        self._log.info(f'[PlaceSkill] [{side}] wheel mode')

        hover = Pose()
        hover.position.x  = place_pose.position.x
        hover.position.y  = place_pose.position.y
        hover.position.z  = place_pose.position.z + approach_height
        hover.orientation = place_pose.orientation

        self._moveit.move_lift(lift_home)

        result = _move_with_retry(
            lambda p, _arm=arm: self._moveit.move_to_pose(
                p, arm=_arm, pipeline=global_pipeline, planner=global_planner,
            ),
            hover, self._log, f'PlaceSkill/{side}/hover',
            same_retries=planning_retries,
            jitter_retries=jitter_retries,
            jitter_std=jitter_std,
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
