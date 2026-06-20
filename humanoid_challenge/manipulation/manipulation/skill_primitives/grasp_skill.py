# skill_primitives/grasp_skill.py
#
# TODO: wrap this into a ROS2 Action Server (systems team requirement).
# The action server should expose:
#   Goal     : side ('left'|'right'), object_name (str), stable_duration (float)
#   Result   : GraspResult (SUCCESS | FAILURE | TIMEOUT)
#   Feedback : grasp phase (closing, assessing, stable, failed)
#
# Exception handling to implement:
#   - Retry grasp on first failure
#   - Detect slip/drop during transport via GraspAssessment.assess()
#   - Safe reposition and return to init pose on unrecoverable failure
#
# NOTE: TIMEOUT is currently not distinguished from FAILURE — assess_stable()
# returns False for both. Distinction will be implemented in the Action Server.

from enum import Enum
from typing import Optional

from manipulation.robot_interface.gripper_controller import (
    GripperInterface,
)
from manipulation.skill_primitives.grasp_assessment import (
    GraspAssessment,
)

DEFAULT_OBJECT_NAME = 'ETC'


class GraspResult(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'
    TIMEOUT = 'timeout'


class GraspSkill:
    """
    Closes the gripper and assesses whether an object was successfully grasped.
    Re-opens automatically on failure.

    This class is the placeholder for the future GraspActionServer.
    """

    def __init__(self, node, gripper: GripperInterface, assessment: GraspAssessment):
        self._node       = node
        self._log        = node.get_logger()
        self._gripper    = gripper
        self._assessment = assessment

    def grasp(
        self,
        side: str,
        object_name: Optional[str] = None,
        stable_duration: float = 1.0,
    ) -> GraspResult:
        """
        Close the gripper and assess grasp stability.

        Parameters
        ----------
        side            : 'left' | 'right'
        object_name     : object key in object_lut.json. Defaults to 'ETC'.
        stable_duration : seconds both position + effort must hold to confirm grasp.

        Returns
        -------
        GraspResult : SUCCESS if grasp is stable, FAILURE otherwise.
                      Gripper is re-opened automatically on FAILURE.
        """
        side = side.lower()

        if object_name is None:
            object_name = DEFAULT_OBJECT_NAME

        self._log.info(
            f'[GraspSkill] [{side}] grasping object={object_name!r} '
            f'stable_duration={stable_duration}s'
        )

        self._gripper.close(side)
        self._gripper.wait_until_executed()  # wait for command to be dispatched
        self._gripper.wait_motion()          # wait for physical gripper to close

        success = self._assessment.assess_stable(
            side,
            object_name,
            duration=stable_duration,
        )

        if success:
            self._log.info(f'[GraspSkill] [{side}] grasp SUCCEEDED')
            return GraspResult.SUCCESS

        self._log.warn(f'[GraspSkill] [{side}] grasp FAILED — re-opening gripper')
        self._gripper.open(side)
        return GraspResult.FAILURE
