import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)


class GripperCommand:

    # If controller topic changes,
    # change here.
    # Gripper joints live inside the arm controllers (see ffw_sg2_follower_ai_hardware_controller.yaml).
    # The follower_ai launch remaps /arm_*/controller/joint_trajectory to the leader topics below.
    # Previous topics that were remapped away (silent drop):
    #   'left':  '/arm_l_controller/joint_trajectory'
    #   'right': '/arm_r_controller/joint_trajectory'
    # Original incorrect topics that do not exist on the robot:
    #   'left':  '/gripper_l_controller/joint_trajectory'
    #   'right': '/gripper_r_controller/joint_trajectory'
    # TODO: for a launch-independent solution, switch to the FollowJointTrajectory action client:
    #   'left':  '/arm_l_controller/follow_joint_trajectory'
    #   'right': '/arm_r_controller/follow_joint_trajectory'
    CONTROLLER_TOPICS = {
        'left':  '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
        'right': '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
    }

    # If joint names change,
    # change here.

    ARM_JOINTS = {
        'left': [
            'arm_l_joint1',
            'arm_l_joint2',
            'arm_l_joint3',
            'arm_l_joint4',
            'arm_l_joint5',
            'arm_l_joint6',
            'arm_l_joint7',
        ],
        'right': [
            'arm_r_joint1',
            'arm_r_joint2',
            'arm_r_joint3',
            'arm_r_joint4',
            'arm_r_joint5',
            'arm_r_joint6',
            'arm_r_joint7',
        ],
    }

    GRIPPER_JOINTS = {
        'left': 'gripper_l_joint1',
        'right': 'gripper_r_joint1',
    }

    VALID_SIDES = ('left', 'right', 'both')

    # Trajectory execution time.
    # Increase if motion is unstable.
    MOTION_TIME = 1.0
    POSITION_MIN = 0.0
    POSITION_MAX = 1.0

    def __init__(self, node, callback_group=None):

        self._node = node
        self._log  = node.get_logger()

        cb = callback_group or ReentrantCallbackGroup()

        self._joint_positions = {}

        self._node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
            callback_group=cb,
        )

        self._pubs = {
            'left': self._node.create_publisher(
                JointTrajectory,
                self.CONTROLLER_TOPICS['left'],
                10,
            ),
            'right': self._node.create_publisher(
                JointTrajectory,
                self.CONTROLLER_TOPICS['right'],
                10,
            ),
        }

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _joint_state_cb(self, msg):

        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = pos

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _validate_side(self, func_name: str, side: str) -> bool:
        """Returns True if side is valid, logs error and returns False if not."""
        if side not in self.VALID_SIDES:
            self._log.error(
                f'[GripperCommand.{func_name}] invalid side {side!r} — '
                f'expected one of {self.VALID_SIDES}'
            )
            return False
        return True

    def _validate_position(self, func_name: str, position: float) -> bool:
        """
        Checks that position is in the normalized 0.0–1.0 range.
        Logs a warning (not error) if out of range — the value is clamped
        upstream by GripperInterface, but this catches unexpected direct calls.
        """
        if not (self.POSITION_MIN <= position <= self.POSITION_MAX):
            self._log.warn(
                f'[GripperCommand.{func_name}] position {position:.4f} outside '
                f'[{self.POSITION_MIN}, {self.POSITION_MAX}] — '
                f'publishing as-is; verify driver expects normalized 0.0–1.0'
            )
            return False
        return True

    # def _send_single(self, side, position):
    #     """
    #     Publish a JointTrajectory to one side's controller.
    #     Arm joints are held at their current positions; only the gripper
    #     joint is commanded to `position`.
    #     """

    #     # Guard: side must be 'left' or 'right' (not 'both') here.
    #     if side not in ('left', 'right'):
    #         self._log.error(
    #             f'[GripperCommand._send_single] invalid side {side!r} — '
    #             f'only "left" or "right" accepted here'
    #         )
    #         return False

    #     msg = JointTrajectory()

    #     point = JointTrajectoryPoint()

    #     point.time_from_start = Duration(
    #         seconds=self.MOTION_TIME
    #     ).to_msg()

    #     arm_positions = []

    #     # Keep current arm pose fixed.
    #     # Log a warning for any joint whose state has not yet been received.
    #     for joint in self.ARM_JOINTS[side]:

    #         pos = self._joint_positions.get(joint)

    #         if pos is None:
    #             self._log.warn(
    #                 f'[GripperCommand._send_single] joint state for {joint!r} '
    #                 f'not yet received — defaulting to 0.0'
    #             )
    #             pos = 0.0

    #         arm_positions.append(float(pos))

    #     msg.joint_names = (
    #         self.ARM_JOINTS[side]
    #         + [self.GRIPPER_JOINTS[side]]
    #     )

    #     point.positions = arm_positions + [position]

    #     msg.points = [point]

    #     self._pubs[side].publish(msg)
    #     return True
    def _send_single(self, side, position):
        """
        Publish a JointTrajectory to one side's controller.
        Arm joints are held at their current positions; only the gripper
        joint is commanded to `position`.
        """

        # Guard: side must be 'left' or 'right' (not 'both') here.
        if side not in ('left', 'right'):
            self._log.error(
                f'[GripperCommand._send_single] invalid side {side!r} — '
                f'only "left" or "right" accepted here'
            )
            return False

        msg = JointTrajectory()

        point = JointTrajectoryPoint()

        point.time_from_start = Duration(
            seconds=self.MOTION_TIME
        ).to_msg()

        # Keep current arm pose fixed.
        # Log a warning for any joint whose state has not yet been received.
        msg.joint_names = [self.GRIPPER_JOINTS[side]]
        point.positions = [position]

        msg.points = [point]

        self._pubs[side].publish(msg)
        return True

    # ── Public API ────────────────────────────────────────────────────────────

    def send(self, side, position) -> bool:

        side = side.lower()

        if not self._validate_side('send', side):
            return False

        self._validate_position('send', position)

        # 'both' → publish to both controllers independently.
        if side == 'both':
            left_ok  = self._send_single('left', position)
            right_ok = self._send_single('right', position)
            return left_ok and right_ok

        return self._send_single(side, position)

    def get_position(self, side):
        """
        Returns the current normalized gripper position (0.0–1.0) for the
        given side, or None if /joint_states has not been received yet.

        For side='both', returns the average of left and right positions,
        or None if either side has not been received.
        """

        side = side.lower()

        if not self._validate_side('get_position', side):
            return None

        if side == 'both':

            left  = self._joint_positions.get(self.GRIPPER_JOINTS['left'])
            right = self._joint_positions.get(self.GRIPPER_JOINTS['right'])

            if left is None or right is None:
                self._log.warn(
                    f'[GripperCommand.get_position] one or both gripper joint states '
                    f'not yet received (left={left}, right={right})'
                )
                return None

            return (left + right) / 2.0

        pos = self._joint_positions.get(self.GRIPPER_JOINTS[side])

        if pos is None:
            self._log.warn(
                f'[GripperCommand.get_position] [{side}] joint state not yet received'
            )

        return pos

    def wait_motion(self):

        time.sleep(self.MOTION_TIME + 0.2)