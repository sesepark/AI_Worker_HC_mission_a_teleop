# Talks to MoveIt2's move_group node.
# Receives a clean Pose in base_link frame and handles all motion planning and execution.
# All other files feed into this one — nothing else touches MoveIt2 directly.

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
from enum import Enum
from geometry_msgs.msg import Pose

class MoveResult(Enum):
    SUCCEEDED = "succeeded" # motion completed as planned
    FAILED = "failed"       # motion started but didn't finish
    ABORTED = "aborted"     # planning succeeded but execution was cancelled
    INVALID = "invalid"     # bad input - unreachable pose

class Arm(Enum):
    RIGHT = "right"
    LEFT = "left"

class MoveItClient:
    def __init__(self):
        rclpy.init()
        self.node = Node('moveit_client')
        self.callback_group = ReentrantCallbackGroup()

        #creates a moveit2 object
        self.moveit2_r = MoveIt2(
            node=self.node,
            joint_names=['arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7'],
            base_link_name='base_link',
            end_effector_name='end_effector_r_link',
            group_name='arm_r',
            callback_group=self.callback_group,
            use_move_group_action=True,
        )

        self.moveit2_l = MoveIt2(
            node=self.node,
            joint_names=['arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7'],
            base_link_name='base_link',
            end_effector_name='end_effector_l_link',
            group_name='arm_l',
            callback_group=self.callback_group,
            use_move_group_action=True,
        )

        # Wait for move_group action servers before accepting any motion commands
        self.node.get_logger().info('Waiting for move_group action servers...')
        while not self.moveit2_r._MoveIt2__move_action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn('Right arm move_group not yet available...')
            rclpy.spin_once(self.node, timeout_sec=0.1)
        while not self.moveit2_l._MoveIt2__move_action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn('Left arm move_group not yet available...')
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().info('move_group action servers ready')

        # Spin until joint states are available before any motion method is called
        while self.moveit2_r.joint_state is None or self.moveit2_l.joint_state is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().info('Joint states ready')

    def _arm(self, arm: Arm) -> MoveIt2:
        return self.moveit2_r if arm == Arm.RIGHT else self.moveit2_l

#MOVE Functions
    def _wait(self, moveit2) -> bool:
        while moveit2._MoveIt2__is_motion_requested or moveit2._MoveIt2__is_executing:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return moveit2.motion_suceeded

    def move_to_pose(self, pose, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        moveit2 = self._arm(arm)
        moveit2.motion_suceeded = False
        moveit2.max_velocity = velocity_scaling
        moveit2.max_acceleration = acceleration_scaling
        moveit2.move_to_pose(pose=pose)
        success = self._wait(moveit2)
        rclpy.spin_once(self.node, timeout_sec=0.2)
        return self._to_move_result(success, moveit2)

    def move_to_joints(self, joint_positions: list, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        moveit2 = self._arm(arm)
        moveit2.motion_suceeded = False
        moveit2.max_velocity = velocity_scaling
        moveit2.max_acceleration = acceleration_scaling
        moveit2.move_to_configuration(joint_positions)
        success = self._wait(moveit2)
        rclpy.spin_once(self.node, timeout_sec=0.2)
        return self._to_move_result(success, moveit2)

    def cartesian_move(self, pose, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        moveit2 = self._arm(arm)
        moveit2.motion_suceeded = False
        moveit2.pipeline_id = "pilz_industrial_motion_planner"
        moveit2.planner_id = "LIN"

        moveit2.max_velocity = velocity_scaling
        moveit2.max_acceleration = acceleration_scaling

        moveit2.move_to_pose(pose=pose)
        success = self._wait(moveit2)
        rclpy.spin_once(self.node, timeout_sec=0.2)

        #ALWAYS reset
        moveit2.pipeline_id = ""
        moveit2.planner_id = ""

        return self._to_move_result(success, moveit2)

    def move_to_home(self, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        return self.move_to_joints(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            arm=arm,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling
        )

#Monitor Functions
    def get_joint_positions(self, arm=Arm.RIGHT) -> list:
        moveit2 = self._arm(arm)
        joint_state = moveit2.joint_state

        if joint_state is None:
            self.node.get_logger().warn('Joint state not yet received')
            return []
        return list(joint_state.position)

    def get_current_pose(self, arm=Arm.RIGHT) -> Pose | None:
        moveit2 = self._arm(arm)
        future = moveit2.compute_fk_async()

        if future is None:
            self.node.get_logger().warn('Forward Kinematics computation failed')
            return None

        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        result = moveit2.get_compute_fk_result(future)

        if result is None:
            return None

        return result.pose
    
    def check_reachable(self, pose: Pose, arm: Arm = Arm.RIGHT) -> bool:
        moveit2 = self._arm(arm)
        position = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ]

        quat_xyzw = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

        future = moveit2.compute_ik_async(position, quat_xyzw)
        if future is None:
            return False
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        result = moveit2.get_compute_ik_result(future)
        return result is not None and len(result.solution.joint_state.position) > 0
    
#Others
    def _to_move_result(self, success: bool, moveit2: MoveIt2) -> MoveResult:
        if success:
            return MoveResult.SUCCEEDED

        error = moveit2.get_last_execution_error_code()        # this is the moveit method to return the raw error

        if error is None:
            return MoveResult.INVALID

        return MoveResult.FAILED


    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
