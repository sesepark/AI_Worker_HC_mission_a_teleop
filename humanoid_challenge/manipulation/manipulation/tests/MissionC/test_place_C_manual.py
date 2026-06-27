"""
Mission C place — 수동 좌표 버전.
아래 상수값을 직접 수정하여 사용.

실행:
  ros2 run manipulation test_place_c_manual
"""

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.moveit_client import Arm, MoveItClient
from manipulation.skill_primitives.place_skill import PlaceCSkill

ARM            = Arm.RIGHT
PLACE_Y_OFFSET = 0 #-0.030   # 우로 3cm

# ── 여기를 수정 ──────────────────────────────────────────────────────
PIPE_X            = 0.40   # [m]
PIPE_Y            = -0.335   # [m]
PIPE_Z            = 0.90   # [m]
GRIPPER_OPEN      = 0.5    # 0.0=완전열림, 1.0=완전닫힘
# ─────────────────────────────────────────────────────────────────────


def _make_pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def main():
    rclpy.init()
    node    = Node('test_place_c_manual')
    log     = node.get_logger()
    client  = MoveItClient(node)
    gripper = GripperInterface(node)
    place   = PlaceCSkill(node, client, gripper)

    pipe_pose = _make_pose(PIPE_X, PIPE_Y + PLACE_Y_OFFSET, PIPE_Z)
    result = place.place(pipe_pose, arm=ARM, gripper_open_amount=GRIPPER_OPEN)
    log.info(f'[place_c_manual] 결과: {result.value}')


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
