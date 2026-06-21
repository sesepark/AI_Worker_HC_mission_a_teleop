"""
지정 pose로 이동 테스트.
좌표와 자세를 직접 수정해서 팔이 원하는 위치로 가는지 확인.

실행:
  ros2 run manipulation test_move_to_pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm

# ── 파라미터 ──────────────────────────────────────────────────────
ARM      = Arm.RIGHT
PIPELINE = 'ompl'
PLANNER  = 'RRTConnect'

# 목표 위치 (base_link 기준, 단위: m)
TARGET_X =  0.400
TARGET_Y = -0.200
TARGET_Z =  0.900

# top-down 수직 자세 quaternion (pitch=90°, roll=0°, yaw=0°)
TARGET_QX, TARGET_QY, TARGET_QZ, TARGET_QW = 0.0, 0.7071, 0.0, 0.7071
# ─────────────────────────────────────────────────────────────────


def main():
    rclpy.init()
    node = Node('test_move_to_pose')
    log  = node.get_logger()

    client = MoveItClient(node)

    # 목표 pose 구성
    pose = Pose()
    pose.position.x    = TARGET_X
    pose.position.y    = TARGET_Y
    pose.position.z    = TARGET_Z
    pose.orientation.x = TARGET_QX
    pose.orientation.y = TARGET_QY
    pose.orientation.z = TARGET_QZ
    pose.orientation.w = TARGET_QW

    log.info(
        f'[test_move_to_pose] 이동 시작 — '
        f'arm={ARM.value} pos=({TARGET_X}, {TARGET_Y}, {TARGET_Z})'
    )
    result = client.move_to_pose(pose, arm=ARM, pipeline=PIPELINE, planner=PLANNER)
    log.info(f'[test_move_to_pose] 결과: {result.value}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
