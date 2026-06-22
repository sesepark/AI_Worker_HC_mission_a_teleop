"""
Capture pose 이동 테스트.
wrist cam이 yellow_box 내부를 향하는 대기 자세로 right arm을 이동.

joint 값은 시뮬 기준 임시값 — 실물 로봇 텔레옵으로 확인 후 갱신 필요

실행:
  ros2 run manipulation test_move_to_capture_pose
"""

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient, Arm

ARM = Arm.RIGHT

# 테이블 앞 엣지 x=0.050 기준, x=0.320, y=-0.250, z=1.020, identity quat IK 결과
# joints = [-2.454825, -1.086682, 2.237191, -1.941996, 2.730438, 0.845307, -1.570796]
# joints = [0.174502, -2.953452, -0.039620, -1.813645, -3.139435, 1.158934, -0.185651] - 이상한거
# TODO: 실물 로봇 텔레옵으로 top-down 방향 확인 후 갱신
CAPTURE_JOINTS = [-0.514537, -1.079939, 0.611448, -2.036518, -2.695534, 1.082374, -1.580207]


def main():
    rclpy.init()
    node   = Node('test_move_to_capture_pose')
    log    = node.get_logger()
    client = MoveItClient(node)

    log.info(f'[capture_pose] joints={[f"{v:.3f}" for v in CAPTURE_JOINTS]}')

    r = client.move_to_joints(CAPTURE_JOINTS, arm=ARM, velocity=0.2, acceleration=0.2)
    log.info(f'[capture_pose] 결과: {r.value}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
