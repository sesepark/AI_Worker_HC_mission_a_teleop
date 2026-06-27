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

# inset=0.300, x=0.270, y=-0.250, z=1.020, identity quat 기준 IK 결과
# TODO: 실물 로봇 텔레옵으로 top-down 방향 확인 후 갱신
CAPTURE_JOINTS = [-2.707296, -0.299926, 2.883739, -2.112988, -1.433787, 0.369488, 0.659124]

#[-2.707296, -0.299926, 2.883739, -2.112988, -1.433787, 0.369488, 0.659124]
#[-2.577387, -0.293513, 2.888004, -2.106759, -1.435530, 0.362080, 0.533107]
#[-2.904767, -0.426636, 3.043781, -2.647364, -1.441257, 0.418577, 0.306154]
# [-1.135108, -1.142945, 1.292197, -2.484786, 0.444408, 1.302167, 1.820100]
#[0.570138, -2.243647, -0.439813, -2.061163, -3.109835, 0.737009, -0.950150]
#[0.569822, -2.243116, -0.440434, -2.060918, -3.110732, 0.737690, -0.951402]
#[-0.687697, -1.065868, 1.047603, -2.396016, 0.701864, 1.187090, 1.820100]
#[-0.845728, -1.145170, 1.013616, -1.996687, -2.846256, 0.801760, -1.580400]
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
