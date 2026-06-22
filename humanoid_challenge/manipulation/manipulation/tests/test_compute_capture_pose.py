"""
Capture pose의 z 위치를 조정한 IK 결과를 출력.

현재 CAPTURE pose: x=0.320, y=-0.250, z=1.020, identity quat
Z_DELTA 를 바꿔서 원하는 높이에서의 joint 값을 구한다.

실행:
  ros2 run manipulation test_compute_capture_pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm

ARM = Arm.RIGHT

# 현재 capture pose 기준 좌표
CAPTURE_X = 0.320
CAPTURE_Y = -0.250
CAPTURE_Z = 1.020   # 기준 z

# 올리고 싶은 z 오프셋 (미터). 양수 = 위로.
Z_DELTA = 0.10      # ← 여기만 바꾸면 됨


def _make_pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def main():
    rclpy.init()
    node   = Node('test_compute_capture_pose')
    log    = node.get_logger()
    client = MoveItClient(node)

    target_z = CAPTURE_Z + Z_DELTA
    pose = _make_pose(CAPTURE_X, CAPTURE_Y, target_z)
    log.info(f'IK 요청: x={CAPTURE_X}, y={CAPTURE_Y}, z={target_z:.3f} (delta={Z_DELTA:+.3f}m)')

    joints = client.solve_ik(pose, arm=ARM)

    if joints is None:
        log.error('IK 실패 — 해당 위치에 도달 불가')
    else:
        formatted = [f'{v:.6f}' for v in joints]
        log.info(f'CAPTURE_JOINTS (z={target_z:.3f}):')
        log.info(f'  {formatted}')
        log.info('--- 아래 값을 test_move_to_capture_pose.py 의 CAPTURE_JOINTS 에 붙여넣기 ---')
        log.info(f'CAPTURE_JOINTS = [{", ".join(formatted)}]')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
