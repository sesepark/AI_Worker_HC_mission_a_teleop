"""
여러 grasp 후보 좌표에 대해 IK 가능 여부를 일괄 확인.

실행:
  ros2 run manipulation test_compute_capture_pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm

ARM = Arm.RIGHT

# 테스트할 (x, y, z, qx, qy, qz, qw) 목록
_SQ2 = 0.7071067811865476
TEST_POSES = [
    # 최근 실패 좌표들 (yaw0 standard)
    (0.442, -0.051, 0.830,  0.0, 0.0, 0.0, 1.0, 'fail_yaw0_a'),
    (0.469, -0.186, 0.830,  0.0, 0.0, 0.0, 1.0, 'fail_yaw0_b'),
    (0.416, -0.328, 0.830,  0.0, 0.0, 0.0, 1.0, 'test_pick_yaw0'),
    # hover 높이
    (0.442, -0.051, 0.930,  0.0, 0.0, 0.0, 1.0, 'fail_yaw0_a_hover'),
    (0.416, -0.328, 0.930,  0.0, 0.0, 0.0, 1.0, 'test_pick_hover'),
    # yaw90으로 시도
    (0.442, -0.051, 0.830,  0.0, 0.0, _SQ2, _SQ2, 'fail_yaw90_a'),
    (0.416, -0.328, 0.830,  0.0, 0.0, _SQ2, _SQ2, 'test_pick_yaw90'),
    # 성공했던 좌표 (기준점)
    (0.213, -0.042, 0.830,  0.0, 0.0, 0.0, 1.0, 'success_ref'),
]


def _make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def main():
    rclpy.init()
    node   = Node('test_compute_capture_pose')
    log    = node.get_logger()
    client = MoveItClient(node)

    log.info('=== IK 가능 여부 일괄 테스트 ===')
    for entry in TEST_POSES:
        x, y, z, qx, qy, qz, qw, label = entry
        pose = _make_pose(x, y, z, qx, qy, qz, qw)
        reachable = client.check_reachable(pose, arm=ARM)
        status = 'OK ✓' if reachable else 'FAIL ✗'
        log.info(f'[{status}] {label:25s} ({x:.3f},{y:.3f},{z:.3f}) qz={qz:.4f}')
        import time; time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
