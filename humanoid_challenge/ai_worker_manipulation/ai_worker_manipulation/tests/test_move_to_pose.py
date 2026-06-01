#!/usr/bin/env python3
"""
Phase 0 test — move right arm to hardcoded poses and confirm execution.

Usage (inside container, after Gazebo + MoveIt are running):
  ros2 run ai_worker_manipulation test_move_to_pose
"""

import rclpy
from geometry_msgs.msg import Pose
from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient

# (x, y, z, qx, qy, qz, qw) in base_link frame
# Poses chosen to be safely in the right arm's reachable workspace
TARGET_POSES = [
    (0.40, -0.20, 0.90, 0.0, 0.0, 0.0, 1.0),
    (0.35, -0.30, 0.85, 0.0, 0.0, 0.0, 1.0),
]


def make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
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
    client = MoveItClient()
    log = client.node.get_logger()

    log.info('Moving to home position')
    client.move_to_home()

    for i, args in enumerate(TARGET_POSES):
        x, y, z = args[:3]
        pose = make_pose(*args)
        log.info(f'[{i+1}/{len(TARGET_POSES)}] Moving to ({x}, {y}, {z})')
        success = client.move_to_pose(pose)
        log.info(f'Result: {"SUCCESS" if success else "FAILED"}')

    client.shutdown()


if __name__ == '__main__':
    main()
