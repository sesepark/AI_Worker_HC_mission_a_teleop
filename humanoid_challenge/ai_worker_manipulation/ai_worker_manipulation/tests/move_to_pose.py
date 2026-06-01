#!/usr/bin/env python3
"""
Move the right arm to a specified Cartesian pose.

Usage:
  ros2 run ai_worker_manipulation move_to_pose X Y Z [QX QY QZ QW]

  X Y Z          — position in base_link frame (metres)
  QX QY QZ QW    — orientation quaternion (optional, defaults to identity)

Examples:
  ros2 run ai_worker_manipulation move_to_pose 0.4 -0.2 0.9
  ros2 run ai_worker_manipulation move_to_pose 0.4 -0.2 0.9 0.0 0.0 0.0 1.0
"""

import sys
from geometry_msgs.msg import Pose
from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient


def main():
    args = sys.argv[1:]

    if len(args) < 3:
        print(__doc__)
        sys.exit(1)

    try:
        x, y, z = float(args[0]), float(args[1]), float(args[2])
        qx, qy, qz, qw = (float(args[3]), float(args[4]),
                           float(args[5]), float(args[6])) if len(args) >= 7 else (0.0, 0.0, 0.0, 1.0)
    except ValueError:
        print('Error: all arguments must be numbers')
        sys.exit(1)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

    client = MoveItClient()
    log = client.node.get_logger()
    log.info(f'Moving to ({x}, {y}, {z})')
    success = client.move_to_pose(pose)
    log.info(f'Result: {"SUCCESS" if success else "FAILED"}')
    client.shutdown()


if __name__ == '__main__':
    main()
