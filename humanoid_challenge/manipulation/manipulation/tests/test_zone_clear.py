"""
Planning Scene 전체 초기화 테스트.
MoveIt Planning Scene의 모든 collision object를 제거한다.

실행:
  ros2 run manipulation test_zone_clear
"""

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient
from manipulation.robot_interface.planning_scene import clear_all_objects


def main():
    rclpy.init()
    node = Node('test_zone_clear')
    log = node.get_logger()

    log.info('[test_zone_clear] 시작')
    client = MoveItClient(node)

    log.info('[test_zone_clear] 모든 collision objects 제거 중...')
    clear_all_objects(client)
    log.info('[test_zone_clear] 완료')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
