"""
Place 단독 테스트.
nav 완료 후 로봇이 tray 앞에 위치했다는 가정 하에 PlaceSkill hover 실행.
hover 위치로 팔 정렬 → cartesian 하강 → 그리퍼 열기 → cartesian 상승 → carry_z 대기.

실행:
  ros2 run manipulation test_place
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import setup_zone_a, clear_all_objects
from manipulation.skill_primitives.planning_filter import PlanningFilter
from manipulation.skill_primitives.place_skill import PlaceSkill, PlaceResult

# ── 파라미터 ─────────────────────────────────────────────────────────
ARM        = Arm.RIGHT
APPROACH_H = 0.05

PLACE_X = 0.300
PLACE_Y = -0.15
PLACE_Z = 0.930   # tray 내부 (table 0.800 + tray_floor 0.010 + margin 0.060)

CARRY_Z = 1.150   # place 완료 후 올라갈 carry 높이 → navigation 대기
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
    node    = Node('test_place')
    log     = node.get_logger()
    client  = MoveItClient(node)
    gripper = GripperInterface(node)
    pfilter = PlanningFilter(client, log=log)
    place   = PlaceSkill(node, client, gripper, pfilter)

    log.info('[test_place] Scene 초기화')
    clear_all_objects(client)
    setup_zone_a(client)

    place_pose = _make_pose(PLACE_X, PLACE_Y, PLACE_Z)
    log.info(f'[test_place] place=({PLACE_X},{PLACE_Y},{PLACE_Z}) approach_h={APPROACH_H}')

    result = place.place(place_pose, arm=ARM, approach_height=APPROACH_H)
    log.info(f'[test_place] 결과: {result.value}')

    if result == PlaceResult.SUCCESS:
        carry = _make_pose(PLACE_X, PLACE_Y, CARRY_Z)
        r = client.move_to_pose(carry, arm=ARM, velocity=0.3, acceleration=0.3)
        log.info(f'[test_place] carry 상승(z={CARRY_Z}): {r.value}')
        log.info('[test_place] navigation 대기 상태')
    else:
        log.warn('[test_place] place 실패 — carry 상승 스킵')

    clear_all_objects(client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
