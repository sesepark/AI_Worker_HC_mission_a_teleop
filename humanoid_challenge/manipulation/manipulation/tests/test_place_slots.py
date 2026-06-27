"""
Place 슬롯 순환 테스트.
blue tray 내 5개 좌표(중앙 + 4코너)를 --ros-args -p slot:=N 으로 선택해서 실행.

실행:
  ros2 run manipulation test_place_slots                         # slot 0 (중앙)
  ros2 run manipulation test_place_slots --ros-args -p slot:=1  # 앞-우
  ros2 run manipulation test_place_slots --ros-args -p slot:=2  # 앞-좌
  ros2 run manipulation test_place_slots --ros-args -p slot:=3  # 뒤-우
  ros2 run manipulation test_place_slots --ros-args -p slot:=4  # 뒤-좌
"""

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import setup_zone_a, clear_all_objects
from manipulation.skill_primitives.planning_filter import PlanningFilter
from manipulation.skill_primitives.place_skill import PlaceSkill, PlaceResult

# ── 중앙 좌표 (내일 실측 후 이 값만 수정) ────────────────────────────
CENTER_X = 0.370
CENTER_Y = -0.130
CENTER_Z = 1.000

# 중앙 기준 오프셋 (보수적: tray 내측 여유 확보)
DX = 0.012   # x 오프셋 (앞/뒤)
DY = 0.008   # y 오프셋 (우/좌)

PLACE_SLOTS = [
    (CENTER_X,      CENTER_Y,      CENTER_Z),   # 0: 중앙
    (CENTER_X - DX, CENTER_Y - DY, CENTER_Z),   # 1: 앞-우
    (CENTER_X - DX, CENTER_Y + DY, CENTER_Z),   # 2: 앞-좌
    (CENTER_X + DX, CENTER_Y - DY, CENTER_Z),   # 3: 뒤-우
    (CENTER_X + DX, CENTER_Y + DY, CENTER_Z),   # 4: 뒤-좌
]

SLOT_LABELS = ['중앙', '앞-우', '앞-좌', '뒤-우', '뒤-좌']

ARM        = Arm.RIGHT
APPROACH_H = 0.05
CARRY_Z    = 1.100
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
    node = Node('test_place_slots')
    log  = node.get_logger()

    node.declare_parameter('slot', 0)
    slot = node.get_parameter('slot').get_parameter_value().integer_value

    if slot < 0 or slot >= len(PLACE_SLOTS):
        log.error(f'[test_place_slots] slot={slot} 범위 초과 (0~{len(PLACE_SLOTS)-1})')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    x, y, z = PLACE_SLOTS[slot]
    label    = SLOT_LABELS[slot]
    log.info(f'[test_place_slots] slot={slot} ({label}) → ({x}, {y}, {z})')

    client  = MoveItClient(node)
    gripper = GripperInterface(node)
    pfilter = PlanningFilter(client, log=log)
    place   = PlaceSkill(node, client, gripper, pfilter)

    log.info('[test_place_slots] Scene 초기화')
    clear_all_objects(client)
    setup_zone_a(client)

    place_pose = _make_pose(x, y, z)
    result = place.place(place_pose, arm=ARM, approach_height=APPROACH_H)
    log.info(f'[test_place_slots] 결과: {result.value}')

    if result == PlaceResult.SUCCESS:
        carry = _make_pose(x, y, CARRY_Z)
        r = client.move_to_pose(carry, arm=ARM, velocity=0.3, acceleration=0.3)
        log.info(f'[test_place_slots] carry 상승(z={CARRY_Z}): {r.value}')
    else:
        log.warn('[test_place_slots] place 실패 — carry 상승 스킵')

    clear_all_objects(client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
