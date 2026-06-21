"""
Place 단독 테스트.
nav 완료 후 로봇이 tray 앞에 위치했다는 가정 하에 PlaceSkill hover 실행.
hover 위치로 팔 정렬 → cartesian 하강 → 그리퍼 열기 → cartesian 상승.

nav 없이 테스트 시: pick 좌표 근처에서 동작 (같은 박스 위에서 내려놓는 형태).
nav 적용 시: place 호출 전 로봇이 이미 tray 앞으로 이동한 상태.

실행:
  ros2 run manipulation test_place
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import (
    setup_zone_a, clear_all_objects,
)
from manipulation.skill_primitives.place_skill import PlaceSkill

# ── 파라미터 ─────────────────────────────────────────────────────────
ARM        = Arm.RIGHT
APPROACH_H = 0.05   # hover offset = place_z + 0.05

# nav 완료 후 로봇 기준 tray 중앙 좌표
# nav 없이 테스트 시 pick 위치(x=0.270, y=-0.250) 근처에서 동작
# nav 적용 시 로봇이 tray 앞으로 이동했으므로 y ≈ 0.0으로 조정
PLACE_X = 0.270   # tray center_x (로봇 기준, nav 후 pick x와 유사)
PLACE_Y = -0.250  # nav 없는 테스트용 — nav 후에는 0.0 근처
PLACE_Z = 0.870   # tray 내부 (table 0.800 + tray_floor 0.010 + margin 0.060)
# ─────────────────────────────────────────────────────────────────────


def main():
    rclpy.init()
    node    = Node('test_place')
    log     = node.get_logger()
    client  = MoveItClient(node)
    gripper = GripperInterface(node)
    place   = PlaceSkill(node, client, gripper)

    log.info('[test_place] Scene 초기화')
    clear_all_objects(client)
    setup_zone_a(client)

    pose = Pose()
    pose.position.x = PLACE_X
    pose.position.y = PLACE_Y
    pose.position.z = PLACE_Z
    pose.orientation.w = 1.0

    log.info(f'[test_place] place=({PLACE_X},{PLACE_Y},{PLACE_Z})  approach_h={APPROACH_H}')
    result = place.place(
        place_pose=pose,
        arm=ARM,
        local_mode='hover',
        approach_height=APPROACH_H,
    )
    log.info(f'[test_place] 결과: {result.value}')

    clear_all_objects(client)
    log.info('[test_place] planning scene 정리 완료')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
