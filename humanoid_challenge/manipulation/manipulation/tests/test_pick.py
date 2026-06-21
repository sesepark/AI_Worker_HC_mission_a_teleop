"""
Pick 단독 테스트.
Zone A collision object 등록 후, 하드코딩된 grasp pose로 pick 실행.
pick 완료 후 carry 높이(박스 위)로 상승해 멈춤 → navigation 대기 상태.

실행:
  ros2 run manipulation test_pick
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import (
    setup_zone_a, clear_all_objects,
)
from manipulation.skill_primitives.grasp_assessment import GraspAssessment
from manipulation.skill_primitives.grasp_skill import GraspSkill
from manipulation.skill_primitives.pick_skill import PickSkill

# ── 파라미터 ─────────────────────────────────────────────────────────
ARM        = Arm.RIGHT
LOCAL_MODE = 'hover'

# yellow_box 내부 grasp 좌표 (테이블 앞 엣지 x=0.050 기준)
# yellow_box: center_x=0.320, center_y=-0.295
#
# 시뮬 검증 완료 좌표 (모두 SUCCEEDED):
#   중앙:       GRASP_X=0.320, GRASP_Y=-0.250  ← 기본값
#   앞쪽(로봇): GRASP_X=0.250, GRASP_Y=-0.250
#   깊숙이:     GRASP_X=0.400, GRASP_Y=-0.250
#   옆쪽(-y):   GRASP_X=0.320, GRASP_Y=-0.350
GRASP_X =  0.320   # 박스 x 중앙
GRASP_Y = -0.250   # 박스 y 중앙 근처
GRASP_Z =  0.830   # 테이블(0.800) + 물체 여유

APPROACH_H = 0.05  # grasp 전 hover offset (m)

# pick 완료 후 올라갈 carry 높이
# 테이블(0.800) + 박스(0.200) + 여유(0.020) = 1.020
CARRY_Z    = 1.020

GRASP_QX, GRASP_QY, GRASP_QZ, GRASP_QW = 0.0, 0.0, 0.0, 1.0
# ─────────────────────────────────────────────────────────────────────


def _make_pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = GRASP_QX
    pose.orientation.y = GRASP_QY
    pose.orientation.z = GRASP_QZ
    pose.orientation.w = GRASP_QW
    return pose


def main():
    rclpy.init()
    node = Node('test_pick')
    log  = node.get_logger()

    client  = MoveItClient(node)
    gripper = GripperInterface(node)
    assess  = GraspAssessment(node)
    grasp   = GraspSkill(node, gripper, assess)
    pick    = PickSkill(node, client, gripper, grasp)

    log.info('[test_pick] Scene 초기화')
    clear_all_objects(client)
    setup_zone_a(client)

    pose = _make_pose(GRASP_X, GRASP_Y, GRASP_Z)

    log.info(f'[test_pick] pick 시작 — mode={LOCAL_MODE}  grasp=({GRASP_X},{GRASP_Y},{GRASP_Z})')
    result = pick.pick(
        grasp_pose=pose,
        arm=ARM,
        local_mode=LOCAL_MODE,
        approach_height=APPROACH_H,
    )
    log.info(f'[test_pick] pick 결과: {result.value}')

    if result.value == 'success':
        # 박스 위로 carry 높이까지 상승 → navigation 대기
        carry = _make_pose(GRASP_X, GRASP_Y, CARRY_Z)
        r = client.move_to_pose(carry, arm=ARM, velocity=0.3, acceleration=0.3)
        log.info(f'[test_pick] carry 상승(z={CARRY_Z}): {r.value}')
        log.info('[test_pick] navigation 대기 상태 — 팔이 carry 위치에서 멈춤')
    else:
        log.warn('[test_pick] pick 실패 — carry 상승 스킵')

    clear_all_objects(client)
    log.info('[test_pick] planning scene 정리 완료')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
