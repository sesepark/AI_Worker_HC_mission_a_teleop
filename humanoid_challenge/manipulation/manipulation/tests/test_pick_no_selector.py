"""
Pick 단독 테스트 (selector 없음).
orientation quaternion을 직접 수정해 grasp 각도 실험용.

실행:
  ros2 run manipulation test_pick_no_selector
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

GRASP_X =  0.45381910861259245 - 0.045
GRASP_Y = -0.3390095419490655
GRASP_Z =  0.830   # 테이블(0.800) + 물체 여유

APPROACH_H = 0.1  # grasp 전 hover offset (m)
CARRY_Z    = 1.100

# orientation quaternion (identity = 0,0,0,1)
GRASP_QX = 0.0
GRASP_QY = 0.0
GRASP_QZ = 0.707
GRASP_QW = 0.707
# ─────────────────────────────────────────────────────────────────────


def _make_pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x    = x
    pose.position.y    = y
    pose.position.z    = z
    pose.orientation.x = GRASP_QX
    pose.orientation.y = GRASP_QY
    pose.orientation.z = GRASP_QZ
    pose.orientation.w = GRASP_QW
    return pose


def main():
    rclpy.init()
    node = Node('test_pick_no_selector')
    log  = node.get_logger()

    client  = MoveItClient(node)
    gripper = GripperInterface(node)
    assess  = GraspAssessment(node)
    grasp   = GraspSkill(node, gripper, assess)
    pick    = PickSkill(node, client, gripper, grasp)

    log.info('[test_pick_no_selector] Scene 초기화')
    clear_all_objects(client)
    setup_zone_a(client)

    pose = _make_pose(GRASP_X, GRASP_Y, GRASP_Z)

    log.info(f'[test_pick_no_selector] pick 시작 — mode={LOCAL_MODE}  grasp=({GRASP_X},{GRASP_Y},{GRASP_Z})  quat=({GRASP_QX},{GRASP_QY},{GRASP_QZ},{GRASP_QW})')
    result = pick.pick(
        grasp_pose=pose,
        arm=ARM,
        local_mode=LOCAL_MODE,
        approach_height=APPROACH_H,
    )
    log.info(f'[test_pick_no_selector] pick 결과: {result.value}')

    if result.value == 'success':
        carry = _make_pose(GRASP_X, GRASP_Y, CARRY_Z)
        r = client.move_to_pose(carry, arm=ARM, velocity=0.3, acceleration=0.3)
        log.info(f'[test_pick_no_selector] carry 상승(z={CARRY_Z}): {r.value}')
        log.info('[test_pick_no_selector] navigation 대기 상태 — 팔이 carry 위치에서 멈춤')
    else:
        log.warn('[test_pick_no_selector] pick 실패 — carry 상승 스킵')

    clear_all_objects(client)
    log.info('[test_pick_no_selector] planning scene 정리 완료')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
