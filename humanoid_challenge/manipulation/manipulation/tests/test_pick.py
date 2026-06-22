"""
Pick 단독 테스트.
Zone A collision object 등록 후, 하드코딩 중앙좌표 → adapter → pick 실행.
pick 완료 후 carry 높이로 상승 → navigation 대기.

실행:
  ros2 run manipulation test_pick
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import setup_zone_a, clear_all_objects
from manipulation.skill_primitives.grasp_assessment import GraspAssessment
from manipulation.skill_primitives.grasp_skill import GraspSkill
from manipulation.skill_primitives.pick_skill import PickSkill, PickResult
from manipulation.skill_primitives.planning_filter import PlanningFilter
from manipulation.skill_primitives.mission_a_grasp_adapter import build_mission_a_grasp_pose

# ── 파라미터 ──────────────────────────────────────────────────────────
ARM = Arm.RIGHT

# 퍼셉션에서 수동으로 받은 물체 중앙 좌표 (base_link 기준)
CENTER_X =  0.42
CENTER_Y = -0.29
CENTER_Z =  0.0   # adapter가 z를 GRASP_Z(0.83)으로 덮어씌움

CARRY_Z = 1.150   # pick 완료 후 올라갈 carry 높이
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
    node = Node('test_pick')
    log  = node.get_logger()

    client  = MoveItClient(node)
    gripper = GripperInterface(node)
    assess  = GraspAssessment(node)
    grasp   = GraspSkill(node, gripper, assess)
    pfilter = PlanningFilter(client, log=log)
    pick    = PickSkill(node, client, gripper, grasp, pfilter)

    log.info('[test_pick] Scene 초기화')
    clear_all_objects(client)
    setup_zone_a(client)

    center     = _make_pose(CENTER_X, CENTER_Y, CENTER_Z)
    grasp_pose = build_mission_a_grasp_pose(center)
    p = grasp_pose.position
    log.info(f'[test_pick] grasp pose=({p.x:.3f},{p.y:.3f},{p.z:.3f})')

    result = pick.pick(grasp_pose, arm=ARM)
    log.info(f'[test_pick] pick 결과: {result.value}')

    if result == PickResult.SUCCESS:
        carry = _make_pose(p.x, p.y, CARRY_Z)
        r = client.move_to_pose(carry, arm=ARM, velocity=0.3, acceleration=0.3)
        log.info(f'[test_pick] carry 상승(z={CARRY_Z}): {r.value}')
        log.info('[test_pick] navigation 대기 상태')
    else:
        log.warn('[test_pick] pick 실패 — carry 상승 스킵')

    clear_all_objects(client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
