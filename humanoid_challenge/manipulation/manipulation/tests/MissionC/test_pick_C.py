"""
Mission C 픽 단독 테스트.
파이프(peg)에 끼울 부품을 yaw-90° 자세로 집는 테스트.

오프셋 규칙:
  - orientation : (0, 0, 0.707, 0.707)  — yaw 90°, 파이프 축 방향에서 접근
  - x offset    : GRASP_X_OFFSET (-0.045 m)
  - z           : GRASP_Z (고정값)

실행:
  ros2 run manipulation test_pick_c
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import setup_zone_c_table, remove_zone_c_table
from manipulation.skill_primitives.grasp_assessment import GraspAssessment
from manipulation.skill_primitives.grasp_skill import GraspSkill
from manipulation.skill_primitives.pick_skill import PickSkill, PickResult
from manipulation.skill_primitives.planning_filter import PlanningFilter
from manipulation.skill_primitives.mission_c_arm_selector import select_arm

# ── 파라미터 ──────────────────────────────────────────────────────────
# 퍼셉션에서 받은 부품 중앙 좌표 (base_link 기준, m)
# TODO: 실물 테이블 계측 후 갱신
CENTER_X = 0.25
CENTER_Y = 0.20
CENTER_Z = 0.0   # GRASP_Z 로 덮어씌워짐

GRASP_Z       =  0.83    # 그리퍼 목표 높이 (m)
GRASP_Y_OFFSET = -0.045  # mission_a adapter와 동일한 x 오프셋
CARRY_Z        =  1.150  # pick 완료 후 carry 높이
# ─────────────────────────────────────────────────────────────────────

_QUAT_YAW90 = (0.0, 0.0, 0.0, 1.0)   # yaw 90° around Z


def build_c_grasp_pose(cx: float, cy: float) -> Pose:
    """부품 중심 좌표 → Mission C grasp pose 변환."""
    pose = Pose()
    pose.position.x = cx 
    pose.position.y = cy + GRASP_Y_OFFSET
    pose.position.z = GRASP_Z
    pose.orientation.x, pose.orientation.y, \
        pose.orientation.z, pose.orientation.w = _QUAT_YAW90
    return pose


def main():
    rclpy.init()
    node = Node('test_pick_c')
    log  = node.get_logger()

    client  = MoveItClient(node)
    gripper = GripperInterface(node)

    setup_zone_c_table(client)

    assess  = GraspAssessment(node)
    grasp   = GraspSkill(node, gripper, assess)
    pfilter = PlanningFilter(client, log=log)
    pick    = PickSkill(node, client, gripper, grasp, pfilter)

    arm = select_arm(CENTER_Y)
    log.info(f'[test_pick_c] 선택된 팔: {arm.value} (CENTER_Y={CENTER_Y})')

    grasp_pose = build_c_grasp_pose(CENTER_X, CENTER_Y)
    p = grasp_pose.position
    log.info(
        f'[test_pick_c] grasp pose=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
        f'quat=(0, 0, 0.0, 1.0)'
    )

    result = pick.pick(grasp_pose, arm=arm)
    log.info(f'[test_pick_c] pick 결과: {result.value}')

    if result != PickResult.SUCCESS: # 실제 로봇 작동 시 != -> == 로 변경 
        carry = Pose()
        carry.position.x = p.x
        carry.position.y = p.y
        carry.position.z = CARRY_Z
        carry.orientation.x, carry.orientation.y, \
            carry.orientation.z, carry.orientation.w = _QUAT_YAW90
        r = client.move_to_pose(carry, arm=arm, velocity=0.3, acceleration=0.3)
        log.info(f'[test_pick_c] carry 상승(z={CARRY_Z}): {r.value}')
    else:
        log.warn('[test_pick_c] pick 실패 — carry 상승 스킵')

    remove_zone_c_table(client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
