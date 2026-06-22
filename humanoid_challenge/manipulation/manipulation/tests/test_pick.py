"""
Pick 단독 테스트.
Zone A collision object 등록 후, 물체 중앙 좌표를 TopDownPoseSelector 에 넘겨
grasp pose(y -0.05 오프셋 적용)를 받아 pick 실행.
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
from manipulation.skill_primitives.top_down_pose_selector import TopDownPoseSelector

# ── 파라미터 ─────────────────────────────────────────────────────────
ARM = Arm.RIGHT

# yellow_box 물체 중앙 좌표 (테이블 앞 엣지 x=0.050 기준)
# TopDownPoseSelector 가 grasp_y_offset=-0.05 를 적용해 실제 grasp pose 를 계산한다.
# yellow_box: center_x=0.320, center_y=-0.295
#
# 시뮬 검증 완료 좌표 (모두 SUCCEEDED):
#   중앙:       CENTER_X=0.320, CENTER_Y=-0.250  ← 기본값
#   앞쪽(로봇): CENTER_X=0.250, CENTER_Y=-0.250
#   깊숙이:     CENTER_X=0.400, CENTER_Y=-0.250
#   옆쪽(-y):   CENTER_X=0.320, CENTER_Y=-0.350
# 시뮬 -0.01 ~ -0.58
CENTER_X =  0.39992105050980137
CENTER_Y = -0.13685742999600736
CENTER_Z =  0.83   # 테이블(0.800) + 물체 여유

# pick 완료 후 올라갈 carry 높이
# 테이블(0.800) + 박스(0.200) + 여유(0.020) = 1.020
CARRY_Z = 1.150
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

    client   = MoveItClient(node)
    gripper  = GripperInterface(node)
    assess   = GraspAssessment(node)
    grasp    = GraspSkill(node, gripper, assess)
    pick     = PickSkill(node, client, gripper, grasp)
    selector = TopDownPoseSelector(client, log=log)

    log.info('[test_pick] Scene 초기화')
    clear_all_objects(client)
    setup_zone_a(client)

    center = _make_pose(CENTER_X, CENTER_Y, CENTER_Z)
    log.info(f'[test_pick] selector 실행 — center=({CENTER_X},{CENTER_Y},{CENTER_Z})')

    selection = selector.select_grasp_from_center(center, fixed_arm=ARM)
    if selection is None:
        log.error(f'[test_pick] selector 실패: {selector.last_failure_reason}')
        node.destroy_node()
        rclpy.shutdown()
        return

    approach_height = selection.pre_pose.position.z - selection.target_pose.position.z
    p = selection.target_pose.position
    log.info(
        f'[test_pick] grasp pose=({p.x:.3f},{p.y:.3f},{p.z:.3f}) '
        f'arm={selection.arm.value} planner={selection.global_planner} '
        f'approach={approach_height:.3f}m'
    )

    result = pick.pick(
        grasp_pose=selection.target_pose,
        arm=selection.arm,
        local_mode=selection.local_mode,
        global_pipeline=selection.global_pipeline,
        global_planner=selection.global_planner,
        approach_height=approach_height,
    )
    log.info(f'[test_pick] pick 결과: {result.value}')

    if result.value == 'success':
        carry = _make_pose(p.x, p.y, CARRY_Z)
        r = client.move_to_pose(carry, arm=selection.arm, velocity=0.3, acceleration=0.3)
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
