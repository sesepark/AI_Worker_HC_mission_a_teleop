"""
Mission C Perception 연동 Pick 테스트.
1차 capture pose(joint) 이동 → 1차 스캔(대략 좌표) →
detail capture pose(pose) 이동 → 2차 스캔(정밀 좌표) → pick 실행.

실행:
  ros2 run manipulation test_pick_c
"""

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient, Arm
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import setup_zone_c_table, remove_zone_c_table
from manipulation.skill_primitives.grasp_assessment import GraspAssessment
from manipulation.skill_primitives.grasp_skill import GraspSkill
from manipulation.skill_primitives.pick_skill import PickSkill, PickResult
from manipulation.skill_primitives.planning_filter import PlanningFilter
from manipulation.skill_primitives.mission_c_grasp_adapter import build_c_grasp_pose
from manipulation.skill_primitives.two_stage_capture import TwoStageCapture

ARM = Arm.RIGHT

CAPTURE_JOINTS     = [-0.514537, -1.079939,  0.611448, -2.036518, -2.695534,  1.082374, -1.580207]
CAPTURE_Z          = 1.050
CAPTURE_SETTLE     = 2.0
PERCEPTION_TIMEOUT = 100.0
CARRY_Z            = 1.150


def _make_pose(x, y, z):
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def main():
    rclpy.init()
    node = Node('test_pick_c')
    log  = node.get_logger()

    client  = MoveItClient(node)
    gripper = GripperInterface(node)
    assess  = GraspAssessment(node)
    grasp   = GraspSkill(node, gripper, assess)
    pfilter = PlanningFilter(client, log=log)
    pick    = PickSkill(node, client, gripper, grasp, pfilter)

    log.info('[pick_c] Scene 초기화')

    # ── 2단계 캡처 ───────────────────────────────────────────────────
    capture = TwoStageCapture(
        node, client,
        capture_joints=CAPTURE_JOINTS,
        capture_z=CAPTURE_Z,
        settle=CAPTURE_SETTLE,
        perception_timeout=PERCEPTION_TIMEOUT,
        arm=ARM,
    )
    center_pose = capture.run()

    if center_pose is None:
        log.error('[pick_c] 2단계 캡처 실패 — 종료')
        node.destroy_node()
        rclpy.shutdown()
        return

    # ── Pick ─────────────────────────────────────────────────────────
    # setup_zone_c_table(client)

    grasp_pose = build_c_grasp_pose(center_pose)
    p = grasp_pose.position
    log.info(f'[pick_c] grasp pose=({p.x:.3f},{p.y:.3f},{p.z:.3f})')

    result = pick.pick(grasp_pose, arm=ARM)
    log.info(f'[pick_c] pick 결과: {result.value}')

    if result == PickResult.SUCCESS:
        carry = _make_pose(p.x, p.y, CARRY_Z)
        r = client.move_to_pose(carry, arm=ARM, velocity=0.3, acceleration=0.3)
        log.info(f'[pick_c] carry 상승(z={CARRY_Z}): {r.value}')
    else:
        log.warning('[pick_c] pick 실패 — carry 상승 스킵')

    # remove_zone_c_table(client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
