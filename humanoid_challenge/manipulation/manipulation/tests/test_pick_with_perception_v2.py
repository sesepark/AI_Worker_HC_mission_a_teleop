"""
Perception 연동 Pick 테스트 v2.
1차 capture pose(joint) 이동 → 1차 스캔(대략 좌표) →
detail capture pose(pose) 이동 → 2차 스캔(정밀 좌표) → pick 실행.

실행:
  ros2 run manipulation test_pick_with_perception_v2
"""

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient, Arm
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import setup_zone_a, clear_all_objects
from manipulation.skill_primitives.grasp_assessment import GraspAssessment
from manipulation.skill_primitives.grasp_skill import GraspSkill
from manipulation.skill_primitives.pick_skill import PickSkill, PickResult
from manipulation.skill_primitives.planning_filter import PlanningFilter
from manipulation.skill_primitives.mission_a_grasp_adapter import build_mission_a_grasp_pose
from manipulation.skill_primitives.two_stage_capture import TwoStageCapture

ARM = Arm.RIGHT

CAPTURE_JOINTS     = [-0.845728, -1.145170, 1.013616, -1.996687, -2.846256, 0.801760, -1.580400]
CAPTURE_Z          = 1.020
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
    node = Node('test_pick_with_perception_v2')
    log  = node.get_logger()

    client  = MoveItClient(node)
    gripper = GripperInterface(node)
    assess  = GraspAssessment(node)
    grasp   = GraspSkill(node, gripper, assess)
    pfilter = PlanningFilter(client, log=log)
    pick    = PickSkill(node, client, gripper, grasp, pfilter)

    log.info('[perception_pick_v2] Scene 초기화')
    clear_all_objects(client)
    setup_zone_a(client)

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
        log.error('[perception_pick_v2] 2단계 캡처 실패 — 종료')
        clear_all_objects(client)
        node.destroy_node()
        rclpy.shutdown()
        return

    # ── Pick ─────────────────────────────────────────────────────────
    grasp_pose = build_mission_a_grasp_pose(center_pose)
    p = grasp_pose.position
    log.info(f'[perception_pick_v2] grasp pose=({p.x:.3f},{p.y:.3f},{p.z:.3f})')

    result = pick.pick(grasp_pose, arm=ARM)
    log.info(f'[perception_pick_v2] pick 결과: {result.value}')

    if result == PickResult.SUCCESS:
        carry = _make_pose(p.x, p.y, CARRY_Z)
        r = client.move_to_pose(carry, arm=ARM, velocity=0.3, acceleration=0.3,
                                pipeline='pilz_industrial_motion_planner', planner='PTP')
        log.info(f'[perception_pick_v2] carry 상승(z={CARRY_Z}): {r.value}')
        log.info('[perception_pick_v2] navigation 대기 상태')
    else:
        log.warn('[perception_pick_v2] pick 실패 — carry 상승 스킵')

    clear_all_objects(client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
