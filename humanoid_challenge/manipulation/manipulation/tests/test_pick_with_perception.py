"""
Perception 연동 Pick 테스트.
capture pose 이동 후 /perception/wrist/target_one_pose 수신 → pick 실행.

실행:
  ros2 run manipulation test_pick_with_perception
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped

from manipulation.robot_interface.moveit_client import MoveItClient, Arm
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import setup_zone_a, clear_all_objects
from manipulation.skill_primitives.grasp_assessment import GraspAssessment
from manipulation.skill_primitives.grasp_skill import GraspSkill
from manipulation.skill_primitives.pick_skill import PickSkill, PickResult
from manipulation.skill_primitives.planning_filter import PlanningFilter
from manipulation.skill_primitives.mission_a_grasp_adapter import build_mission_a_grasp_pose

ARM = Arm.RIGHT

CAPTURE_JOINTS = [-0.845728, -1.145170, 1.013616, -1.996687, -2.846256, 0.801760, -1.580400]
CAPTURE_SETTLE  = 2.0   # 캡쳐 포즈 안정화 대기 (초)
PERCEPTION_TIMEOUT = 100.0  # 토픽 대기 타임아웃 (초)
CARRY_Z = 1.150


def _make_pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def main():
    rclpy.init()
    node = Node('test_pick_with_perception')
    log  = node.get_logger()

    client  = MoveItClient(node)
    gripper = GripperInterface(node)
    assess  = GraspAssessment(node)
    grasp   = GraspSkill(node, gripper, assess)
    pfilter = PlanningFilter(client, log=log)
    pick    = PickSkill(node, client, gripper, grasp, pfilter)

    # ── 1. Scene 초기화 ───────────────────────────────────────────────
    log.info('[perception_pick] Scene 초기화')
    clear_all_objects(client)
    setup_zone_a(client)

    # ── 2. Capture pose 이동 ──────────────────────────────────────────
    log.info(f'[perception_pick] capture pose 이동: {[f"{v:.3f}" for v in CAPTURE_JOINTS]}')
    r = client.move_to_joints(CAPTURE_JOINTS, arm=ARM, velocity=0.2, acceleration=0.2)
    log.info(f'[perception_pick] capture pose 결과: {r.value}')
    if r.value != 'succeeded':
        log.error('[perception_pick] capture pose 실패 — 종료')
        clear_all_objects(client)
        node.destroy_node()
        rclpy.shutdown()
        return

    log.info(f'[perception_pick] settle {CAPTURE_SETTLE}s 대기')
    time.sleep(CAPTURE_SETTLE)

    # ── 3. Perception 토픽 구독 ───────────────────────────────────────
    received: list[Pose] = []
    event = threading.Event()

    def _cb(msg: PoseStamped) -> None:
        if event.is_set():
            return
        p = msg.pose.position
        log.info(f'[perception_pick] 수신: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}')
        received.append(_make_pose(p.x, p.y, p.z))
        event.set()

    cb_group = ReentrantCallbackGroup()
    node.create_subscription(PoseStamped, '/perception/wrist/target_one_pose', _cb, 10,
                             callback_group=cb_group)
    log.info(f'[perception_pick] /perception/wrist/target_one_pose 대기 (최대 {PERCEPTION_TIMEOUT}s)')

    if not event.wait(timeout=PERCEPTION_TIMEOUT):
        log.error('[perception_pick] 타임아웃 — 토픽 수신 없음')
        clear_all_objects(client)
        node.destroy_node()
        rclpy.shutdown()
        return

    # ── 4. Grasp pose 구성 → Pick ─────────────────────────────────────
    center_pose = received[0]
    grasp_pose  = build_mission_a_grasp_pose(center_pose)
    p = grasp_pose.position
    log.info(f'[perception_pick] grasp pose=({p.x:.3f},{p.y:.3f},{p.z:.3f})')

    result = pick.pick(grasp_pose, arm=ARM)
    log.info(f'[perception_pick] pick 결과: {result.value}')

    if result == PickResult.SUCCESS:
        carry = _make_pose(p.x, p.y, CARRY_Z)
        r = client.move_to_pose(carry, arm=ARM, velocity=0.3, acceleration=0.3)
        log.info(f'[perception_pick] carry 상승(z={CARRY_Z}): {r.value}')
        log.info('[perception_pick] navigation 대기 상태')
    else:
        log.warn('[perception_pick] pick 실패 — carry 상승 스킵')

    clear_all_objects(client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
