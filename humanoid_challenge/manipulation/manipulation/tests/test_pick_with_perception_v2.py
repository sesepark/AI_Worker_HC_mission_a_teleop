"""
Perception 연동 Pick 테스트 v2.
1차 capture pose(joint) 이동 → 1차 스캔(대략 좌표) →
detail capture pose(pose) 이동 → 2차 스캔(정밀 좌표) → pick 실행.

실행:
  ros2 run manipulation test_pick_with_perception_v2
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

CAPTURE_JOINTS  = [-0.845728, -1.145170, 1.013616, -1.996687, -2.846256, 0.801760, -1.580400]
CAPTURE_Z       = 1.120   # detail capture pose 고정 z — CAPTURE_JOINTS FK z와 일치시켜야 IK 성공
CAPTURE_SETTLE  = 2.0     # 각 pose 안정화 대기 (초)
PERCEPTION_TIMEOUT = 100.0
CARRY_Z = 1.150


def _make_pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def _wait_for_pose(node, log, label: str) -> Pose | None:
    """'/perception/wrist/target_one_pose' 에서 Pose 한 번 수신해 반환."""
    received: list[Pose] = []
    event = threading.Event()

    def _cb(msg: PoseStamped) -> None:
        if event.is_set():
            return
        p = msg.pose.position
        log.info(f'[perception_pick_v2] {label} 수신: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}')
        received.append(_make_pose(p.x, p.y, p.z))
        event.set()

    cb_group = ReentrantCallbackGroup()
    sub = node.create_subscription(
        PoseStamped,
        '/perception/wrist/target_one_pose',
        _cb,
        10,
        callback_group=cb_group,
    )

    log.info(f'[perception_pick_v2] {label} 대기 (최대 {PERCEPTION_TIMEOUT}s)')
    ok = event.wait(timeout=PERCEPTION_TIMEOUT)
    node.destroy_subscription(sub)

    if not ok:
        log.error(f'[perception_pick_v2] {label} 타임아웃')
        return None

    return received[0]


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

    # ── 1. Scene 초기화 ───────────────────────────────────────────────
    log.info('[perception_pick_v2] Scene 초기화')
    clear_all_objects(client)
    # setup_zone_a는 pick 직전에 호출 — scan 자세에서 arm link가 yellow box wall을
    # 통과하므로, 미리 추가하면 start state invalid로 이후 planning이 전부 fail함.

    # ── 2. 1차 capture pose 이동 (joint) ─────────────────────────────
    log.info(f'[perception_pick_v2] 1차 capture pose 이동: {[f"{v:.3f}" for v in CAPTURE_JOINTS]}')
    r = client.move_to_joints(CAPTURE_JOINTS, arm=ARM, velocity=0.2, acceleration=0.2)
    log.info(f'[perception_pick_v2] 1차 결과: {r.value}')
    if r.value != 'succeeded':
        log.error('[perception_pick_v2] 1차 capture pose 실패 — 종료')
        clear_all_objects(client)
        node.destroy_node()
        rclpy.shutdown()
        return

    log.info(f'[perception_pick_v2] settle {CAPTURE_SETTLE}s 대기')
    time.sleep(CAPTURE_SETTLE)

    # CAPTURE_JOINTS의 손목 orientation을 FK로 읽어둠.
    # identity quat은 z=CAPTURE_Z에서 IK solution이 없으므로 반드시 재사용해야 함.
    fk_pose = client.get_pose(arm=ARM)
    if fk_pose is None:
        log.error('[perception_pick_v2] capture pose FK 실패 — 종료')
        clear_all_objects(client)
        node.destroy_node()
        rclpy.shutdown()
        return

    # ── 3. 1차 스캔 (대략 좌표) ───────────────────────────────────────
    rough_pose = _wait_for_pose(node, log, '1차 스캔')
    if rough_pose is None:
        clear_all_objects(client)
        node.destroy_node()
        rclpy.shutdown()
        return

    # ── 4. Detail capture pose 이동 (joints via IK) ───────────────────
    # move_to_pose + OMPL은 CAPTURE_JOINTS seed에서 IK sampling이 즉시 실패함.
    # solve_ik로 joint 값을 먼저 구한 뒤 move_to_joints로 이동 (Pilz PTP의 내부 동작과 동일).
    detail_capture = _make_pose(rough_pose.position.x, rough_pose.position.y, CAPTURE_Z)
    detail_capture.orientation = fk_pose.orientation
    p = detail_capture.position
    log.info(f'[perception_pick_v2] detail capture pose IK: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}')
    detail_joints = client.solve_ik(detail_capture, arm=ARM)
    if detail_joints is None:
        log.error('[perception_pick_v2] detail capture IK 실패 — 종료')
        clear_all_objects(client)
        node.destroy_node()
        rclpy.shutdown()
        return
    r = client.move_to_joints(detail_joints, arm=ARM, velocity=0.2, acceleration=0.2)
    log.info(f'[perception_pick_v2] detail capture 결과: {r.value}')
    if r.value != 'succeeded':
        log.error('[perception_pick_v2] detail capture pose 실패 — 종료')
        clear_all_objects(client)
        node.destroy_node()
        rclpy.shutdown()
        return

    log.info(f'[perception_pick_v2] settle {CAPTURE_SETTLE}s 대기')
    time.sleep(CAPTURE_SETTLE)

    # ── 5. 2차 스캔 (정밀 좌표) ───────────────────────────────────────
    center_pose = _wait_for_pose(node, log, '2차 스캔')
    if center_pose is None:
        clear_all_objects(client)
        node.destroy_node()
        rclpy.shutdown()
        return

    # ── 6. Grasp pose 구성 → Pick ─────────────────────────────────────
    # setup_zone_a(client)
    time.sleep(0.5)   # collision objects가 MoveIt planning scene에 전파될 때까지 대기
    grasp_pose = build_mission_a_grasp_pose(center_pose)
    p = grasp_pose.position
    log.info(f'[perception_pick_v2] grasp pose=({p.x:.3f},{p.y:.3f},{p.z:.3f})')

    result = pick.pick(grasp_pose, arm=ARM)
    log.info(f'[perception_pick_v2] pick 결과: {result.value}')

    if result == PickResult.SUCCESS:
        carry = _make_pose(p.x, p.y, CARRY_Z)
        r = client.move_to_pose(carry, arm=ARM, velocity=0.3, acceleration=0.3)
        log.info(f'[perception_pick_v2] carry 상승(z={CARRY_Z}): {r.value}')
        log.info('[perception_pick_v2] navigation 대기 상태')
    else:
        log.warn('[perception_pick_v2] pick 실패 — carry 상승 스킵')

    clear_all_objects(client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
