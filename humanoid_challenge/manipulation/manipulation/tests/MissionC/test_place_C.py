"""
Mission C place — Perception 연동 버전.
토픽에서 파이프 좌표를 수신하여 부품 삽입.

실행:
  ros2 run manipulation test_place_c
  ros2 run manipulation test_place_c -- --gripper-open 0.3
"""

import argparse
import threading

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.moveit_client import Arm, MoveItClient
from manipulation.skill_primitives.place_skill import PlaceCSkill

ARM                = Arm.RIGHT
PLACE_Y_OFFSET     = -0.030   # 우로 3cm
PERCEPTION_TOPIC   = '/perception/wrist/target_one_pose' # 추후 수정 예정
PERCEPTION_TIMEOUT = 100.0


def _make_pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def _wait_for_pose(node: Node, log, timeout: float) -> Pose | None:
    received: list[Pose] = []
    event = threading.Event()

    def _cb(msg: PoseStamped) -> None:
        if event.is_set():
            return
        p = msg.pose.position
        log.info(f'[place_c] pipe 좌표 수신: ({p.x:.3f},{p.y:.3f},{p.z:.3f})')
        received.append(_make_pose(p.x, p.y, p.z))
        event.set()

    sub = node.create_subscription(
        PoseStamped, PERCEPTION_TOPIC, _cb, 10,
        callback_group=ReentrantCallbackGroup(),
    )

    log.info(f'[place_c] 좌표 대기 (최대 {timeout}s)')
    ok = event.wait(timeout=timeout)
    node.destroy_subscription(sub)

    if not ok:
        log.error('[place_c] 좌표 수신 타임아웃')
        return None
    return received[0]


def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gripper-open', type=float, default=0.0, dest='gripper_open',
                        metavar='AMOUNT', help='그리퍼 벌림 정도 (0.0=완전열림, 1.0=완전닫힘)')
    return parser.parse_args()


def main():
    args = _parse_args()

    rclpy.init()
    node    = Node('test_place_c')
    log     = node.get_logger()
    client  = MoveItClient(node)
    gripper = GripperInterface(node)
    place   = PlaceCSkill(node, client, gripper)

    pipe_pose = _wait_for_pose(node, log, PERCEPTION_TIMEOUT)
    if pipe_pose is None:
        node.destroy_node()
        rclpy.shutdown()
        return

    pipe_pose.position.y += PLACE_Y_OFFSET
    result = place.place(pipe_pose, arm=ARM, gripper_open_amount=args.gripper_open)
    log.info(f'[place_c] 결과: {result.value}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
