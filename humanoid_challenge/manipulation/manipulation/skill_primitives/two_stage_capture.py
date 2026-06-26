# skill_primitives/two_stage_capture.py
#
# 2단계 캡처: joint 1차 이동 → 1차 스캔 → detail pose 이동 → 2차 스캔 → 정밀 좌표 반환.
# 시스템 통합 시 이 클래스만 서버에 연결하면 됨.

import threading
import time
from typing import List

from geometry_msgs.msg import Pose, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import MoveItClient, Arm, MoveResult


class TwoStageCapture:

    def __init__(
        self,
        node: Node,
        client: MoveItClient,
        capture_joints: List[float],
        capture_z: float = 1.020,
        settle: float = 2.0,
        perception_timeout: float = 10.0,
        arm: Arm = Arm.RIGHT,
    ) -> None:
        self._node = node
        self._log = node.get_logger()
        self._client = client
        self._capture_joints = capture_joints
        self._capture_z = capture_z
        self._settle = settle
        self._timeout = perception_timeout
        self._arm = arm

    def run(self) -> Pose | None:
        """2단계 캡처 실행. 성공 시 정밀 좌표 Pose 반환, 실패 시 None."""

        # ── 1. 1차 capture pose 이동 (joint) ─────────────────────────
        self._log.info('[TwoStageCapture] 1차 capture pose 이동')
        r = self._client.move_to_joints(
            self._capture_joints, arm=self._arm,
            velocity=0.2, acceleration=0.2,
            pipeline='pilz_industrial_motion_planner', planner='PTP',
        )
        if r != MoveResult.SUCCEEDED:
            self._log.error(f'[TwoStageCapture] 1차 이동 실패: {r.value}')
            return None

        time.sleep(self._settle)

        # ── 2. 1차 스캔 (대략 좌표) ──────────────────────────────────
        rough = self._wait_for_pose('1차 스캔')
        if rough is None:
            return None

        # ── 3. Detail capture pose 이동 (rough x,y + 고정 z) ─────────
        detail = _make_pose(rough.position.x, rough.position.y, self._capture_z)
        p = detail.position
        self._log.info(f'[TwoStageCapture] detail capture 이동: ({p.x:.3f},{p.y:.3f},{p.z:.3f})')
        r = self._client.move_to_pose(
            detail, arm=self._arm,
            velocity=0.2, acceleration=0.2,
            pipeline='pilz_industrial_motion_planner', planner='PTP',
        )
        if r != MoveResult.SUCCEEDED:
            self._log.error(f'[TwoStageCapture] detail 이동 실패: {r.value}')
            return None

        time.sleep(self._settle)

        # ── 4. 2차 스캔 (정밀 좌표) ──────────────────────────────────
        precise = self._wait_for_pose('2차 스캔')
        if precise is None:
            return None

        p = precise.position
        self._log.info(f'[TwoStageCapture] 정밀 좌표: ({p.x:.3f},{p.y:.3f},{p.z:.3f})')
        return precise

    def _wait_for_pose(self, label: str) -> Pose | None:
        """'/perception/wrist/target_one_pose' 에서 최신 Pose 한 번 수신."""
        received: list[Pose] = []
        event = threading.Event()

        def _cb(msg: PoseStamped) -> None:
            if event.is_set():
                return
            p = msg.pose.position
            self._log.info(f'[TwoStageCapture] {label}: ({p.x:.3f},{p.y:.3f},{p.z:.3f})')
            received.append(_make_pose(p.x, p.y, p.z))
            event.set()

        sub = self._node.create_subscription(
            PoseStamped,
            '/perception/wrist/target_one_pose',
            _cb,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        self._log.info(f'[TwoStageCapture] {label} 대기 (최대 {self._timeout}s)')
        ok = event.wait(timeout=self._timeout)
        self._node.destroy_subscription(sub)

        if not ok:
            self._log.error(f'[TwoStageCapture] {label} 타임아웃')
            return None
        return received[0]


def _make_pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose
