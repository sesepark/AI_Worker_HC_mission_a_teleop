import time
from typing import Optional

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState


POSITION_CLOSED = 1150   # raw position 기준 완전 닫힘

# 파지 판정 threshold (실로봇 18회 측정 기반)
# 열림:         pos=0~7      eff=0~116
# 파지 성공:    pos=986~1099 eff=660~661
# 파지 실패:    pos=1146~1148 eff=1~30
DEFAULT_POSITION_LOW  = 800   # 하한: 열림(max 7) 대비 여유
DEFAULT_POSITION_HIGH = 1130  # 상한: 파지 성공 max(1099)과 실패 min(1146) 사이
DEFAULT_EFFORT_THRESH = 400   # 열림 max(116) 대비 284 여유, 파지 min(660) 대비 260 여유

STABLE_DURATION = 1.0   # 안정성 판정 유지 시간 (초)
STABLE_POLL_HZ  = 20


class GraspAssessment:

    def __init__(
        self,
        node: Node,
        position_low: float  = DEFAULT_POSITION_LOW,
        position_high: float = DEFAULT_POSITION_HIGH,
        effort_thresh: float = DEFAULT_EFFORT_THRESH,
        callback_group       = None,
    ):
        self._node          = node
        self._log           = node.get_logger()
        self._position_low  = position_low
        self._position_high = position_high
        self._effort_thresh = effort_thresh

        cb = callback_group or ReentrantCallbackGroup()

        self._positions: dict[str, float] = {}
        self._efforts:   dict[str, float] = {}

        self._node.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10,
            callback_group=cb,
        )
        self._log.info(
            f'[GraspAssessment] 초기화 완료 '
            f'(pos={position_low}~{position_high}, eff_thresh={effort_thresh})'
        )

    def _joint_state_cb(self, msg: JointState):
        for name, pos, eff in zip(msg.name, msg.position, msg.effort):
            self._positions[name] = pos
            self._efforts[name]   = eff

    def _joint_name(self, side: str) -> str:
        prefix = 'l' if side == 'left' else 'r'
        return f'gripper_{prefix}_joint1'

    def _get_position_raw(self, side: str) -> Optional[float]:
        norm = self._positions.get(self._joint_name(side))
        return None if norm is None else norm * POSITION_CLOSED

    def _get_effort_abs(self, side: str) -> float:
        return abs(self._efforts.get(self._joint_name(side), 0.0))

    def assess(self, side: str) -> dict:
        """현재 gripper position/effort로 파지 여부 즉시 판정."""
        pos = self._get_position_raw(side)
        eff = self._get_effort_abs(side)

        if pos is None:
            self._log.warn(f'[GraspAssessment] {side} joint_states 미수신')
            return {
                'is_grasping': False,
                'position_ok': False,
                'effort_ok':   False,
                'position':    None,
                'effort':      eff,
            }

        position_ok = self._position_low < pos < self._position_high
        effort_ok   = eff > self._effort_thresh     # 힘을 쓰고 있는지
        is_grasping = position_ok and effort_ok

        self._log.info(
            f'[GraspAssessment] pos={pos:.1f}({self._position_low}~{self._position_high})  '
            f'eff={eff:.4f}/{self._effort_thresh}  '
            f'position_ok={position_ok}  effort_ok={effort_ok}  '
            f'is_grasping={is_grasping}'
        )

        return {
            'is_grasping': is_grasping,
            'position_ok': position_ok,
            'effort_ok':   effort_ok,
            'position':    round(pos, 2),
            'effort':      round(eff, 4),
        }

    def assess_stable(self, side: str, duration: float = STABLE_DURATION) -> bool:
        """position_ok AND effort_ok 가 duration초 이상 유지되면 True."""
        self._log.info(
            f'[GraspAssessment] {side} 안정성 평가 시작 (목표: {duration}초 유지)'
        )

        stable_start: Optional[float] = None
        poll_interval = 1.0 / STABLE_POLL_HZ
        deadline = time.time() + duration * 5

        while time.time() < deadline:
            time.sleep(poll_interval)
            result = self.assess(side)

            if result['is_grasping']:
                if stable_start is None:
                    stable_start = time.time()
                elapsed = time.time() - stable_start
                if elapsed >= duration:
                    self._log.info(
                        f'[GraspAssessment] ✅ 파지 안정 확인 ({elapsed:.2f}초 유지)'
                    )
                    return True
            else:
                if stable_start is not None:
                    self._log.info('[GraspAssessment] 조건 불충족 → 타이머 리셋')
                stable_start = None

        self._log.warn('[GraspAssessment] ❌ 안정성 조건 미충족 (시간 초과)')
        return False
