"""
grasp_assessment.py
--------------------
RH-P12-RN 그리퍼의 파지 품질을 평가하는 모듈.

평가 기준 (두 가지 동시 충족 필요):
  1. Position 기준 : present_position < LUT[object].position_min
                    → 물체가 끼어 그리퍼가 완전히 닫히지 못함
  2. Effort  기준 : abs(effort) > LUT[object].effort_min
                    → 실제 파지력이 임계값 이상

두 조건이 STABLE_DURATION(기본 1.0초) 이상 유지되면 "파지 성공".

사용 예시:
    from grasp_assessment import GraspAssessment
    ga = GraspAssessment(node)
    result = ga.assess('right', 'bottle')
"""

import time
import json
import os
from typing import Optional

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState


# ── LUT 파일 경로 ────────────────────────────────────────────────────────────
LUT_PATH = os.path.join(os.path.dirname(__file__), '..', 'data', 'object_lut.json')


def _load_lut(path: str) -> dict:
    with open(path, 'r') as f:
        data = json.load(f)
    return data['objects']


# ── 상수 ─────────────────────────────────────────────────────────────────────
# RH-P12-RN position 범위 (raw, 0 = 완전히 열림 / 1150 = 완전히 닫힘)
POSITION_OPEN   = 0
POSITION_CLOSED = 1150

# 안정성 판단 파라미터
STABLE_DURATION  = 1.0   # 이 시간(초) 동안 두 조건 유지 → 성공
STABLE_POLL_HZ   = 20    # 폴링 주기 (Hz)


class GraspAssessment:
    """
    파지 품질 평가기.

    Parameters
    ----------
    node : rclpy.node.Node
        외부에서 생성된 ROS2 노드.
    lut_path : str, optional
        object_lut.json 경로. 기본값은 패키지 내 data/ 디렉토리.
    """

    def __init__(self, node: Node, lut_path: str = LUT_PATH, callback_group=None):
        self._node = node
        self._log  = node.get_logger()
        self._lut  = _load_lut(lut_path)

        cb = callback_group or ReentrantCallbackGroup()

        # /joint_states 에서 읽어온 최신 상태
        self._positions: dict[str, float] = {}
        self._efforts:   dict[str, float] = {}

        self._sub = self._node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
            callback_group=cb,
        )
        self._log.info('[GraspAssessment] 초기화 완료. LUT 로드됨.')

    # ── 콜백 ─────────────────────────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        for name, pos, eff in zip(msg.name, msg.position, msg.effort):
            self._positions[name] = pos
            self._efforts[name]   = eff

    # ── 내부 헬퍼 ────────────────────────────────────────────────────────────
    def _joint_name(self, side: str) -> str:
        """'left'/'right' → 'gripper_l_joint1'/'gripper_r_joint1'"""
        if side not in ('left', 'right'):
            raise ValueError(f"side는 'left' 또는 'right' 여야 합니다. 입력값: {side}")
        prefix = 'l' if side == 'left' else 'r'
        return f'gripper_{prefix}_joint1'

    def _get_position_raw(self, side: str) -> Optional[float]:
        """
        /joint_states 의 position 값을 RH-P12-RN raw 단위(0~1150)로 변환.
        팀원 GripperController 는 0.0~1.0 으로 정규화해 발행하므로
        * POSITION_CLOSED 를 곱해 raw 값으로 복원.
        """
        joint = self._joint_name(side)
        norm = self._positions.get(joint)
        if norm is None:
            return None
        return norm * POSITION_CLOSED  # 0.0~1.0 → 0~1150

    def _get_effort_abs(self, side: str) -> float:
        joint = self._joint_name(side)
        return abs(self._efforts.get(joint, 0.0))

    # ── Public API ────────────────────────────────────────────────────────────
    def get_lut_objects(self) -> list[str]:
        """사용 가능한 LUT 오브젝트 이름 목록 반환."""
        return list(self._lut.keys())

    def assess(self, side: str, object_name: str) -> dict:
        """
        파지 상태를 평가하고 결과를 반환.

        Parameters
        ----------
        side        : 'left' | 'right'
        object_name : LUT에 정의된 오브젝트 이름 (예: 'bottle')

        Returns
        -------
        dict
            {
                'is_grasping' : bool,
                'position_ok' : bool,   # position 조건 충족 여부
                'effort_ok'   : bool,   # effort  조건 충족 여부
                'position'    : float,  # 현재 raw position
                'effort'      : float,  # 현재 effort 절댓값
                'pos_thresh'  : float,  # LUT 기준값
                'eff_thresh'  : float,  # LUT 기준값
            }
        """
        if object_name not in self._lut:
            available = list(self._lut.keys())
            raise KeyError(
                f"오브젝트 '{object_name}' 가 LUT에 없습니다. "
                f"사용 가능: {available}"
            )

        entry        = self._lut[object_name]
        pos_thresh   = float(entry['position_min'])   # raw 0~1150
        eff_thresh   = float(entry['effort_min'])     # Nm (절댓값)

        pos = self._get_position_raw(side)
        eff = self._get_effort_abs(side)

        if pos is None:
            self._log.warn(
                f'[GraspAssessment] {side} 그리퍼 joint_states 미수신.'
            )
            return {
                'is_grasping': False,
                'position_ok': False,
                'effort_ok':   False,
                'position':    None,
                'effort':      eff,
                'pos_thresh':  pos_thresh,
                'eff_thresh':  eff_thresh,
            }

        # ── 두 조건 평가 ──────────────────────────────────────────────────
        # Position 조건:
        #   close(740) 명령 기준으로 position이 pos_thresh 보다 작아야 함
        #   즉 물체가 끼어서 그리퍼가 완전히 못 닫힌 상태
        position_ok = pos < pos_thresh

        # Effort 조건:
        #   파지력이 임계값 이상
        effort_ok = eff > eff_thresh

        is_grasping = position_ok and effort_ok

        return {
            'is_grasping': is_grasping,
            'position_ok': position_ok,
            'effort_ok':   effort_ok,
            'position':    round(pos, 2),
            'effort':      round(eff, 4),
            'pos_thresh':  pos_thresh,
            'eff_thresh':  eff_thresh,
        }

    def assess_stable(self, side: str, object_name: str,
                      duration: float = STABLE_DURATION) -> bool:
        """
        두 조건(position + effort)이 duration 초 이상 동시에 유지되면 True.
        grasp 성공 판단에 사용.

        Parameters
        ----------
        side        : 'left' | 'right'
        object_name : LUT 오브젝트 이름
        duration    : 안정 유지 시간 (초), 기본 1.0

        Returns
        -------
        bool : 파지 안정 성공 여부
        """
        self._log.info(
            f'[GraspAssessment] {side}/{object_name} 안정성 평가 시작 '
            f'(목표: {duration}초 유지)'
        )

        stable_start: Optional[float] = None
        poll_interval = 1.0 / STABLE_POLL_HZ
        deadline = time.time() + duration * 5  # 최대 대기 = duration * 5

        while time.time() < deadline:
            time.sleep(poll_interval)
            result = self.assess(side, object_name)

            if result['is_grasping']:
                if stable_start is None:
                    stable_start = time.time()
                    self._log.info(
                        f'[GraspAssessment] 조건 충족 시작 '
                        f"pos={result['position']:.1f} "
                        f"eff={result['effort']:.4f}"
                    )
                elapsed = time.time() - stable_start
                if elapsed >= duration:
                    self._log.info(
                        f'[GraspAssessment] ✅ 파지 안정 확인 ({elapsed:.2f}초 유지)'
                    )
                    return True
            else:
                if stable_start is not None:
                    self._log.info(
                        f'[GraspAssessment] 조건 불충족 → 타이머 리셋 '
                        f"position_ok={result['position_ok']} "
                        f"effort_ok={result['effort_ok']}"
                    )
                stable_start = None

        self._log.warn(
            '[GraspAssessment] ❌ 안정성 조건 미충족 (시간 초과)'
        )
        return False