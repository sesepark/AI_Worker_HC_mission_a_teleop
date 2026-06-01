"""
grasp_assessment.py
--------------------
RH-P12-RN 그리퍼 파지 품질 평가 모듈 (재작성).

평가 파이프라인:
  1. LUT 조회  : 오브젝트별 effort / position 임계값 취득.
                 미등록 오브젝트는 "ETC" 항목으로 폴백.
  2. 실시간 평가: moving-average window(크기 5)로 노이즈를 제거.
                 effort > eff_thresh AND position > pos_thresh 이 1초 이상
                 유지 → 파지 성공.
  3. 이동 중 평가: assess_moving() 으로 슬립 감지.
                 슬립 감지 시 ERR 출력 + 파지력 10% 증가.
  4. open 명령  : stop() 호출로 평가/클로즈 루프 즉시 종료.

PCD 관련 함수는 스텁(stub)으로 선언되어 있으며, 향후 구현 필요.
"""

import time
import json
import os
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ── 패키지 경로 ──────────────────────────────────────────────────────────────
import ament_index_python.packages as ament
_PKG_DIR = ament.get_package_share_directory('ai_worker_manipulation')
LUT_PATH = os.path.join(_PKG_DIR, 'data', 'object_lut.json')

# ── 하드웨어 상수 ─────────────────────────────────────────────────────────────
# RH-P12-RN: 0 = 완전 닫힘, 1150 = 완전 열림
POSITION_OPEN   = 1130   # Open 명령 목표 위치
POSITION_CLOSED = 0      # Close 명령 목표 위치
HW_MAX_POSITION = 1150   # 하드웨어 최대치 (정규화 역산용)

# ── 평가 파라미터 ─────────────────────────────────────────────────────────────
WINDOW_SIZE      = 5     # moving-average 윈도우 크기
STABLE_DURATION  = 1.0   # 파지 성공 판정 유지 시간 (초)
POLL_HZ          = 20    # 폴링 주기 (Hz)
GRASP_FORCE_INC  = 0.10  # 슬립 시 파지력 증가율 (10%)


# ══════════════════════════════════════════════════════════════════════════════
# PCD (Point Cloud Data) 스텁 함수
# 아래 두 함수는 구현이 완료되지 않았으며, 향후 실제 PCD 처리 로직으로 교체 필요.
# ══════════════════════════════════════════════════════════════════════════════

def is_object(side: str, object_name: str) -> bool:
    """
    [STUB] 현재 그리퍼가 잡고 있는 물체의 PCD가 LUT의 PCD와 일치하는지 확인.

    Parameters
    ----------
    side        : 'left' | 'right'
    object_name : LUT 오브젝트 이름

    Returns
    -------
    bool : PCD 일치 여부 (True = 동일 물체)
    """
    raise NotImplementedError(
        "[PCD STUB] is_object() 미구현 — PCD 비교 로직 구현 필요."
    )


def is_stable_pcd(side: str) -> bool:
    """
    [STUB] 파지 직후 PCD와 현재 PCD를 비교하여 물체가 안정적으로 잡혀있는지 확인.

    Parameters
    ----------
    side : 'left' | 'right'

    Returns
    -------
    bool : PCD 안정 여부 (True = 안정)
    """
    raise NotImplementedError(
        "[PCD STUB] is_stable_pcd() 미구현 — PCD 안정성 비교 로직 구현 필요."
    )


# ══════════════════════════════════════════════════════════════════════════════
# LUT 로더
# ══════════════════════════════════════════════════════════════════════════════

def _load_lut(path: str) -> dict:
    with open(path, 'r') as f:
        data = json.load(f)
    return data['objects']


# ══════════════════════════════════════════════════════════════════════════════
# GraspAssessment
# ══════════════════════════════════════════════════════════════════════════════

class GraspAssessment:
    """
    실시간 파지 품질 평가기.

    Parameters
    ----------
    node     : rclpy.node.Node  — 외부 ROS2 노드
    lut_path : str              — object_lut.json 경로
    """

    def __init__(self, node: Node, lut_path: str = LUT_PATH):
        self._node = node
        self._lut  = _load_lut(lut_path)

        # /joint_states 수신 버퍼
        self._positions: dict[str, float] = {}
        self._efforts:   dict[str, float] = {}

        # moving-average 윈도우 (side → deque)
        self._pos_window: dict[str, deque] = {}
        self._eff_window: dict[str, deque] = {}

        # 평가 루프 제어 플래그
        self._running: bool = False

        self._sub = self._node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
        )
        self._node.get_logger().info('[GraspAssessment] 초기화 완료.')

    # ── 콜백 ─────────────────────────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        for name, pos, eff in zip(msg.name, msg.position, msg.effort):
            self._positions[name] = pos
            self._efforts[name]   = eff

    # ── 헬퍼 ─────────────────────────────────────────────────────────────────
    def _joint_name(self, side: str) -> str:
        if side not in ('left', 'right'):
            raise ValueError(f"side 는 'left' 또는 'right' 여야 합니다: {side}")
        prefix = 'l' if side == 'left' else 'r'
        return f'gripper_{prefix}_joint1'

    def _raw_position(self, side: str) -> Optional[float]:
        """정규화된 joint position (0~1) → raw (0~1150)"""
        joint = self._joint_name(side)
        norm  = self._positions.get(joint)
        if norm is None:
            return None
        return norm * HW_MAX_POSITION

    def _raw_effort(self, side: str) -> float:
        joint = self._joint_name(side)
        return abs(self._efforts.get(joint, 0.0))

    def _avg_position(self, side: str) -> Optional[float]:
        """moving-average 적용 position"""
        raw = self._raw_position(side)
        if raw is None:
            return None
        win = self._pos_window.setdefault(side, deque(maxlen=WINDOW_SIZE))
        win.append(raw)
        return sum(win) / len(win)

    def _avg_effort(self, side: str) -> float:
        """moving-average 적용 effort"""
        raw = self._raw_effort(side)
        win = self._eff_window.setdefault(side, deque(maxlen=WINDOW_SIZE))
        win.append(raw)
        return sum(win) / len(win)

    def _get_lut_entry(self, object_name: str) -> dict:
        """LUT 조회. 미등록 시 'ETC' 항목 반환."""
        if object_name in self._lut:
            return self._lut[object_name]
        available = list(self._lut.keys())
        self._node.get_logger().warn(
            f"[GraspAssessment] '{object_name}' 가 LUT에 없습니다. "
            f"'ETC' 항목으로 폴백합니다. 등록된 오브젝트: {available}"
        )
        if 'ETC' not in self._lut:
            raise KeyError(
                f"'{object_name}' 도, 'ETC' 항목도 LUT에 없습니다."
            )
        return self._lut['ETC']

    # ── 공개 API ──────────────────────────────────────────────────────────────
    def stop(self):
        """
        open 명령 수신 시 호출.
        진행 중인 assess_on_close / assess_moving 루프를 즉시 종료.
        """
        self._running = False
        self._node.get_logger().info('[GraspAssessment] stop() 호출 — 평가 중단.')

    def get_lut_objects(self) -> list:
        return list(self._lut.keys())

    # ── 1단계: 닫힘 중 평가 ───────────────────────────────────────────────────
    def assess_on_close(self, side: str, object_name: str, timeout: float = 5.0) -> bool:
        """
        Close 명령 실행과 동시에 호출.
        position 과 effort 조건이 STABLE_DURATION 초 이상 유지되면 True 반환.
        open 명령(stop()) 수신 시 즉시 False 반환.

        ERR 출력 조건:
          - position 조건 미충족
          - effort 조건 미충족
          - PCD 물체 불일치  (is_object 가 구현된 경우)
          - PCD 불안정       (is_stable_pcd 가 구현된 경우)

        Returns
        -------
        bool : True = 파지 성공
        """
        entry      = self._get_lut_entry(object_name)
        pos_thresh = float(entry['position_min'])
        eff_thresh = float(entry['effort_min'])

        self._running    = True
        stable_start: Optional[float] = None
        poll_interval    = 1.0 / POLL_HZ
        deadline         = time.time() + timeout

        self._node.get_logger().info(
            f'[GraspAssessment] 닫힘 중 평가 시작 | side={side} obj={object_name} '
            f'pos_thresh={pos_thresh} eff_thresh={eff_thresh}'
        )

        while self._running and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=poll_interval)

            pos = self._avg_position(side)
            eff = self._avg_effort(side)

            if pos is None:
                self._node.get_logger().warn(
                    f'[GraspAssessment] {side} joint_states 미수신.'
                )
                stable_start = None
                continue

            # ── 개별 조건 평가 ────────────────────────────────────────────
            # 닫히면서 position 이 커짐(0→). 물체를 잡으면 pos_thresh 이상 유지.
            position_ok = pos < pos_thresh
            effort_ok   = eff >  eff_thresh

            # ERR 출력
            if not position_ok:
                self._node.get_logger().error(
                    f'[ERR] Position 조건 미충족 | '
                    f'pos={pos:.1f} >= threshold={pos_thresh}'
                )
            if not effort_ok:
                self._node.get_logger().error(
                    f'[ERR] Effort 조건 미충족 | '
                    f'eff={eff:.4f} <= threshold={eff_thresh}'
                )

            # PCD 검사 (스텁 — NotImplementedError 발생 시 건너뜀)
            try:
                if not is_object(side, object_name):
                    self._node.get_logger().error(
                        f'[ERR] PCD 불일치 — 파지 물체가 {object_name} 이 아닙니다.'
                    )
                if not is_stable_pcd(side):
                    self._node.get_logger().error(
                        '[ERR] PCD 불안정 — 물체가 흔들리고 있습니다.'
                    )
            except NotImplementedError:
                pass  # PCD 미구현 상태에서는 건너뜀

            # ── 안정성 타이머 ────────────────────────────────────────────
            if position_ok and effort_ok:
                if stable_start is None:
                    stable_start = time.time()
                elapsed = time.time() - stable_start
                if elapsed >= STABLE_DURATION:
                    self._node.get_logger().info(
                        f'[GraspAssessment] ✅ 파지 성공 확인 ({elapsed:.2f}초 유지)'
                    )
                    return True
            else:
                stable_start = None

        if time.time() >= deadline:
            self._node.get_logger().warn('[GraspAssessment] ❌ 평가 시간 초과.')
        else:
            self._node.get_logger().info('[GraspAssessment] 평가 중단 (open 명령).')
        return False

    # ── 3단계: 이동 중 슬립 감지 — GripperDriver 준비 후 활성화 ─────────────────
    # def assess_moving(
    #     self,
    #     side: str,
    #     object_name: str,
    #     current_goal_current: float,
    # ) -> float:
    #     """
    #     이동 중 지속적으로 파지 상태를 감시.
    #     슬립 감지 시 ERR 출력 + 파지력 10% 증가.
    #     GripperDriver.set_goal_current() 필요 — 준비 후 활성화.
    #     """
    #     entry      = self._get_lut_entry(object_name)
    #     pos_thresh = float(entry['position_min'])
    #     eff_thresh = float(entry['effort_min'])
    #     self._running = True
    #     goal_current  = current_goal_current
    #     poll_interval = 1.0 / POLL_HZ
    #     while self._running:
    #         rclpy.spin_once(self._node, timeout_sec=poll_interval)
    #         pos = self._avg_position(side)
    #         eff = self._avg_effort(side)
    #         if pos is None:
    #             continue
    #         position_ok = pos < pos_thresh
    #         effort_ok   = eff > eff_thresh
    #         if not (position_ok and effort_ok):
    #             self._node.get_logger().error('[ERR] Slip detected!')
    #             goal_current *= (1.0 + GRASP_FORCE_INC)
    #     return goal_current