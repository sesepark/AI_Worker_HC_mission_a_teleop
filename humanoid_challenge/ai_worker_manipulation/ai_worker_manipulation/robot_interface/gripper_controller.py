import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from ai_worker_manipulation.robot_interface.grasp_assessment import GraspAssessment


# ══════════════════════════════════════════════════════════════════════════════
# ★ 사용자 조정 파라미터 ★
# ══════════════════════════════════════════════════════════════════════════════

# 그리퍼 속도 — trajectory time_from_start (초)
# 값이 작을수록 빠름. 너무 작으면 하드웨어가 못 따라감.
GRIPPER_SPEED: float = 1.0   # ← 여기서 속도를 변경하세요 (단위: 초)

# ══════════════════════════════════════════════════════════════════════════════
# 하드웨어 상수
# ══════════════════════════════════════════════════════════════════════════════

# 정규화 position (0.0 ~ 1.0)
POSITION_OPEN   = 0.98   # Open  목표 (raw 1130 / 1150 ≈ 0.98)
POSITION_CLOSED = 0.0    # Close 목표 (raw 0)

# position == 0 판정 허용 오차 (raw 단위 환산 시 ≈ 5/1150)
CLOSED_THRESHOLD = 0.005

MAX_GRASP_RETRY = 3

# joint 이름
GRIPPER_JOINT = {
    'left' : 'gripper_l_joint1',
    'right': 'gripper_r_joint1',
}

# 컨트롤러 토픽
CONTROLLER_TOPIC = {
    'left' : '/arm_l_controller/joint_trajectory',
    'right': '/arm_r_controller/joint_trajectory',
}


# ══════════════════════════════════════════════════════════════════════════════
# GripperController
# ══════════════════════════════════════════════════════════════════════════════

class GripperController:
    """
    JointTrajectoryController 기반 그리퍼 고수준 제어기.

    Parameters
    ----------
    node : rclpy.node.Node — 외부 ROS2 노드
    """

    def __init__(self, node: Node):
        self._node = node
        self._assessment = GraspAssessment(node)
        self._moving_thread: Optional[threading.Thread] = None

        # side 별 publisher 생성
        self._pubs = {}
        for side, topic in CONTROLLER_TOPIC.items():
            self._pubs[side] = self._node.create_publisher(
                JointTrajectory, topic, 10
            )

        self._node.get_logger().info('[GripperController] 초기화 완료.')

    # ── 내부 헬퍼 ─────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        self._node.get_logger().info(f'[GripperController] {msg}')

    def _err(self, msg: str):
        self._node.get_logger().error(f'[ERR][GripperController] {msg}')

    def _send_position(self, side: str, position: float):
        """
        JointTrajectory 메시지를 생성해 컨트롤러로 전송.

        Parameters
        ----------
        side     : 'left' | 'right'
        position : 정규화 목표 위치 (0.0 ~ 1.0)
        """
        if side not in GRIPPER_JOINT:
            raise ValueError(f"side 는 'left' 또는 'right' 여야 합니다: {side}")

        msg = JointTrajectory()
        msg.joint_names = [GRIPPER_JOINT[side]]

        point = JointTrajectoryPoint()
        point.positions = [float(position)]

        # GRIPPER_SPEED 를 초 단위로 Duration 으로 변환
        secs     = int(GRIPPER_SPEED)
        nanosecs = int((GRIPPER_SPEED - secs) * 1e9)
        point.time_from_start = Duration(sec=secs, nanosec=nanosecs)

        msg.points = [point]
        self._pubs[side].publish(msg)

    def _get_present_position(self, side: str) -> Optional[float]:
        """
        GraspAssessment 의 내부 position 버퍼에서 현재 정규화 position 읽기.
        (별도 subscriber 없이 assessment 의 _positions 재활용)
        """
        joint = GRIPPER_JOINT[side]
        return self._assessment._positions.get(joint)

    # ══════════════════════════════════════════════════════════════════════════
    # Public API
    # ══════════════════════════════════════════════════════════════════════════

    def Open(self, side: str):
        """
        그리퍼를 POSITION_OPEN 으로 열기.
        진행 중인 파지 평가 루프를 즉시 중단.

        Parameters
        ----------
        side : 'left' | 'right'
        """
        # 1. 평가 루프 즉시 중단
        self._assessment.stop()

        if self._moving_thread and self._moving_thread.is_alive():
            self._moving_thread.join(timeout=1.0)

        # 2. 그리퍼 열기
        self._send_position(side, POSITION_OPEN)
        self._log(f'Open({side}) — position {POSITION_OPEN} 전송.')

    def Close(self, side: str):
        """
        그리퍼를 POSITION_CLOSED 로 닫기.

        Parameters
        ----------
        side : 'left' | 'right'
        """
        self._send_position(side, POSITION_CLOSED)
        self._log(f'Close({side}) — position {POSITION_CLOSED} 전송.')

    def Grasp(self, side: str, object_name: str) -> bool:
        """
        물체 파지 시퀀스.

        순서:
          1. Close(side)
          2. assess_on_close() — 파지 평가 (1초 이상 조건 유지 시 성공)
             position == 0 이고 평가 실패 → Open + 재시도 (최대 3회)
             3회 실패 시 ERR 출력 후 False 반환
          3. 파지 성공 → goal_position 을 현재 위치의 절반으로 설정
          4. assess_moving() 을 별도 스레드로 실행 — 이동 중 슬립 감지

        Parameters
        ----------
        side        : 'left' | 'right'
        object_name : LUT 오브젝트 이름 (예: 'bottle')

        Returns
        -------
        bool : True = 파지 성공, False = 파지 실패
        """
        for attempt in range(1, MAX_GRASP_RETRY + 1):
            self._log(f'Grasp({side}, {object_name}) — 시도 {attempt}/{MAX_GRASP_RETRY}')

            # ── 1. Close ────────────────────────────────────────────────────
            self.Close(side)

            # ── 2. 닫힘 중 파지 평가 ────────────────────────────────────────
            grasped = self._assessment.assess_on_close(side, object_name)

            # ── 3. position == 0 도달 여부 확인 ─────────────────────────────
            current_pos = self._get_present_position(side)
            reached_zero = (
                current_pos is not None and current_pos <= CLOSED_THRESHOLD
            )

            if reached_zero and not grasped:
                self._log('position == 0 도달, 파지 미감지 — Open 후 재시도.')
                self.Open(side)
                continue

            if grasped:
                # ── 4. 파지 성공 처리 ────────────────────────────────────
                self._log(f'파지 성공! ({side}/{object_name})')

                # 현재 위치의 절반으로 position 감소
                half_pos = (current_pos / 2.0) if current_pos else 0.0
                self._send_position(side, half_pos)
                self._log(
                    f'position → {half_pos:.3f} '
                    f'(현재 {current_pos:.3f} 의 절반)'
                )

                # ── 5. 이동 중 슬립 감지 (별도 스레드) ──────────────────
                self._moving_thread = threading.Thread(
                    target=self._moving_monitor,
                    args=(side, object_name),
                    daemon=True,
                )
                self._moving_thread.start()
                return True

            # open 명령 등으로 평가가 중단된 경우
            self._log('평가 중단 또는 실패.')
            self.Open(side)

        # ── 3회 모두 실패 ────────────────────────────────────────────────────
        self._err(
            f'Grasp({side}, {object_name}) — '
            f'{MAX_GRASP_RETRY}회 시도 모두 실패! Failure!'
        )
        return False

    # ── 이동 중 슬립 감지 스레드 ──────────────────────────────────────────────
    def _moving_monitor(self, side: str, object_name: str):
        """
        이동 중 슬립 감지 루프.
        슬립 감지 시 assess_moving() 내부에서 ERR 출력.
        JointTrajectoryController 는 Goal Current 직접 제어 불가이므로
        슬립 발생 시 Close() 재전송으로 파지력 유지를 시도.
        """
        # assess_moving 은 슬립 감지 시 ERR 를 출력하고 증가된 current 를 반환.
        # JTC 환경에서는 current 직접 제어 대신 Close 재전송으로 대응.
        self._assessment.assess_moving(
            side, object_name, goal_current=0.0
        )