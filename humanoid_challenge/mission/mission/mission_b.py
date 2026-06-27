#!/usr/bin/env python3
"""Mission B 통합 오케스트레이터 — 부품 운반 (실 인터페이스 통합판).

feature/mission-b 의 mock 검증용 FSM(이상적 action 계약)을 **실제 팀 산출물**에 맞춰
개조한 버전. 팀 코드는 무변동으로 두고, 본 FSM 이 인터페이스 글루 역할만 수행한다.

실제 인터페이스
- Navigation : `ffw_mission_b_nav` 코디네이터(String 토픽).
    · 송신 /mission_b/system/action  ← A_TO_B / APPROACH_B / B_TO_A / STOP
    · 수신 /mission_b/nav/event       → READY / A_TO_B_ACCEPTED / REACHED_B_STOP_LINE
                                         / APPROACH_B_ACCEPTED / REACHED_B_PLACE_POSE
                                         / B_TO_A_ACCEPTED / REACHED_A / REJECTED:* / FAILED:*
    A_TO_B 내부에서 후진→우횡이동→전진 + LiDAR 정렬(테이블 거리/중앙)을 수행한다.
- Manipulation: `manipulation` 패키지의 독립 실행 스크립트를 subprocess 로 호출.
    · pick  = `ros2 run manipulation test_dual_pick`   (양팔 클램프 + lift)
    · place = `ros2 run manipulation test_dual_place`  (안착 + 해제 가정)
    스크립트는 MoveIt2/컨트롤러를 직접 구동하고 sys.exit(0/1) 로 종료.
- Perception : 이번 통합에서는 배제(있다고 가정). 출발/완료 신호는 동작 완료 직후 open-loop.

채점 단위(각각 단독 실행 가능, stage 파라미터)
  Ⓑ-1 박스 파지 + 출발 선언   = {B1_GRASP, B1_DEPART}
  Ⓑ-2 정지선 도착            = {B2_CARRY, B2_STOPLINE}
  Ⓑ-3 안착 + 왕복 이동       = {B2_APPROACH_TABLE, B3_PLACE, B3_COMPLETE, RETURN}
stage=all 은 전체 cycle 을 연속 실행하되, 단계 경계(B1→B2, B2→B3, 다음 cycle)에서
조종자 확인(`/mission_b/operator_event` == proceed_event)을 대기한다(auto_chain=true 면 무인).

안전: /mission_b/estop(Bool) → nav STOP + manipulation subprocess 종료 후 E_STOP 정지.
"""
from __future__ import annotations

import json
import shlex
import subprocess
import time
from enum import Enum, auto

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String


class State(Enum):
    INIT               = auto()   # nav READY 대기 + 시작 상태 결정
    B1_GRASP           = auto()   # dual_pick subprocess
    B1_DEPART          = auto()   # 출발 신호(open-loop)
    B2_CARRY           = auto()   # A_TO_B → REACHED_B_STOP_LINE
    B2_STOPLINE        = auto()   # 정지선 도착 인정 + dwell (Ⓑ-2)
    B2_APPROACH_TABLE  = auto()   # APPROACH_B → REACHED_B_PLACE_POSE
    B3_PLACE           = auto()   # dual_place subprocess
    B3_COMPLETE        = auto()   # 완료 신호 + count++ (open-loop)
    RETURN             = auto()   # B_TO_A → REACHED_A
    DONE_B             = auto()
    RECOVERY           = auto()
    E_STOP             = auto()


# stage 별 단위 마지막 상태(여기서 단독 stage 는 DONE_B 로 종료)
STAGE_UNITS = {
    'b1': (State.B1_GRASP, State.B1_DEPART),
    'b2': (State.B2_CARRY, State.B2_STOPLINE),
    'b3': (State.B2_APPROACH_TABLE, State.B3_PLACE, State.B3_COMPLETE, State.RETURN),
}


class MissionB(Node):
    def __init__(self) -> None:
        super().__init__('mission_b')

        # --- Parameters ---
        self.mode = str(self.declare_parameter('mode', 'autonomous').value)
        self.stage = str(self.declare_parameter('stage', 'all').value).lower()
        self.auto_chain = bool(self.declare_parameter('auto_chain', False).value)
        self.proceed_event = str(self.declare_parameter('proceed_event', 'proceed').value)
        self.max_boxes = int(self.declare_parameter('max_boxes', 4).value)
        self.max_attempts = int(self.declare_parameter('max_attempts', 8).value)
        self.stop_line_dwell_sec = float(
            self.declare_parameter('stop_line_dwell_sec', 1.5).value)

        # 타임아웃(초) — manipulation/nav 는 실동작이 길어 넉넉하게.
        self.nav_ready_timeout = float(self.declare_parameter('nav_ready_timeout', 30.0).value)
        self.nav_timeout = float(self.declare_parameter('nav_timeout', 180.0).value)
        self.manip_timeout = float(self.declare_parameter('manip_timeout', 300.0).value)
        self.init_timeout = float(self.declare_parameter('init_timeout', 30.0).value)

        # manipulation 실행 명령(파라미터화 — 드라이런 시 'true' 등으로 오버라이드).
        self.pick_cmd = str(self.declare_parameter(
            'pick_cmd', 'ros2 run manipulation test_dual_pick').value)
        self.place_cmd = str(self.declare_parameter(
            'place_cmd', 'ros2 run manipulation test_dual_place').value)

        # 토픽 이름(계약 — 파라미터화)
        nav_action_topic = str(self.declare_parameter(
            'nav_action_topic', '/mission_b/system/action').value)
        nav_event_topic = str(self.declare_parameter(
            'nav_event_topic', '/mission_b/nav/event').value)

        if self.stage not in ('b1', 'b2', 'b3', 'all'):
            self.get_logger().error(f'잘못된 stage={self.stage} → all 로 강제')
            self.stage = 'all'

        cbg = ReentrantCallbackGroup()

        # nav event 는 latched(transient_local) 발행 → 동일 durability 로 구독해야 READY 수신.
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # --- Publishers ---
        self.pub_nav_action = self.create_publisher(String, nav_action_topic, 10)
        self.pub_active_mission = self.create_publisher(String, '/active_mission', 10)
        self.pub_monitor = self.create_publisher(String, '/mission_b/monitor', 10)

        # --- Subscribers ---
        self.sub_nav_event = self.create_subscription(
            String, nav_event_topic, self._on_nav_event, latched_qos, callback_group=cbg)
        self.sub_operator = self.create_subscription(
            String, '/mission_b/operator_event', self._on_operator_event, 10,
            callback_group=cbg)
        self.sub_estop = self.create_subscription(
            Bool, '/mission_b/estop', self._on_estop, 10, callback_group=cbg)

        # --- State storage ---
        self.state: State = State.INIT
        self._state_enter_time = self._now()
        self.box_count = 0
        self.attempts = 0

        # 신호/검증 플래그 (per-box, _on_enter(B1_GRASP)에서 리셋)
        self.departure_ready = False
        self.delivery_complete = False
        self.stopline_reached = False

        # nav event 1건 추적(store-only + 1회 소비)
        self._nav_event: str | None = None
        self._nav_event_consumed = True
        self._nav_cmd_sent = False     # 상태별 nav action 1회 송신 가드
        self._gate_open = False        # stage 경계 게이트 통과 표시
        self._nav_arrived = False      # 현재 상태의 nav 도착 event 수신 완료(per-state)
        self._departed_from_a = False  # A 출발 후(B 측) 여부 — RECOVERY 복귀 판단

        self._last_operator_event: str | None = None
        self._estop = False

        # manipulation subprocess 1건 추적
        self._manip = {'key': None, 'proc': None}

        self.timer = self.create_timer(0.1, self._tick, callback_group=cbg)
        self.get_logger().info(
            f'mission_b started state={self.state.name} stage={self.stage} '
            f'auto_chain={self.auto_chain} max_boxes={self.max_boxes}')

    # ----------------------------------------------------------------------- #
    # Helpers
    # ----------------------------------------------------------------------- #
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self) -> float:
        return self._now() - self._state_enter_time

    # --- nav event ---
    def _on_nav_event(self, msg: String) -> None:
        self._nav_event = msg.data.strip()
        self._nav_event_consumed = False
        self.get_logger().info(f'[nav_event] {self._nav_event}')

    def _peek_nav_event(self) -> str | None:
        return None if self._nav_event_consumed else self._nav_event

    def _consume_nav_event(self, name: str) -> bool:
        if not self._nav_event_consumed and self._nav_event == name:
            self._nav_event_consumed = True
            return True
        return False

    def _nav_failed(self) -> str | None:
        """미소비 nav event 가 REJECTED/FAILED 면 그 문자열 반환(소비)."""
        ev = self._peek_nav_event()
        if ev and (ev.startswith('REJECTED') or ev.startswith('FAILED')):
            self._nav_event_consumed = True
            return ev
        return None

    def _send_nav_once(self, action: str) -> None:
        if self._nav_cmd_sent:
            return
        self.pub_nav_action.publish(String(data=action))
        self._nav_cmd_sent = True
        self.get_logger().info(f'[nav] action 송신: {action}')

    # --- operator event (stage 경계 게이트) ---
    def _on_operator_event(self, msg: String) -> None:
        self._last_operator_event = msg.data.strip()
        self.get_logger().info(f'[operator_event] {self._last_operator_event}')

    def _consume_operator_event(self, name: str) -> bool:
        if self._last_operator_event == name:
            self._last_operator_event = None
            return True
        return False

    def _stage_gate(self, next_unit: str) -> bool:
        """stage=all 단계 경계: auto_chain 또는 조종자 proceed 시 True."""
        if self.stage != 'all' or self.auto_chain:
            return True
        if self._gate_open or self._consume_operator_event(self.proceed_event):
            self._gate_open = True
            return True
        self.get_logger().info(
            f'[gate] 조종자 확인 대기 — {next_unit} 진행하려면 '
            f"operator_event '{self.proceed_event}' 발행",
            throttle_duration_sec=3.0)
        return False

    def _on_estop(self, msg: Bool) -> None:
        if msg.data and not self._estop:
            self.get_logger().error('[E-STOP] 원격 비상정지 수신')
        self._estop = bool(msg.data)

    # --- manipulation subprocess ---
    def _start_manip(self, key: str, cmd_str: str) -> None:
        if self._manip['key'] == key and self._manip['proc'] is not None:
            return
        cmd = shlex.split(cmd_str)
        self.get_logger().info(f'[{key}] manipulation 실행: {cmd_str}')
        self._manip = {'key': key, 'proc': subprocess.Popen(cmd)}

    def _manip_poll(self, key: str) -> int | None:
        """실행 중이면 None, 종료되었으면 returncode."""
        m = self._manip
        if m['key'] != key or m['proc'] is None:
            return None
        return m['proc'].poll()

    def _stop_manip(self) -> None:
        proc = self._manip.get('proc')
        if proc is not None and proc.poll() is None:
            proc.terminate()
            deadline = time.monotonic() + 3.0
            while proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if proc.poll() is None:
                proc.kill()
        self._manip = {'key': None, 'proc': None}

    # ----------------------------------------------------------------------- #
    # State dispatch
    # ----------------------------------------------------------------------- #
    def _tick(self) -> None:
        self._publish_monitor()

        if self._estop and self.state != State.E_STOP:
            self._transition(State.E_STOP)
            return

        handler = {
            State.INIT:              self._run_init,
            State.B1_GRASP:          self._run_b1_grasp,
            State.B1_DEPART:         self._run_b1_depart,
            State.B2_CARRY:          self._run_b2_carry,
            State.B2_STOPLINE:       self._run_b2_stopline,
            State.B2_APPROACH_TABLE: self._run_b2_approach_table,
            State.B3_PLACE:          self._run_b3_place,
            State.B3_COMPLETE:       self._run_b3_complete,
            State.RETURN:            self._run_return,
            State.DONE_B:            self._run_done,
            State.RECOVERY:          self._run_recovery,
            State.E_STOP:            self._run_estop,
        }[self.state]
        handler()

    def _transition(self, new_state: State) -> None:
        if new_state == self.state:
            return
        self.get_logger().info(f'[state] {self.state.name} -> {new_state.name}')
        self.state = new_state
        self._state_enter_time = self._now()
        self._nav_cmd_sent = False
        self._gate_open = False
        self._nav_arrived = False
        self._on_enter(new_state)

    def _on_enter(self, state: State) -> None:
        if state == State.B1_GRASP:
            # 새 박스 사이클 시작(컨베이어 A) — per-box 플래그 + 위치 상태 리셋
            self.departure_ready = False
            self.delivery_complete = False
            self.stopline_reached = False
            self._departed_from_a = False
        elif state == State.B2_CARRY:
            # A_TO_B 로 A 를 떠난다 → 이후 실패 복구는 B_TO_A(RETURN) 경유.
            self._departed_from_a = True

    def _timed_out(self, limit: float) -> bool:
        return self._elapsed() > limit

    def _fail(self, reason: str) -> None:
        self._stop_manip()
        self.attempts += 1
        self.get_logger().warning(
            f'[{self.state.name}] FAIL({self.attempts}/{self.max_attempts}): {reason}')
        self._transition(State.RECOVERY)

    def _stage_done(self, last_state: State) -> bool:
        """단독 stage 실행 시 해당 유닛 마지막 상태에서 종료해야 하면 True."""
        return self.stage != 'all' and last_state in STAGE_UNITS.get(self.stage, ())

    # ----------------------------------------------------------------------- #
    # Per-state handlers
    # ----------------------------------------------------------------------- #
    def _run_init(self) -> None:
        self.pub_active_mission.publish(String(data='B'))

        # nav 가 필요한 stage(b2/b3/all)는 READY 를 기다린다(없으면 경고 후 강행).
        needs_nav = self.stage in ('b2', 'b3', 'all')
        if needs_nav and self._peek_nav_event() != 'READY' \
                and self._nav_event != 'READY':
            if self._elapsed() <= self.nav_ready_timeout:
                self.get_logger().info('[INIT] nav READY 대기...',
                                       throttle_duration_sec=2.0)
                return
            self.get_logger().warning('[INIT] nav READY 미수신(timeout) — 강행')
        # READY 는 게이트가 아니라 링크 확인용 → 소비.
        self._consume_nav_event('READY')

        start = {
            'b1':  State.B1_GRASP,
            'all': State.B1_GRASP,
            'b2':  State.B2_CARRY,
            'b3':  State.B2_APPROACH_TABLE,
        }[self.stage]
        self.get_logger().info(f'[INIT] stage={self.stage} 시작 -> {start.name}')
        self._transition(start)

    # --- Ⓑ-1 ---
    def _run_b1_grasp(self) -> None:
        self._start_manip('pick', self.pick_cmd)
        rc = self._manip_poll('pick')
        if rc is None:
            if self._timed_out(self.manip_timeout):
                self._stop_manip()
                self._fail('dual_pick timeout')
            return
        self._stop_manip()
        if rc == 0:
            self._transition(State.B1_DEPART)
        else:
            self._fail(f'dual_pick 실패 rc={rc}')

    def _run_b1_depart(self) -> None:
        if not self.departure_ready:
            self.departure_ready = True
            self.get_logger().info('[B1_DEPART] 출발 가능 신호 출력 (open-loop, pick 완료)')
        if self._stage_done(State.B1_DEPART):
            self._transition(State.DONE_B)
            return
        if self._stage_gate('Ⓑ-2'):
            self._transition(State.B2_CARRY)

    # --- Ⓑ-2 ---
    def _run_b2_carry(self) -> None:
        self._send_nav_once('A_TO_B')
        ev = self._nav_failed()
        if ev:
            self._fail(f'nav {ev}')
            return
        if self._consume_nav_event('REACHED_B_STOP_LINE'):
            self._transition(State.B2_STOPLINE)
        elif self._timed_out(self.nav_timeout):
            self._fail('A_TO_B timeout (REACHED_B_STOP_LINE 미수신)')

    def _run_b2_stopline(self) -> None:
        if not self.stopline_reached:
            self.stopline_reached = True
            self.get_logger().info(
                f'[B2_STOPLINE] 정지선 도착 인정 — 일시정지 {self.stop_line_dwell_sec}s (Ⓑ-2)')
        if self._elapsed() < self.stop_line_dwell_sec:
            return
        if self._stage_done(State.B2_STOPLINE):
            self._transition(State.DONE_B)
            return
        if self._stage_gate('Ⓑ-3'):
            self._transition(State.B2_APPROACH_TABLE)

    # --- Ⓑ-3 ---
    def _run_b2_approach_table(self) -> None:
        self._send_nav_once('APPROACH_B')
        ev = self._nav_failed()
        if ev:
            self._fail(f'nav {ev}')
            return
        if self._consume_nav_event('REACHED_B_PLACE_POSE'):
            self._transition(State.B3_PLACE)
        elif self._timed_out(self.nav_timeout):
            self._fail('APPROACH_B timeout (REACHED_B_PLACE_POSE 미수신)')

    def _run_b3_place(self) -> None:
        self._start_manip('place', self.place_cmd)
        rc = self._manip_poll('place')
        if rc is None:
            if self._timed_out(self.manip_timeout):
                self._stop_manip()
                self._fail('dual_place timeout')
            return
        self._stop_manip()
        if rc == 0:
            self._transition(State.B3_COMPLETE)
        else:
            self._fail(f'dual_place 실패 rc={rc}')

    def _run_b3_complete(self) -> None:
        self.box_count += 1
        self.delivery_complete = True
        self.get_logger().info(
            f'[B3_COMPLETE] 안착 완료 신호 출력 + count++ -> '
            f'box_count={self.box_count}/{self.max_boxes}')
        if self.box_count >= self.max_boxes:
            # 마지막 박스 안착 → A 로 복귀하지 않고 현재 위치(B)에서 종료.
            self.get_logger().info(
                '[B3_COMPLETE] 마지막 박스 안착 — A 복귀 생략, 현 위치에서 종료')
            self._transition(State.DONE_B)
        else:
            self._transition(State.RETURN)

    def _run_return(self) -> None:
        self._send_nav_once('B_TO_A')
        ev = self._nav_failed()
        if ev:
            self._fail(f'nav {ev}')
            return
        # 도착 전: REACHED_A 대기(+ nav 타임아웃). 도착 후에는 타임아웃을 다시 적용하지
        # 않는다(조종자 확인 게이트 대기 중 spurious timeout 방지).
        if not self._nav_arrived:
            if self._consume_nav_event('REACHED_A'):
                self._nav_arrived = True
                self._departed_from_a = False  # A 복귀 완료
            elif self._timed_out(self.nav_timeout):
                self._fail('B_TO_A timeout (REACHED_A 미수신)')
                return
            else:
                return
        # A 복귀 완료
        if self._stage_done(State.RETURN):
            self._transition(State.DONE_B)
            return
        if self.box_count >= self.max_boxes:
            self._transition(State.DONE_B)
            return
        # 다음 cycle (조종자 확인 게이트)
        if self._stage_gate('다음 박스(Ⓑ-1)'):
            self._transition(State.B1_GRASP)

    # --- 종료/복구/안전 ---
    def _run_done(self) -> None:
        self.get_logger().info(
            f'[DONE_B] Mission B(stage={self.stage}) 완료: '
            f'box_count={self.box_count}/{self.max_boxes}, attempts={self.attempts}',
            throttle_duration_sec=5.0)

    def _run_recovery(self) -> None:
        if self.stage != 'all' or self.box_count >= self.max_boxes \
                or self.attempts >= self.max_attempts:
            self.get_logger().error(
                f'[RECOVERY] 종료(box_count={self.box_count}, attempts={self.attempts}) -> DONE_B')
            self._transition(State.DONE_B)
        elif self._departed_from_a:
            # B 측에서 실패 → A 로 복귀 후 다음 박스.
            self.get_logger().warning('[RECOVERY] B 측 실패 → RETURN(A 복귀) 후 다음 박스')
            self._transition(State.RETURN)
        else:
            # A 에서 출발 전 실패(B1) → 이미 A 이므로 복귀 불필요, 다음 박스 재시도.
            self.get_logger().warning('[RECOVERY] A 출발 전 실패 → 다음 박스 재시도(Ⓑ-1)')
            self._transition(State.B1_GRASP)

    def _run_estop(self) -> None:
        # 진입 시 1회 정지 명령(중복 송신 가드).
        if not self._nav_cmd_sent:
            self.pub_nav_action.publish(String(data='STOP'))
            self._nav_cmd_sent = True
            self._stop_manip()
            self.get_logger().error('[E_STOP] nav STOP 송신 + manipulation 종료')
        self.get_logger().error('[E_STOP] 정지 유지 (해제 시 재기동 필요)',
                                throttle_duration_sec=5.0)

    # ----------------------------------------------------------------------- #
    # Monitor (심사용 신호 채널)
    # ----------------------------------------------------------------------- #
    def _publish_monitor(self) -> None:
        payload = {
            'state': self.state.name,
            'mode': self.mode,
            'stage': self.stage,
            'box_count': self.box_count,
            'max_boxes': self.max_boxes,
            'departure_ready': self.departure_ready,
            'delivery_complete': self.delivery_complete,
            'stopline_reached': self.stopline_reached,
            'departure_text': '출발 가능' if self.departure_ready else '',
            'delivery_text': '안착 완료' if self.delivery_complete else '',
            'stopline_text': '정지선 도착' if self.stopline_reached else '',
            'attempts': self.attempts,
            'ts': round(self._now(), 3),
        }
        self.pub_monitor.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionB()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_manip()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
