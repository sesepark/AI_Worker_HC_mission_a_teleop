#!/usr/bin/env python3
"""Mission A State Machine.

Mission A 자율 시나리오 상태 기계 (rclpy Node).
P0 구현: 패키지화 + task_list 로직 + state timeout + --sim 모드.
Perception 입력(`/perception/wrist/target_one_pose`, `/monitor_ocr/result`,
`/detections`)은 검증됨. Manipulation 연동(A3_PICK/PLACE Action)은 Phase 2 TODO.

Reference: humanoid_challenge/docs/MISSION_A_SCENARIO_PLAN.md "mission_a.py 초안 작성 계획"
"""
from __future__ import annotations

import json
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from mission.task_list import TaskList, CLASS_TO_PART_NAME

# perception_part_detector message package may not yet be on PYTHONPATH
# during early phases — guard the import so the node still starts.
try:
    from perception_part_detector.msg import PartDetectionArray
except ImportError:
    PartDetectionArray = None  # type: ignore[assignment]


# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #
GRASP_ASSESSMENT_ENABLED = False  # flip to True after Hand-Eye Calibration

# Timeout (seconds)
TIMEOUT_INIT       = 60
TIMEOUT_A1_MONITOR = 90
TIMEOUT_A1_OCR     = 20
TIMEOUT_A2_SCAN    = 90
TIMEOUT_PICK_PLACE = 45
TIMEOUT_VERIFY     = 20
FALLBACK_OK_DELAY  = 10   # OCR 실패 시 강제 OK 딜레이

MAX_RECOVERY_RETRY = 3


# --------------------------------------------------------------------------- #
# State enum
# --------------------------------------------------------------------------- #
class State(Enum):
    INIT        = auto()
    A1_MONITOR  = auto()
    A2_SCAN     = auto()
    A3_PICK     = auto()
    A3_PLACE    = auto()
    VERIFY      = auto()
    DONE        = auto()
    RECOVERY    = auto()
    MANUAL_WAIT = auto()


# state 별 timeout 매핑 (없으면 무제한)
STATE_TIMEOUT: dict[State, float] = {
    State.INIT:       TIMEOUT_INIT,
    State.A1_MONITOR: TIMEOUT_A1_MONITOR,
    State.A2_SCAN:    TIMEOUT_A2_SCAN,
    State.A3_PICK:    TIMEOUT_PICK_PLACE,
    State.A3_PLACE:   TIMEOUT_PICK_PLACE,
    State.VERIFY:     TIMEOUT_VERIFY,
}


# --------------------------------------------------------------------------- #
# Node
# --------------------------------------------------------------------------- #
class MissionA(Node):
    def __init__(self) -> None:
        super().__init__('mission_a')

        # --- Parameters ---
        self.sim_mode = bool(
            self.declare_parameter('sim_mode', False).value)

        # --- Subscribers ---
        self.sub_manipulator_state = self.create_subscription(
            String, '/manipulator_state', self._on_manipulator_state, 10)
        self.sub_monitor_ocr = self.create_subscription(
            String, '/monitor_ocr/result', self._on_monitor_ocr, 10)
        if PartDetectionArray is not None:
            self.sub_detections = self.create_subscription(
                PartDetectionArray, '/detections', self._on_detections, 10)
        else:
            self.get_logger().warning(
                'perception_part_detector.msg not importable — '
                '/detections subscription disabled. Build the message package '
                'and re-source the workspace.')
            self.sub_detections = None
        # A2_SCAN/A3_PICK target — wrist_task_grasp_planner_node 의 최종 1개 출력.
        self.sub_target_pose = self.create_subscription(
            PoseStamped, '/perception/wrist/target_one_pose',
            self._on_target_pose, 10)
        self.sub_attached_object = self.create_subscription(
            String, '/attached_object', self._on_attached_object, 10)
        # Perception task_management 의 잔여 task list (OCR목표 − 트레이관측).
        # 발행되면 이게 task 진행의 source of truth → OCR 자체파싱/차감 불필요.
        self.sub_task_list = self.create_subscription(
            String, '/perception/task_list', self._on_task_list, 10)

        # --- Publishers ---
        self.pub_active_mission = self.create_publisher(
            String, '/active_mission', 10)
        self.pub_attach_cmd = self.create_publisher(
            String, '/attach_cmd', 10)
        self.pub_detach_cmd = self.create_publisher(
            String, '/detach_cmd', 10)

        # --- State storage ---
        self.state: State = State.INIT
        self.recovery_count: int = 0
        self.cycle: int = 0                 # scan→pick→place 루프 카운터 (sim 키)
        self._state_enter_time: float = self._now()
        self._fallback_done: bool = False   # A1 강제 OK 1회 표식

        # Latest topic snapshots (None until first message)
        self.last_manipulator_state: str | None = None
        self.last_ocr_result: dict | None = None
        self.last_detections = None
        self.last_target_pose: PoseStamped | None = None
        self.last_attached_object: str | None = None

        # 미션 진행 상태
        self.task_list: TaskList = TaskList()
        self.current_target_pose: PoseStamped | None = None
        self.current_pick_class: str | None = None
        # /perception/task_list 가 한 번이라도 오면 perception 이 task 상태를 소유.
        self._perception_owns_tasklist: bool = False
        self._tasklist_baseline: int = 0   # VERIFY 시 차감 확인용 스냅샷

        # --- Sim driver (optional) ---
        self._sim = None
        if self.sim_mode:
            from mission.sim_driver import SimDriver
            self._sim = SimDriver(self, State)

        # State tick at 10 Hz
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'mission_a started in state={self.state.name} '
            f'(sim_mode={self.sim_mode}, '
            f'GRASP_ASSESSMENT_ENABLED={GRASP_ASSESSMENT_ENABLED})')

    # ----------------------------------------------------------------------- #
    # Helpers
    # ----------------------------------------------------------------------- #
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self) -> float:
        return self._now() - self._state_enter_time

    def _timed_out(self) -> bool:
        limit = STATE_TIMEOUT.get(self.state)
        return limit is not None and self._elapsed() > limit

    # --- Subscription callbacks: store only, decisions happen in _tick ---
    def _on_manipulator_state(self, msg: String) -> None:
        self.last_manipulator_state = msg.data
        self.get_logger().debug(f'[sub] /manipulator_state = {msg.data}')

    def _on_monitor_ocr(self, msg: String) -> None:
        try:
            self.last_ocr_result = json.loads(msg.data)
            self.get_logger().info(
                '[sub] /monitor_ocr/result received '
                f'(screen_detected={self.last_ocr_result.get("latest_screen_detected")})')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'/monitor_ocr/result JSON decode failed: {e}')

    def _on_detections(self, msg) -> None:
        self.last_detections = msg
        self.get_logger().debug('[sub] /detections received')

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self.last_target_pose = msg
        self.get_logger().debug('[sub] /perception/wrist/target_one_pose received')

    def _on_attached_object(self, msg: String) -> None:
        self.last_attached_object = msg.data
        self.get_logger().debug(f'[sub] /attached_object = "{msg.data}"')

    def _on_task_list(self, msg: String) -> None:
        """Perception 잔여 task list 수신 → task_list 갱신 (source of truth)."""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'/perception/task_list JSON decode failed: {e}')
            return
        self.task_list.build_from_task_list_payload(payload.get('parts', []))
        if not self._perception_owns_tasklist:
            self._perception_owns_tasklist = True
            self.get_logger().info(
                '[sub] /perception/task_list 수신 — perception 이 task 상태 소유 '
                f'(OCR 자체파싱 비활성). {self.task_list}')

    # ----------------------------------------------------------------------- #
    # State dispatch
    # ----------------------------------------------------------------------- #
    def _tick(self) -> None:
        handler = {
            State.INIT:        self._run_init,
            State.A1_MONITOR:  self._run_a1_monitor,
            State.A2_SCAN:     self._run_a2_scan,
            State.A3_PICK:     self._run_a3_pick,
            State.A3_PLACE:    self._run_a3_place,
            State.VERIFY:      self._run_verify,
            State.DONE:        self._run_done,
            State.RECOVERY:    self._run_recovery,
            State.MANUAL_WAIT: self._run_manual_wait,
        }[self.state]
        handler()

    def _transition(self, new_state: State) -> None:
        if new_state == self.state:
            return
        self.get_logger().info(f'[state] {self.state.name} -> {new_state.name}')
        self.state = new_state
        self._state_enter_time = self._now()
        self._on_enter(new_state)

    def _on_enter(self, state: State) -> None:
        """state 진입 시 per-cycle 변수 리셋."""
        if state == State.A2_SCAN:
            # 새 scan 사이클 시작 — 이전 target/attached 흔적 제거 (consume-once)
            self.cycle += 1
            self.last_target_pose = None
            self.current_target_pose = None
            self.last_attached_object = None
            self.current_pick_class = None

    # ----------------------------------------------------------------------- #
    # Per-state handlers
    # ----------------------------------------------------------------------- #
    def _run_init(self) -> None:
        # TODO(Phase2): verify head/wrist camera timestamps; MoveIt scene 등록.
        self.pub_active_mission.publish(String(data='A'))
        if self.last_manipulator_state == 'IDLE':
            self.get_logger().info('[INIT] manipulator IDLE 확인 -> A1_MONITOR')
            self._transition(State.A1_MONITOR)
        elif self._timed_out():
            self.get_logger().warning(
                '[INIT] manipulator IDLE 미수신 (timeout) -> A1_MONITOR 강행')
            self._transition(State.A1_MONITOR)

    def _run_a1_monitor(self) -> None:
        # 1순위: perception /perception/task_list (트레이 비전 자동 차감, source of truth)
        if self._perception_owns_tasklist and not self.task_list.is_empty():
            total = self.task_list.total_remaining()
            # TODO(Phase1): 모니터에 OK 사인 출력
            if total > 0:
                self.get_logger().info(
                    f'[A1_MONITOR] perception task_list 확정: {self.task_list} '
                    f'(총 {total}) -> A2_SCAN')
                self._transition(State.A2_SCAN)
            else:
                self.get_logger().info('[A1_MONITOR] perception task_list 잔여 0 -> VERIFY')
                self._transition(State.VERIFY)
            return

        # 2순위(폴백): OCR 직접 파싱 (management_node 미가동 시)
        if self.last_ocr_result and self.last_ocr_result.get('latest_screen_detected'):
            parts = self.last_ocr_result.get('parts', [])
            self.task_list.build_from_ocr_parts(parts)
            if not self.task_list.is_empty():
                self.get_logger().info(
                    f'[A1_MONITOR] (OCR 폴백) task_list: {self.task_list} '
                    f'(총 {self.task_list.total_remaining()}) -> A2_SCAN')
                self._transition(State.A2_SCAN)
                return

        # 폴백: FALLBACK_OK_DELAY 경과 시 강제 OK (점수 10점 확보)
        if not self._fallback_done and self._elapsed() >= FALLBACK_OK_DELAY:
            self._fallback_done = True
            self.get_logger().warning(
                f'[A1_MONITOR] OCR {FALLBACK_OK_DELAY}s 내 미파싱 -> '
                '강제 OK 폴백 (task_list 비어있음)')
            # TODO(Phase1): 강제 OK 사인 출력. 빈 task_list 면 VERIFY 에서 즉시 DONE.
            self._transition(State.A2_SCAN)

    def _run_a2_scan(self) -> None:
        # planner 가 task 필터링까지 수행 → 최종 1개 target 수신 대기
        if self.last_target_pose is not None:
            frame = self.last_target_pose.header.frame_id
            if frame != 'base_link':
                self.get_logger().warning(
                    f'[A2_SCAN] target frame_id={frame!r} (base_link 아님) — 무시')
                self.last_target_pose = None
                return
            self.current_target_pose = self.last_target_pose
            self.last_target_pose = None  # consume
            # VERIFY 에서 트레이 차감 확인용 잔여 스냅샷
            self._tasklist_baseline = self.task_list.total_remaining()
            p = self.current_target_pose.pose.position
            self.get_logger().info(
                f'[A2_SCAN] target 수신 ({p.x:.3f},{p.y:.3f},{p.z:.3f}) -> A3_PICK')
            self._transition(State.A3_PICK)
            return

        # task 가 비었으면 (폴백 경로) 스캔할 것 없음 → VERIFY 로 보내 DONE 처리
        if self.task_list.is_empty():
            self.get_logger().info('[A2_SCAN] task_list 비어있음 -> VERIFY (완료 판정)')
            self._transition(State.VERIFY)
            return

        if self._timed_out():
            self.get_logger().warning('[A2_SCAN] target 미수신 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_a3_pick(self) -> None:
        # TODO(Phase2): bin_pick Action 호출 (goal=current_target_pose).
        #   Calib 전 우회: GRASP_ASSESSMENT_ENABLED=False 면 /attach_cmd 수동 attach.
        if self.last_attached_object:
            self.current_pick_class = self.last_attached_object
            self.get_logger().info(
                f'[A3_PICK] 파지 성공 attached="{self.current_pick_class}" -> A3_PLACE')
            self._transition(State.A3_PLACE)
        elif self._timed_out():
            self.get_logger().warning('[A3_PICK] 파지 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_a3_place(self) -> None:
        # TODO(Phase2): tray_place Action 호출 (/tray_region 기반 place pose).
        if self.last_attached_object == '':
            self.get_logger().info('[A3_PLACE] place 완료 (attached 비움) -> VERIFY')
            self._transition(State.VERIFY)
        elif self._timed_out():
            self.get_logger().warning('[A3_PLACE] place timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_verify(self) -> None:
        # perception 소유: 트레이 비전이 차감 → /perception/task_list 가 줄었는지 확인
        if self._perception_owns_tasklist:
            cur = self.task_list.total_remaining()
            if cur < self._tasklist_baseline:
                self.get_logger().info(
                    f'[VERIFY] 트레이 적재 검증 (잔여 {self._tasklist_baseline}→{cur})')
            elif not self._timed_out():
                return  # 아직 트레이에 안 잡힘 → 대기
            else:
                self.get_logger().warning('[VERIFY] 차감 미확인 timeout — 진행')
            if cur > 0:
                self.get_logger().info('[VERIFY] 잔여 > 0 -> A2_SCAN 복귀')
                self._transition(State.A2_SCAN)
            else:
                self.get_logger().info('[VERIFY] 잔여 0 -> DONE')
                self._transition(State.DONE)
            return

        # 레거시(OCR 직접 / sim-without-mgmt): 자체 차감 (성공 가정)
        if self.current_pick_class:
            left = self.task_list.decrement(self.current_pick_class)
            kor = CLASS_TO_PART_NAME.get(self.current_pick_class, self.current_pick_class)
            self.get_logger().info(
                f'[VERIFY] {kor} 적재 성공 가정 → 잔여 {left} '
                f'(총 {self.task_list.total_remaining()})')
            self.current_pick_class = None

        if self.task_list.total_remaining() > 0:
            self.get_logger().info('[VERIFY] 잔여 > 0 -> A2_SCAN 복귀')
            self._transition(State.A2_SCAN)
        else:
            self.get_logger().info('[VERIFY] 잔여 0 -> DONE')
            self._transition(State.DONE)

    def _run_done(self) -> None:
        self.get_logger().info('[DONE] mission A 완료')
        self.timer.cancel()

    def _run_recovery(self) -> None:
        if self.recovery_count < MAX_RECOVERY_RETRY:
            self.recovery_count += 1
            self.get_logger().warning(
                f'[RECOVERY] 재시도 {self.recovery_count}/{MAX_RECOVERY_RETRY} -> A2_SCAN')
            self._transition(State.A2_SCAN)
        else:
            self.get_logger().error('[RECOVERY] 재시도 초과 -> MANUAL_WAIT')
            self._transition(State.MANUAL_WAIT)

    def _run_manual_wait(self) -> None:
        # TODO(Phase2): 운용자 재개 신호 수신 시 -> A2_SCAN.
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
