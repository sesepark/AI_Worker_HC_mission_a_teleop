#!/usr/bin/env python3
"""Mission A State Machine.

Mission A 자율 시나리오 상태 기계 (rclpy Node).
P0 구현: 패키지화 + task_list 로직 + state timeout + --sim 모드.
Perception 입력(`/perception/wrist/target_one_pose`, `/detections`)은 검증됨.
모니터 OCR task list는 `/mission_a/task_list` 서비스로만 받는다.
Manipulation 연동(A3_PICK/PLACE Action)은 Phase 2 TODO.

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
from mission_interfaces.srv import GetTaskList
from perception.msg import PartDetectionArray


# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #
GRASP_ASSESSMENT_ENABLED = False  # flip to True after Hand-Eye Calibration

# Timeout (seconds)
TIMEOUT_INIT       = 60
TIMEOUT_A1_MONITOR = 90
TIMEOUT_A1_OCR     = 20.0
TIMEOUT_A2_SCAN    = 90
TIMEOUT_PICK_PLACE = 45
TIMEOUT_VERIFY     = 20

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
        self.task_list_service_name = str(
            self.declare_parameter('task_list_service_name', '/mission_a/task_list').value)
        self.task_list_service_timeout_sec = float(
            self.declare_parameter('task_list_service_timeout_sec', float(TIMEOUT_A1_OCR)).value)
        self.task_list_service_frame_count = int(
            self.declare_parameter('task_list_service_frame_count', 3).value)
        # 검증된 토픽 파이프라인: management_node → /perception/task_list (std_msgs/String JSON).
        self.task_list_topic = str(
            self.declare_parameter('task_list_topic', '/perception/task_list').value)
        # 기본은 토픽 구독. True 면 (구) GetTaskList 서비스 능동요청 경로 병행.
        self.use_task_list_service = bool(
            self.declare_parameter('use_task_list_service', False).value)
        # VERIFY 완료 판정을 토픽 잔량으로 할지(순수 토픽) 여부. 기본은 하이브리드(자체 차감).
        self.verify_use_topic_remaining = bool(
            self.declare_parameter('verify_use_topic_remaining', False).value)

        # --- Subscribers ---
        self.sub_manipulator_state = self.create_subscription(
            String, '/manipulator_state', self._on_manipulator_state, 10)
        self.sub_detections = self.create_subscription(
            PartDetectionArray, '/detections', self._on_detections, 10)
        # A2_SCAN/A3_PICK target — wrist_task_grasp_planner_node 의 최종 1개 출력.
        self.sub_target_pose = self.create_subscription(
            PoseStamped, '/perception/wrist/target_one_pose',
            self._on_target_pose, 10)
        self.sub_attached_object = self.create_subscription(
            String, '/attached_object', self._on_attached_object, 10)
        # task list 입력 = management_node 토픽(검증된 파이프라인).
        self.sub_task_list = self.create_subscription(
            String, self.task_list_topic, self._on_task_list, 10)

        # --- Service clients (보존: use_task_list_service=True 시 병행 사용) ---
        self.task_list_client = self.create_client(
            GetTaskList, self.task_list_service_name)

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

        # Latest topic snapshots (None until first message)
        self.last_manipulator_state: str | None = None
        self.last_detections = None
        self.last_target_pose: PoseStamped | None = None
        self.last_attached_object: str | None = None
        # /perception/task_list 토픽 최신 스냅샷 (VERIFY 교차확인용)
        self.last_task_list_payload: dict | None = None
        self._last_topic_remaining: int | None = None

        # 미션 진행 상태
        self.task_list: TaskList = TaskList()
        self.current_target_pose: PoseStamped | None = None
        self.current_pick_class: str | None = None
        self._task_list_service_inflight: bool = False
        self._task_list_service_next_try_time: float = 0.0

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
        """management_node /perception/task_list (String JSON) 구독.

        JSON: {"parts": [{"name": "flange nut", "count": 1}, ...], ...}
        - INIT/A1_MONITOR 단계에서만 목표 task_list 를 (재)빌드한다.
          (A2 이후의 자체 차감 상태를 토픽이 덮어쓰지 않도록 보호)
        - 모든 단계에서 토픽 잔량 스냅샷은 VERIFY 교차확인용으로 갱신.
        """
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to parse {self.task_list_topic} JSON: {exc}')
            return

        parts = [
            {'name': item.get('name', ''), 'count': item.get('count', 0)}
            for item in data.get('parts', [])
            if isinstance(item, dict)
        ]
        self.last_task_list_payload = data
        self._last_topic_remaining = sum(
            max(int(p.get('count', 0) or 0), 0) for p in parts)

        if self.state in (State.INIT, State.A1_MONITOR):
            self.task_list.build_from_ocr_parts(parts)
            self.get_logger().debug(
                f'[sub] {self.task_list_topic} -> {self.task_list}')

    def _request_task_list_service(self) -> None:
        if self._task_list_service_inflight or self._now() < self._task_list_service_next_try_time:
            return

        if not self.task_list_client.service_is_ready():
            self.get_logger().warn(
                f'task_list service not ready: {self.task_list_service_name}',
                throttle_duration_sec=5.0)
            self._task_list_service_next_try_time = self._now() + 1.0
            return

        request = GetTaskList.Request()
        request.timeout_sec = float(self.task_list_service_timeout_sec)
        request.frame_count = int(self.task_list_service_frame_count)

        self._task_list_service_inflight = True
        self._task_list_service_next_try_time = self._now() + max(1.0, request.timeout_sec)
        self.get_logger().info(
            f'[A1_MONITOR] task_list service 요청: {self.task_list_service_name}')

        future = self.task_list_client.call_async(request)
        future.add_done_callback(self._on_task_list_service_result)

    def _on_task_list_service_result(self, future) -> None:
        self._task_list_service_inflight = False

        try:
            response = future.result()
        except Exception as exc:
            self._task_list_service_next_try_time = self._now() + 2.0
            self.get_logger().warn(f'task_list service failed: {exc}')
            return

        self._task_list_service_next_try_time = self._now() + 2.0
        if not response.success:
            self.get_logger().warn(f'task_list service failed: {response.message}')
            return

        parts = [
            {'name': item.name, 'count': item.count}
            for item in response.parts
        ]
        self.task_list.build_from_ocr_parts(parts)
        self.get_logger().info(
            f'[A1_MONITOR] task_list service result: {self.task_list} '
            f'(frames={response.frames_used})')

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
        if not self.task_list.is_empty():
            total = self.task_list.total_remaining()
            if total > 0:
                self.get_logger().info(
                    f'[A1_MONITOR] task_list 확정: {self.task_list} '
                    f'(총 {total}) -> A2_SCAN')
                self._transition(State.A2_SCAN)
            else:
                self.get_logger().info('[A1_MONITOR] task_list 잔여 0 -> VERIFY')
                self._transition(State.VERIFY)
            return

        # 기본: /perception/task_list 토픽이 _on_task_list 에서 task_list 를 채운다(수동 대기).
        # use_task_list_service=True 면 (구) GetTaskList 서비스 능동요청 경로 병행.
        if self.use_task_list_service:
            self._request_task_list_service()

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
            p = self.current_target_pose.pose.position
            self.get_logger().info(
                f'[A2_SCAN] target 수신 ({p.x:.3f},{p.y:.3f},{p.z:.3f}) -> A3_PICK')
            self._transition(State.A3_PICK)
            return

        # task 가 비었으면 스캔할 것 없음 → VERIFY 로 보내 DONE 처리
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
        # 하이브리드: 자체 차감(local decrement)을 완료 구동축으로 유지(트레이 비전 없이도 완료).
        # /perception/task_list(management) 잔량은 교차확인·로그용.
        # verify_use_topic_remaining=True 면 토픽 잔량으로 완료 판정(트레이 실검증 후 전환).
        if self.current_pick_class:
            left = self.task_list.decrement(self.current_pick_class)
            kor = CLASS_TO_PART_NAME.get(self.current_pick_class, self.current_pick_class)
            self.get_logger().info(
                f'[VERIFY] {kor} 적재 성공 가정 → 잔여 {left} '
                f'(총 {self.task_list.total_remaining()}, '
                f'topic_remaining={self._last_topic_remaining})')
            self.current_pick_class = None

        if self.verify_use_topic_remaining and self._last_topic_remaining is not None:
            remaining = self._last_topic_remaining
        else:
            remaining = self.task_list.total_remaining()

        if remaining > 0:
            self.get_logger().info(f'[VERIFY] 잔여 {remaining} > 0 -> A2_SCAN 복귀')
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
