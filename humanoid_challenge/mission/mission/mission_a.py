#!/usr/bin/env python3
"""Mission A State Machine.

Mission A 자율 시나리오 상태 기계 (rclpy Node).
SDR v2.3 반영:
  - A2_SCAN_POSE (신규): manipulation 스캔 초기 포즈 형성(MoveToScanPose **Action**, 미구현→mock).
  - A3_MOVE_TO_TRAY / A3_RETURN_TO_BOX (신규): 모바일 베이스 측방 이동(좌/우 base_shift_mm).
      navigation 연동 = **Service `MoveBaseLateral.srv`** (Action 의 다중 엔드포인트가 통합
      디스커버리 병목을 유발 → Service 로 단순화). `nav_mode={stub|service}`(기본 stub)로 단계화.
      carry 자세는 manipulation 내부이며 FSM 은 base 이동만.
  - place 시점 차감 게이트: 차감을 VERIFY→A3_PLACE 로 이관(C1 pick class ∧ C2 grip 유지 ∧
      C3 트레이 place 위치 유효[guard]).

DDS 위생(병목 재발 방지): ActionClient(scan)는 1개뿐(통합 검증 정상). 클라이언트를 서브보다
  먼저 생성하고 INIT 에서 사전 디스커버리 게이트로 서버 준비를 확인한 뒤 진행한다.

검증된 토픽 파이프라인: management_node → /perception/task_list (std_msgs/String JSON).
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from mission.task_list import TaskList, CLASS_TO_PART_NAME
from mission_interfaces.srv import GetTaskList, MoveBaseLateral
from mission_interfaces.action import MoveToScanPose
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
    INIT             = auto()
    A1_MONITOR       = auto()
    A2_SCAN_POSE     = auto()   # 신규: manipulation 스캔 초기 포즈 형성(Action)
    A2_SCAN          = auto()
    A3_PICK          = auto()
    A3_MOVE_TO_TRAY  = auto()   # 신규: 베이스 좌 675mm (nav Service/stub)
    A3_PLACE         = auto()
    A3_RETURN_TO_BOX = auto()   # 신규: 베이스 우 675mm (nav Service/stub)
    VERIFY           = auto()
    DONE             = auto()
    RECOVERY         = auto()
    MANUAL_WAIT      = auto()


# state 별 (정적) timeout. 신규 상태(스캔/이동)는 파라미터로 __init__ 에서 합성.
STATE_TIMEOUT: dict[State, float] = {
    State.INIT:       TIMEOUT_INIT,
    State.A1_MONITOR: TIMEOUT_A1_MONITOR,
    State.A2_SCAN:    TIMEOUT_A2_SCAN,
    State.A3_PICK:    TIMEOUT_PICK_PLACE,
    State.A3_PLACE:   TIMEOUT_PICK_PLACE,
    State.VERIFY:     TIMEOUT_VERIFY,
}


# --------------------------------------------------------------------------- #
# Async latch — send-once-on-entry, poll-in-tick (scan action / nav service 공용)
# --------------------------------------------------------------------------- #
@dataclass
class AsyncLatch:
    """비동기 호출 1회 송신/결과 폴링용 래치.

    callback 은 result 를 먼저 채우고 done 을 마지막에 True 로(원자성), tick 은 done 만 검사.
    상태 진입(_on_enter)에서 reset() 으로 재무장.
    """
    sent: bool = False
    goal_handle: object = None
    result: object = None
    done: bool = False

    def reset(self) -> None:
        self.sent = False
        self.goal_handle = None
        self.result = None
        self.done = False


# --------------------------------------------------------------------------- #
# Node
# --------------------------------------------------------------------------- #
class MissionA(Node):
    def __init__(self) -> None:
        super().__init__('mission_a')

        # --- Parameters (기존) ---
        self.sim_mode = bool(
            self.declare_parameter('sim_mode', False).value)
        self.task_list_service_name = str(
            self.declare_parameter('task_list_service_name', '/mission_a/task_list').value)
        self.task_list_service_timeout_sec = float(
            self.declare_parameter('task_list_service_timeout_sec', float(TIMEOUT_A1_OCR)).value)
        self.task_list_service_frame_count = int(
            self.declare_parameter('task_list_service_frame_count', 3).value)
        self.task_list_topic = str(
            self.declare_parameter('task_list_topic', '/perception/task_list').value)
        self.use_task_list_service = bool(
            self.declare_parameter('use_task_list_service', False).value)
        self.verify_use_topic_remaining = bool(
            self.declare_parameter('verify_use_topic_remaining', False).value)

        # --- Parameters (v2.3 신규) ---
        # navigation 연동 모드. stub=외부호출 없이 instant success(로봇/실서비스 없이 전구간 검증),
        #   service=MoveBaseLateral.srv 실제 호출.
        self.nav_mode = str(self.declare_parameter('nav_mode', 'stub').value).strip().lower()
        if self.nav_mode not in ('stub', 'service'):
            self.get_logger().warn(f"nav_mode={self.nav_mode!r} 미지원 → 'stub' 사용")
            self.nav_mode = 'stub'
        self.base_shift_mm = float(self.declare_parameter('base_shift_mm', 675.0).value)
        self.scan_pose_preset_id = str(self.declare_parameter('scan_pose_preset_id', '').value)
        self.scan_pose_timeout_sec = float(
            self.declare_parameter('scan_pose_timeout_sec', 30.0).value)
        self.base_move_timeout_sec = float(
            self.declare_parameter('base_move_timeout_sec', 30.0).value)
        self.use_place_pose_check = bool(
            self.declare_parameter('use_place_pose_check', False).value)
        self.place_pose_valid_debounce_sec = float(
            self.declare_parameter('place_pose_valid_debounce_sec', 0.3).value)
        self.rescan_each_cycle = bool(
            self.declare_parameter('rescan_each_cycle', True).value)
        self.nav_service_name = str(
            self.declare_parameter('nav_service_name', 'move_base_lateral').value)
        self.scan_action_name = str(
            self.declare_parameter('scan_action_name', 'move_to_scan_pose').value)

        # 신규 상태 timeout(파라미터 기반)을 인스턴스 맵에 합성.
        self.state_timeout: dict[State, float] = dict(STATE_TIMEOUT)
        self.state_timeout[State.A2_SCAN_POSE]     = self.scan_pose_timeout_sec
        self.state_timeout[State.A3_MOVE_TO_TRAY]  = self.base_move_timeout_sec
        self.state_timeout[State.A3_RETURN_TO_BOX] = self.base_move_timeout_sec

        # --- Callback group ---
        # 모든 콜백(timer/scan action/nav service/subs)을 단일 Reentrant 그룹에.
        #   ActionClient 는 scan 1개뿐 → v2.2 의 2번째 액션 EDP 굶주림(병목) 원천 제거.
        self._cbg = ReentrantCallbackGroup()

        # --- 외부 기능 클라이언트 (DDS 위생: 서브보다 먼저 생성) ---
        self._scan_cli = ActionClient(
            self, MoveToScanPose, self.scan_action_name, callback_group=self._cbg)
        self._nav_cli = self.create_client(
            MoveBaseLateral, self.nav_service_name, callback_group=self._cbg)

        # --- Subscribers ---
        self.sub_manipulator_state = self.create_subscription(
            String, '/manipulator_state', self._on_manipulator_state, 10,
            callback_group=self._cbg)
        self.sub_detections = self.create_subscription(
            PartDetectionArray, '/detections', self._on_detections, 10,
            callback_group=self._cbg)
        self.sub_target_pose = self.create_subscription(
            PoseStamped, '/perception/wrist/target_one_pose',
            self._on_target_pose, 10, callback_group=self._cbg)
        self.sub_attached_object = self.create_subscription(
            String, '/attached_object', self._on_attached_object, 10,
            callback_group=self._cbg)
        self.sub_task_list = self.create_subscription(
            String, self.task_list_topic, self._on_task_list, 10,
            callback_group=self._cbg)
        # C3: 트레이 place 위치 유효성(perception 신규, guard). std_msgs/String JSON.
        self.sub_place_pose_valid = self.create_subscription(
            String, '/perception/place_pose_valid', self._on_place_pose_valid, 10,
            callback_group=self._cbg)

        # --- Service clients (보존: use_task_list_service=True 시 병행 사용) ---
        self.task_list_client = self.create_client(
            GetTaskList, self.task_list_service_name, callback_group=self._cbg)

        # --- Publishers ---
        self.pub_active_mission = self.create_publisher(String, '/active_mission', 10)
        self.pub_attach_cmd = self.create_publisher(String, '/attach_cmd', 10)
        self.pub_detach_cmd = self.create_publisher(String, '/detach_cmd', 10)

        # --- State storage ---
        self.state: State = State.INIT
        self.recovery_count: int = 0
        self.cycle: int = 0
        self.placed_count: int = 0
        self._state_enter_time: float = self._now()

        self.last_manipulator_state: str | None = None
        self.last_detections = None
        self.last_target_pose: PoseStamped | None = None
        self.last_attached_object: str | None = None
        self.last_task_list_payload: dict | None = None
        self._last_topic_remaining: int | None = None
        self._place_pose_valid: dict | None = None
        self._place_valid_since: float | None = None

        self.task_list: TaskList = TaskList()
        self.current_target_pose: PoseStamped | None = None
        self.current_pick_class: str | None = None
        self._task_list_service_inflight: bool = False
        self._task_list_service_next_try_time: float = 0.0

        # 비동기 래치 (상태 진입 시 reset)
        self._scan = AsyncLatch()
        self._move_tray = AsyncLatch()
        self._return = AsyncLatch()
        self._release_issued: bool = False

        # --- Sim driver (optional) ---
        self._sim = None
        if self.sim_mode:
            from mission.sim_driver import SimDriver
            self._sim = SimDriver(self, State)

        self.timer = self.create_timer(0.1, self._tick, callback_group=self._cbg)
        self.get_logger().info(
            f'mission_a started in state={self.state.name} '
            f'(sim_mode={self.sim_mode}, nav_mode={self.nav_mode}, '
            f'base_shift_mm={self.base_shift_mm}, use_place_pose_check={self.use_place_pose_check}, '
            f'GRASP_ASSESSMENT_ENABLED={GRASP_ASSESSMENT_ENABLED})')

    # ----------------------------------------------------------------------- #
    # Helpers
    # ----------------------------------------------------------------------- #
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self) -> float:
        return self._now() - self._state_enter_time

    def _timed_out(self) -> bool:
        limit = self.state_timeout.get(self.state)
        return limit is not None and self._elapsed() > limit

    # --- Subscription callbacks: store only ---
    def _on_manipulator_state(self, msg: String) -> None:
        self.last_manipulator_state = msg.data

    def _on_detections(self, msg) -> None:
        self.last_detections = msg

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self.last_target_pose = msg

    def _on_attached_object(self, msg: String) -> None:
        self.last_attached_object = msg.data

    def _on_place_pose_valid(self, msg: String) -> None:
        """C3: /perception/place_pose_valid (String JSON) — store only + 디바운스 추적."""
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        valid = bool(data.get('valid', False))
        now = self._now()
        if valid:
            if self._place_valid_since is None:
                self._place_valid_since = now
        else:
            self._place_valid_since = None
        data['recv_time'] = now
        self._place_pose_valid = data

    def _place_pose_valid_now(self) -> bool:
        """C3 판정: 최신 valid && 신선(<=1s) && 디바운스 경과."""
        snap = self._place_pose_valid
        if snap is None or not snap.get('valid', False):
            return False
        now = self._now()
        if now - float(snap.get('recv_time', 0.0)) > 1.0:
            return False
        since = self._place_valid_since
        if since is None:
            return False
        return (now - since) >= self.place_pose_valid_debounce_sec

    def _on_task_list(self, msg: String) -> None:
        """management_node /perception/task_list (String JSON) 구독."""
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
        parts = [{'name': item.name, 'count': item.count} for item in response.parts]
        self.task_list.build_from_ocr_parts(parts)
        self.get_logger().info(
            f'[A1_MONITOR] task_list service result: {self.task_list} '
            f'(frames={response.frames_used})')

    # ----------------------------------------------------------------------- #
    # 외부 기능 호출 (scan Action / nav Service)
    # ----------------------------------------------------------------------- #
    def _servers_ready(self) -> bool:
        """DDS 위생(INIT 사전 게이트): 다음에 곧바로 필요한 의존성만 확인한다.

        - scan action 서버: 직후 A2_SCAN_POSE 에서 사용 → INIT 에서 워밍·확인.
        - nav 서비스: A3_MOVE_TO_TRAY(한 사이클 뒤)에서 사용 → 그 핸들러가 자체 대기.
          (동시 기동 시 nav 서비스 디스커버리 지연으로 INIT 이 막히지 않도록 게이트에서 제외.)
        """
        if not self._scan_cli.server_is_ready():
            self._scan_cli.wait_for_server(timeout_sec=0.0)  # 그래프 nudge
            if not self._scan_cli.server_is_ready():
                return False
        return True

    def _on_scan_goal_response(self, future) -> None:
        try:
            gh = future.result()
        except Exception as exc:
            self.get_logger().warn(f'[A2_SCAN_POSE] goal 응답 예외: {exc}')
            self._scan.result = None
            self._scan.done = True
            return
        if not gh.accepted:
            self.get_logger().warn('[A2_SCAN_POSE] goal 거부됨')
            self._scan.result = None
            self._scan.done = True
            return
        self._scan.goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_scan_result)

    def _on_scan_result(self, future) -> None:
        try:
            self._scan.result = future.result().result   # store FIRST
        except Exception as exc:
            self.get_logger().warn(f'[A2_SCAN_POSE] result 예외: {exc}')
            self._scan.result = None
        self._scan.done = True                            # latch LAST

    def _on_nav_result(self, latch: AsyncLatch, future) -> None:
        try:
            latch.result = future.result()   # store FIRST
        except Exception as exc:
            self.get_logger().warn(f'nav service 예외: {exc}')
            latch.result = None
        latch.done = True                    # latch LAST

    def _nav_step(self, latch: AsyncLatch, direction: str, label: str) -> str:
        """베이스 측방 이동 1스텝. 반환: 'arrived' | 'pending' | 'failed'.

        stub/sim: 외부 호출 없이 즉시 arrived. service: MoveBaseLateral.srv 호출 후 폴링.
        """
        if self.sim_mode or self.nav_mode == 'stub':
            if not latch.sent:
                latch.sent = True
                self.get_logger().info(
                    f'[{label}] nav stub instant success ({direction} {self.base_shift_mm:.0f}mm)')
            return 'arrived'
        # service 모드: wait_for_service 로 디스커버리를 능동 구동(ros2 service call 과 동일).
        #   단일 Reentrant 그룹+MTE 이므로 한 스레드가 여기서 잠깐 블록해도 다른 스레드가
        #   디스커버리를 처리한다. 재진입 pile-up 방지를 위해 sent 를 먼저 claim.
        if not latch.sent:
            if not self._nav_cli.service_is_ready():
                latch.sent = True
                if not self._nav_cli.wait_for_service(timeout_sec=2.0):
                    latch.sent = False
                    self.get_logger().warn(
                        f'[{label}] nav 서비스 준비 대기 중', throttle_duration_sec=2.0)
                    return 'pending'
            latch.sent = True
            req = MoveBaseLateral.Request()
            req.direction = direction
            req.distance_mm = float(self.base_shift_mm)
            self.get_logger().info(
                f'[{label}] MoveBaseLateral.srv 호출 ({direction} {self.base_shift_mm:.0f}mm)')
            fut = self._nav_cli.call_async(req)
            fut.add_done_callback(lambda f, lc=latch: self._on_nav_result(lc, f))
            return 'pending'
        if not latch.done:
            return 'pending'
        res = latch.result
        return 'arrived' if (res is not None and res.arrived) else 'failed'

    # ----------------------------------------------------------------------- #
    # State dispatch
    # ----------------------------------------------------------------------- #
    def _tick(self) -> None:
        handler = {
            State.INIT:             self._run_init,
            State.A1_MONITOR:       self._run_a1_monitor,
            State.A2_SCAN_POSE:     self._run_a2_scan_pose,
            State.A2_SCAN:          self._run_a2_scan,
            State.A3_PICK:          self._run_a3_pick,
            State.A3_MOVE_TO_TRAY:  self._run_a3_move_to_tray,
            State.A3_PLACE:         self._run_a3_place,
            State.A3_RETURN_TO_BOX: self._run_a3_return_to_box,
            State.VERIFY:           self._run_verify,
            State.DONE:             self._run_done,
            State.RECOVERY:         self._run_recovery,
            State.MANUAL_WAIT:      self._run_manual_wait,
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
        """state 진입 시 per-cycle 변수·래치 리셋."""
        if state == State.A2_SCAN_POSE:
            self._scan.reset()
        elif state == State.A2_SCAN:
            self.cycle += 1
            self.last_target_pose = None
            self.current_target_pose = None
            self.last_attached_object = None
            self.current_pick_class = None
        elif state == State.A3_PICK:
            # 파지 트리거(기존 미사용 퍼블리셔 활성화). mock/실 manipulation 이 반응해
            #  /attached_object 로 파지 class 를 보고. sim_mode 에선 SimDriver 가 무시.
            self.pub_attach_cmd.publish(String(data='pick'))
        elif state == State.A3_MOVE_TO_TRAY:
            self._move_tray.reset()
        elif state == State.A3_PLACE:
            self._release_issued = False
        elif state == State.A3_RETURN_TO_BOX:
            self._return.reset()

    # ----------------------------------------------------------------------- #
    # Per-state handlers
    # ----------------------------------------------------------------------- #
    def _run_init(self) -> None:
        self.pub_active_mission.publish(String(data='A'))
        # DDS 위생: scan action(+service 모드면 nav) 서버 준비 + manipulator IDLE 동시 확인.
        servers_ok = self.sim_mode or self._servers_ready()
        idle_ok = (self.last_manipulator_state == 'IDLE')
        if servers_ok and idle_ok:
            self.get_logger().info('[INIT] manipulator IDLE + 서버 준비 확인 -> A1_MONITOR')
            self._transition(State.A1_MONITOR)
        elif self._timed_out():
            self.get_logger().warning(
                f'[INIT] 준비 미완(servers={servers_ok}, idle={idle_ok}) timeout -> A1_MONITOR 강행')
            self._transition(State.A1_MONITOR)

    def _run_a1_monitor(self) -> None:
        if not self.task_list.is_empty():
            total = self.task_list.total_remaining()
            if total > 0:
                self.get_logger().info(
                    f'[A1_MONITOR] task_list 확정: {self.task_list} (총 {total}) -> A2_SCAN_POSE')
                self._transition(State.A2_SCAN_POSE)
            else:
                self.get_logger().info('[A1_MONITOR] task_list 잔여 0 -> VERIFY')
                self._transition(State.VERIFY)
            return
        if self.use_task_list_service:
            self._request_task_list_service()

    def _run_a2_scan_pose(self) -> None:
        # manipulation 스캔 초기 포즈 형성(MoveToScanPose Action). sim_mode 우회.
        if self.sim_mode:
            self._transition(State.A2_SCAN)
            return
        if not self._scan.sent:
            if not self._scan_cli.server_is_ready():
                self._scan_cli.wait_for_server(timeout_sec=0.0)
            if not self._scan_cli.server_is_ready():
                self.get_logger().warn(
                    '[A2_SCAN_POSE] scan action 서버 준비 대기 중', throttle_duration_sec=2.0)
                if self._timed_out():
                    self.get_logger().warning('[A2_SCAN_POSE] scan 서버 timeout -> RECOVERY')
                    self._transition(State.RECOVERY)
                return
            self._scan.sent = True
            goal = MoveToScanPose.Goal()
            goal.preset_id = self.scan_pose_preset_id
            self.get_logger().info('[A2_SCAN_POSE] MoveToScanPose goal 송신')
            fut = self._scan_cli.send_goal_async(goal)
            fut.add_done_callback(self._on_scan_goal_response)
            return
        if self._scan.done:
            if self._scan.result is not None and self._scan.result.success:
                self.get_logger().info('[A2_SCAN_POSE] 스캔 포즈 형성 완료 -> A2_SCAN')
                self._transition(State.A2_SCAN)
            else:
                self.get_logger().warning('[A2_SCAN_POSE] 스캔 포즈 실패 -> RECOVERY')
                self._transition(State.RECOVERY)
        elif self._timed_out():
            self.get_logger().warning('[A2_SCAN_POSE] 스캔 포즈 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_a2_scan(self) -> None:
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
        if self.task_list.is_empty():
            self.get_logger().info('[A2_SCAN] task_list 비어있음 -> VERIFY (완료 판정)')
            self._transition(State.VERIFY)
            return
        if self._timed_out():
            self.get_logger().warning('[A2_SCAN] target 미수신 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_a3_pick(self) -> None:
        # 파지 반응형: manipulation(mock/실) 이 /attached_object 로 파지 class 를 보고.
        if self.last_attached_object:
            self.current_pick_class = self.last_attached_object
            self.get_logger().info(
                f'[A3_PICK] 파지 성공 attached="{self.current_pick_class}" -> A3_MOVE_TO_TRAY')
            self._transition(State.A3_MOVE_TO_TRAY)
        elif self._timed_out():
            self.get_logger().warning('[A3_PICK] 파지 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_a3_move_to_tray(self) -> None:
        # 이동 중 C2 모니터: 파지 손실(드롭) → 무차감 RECOVERY.
        if self.last_attached_object == '':
            self.get_logger().warning(
                '[A3_MOVE_TO_TRAY] 이동 중 파지 손실(드롭) -> RECOVERY (무차감)')
            self._transition(State.RECOVERY)
            return
        st = self._nav_step(self._move_tray, 'left', 'A3_MOVE_TO_TRAY')
        if st == 'arrived':
            self.get_logger().info(
                f'[A3_MOVE_TO_TRAY] 트레이 도착(left {self.base_shift_mm:.0f}mm) -> A3_PLACE')
            self._transition(State.A3_PLACE)
        elif st == 'failed':
            self.get_logger().warning('[A3_MOVE_TO_TRAY] 이동 실패 -> RECOVERY')
            self._transition(State.RECOVERY)
        elif self._timed_out():
            self.get_logger().warning('[A3_MOVE_TO_TRAY] 이동 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_a3_place(self) -> None:
        # 차감 게이트: C1(pick class) ∧ C2(grip 유지) ∧ C3(place 위치 유효, guard).
        # _release_issued 래치가 ''(드롭 vs 해제확정)을 구분.
        if not self._release_issued:
            if not self.current_pick_class:
                self.get_logger().warning('[A3_PLACE] current_pick_class 없음 -> RECOVERY')
                self._transition(State.RECOVERY)
                return
            if self.last_attached_object == '':
                self.get_logger().warning(
                    '[A3_PLACE] release 전 파지 손실(드롭) -> RECOVERY (무차감)')
                self._transition(State.RECOVERY)
                return
            if self.use_place_pose_check and not self._place_pose_valid_now():
                if self._timed_out():
                    self.get_logger().warning(
                        '[A3_PLACE] place 위치 무효 timeout -> RECOVERY (release 안함)')
                    self._transition(State.RECOVERY)
                return  # 유효 전 release 금지, 대기
            self.pub_detach_cmd.publish(String(data=self.current_pick_class))
            self._release_issued = True
            self.get_logger().info(
                f'[A3_PLACE] 게이트 통과 — /detach_cmd 발행 ({self.current_pick_class})')
            return

        if self.last_attached_object == '':
            left = self.task_list.decrement(self.current_pick_class)
            self.placed_count += 1
            kor = CLASS_TO_PART_NAME.get(self.current_pick_class, self.current_pick_class)
            self.get_logger().info(
                f'[A3_PLACE] 적재 확정 {kor} → 잔여 {left} '
                f'(총 {self.task_list.total_remaining()}, '
                f'placed={self.placed_count}, topic_remaining={self._last_topic_remaining})')
            self.current_pick_class = None
            self._transition(State.VERIFY)
        elif self._timed_out():
            self.get_logger().warning('[A3_PLACE] release 확인 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_verify(self) -> None:
        # 라우팅 전용(차감은 A3_PLACE 로 이관). 토픽 잔량은 교차확인·로그용.
        if self.verify_use_topic_remaining and self._last_topic_remaining is not None:
            remaining = self._last_topic_remaining
        else:
            remaining = self.task_list.total_remaining()
        if remaining > 0:
            self.get_logger().info(f'[VERIFY] 잔여 {remaining} > 0 -> A3_RETURN_TO_BOX')
            self._transition(State.A3_RETURN_TO_BOX)
        else:
            self.get_logger().info('[VERIFY] 잔여 0 -> DONE')
            self._transition(State.DONE)

    def _run_a3_return_to_box(self) -> None:
        # 베이스 우 base_shift_mm (트레이→박스). 이미 적재·차감 완료(C2 모니터 없음).
        st = self._nav_step(self._return, 'right', 'A3_RETURN_TO_BOX')
        if st == 'arrived':
            self.get_logger().info(
                f'[A3_RETURN_TO_BOX] 박스 복귀(right {self.base_shift_mm:.0f}mm) -> A2_SCAN_POSE')
            self._transition(State.A2_SCAN_POSE)
        elif st == 'failed':
            self.get_logger().warning('[A3_RETURN_TO_BOX] 복귀 실패 -> RECOVERY')
            self._transition(State.RECOVERY)
        elif self._timed_out():
            self.get_logger().warning('[A3_RETURN_TO_BOX] 복귀 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_done(self) -> None:
        self.get_logger().info(f'[DONE] mission A 완료 (적재 {self.placed_count}개)')
        self.timer.cancel()

    def _run_recovery(self) -> None:
        if self.recovery_count < MAX_RECOVERY_RETRY:
            self.recovery_count += 1
            self.get_logger().warning(
                f'[RECOVERY] 재시도 {self.recovery_count}/{MAX_RECOVERY_RETRY} -> A2_SCAN_POSE')
            # NOTE(v2.3): 드롭/실패 시 base 가 트레이(좌) 위치일 수 있음. 박스 복귀 측방이동을
            #   선행하는 RECOVERY 서브스텝은 실로봇 캘리브 후 별도(현 범위 외).
            self._transition(State.A2_SCAN_POSE)
        else:
            self.get_logger().error('[RECOVERY] 재시도 초과 -> MANUAL_WAIT')
            self._transition(State.MANUAL_WAIT)

    def _run_manual_wait(self) -> None:
        # TODO(Phase2): 운용자 재개 신호 수신 시 -> A2_SCAN_POSE.
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionA()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
