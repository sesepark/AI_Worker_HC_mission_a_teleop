#!/usr/bin/env python3
"""Mission C State Machine.

Mission C 자율 시나리오 상태 기계 (rclpy Node). **Mission A 재사용 극대화**:
A는 트레이에서 너트를 집어 *다른 트레이에 place*, C는 동일하게 집되 *링(peg)에 삽입*한다.
따라서 C 는 A(`mission_a.py`)의 흐름·외부 계약을 그대로 복제하고, **place 단계만 insert 로
교체**하며 **양팔(select_arm)·peg 타깃 선택**을 추가한다.

A 대비 차이(핵심):
  - 상태: A3_PLACE → **C3_INSERT**(peg hover→하강→gripper open 삽입). A3_MOVE_TO_TRAY/
    RETURN_TO_BOX → **C3_MOVE_TO_PEG / C3_RETURN**(동일 MoveBaseLateral 좌/우 측방 이동 재사용).
  - **peg 타깃**: `/perception/head/pipe_top_centers`(PoseArray, base_link) 구독 — 학습 모델 또는
    preset(dummy) 퍼블리셔(`pipe_centers_preset_pub`)가 공급. 학습 완료 시 동일 토픽으로 무변경 교체.
  - **양팔 선택**: `select_arm(y)` (y>=0 → 'left', else 'right'). pick 타깃 y 기준.
  - C3 게이트: A의 트레이 place 유효성(`/perception/place_pose_valid`)을 **peg 삽입 유효성**으로 재사용.

재사용(무변경 계약): MoveToScanPose action, /attach_cmd·/detach_cmd·/attached_object·
  /manipulator_state, /perception/wrist/target_one_pose(pick 타깃), task_list, MoveBaseLateral.srv.
추가 계약(실 C manip 서버용): /mission_c/insert_target(PoseStamped, 선택 peg) + /mission_c/insert_arm(String).
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
from geometry_msgs.msg import PoseStamped, PoseArray

from mission.task_list import TaskList, CLASS_TO_PART_NAME
from mission_interfaces.msg import TaskItem
from mission_interfaces.srv import GetTaskList, MoveBaseLateral
from mission_interfaces.action import MoveToScanPose
from perception.msg import PartDetectionArray


# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #
GRASP_ASSESSMENT_ENABLED = False  # flip to True after Hand-Eye Calibration

# Timeout (seconds)
TIMEOUT_INIT       = 60
TIMEOUT_C1_MONITOR = 90
TIMEOUT_C1_OCR     = 20.0
TIMEOUT_C2_SCAN    = 90
TIMEOUT_PICK_PLACE = 45
TIMEOUT_VERIFY     = 20

MAX_RECOVERY_RETRY = 3

# Mission C base-sequence layout (base frame origin = start pose, y = left +).
BASE_POSITION_A_Y_MM = 0.0
BASE_POSITION_B_Y_MM = 300.0
NUT_POSITIONS_AT_A = {4, 5}
NUT_POSITIONS_AT_B = {1, 2, 3}

# Pipe gaps: pipe1-2, pipe2-3, pipe3-4 [mm].
PIPE_GAPS_MM = (172.0, 179.0, 185.0)
PIPE_REFERENCE_NO = 3

# test_place_c_manual.py reference: at position A, this places on pipe3.
MANUAL_PIPE3_X_M = 0.40
MANUAL_PIPE3_Y_M = -0.335
MANUAL_PIPE3_Z_M = 0.90

# test_place_c.py y offset is applied by mission_c_manipulation_server.


def select_arm(y: float) -> str:
    """양팔 선택: y>=0(로봇 좌측) → 'left', y<0 → 'right'.

    manipulation.mission_c_arm_selector.select_arm 과 동일 규약(Arm enum 대신 문자열 반환 —
    FSM 은 manipulation 패키지에 의존하지 않고 'left'/'right' 만 필요).
    """
    return 'left' if y >= 0.0 else 'right'


# --------------------------------------------------------------------------- #
# State enum
# --------------------------------------------------------------------------- #
class State(Enum):
    INIT            = auto()
    C1_MONITOR      = auto()
    C2_SCAN_POSE    = auto()   # manipulation 스캔 초기 포즈 형성(Action) — A 재사용
    C2_SCAN         = auto()
    C3_PICK         = auto()
    C3_MOVE_TO_PEG  = auto()   # 베이스 측방 이동(MoveBaseLateral) — A3_MOVE_TO_TRAY 재사용
    C3_INSERT       = auto()   # peg 삽입(A3_PLACE 교체)
    C3_RETURN       = auto()   # 베이스 복귀(MoveBaseLateral) — A3_RETURN_TO_BOX 재사용
    C_BSEQ_PICK_MOVE  = auto()  # (base_seq) 너트 A/B 정렬 후 pick
    C_BSEQ_PLACE_MOVE = auto()  # (base_seq) pipe3 기준 gap 이동 후 insert
    VERIFY          = auto()
    DONE            = auto()
    RECOVERY        = auto()
    MANUAL_WAIT     = auto()


STATE_TIMEOUT: dict[State, float] = {
    State.INIT:       TIMEOUT_INIT,
    State.C1_MONITOR: TIMEOUT_C1_MONITOR,
    State.C2_SCAN:    TIMEOUT_C2_SCAN,
    State.C3_PICK:    TIMEOUT_PICK_PLACE,
    State.C3_INSERT:  TIMEOUT_PICK_PLACE,
    State.VERIFY:     TIMEOUT_VERIFY,
}


# --------------------------------------------------------------------------- #
# Async latch (A 와 동일)
# --------------------------------------------------------------------------- #
@dataclass
class AsyncLatch:
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
class MissionC(Node):
    def __init__(self) -> None:
        super().__init__('mission_c')

        # --- Parameters (A 공통) ---
        self.sim_mode = bool(self.declare_parameter('sim_mode', False).value)
        self.task_list_service_name = str(
            self.declare_parameter('task_list_service_name', '/mission_c/task_list').value)
        self.task_list_service_timeout_sec = float(
            self.declare_parameter('task_list_service_timeout_sec', float(TIMEOUT_C1_OCR)).value)
        self.task_list_service_frame_count = int(
            self.declare_parameter('task_list_service_frame_count', 3).value)
        self.task_list_topic = str(
            self.declare_parameter('task_list_topic', '/perception/task_list').value)
        self.use_task_list_service = bool(
            self.declare_parameter('use_task_list_service', False).value)
        self.use_monitor_ocr = bool(
            self.declare_parameter('use_monitor_ocr', True).value)
        self.verify_use_topic_remaining = bool(
            self.declare_parameter('verify_use_topic_remaining', False).value)

        # --- nav (A 재사용: 좌/우 측방 이동) ---
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
        # dry-run: C3_INSERT 를 임의/안전 위치 release 로 항상 통과시켜 전 사이클 다회 시험.
        #   기본 OFF → 기존 동작 무회귀. ON 시 C3 유효성 게이트 우회 + 해제 강제확정(grace).
        #   실제 링 삽입 검증이 아님(로그에 [DRY-RUN] 명시). 실 manip 서버 insert_dry_run 과 함께 사용.
        self.insert_dry_run = bool(
            self.declare_parameter('insert_dry_run', False).value)
        self.insert_dry_run_grace_sec = float(
            self.declare_parameter('insert_dry_run_grace_sec', 2.0).value)
        self.place_pose_valid_debounce_sec = float(
            self.declare_parameter('place_pose_valid_debounce_sec', 0.3).value)
        self.rescan_each_cycle = bool(
            self.declare_parameter('rescan_each_cycle', True).value)
        self.nav_service_name = str(
            self.declare_parameter('nav_service_name', 'move_base_lateral').value)
        self.nav_service_wait_sec = float(
            self.declare_parameter('nav_service_wait_sec', 10.0).value)
        self.scan_action_name = str(
            self.declare_parameter('scan_action_name', 'move_to_scan_pose').value)

        # --- C 신규 파라미터 ---
        # peg 타깃 토픽(학습 모델/preset 공용). PoseArray(base_link).
        self.pipe_centers_topic = str(self.declare_parameter(
            'pipe_centers_topic', '/perception/head/pipe_top_centers').value)
        # peg 타깃 준비 대기(초). 없으면 C3_MOVE_TO_PEG 에서 대기 후 timeout.
        self.require_pipe_centers = bool(
            self.declare_parameter('require_pipe_centers', True).value)
        # C3_MOVE_TO_PEG/RETURN 측방 이동 방향(A: left→tray, right→box).
        self.move_to_peg_dir = str(self.declare_parameter('move_to_peg_dir', 'left').value)
        self.return_dir = str(self.declare_parameter('return_dir', 'right').value)
        # arm_mode: 'right'(현 단계 기본 — A 처럼 우완 단일팔) | 'left' | 'auto'(select_arm 양팔).
        #   실 manip 서버의 arm_mode 와 일치시킬 것. FSM 은 insert_arm 통지에 이 값을 반영.
        self.arm_mode = str(self.declare_parameter('arm_mode', 'right').value).strip().lower()
        if self.arm_mode not in ('right', 'left', 'auto'):
            self.get_logger().warn(f"arm_mode={self.arm_mode!r} 미지원 → 'right' 사용")
            self.arm_mode = 'right'

        # --- 미션 C 베이스 시퀀스 (옵션, 기본 OFF=기존 동작 유지) ---
        # base_seq_enable=True 일 때만 너트 A/B 정렬 + 파이프 기준 place 시퀀스를 사용.
        #   OFF 면 기존 C3_MOVE_TO_PEG/C3_RETURN(고정 측방 왕복) 경로 그대로.
        self.base_seq_enable = bool(
            self.declare_parameter('base_seq_enable', False).value)
        # Legacy parameter. Nut pick is always camera/perception in base_seq.
        self.use_camera = bool(self.declare_parameter('use_camera', True).value)
        if not self.use_camera:
            self.get_logger().warn(
                '[base_seq] use_camera:=false 는 더 이상 pick에 적용되지 않습니다. '
                '너트 파지는 항상 perception/camera target을 사용합니다.')
        # false: pipe place target = test_place_c_manual.py hardcoded pipe3 pose.
        # true: pipe place target = perception pipe center, like test_place_c.py.
        self.use_pipe_camera = bool(
            self.declare_parameter('use_pipe_camera', False).value)
        # Legacy parameters kept for launch/backward compatibility; base_seq pick no longer
        # uses per-nut pitch movement.
        self.nut_pitch_mm = float(self.declare_parameter('nut_pitch_mm', 150.0).value)
        self.place_forward_mm = float(
            self.declare_parameter('place_forward_mm', 100.0).value)
        # Legacy no-op parameters kept so older launch overrides do not break startup.
        self.place_a_nut_index = int(self.declare_parameter('place_a_nut_index', 2).value)
        self.place_b_nut_index = int(self.declare_parameter('place_b_nut_index', 4).value)
        self.place_split_count = int(self.declare_parameter('place_split_count', 2).value)
        # Legacy dry-test constants. Normal Mission C pick always uses perception.
        self.nocam_pick_x = float(self.declare_parameter('nocam_pick_x', 0.35).value)
        self.nocam_pick_z = float(self.declare_parameter('nocam_pick_z', 0.82).value)
        self.nocam_place_x = float(self.declare_parameter('nocam_place_x', 0.50).value)
        self.nocam_place_z = float(self.declare_parameter('nocam_place_z', 0.90).value)
        self.nocam_place_ys = list(self.declare_parameter(
            'nocam_place_ys', [-0.10, -0.25, -0.10, -0.25]).value)
        # 공급대 슬롯(왼→오, 0~4) 별 너트 종류(class) 고정 배치 — 대회 실배치.
        #   슬롯 index = 너트 위치(1~5)-1. 위치 1~3은 B, 4~5는 A에서 카메라로 pick.
        self.nut_slot_order = list(self.declare_parameter(
            'nut_slot_order',
            ['flange_nut', 'gear_ring', 'spacer_ring', 'hex_nut', 'dome_nut']).value)
        # base_seq pick 순서(class) — 파이프 1,2,3,4 에 넣을 너트를 이 순서로 집는다.
        #   베이스가 이 순서대로 너트 슬롯으로 이동 + FSM 이 perception 에 해당 class 만 픽하도록 통지.
        #   한 번 집은 class 는 다음 인덱스로 넘어가 자동 제외(중복 없는 4종 전제).
        self.pick_order = list(self.declare_parameter(
            'pick_order',
            ['flange_nut', 'gear_ring', 'spacer_ring', 'hex_nut']).value)
        # pick 순서를 너트 "위치"(1~5, 왼→오)로 지정 — 예: '4-5-3-1'. 설정 시 pick_order 대체.
        #   위치 p → nut_slot_order[p-1] class. 구분자는 '-'·','·공백 모두 허용.
        self.pick_positions = str(
            self.declare_parameter('pick_positions', '').value).strip()
        if self.pick_positions:
            seq: list[str] = []
            for tok in self.pick_positions.replace('-', ' ').replace(',', ' ').split():
                try:
                    p = int(tok)
                except ValueError:
                    self.get_logger().warn(f'[pick_positions] 정수 아님 무시: {tok!r}')
                    continue
                if 1 <= p <= len(self.nut_slot_order):
                    seq.append(self.nut_slot_order[p - 1])
                else:
                    self.get_logger().warn(
                        f'[pick_positions] 범위 밖 무시: {p} (1~{len(self.nut_slot_order)})')
            if seq:
                self.pick_order = seq
                self.get_logger().info(
                    f'[pick_positions] {self.pick_positions!r} → pick_order={self.pick_order}')

        # 신규 상태 timeout 합성.
        self.state_timeout: dict[State, float] = dict(STATE_TIMEOUT)
        self.state_timeout[State.C2_SCAN_POSE]      = self.scan_pose_timeout_sec
        self.state_timeout[State.C3_MOVE_TO_PEG]    = self.base_move_timeout_sec
        self.state_timeout[State.C3_RETURN]         = self.base_move_timeout_sec
        self.state_timeout[State.C_BSEQ_PICK_MOVE]  = self.base_move_timeout_sec
        self.state_timeout[State.C_BSEQ_PLACE_MOVE] = self.base_move_timeout_sec

        # --- Callback group ---
        self._cbg = ReentrantCallbackGroup()

        # --- 외부 기능 클라이언트 (서브보다 먼저 생성) ---
        self._scan_cli = ActionClient(
            self, MoveToScanPose, self.scan_action_name, callback_group=self._cbg)
        self._nav_cli = self.create_client(
            MoveBaseLateral, self.nav_service_name, callback_group=self._cbg)

        # --- Subscribers (A 재사용) ---
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
            GetTaskList.Response, self.task_list_topic, self._on_task_list, 10,
            callback_group=self._cbg)
        # C3 게이트: peg 삽입 위치 유효성(perception, guard). std_msgs/String JSON.
        self.sub_place_pose_valid = self.create_subscription(
            String, '/perception/place_pose_valid', self._on_place_pose_valid, 10,
            callback_group=self._cbg)
        # C 신규: peg 상단 중심(PoseArray, base_link).
        self.sub_pipe_centers = self.create_subscription(
            PoseArray, self.pipe_centers_topic, self._on_pipe_centers, 10,
            callback_group=self._cbg)

        # --- Service clients ---
        self.task_list_client = self.create_client(
            GetTaskList, self.task_list_service_name, callback_group=self._cbg)

        # --- Publishers ---
        self.pub_active_mission = self.create_publisher(String, '/active_mission', 10)
        self.pub_attach_cmd = self.create_publisher(String, '/attach_cmd', 10)
        self.pub_detach_cmd = self.create_publisher(String, '/detach_cmd', 10)
        self.pub_manual_task_list = self.create_publisher(
            GetTaskList.Response, self.task_list_topic, 10)
        # C 신규: 선택된 peg 타깃·팔을 실 C manip 서버에 통지(서버 미구현 시 무해).
        self.pub_insert_target = self.create_publisher(
            PoseStamped, '/mission_c/insert_target', 10)
        self.pub_insert_arm = self.create_publisher(String, '/mission_c/insert_arm', 10)
        self.pub_place_mode = self.create_publisher(String, '/mission_c/place_mode', 10)
        # 미션 C 카메라 미사용 모드: pick 타깃 상수를 직접 발행(perception 우회).
        self.pub_wrist_target = self.create_publisher(
            PoseStamped, '/perception/wrist/target_one_pose', 10)
        # base_seq: FSM 이 "지금 집을 너트 class" 를 perception 에 통지 → 그 class 만 픽(조율+제외).
        self.pub_pick_target_class = self.create_publisher(
            String, '/perception/wrist/target_class', 10)

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
        self.last_task_list_response: GetTaskList.Response | None = None
        self._last_topic_remaining: int | None = None
        self._place_pose_valid: dict | None = None
        self._place_valid_since: float | None = None

        # C 신규: peg 중심들 + 진행 인덱스 + 현 사이클 선택.
        self.last_pipe_centers: PoseArray | None = None
        self.peg_index: int = 0
        self.current_insert_arm: str | None = None
        self.current_insert_pose: PoseStamped | None = None

        # base_seq: 베이스 절대 위치(시작 기준, mm; x=앞+, y=왼+) + 진행 인덱스 + 2축 이동 상태.
        self.base_x_mm: float = 0.0
        self.base_y_mm: float = 0.0
        self.bseq_index: int = 0          # 현재 처리 중 파이프/너트 0-based
        self.pipe_nut_classes: list[str] = []   # 파이프 순서대로 넣을 너트 class (task_list)
        self._bseq_phase: int = 0         # 0=측방, 1=전진, 2=완료
        self._bseq_lat = AsyncLatch()
        self._bseq_fwd = AsyncLatch()

        self.task_list: TaskList = TaskList()
        self.current_target_pose: PoseStamped | None = None
        self.current_pick_class: str | None = None
        self._task_list_service_inflight: bool = False
        self._task_list_service_next_try_time: float = 0.0
        self._manual_task_list_msg: GetTaskList.Response | None = None

        self._scan = AsyncLatch()
        self._move_peg = AsyncLatch()
        self._return = AsyncLatch()
        self._release_issued: bool = False
        self._release_issued_time: float = 0.0   # dry-run grace 기준(detach 발행 시각)

        self._sim = None
        if self.sim_mode:
            from mission.sim_driver import SimDriver
            self._sim = SimDriver(self, State)

        self.timer = self.create_timer(0.1, self._tick, callback_group=self._cbg)
        self.manual_task_timer = self.create_timer(
            1.0, self._publish_manual_task_list, callback_group=self._cbg)
        self.get_logger().info(
            f'mission_c started in state={self.state.name} '
            f'(sim_mode={self.sim_mode}, nav_mode={self.nav_mode}, arm_mode={self.arm_mode}, '
            f'base_shift_mm={self.base_shift_mm}, use_place_pose_check={self.use_place_pose_check}, '
            f'insert_dry_run={self.insert_dry_run}, '
            f'base_seq_enable={self.base_seq_enable}, use_pipe_camera={self.use_pipe_camera}, '
            f'use_monitor_ocr={self.use_monitor_ocr}, '
            f'pipe_centers_topic={self.pipe_centers_topic}, '
            f'GRASP_ASSESSMENT_ENABLED={GRASP_ASSESSMENT_ENABLED})')

    # ----------------------------------------------------------------------- #
    # Helpers (A 동일)
    # ----------------------------------------------------------------------- #
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self) -> float:
        return self._now() - self._state_enter_time

    def _timed_out(self) -> bool:
        limit = self.state_timeout.get(self.state)
        return limit is not None and self._elapsed() > limit

    def _on_manipulator_state(self, msg: String) -> None:
        self.last_manipulator_state = msg.data

    def _on_detections(self, msg) -> None:
        self.last_detections = msg

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self.last_target_pose = msg

    def _on_attached_object(self, msg: String) -> None:
        self.last_attached_object = msg.data

    def _on_pipe_centers(self, msg: PoseArray) -> None:
        self.last_pipe_centers = msg

    def _on_place_pose_valid(self, msg: String) -> None:
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

    def _on_task_list(self, msg: GetTaskList.Response) -> None:
        if not self.use_monitor_ocr:
            return
        parts = [{'name': item.name, 'count': item.count} for item in msg.parts]
        self.last_task_list_response = msg
        self._last_topic_remaining = sum(
            max(int(p.get('count', 0) or 0), 0) for p in parts)
        if self.state in (State.INIT, State.C1_MONITOR):
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
            f'[C1_MONITOR] task_list service 요청: {self.task_list_service_name}')
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
            f'[C1_MONITOR] task_list service result: {self.task_list} '
            f'(frames={response.frames_used})')

    def _build_manual_task_list_from_pick_order(self) -> bool:
        """OCR 미사용 시 pick_order/pick_positions 기반 task_list를 생성한다."""
        classes = [c for c in self.pick_order if c in self.nut_slot_order]
        if not classes:
            self.get_logger().error(
                '[C1_MONITOR] use_monitor_ocr:=false 이지만 유효한 pick_order가 없습니다. '
                '예: pick_positions:=3-5-1-2')
            return False

        counts: dict[str, int] = {}
        for cls in classes:
            counts[cls] = counts.get(cls, 0) + 1

        parts = [
            {'name': CLASS_TO_PART_NAME.get(cls, cls), 'count': count}
            for cls, count in counts.items()
        ]
        self.task_list.build_from_ocr_parts(parts)
        self._last_topic_remaining = self.task_list.total_remaining()

        msg = GetTaskList.Response()
        msg.success = True
        msg.message = json.dumps({
            'source': 'manual_pick_positions',
            'pick_positions': self.pick_positions,
            'pick_order': classes,
            'use_monitor_ocr': False,
        }, ensure_ascii=False)
        msg.screen_detected = True
        msg.all_counts_recognized = True
        msg.frames_used = 0
        msg.parts = [
            TaskItem(name=str(part['name']), count=int(part['count']))
            for part in parts
        ]
        self._manual_task_list_msg = msg
        self._publish_manual_task_list()
        self.get_logger().info(
            f'[C1_MONITOR] manual task_list 생성: {self.task_list} '
            f'(pick_order={classes})')
        return True

    def _publish_manual_task_list(self) -> None:
        if self.use_monitor_ocr or self._manual_task_list_msg is None:
            return
        self.pub_manual_task_list.publish(self._manual_task_list_msg)

    # ----------------------------------------------------------------------- #
    # 외부 기능 호출 (scan Action / nav Service) — A 동일
    # ----------------------------------------------------------------------- #
    def _servers_ready(self) -> bool:
        if not self._scan_cli.server_is_ready():
            self._scan_cli.wait_for_server(timeout_sec=0.0)
            if not self._scan_cli.server_is_ready():
                return False
        if self.nav_mode == 'service' and not self._nav_cli.service_is_ready():
            self._nav_cli.wait_for_service(timeout_sec=0.0)
        return True

    def _on_scan_goal_response(self, future) -> None:
        try:
            gh = future.result()
        except Exception as exc:
            self.get_logger().warn(f'[C2_SCAN_POSE] goal 응답 예외: {exc}')
            self._scan.result = None
            self._scan.done = True
            return
        if not gh.accepted:
            self.get_logger().warn('[C2_SCAN_POSE] goal 거부됨')
            self._scan.result = None
            self._scan.done = True
            return
        self._scan.goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_scan_result)

    def _on_scan_result(self, future) -> None:
        try:
            self._scan.result = future.result().result
        except Exception as exc:
            self.get_logger().warn(f'[C2_SCAN_POSE] result 예외: {exc}')
            self._scan.result = None
        self._scan.done = True

    def _on_nav_result(self, latch: AsyncLatch, future) -> None:
        try:
            latch.result = future.result()
        except Exception as exc:
            self.get_logger().warn(f'nav service 예외: {exc}')
            latch.result = None
        latch.done = True

    def _nav_step(self, latch: AsyncLatch, direction: str, label: str,
                  distance_mm: float | None = None) -> str:
        """베이스 1축 이동 1스텝(A 재사용). 반환: 'arrived' | 'pending' | 'failed'.

        distance_mm=None 이면 기존 동작(base_shift_mm). base_seq 는 축별 델타를 명시 전달.
        direction 은 'left'|'right'(측방) 또는 'forward'|'back'(전진/후진, 미션 C 확장).
        """
        dist = float(self.base_shift_mm if distance_mm is None else distance_mm)
        if self.sim_mode or self.nav_mode == 'stub':
            if not latch.sent:
                latch.sent = True
                self.get_logger().info(
                    f'[{label}] nav stub instant success ({direction} {dist:.0f}mm)')
            return 'arrived'
        if not latch.sent:
            if not self._nav_cli.service_is_ready():
                latch.sent = True
                if not self._nav_cli.wait_for_service(timeout_sec=self.nav_service_wait_sec):
                    latch.sent = False
                    self.get_logger().warn(
                        f'[{label}] nav 서비스 준비 대기 중', throttle_duration_sec=2.0)
                    return 'pending'
            latch.sent = True
            req = MoveBaseLateral.Request()
            req.direction = direction
            req.distance_mm = dist
            self.get_logger().info(
                f'[{label}] MoveBaseLateral.srv 호출 ({direction} {dist:.0f}mm)')
            fut = self._nav_cli.call_async(req)
            fut.add_done_callback(lambda f, lc=latch: self._on_nav_result(lc, f))
            return 'pending'
        if not latch.done:
            return 'pending'
        res = latch.result
        return 'arrived' if (res is not None and res.arrived) else 'failed'

    # --- base_seq: 2축(측방→전진) 절대위치 이동 + 카메라 미사용 상수 주입 ---
    def _goto_base(self, target_x_mm: float, target_y_mm: float, label: str) -> str:
        """현재 base (x,y)에서 목표로 측방→전진 순차 이동. 'arrived'|'pending'|'failed'.

        _bseq_phase: 0=측방(y), 1=전진(x), 2=완료. 상태 진입 시 0/래치 리셋되어 있어야 함.
        """
        if self._bseq_phase == 0:
            dy = target_y_mm - self.base_y_mm
            if abs(dy) < 1.0:
                self._bseq_phase = 1
                self._bseq_fwd.reset()
            else:
                direction = 'left' if dy > 0.0 else 'right'
                st = self._nav_step(self._bseq_lat, direction, label + '-lat', abs(dy))
                if st == 'arrived':
                    self.base_y_mm = target_y_mm
                    self._bseq_phase = 1
                    self._bseq_fwd.reset()
                elif st == 'failed':
                    return 'failed'
                else:
                    return 'pending'
        if self._bseq_phase == 1:
            dx = target_x_mm - self.base_x_mm
            if abs(dx) < 1.0:
                self._bseq_phase = 2
            else:
                direction = 'forward' if dx > 0.0 else 'back'
                st = self._nav_step(self._bseq_fwd, direction, label + '-fwd', abs(dx))
                if st == 'arrived':
                    self.base_x_mm = target_x_mm
                    self._bseq_phase = 2
                elif st == 'failed':
                    return 'failed'
                else:
                    return 'pending'
        return 'arrived' if self._bseq_phase == 2 else 'pending'

    def _publish_nocam_pick(self) -> None:
        """Legacy helper for old dry tests. Base-seq Mission C no longer calls this."""
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = self.nocam_pick_x
        ps.pose.position.y = 0.0
        ps.pose.position.z = self.nocam_pick_z
        ps.pose.orientation.w = 1.0
        self.pub_wrist_target.publish(ps)

    def _publish_insert_target_pose(
        self, ps: PoseStamped, arm: str, label: str, mode: str
    ) -> None:
        """Publish the selected pipe target to the Mission C manipulation server."""
        self.current_insert_pose = ps
        self.current_insert_arm = arm
        self.pub_place_mode.publish(String(data=mode))
        self.pub_insert_target.publish(ps)
        self.pub_insert_arm.publish(String(data=arm))
        p = ps.pose.position
        self.get_logger().info(
            f'[{label}] insert target -> mode={mode} arm={arm} '
            f'pos=({p.x:.3f},{p.y:+.3f},{p.z:.3f})')

    def _build_pipe_nut_classes(self) -> None:
        """파이프 순서 너트 class 리스트 = pick_order(설정).

        파이프 1,2,3,4 에 이 순서대로 너트를 집어 넣는다(베이스는 너트 위치에 따라
        A/B 로만 이동하고, perception 에 해당 class 만 픽하도록 통지). 파이프 수 = len(pick_order).
        nut_slot_order 에 없는 class 는 제외(슬롯 매핑 불가).
        """
        classes = [c for c in self.pick_order if c in self.nut_slot_order]
        dropped = [c for c in self.pick_order if c not in self.nut_slot_order]
        if dropped:
            self.get_logger().warn(
                f'[C1_MONITOR] pick_order 중 nut_slot_order 에 없는 class 제외: {dropped}')
        self.pipe_nut_classes = classes
        self.get_logger().info(
            f'[C1_MONITOR] base_seq 픽 순서(pick_order): {classes} '
            f'(slot_order={self.nut_slot_order})')

    def _slot_of_current(self) -> int:
        """현재 파이프(bseq_index)의 너트 class → 공급대 슬롯 index. 매핑 실패 시 순차 폴백."""
        if self.pipe_nut_classes and self.bseq_index < len(self.pipe_nut_classes):
            cls = self.pipe_nut_classes[self.bseq_index]
            if cls in self.nut_slot_order:
                return self.nut_slot_order.index(cls)
            self.get_logger().warn(
                f'[base_seq] class {cls!r} 가 nut_slot_order 에 없음 → 순차 슬롯 폴백')
        return self.bseq_index

    def _current_nut_position(self) -> int:
        """현재 pick class 의 공급대 위치(1~5, 왼→오)."""
        return self._slot_of_current() + 1

    def _bseq_pick_target(self) -> tuple[float, str, int]:
        """현재 너트의 pick 베이스 목표 y(mm)와 A/B 그룹."""
        nut_pos = self._current_nut_position()
        if nut_pos in NUT_POSITIONS_AT_B:
            return BASE_POSITION_B_Y_MM, 'B', nut_pos
        if nut_pos not in NUT_POSITIONS_AT_A:
            self.get_logger().warn(
                f'[base_seq] 알 수 없는 너트 위치 {nut_pos} -> 위치 A 사용')
        return BASE_POSITION_A_Y_MM, 'A', nut_pos

    def _pipe_base_target_y_mm(self, pipe_no: int) -> float:
        """pipe3 at A 기준으로 target pipe가 같은 arm pose에 오도록 하는 base y(mm)."""
        if pipe_no < 1 or pipe_no > len(PIPE_GAPS_MM) + 1:
            self.get_logger().warn(
                f'[base_seq] pipe 번호 {pipe_no} 범위 밖 -> pipe{PIPE_REFERENCE_NO} 기준 사용')
            return BASE_POSITION_A_Y_MM
        if pipe_no == PIPE_REFERENCE_NO:
            return BASE_POSITION_A_Y_MM
        if pipe_no < PIPE_REFERENCE_NO:
            gap = sum(PIPE_GAPS_MM[pipe_no - 1:PIPE_REFERENCE_NO - 1])
            return BASE_POSITION_A_Y_MM + gap
        gap = sum(PIPE_GAPS_MM[PIPE_REFERENCE_NO - 1:pipe_no - 1])
        return BASE_POSITION_A_Y_MM - gap

    def _bseq_place_target(self) -> tuple[float, float, int]:
        """현재 파이프의 place 베이스 목표 (x_mm, y_mm, pipe_no)."""
        pipe_no = self.bseq_index + 1
        y_mm = self._pipe_base_target_y_mm(pipe_no)
        return self.base_x_mm, y_mm, pipe_no

    def _set_manual_pipe_place(self, pipe_no: int) -> bool:
        """test_place_c_manual.py 와 같은 pipe3 기준 하드코딩 target을 발행."""
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = MANUAL_PIPE3_X_M
        ps.pose.position.y = MANUAL_PIPE3_Y_M
        ps.pose.position.z = MANUAL_PIPE3_Z_M
        ps.pose.orientation.w = 1.0
        arm = self.arm_mode if self.arm_mode in ('right', 'left') else select_arm(MANUAL_PIPE3_Y_M)
        self._publish_insert_target_pose(ps, arm, f'C_BSEQ_PLACE_MOVE pipe{pipe_no} manual', 'manual')
        return True

    def _set_camera_pipe_place(self, pipe_no: int) -> bool:
        """perception pipe center raw target을 발행. y-offset은 manip 서버가 적용한다."""
        centers = self.last_pipe_centers
        idx = pipe_no - 1
        if centers is None or idx < 0 or idx >= len(centers.poses):
            self.get_logger().warn(
                f'[C_BSEQ_PLACE_MOVE] pipe{pipe_no} perception 중심 대기({self.pipe_centers_topic})',
                throttle_duration_sec=2.0)
            return False
        pose = centers.poses[idx]
        ps = PoseStamped()
        ps.header.frame_id = centers.header.frame_id or 'base_link'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = pose.position.x
        ps.pose.position.y = pose.position.y
        ps.pose.position.z = pose.position.z
        ps.pose.orientation = pose.orientation
        arm = self.arm_mode if self.arm_mode in ('right', 'left') else select_arm(ps.pose.position.y)
        self._publish_insert_target_pose(ps, arm, f'C_BSEQ_PLACE_MOVE pipe{pipe_no} camera', 'camera')
        return True

    def _prepare_bseq_place_target(self, pipe_no: int) -> bool:
        if self.use_pipe_camera:
            return self._set_camera_pipe_place(pipe_no)
        return self._set_manual_pipe_place(pipe_no)

    # --- C 신규: peg 선택 + 삽입 타깃 통지 ---
    def _select_next_peg(self) -> bool:
        """다음 peg 중심을 선택해 current_insert_pose/arm 설정 + 통지. 성공 시 True."""
        centers = self.last_pipe_centers
        if centers is None or not centers.poses:
            return False
        pose = centers.poses[self.peg_index % len(centers.poses)]
        ps = PoseStamped()
        ps.header.frame_id = centers.header.frame_id or 'base_link'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose
        # arm_mode: right/left 고정 또는 auto=select_arm(peg y). 서버 arm_mode 와 일치시킬 것.
        arm = self.arm_mode if self.arm_mode in ('right', 'left') else select_arm(pose.position.y)
        self.current_insert_pose = ps
        self.current_insert_arm = arm
        self.pub_place_mode.publish(String(data='camera'))
        self.pub_insert_target.publish(ps)
        self.pub_insert_arm.publish(String(data=arm))
        self.get_logger().info(
            f'[C3] peg#{self.peg_index} 선택 → arm={arm} '
            f'pos=({pose.position.x:.3f},{pose.position.y:+.3f},{pose.position.z:.3f})')
        return True

    # ----------------------------------------------------------------------- #
    # State dispatch
    # ----------------------------------------------------------------------- #
    def _tick(self) -> None:
        handler = {
            State.INIT:            self._run_init,
            State.C1_MONITOR:      self._run_c1_monitor,
            State.C2_SCAN_POSE:    self._run_c2_scan_pose,
            State.C2_SCAN:         self._run_c2_scan,
            State.C3_PICK:         self._run_c3_pick,
            State.C3_MOVE_TO_PEG:  self._run_c3_move_to_peg,
            State.C3_INSERT:       self._run_c3_insert,
            State.C3_RETURN:       self._run_c3_return,
            State.C_BSEQ_PICK_MOVE:  self._run_bseq_pick_move,
            State.C_BSEQ_PLACE_MOVE: self._run_bseq_place_move,
            State.VERIFY:          self._run_verify,
            State.DONE:            self._run_done,
            State.RECOVERY:        self._run_recovery,
            State.MANUAL_WAIT:     self._run_manual_wait,
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
        if state == State.C2_SCAN_POSE:
            self._scan.reset()
        elif state == State.C2_SCAN:
            self.cycle += 1
            self.last_target_pose = None
            self.current_target_pose = None
            self.last_attached_object = None
            self.current_pick_class = None
        elif state == State.C3_PICK:
            self.pub_attach_cmd.publish(String(data='pick'))
        elif state == State.C3_MOVE_TO_PEG:
            self._move_peg.reset()
        elif state == State.C3_INSERT:
            self._release_issued = False
            self._release_issued_time = 0.0
        elif state == State.C3_RETURN:
            self._return.reset()
        elif state in (State.C_BSEQ_PICK_MOVE, State.C_BSEQ_PLACE_MOVE):
            self._bseq_phase = 0
            self._bseq_lat.reset()
            self._bseq_fwd.reset()
            if state == State.C_BSEQ_PICK_MOVE:
                # 너트 pick은 항상 카메라/perception 사용: 이번 class만 픽하도록 통지.
                cls = (self.pipe_nut_classes[self.bseq_index]
                       if self.bseq_index < len(self.pipe_nut_classes) else '')
                if cls:
                    self.pub_pick_target_class.publish(String(data=cls))
                    self.get_logger().info(
                        f'[C_BSEQ_PICK_MOVE] perception 타깃 class 통지: {cls} '
                        f'(파이프{self.bseq_index + 1})')
            if state == State.C_BSEQ_PLACE_MOVE:
                self.current_insert_pose = None
                self.current_insert_arm = None

    # ----------------------------------------------------------------------- #
    # Per-state handlers
    # ----------------------------------------------------------------------- #
    def _run_init(self) -> None:
        self.pub_active_mission.publish(String(data='C'))
        servers_ok = self.sim_mode or self._servers_ready()
        idle_ok = (self.last_manipulator_state == 'IDLE')
        if servers_ok and idle_ok:
            self.get_logger().info('[INIT] manipulator IDLE + 서버 준비 확인 -> C1_MONITOR')
            self._transition(State.C1_MONITOR)
        elif self._timed_out():
            self.get_logger().warning(
                f'[INIT] 준비 미완(servers={servers_ok}, idle={idle_ok}) timeout -> C1_MONITOR 강행')
            self._transition(State.C1_MONITOR)

    def _run_c1_monitor(self) -> None:
        if (self.base_seq_enable and not self.use_monitor_ocr
                and self.task_list.is_empty()):
            if not self._build_manual_task_list_from_pick_order():
                if self._timed_out():
                    self._transition(State.RECOVERY)
                return

        if not self.task_list.is_empty():
            total = self.task_list.total_remaining()
            if total > 0:
                if self.base_seq_enable:
                    self._build_pipe_nut_classes()
                    self.get_logger().info(
                        f'[C1_MONITOR] task_list 확정: {self.task_list} (총 {total}) '
                        f'-> C_BSEQ_PICK_MOVE (base_seq)')
                    self._transition(State.C_BSEQ_PICK_MOVE)
                    return
                self.get_logger().info(
                    f'[C1_MONITOR] task_list 확정: {self.task_list} (총 {total}) -> C2_SCAN_POSE')
                self._transition(State.C2_SCAN_POSE)
            else:
                self.get_logger().info('[C1_MONITOR] task_list 잔여 0 -> VERIFY')
                self._transition(State.VERIFY)
            return
        if self.use_monitor_ocr and self.use_task_list_service:
            self._request_task_list_service()

    def _run_c2_scan_pose(self) -> None:
        if self.sim_mode:
            self._transition(State.C2_SCAN)
            return
        if not self._scan.sent:
            if not self._scan_cli.server_is_ready():
                self._scan_cli.wait_for_server(timeout_sec=0.0)
            if not self._scan_cli.server_is_ready():
                self.get_logger().warn(
                    '[C2_SCAN_POSE] scan action 서버 준비 대기 중', throttle_duration_sec=2.0)
                if self._timed_out():
                    self.get_logger().warning('[C2_SCAN_POSE] scan 서버 timeout -> RECOVERY')
                    self._transition(State.RECOVERY)
                return
            self._scan.sent = True
            goal = MoveToScanPose.Goal()
            goal.preset_id = self.scan_pose_preset_id
            self.get_logger().info('[C2_SCAN_POSE] MoveToScanPose goal 송신')
            fut = self._scan_cli.send_goal_async(goal)
            fut.add_done_callback(self._on_scan_goal_response)
            return
        if self._scan.done:
            if self._scan.result is not None and self._scan.result.success:
                self.get_logger().info('[C2_SCAN_POSE] 스캔 포즈 형성 완료 -> C2_SCAN')
                self._transition(State.C2_SCAN)
            else:
                self.get_logger().warning('[C2_SCAN_POSE] 스캔 포즈 실패 -> RECOVERY')
                self._transition(State.RECOVERY)
        elif self._timed_out():
            self.get_logger().warning('[C2_SCAN_POSE] 스캔 포즈 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_c2_scan(self) -> None:
        if self.last_target_pose is not None:
            frame = self.last_target_pose.header.frame_id
            if frame != 'base_link':
                self.get_logger().warning(
                    f'[C2_SCAN] target frame_id={frame!r} (base_link 아님) — 무시')
                self.last_target_pose = None
                return
            self.current_target_pose = self.last_target_pose
            self.last_target_pose = None
            p = self.current_target_pose.pose.position
            self.get_logger().info(
                f'[C2_SCAN] target 수신 ({p.x:.3f},{p.y:.3f},{p.z:.3f}) -> C3_PICK')
            self._transition(State.C3_PICK)
            return
        if self.task_list.is_empty():
            self.get_logger().info('[C2_SCAN] task_list 비어있음 -> VERIFY (완료 판정)')
            self._transition(State.VERIFY)
            return
        if self._timed_out():
            self.get_logger().warning('[C2_SCAN] target 미수신 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_c3_pick(self) -> None:
        if self.last_attached_object:
            self.current_pick_class = self.last_attached_object
            nxt = State.C_BSEQ_PLACE_MOVE if self.base_seq_enable else State.C3_MOVE_TO_PEG
            self.get_logger().info(
                f'[C3_PICK] 파지 성공 attached="{self.current_pick_class}" -> {nxt.name}')
            self._transition(nxt)
        elif self._timed_out():
            self.get_logger().warning('[C3_PICK] 파지 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    # --- base_seq 전용 핸들러 (base_seq_enable=True 일 때만 진입) ---
    def _run_bseq_pick_move(self) -> None:
        """너트 위치를 A/B로만 정렬한 뒤 C2_SCAN_POSE."""
        ty, group, nut_pos = self._bseq_pick_target()
        cls = (self.pipe_nut_classes[self.bseq_index]
               if self.bseq_index < len(self.pipe_nut_classes) else '?')
        st = self._goto_base(0.0, ty, f'C_BSEQ_PICK_MOVE#{self.bseq_index}({group})')
        if st == 'arrived':
            self.get_logger().info(
                f'[C_BSEQ_PICK_MOVE] 파이프{self.bseq_index + 1} 너트={cls} '
                f'위치{nut_pos}->{group} 정렬 완료 '
                f'(base x={self.base_x_mm:.0f} y={self.base_y_mm:.0f}mm) -> C2_SCAN_POSE')
            self._transition(State.C2_SCAN_POSE)
        elif st == 'failed':
            self.get_logger().warning('[C_BSEQ_PICK_MOVE] 이동 실패 -> RECOVERY')
            self._transition(State.RECOVERY)
        elif self._timed_out():
            self.get_logger().warning('[C_BSEQ_PICK_MOVE] 이동 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_bseq_place_move(self) -> None:
        """pipe3 기준 gap으로 정렬 후 insert target을 선택하고 C3_INSERT."""
        if self.last_attached_object == '':
            self.get_logger().warning(
                '[C_BSEQ_PLACE_MOVE] 이동 중 파지 손실(드롭) -> RECOVERY (무차감)')
            self._transition(State.RECOVERY)
            return
        tx, ty, pipe_no = self._bseq_place_target()
        st = self._goto_base(tx, ty, f'C_BSEQ_PLACE_MOVE#{self.bseq_index}(pipe{pipe_no})')
        if st == 'arrived':
            if not self._prepare_bseq_place_target(pipe_no):
                if self._timed_out():
                    self.get_logger().warning(
                        f'[C_BSEQ_PLACE_MOVE] pipe{pipe_no} target 미수신 timeout -> RECOVERY')
                    self._transition(State.RECOVERY)
                return
            self.get_logger().info(
                f'[C_BSEQ_PLACE_MOVE] pipe{pipe_no} 정렬 완료 '
                f'(base x={self.base_x_mm:.0f} y={self.base_y_mm:.0f}mm) -> C3_INSERT')
            self._transition(State.C3_INSERT)
        elif st == 'failed':
            self.get_logger().warning('[C_BSEQ_PLACE_MOVE] 이동 실패 -> RECOVERY')
            self._transition(State.RECOVERY)
        elif self._timed_out():
            self.get_logger().warning('[C_BSEQ_PLACE_MOVE] 이동 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_c3_move_to_peg(self) -> None:
        # 이동 중 C2 모니터: 파지 손실(드롭) → 무차감 RECOVERY.
        if self.last_attached_object == '':
            self.get_logger().warning(
                '[C3_MOVE_TO_PEG] 이동 중 파지 손실(드롭) -> RECOVERY (무차감)')
            self._transition(State.RECOVERY)
            return
        # peg 타깃 선택(최초 1회) — 선택된 peg/arm 을 실 C 서버에 통지.
        if self.current_insert_pose is None:
            if not self._select_next_peg():
                if self.require_pipe_centers:
                    self.get_logger().warn(
                        f'[C3_MOVE_TO_PEG] peg 중심 대기({self.pipe_centers_topic})',
                        throttle_duration_sec=2.0)
                    if self._timed_out():
                        self.get_logger().warning('[C3_MOVE_TO_PEG] peg 미수신 timeout -> RECOVERY')
                        self._transition(State.RECOVERY)
                    return
        st = self._nav_step(self._move_peg, self.move_to_peg_dir, 'C3_MOVE_TO_PEG')
        if st == 'arrived':
            self.get_logger().info(
                f'[C3_MOVE_TO_PEG] peg 정렬 도착({self.move_to_peg_dir} {self.base_shift_mm:.0f}mm) '
                f'-> C3_INSERT')
            self._transition(State.C3_INSERT)
        elif st == 'failed':
            self.get_logger().warning('[C3_MOVE_TO_PEG] 이동 실패 -> RECOVERY')
            self._transition(State.RECOVERY)
        elif self._timed_out():
            self.get_logger().warning('[C3_MOVE_TO_PEG] 이동 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _confirm_insert(self, forced: bool) -> None:
        """해제 확정 후 차감 + 다음 사이클 준비 + VERIFY 전이.

        forced=True 는 dry-run 의 해제 강제확정(grace 경과, 실제 삽입 검증 아님).
        실/dry 공통으로 사이클 성공 완료 시 recovery_count 를 리셋(데드엔드 예방, 2.4).
        """
        left = self.task_list.decrement(self.current_pick_class)
        self.placed_count += 1
        self.peg_index += 1
        self.recovery_count = 0  # 사이클 성공 완료 → 누적 RECOVERY 리셋
        kor = CLASS_TO_PART_NAME.get(self.current_pick_class, self.current_pick_class)
        tag = '[DRY-RUN] ' if forced else ''
        self.get_logger().info(
            f'[C3_INSERT]{tag}삽입 확정 {kor} → 잔여 {left} '
            f'(총 {self.task_list.total_remaining()}, '
            f'placed={self.placed_count}, peg#{self.peg_index - 1}, '
            f'topic_remaining={self._last_topic_remaining})')
        self.current_pick_class = None
        self.current_insert_pose = None
        self.current_insert_arm = None
        self._transition(State.VERIFY)

    def _run_c3_insert(self) -> None:
        # 차감 게이트: C1(pick class) ∧ C2(grip 유지) ∧ C3(peg 삽입 위치 유효, guard).
        # peg 삽입 모션(hover→Cartesian 하강→gripper open)은 실 C manip 서버 담당.
        # FSM 은 A3_PLACE 와 동일하게 release(/detach_cmd) 후 /attached_object=='' 로 완료 확인.
        # dry-run: C3 유효성 대기를 우회하고, /attached_object="" 미수신 시 grace 후 강제확정.
        if not self._release_issued:
            if not self.current_pick_class:
                self.get_logger().warning('[C3_INSERT] current_pick_class 없음 -> RECOVERY')
                self._transition(State.RECOVERY)
                return
            if self.last_attached_object == '':
                self.get_logger().warning(
                    '[C3_INSERT] release 전 파지 손실(드롭) -> RECOVERY (무차감)')
                self._transition(State.RECOVERY)
                return
            # C3 게이트(삽입 위치 유효): dry-run 이면 우회(C1·C2 는 위에서 유지).
            if (self.use_place_pose_check and not self.insert_dry_run
                    and not self._place_pose_valid_now()):
                if self._timed_out():
                    self.get_logger().warning(
                        '[C3_INSERT] 삽입 위치 무효 timeout -> RECOVERY (release 안함)')
                    self._transition(State.RECOVERY)
                return  # 유효 전 release 금지, 대기
            if self.insert_dry_run and self.use_place_pose_check:
                self.get_logger().info(
                    '[C3_INSERT][DRY-RUN] C3 게이트 우회 — release 발행 (실제 삽입 검증 아님)',
                    throttle_duration_sec=5.0)
            self.pub_detach_cmd.publish(String(data=self.current_pick_class))
            self._release_issued = True
            self._release_issued_time = self._now()
            self.get_logger().info(
                f'[C3_INSERT] 게이트 통과 — /detach_cmd 발행 ({self.current_pick_class}, '
                f'arm={self.current_insert_arm})')
            return

        if self.last_attached_object == '':
            self._confirm_insert(forced=False)
        elif (self.insert_dry_run
              and (self._now() - self._release_issued_time) >= self.insert_dry_run_grace_sec):
            self.get_logger().info(
                '[C3_INSERT][DRY-RUN] 해제 강제확정 (grace 경과, 실제 삽입 검증 아님)')
            self._confirm_insert(forced=True)
        elif self._timed_out():
            self.get_logger().warning('[C3_INSERT] release 확인 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_verify(self) -> None:
        # base_seq: 진행은 pick_order(파이프 수)로 결정 — 모든 파이프 처리하면 DONE.
        if self.base_seq_enable:
            if self.bseq_index + 1 < len(self.pipe_nut_classes):
                self.bseq_index += 1   # 다음 파이프/너트로
                self.get_logger().info(
                    f'[VERIFY] base_seq 다음 파이프 -> C_BSEQ_PICK_MOVE '
                    f'(파이프{self.bseq_index + 1}/{len(self.pipe_nut_classes)}, '
                    f'너트={self.pipe_nut_classes[self.bseq_index]})')
                self._transition(State.C_BSEQ_PICK_MOVE)
            else:
                self.get_logger().info(
                    f'[VERIFY] base_seq 전 파이프({len(self.pipe_nut_classes)}) 완료 -> DONE')
                self._transition(State.DONE)
            return
        if self.verify_use_topic_remaining and self._last_topic_remaining is not None:
            remaining = self._last_topic_remaining
        else:
            remaining = self.task_list.total_remaining()
        if remaining > 0:
            self.get_logger().info(f'[VERIFY] 잔여 {remaining} > 0 -> C3_RETURN')
            self._transition(State.C3_RETURN)
        else:
            self.get_logger().info('[VERIFY] 잔여 0 -> DONE')
            self._transition(State.DONE)

    def _run_c3_return(self) -> None:
        st = self._nav_step(self._return, self.return_dir, 'C3_RETURN')
        if st == 'arrived':
            self.get_logger().info(
                f'[C3_RETURN] 복귀({self.return_dir} {self.base_shift_mm:.0f}mm) -> C2_SCAN_POSE')
            self._transition(State.C2_SCAN_POSE)
        elif st == 'failed':
            self.get_logger().warning('[C3_RETURN] 복귀 실패 -> RECOVERY')
            self._transition(State.RECOVERY)
        elif self._timed_out():
            self.get_logger().warning('[C3_RETURN] 복귀 timeout -> RECOVERY')
            self._transition(State.RECOVERY)

    def _run_done(self) -> None:
        self.get_logger().info(f'[DONE] mission C 완료 (삽입 {self.placed_count}개)')
        self.timer.cancel()

    def _run_recovery(self) -> None:
        if self.recovery_count < MAX_RECOVERY_RETRY:
            self.recovery_count += 1
            self.get_logger().warning(
                f'[RECOVERY] 재시도 {self.recovery_count}/{MAX_RECOVERY_RETRY} -> C2_SCAN_POSE')
            self._transition(State.C2_SCAN_POSE)
        else:
            self.get_logger().error('[RECOVERY] 재시도 초과 -> MANUAL_WAIT')
            self._transition(State.MANUAL_WAIT)

    def _run_manual_wait(self) -> None:
        # 자동 복구 예산 소진 → 수동 개입 대기. 영구 무반응으로 보이지 않도록 주기 로그.
        self.get_logger().error(
            '[MANUAL_WAIT] 자동 복구 예산 소진 — 수동 개입 대기(정지). '
            'dry-run(insert_dry_run:=true) 으로 전 사이클 시험 가능.',
            throttle_duration_sec=10.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionC()
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
