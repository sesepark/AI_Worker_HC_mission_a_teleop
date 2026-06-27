#!/usr/bin/env python3
"""Mission D System FSM 노드.

이 노드는 Mission D의 gate/orchestrator만 담당한다. 인식, 주행 계획, 조작 궤적 생성,
IK, grasp planning, drill 제어 같은 알고리즘 내부는 각 팀 패키지의 책임으로 남기고,
여기서는 perception JSON을 받아 조건을 확인한 뒤 navigation service와 manipulation action을
순서대로 호출한다.
"""
from __future__ import annotations

import copy
import json
from dataclasses import dataclass, field
from enum import Enum, auto
from types import SimpleNamespace
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mission_interfaces.action import MissionDManipulation
from mission_interfaces.srv import MoveBaseRelative


DEFAULT_NAV_PATH_IDS = {
    'd1_wheel_align_left': 'd1_wheel_align_left',
    'wheel_align_to_tool_area': 'wheel_align_to_tool_area',
    'start_to_tool_area': 'start_to_tool_area',
    'tool_to_fixture': 'tool_to_fixture',
    'fixture_to_tool_area': 'fixture_to_tool_area',
    'd1_move_to_wheel_drop_space': 'd1_move_to_wheel_drop_space',
    'd1_return_from_wheel_drop_space': 'd1_return_from_wheel_drop_space',
    'd3_move_near_loose_bolt_front': 'd3_move_near_loose_bolt_front',
    'd3_move_near_loose_bolt_left': 'd3_move_near_loose_bolt_left',
    'd3_move_near_loose_bolt_right': 'd3_move_near_loose_bolt_right',
    'd3_move_to_bolt_drop_space': 'd3_move_to_bolt_drop_space',
    'return_to_start_pose': 'return_to_start_pose',
}

LOOSE_BOLT_REGIONS = {'front', 'left', 'right'}

BASE_START = 'START'
BASE_TOOL_AREA = 'TOOL_AREA'
BASE_FIXTURE = 'FIXTURE'
BASE_WHEEL_ALIGN = 'WHEEL_ALIGN'
BASE_NEAR_LOOSE_BOLT = 'NEAR_LOOSE_BOLT'
BASE_BOLT_DROP_SPACE = 'BOLT_DROP_SPACE'


class State(Enum):
    INIT = auto()
    D1_DETECT_WHEEL = auto()
    D1_PLAN_WHEEL_GRASP = auto()
    D1_GRASP_WHEEL_RIGHT = auto()
    D1_MOVE_BASE_LEFT_FOR_WHEEL_ALIGN = auto()
    D1_DETECT_WHEEL_FIXTURE_CENTER = auto()
    D1_ALIGN_WHEEL_TO_FIXTURE = auto()
    D1_INSERT_WHEEL_FORWARD = auto()
    D1_VERIFY_WHEEL_INSERTION = auto()
    D1_MOVE_TO_WHEEL_DROP_SPACE = auto()
    D1_RELEASE_WHEEL_TO_FLOOR = auto()
    D1_WAIT_AFTER_WHEEL_DROP = auto()
    D1_RETURN_FROM_WHEEL_DROP_SPACE = auto()
    D1_MARK_WHEEL_DISABLED = auto()
    D2_MOVE_TO_TOOL_AREA = auto()
    D2_DETECT_BOLT = auto()
    D2_PLAN_BOLT_GRASP = auto()
    D2_GRASP_BOLT_LEFT = auto()
    D2_RECOVER_BOLT_GRASP_FAILURE = auto()
    D2_DROP_FAILED_GRASP_BOLT_CANDIDATE = auto()
    D2_WAIT_AFTER_FAILED_BOLT_DROP = auto()
    D2_DETECT_DRILL = auto()
    D2_PLAN_DRILL_GRASP = auto()
    D2_GRASP_DRILL_RIGHT = auto()
    D2_HANDLE_TOOL_HOLDING_RESULT = auto()
    D3_MOVE_TO_FIXTURE = auto()
    D3_DETECT_FIXTURE_CENTER = auto()
    D3_PLAN_BOLT_INSERT = auto()
    D3_ALIGN_BOLT_TO_FIXTURE = auto()
    D3_INSERT_BOLT_LEFT = auto()
    D3_VERIFY_BOLT_INSERTED = auto()
    D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL = auto()
    D3_MOVE_NEAR_LOOSE_BOLT = auto()
    D3_GRASP_LOOSE_BOLT_CANDIDATE = auto()
    D3_MOVE_TO_BOLT_DROP_SPACE = auto()
    D3_RELEASE_LOOSE_BOLT_TO_FLOOR = auto()
    D3_WAIT_AFTER_LOOSE_BOLT_DROP = auto()
    D3_WAIT_FOR_BOLT_REAPPEAR = auto()
    D3_RETURN_TO_START_POSE = auto()
    D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT = auto()
    D4_DETECT_DRILL_AFTER_BOLT_INSERT = auto()
    D4_PLAN_DRILL_GRASP_AFTER_BOLT_INSERT = auto()
    D4_GRASP_DRILL_RIGHT_AFTER_BOLT_INSERT = auto()
    D4_MOVE_TO_FIXTURE_WITH_DRILL_AFTER_BOLT_INSERT = auto()
    D4_PLAN_DRILL_FASTEN = auto()
    D4_FASTEN_WITH_DRILL = auto()
    D4_VERIFY_FASTENING = auto()
    D4_RETURN_INITIAL_FOR_DRILL_RETRY = auto()
    MANUAL_WAIT = auto()
    DONE = auto()


ACTION_STATES = {
    State.D1_PLAN_WHEEL_GRASP,
    State.D1_INSERT_WHEEL_FORWARD,
    State.D1_RELEASE_WHEEL_TO_FLOOR,
    State.D2_PLAN_BOLT_GRASP,
    State.D2_DROP_FAILED_GRASP_BOLT_CANDIDATE,
    State.D2_PLAN_DRILL_GRASP,
    State.D3_INSERT_BOLT_LEFT,
    State.D3_GRASP_LOOSE_BOLT_CANDIDATE,
    State.D3_RELEASE_LOOSE_BOLT_TO_FLOOR,
    State.D4_PLAN_DRILL_GRASP_AFTER_BOLT_INSERT,
    State.D4_FASTEN_WITH_DRILL,
    State.D4_RETURN_INITIAL_FOR_DRILL_RETRY,
}

NAV_STATES = {
    State.D1_MOVE_BASE_LEFT_FOR_WHEEL_ALIGN,
    State.D1_MOVE_TO_WHEEL_DROP_SPACE,
    State.D1_RETURN_FROM_WHEEL_DROP_SPACE,
    State.D2_MOVE_TO_TOOL_AREA,
    State.D3_MOVE_TO_FIXTURE,
    State.D3_MOVE_NEAR_LOOSE_BOLT,
    State.D3_MOVE_TO_BOLT_DROP_SPACE,
    State.D3_RETURN_TO_START_POSE,
    State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT,
    State.D4_MOVE_TO_FIXTURE_WITH_DRILL_AFTER_BOLT_INSERT,
}

PRIMARY_POSE_SKILLS = {
    'GRASP_WHEEL_RIGHT',
    'INSERT_WHEEL_FORWARD',
    'GRASP_BOLT_LEFT',
    'DROP_FAILED_GRASP_BOLT_CANDIDATE',
    'GRASP_DRILL_RIGHT',
    'INSERT_BOLT_LEFT',
    'FASTEN_WITH_DRILL',
}

SNAPSHOT_RESET_STATES = {
    State.D1_DETECT_WHEEL,
    State.D1_PLAN_WHEEL_GRASP,
    State.D1_DETECT_WHEEL_FIXTURE_CENTER,
    State.D2_DETECT_BOLT,
    State.D2_PLAN_BOLT_GRASP,
    State.D2_DETECT_DRILL,
    State.D2_PLAN_DRILL_GRASP,
    State.D3_DETECT_FIXTURE_CENTER,
    State.D3_PLAN_BOLT_INSERT,
    State.D4_DETECT_DRILL_AFTER_BOLT_INSERT,
    State.D4_PLAN_DRILL_GRASP_AFTER_BOLT_INSERT,
    State.D4_PLAN_DRILL_FASTEN,
}


@dataclass
class AsyncLatch:
    """비동기 service/action을 state 진입마다 한 번만 보내기 위한 래치."""

    sent: bool = False
    goal_handle: object | None = None
    result: object | None = None
    done: bool = False
    accepted: bool = False
    meta: dict[str, Any] = field(default_factory=dict)

    def reset(self) -> None:
        self.sent = False
        self.goal_handle = None
        self.result = None
        self.done = False
        self.accepted = False
        self.meta = {}


class MissionD(Node):
    """Mission D 상태, gate 조건, retry/recovery를 조율하는 ROS 2 Node."""

    def __init__(self) -> None:
        super().__init__('mission_d')

        self.nav_service_name = str(
            self.declare_parameter('nav_service_name', 'move_base_relative').value)
        self.nav_service_wait_sec = float(
            self.declare_parameter('nav_service_wait_sec', 10.0).value)
        self.manipulation_action_name = str(
            self.declare_parameter('manipulation_action_name', '/mission_d/manipulation').value)

        self.use_test_env = bool(self.declare_parameter('use_test_env', True).value)
        self.test_env_required = bool(self.declare_parameter('test_env_required', True).value)
        self.setup_environment_service_name = str(
            self.declare_parameter(
                'setup_environment_service_name', '/mission_d/setup_environment').value)
        self.test_env_timeout_sec = float(
            self.declare_parameter('test_env_timeout_sec', 60.0).value)

        self.timeout_init_sec = float(self.declare_parameter('timeout_init_sec', 60.0).value)
        self.timeout_perception_sec = float(
            self.declare_parameter('timeout_perception_sec', 12.0).value)
        self.timeout_planning_sec = float(
            self.declare_parameter('timeout_planning_sec', 8.0).value)
        self.timeout_navigation_sec = float(
            self.declare_parameter('timeout_navigation_sec', 35.0).value)
        self.timeout_wheel_grasp_sec = float(
            self.declare_parameter('timeout_wheel_grasp_sec', 30.0).value)
        self.timeout_wheel_manip_sec = float(
            self.declare_parameter('timeout_wheel_manip_sec', 25.0).value)
        self.timeout_wheel_drop_move_sec = float(
            self.declare_parameter('timeout_wheel_drop_move_sec', 20.0).value)
        self.timeout_wheel_release_sec = float(
            self.declare_parameter('timeout_wheel_release_sec', 8.0).value)
        self.wheel_drop_wait_sec = float(
            self.declare_parameter('wheel_drop_wait_sec', 30.0).value)
        self.timeout_bolt_grasp_sec = float(
            self.declare_parameter('timeout_bolt_grasp_sec', 25.0).value)
        self.timeout_failed_bolt_drop_sec = float(
            self.declare_parameter('timeout_failed_bolt_drop_sec', 20.0).value)
        self.failed_bolt_drop_wait_sec = float(
            self.declare_parameter('failed_bolt_drop_wait_sec', 30.0).value)
        self.timeout_drill_grasp_sec = float(
            self.declare_parameter('timeout_drill_grasp_sec', 25.0).value)
        self.timeout_bolt_insert_sec = float(
            self.declare_parameter('timeout_bolt_insert_sec', 30.0).value)
        self.timeout_verify_wheel_inserted_sec = float(
            self.declare_parameter('timeout_verify_wheel_inserted_sec', 5.0).value)
        self.timeout_verify_bolt_inserted_sec = float(
            self.declare_parameter('timeout_verify_bolt_inserted_sec', 12.0).value)
        self.timeout_verify_fastening_sec = float(
            self.declare_parameter('timeout_verify_fastening_sec', 3.0).value)
        self.bolt_visible_hold_sec = float(
            self.declare_parameter('bolt_visible_hold_sec', 4.0).value)
        self.wait_for_bolt_reappear_sec = float(
            self.declare_parameter('wait_for_bolt_reappear_sec', 30.0).value)
        self.manual_wait_sec = float(
            self.declare_parameter('manual_wait_sec', 5.0).value)
        self.timeout_drill_fasten_sec = float(
            self.declare_parameter('timeout_drill_fasten_sec', 30.0).value)
        self.timeout_drill_retry_return_sec = float(
            self.declare_parameter('timeout_drill_retry_return_sec', 25.0).value)

        self.max_wheel_detection_attempts = int(
            self.declare_parameter('max_wheel_detection_attempts', 3).value)
        self.max_wheel_grasp_attempts = int(
            self.declare_parameter('max_wheel_grasp_attempts', 3).value)
        self.max_bolt_grasp_attempts = int(
            self.declare_parameter('max_bolt_grasp_attempts', 5).value)
        self.max_drill_grasp_attempts = int(
            self.declare_parameter('max_drill_grasp_attempts', 3).value)
        self.max_fastening_attempts = int(
            self.declare_parameter('max_fastening_attempts', 3).value)
        self.max_loose_bolt_nav_failures = int(
            self.declare_parameter('max_loose_bolt_nav_failures', 3).value)

        self.nav_path_ids = self._json_param('nav_path_ids_json', DEFAULT_NAV_PATH_IDS)

        self.state_timeout = self._build_state_timeout()
        self._cbg = ReentrantCallbackGroup()

        # DDS discovery 안정성을 위해 외부 기능 client는 subscriber보다 먼저 만든다.
        self._nav_cli = self.create_client(
            MoveBaseRelative, self.nav_service_name, callback_group=self._cbg)
        self._manip_cli = ActionClient(
            self, MissionDManipulation, self.manipulation_action_name,
            callback_group=self._cbg)
        self._setup_cli = None
        if self.use_test_env:
            self._setup_cli = self.create_client(
                Trigger, self.setup_environment_service_name, callback_group=self._cbg)

        self.create_subscription(
            String, '/mission_d/perception/wheel', self._on_wheel, 10,
            callback_group=self._cbg)
        self.create_subscription(
            String, '/mission_d/perception/wheel_fixture', self._on_wheel_fixture, 10,
            callback_group=self._cbg)
        self.create_subscription(
            String, '/mission_d/perception/tools', self._on_tools, 10,
            callback_group=self._cbg)
        self.create_subscription(
            String, '/mission_d/perception/fixture', self._on_fixture, 10,
            callback_group=self._cbg)
        self.create_subscription(
            String, '/mission_d/perception/verification', self._on_verification, 10,
            callback_group=self._cbg)

        self.pub_active_mission = self.create_publisher(String, '/active_mission', 10)
        self.pub_state = self.create_publisher(String, '/mission_d/state', 10)
        self.pub_status = self.create_publisher(String, '/mission_d/status', 10)

        self.state = State.INIT
        self._state_enter_time = self._now()
        self._done_logged = False
        self._state_phase = ''
        self._state_phase_enter_time = self._state_enter_time

        self.wheel_disabled = False
        self.wheel_done = False
        self.right_hand = ''
        self.left_hand = ''
        self.base_context = BASE_START
        self.bolt_grasped = False
        self.drill_grasped = False
        self.bolt_inserted = False
        self.fastened = False

        self.wheel_detection_attempts = 0
        self.wheel_grasp_attempts = 0
        self.bolt_grasp_attempts = 0
        self.drill_grasp_attempts = 0
        self.fastening_attempts = 0
        self.loose_bolt_nav_failures = 0
        self.loose_bolt_region = ''
        self.loose_bolt_unreachable_count = 0
        self.total_bolt_grasp_failures = 0
        self.total_drill_grasp_failures = 0

        self.nav_path_failures: dict[str, int] = {}
        self.current_nav_path: dict[str, Any] | None = None
        self.current_tool_path_id = ''
        self.current_fixture_path_id = ''
        self.last_nav_result: object | None = None
        self.last_wheel_insert_nav_result: object | None = None

        self.latest_wheel: dict[str, Any] | None = None
        self.latest_wheel_fixture: dict[str, Any] | None = None
        self.latest_tools: dict[str, Any] | None = None
        self.latest_fixture: dict[str, Any] | None = None
        self.latest_verification: dict[str, Any] | None = None
        self.planning_snapshot: dict[str, Any] | None = None
        self.drill_original_pose: dict[str, Any] | None = None
        self.last_base_motion_time = self._state_enter_time

        self.last_failure_reason = ''
        self.status_notes: list[str] = []
        self._bolt_visible_since: float | None = None
        self._after_failed_bolt_drop_state = State.D2_DETECT_BOLT
        self._after_return_to_start_state = State.D2_MOVE_TO_TOOL_AREA
        self._after_manual_wait_state = State.D2_MOVE_TO_TOOL_AREA
        self._drill_reacquire_only = False

        self._setup = AsyncLatch()
        self._nav = AsyncLatch()
        self._manip = AsyncLatch()
        self._in_tick = False

        self.timer = self.create_timer(0.1, self._tick, callback_group=self._cbg)
        self.get_logger().info(
            f'mission_d started service={self.nav_service_name} '
            f'action={self.manipulation_action_name}')

    def _build_state_timeout(self) -> dict[State, float | None]:
        return {
            State.INIT: max(self.timeout_init_sec, self.test_env_timeout_sec),
            State.D1_DETECT_WHEEL: self.timeout_perception_sec,
            State.D1_PLAN_WHEEL_GRASP: self.timeout_planning_sec,
            State.D1_GRASP_WHEEL_RIGHT: self.timeout_wheel_grasp_sec,
            State.D1_MOVE_BASE_LEFT_FOR_WHEEL_ALIGN: self.timeout_navigation_sec,
            State.D1_DETECT_WHEEL_FIXTURE_CENTER: self.timeout_perception_sec,
            State.D1_ALIGN_WHEEL_TO_FIXTURE: self.timeout_wheel_manip_sec,
            State.D1_INSERT_WHEEL_FORWARD: self.timeout_wheel_manip_sec,
            State.D1_VERIFY_WHEEL_INSERTION: self.timeout_verify_wheel_inserted_sec,
            State.D1_MOVE_TO_WHEEL_DROP_SPACE: self.timeout_wheel_drop_move_sec,
            State.D1_RELEASE_WHEEL_TO_FLOOR: self.timeout_wheel_release_sec,
            State.D1_WAIT_AFTER_WHEEL_DROP: self.wheel_drop_wait_sec,
            State.D1_RETURN_FROM_WHEEL_DROP_SPACE: self.timeout_wheel_drop_move_sec,
            State.D2_MOVE_TO_TOOL_AREA: self.timeout_navigation_sec,
            State.D2_DETECT_BOLT: self.timeout_perception_sec,
            State.D2_PLAN_BOLT_GRASP: self.timeout_planning_sec,
            State.D2_GRASP_BOLT_LEFT: self.timeout_bolt_grasp_sec,
            State.D2_RECOVER_BOLT_GRASP_FAILURE: self.timeout_perception_sec,
            State.D2_DROP_FAILED_GRASP_BOLT_CANDIDATE: self.timeout_failed_bolt_drop_sec,
            State.D2_WAIT_AFTER_FAILED_BOLT_DROP: self.failed_bolt_drop_wait_sec,
            State.D2_DETECT_DRILL: self.timeout_perception_sec,
            State.D2_PLAN_DRILL_GRASP: self.timeout_planning_sec,
            State.D2_GRASP_DRILL_RIGHT: self.timeout_drill_grasp_sec,
            State.D3_MOVE_TO_FIXTURE: self.timeout_navigation_sec,
            State.D3_DETECT_FIXTURE_CENTER: self.timeout_perception_sec,
            State.D3_PLAN_BOLT_INSERT: self.timeout_planning_sec,
            State.D3_ALIGN_BOLT_TO_FIXTURE: self.timeout_bolt_insert_sec,
            State.D3_INSERT_BOLT_LEFT: self.timeout_bolt_insert_sec,
            State.D3_VERIFY_BOLT_INSERTED: self.timeout_verify_bolt_inserted_sec,
            State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL: self.timeout_perception_sec,
            State.D3_MOVE_NEAR_LOOSE_BOLT: self.timeout_navigation_sec,
            State.D3_GRASP_LOOSE_BOLT_CANDIDATE: self.timeout_bolt_grasp_sec,
            State.D3_MOVE_TO_BOLT_DROP_SPACE: self.timeout_navigation_sec,
            State.D3_RELEASE_LOOSE_BOLT_TO_FLOOR: self.timeout_failed_bolt_drop_sec,
            State.D3_WAIT_AFTER_LOOSE_BOLT_DROP: self.failed_bolt_drop_wait_sec,
            State.D3_WAIT_FOR_BOLT_REAPPEAR: self.wait_for_bolt_reappear_sec,
            State.D3_RETURN_TO_START_POSE: self.timeout_navigation_sec,
            State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT: self.timeout_navigation_sec,
            State.D4_DETECT_DRILL_AFTER_BOLT_INSERT: self.timeout_perception_sec,
            State.D4_PLAN_DRILL_GRASP_AFTER_BOLT_INSERT: self.timeout_planning_sec,
            State.D4_GRASP_DRILL_RIGHT_AFTER_BOLT_INSERT: self.timeout_drill_grasp_sec,
            State.D4_MOVE_TO_FIXTURE_WITH_DRILL_AFTER_BOLT_INSERT: self.timeout_navigation_sec,
            State.D4_PLAN_DRILL_FASTEN: self.timeout_planning_sec,
            State.D4_FASTEN_WITH_DRILL: self.timeout_drill_fasten_sec,
            State.D4_VERIFY_FASTENING: self.timeout_verify_fastening_sec,
            State.D4_RETURN_INITIAL_FOR_DRILL_RETRY: self.timeout_drill_retry_return_sec,
            State.MANUAL_WAIT: self.manual_wait_sec,
        }

    def _json_param(self, name: str, default: Any) -> Any:
        value = self.declare_parameter(name, json.dumps(default)).value
        if isinstance(value, str):
            try:
                return json.loads(value)
            except Exception:
                self.get_logger().warn(f'{name} 파싱 실패, 기본값 사용')
                return copy.deepcopy(default)
        return value

    def _nav_path_id(self, key: str) -> str:
        if isinstance(self.nav_path_ids, dict) and key in self.nav_path_ids:
            return str(self.nav_path_ids[key])
        return str(DEFAULT_NAV_PATH_IDS.get(key, key))

    def _store_json(self, msg: String, label: str) -> dict[str, Any] | None:
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'{label} JSON 파싱 실패: {exc}')
            return None
        if not isinstance(data, dict):
            self.get_logger().warn(f'{label} JSON payload가 object가 아님')
            return None
        data['recv_time'] = self._now()
        return data

    def _on_wheel(self, msg: String) -> None:
        data = self._store_json(msg, 'wheel')
        if data is not None:
            self.latest_wheel = data

    def _on_wheel_fixture(self, msg: String) -> None:
        data = self._store_json(msg, 'wheel_fixture')
        if data is not None:
            self.latest_wheel_fixture = data

    def _on_tools(self, msg: String) -> None:
        data = self._store_json(msg, 'tools')
        if data is not None:
            self.latest_tools = data

    def _on_fixture(self, msg: String) -> None:
        data = self._store_json(msg, 'fixture')
        if data is not None:
            self.latest_fixture = data

    def _on_verification(self, msg: String) -> None:
        data = self._store_json(msg, 'verification')
        if data is not None:
            self.latest_verification = data

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self) -> float:
        return self._now() - self._state_enter_time

    def _timed_out(self) -> bool:
        limit = self.state_timeout.get(self.state)
        return limit is not None and self._elapsed() >= limit

    def _phase_elapsed(self) -> float:
        return self._now() - self._state_phase_enter_time

    def _phase_timed_out(self, timeout_sec: float) -> bool:
        return self._phase_elapsed() >= timeout_sec

    def _servers_ready(self) -> bool:
        """INIT에서 즉시 필요한 외부 기능을 확인한다.

        Manipulation action server는 첫 planning/action state에서 곧바로 필요하므로 gate한다.
        Navigation service는 이후 state에서 자체 대기하므로 여기서는 non-blocking discovery만 건다.
        """
        if not self._manip_cli.server_is_ready():
            self._manip_cli.wait_for_server(timeout_sec=0.0)
            if not self._manip_cli.server_is_ready():
                return False
        if not self._nav_cli.service_is_ready():
            self._nav_cli.wait_for_service(timeout_sec=0.0)
        return True

    def _mark_base_context(self, context: str) -> None:
        self.base_context = context
        self.last_base_motion_time = self._now()

    def _fresh_since(self, data: dict[str, Any] | None, timestamp: float) -> bool:
        if not isinstance(data, dict):
            return False
        try:
            return float(data.get('recv_time', 0.0) or 0.0) >= timestamp
        except (TypeError, ValueError):
            return False

    def _fresh_after_base_motion(self, data: dict[str, Any] | None) -> bool:
        return self._fresh_since(data, self.last_base_motion_time)

    def _fresh_after_state_entry(self, data: dict[str, Any] | None) -> bool:
        return self._fresh_since(data, self._state_enter_time)

    def _publish_state(self) -> None:
        self.pub_active_mission.publish(String(data='D'))
        self.pub_state.publish(String(data=self.state.name))

    def _publish_status(self) -> None:
        payload = {
            'state': self.state.name,
            'state_elapsed_sec': round(self._elapsed(), 3),
            'state_phase': self._state_phase,
            'state_phase_elapsed_sec': round(self._phase_elapsed(), 3),
            'wheel_disabled': self.wheel_disabled,
            'wheel_done': self.wheel_done,
            'right_hand': self.right_hand,
            'left_hand': self.left_hand,
            'base_context': self.base_context,
            'bolt_grasped': self.bolt_grasped,
            'drill_grasped': self.drill_grasped,
            'bolt_inserted': self.bolt_inserted,
            'fastened': self.fastened,
            'wheel_detection_attempts': self.wheel_detection_attempts,
            'wheel_grasp_attempts': self.wheel_grasp_attempts,
            'bolt_grasp_attempts': self.bolt_grasp_attempts,
            'drill_grasp_attempts': self.drill_grasp_attempts,
            'fastening_attempts': self.fastening_attempts,
            'loose_bolt_nav_failures': self.loose_bolt_nav_failures,
            'loose_bolt_region': self.loose_bolt_region,
            'loose_bolt_unreachable_count': self.loose_bolt_unreachable_count,
            'total_bolt_grasp_failures': self.total_bolt_grasp_failures,
            'total_drill_grasp_failures': self.total_drill_grasp_failures,
            'nav_path_failures': self.nav_path_failures,
            'current_tool_path_id': self.current_tool_path_id,
            'current_fixture_path_id': self.current_fixture_path_id,
            'last_failure_reason': self.last_failure_reason,
            'done': self.state == State.DONE,
            'manual_wait': self.state == State.MANUAL_WAIT,
            'after_manual_wait_state': self._after_manual_wait_state.name,
            'status_notes': self.status_notes[-8:],
        }
        self.pub_status.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _manual_wait(self, next_state: State, reason: str | None = None) -> None:
        if reason:
            self.last_failure_reason = reason
        self._after_manual_wait_state = next_state
        self._transition(State.MANUAL_WAIT)

    def _transition(self, new_state: State) -> None:
        old_state = self.state
        elapsed = self._elapsed()
        phase = self._state_phase or '-'
        phase_elapsed = self._phase_elapsed()
        retry_note = ' retry_reset=true' if new_state == old_state else ''
        self.get_logger().info(
            f'[state] {old_state.name} -> {new_state.name}'
            f'{retry_note} elapsed={elapsed:.2f}s phase={phase}'
            f' phase_elapsed={phase_elapsed:.2f}s base={self.base_context}'
            f' hands=L:{self.left_hand or "-"},R:{self.right_hand or "-"}'
            f' flags=wheel:{self.wheel_done},bolt:{self.bolt_inserted},fasten:{self.fastened}'
            f' last_failure={self.last_failure_reason or "-"}')
        # Mission D는 같은 state로의 transition을 retry reset으로 사용한다.
        # Mission A처럼 same-state transition을 무시하면 nav/action latch 재무장이 깨진다.
        self.state = new_state
        self._state_enter_time = self._now()
        self._on_enter(new_state)
        self._publish_state()
        self._publish_status()

    def _on_enter(self, state: State) -> None:
        self._state_phase = ''
        self._state_phase_enter_time = self._state_enter_time
        if state == State.INIT:
            self._setup.reset()
        if state in SNAPSHOT_RESET_STATES:
            self.planning_snapshot = None
        if state in ACTION_STATES:
            self._manip.reset()
        if state in NAV_STATES:
            self._nav.reset()
        if state == State.D3_VERIFY_BOLT_INSERTED:
            self._bolt_visible_since = None
        if state == State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL:
            self.loose_bolt_nav_failures = 0
            self.loose_bolt_region = ''
        if state in (State.D3_PLAN_BOLT_INSERT, State.D3_INSERT_BOLT_LEFT):
            self.loose_bolt_unreachable_count = 0
        if state == State.D2_MOVE_TO_TOOL_AREA:
            self.bolt_grasp_attempts = 0
            self.drill_grasp_attempts = 0
        if state == State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT:
            self.drill_grasp_attempts = 0

    def _capture_snapshot(self, source_name: str) -> dict[str, Any] | None:
        sources = {
            'wheel': self.latest_wheel,
            'wheel_fixture': self.latest_wheel_fixture,
            'tools': self.latest_tools,
            'fixture': self.latest_fixture,
            'verification': self.latest_verification,
        }
        source = sources.get(source_name)
        self.planning_snapshot = copy.deepcopy(source) if source is not None else None
        return self.planning_snapshot

    def _point_is_valid(self, point: Any) -> bool:
        if not isinstance(point, dict):
            return False
        for key in ('x', 'y', 'z'):
            if key not in point:
                return False
            try:
                float(point[key])
            except (TypeError, ValueError):
                return False
        return True

    def _has_frame_id(self, data: dict[str, Any] | None) -> bool:
        return isinstance(data, dict) and isinstance(data.get('frame_id'), str) and bool(data['frame_id'])

    def _valid_top(self, data: dict[str, Any] | None, require_point: bool = True) -> bool:
        if not isinstance(data, dict) or data.get('valid') is not True:
            return False
        if not self._has_frame_id(data):
            return False
        if require_point and not self._point_is_valid(data.get('point')):
            return False
        return True

    def _valid_item(
        self,
        data: dict[str, Any] | None,
        item: str,
        require_point: bool = True,
    ) -> bool:
        if not self._valid_top(data, require_point=False):
            return False
        target = data.get(item)
        if not isinstance(target, dict) or target.get('valid') is not True:
            return False
        if require_point and not self._point_is_valid(target.get('point')):
            return False
        return True

    def _valid_loose_bolt_snapshot(self, data: dict[str, Any] | None) -> bool:
        return (
            self._valid_item(data, 'bolt')
            or self._valid_top(data)
            or (
                self._valid_top(data, require_point=False)
                and self._point_is_valid((data or {}).get('bolt_point'))
            )
        )

    def _valid_verification(self, data: dict[str, Any] | None) -> bool:
        return isinstance(data, dict) and data.get('valid') is True

    def _loose_bolt_region(self, data: dict[str, Any]) -> str:
        region = str(
            data.get('loose_bolt_region')
            or data.get('bolt_region')
            or ''
        ).strip().lower()
        aliases = {
            'center': 'front',
            'middle': 'front',
            'table_front': 'front',
            'table_left': 'left',
            'table_right': 'right',
            'not_reachable': 'unreachable',
            'out_of_reach': 'unreachable',
            'none': 'unknown',
            '': 'unknown',
        }
        return aliases.get(region, region)

    def _plan_success(self, data: dict[str, Any] | None) -> bool:
        return isinstance(data, dict) and data.get('plan_success') is True

    def _path(
        self,
        path_id: str,
        timeout_sec: float | None = None,
    ) -> dict[str, Any]:
        # System은 실제 상대 이동량을 소유하지 않고, navigation이 path_id를 해석한다.
        return {
            'path_id': path_id,
            'timeout_sec': float(timeout_sec or self.timeout_navigation_sec),
        }

    def _record_nav_failure(self, path: dict[str, Any] | None, reason: str) -> None:
        path_id = str((path or {}).get('path_id', 'unknown_path'))
        self.nav_path_failures[path_id] = self.nav_path_failures.get(path_id, 0) + 1
        self.last_failure_reason = reason
        self.get_logger().warn(f'[nav] {path_id} 실패 기록: {reason}')

    def _on_setup_result(self, future) -> None:
        try:
            self._setup.result = future.result()
        except Exception as exc:
            self._setup.result = SimpleNamespace(success=False, message=str(exc))
        self._setup.done = True

    def _on_nav_result(self, future) -> None:
        try:
            self._nav.result = future.result()
        except Exception as exc:
            self._nav.result = SimpleNamespace(
                arrived=False,
                within_expected_range=False,
                actual_translation_mm=0.0,
                actual_yaw_deg=0.0,
                message=str(exc),
            )
        self.last_nav_result = self._nav.result
        self._nav.done = True

    def _nav_step(
        self,
        path: dict[str, Any] | None,
        require_within_expected_range: bool = False,
    ) -> str:
        if path is None:
            return 'failed'
        if not self._nav.sent:
            if not self._nav_cli.service_is_ready():
                if not self._nav_cli.wait_for_service(
                    timeout_sec=min(self.nav_service_wait_sec, 0.5)
                ):
                    return 'pending'
            req = MoveBaseRelative.Request()
            req.path_id = str(path.get('path_id', ''))
            req.dx_mm = 0.0
            req.dy_mm = 0.0
            req.dyaw_deg = 0.0
            req.timeout_sec = float(path.get('timeout_sec', self.timeout_navigation_sec) or 0.0)
            self._nav.sent = True
            self._nav.meta = copy.deepcopy(path)
            self.get_logger().info(
                f'[{self.state.name}] MoveBaseRelative path={req.path_id} '
                f'timeout={req.timeout_sec:.1f}s')
            future = self._nav_cli.call_async(req)
            future.add_done_callback(self._on_nav_result)
            return 'pending'
        if not self._nav.done:
            return 'pending'
        result = self._nav.result
        arrived = bool(getattr(result, 'arrived', False))
        in_range = bool(getattr(result, 'within_expected_range', False))
        if arrived and (in_range or not require_within_expected_range):
            return 'arrived'
        return 'failed'

    def _pose_from_point(self, frame_id: str, point: dict[str, Any] | None) -> PoseStamped | None:
        if not frame_id or not self._point_is_valid(point):
            return None
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point.get('x', 0.0) or 0.0)
        pose.pose.position.y = float(point.get('y', 0.0) or 0.0)
        pose.pose.position.z = float(point.get('z', 0.0) or 0.0)
        pose.pose.orientation.w = 1.0
        return pose

    def _extract_point(self, data: dict[str, Any] | None, *keys: str) -> dict[str, Any] | None:
        if not isinstance(data, dict):
            return None
        for key in keys:
            value = data.get(key)
            if self._point_is_valid(value):
                return value
        return None

    def _pose_from_snapshot(self, data: dict[str, Any] | None, *keys: str) -> PoseStamped | None:
        frame_id = str(data.get('frame_id')) if self._has_frame_id(data) else ''
        return self._pose_from_point(frame_id, self._extract_point(data, *keys))

    def _pose_from_tool(self, data: dict[str, Any] | None, item: str) -> PoseStamped | None:
        frame_id = str(data.get('frame_id')) if self._has_frame_id(data) else ''
        nested = (data or {}).get(item, {})
        return self._pose_from_point(
            frame_id,
            self._extract_point(nested, 'point'),
        )

    def _pose_from_loose_bolt_snapshot(self, data: dict[str, Any] | None) -> PoseStamped | None:
        if self._valid_item(data, 'bolt'):
            return self._pose_from_tool(data, 'bolt')
        return self._pose_from_snapshot(data, 'point', 'bolt_point')

    def _send_manip_goal(
        self,
        skill_id: str,
        hand: str = '',
        primary_pose: PoseStamped | None = None,
        secondary_pose: PoseStamped | None = None,
        params: dict[str, Any] | None = None,
    ) -> str:
        if not self._manip.sent:
            if not self._manip_cli.server_is_ready():
                if not self._manip_cli.wait_for_server(timeout_sec=0.5):
                    return 'pending'
            if skill_id in PRIMARY_POSE_SKILLS and primary_pose is None:
                self._manip.sent = True
                self._manip.done = True
                self._manip.result = SimpleNamespace(
                    success=False,
                    result_code=f'{skill_id.lower()}_missing_primary_point',
                    message='perception point is missing or invalid',
                    result_json='{}',
                )
                return 'failed'
            goal = MissionDManipulation.Goal()
            goal.skill_id = skill_id
            goal.hand = hand
            goal.primary_pose = primary_pose if primary_pose is not None else PoseStamped()
            goal.secondary_pose = secondary_pose if secondary_pose is not None else PoseStamped()
            goal.params_json = json.dumps(params or {}, sort_keys=True)
            self._manip.sent = True
            self._manip.meta = {'skill_id': skill_id, 'params': params or {}}
            self.get_logger().info(f'[{self.state.name}] action goal={skill_id}')
            future = self._manip_cli.send_goal_async(
                goal, feedback_callback=self._on_manip_feedback)
            future.add_done_callback(self._on_manip_goal_response)
            return 'pending'
        if not self._manip.done:
            return 'pending'
        result = self._manip.result
        return 'success' if bool(getattr(result, 'success', False)) else 'failed'

    def _on_manip_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        if feedback.phase:
            self.status_notes.append(
                f'{self._manip.meta.get("skill_id", "")}:{feedback.phase}:{feedback.progress:.2f}')

    def _on_manip_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._manip.result = SimpleNamespace(
                success=False, result_code='goal_exception', message=str(exc), result_json='{}')
            self._manip.done = True
            return
        self._manip.goal_handle = goal_handle
        self._manip.accepted = bool(goal_handle.accepted)
        if not goal_handle.accepted:
            self._manip.result = SimpleNamespace(
                success=False, result_code='goal_rejected', message='goal rejected', result_json='{}')
            self._manip.done = True
            return
        goal_handle.get_result_async().add_done_callback(self._on_manip_result)

    def _on_manip_result(self, future) -> None:
        try:
            self._manip.result = future.result().result
        except Exception as exc:
            self._manip.result = SimpleNamespace(
                success=False, result_code='result_exception', message=str(exc), result_json='{}')
        self._manip.done = True

    def _cancel_manip_if_needed(self) -> None:
        if self._manip.goal_handle is None or self._manip.done:
            return
        try:
            self._manip.goal_handle.cancel_goal_async()
        except Exception:
            pass

    def _result_json(self) -> dict[str, Any]:
        try:
            return json.loads(getattr(self._manip.result, 'result_json', '{}') or '{}')
        except Exception:
            return {}

    def _apply_manip_result(self, skill_id: str) -> dict[str, Any]:
        data = self._result_json()
        if 'right_hand' in data:
            self.right_hand = str(data.get('right_hand') or '')
        if 'left_hand' in data:
            self.left_hand = str(data.get('left_hand') or '')
        if 'bolt_grasped' in data:
            self.bolt_grasped = bool(data.get('bolt_grasped'))
        if 'drill_grasped' in data:
            self.drill_grasped = bool(data.get('drill_grasped'))
        if 'bolt_inserted' in data:
            self.bolt_inserted = bool(data.get('bolt_inserted'))
        if 'fastened' in data:
            self.fastened = bool(data.get('fastened'))

        if skill_id == 'GRASP_WHEEL_RIGHT':
            self.right_hand = 'wheel'
        elif skill_id == 'RELEASE_WHEEL_TO_FLOOR':
            self.right_hand = ''
        elif skill_id == 'INSERT_WHEEL_FORWARD':
            self.right_hand = ''
        elif skill_id == 'GRASP_BOLT_LEFT':
            self.left_hand = 'bolt'
            self.bolt_grasped = True
        elif skill_id == 'GRASP_DRILL_RIGHT':
            self.right_hand = 'drill'
            self.drill_grasped = True
        elif skill_id == 'INSERT_BOLT_LEFT':
            self.left_hand = ''
            self.bolt_grasped = False
            self.bolt_inserted = False
        elif skill_id == 'DROP_FAILED_GRASP_BOLT_CANDIDATE':
            self.left_hand = ''
            self.bolt_grasped = False
        elif skill_id == 'RELEASE_BOLT_TO_FLOOR':
            self.left_hand = ''
            self.bolt_grasped = False
        elif skill_id == 'FASTEN_WITH_DRILL':
            self.fastened = True
        return data

    def _manip_failure_reason(self, fallback: str) -> str:
        result = self._manip.result
        reason = str(getattr(result, 'result_code', '') or fallback)
        self.last_failure_reason = reason
        return reason

    def _tick(self) -> None:
        if self._in_tick:
            return
        self._in_tick = True
        try:
            self._tick_once()
        finally:
            self._in_tick = False

    def _tick_once(self) -> None:
        self._publish_state()
        self._publish_status()
        handlers = {
            State.INIT: self._run_init,
            State.D1_DETECT_WHEEL: self._run_d1_detect_wheel,
            State.D1_PLAN_WHEEL_GRASP: self._run_d1_plan_wheel_grasp,
            State.D1_GRASP_WHEEL_RIGHT: self._run_d1_grasp_wheel_right,
            State.D1_MOVE_BASE_LEFT_FOR_WHEEL_ALIGN:
                self._run_d1_move_base_left_for_wheel_align,
            State.D1_DETECT_WHEEL_FIXTURE_CENTER:
                self._run_d1_detect_wheel_fixture_center,
            State.D1_ALIGN_WHEEL_TO_FIXTURE: self._run_d1_align_wheel_to_fixture,
            State.D1_INSERT_WHEEL_FORWARD: self._run_d1_insert_wheel_forward,
            State.D1_VERIFY_WHEEL_INSERTION: self._run_d1_verify_wheel_insertion,
            State.D1_MOVE_TO_WHEEL_DROP_SPACE:
                self._run_d1_move_to_wheel_drop_space,
            State.D1_RELEASE_WHEEL_TO_FLOOR: self._run_d1_release_wheel_to_floor,
            State.D1_WAIT_AFTER_WHEEL_DROP: self._run_d1_wait_after_wheel_drop,
            State.D1_RETURN_FROM_WHEEL_DROP_SPACE:
                self._run_d1_return_from_wheel_drop_space,
            State.D1_MARK_WHEEL_DISABLED: self._run_d1_mark_wheel_disabled,
            State.D2_MOVE_TO_TOOL_AREA: self._run_d2_move_to_tool_area,
            State.D2_DETECT_BOLT: self._run_d2_detect_bolt,
            State.D2_PLAN_BOLT_GRASP: self._run_d2_plan_bolt_grasp,
            State.D2_GRASP_BOLT_LEFT: self._run_d2_grasp_bolt_left,
            State.D2_RECOVER_BOLT_GRASP_FAILURE:
                self._run_d2_recover_bolt_grasp_failure,
            State.D2_DROP_FAILED_GRASP_BOLT_CANDIDATE:
                self._run_d2_drop_failed_grasp_bolt_candidate,
            State.D2_WAIT_AFTER_FAILED_BOLT_DROP:
                self._run_d2_wait_after_failed_bolt_drop,
            State.D2_DETECT_DRILL: self._run_d2_detect_drill,
            State.D2_PLAN_DRILL_GRASP: self._run_d2_plan_drill_grasp,
            State.D2_GRASP_DRILL_RIGHT: self._run_d2_grasp_drill_right,
            State.D2_HANDLE_TOOL_HOLDING_RESULT:
                self._run_d2_handle_tool_holding_result,
            State.D3_MOVE_TO_FIXTURE: self._run_d3_move_to_fixture,
            State.D3_DETECT_FIXTURE_CENTER: self._run_d3_detect_fixture_center,
            State.D3_PLAN_BOLT_INSERT: self._run_d3_plan_bolt_insert,
            State.D3_ALIGN_BOLT_TO_FIXTURE: self._run_d3_align_bolt_to_fixture,
            State.D3_INSERT_BOLT_LEFT: self._run_d3_insert_bolt_left,
            State.D3_VERIFY_BOLT_INSERTED: self._run_d3_verify_bolt_inserted,
            State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL:
                self._run_d3_check_loose_bolt_after_insert_fail,
            State.D3_MOVE_NEAR_LOOSE_BOLT: self._run_d3_move_near_loose_bolt,
            State.D3_GRASP_LOOSE_BOLT_CANDIDATE:
                self._run_d3_grasp_loose_bolt_candidate,
            State.D3_MOVE_TO_BOLT_DROP_SPACE:
                self._run_d3_move_to_bolt_drop_space,
            State.D3_RELEASE_LOOSE_BOLT_TO_FLOOR:
                self._run_d3_release_loose_bolt_to_floor,
            State.D3_WAIT_AFTER_LOOSE_BOLT_DROP:
                self._run_d3_wait_after_loose_bolt_drop,
            State.D3_WAIT_FOR_BOLT_REAPPEAR:
                self._run_d3_wait_for_bolt_reappear,
            State.D3_RETURN_TO_START_POSE: self._run_d3_return_to_start_pose,
            State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT:
                self._run_d4_move_back_to_tool_for_drill_after_bolt_insert,
            State.D4_DETECT_DRILL_AFTER_BOLT_INSERT:
                self._run_d4_detect_drill_after_bolt_insert,
            State.D4_PLAN_DRILL_GRASP_AFTER_BOLT_INSERT:
                self._run_d4_plan_drill_grasp_after_bolt_insert,
            State.D4_GRASP_DRILL_RIGHT_AFTER_BOLT_INSERT:
                self._run_d4_grasp_drill_right_after_bolt_insert,
            State.D4_MOVE_TO_FIXTURE_WITH_DRILL_AFTER_BOLT_INSERT:
                self._run_d4_move_to_fixture_with_drill_after_bolt_insert,
            State.D4_PLAN_DRILL_FASTEN: self._run_d4_plan_drill_fasten,
            State.D4_FASTEN_WITH_DRILL: self._run_d4_fasten_with_drill,
            State.D4_VERIFY_FASTENING: self._run_d4_verify_fastening,
            State.D4_RETURN_INITIAL_FOR_DRILL_RETRY:
                self._run_d4_return_initial_for_drill_retry,
            State.MANUAL_WAIT: self._run_manual_wait,
            State.DONE: self._run_done,
        }
        handlers[self.state]()

    def _run_init(self) -> None:
        # test_env는 장애물/계획 장면을 준비하는 안전 gate다. 이 확인이 끝나기 전에는
        # 로봇 이동 명령을 내리지 않는다.
        if not self._servers_ready():
            self.get_logger().warn(
                '[INIT] manipulation action server 준비 대기 중 '
                '(navigation service discovery warm-up 병행)',
                throttle_duration_sec=2.0)
            if self._timed_out():
                self.last_failure_reason = 'dependency_server_timeout'
                self._manual_wait(State.INIT)
            return
        if not self.use_test_env:
            self._transition(State.D1_DETECT_WHEEL)
            return
        if self._setup_cli is None:
            self.last_failure_reason = 'setup_client_missing'
            self._manual_wait(State.INIT) if self.test_env_required else self._transition(
                State.D1_DETECT_WHEEL)
            return
        if not self._setup.sent:
            if not self._setup_cli.service_is_ready():
                self._setup_cli.wait_for_service(timeout_sec=0.0)
                if self._timed_out():
                    self.last_failure_reason = 'setup_environment_service_timeout'
                    self._manual_wait(State.INIT) if self.test_env_required else self._transition(
                        State.D1_DETECT_WHEEL)
                return
            self._setup.sent = True
            self._setup_cli.call_async(Trigger.Request()).add_done_callback(self._on_setup_result)
            return
        if not self._setup.done:
            if self._timed_out():
                self.last_failure_reason = 'setup_environment_call_timeout'
                self._manual_wait(State.INIT) if self.test_env_required else self._transition(
                    State.D1_DETECT_WHEEL)
            return
        if bool(getattr(self._setup.result, 'success', False)):
            self._transition(State.D1_DETECT_WHEEL)
            return
        self.last_failure_reason = str(getattr(self._setup.result, 'message', 'setup_failed'))
        self._manual_wait(State.INIT) if self.test_env_required else self._transition(
            State.D1_DETECT_WHEEL)

    def _wheel_detection_failed(self, reason: str) -> None:
        self.wheel_detection_attempts += 1
        self.last_failure_reason = reason
        if self.right_hand == 'wheel':
            self._transition(State.D1_MOVE_TO_WHEEL_DROP_SPACE)
            return
        if self.wheel_detection_attempts >= self.max_wheel_detection_attempts:
            self._transition(State.D1_MARK_WHEEL_DISABLED)
        else:
            self._transition(State.D1_DETECT_WHEEL)

    def _wheel_grasp_failed(self, reason: str) -> None:
        self.wheel_grasp_attempts += 1
        self.last_failure_reason = reason
        if self.right_hand == 'wheel':
            self._transition(State.D1_MOVE_TO_WHEEL_DROP_SPACE)
            return
        if self.wheel_grasp_attempts >= self.max_wheel_grasp_attempts:
            self._transition(State.D1_MARK_WHEEL_DISABLED)
        else:
            self._transition(State.D1_DETECT_WHEEL)

    def _run_d1_detect_wheel(self) -> None:
        # wheel_disabled가 한 번 true가 되면 이후 D2/D3/D4 재시작에서도 wheel은 다시 시도하지 않는다.
        if self.wheel_disabled or self.wheel_done:
            self._transition(State.D2_MOVE_TO_TOOL_AREA)
            return
        if self._valid_top(self.latest_wheel):
            self._transition(State.D1_PLAN_WHEEL_GRASP)
        elif self._timed_out():
            self._wheel_detection_failed('wheel_detection_timeout')

    def _run_d1_plan_wheel_grasp(self) -> None:
        snapshot = self.planning_snapshot or self._capture_snapshot('wheel')
        if not self._valid_top(snapshot) or not self._plan_success(snapshot):
            if self._timed_out() or snapshot is not None:
                self._wheel_detection_failed('wheel_plan_input_invalid')
            return
        status = self._send_manip_goal(
            'GRASP_WHEEL_RIGHT',
            hand='right',
            primary_pose=self._pose_from_snapshot(snapshot, 'point', 'wheel_center', 'center'),
            params={'snapshot': snapshot},
        )
        if status == 'success':
            self._apply_manip_result('GRASP_WHEEL_RIGHT')
            self._transition(State.D1_GRASP_WHEEL_RIGHT)
        elif status == 'failed':
            self._wheel_grasp_failed(self._manip_failure_reason('wheel_grasp_failed'))
        elif self._timed_out():
            self._cancel_manip_if_needed()
            self._wheel_grasp_failed('wheel_grasp_timeout')

    def _run_d1_grasp_wheel_right(self) -> None:
        if self.right_hand == 'wheel':
            self._transition(State.D1_MOVE_BASE_LEFT_FOR_WHEEL_ALIGN)
        else:
            self._wheel_grasp_failed('wheel_grasp_state_missing')

    def _run_d1_move_base_left_for_wheel_align(self) -> None:
        path = self._path(
            self._nav_path_id('d1_wheel_align_left'),
            timeout_sec=self.timeout_navigation_sec,
        )
        status = self._nav_step(path)
        if status == 'arrived':
            self._mark_base_context(BASE_WHEEL_ALIGN)
            self._transition(State.D1_DETECT_WHEEL_FIXTURE_CENTER)
        elif status == 'failed':
            self._record_nav_failure(path, 'wheel_left_align_nav_failed')
            self._wheel_grasp_failed('wheel_left_align_nav_failed')
        elif self._timed_out():
            self._record_nav_failure(path, 'wheel_left_align_timeout')
            self._manual_wait(State.D1_MOVE_BASE_LEFT_FOR_WHEEL_ALIGN)

    def _run_d1_detect_wheel_fixture_center(self) -> None:
        if (
            self._valid_top(self.latest_wheel_fixture)
            and self._fresh_after_base_motion(self.latest_wheel_fixture)
        ):
            self._transition(State.D1_ALIGN_WHEEL_TO_FIXTURE)
        elif self._timed_out():
            self._wheel_detection_failed('wheel_fixture_detection_timeout')

    def _run_d1_align_wheel_to_fixture(self) -> None:
        if self.right_hand != 'wheel':
            self._wheel_grasp_failed('wheel_missing_before_insert')
            return
        snapshot = self.planning_snapshot or self._capture_snapshot('wheel_fixture')
        if (
            not self._valid_top(snapshot)
            or not self._plan_success(snapshot)
            or not self._fresh_after_base_motion(snapshot)
        ):
            if self._timed_out() or snapshot is not None:
                self._wheel_grasp_failed('wheel_insert_plan_input_invalid')
            return
        self._transition(State.D1_INSERT_WHEEL_FORWARD)

    def _run_d1_insert_wheel_forward(self) -> None:
        # center align과 wheel 삽입 전 과정은 manipulation skill 하나가 수행한다.
        # 실패하면 같은 wheel을 다시 밀지 않고 drop한다.
        snapshot = self.planning_snapshot
        status = self._send_manip_goal(
            'INSERT_WHEEL_FORWARD',
            hand='right',
            primary_pose=self._pose_from_snapshot(snapshot, 'point', 'fixture_center', 'center'),
            params={'snapshot': snapshot},
        )
        if status == 'success':
            self._apply_manip_result('INSERT_WHEEL_FORWARD')
            self.last_wheel_insert_nav_result = SimpleNamespace(
                arrived=True,
                within_expected_range=True,
                actual_translation_mm=0.0,
                actual_yaw_deg=0.0,
                message='wheel inserted by manipulation',
            )
            self._transition(State.D1_VERIFY_WHEEL_INSERTION)
        elif status == 'failed':
            self.last_failure_reason = self._manip_failure_reason('wheel_insert_failed')
            self._transition(State.D1_MOVE_TO_WHEEL_DROP_SPACE)
        elif self._timed_out():
            self._cancel_manip_if_needed()
            self.last_failure_reason = 'wheel_insert_timeout'
            self._transition(State.D1_MOVE_TO_WHEEL_DROP_SPACE)

    def _run_d1_verify_wheel_insertion(self) -> None:
        result = self.last_wheel_insert_nav_result
        arrived = bool(getattr(result, 'arrived', False))
        in_range = bool(getattr(result, 'within_expected_range', False))
        if arrived and in_range:
            self.wheel_done = True
            self.right_hand = ''
            self._transition(State.D2_MOVE_TO_TOOL_AREA)
            return
        self.last_failure_reason = 'wheel_insert_expected_range_failed'
        self._transition(State.D1_MOVE_TO_WHEEL_DROP_SPACE)

    def _run_d1_move_to_wheel_drop_space(self) -> None:
        path = self._path(
            self._nav_path_id('d1_move_to_wheel_drop_space'),
            timeout_sec=self.timeout_wheel_drop_move_sec,
        )
        status = self._nav_step(path)
        if status == 'arrived':
            self._transition(State.D1_RELEASE_WHEEL_TO_FLOOR)
        elif status == 'failed':
            self._record_nav_failure(path, 'wheel_drop_move_failed')
            self._manual_wait(State.D1_MOVE_TO_WHEEL_DROP_SPACE)
        elif self._timed_out():
            self._record_nav_failure(path, 'wheel_drop_move_timeout')
            self._manual_wait(State.D1_MOVE_TO_WHEEL_DROP_SPACE)

    def _run_d1_release_wheel_to_floor(self) -> None:
        # wheel release 이후에는 환경 안정 시간을 둔 뒤 자동으로 복귀한다.
        status = self._send_manip_goal('RELEASE_WHEEL_TO_FLOOR', hand='right')
        if status == 'success':
            self._apply_manip_result('RELEASE_WHEEL_TO_FLOOR')
            self._transition(State.D1_WAIT_AFTER_WHEEL_DROP)
        elif status == 'failed' or self._timed_out():
            self.right_hand = ''
            self.last_failure_reason = self._manip_failure_reason('wheel_release_failed')
            self._transition(State.D1_WAIT_AFTER_WHEEL_DROP)

    def _run_d1_wait_after_wheel_drop(self) -> None:
        # wait-state timeout은 실패가 아니라 정해진 대기 시간이 끝났다는 뜻이다.
        if self._timed_out():
            self._transition(State.D1_RETURN_FROM_WHEEL_DROP_SPACE)

    def _run_d1_return_from_wheel_drop_space(self) -> None:
        path = self._path(
            self._nav_path_id('d1_return_from_wheel_drop_space'),
            timeout_sec=self.timeout_wheel_drop_move_sec,
        )
        status = self._nav_step(path)
        if status == 'arrived':
            self._mark_base_context(BASE_START)
            self._transition(State.D1_MARK_WHEEL_DISABLED)
        elif status == 'failed':
            self._record_nav_failure(path, 'wheel_drop_return_failed')
            self._manual_wait(State.D1_RETURN_FROM_WHEEL_DROP_SPACE)
        elif self._timed_out():
            self._record_nav_failure(path, 'wheel_drop_return_timeout')
            self._manual_wait(State.D1_RETURN_FROM_WHEEL_DROP_SPACE)

    def _run_d1_mark_wheel_disabled(self) -> None:
        # wheel_disabled는 이후 어떤 recovery/restart에서도 wheel을 다시 시도하지 않도록 막는 latch다.
        self.wheel_disabled = True
        if 'wheel_disabled=true' not in self.status_notes:
            self.status_notes.append('wheel_disabled=true')
        self._transition(State.D2_MOVE_TO_TOOL_AREA)

    def _run_d2_move_to_tool_area(self) -> None:
        if self.base_context == BASE_TOOL_AREA:
            self.current_nav_path = None
            self._transition(State.D2_DETECT_BOLT)
            return
        if self.base_context == BASE_START:
            path_key = 'start_to_tool_area'
        elif self.base_context == BASE_WHEEL_ALIGN:
            path_key = 'wheel_align_to_tool_area'
        elif self.base_context == BASE_FIXTURE:
            path_key = 'fixture_to_tool_area'
        else:
            self._after_return_to_start_state = State.D2_MOVE_TO_TOOL_AREA
            self.current_nav_path = None
            self._transition(State.D3_RETURN_TO_START_POSE)
            return
        if self.current_nav_path is None:
            self.current_nav_path = self._path(
                self._nav_path_id(path_key),
                timeout_sec=self.timeout_navigation_sec,
            )
            self.current_tool_path_id = str(self.current_nav_path.get('path_id', ''))
        status = self._nav_step(self.current_nav_path)
        if status == 'arrived':
            self._mark_base_context(BASE_TOOL_AREA)
            self.current_nav_path = None
            self._transition(State.D2_DETECT_BOLT)
        elif status == 'failed':
            self._record_nav_failure(self.current_nav_path, 'tool_area_nav_failed')
            self.current_nav_path = None
            self._transition(State.D2_MOVE_TO_TOOL_AREA)
        elif self._timed_out():
            self._record_nav_failure(self.current_nav_path, 'tool_area_nav_timeout')
            self.current_nav_path = None
            self._manual_wait(State.D2_MOVE_TO_TOOL_AREA)

    def _bolt_attempt_failed(self, reason: str, drop_failed_candidate: bool) -> None:
        self.bolt_grasp_attempts += 1
        self.total_bolt_grasp_failures += 1
        self.last_failure_reason = reason
        if self.bolt_grasp_attempts >= self.max_bolt_grasp_attempts:
            self._transition(State.D2_DETECT_DRILL)
            return
        self._transition(
            State.D2_RECOVER_BOLT_GRASP_FAILURE
            if drop_failed_candidate
            else State.D2_DETECT_BOLT)

    def _run_d2_detect_bolt(self) -> None:
        if (
            self._valid_item(self.latest_tools, 'bolt')
            and self._fresh_after_base_motion(self.latest_tools)
        ):
            self._transition(State.D2_PLAN_BOLT_GRASP)
        elif self._timed_out():
            self._bolt_attempt_failed('bolt_detection_timeout', drop_failed_candidate=False)

    def _run_d2_plan_bolt_grasp(self) -> None:
        snapshot = self.planning_snapshot or self._capture_snapshot('tools')
        if not self._valid_item(snapshot, 'bolt') or not self._plan_success(snapshot):
            if self._timed_out() or snapshot is not None:
                self._bolt_attempt_failed('bolt_plan_input_invalid', drop_failed_candidate=False)
            return
        status = self._send_manip_goal(
            'GRASP_BOLT_LEFT',
            hand='left',
            primary_pose=self._pose_from_tool(snapshot, 'bolt'),
            params={'snapshot': snapshot},
        )
        if status == 'success':
            self._apply_manip_result('GRASP_BOLT_LEFT')
            self._transition(State.D2_GRASP_BOLT_LEFT)
        elif status == 'failed':
            self._bolt_attempt_failed(
                self._manip_failure_reason('bolt_grasp_failed'), drop_failed_candidate=True)
        elif self._timed_out():
            self._cancel_manip_if_needed()
            self._bolt_attempt_failed('bolt_grasp_timeout', drop_failed_candidate=True)

    def _run_d2_grasp_bolt_left(self) -> None:
        if self.left_hand == 'bolt':
            self._transition(
                State.D2_HANDLE_TOOL_HOLDING_RESULT
                if self.right_hand == 'drill'
                else State.D2_DETECT_DRILL)
        else:
            self._bolt_attempt_failed('bolt_grasp_state_missing', drop_failed_candidate=True)

    def _run_d2_recover_bolt_grasp_failure(self) -> None:
        # bolt grasp 실패 후 새 perception으로 후보 좌표를 다시 고정하고,
        # manipulation이 그 후보를 실제로 잡아 drop하도록 recovery skill을 호출한다.
        if (
            self._valid_item(self.latest_tools, 'bolt')
            and self._fresh_after_state_entry(self.latest_tools)
        ):
            self.planning_snapshot = copy.deepcopy(self.latest_tools)
            self._after_failed_bolt_drop_state = State.D2_DETECT_BOLT
            self._transition(State.D2_DROP_FAILED_GRASP_BOLT_CANDIDATE)
        elif self._timed_out():
            self._after_failed_bolt_drop_state = State.D2_DETECT_BOLT
            self.left_hand = ''
            self.bolt_grasped = False
            self.last_failure_reason = 'drop_failed_grasp_bolt_candidate_detection_timeout'
            self._transition(State.D2_WAIT_AFTER_FAILED_BOLT_DROP)

    def _run_d2_drop_failed_grasp_bolt_candidate(self) -> None:
        if not self._valid_item(self.planning_snapshot, 'bolt'):
            if (
                self._valid_item(self.latest_tools, 'bolt')
                and self._fresh_after_state_entry(self.latest_tools)
            ):
                self.planning_snapshot = copy.deepcopy(self.latest_tools)
            elif self._timed_out():
                self.last_failure_reason = 'drop_failed_grasp_bolt_candidate_point_timeout'
                self.left_hand = ''
                self.bolt_grasped = False
                self._transition(State.D2_WAIT_AFTER_FAILED_BOLT_DROP)
            return
        status = self._send_manip_goal(
            'DROP_FAILED_GRASP_BOLT_CANDIDATE',
            hand='left',
            primary_pose=self._pose_from_tool(self.planning_snapshot, 'bolt'),
            params={'snapshot': self.planning_snapshot, 'reason': self.last_failure_reason},
        )
        if status == 'success':
            self._apply_manip_result('DROP_FAILED_GRASP_BOLT_CANDIDATE')
            self._transition(State.D2_WAIT_AFTER_FAILED_BOLT_DROP)
        elif status == 'failed':
            self.last_failure_reason = self._manip_failure_reason(
                'drop_failed_grasp_bolt_failed')
            self.left_hand = ''
            self.bolt_grasped = False
            self._transition(State.D2_WAIT_AFTER_FAILED_BOLT_DROP)
        elif self._timed_out():
            self._cancel_manip_if_needed()
            self.last_failure_reason = self._manip_failure_reason(
                'drop_failed_grasp_bolt_timeout')
            self.left_hand = ''
            self.bolt_grasped = False
            self._transition(State.D2_WAIT_AFTER_FAILED_BOLT_DROP)

    def _run_d2_wait_after_failed_bolt_drop(self) -> None:
        # wait-state timeout은 실패 후보를 drop한 뒤 환경이 안정될 시간을 채웠다는 뜻이다.
        if self._timed_out():
            self._transition(self._after_failed_bolt_drop_state)

    def _run_d2_detect_drill(self) -> None:
        if self.right_hand == 'drill':
            self._transition(State.D2_HANDLE_TOOL_HOLDING_RESULT)
            return
        if (
            self._valid_item(self.latest_tools, 'drill')
            and self._fresh_after_base_motion(self.latest_tools)
        ):
            self._transition(State.D2_PLAN_DRILL_GRASP)
        elif self._timed_out():
            self._drill_attempt_failed('drill_detection_timeout')

    def _run_d2_plan_drill_grasp(self) -> None:
        if self.right_hand == 'drill':
            self._transition(State.D2_HANDLE_TOOL_HOLDING_RESULT)
            return
        snapshot = self.planning_snapshot or self._capture_snapshot('tools')
        if not self._valid_item(snapshot, 'drill') or not self._plan_success(snapshot):
            if self._timed_out() or snapshot is not None:
                self._drill_attempt_failed('drill_plan_input_invalid')
            return
        self.drill_original_pose = copy.deepcopy((snapshot or {}).get('drill', {}))
        status = self._send_manip_goal(
            'GRASP_DRILL_RIGHT',
            hand='right',
            primary_pose=self._pose_from_tool(snapshot, 'drill'),
            params={'snapshot': snapshot},
        )
        if status == 'success':
            self._apply_manip_result('GRASP_DRILL_RIGHT')
            self._transition(State.D2_GRASP_DRILL_RIGHT)
        elif status == 'failed':
            self._drill_attempt_failed(self._manip_failure_reason('drill_grasp_failed'))
        elif self._timed_out():
            self._cancel_manip_if_needed()
            self._drill_attempt_failed('drill_grasp_timeout')

    def _run_d2_grasp_drill_right(self) -> None:
        if self.right_hand == 'drill':
            self._transition(State.D2_HANDLE_TOOL_HOLDING_RESULT)
        else:
            self._drill_attempt_failed('drill_grasp_state_missing')

    def _drill_attempt_failed(self, reason: str) -> None:
        self.drill_grasp_attempts += 1
        self.total_drill_grasp_failures += 1
        self.last_failure_reason = reason
        if self.drill_grasp_attempts >= self.max_drill_grasp_attempts:
            self._transition(State.D2_HANDLE_TOOL_HOLDING_RESULT)
        else:
            self._transition(State.D2_DETECT_DRILL)

    def _drill_reacquire_failed(self, reason: str) -> None:
        self.drill_grasp_attempts += 1
        self.total_drill_grasp_failures += 1
        self.last_failure_reason = reason
        if self.drill_grasp_attempts >= self.max_drill_grasp_attempts:
            if self._drill_reacquire_only and not self.bolt_inserted:
                self.drill_grasp_attempts = 0
                self._manual_wait(State.D4_DETECT_DRILL_AFTER_BOLT_INSERT)
                return
            self.fastened = False
            self._transition(State.DONE)
        else:
            self._transition(State.D4_DETECT_DRILL_AFTER_BOLT_INSERT)

    def _fastening_stage_failed(self, reason: str) -> None:
        self.fastening_attempts += 1
        self.fastened = False
        self.last_failure_reason = reason
        if self.fastening_attempts >= self.max_fastening_attempts:
            self._transition(State.DONE)
        else:
            self._transition(State.D4_RETURN_INITIAL_FOR_DRILL_RETRY)

    def _run_d2_handle_tool_holding_result(self) -> None:
        self.bolt_grasped = self.left_hand == 'bolt'
        self.drill_grasped = self.right_hand == 'drill'
        if self.bolt_grasped:
            self._drill_reacquire_only = False
            self._transition(State.D3_MOVE_TO_FIXTURE)
        elif self.drill_grasped:
            if 'retry_bolt_with_drill_in_hand' not in self.status_notes:
                self.status_notes.append('retry_bolt_with_drill_in_hand')
            self.bolt_grasp_attempts = 0
            self._transition(State.D2_DETECT_BOLT)
        else:
            self._transition(State.D2_MOVE_TO_TOOL_AREA)

    def _run_d3_move_to_fixture(self) -> None:
        if self.base_context == BASE_FIXTURE:
            self.current_nav_path = None
            self._transition(State.D3_DETECT_FIXTURE_CENTER)
            return
        if self.base_context != BASE_TOOL_AREA:
            self.last_failure_reason = f'invalid_base_context_for_fixture:{self.base_context}'
            self._manual_wait(State.D2_MOVE_TO_TOOL_AREA)
            return
        if self.current_nav_path is None:
            self.current_nav_path = self._path(
                self._nav_path_id('tool_to_fixture'),
                timeout_sec=self.timeout_navigation_sec,
            )
            self.current_fixture_path_id = str(self.current_nav_path.get('path_id', ''))
        status = self._nav_step(self.current_nav_path)
        if status == 'arrived':
            self._mark_base_context(BASE_FIXTURE)
            self.current_nav_path = None
            self._transition(State.D3_DETECT_FIXTURE_CENTER)
        elif status == 'failed':
            self._record_nav_failure(self.current_nav_path, 'fixture_nav_failed')
            self.current_nav_path = None
            self._transition(State.D3_MOVE_TO_FIXTURE)
        elif self._timed_out():
            self._record_nav_failure(self.current_nav_path, 'fixture_nav_timeout')
            self.current_nav_path = None
            self._manual_wait(State.D3_MOVE_TO_FIXTURE)

    def _run_d3_detect_fixture_center(self) -> None:
        if (
            self._valid_top(self.latest_fixture)
            and self._fresh_after_base_motion(self.latest_fixture)
        ):
            if self.bolt_inserted and self.right_hand == 'drill':
                self._transition(State.D4_PLAN_DRILL_FASTEN)
            else:
                self._transition(State.D3_PLAN_BOLT_INSERT)
        elif self._timed_out():
            self.last_failure_reason = 'fixture_center_detection_timeout'
            self._manual_wait(State.D3_DETECT_FIXTURE_CENTER)

    def _run_d3_plan_bolt_insert(self) -> None:
        if self.left_hand != 'bolt':
            self.bolt_grasped = False
            self.last_failure_reason = 'bolt_missing_before_insert'
            self._transition(State.D2_MOVE_TO_TOOL_AREA)
            return
        snapshot = self.planning_snapshot or self._capture_snapshot('fixture')
        if (
            not self._valid_top(snapshot)
            or not self._plan_success(snapshot)
            or not self._fresh_after_base_motion(snapshot)
        ):
            if self._timed_out() or snapshot is not None:
                self.last_failure_reason = 'bolt_insert_plan_input_invalid'
                self._manual_wait(State.D3_DETECT_FIXTURE_CENTER)
            return
        self._transition(State.D3_ALIGN_BOLT_TO_FIXTURE)

    def _run_d3_align_bolt_to_fixture(self) -> None:
        if self.left_hand == 'bolt':
            self._transition(State.D3_INSERT_BOLT_LEFT)
        else:
            self.last_failure_reason = 'bolt_align_state_missing'
            self._transition(State.D2_MOVE_TO_TOOL_AREA)

    def _run_d3_insert_bolt_left(self) -> None:
        # center align, 필요한 base/arm 이동, bolt 삽입은 manipulation skill 하나가 수행한다.
        snapshot = self.planning_snapshot
        status = self._send_manip_goal(
            'INSERT_BOLT_LEFT',
            hand='left',
            primary_pose=self._pose_from_snapshot(snapshot, 'point', 'fixture_center', 'center'),
            params={'snapshot': snapshot},
        )
        if status == 'success':
            self._apply_manip_result('INSERT_BOLT_LEFT')
            self._transition(State.D3_VERIFY_BOLT_INSERTED)
        elif status == 'failed':
            data = self._result_json()
            if 'left_hand' in data:
                self.left_hand = str(data.get('left_hand') or '')
            self.bolt_grasped = self.left_hand == 'bolt'
            self.last_failure_reason = self._manip_failure_reason('bolt_insert_failed')
            if self.left_hand == 'bolt':
                self.status_notes.append('bolt_insert_failed_retry_with_held_bolt')
                self._manual_wait(State.D3_DETECT_FIXTURE_CENTER)
                return
            self.get_logger().warn(
                '[D3_INSERT_BOLT_LEFT] action phase 실패: loose bolt verification 분기로 진입')
            self._transition(State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL)
        elif self._phase_timed_out(self.timeout_bolt_insert_sec):
            self._cancel_manip_if_needed()
            self.last_failure_reason = 'bolt_insert_action_timeout'
            self.bolt_grasped = self.left_hand == 'bolt'
            if self.left_hand == 'bolt':
                self.status_notes.append('bolt_insert_timeout_retry_with_held_bolt')
                self._manual_wait(State.D3_DETECT_FIXTURE_CENTER)
                return
            self.get_logger().warn(
                '[D3_INSERT_BOLT_LEFT] action phase timeout: goal cancel 요청 후 verification 분기로 진입')
            self._transition(State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL)

    def _nav_result_for_status(self) -> dict[str, Any]:
        result = self.last_nav_result
        return {
            'arrived': bool(getattr(result, 'arrived', False)),
            'within_expected_range': bool(getattr(result, 'within_expected_range', False)),
            'actual_translation_mm': float(getattr(result, 'actual_translation_mm', 0.0) or 0.0),
            'actual_yaw_deg': float(getattr(result, 'actual_yaw_deg', 0.0) or 0.0),
            'message': str(getattr(result, 'message', '')),
        }

    def _run_d3_verify_bolt_inserted(self) -> None:
        verification = (
            self.latest_verification
            if (
                self._valid_verification(self.latest_verification)
                and self._fresh_after_state_entry(self.latest_verification)
            )
            else {}
        )
        visible = bool(verification.get('bolt_visible', False))
        now = self._now()
        if visible:
            if self._bolt_visible_since is None:
                self._bolt_visible_since = now
            if now - self._bolt_visible_since >= self.bolt_visible_hold_sec:
                self.bolt_inserted = True
                self.left_hand = ''
                self.bolt_grasped = False
                self._drill_reacquire_only = False
                self._transition(
                    State.D4_PLAN_DRILL_FASTEN
                    if self.right_hand == 'drill'
                    else State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT)
            return
        self._bolt_visible_since = None
        if self._timed_out():
            self.last_failure_reason = 'bolt_insert_verification_failed'
            self._transition(State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL)

    def _run_d3_check_loose_bolt_after_insert_fail(self) -> None:
        if self.left_hand == 'bolt':
            self.bolt_grasped = True
            self.last_failure_reason = 'held_bolt_before_loose_bolt_recovery'
            self.status_notes.append('held_bolt_retry_insert_before_loose_recovery')
            self._manual_wait(State.D3_DETECT_FIXTURE_CENTER)
            return
        has_fresh_verification = (
            self._valid_verification(self.latest_verification)
            and self._fresh_after_state_entry(self.latest_verification)
        )
        if not has_fresh_verification:
            if self._timed_out():
                self.last_failure_reason = 'bolt_recovery_verification_timeout'
                self._transition(State.D3_WAIT_FOR_BOLT_REAPPEAR)
            return
        verification = self.latest_verification or {}
        self.bolt_grasped = self.left_hand == 'bolt'
        if bool(verification.get('bolt_visible_any', False)):
            region = self._loose_bolt_region(verification)
            reachable_value = verification.get('loose_bolt_reachable')
            reachable = reachable_value if isinstance(reachable_value, bool) else region != 'unreachable'
            if region in LOOSE_BOLT_REGIONS and reachable:
                if not self._valid_loose_bolt_snapshot(verification):
                    self.last_failure_reason = 'loose_bolt_point_missing'
                    self.status_notes.append(f'loose_bolt_point_missing_region={region}')
                    self._manual_wait(State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL)
                    return
                self.loose_bolt_unreachable_count = 0
                self.loose_bolt_region = region
                self.planning_snapshot = copy.deepcopy(verification)
                self.status_notes.append(f'loose_bolt_region={region}')
                self._transition(State.D3_MOVE_NEAR_LOOSE_BOLT)
                return
            if region == 'unreachable' or not reachable:
                self.loose_bolt_unreachable_count += 1
                self.loose_bolt_region = region
                self.last_failure_reason = 'loose_bolt_unreachable'
                self.status_notes.append(
                    f'loose_bolt_unreachable:{self.loose_bolt_unreachable_count}:'
                    f'{verification.get("loose_bolt_message", "")}')
                if self.right_hand == 'drill' or self.drill_grasped:
                    self._manual_wait(State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL)
                else:
                    self._drill_reacquire_only = True
                    self._after_return_to_start_state = (
                        State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT)
                    self._transition(State.D3_RETURN_TO_START_POSE)
                return
            self.loose_bolt_region = region
            self.last_failure_reason = 'loose_bolt_region_unknown'
            self.status_notes.append(f'loose_bolt_region_unknown={region}')
            self._manual_wait(State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL)
            return
        else:
            self._transition(State.D3_WAIT_FOR_BOLT_REAPPEAR)

    def _state_after_loose_bolt_nav_exhausted(self) -> State:
        self.status_notes.append('loose_bolt_nav_failed_recheck_loose_bolt')
        self._after_manual_wait_state = State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL
        return State.MANUAL_WAIT

    def _return_to_start_after_loose_bolt_nav_exhausted(self) -> None:
        self._after_return_to_start_state = self._state_after_loose_bolt_nav_exhausted()
        self.current_nav_path = None
        self._transition(State.D3_RETURN_TO_START_POSE)

    def _run_d3_move_near_loose_bolt(self) -> None:
        region = self.loose_bolt_region or 'front'
        if region not in LOOSE_BOLT_REGIONS:
            self.last_failure_reason = f'invalid_loose_bolt_region:{region}'
            self._manual_wait(State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL)
            return
        path = self._path(
            self._nav_path_id(f'd3_move_near_loose_bolt_{region}'),
            timeout_sec=self.timeout_navigation_sec,
        )
        status = self._nav_step(path)
        if status == 'arrived':
            self.loose_bolt_nav_failures = 0
            self._mark_base_context(BASE_NEAR_LOOSE_BOLT)
            self._transition(State.D3_GRASP_LOOSE_BOLT_CANDIDATE)
        elif status == 'failed':
            self.loose_bolt_nav_failures += 1
            self._record_nav_failure(path, 'move_near_loose_bolt_failed')
            if self.loose_bolt_nav_failures >= self.max_loose_bolt_nav_failures:
                self._return_to_start_after_loose_bolt_nav_exhausted()
            else:
                self._transition(State.D3_MOVE_NEAR_LOOSE_BOLT)
        elif self._timed_out():
            self.loose_bolt_nav_failures += 1
            self._record_nav_failure(path, 'move_near_loose_bolt_timeout')
            self.current_nav_path = None
            if self.loose_bolt_nav_failures >= self.max_loose_bolt_nav_failures:
                self._return_to_start_after_loose_bolt_nav_exhausted()
            else:
                self._transition(State.D3_MOVE_NEAR_LOOSE_BOLT)

    def _run_d3_grasp_loose_bolt_candidate(self) -> None:
        if not self._valid_loose_bolt_snapshot(self.planning_snapshot):
            if self._timed_out():
                self.last_failure_reason = 'grasp_loose_bolt_candidate_point_timeout'
                self.left_hand = ''
                self.bolt_grasped = False
                self._after_return_to_start_state = State.MANUAL_WAIT
                self._after_manual_wait_state = State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL
                self._transition(State.D3_RETURN_TO_START_POSE)
            return
        status = self._send_manip_goal(
            'GRASP_BOLT_LEFT',
            hand='left',
            primary_pose=self._pose_from_loose_bolt_snapshot(self.planning_snapshot),
            params={'snapshot': self.planning_snapshot, 'reason': self.last_failure_reason},
        )
        if status == 'success':
            self._apply_manip_result('GRASP_BOLT_LEFT')
            self._transition(State.D3_MOVE_TO_BOLT_DROP_SPACE)
        elif status == 'failed':
            self.last_failure_reason = self._manip_failure_reason('grasp_loose_bolt_failed')
            self.left_hand = ''
            self.bolt_grasped = False
            self._after_return_to_start_state = State.MANUAL_WAIT
            self._after_manual_wait_state = State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL
            self._transition(State.D3_RETURN_TO_START_POSE)
        elif self._timed_out():
            self._cancel_manip_if_needed()
            self.last_failure_reason = self._manip_failure_reason('grasp_loose_bolt_timeout')
            self.left_hand = ''
            self.bolt_grasped = False
            self._after_return_to_start_state = State.MANUAL_WAIT
            self._after_manual_wait_state = State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL
            self._transition(State.D3_RETURN_TO_START_POSE)

    def _run_d3_move_to_bolt_drop_space(self) -> None:
        if self.left_hand != 'bolt':
            self.last_failure_reason = 'bolt_missing_before_drop_space'
            self._after_return_to_start_state = State.MANUAL_WAIT
            self._after_manual_wait_state = State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL
            self._transition(State.D3_RETURN_TO_START_POSE)
            return
        path = self._path(
            self._nav_path_id('d3_move_to_bolt_drop_space'),
            timeout_sec=self.timeout_navigation_sec,
        )
        status = self._nav_step(path, require_within_expected_range=True)
        if status == 'arrived':
            self._mark_base_context(BASE_BOLT_DROP_SPACE)
            self._transition(State.D3_RELEASE_LOOSE_BOLT_TO_FLOOR)
        elif status == 'failed':
            self._record_nav_failure(path, 'bolt_drop_space_nav_failed')
            self._manual_wait(State.D3_MOVE_TO_BOLT_DROP_SPACE)
        elif self._timed_out():
            self._record_nav_failure(path, 'bolt_drop_space_nav_timeout')
            self._manual_wait(State.D3_MOVE_TO_BOLT_DROP_SPACE)

    def _run_d3_release_loose_bolt_to_floor(self) -> None:
        status = self._send_manip_goal(
            'RELEASE_BOLT_TO_FLOOR',
            hand='left',
            params={'reason': self.last_failure_reason, 'region': self.loose_bolt_region},
        )
        if status == 'success':
            self._apply_manip_result('RELEASE_BOLT_TO_FLOOR')
            self._transition(State.D3_WAIT_AFTER_LOOSE_BOLT_DROP)
        elif status == 'failed':
            self.last_failure_reason = self._manip_failure_reason('release_loose_bolt_failed')
            self._manual_wait(State.D3_RELEASE_LOOSE_BOLT_TO_FLOOR)
        elif self._timed_out():
            self._cancel_manip_if_needed()
            self.last_failure_reason = 'release_loose_bolt_timeout'
            self._manual_wait(State.D3_RELEASE_LOOSE_BOLT_TO_FLOOR)

    def _run_d3_wait_after_loose_bolt_drop(self) -> None:
        if self._timed_out():
            self._after_return_to_start_state = State.D2_MOVE_TO_TOOL_AREA
            self._transition(State.D3_RETURN_TO_START_POSE)

    def _run_d3_wait_for_bolt_reappear(self) -> None:
        # 대기 중에도 fresh verification을 확인한다. 다시 보이면 loose bolt recovery로 복귀하고,
        # 보이지 않은 채 시간이 끝나면 start pose를 거쳐 tool 흐름으로 자동 복귀한다.
        if (
            self._valid_verification(self.latest_verification)
            and self._fresh_after_state_entry(self.latest_verification)
        ):
            verification = self.latest_verification or {}
            if bool(verification.get('bolt_visible', False)):
                self._transition(State.D3_VERIFY_BOLT_INSERTED)
                return
            if bool(verification.get('bolt_visible_any', False)):
                self._transition(State.D3_CHECK_LOOSE_BOLT_AFTER_INSERT_FAIL)
                return
        if self._timed_out():
            self._after_return_to_start_state = State.D2_MOVE_TO_TOOL_AREA
            self._transition(State.D3_RETURN_TO_START_POSE)

    def _run_d3_return_to_start_pose(self) -> None:
        if self.base_context == BASE_START:
            self.current_nav_path = None
            self._transition(self._after_return_to_start_state)
            return
        if self.current_nav_path is None:
            self.current_nav_path = self._path(
                self._nav_path_id('return_to_start_pose'),
                timeout_sec=self.timeout_navigation_sec,
            )
        status = self._nav_step(self.current_nav_path, require_within_expected_range=True)
        if status == 'arrived':
            self._mark_base_context(BASE_START)
            self.current_nav_path = None
            self._transition(self._after_return_to_start_state)
        elif status == 'failed':
            self._record_nav_failure(self.current_nav_path, 'return_to_start_failed')
            self.current_nav_path = None
            self._manual_wait(State.D3_RETURN_TO_START_POSE)
        elif self._timed_out():
            self._record_nav_failure(self.current_nav_path, 'return_to_start_timeout')
            self.current_nav_path = None
            self._manual_wait(State.D3_RETURN_TO_START_POSE)

    def _run_d4_move_back_to_tool_for_drill_after_bolt_insert(self) -> None:
        if self.base_context == BASE_TOOL_AREA:
            self.current_nav_path = None
            self._transition(State.D4_DETECT_DRILL_AFTER_BOLT_INSERT)
            return
        if self.base_context == BASE_FIXTURE:
            path_key = 'fixture_to_tool_area'
        elif self.base_context == BASE_START:
            path_key = 'start_to_tool_area'
        else:
            self._after_return_to_start_state = State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT
            self.current_nav_path = None
            self._transition(State.D3_RETURN_TO_START_POSE)
            return
        if self.current_nav_path is None:
            self.current_nav_path = self._path(
                self._nav_path_id(path_key),
                timeout_sec=self.timeout_navigation_sec,
            )
            self.current_tool_path_id = str(self.current_nav_path.get('path_id', ''))
        status = self._nav_step(self.current_nav_path)
        if status == 'arrived':
            self._mark_base_context(BASE_TOOL_AREA)
            self.current_nav_path = None
            self._transition(State.D4_DETECT_DRILL_AFTER_BOLT_INSERT)
        elif status == 'failed':
            self._record_nav_failure(self.current_nav_path, 'return_tool_for_drill_failed')
            self.current_nav_path = None
            self._transition(State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT)
        elif self._timed_out():
            self._record_nav_failure(self.current_nav_path, 'return_tool_for_drill_timeout')
            self.current_nav_path = None
            self._manual_wait(State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT)

    def _run_d4_detect_drill_after_bolt_insert(self) -> None:
        if (
            self._valid_item(self.latest_tools, 'drill')
            and self._fresh_after_base_motion(self.latest_tools)
        ):
            self._transition(State.D4_PLAN_DRILL_GRASP_AFTER_BOLT_INSERT)
        elif self._timed_out():
            self._drill_reacquire_failed('drill_reacquire_detection_timeout')

    def _run_d4_plan_drill_grasp_after_bolt_insert(self) -> None:
        snapshot = self.planning_snapshot or self._capture_snapshot('tools')
        if not self._valid_item(snapshot, 'drill') or not self._plan_success(snapshot):
            if self._timed_out() or snapshot is not None:
                self._drill_reacquire_failed('drill_reacquire_plan_input_invalid')
            return
        self.drill_original_pose = copy.deepcopy((snapshot or {}).get('drill', {}))
        status = self._send_manip_goal(
            'GRASP_DRILL_RIGHT',
            hand='right',
            primary_pose=self._pose_from_tool(snapshot, 'drill'),
            params={'snapshot': snapshot, 'after_bolt_insert': True},
        )
        if status == 'success':
            self._apply_manip_result('GRASP_DRILL_RIGHT')
            self._transition(State.D4_GRASP_DRILL_RIGHT_AFTER_BOLT_INSERT)
        elif status == 'failed':
            self._drill_reacquire_failed(self._manip_failure_reason('drill_reacquire_failed'))
        elif self._timed_out():
            self._cancel_manip_if_needed()
            self._drill_reacquire_failed('drill_reacquire_timeout')

    def _run_d4_grasp_drill_right_after_bolt_insert(self) -> None:
        if self.right_hand == 'drill':
            if self._drill_reacquire_only and not self.bolt_inserted:
                self._after_return_to_start_state = State.MANUAL_WAIT
                self._after_manual_wait_state = State.D2_MOVE_TO_TOOL_AREA
                self._transition(State.D3_RETURN_TO_START_POSE)
                return
            self._transition(State.D4_MOVE_TO_FIXTURE_WITH_DRILL_AFTER_BOLT_INSERT)
        else:
            self._drill_reacquire_failed('drill_reacquire_state_missing')

    def _run_d4_move_to_fixture_with_drill_after_bolt_insert(self) -> None:
        if self.base_context == BASE_FIXTURE:
            self.current_nav_path = None
            self._transition(State.D4_PLAN_DRILL_FASTEN)
            return
        if self.base_context != BASE_TOOL_AREA:
            self.last_failure_reason = f'invalid_base_context_for_fixture_with_drill:{self.base_context}'
            self._manual_wait(State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT)
            return
        if self.current_nav_path is None:
            self.current_nav_path = self._path(
                self._nav_path_id('tool_to_fixture'),
                timeout_sec=self.timeout_navigation_sec,
            )
            self.current_fixture_path_id = str(self.current_nav_path.get('path_id', ''))
        status = self._nav_step(self.current_nav_path)
        if status == 'arrived':
            self._mark_base_context(BASE_FIXTURE)
            self.current_nav_path = None
            self._transition(State.D4_PLAN_DRILL_FASTEN)
        elif status == 'failed':
            self._record_nav_failure(self.current_nav_path, 'fixture_with_drill_nav_failed')
            self.current_nav_path = None
            self._transition(State.D4_MOVE_TO_FIXTURE_WITH_DRILL_AFTER_BOLT_INSERT)
        elif self._timed_out():
            self._record_nav_failure(self.current_nav_path, 'fixture_with_drill_nav_timeout')
            self.current_nav_path = None
            self._manual_wait(State.D4_MOVE_TO_FIXTURE_WITH_DRILL_AFTER_BOLT_INSERT)

    def _run_d4_plan_drill_fasten(self) -> None:
        if self.base_context != BASE_FIXTURE:
            self.last_failure_reason = f'invalid_base_context_for_fasten:{self.base_context}'
            self._manual_wait(State.D4_MOVE_TO_FIXTURE_WITH_DRILL_AFTER_BOLT_INSERT)
            return
        if not self.bolt_inserted:
            self.last_failure_reason = 'inserted_bolt_missing_before_fasten'
            self._manual_wait(State.D2_MOVE_TO_TOOL_AREA)
            return
        if self.right_hand != 'drill':
            self.last_failure_reason = 'drill_missing_before_fasten'
            self._manual_wait(State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT)
            return
        snapshot = self.planning_snapshot or self._capture_snapshot('fixture')
        if (
            not self._valid_top(snapshot)
            or not self._plan_success(snapshot)
            or not self._fresh_after_base_motion(snapshot)
        ):
            if self._timed_out() or snapshot is not None:
                self.last_failure_reason = 'drill_fasten_plan_input_invalid'
                self._manual_wait(State.D3_DETECT_FIXTURE_CENTER)
            return
        self._transition(State.D4_FASTEN_WITH_DRILL)

    def _run_d4_fasten_with_drill(self) -> None:
        if self.base_context != BASE_FIXTURE:
            self.last_failure_reason = f'fasten_precondition_base:{self.base_context}'
            self._manual_wait(State.D4_MOVE_TO_FIXTURE_WITH_DRILL_AFTER_BOLT_INSERT)
            return
        if not self.bolt_inserted:
            self.last_failure_reason = 'fasten_precondition_bolt_missing'
            self._manual_wait(State.D2_MOVE_TO_TOOL_AREA)
            return
        if self.right_hand != 'drill':
            self.last_failure_reason = 'fasten_precondition_drill_missing'
            self._manual_wait(State.D4_MOVE_BACK_TO_TOOL_FOR_DRILL_AFTER_BOLT_INSERT)
            return
        snapshot = self.planning_snapshot
        status = self._send_manip_goal(
            'FASTEN_WITH_DRILL',
            hand='right',
            primary_pose=self._pose_from_snapshot(snapshot, 'point', 'fixture_center', 'center'),
            params={'snapshot': snapshot},
        )
        if status == 'success':
            self._apply_manip_result('FASTEN_WITH_DRILL')
            self._transition(State.D4_VERIFY_FASTENING)
        elif status == 'failed':
            self._fastening_stage_failed(self._manip_failure_reason('fasten_failed'))
        elif self._timed_out():
            self._cancel_manip_if_needed()
            self._fastening_stage_failed('fasten_timeout')

    def _run_d4_verify_fastening(self) -> None:
        # 체결 성공의 주 기준은 FASTEN_WITH_DRILL action 결과다. lidar/이동량은 status 참고값으로만 둔다.
        if self.fastened:
            self._transition(State.DONE)
        elif self._timed_out():
            self.last_failure_reason = 'fasten_result_not_confirmed'
            self._transition(State.DONE)

    def _run_d4_return_initial_for_drill_retry(self) -> None:
        status = self._send_manip_goal(
            'RETURN_INITIAL_FOR_DRILL_RETRY',
            hand='right',
            params={'fastening_attempts': self.fastening_attempts},
        )
        if status == 'success':
            self._apply_manip_result('RETURN_INITIAL_FOR_DRILL_RETRY')
            self._transition(State.D3_DETECT_FIXTURE_CENTER)
        elif status == 'failed':
            self.last_failure_reason = self._manip_failure_reason('drill_retry_return_failed')
            if self.fastening_attempts >= self.max_fastening_attempts:
                self._transition(State.DONE)
            else:
                self._transition(State.D3_DETECT_FIXTURE_CENTER)
        elif self._timed_out():
            self._cancel_manip_if_needed()
            self.last_failure_reason = 'drill_retry_return_timeout'
            if self.fastening_attempts >= self.max_fastening_attempts:
                self._transition(State.DONE)
            else:
                self._transition(State.D3_DETECT_FIXTURE_CENTER)

    def _run_manual_wait(self) -> None:
        # 완전 자율 운용에서는 사람이 개입하지 않는다. 이 state는 보수적 cooldown 후
        # 마지막 실패 지점에서 지정한 recovery state로 자동 복귀하는 역할만 한다.
        if not self._timed_out():
            return
        next_state = self._after_manual_wait_state
        if next_state == State.MANUAL_WAIT:
            next_state = State.D2_MOVE_TO_TOOL_AREA
        self.status_notes.append(f'manual_wait_retry={next_state.name}')
        self._transition(next_state)

    def _run_done(self) -> None:
        if self._done_logged:
            return
        self._done_logged = True
        self.get_logger().info(
            f'[DONE] wheel_done={self.wheel_done} bolt_inserted={self.bolt_inserted} '
            f'fastened={self.fastened} wheel_disabled={self.wheel_disabled}')
        self._publish_status()
        self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionD()
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
