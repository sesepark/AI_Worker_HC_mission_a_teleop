#!/usr/bin/env python3
"""Mission D team-owned 설정 기반 조작 action mock 노드.

실제 팔 경로, IK, grasp planning, drill 제어는 구현하지 않는다. grasp offset, drill tip offset 같은
manipulation 내부 tuning은 mock parameter로 갖고, System FSM은 skill_id, hand, target pose만
요청한다고 가정한다.
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mission_interfaces.action import MissionDManipulation


class MockManipulationD(Node):
    """MissionDManipulation action을 제공하는 strict-order mock Node."""

    def __init__(self) -> None:
        super().__init__('mock_manipulation_d')

        self.manipulation_action_name = str(
            self.declare_parameter('manipulation_action_name', '/mission_d/manipulation').value)
        self.scenario = str(self.declare_parameter('scenario', 'normal').value)
        self.action_sec = float(self.declare_parameter('action_sec', 1.0).value)
        self.grasp_offsets = self._json_param(
            'grasp_offsets_json',
            {
                'wheel': {'right': [0.0, -0.12, 0.0]},
                'bolt': {'left': [0.0, 0.0, 0.03]},
                'drill': {'right': [-0.06, 0.0, -0.08]},
            },
        )
        self.drill_tip_offset = self._json_param(
            'drill_tip_offset_json', {'right': [0.12, 0.0, 0.01]})
        self.fail_wheel_grasp_attempts = int(
            self.declare_parameter('fail_wheel_grasp_attempts', 0).value)
        self.fail_bolt_grasp_attempts = int(
            self.declare_parameter('fail_bolt_grasp_attempts', 0).value)
        self.fail_drill_grasp_attempts = int(
            self.declare_parameter('fail_drill_grasp_attempts', 0).value)
        self.fail_bolt_insert_attempts = int(
            self.declare_parameter('fail_bolt_insert_attempts', 0).value)
        self.fail_fasten_attempts = int(
            self.declare_parameter('fail_fasten_attempts', 0).value)

        if self.scenario == 'bolt_grasp_fail_with_drop':
            self.fail_bolt_grasp_attempts = max(self.fail_bolt_grasp_attempts, 1)
        if self.scenario == 'drill_only_success':
            self.fail_bolt_grasp_attempts = max(self.fail_bolt_grasp_attempts, 5)
        if self.scenario == 'drill_fastening_fail':
            self.fail_fasten_attempts = max(self.fail_fasten_attempts, 3)

        self.right_hand = ''
        self.left_hand = ''
        self.wheel_grasped = False
        self.wheel_inserted = False
        self.wheel_released = False
        self.bolt_inserted = False
        self.fastened = False
        self._attempts: dict[str, int] = {}

        cbg = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            MissionDManipulation,
            self.manipulation_action_name,
            self._execute,
            callback_group=cbg,
        )
        self.get_logger().info(
            f'mock_manipulation_d ready action={self.manipulation_action_name} '
            f'scenario={self.scenario}')

    def _json_param(self, name: str, default):
        value = self.declare_parameter(name, json.dumps(default)).value
        if isinstance(value, str):
            try:
                return json.loads(value)
            except Exception:
                self.get_logger().warn(f'{name} 파싱 실패, 기본값 사용')
                return default
        return value

    def _offset(self, item: str, hand: str):
        item_offsets = self.grasp_offsets.get(item, {})
        return item_offsets.get(hand, [0.0, 0.0, 0.0])

    def _execute(self, goal_handle):
        goal = goal_handle.request
        self._attempts[goal.skill_id] = self._attempts.get(goal.skill_id, 0) + 1
        self.get_logger().info(
            f'[mock_manip_d] skill={goal.skill_id} hand={goal.hand} '
            f'attempt={self._attempts[goal.skill_id]}')

        steps = 4
        for idx in range(steps):
            time.sleep(max(self.action_sec, 0.0) / steps)
            feedback = MissionDManipulation.Feedback()
            feedback.phase = goal.skill_id
            feedback.progress = float(idx + 1) / float(steps)
            goal_handle.publish_feedback(feedback)

        success, code, message, result_json = self._run_skill(goal)
        goal_handle.succeed()
        result = MissionDManipulation.Result()
        result.success = success
        result.result_code = code
        result.message = message
        result.result_json = json.dumps(result_json, sort_keys=True)
        return result

    def _attempt(self, skill_id: str) -> int:
        return self._attempts.get(skill_id, 0)

    def _fail_until(self, skill_id: str, limit: int, code: str) -> tuple[bool, str] | None:
        if self._attempt(skill_id) <= limit:
            return False, code
        return None

    def _ok(self, message: str = 'ok', extra: dict | None = None) -> tuple[bool, str, str, dict]:
        payload = self._state_json()
        if extra:
            payload.update(extra)
        return True, 'success', message, payload

    def _fail(self, code: str, message: str, extra: dict | None = None) -> tuple[bool, str, str, dict]:
        payload = self._state_json()
        if extra:
            payload.update(extra)
        return False, code, message, payload

    def _state_json(self) -> dict:
        return {
            'right_hand': self.right_hand,
            'left_hand': self.left_hand,
            'wheel_grasped': self.wheel_grasped,
            'wheel_inserted': self.wheel_inserted,
            'bolt_grasped': self.left_hand == 'bolt',
            'drill_grasped': self.right_hand == 'drill',
            'bolt_inserted': self.bolt_inserted,
            'fastened': self.fastened,
        }

    def _run_skill(self, goal) -> tuple[bool, str, str, dict]:
        skill = goal.skill_id

        if skill == 'GRASP_WHEEL_RIGHT':
            injected = self._fail_until(skill, self.fail_wheel_grasp_attempts, 'wheel_grasp_failed')
            if injected:
                return self._fail(injected[1], 'wheel grasp injected failure')
            if self.right_hand:
                return self._fail('invalid_state', 'right hand is not empty')
            self.right_hand = 'wheel'
            self.wheel_grasped = True
            return self._ok(
                'wheel grasped and moved to body',
                {'used_offset': self._offset('wheel', 'right'), 'moved_to_body': True},
            )

        if skill == 'INSERT_WHEEL_FORWARD':
            if self.right_hand != 'wheel':
                return self._fail('invalid_state', 'INSERT_WHEEL_FORWARD requires wheel')
            self.right_hand = ''
            self.wheel_grasped = False
            self.wheel_inserted = True
            return self._ok('wheel aligned and inserted', {'wheel_inserted': True})

        if skill == 'RELEASE_WHEEL_TO_FLOOR':
            self.right_hand = ''
            self.wheel_grasped = False
            self.wheel_released = True
            return self._ok('wheel released to floor', {'released': True})

        if skill == 'GRASP_BOLT_LEFT':
            injected = self._fail_until(skill, self.fail_bolt_grasp_attempts, 'bolt_grasp_failed')
            if injected:
                return self._fail(injected[1], 'bolt grasp injected failure')
            if self.right_hand == 'wheel':
                # wheel 삽입 성공은 MoveBaseRelative 결과로 FSM이 확정한다. mock action server는
                # 그 service 결과를 직접 보지 못하므로 tool 단계가 시작되면 삽입된 wheel을 손에서 제거한다.
                self.right_hand = ''
                self.wheel_grasped = False
            if self.left_hand:
                return self._fail('invalid_state', 'left hand is not empty')
            self.left_hand = 'bolt'
            return self._ok('bolt grasped', {'used_offset': self._offset('bolt', 'left')})

        if skill == 'DROP_FAILED_GRASP_BOLT_CANDIDATE':
            if self.left_hand == 'bolt':
                self.left_hand = ''
            return self._ok(
                'failed grasp bolt candidate grasped and released',
                {
                    'grasped_for_drop': True,
                    'released_to_floor': True,
                    'left_hand': '',
                    'bolt_grasped': False,
                },
            )

        if skill == 'RELEASE_BOLT_TO_FLOOR':
            if self.left_hand != 'bolt':
                return self._fail('invalid_state', 'RELEASE_BOLT_TO_FLOOR requires left bolt')
            self.left_hand = ''
            return self._ok('bolt released to floor', {'released': True})

        if skill == 'GRASP_DRILL_RIGHT':
            injected = self._fail_until(skill, self.fail_drill_grasp_attempts, 'drill_grasp_failed')
            if injected:
                return self._fail(injected[1], 'drill grasp injected failure')
            if self.right_hand:
                return self._fail('invalid_state', 'right hand is not empty')
            self.right_hand = 'drill'
            return self._ok('drill grasped', {'used_offset': self._offset('drill', 'right')})

        if skill == 'INSERT_BOLT_LEFT':
            if self.left_hand != 'bolt':
                return self._fail('invalid_state', 'INSERT_BOLT_LEFT requires left bolt')
            injected = self._fail_until(skill, self.fail_bolt_insert_attempts, 'bolt_insert_failed')
            if injected:
                self.left_hand = ''
                return self._fail(injected[1], 'bolt insert injected failure')
            self.left_hand = ''
            self.bolt_inserted = True
            return self._ok('bolt aligned and inserted')

        if skill == 'FASTEN_WITH_DRILL':
            if self.right_hand != 'drill' or not self.bolt_inserted:
                return self._fail('invalid_state', 'FASTEN_WITH_DRILL requires drill and bolt')
            injected = self._fail_until(skill, self.fail_fasten_attempts, 'fasten_failed')
            if injected:
                return self._fail(injected[1], 'fasten injected failure')
            self.fastened = True
            return self._ok('fastened')

        if skill == 'RETURN_INITIAL_FOR_DRILL_RETRY':
            if self.right_hand != 'drill':
                return self._fail('invalid_state', 'RETURN_INITIAL_FOR_DRILL_RETRY requires drill')
            return self._ok('returned initial for drill retry')

        if skill == 'SAFE_RETREAT':
            return self._ok('safe retreat')

        if skill == 'GO_STABLE_EMPTY':
            if self.right_hand or self.left_hand:
                return self._fail('invalid_state', 'GO_STABLE_EMPTY requires empty hands')
            return self._ok('stable empty')

        if skill == 'GO_STABLE_WITH_BOLT':
            if self.left_hand != 'bolt':
                return self._fail('invalid_state', 'GO_STABLE_WITH_BOLT requires left bolt')
            return self._ok('stable with bolt')

        if skill == 'GO_STABLE_WITH_DRILL':
            if self.right_hand != 'drill':
                return self._fail('invalid_state', 'GO_STABLE_WITH_DRILL requires right drill')
            return self._ok('stable with drill')

        if skill == 'GO_STABLE_WITH_BOLT_AND_DRILL':
            if self.left_hand != 'bolt' or self.right_hand != 'drill':
                return self._fail('invalid_state', 'stable both requires bolt and drill')
            return self._ok('stable with bolt and drill')

        return self._fail('unknown_skill', f'unsupported skill_id={skill}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockManipulationD()
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
