#!/usr/bin/env python3
"""Mission D team-owned 설정 기반 perception JSON mock 노드.

실제 vision 알고리즘은 구현하지 않는다. detection threshold, center tolerance, frame convention을
perception 팀 설정처럼 mock 내부 parameter로 갖고, System FSM에는 valid/plan_success/visibility 같은
계약 결과만 JSON으로 전달한다.
"""
from __future__ import annotations

import json

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class MockPerceptionD(Node):
    """Mission D perception topic 5종을 state 흐름에 맞춰 발행하는 mock Node."""

    def __init__(self) -> None:
        super().__init__('mock_perception_d')

        self.scenario = str(self.declare_parameter('scenario', 'normal').value)
        self.frame_id = str(self.declare_parameter('frame_id', 'base_link').value)
        self.fail_wheel_attempts = int(
            self.declare_parameter('fail_wheel_attempts', 0).value)
        self.fail_bolt_detection_attempts = int(
            self.declare_parameter('fail_bolt_detection_attempts', 0).value)
        self.fail_drill_detection_attempts = int(
            self.declare_parameter('fail_drill_detection_attempts', 0).value)
        self.fail_fixture_detection_attempts = int(
            self.declare_parameter('fail_fixture_detection_attempts', 0).value)
        self.plan_success = bool(self.declare_parameter('plan_success', True).value)
        self.confidence = float(self.declare_parameter('confidence', 0.90).value)
        self.low_confidence = float(
            self.declare_parameter('low_confidence', 0.20).value)
        self.wheel_detection_confidence_threshold = float(
            self.declare_parameter('wheel_detection_confidence_threshold', 0.70).value)
        self.bolt_detection_confidence_threshold = float(
            self.declare_parameter('bolt_detection_confidence_threshold', 0.70).value)
        self.drill_detection_confidence_threshold = float(
            self.declare_parameter('drill_detection_confidence_threshold', 0.70).value)
        self.bolt_visible_confidence_threshold = float(
            self.declare_parameter('bolt_visible_confidence_threshold', 0.80).value)
        self.bolt_center_tolerance_m = float(
            self.declare_parameter('bolt_center_tolerance_m', 0.03).value)
        self.loose_bolt_region = str(
            self.declare_parameter('loose_bolt_region', 'front').value)
        self.loose_bolt_reachable = bool(
            self.declare_parameter('loose_bolt_reachable', True).value)

        if self.scenario == 'wheel_detection_fail':
            self.fail_wheel_attempts = max(self.fail_wheel_attempts, 99)

        self.current_state = 'INIT'
        self._last_state = ''
        self._state_entry_counts: dict[str, int] = {}

        cbg = ReentrantCallbackGroup()
        self.create_subscription(
            String, '/mission_d/state', self._on_state, 10, callback_group=cbg)
        self.pub_wheel = self.create_publisher(
            String, '/mission_d/perception/wheel', 10)
        self.pub_wheel_fixture = self.create_publisher(
            String, '/mission_d/perception/wheel_fixture', 10)
        self.pub_tools = self.create_publisher(
            String, '/mission_d/perception/tools', 10)
        self.pub_fixture = self.create_publisher(
            String, '/mission_d/perception/fixture', 10)
        self.pub_verification = self.create_publisher(
            String, '/mission_d/perception/verification', 10)

        self.create_timer(0.2, self._publish_for_state, callback_group=cbg)
        self.get_logger().info(
            f'mock_perception_d ready scenario={self.scenario} '
            f'confidence={self.confidence:.2f}')

    def _on_state(self, msg: String) -> None:
        self.current_state = msg.data
        if self.current_state != self._last_state:
            self._last_state = self.current_state
            self._state_entry_counts[self.current_state] = (
                self._state_entry_counts.get(self.current_state, 0) + 1
            )

    def _count(self, state: str) -> int:
        return self._state_entry_counts.get(state, 0)

    def _publish_json(self, pub, payload: dict) -> None:
        pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_for_state(self) -> None:
        # 모든 topic을 계속 발행하되, payload 내용은 state count와 scenario에 따라 바뀐다.
        self._publish_json(self.pub_wheel, self._wheel_payload())
        self._publish_json(self.pub_wheel_fixture, self._wheel_fixture_payload())
        self._publish_json(self.pub_tools, self._tools_payload())
        self._publish_json(self.pub_fixture, self._fixture_payload())
        self._publish_json(self.pub_verification, self._verification_payload())

    def _wheel_payload(self) -> dict:
        count = self._count('D1_DETECT_WHEEL')
        failed = 0 < count <= self.fail_wheel_attempts
        confidence = self.low_confidence if failed else self.confidence
        valid = confidence >= self.wheel_detection_confidence_threshold
        return {
            'valid': valid,
            'confidence': confidence,
            'frame_id': self.frame_id,
            'plan_success': self.plan_success and valid,
            'point': {'x': 0.45, 'y': -0.08, 'z': 0.84},
            'pose': {
                'position': {'x': 0.45, 'y': -0.08, 'z': 0.84},
                'orientation': {'w': 1.0},
            },
        }

    def _wheel_fixture_payload(self) -> dict:
        return {
            'valid': self.confidence >= self.wheel_detection_confidence_threshold,
            'confidence': self.confidence,
            'frame_id': self.frame_id,
            'plan_success': self.plan_success
            and self.confidence >= self.wheel_detection_confidence_threshold,
            'point': {'x': 0.56, 'y': 0.00, 'z': 0.92},
            'fixture_center': {'x': 0.56, 'y': 0.00, 'z': 0.92},
        }

    def _tools_payload(self) -> dict:
        bolt_count = self._count('D2_DETECT_BOLT')
        drill_count = (
            self._count('D2_DETECT_DRILL')
            + self._count('D4_DETECT_DRILL_AFTER_BOLT_INSERT')
        )
        bolt_failed = 0 < bolt_count <= self.fail_bolt_detection_attempts
        drill_failed = 0 < drill_count <= self.fail_drill_detection_attempts
        bolt_confidence = self.low_confidence if bolt_failed else self.confidence
        drill_confidence = self.low_confidence if drill_failed else self.confidence
        bolt_valid = bolt_confidence >= self.bolt_detection_confidence_threshold
        drill_valid = drill_confidence >= self.drill_detection_confidence_threshold
        return {
            'valid': bolt_valid or drill_valid,
            'confidence': self.confidence,
            'frame_id': self.frame_id,
            'plan_success': self.plan_success,
            'bolt': {
                'valid': bolt_valid,
                'confidence': bolt_confidence,
                'point': {'x': 0.42, 'y': -0.12, 'z': 0.80},
            },
            'drill': {
                'valid': drill_valid,
                'confidence': drill_confidence,
                'point': {'x': 0.43, 'y': 0.15, 'z': 0.80},
            },
        }

    def _fixture_payload(self) -> dict:
        count = self._count('D3_DETECT_FIXTURE_CENTER')
        failed = 0 < count <= self.fail_fixture_detection_attempts
        confidence = self.low_confidence if failed else self.confidence
        valid = confidence >= self.bolt_detection_confidence_threshold
        return {
            'valid': valid,
            'confidence': confidence,
            'frame_id': self.frame_id,
            'plan_success': self.plan_success and valid,
            'point': {'x': 0.56, 'y': 0.0, 'z': 0.92},
            'fixture_center': {'x': 0.56, 'y': 0.0, 'z': 0.92},
        }

    def _verification_payload(self) -> dict:
        verify_count = self._count('D3_VERIFY_BOLT_INSERTED')
        visible_failure = (
            self.scenario == 'bolt_insert_visible_fail'
            and 0 < verify_count <= 1
        )
        invisible_failure = (
            self.scenario == 'bolt_insert_invisible_fail'
            and 0 < verify_count <= 1
        )
        unreachable_failure = (
            self.scenario == 'bolt_insert_unreachable_fail'
            and 0 < verify_count <= 1
        )
        bolt_confidence = self.low_confidence if (visible_failure or invisible_failure) else self.confidence
        bolt_center_error_m = 0.20 if visible_failure else 0.01
        bolt_visible = (
            not invisible_failure
            and not unreachable_failure
            and bolt_confidence >= self.bolt_visible_confidence_threshold
            and bolt_center_error_m <= self.bolt_center_tolerance_m
        )
        bolt_visible_any = not invisible_failure
        loose_region = 'unreachable' if unreachable_failure else self.loose_bolt_region
        loose_reachable = self.loose_bolt_reachable and loose_region != 'unreachable'
        return {
            'valid': True,
            'confidence': self.confidence,
            'frame_id': self.frame_id,
            'bolt_visible': bolt_visible,
            'bolt_visible_any': bolt_visible_any,
            'bolt_confidence': bolt_confidence,
            'bolt_center_error_m': bolt_center_error_m,
            'loose_bolt_region': loose_region if bolt_visible_any else 'unknown',
            'loose_bolt_reachable': loose_reachable if bolt_visible_any else False,
            'point': {'x': 0.48, 'y': -0.16, 'z': 0.79},
            'bolt_point': {'x': 0.48, 'y': -0.16, 'z': 0.79},
            'bolt': {
                'valid': bolt_visible_any,
                'confidence': bolt_confidence,
                'point': {'x': 0.48, 'y': -0.16, 'z': 0.79},
            },
            'loose_bolt_message': (
                'reachable loose bolt region'
                if loose_reachable
                else 'loose bolt is not reachable by current recovery motion'
            ),
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockPerceptionD()
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
