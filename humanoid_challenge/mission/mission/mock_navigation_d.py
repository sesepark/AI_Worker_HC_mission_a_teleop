#!/usr/bin/env python3
"""Mission D team-owned 설정 기반 mock navigation 노드.

실제 navigation planning이나 odometry/lidar 처리는 하지 않는다. System FSM은 semantic path_id와
timeout만 의미 있게 요청하고, 이 mock은 navigation 팀 설정처럼 path_id별 상대 이동량과 허용 오차를
자체 config에서 읽어 도착/범위 판정을 만든다.
"""
from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mission_interfaces.srv import MoveBaseRelative


DEFAULT_PATH_TABLE = {
    'd1_wheel_align_left': {'dx_mm': 0.0, 'dy_mm': 180.0, 'dyaw_deg': 0.0},
    'wheel_align_to_tool_area': {'dx_mm': 0.0, 'dy_mm': 520.0, 'dyaw_deg': 0.0},
    'start_to_tool_area': {'dx_mm': 0.0, 'dy_mm': 700.0, 'dyaw_deg': 0.0},
    'tool_to_fixture': {'dx_mm': 0.0, 'dy_mm': -700.0, 'dyaw_deg': 0.0},
    'fixture_to_tool_area': {'dx_mm': 0.0, 'dy_mm': 700.0, 'dyaw_deg': 0.0},
    'd1_move_to_wheel_drop_space': {'dx_mm': 0.0, 'dy_mm': 0.0, 'dyaw_deg': -90.0},
    'd1_return_from_wheel_drop_space': {'dx_mm': 0.0, 'dy_mm': 0.0, 'dyaw_deg': 90.0},
    'd3_move_near_loose_bolt_front': {'dx_mm': 120.0, 'dy_mm': 0.0, 'dyaw_deg': 0.0},
    'd3_move_near_loose_bolt_left': {'dx_mm': 90.0, 'dy_mm': 120.0, 'dyaw_deg': 0.0},
    'd3_move_near_loose_bolt_right': {'dx_mm': 90.0, 'dy_mm': -120.0, 'dyaw_deg': 0.0},
    'd3_move_to_bolt_drop_space': {'dx_mm': -250.0, 'dy_mm': 220.0, 'dyaw_deg': 0.0},
    'return_to_start_pose': {'dx_mm': -120.0, 'dy_mm': 0.0, 'dyaw_deg': 0.0},
}


class MockNavigationD(Node):
    """MoveBaseRelative service를 제공하고 path/call 기반 실패를 주입하는 mock Node."""

    def __init__(self) -> None:
        super().__init__('mock_navigation_d')

        self.nav_service_name = str(
            self.declare_parameter('nav_service_name', 'move_base_relative').value)
        self.scenario = str(self.declare_parameter('scenario', 'normal').value)
        self.travel_sec = float(self.declare_parameter('travel_sec', 1.0).value)
        self.path_table = self._json_param('path_table_json', DEFAULT_PATH_TABLE)
        self.translation_tolerance_mm = float(
            self.declare_parameter('translation_tolerance_mm', 20.0).value)
        self.yaw_tolerance_deg = float(
            self.declare_parameter('yaw_tolerance_deg', 5.0).value)
        self.fail_arrive = bool(self.declare_parameter('fail_arrive', False).value)
        self.fail_within_expected_range = bool(
            self.declare_parameter('fail_within_expected_range', False).value)
        self.fail_on_call_indices = set(
            int(x) for x in self._json_param('fail_on_call_indices_json', []))
        self.fail_on_path_ids = set(
            str(x) for x in self._json_param('fail_on_path_ids_json', []))
        self.actual_translation_scale = float(
            self.declare_parameter('actual_translation_scale', 1.0).value)
        self.actual_yaw_scale = float(
            self.declare_parameter('actual_yaw_scale', 1.0).value)

        self._call_count = 0
        cbg = ReentrantCallbackGroup()
        self.create_service(
            MoveBaseRelative, self.nav_service_name, self._on_request, callback_group=cbg)
        self.get_logger().info(
            f'mock_navigation_d ready service={self.nav_service_name} scenario={self.scenario}')

    def _json_param(self, name: str, default):
        value = self.declare_parameter(name, json.dumps(default)).value
        if isinstance(value, str):
            try:
                return json.loads(value)
            except Exception:
                self.get_logger().warn(f'{name} 파싱 실패, 기본값 사용')
                return default
        return value

    def _configured_motion(self, request) -> tuple[float, float, float]:
        # 기본 Mission D 계약에서는 path_id가 주 명령이고, dx/dy/dyaw는 service 호환 fallback이다.
        configured = self.path_table.get(request.path_id, {})
        dx = float(configured.get('dx_mm', request.dx_mm) or 0.0)
        dy = float(configured.get('dy_mm', request.dy_mm) or 0.0)
        dyaw = float(configured.get('dyaw_deg', request.dyaw_deg) or 0.0)
        return dx, dy, dyaw

    def _on_request(self, request, response):
        # travel_sec는 service timeout/retry 흐름을 실제 통합처럼 보이게 하기 위한 mock 지연이다.
        time.sleep(max(self.travel_sec, 0.0))
        self._call_count += 1

        fallback_is_empty = (
            abs(float(request.dx_mm or 0.0)) < 1e-9
            and abs(float(request.dy_mm or 0.0)) < 1e-9
            and abs(float(request.dyaw_deg or 0.0)) < 1e-9
        )
        if request.path_id and request.path_id not in self.path_table and fallback_is_empty:
            response.arrived = False
            response.within_expected_range = False
            response.actual_translation_mm = 0.0
            response.actual_yaw_deg = 0.0
            response.message = f'unknown path_id={request.path_id}'
            self.get_logger().warn(f'[mock_nav_d] {response.message}')
            return response

        dx_mm, dy_mm, dyaw_deg = self._configured_motion(request)
        requested_translation = math.sqrt(dx_mm * dx_mm + dy_mm * dy_mm)
        actual_translation = requested_translation * self.actual_translation_scale
        requested_yaw = abs(dyaw_deg)
        actual_yaw = requested_yaw * self.actual_yaw_scale
        in_translation_range = (
            abs(actual_translation - requested_translation) <= self.translation_tolerance_mm)
        in_yaw_range = abs(actual_yaw - requested_yaw) <= self.yaw_tolerance_deg

        injected_failure = (
            self.fail_arrive
            or self._call_count in self.fail_on_call_indices
            or request.path_id in self.fail_on_path_ids
        )
        response.arrived = not injected_failure
        response.within_expected_range = bool(
            response.arrived
            and not self.fail_within_expected_range
            and in_translation_range
            and in_yaw_range
        )
        response.actual_translation_mm = actual_translation if response.arrived else 0.0
        response.actual_yaw_deg = actual_yaw if response.arrived else 0.0
        response.message = (
            f'arrived call={self._call_count} path={request.path_id} '
            f'dx={dx_mm:.1f} dy={dy_mm:.1f} dyaw={dyaw_deg:.1f}'
            if response.arrived
            else f'injected nav failure call={self._call_count} path={request.path_id}'
        )
        self.get_logger().info(
            f'[mock_nav_d] call={self._call_count} path={request.path_id} '
            f'arrived={response.arrived} within={response.within_expected_range}')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockNavigationD()
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
