#!/usr/bin/env python3
"""Mock Navigation (Mission A) — navigation 팀 미구현 기능 대체.

제공: **MoveBaseLateral Service** 서버 (Action 아님 — SDR v2.3 EDP 병목 제거).
  - 박스(우)↔트레이(좌) 측방 dead-reckon strafe 모사.
  - Request{direction:"left"|"right", distance_mm} → Response{arrived, lateral_error_mm, message}.
  - fail_arrive 주입 시 도착 실패(→ FSM RECOVERY).

675mm 실 정밀도는 로봇/캘리브레이션 영역(본 mock 은 arrived 만 보고).
실 navigation 교체 시 본 노드만 종료하고 동일 서비스 노드를 기동.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from mission_interfaces.srv import MoveBaseLateral


class MockNavigationA(Node):
    def __init__(self) -> None:
        super().__init__('mock_navigation_a')

        self.travel_sec = float(self.declare_parameter('travel_sec', 1.0).value)
        self.fail_arrive = bool(self.declare_parameter('fail_arrive', False).value)
        self.lateral_error_mm = float(
            self.declare_parameter('lateral_error_mm', 2.0).value)

        cbg = ReentrantCallbackGroup()
        self.srv = self.create_service(
            MoveBaseLateral, 'move_base_lateral', self._on_request, callback_group=cbg)
        self.get_logger().info(
            f'mock_navigation_a ready (Service; travel_sec={self.travel_sec}, '
            f'fail_arrive={self.fail_arrive})')

    def _on_request(self, request, response):
        # 이동 소요(모사). MTE 라 블록해도 다른 콜백 처리 가능.
        time.sleep(self.travel_sec)
        arrived = not self.fail_arrive
        response.arrived = arrived
        response.lateral_error_mm = self.lateral_error_mm if arrived else 0.0
        response.message = (
            f'arrived dir={request.direction} {request.distance_mm:.0f}mm'
            if arrived else 'navigate injected-failure')
        self.get_logger().info(
            f'[mock_nav] {request.direction} {request.distance_mm:.0f}mm '
            f'arrived={arrived} err={response.lateral_error_mm:.1f}mm')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockNavigationA()
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
