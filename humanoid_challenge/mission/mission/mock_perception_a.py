#!/usr/bin/env python3
"""Mock Perception (Mission A) — 비-sim 검증용 perception 입력 소스.

제공:
  - /perception/task_list (String JSON): 목표 부품 목록(설정형 parts).
  - /perception/wrist/target_one_pose (PoseStamped, base_link): A2_SCAN target.
  - /perception/place_pose_valid (String JSON): C3 트레이 place 위치 유효성.
    place_pose_invalid / place_pose_flap 주입으로 C3 게이트 검증.

기존 검증 파이프라인(task_list/detections/wrist target) 계약을 로봇 없이 모사.
실 perception 교체 시 본 노드만 종료.
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


# 기본 목표: 총 5개 (≥5 사이클 루프 검증용)
DEFAULT_PARTS = [
    {'name': 'flange nut', 'count': 2},
    {'name': 'hex nut', 'count': 2},
    {'name': 'gear ring', 'count': 1},
]


class MockPerceptionA(Node):
    def __init__(self) -> None:
        super().__init__('mock_perception_a')

        parts_json = str(self.declare_parameter('parts_json', '').value)
        self.place_invalid = bool(self.declare_parameter('place_pose_invalid', False).value)
        self.place_flap = bool(self.declare_parameter('place_pose_flap', False).value)
        self.target_x = float(self.declare_parameter('target_x', 0.5).value)
        self.target_y = float(self.declare_parameter('target_y', 0.0).value)
        self.target_z = float(self.declare_parameter('target_z', 0.3).value)

        if parts_json.strip():
            try:
                self._parts = json.loads(parts_json)
            except Exception:
                self.get_logger().warn('parts_json 파싱 실패 — 기본값 사용')
                self._parts = list(DEFAULT_PARTS)
        else:
            self._parts = list(DEFAULT_PARTS)

        cbg = ReentrantCallbackGroup()
        self.pub_task = self.create_publisher(String, '/perception/task_list', 10)
        self.pub_target = self.create_publisher(
            PoseStamped, '/perception/wrist/target_one_pose', 10)
        self.pub_place = self.create_publisher(String, '/perception/place_pose_valid', 10)

        self._flap = False
        self.create_timer(0.5, self._pub_task, callback_group=cbg)
        self.create_timer(0.5, self._pub_target, callback_group=cbg)
        self.create_timer(0.2, self._pub_place, callback_group=cbg)
        total = sum(int(p.get('count', 0)) for p in self._parts)
        self.get_logger().info(
            f'mock_perception_a ready (parts 총 {total}, '
            f'place_invalid={self.place_invalid}, place_flap={self.place_flap})')

    def _pub_task(self) -> None:
        self.pub_task.publish(String(data=json.dumps({'parts': self._parts})))

    def _pub_target(self) -> None:
        m = PoseStamped()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = self.target_x
        m.pose.position.y = self.target_y
        m.pose.position.z = self.target_z
        m.pose.orientation.w = 1.0
        self.pub_target.publish(m)

    def _pub_place(self) -> None:
        if self.place_flap:
            self._flap = not self._flap
            valid = self._flap
        else:
            valid = not self.place_invalid
        payload = {'valid': valid, 'dx': 0.0, 'dy': 0.0, 'confidence': 0.9}
        self.pub_place.publish(String(data=json.dumps(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockPerceptionA()
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
