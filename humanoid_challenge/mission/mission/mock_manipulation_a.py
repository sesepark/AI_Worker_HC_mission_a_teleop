#!/usr/bin/env python3
"""Mock Manipulation (Mission A) — manipulation 팀 미구현 기능 대체.

제공:
  - MoveToScanPose **action** 서버 (스캔 초기 포즈 형성; 항상 success).
  - /manipulator_state = IDLE 주기 발행 (INIT 통과용).
  - /attach_cmd 수신 → /attached_object = <pick class> (파지). class 는 task_list
    미러에서 next-available 선택(FSM 차감과 동기).
  - /detach_cmd 수신 → /attached_object = "" (해제) + 미러 차감.
  - 드롭 주입(drop_during_move): 파지 후 drop_after_attach_sec 뒤 ""(드롭) 발행 →
    FSM A3_MOVE_TO_TRAY 의 C2 모니터 검증용.

carry 자세는 manipulation 내부 로직 — 본 mock 은 carry 명령(SetCarryPose) 없음.
실 manipulation 교체 시 본 노드만 종료하고 동일 인터페이스 노드를 기동.
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from mission.task_list import TaskList
from mission_interfaces.action import MoveToScanPose


class MockManipulationA(Node):
    def __init__(self) -> None:
        super().__init__('mock_manipulation_a')

        self.scan_delay = float(self.declare_parameter('scan_delay_sec', 0.5).value)
        self.drop_during_move = bool(self.declare_parameter('drop_during_move', False).value)
        self.drop_after_attach_sec = float(
            self.declare_parameter('drop_after_attach_sec', 0.5).value)

        cbg = ReentrantCallbackGroup()
        self.srv_scan = ActionServer(
            self, MoveToScanPose, 'move_to_scan_pose',
            self._exec_scan, callback_group=cbg)

        self.pub_attached = self.create_publisher(String, '/attached_object', 10)
        self.pub_manip = self.create_publisher(String, '/manipulator_state', 10)
        self.sub_attach = self.create_subscription(
            String, '/attach_cmd', self._on_attach, 10, callback_group=cbg)
        self.sub_detach = self.create_subscription(
            String, '/detach_cmd', self._on_detach, 10, callback_group=cbg)
        self.sub_task = self.create_subscription(
            String, '/perception/task_list', self._on_task, 10, callback_group=cbg)

        self._mirror = TaskList()
        self._current: str | None = None
        self._drop_due: float | None = None
        self._pending_attach = False   # task 미러 준비 전 도착한 attach 보류

        self.create_timer(0.2, self._pub_manip, callback_group=cbg)
        self.create_timer(0.1, self._tick_drop, callback_group=cbg)
        self.get_logger().info(
            f'mock_manipulation_a ready (drop_during_move={self.drop_during_move})')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _pub_manip(self) -> None:
        self.pub_manip.publish(String(data='IDLE'))

    def _on_task(self, msg: String) -> None:
        if not self._mirror.is_empty():
            return
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        parts = [
            {'name': i.get('name', ''), 'count': i.get('count', 0)}
            for i in data.get('parts', []) if isinstance(i, dict)
        ]
        self._mirror.build_from_ocr_parts(parts)
        if not self._mirror.is_empty():
            self.get_logger().info(f'[mock_manip] task 미러 빌드: {self._mirror}')

    def _exec_scan(self, goal_handle):
        time.sleep(self.scan_delay)
        goal_handle.succeed()
        result = MoveToScanPose.Result()
        result.success = True
        result.message = 'scan pose formed (mock)'
        self.get_logger().info('[mock_manip] MoveToScanPose -> success')
        return result

    def _on_attach(self, msg: String) -> None:
        cls = self._mirror.next_target_class()
        if not cls:
            # task_list 아직 미수신 → 버리지 말고 보류, 타이머가 미러 준비 후 처리
            self._pending_attach = True
            self.get_logger().warn('[mock_manip] /attach_cmd 수신 — task 미러 미준비, 보류')
            return
        self._do_attach(cls)

    def _do_attach(self, cls: str) -> None:
        self._current = cls
        self.pub_attached.publish(String(data=cls))
        self.get_logger().info(f'[mock_manip] 파지 → /attached_object={cls}')
        if self.drop_during_move:
            self._drop_due = self._now() + self.drop_after_attach_sec

    """
    def _on_attach(self, msg: String) -> None:
        cls = self._mirror.next_target_class()
        if not cls:
            self.get_logger().warn('[mock_manip] /attach_cmd 수신했으나 잔여 class 없음')
            return
        self._current = cls
        self.pub_attached.publish(String(data=cls))
        self.get_logger().info(f'[mock_manip] 파지 → /attached_object={cls}')
        if self.drop_during_move:
            self._drop_due = self._now() + self.drop_after_attach_sec
    """

    def _tick_drop(self) -> None:
        # 보류된 attach 재시도: 미러가 늦게 준비된 경우 여기서 처리(최대 0.1s 지연)
        if self._pending_attach:
            cls = self._mirror.next_target_class()
            if cls:
                self._pending_attach = False
                self.get_logger().info('[mock_manip] 보류 /attach_cmd 처리')
                self._do_attach(cls)
        if self._drop_due is not None and self._now() >= self._drop_due:
            self._drop_due = None
            self.pub_attached.publish(String(data=''))   # 드롭 (적재 아님 → 미러 차감 없음)
            self.get_logger().warn('[mock_manip] 드롭 주입 → /attached_object="" (무차감)')

    """
    def _tick_drop(self) -> None:
        if self._drop_due is not None and self._now() >= self._drop_due:
            self._drop_due = None
            self.pub_attached.publish(String(data=''))   # 드롭 (적재 아님 → 미러 차감 없음)
            self.get_logger().warn('[mock_manip] 드롭 주입 → /attached_object="" (무차감)')
    """

    def _on_detach(self, msg: String) -> None:
        if self._current and self._drop_due is None:
            self.pub_attached.publish(String(data=''))
            self._mirror.decrement(self._current)
            self.get_logger().info(
                f'[mock_manip] /detach_cmd → 해제 ({self._current}), '
                f'미러 잔여 {self._mirror.total_remaining()}')
            self._current = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockManipulationA()
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
