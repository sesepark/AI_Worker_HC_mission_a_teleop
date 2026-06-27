#!/usr/bin/env python3
"""Mission B navigation 코디네이터 경량 mock (무로봇 드라이런용).

실제 `ffw_mission_b_nav` 는 /odom·/scan·/cmd_vel 이 필요해 로봇 없이는 못 돈다.
이 mock 은 String 인터페이스만 동일하게 흉내내어 mission_b FSM 로직·신호 흐름을
로봇 없이 검증하게 한다(실제 이동은 하지 않음).

계약(실 nav 와 동일)
  수신 /mission_b/system/action : A_TO_B / APPROACH_B / B_TO_A / STOP
  송신 /mission_b/nav/event(latched) : READY / *_ACCEPTED / REACHED_* / STOPPED / REJECTED:*

각 action 은 leg_sec 후 해당 도착 event 를 발행한다.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String


# action → (accept event, 도착 event, 진행을 허용하는 직전 상태들)
FLOW = {
    'A_TO_B':     ('A_TO_B_ACCEPTED', 'REACHED_B_STOP_LINE', ('IDLE', 'AT_A')),
    'APPROACH_B': ('APPROACH_B_ACCEPTED', 'REACHED_B_PLACE_POSE', ('AT_B_STOP_LINE',)),
    'B_TO_A':     ('B_TO_A_ACCEPTED', 'REACHED_A', ('AT_B_PLACE_POSE',)),
}
# 도착 event → 다음 상태
NEXT_STATE = {
    'REACHED_B_STOP_LINE': 'AT_B_STOP_LINE',
    'REACHED_B_PLACE_POSE': 'AT_B_PLACE_POSE',
    'REACHED_A': 'AT_A',
}


class MockNavB(Node):
    def __init__(self) -> None:
        super().__init__('mock_nav_b')
        self.leg_sec = float(self.declare_parameter('leg_sec', 2.0).value)

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_event = self.create_publisher(String, '/mission_b/nav/event', latched)
        self.create_subscription(
            String, '/mission_b/system/action', self._on_action, 10)

        self.state = 'IDLE'
        self._pending = None          # (도착 event, 발행 시각)
        self.create_timer(0.1, self._tick)
        self._emit('READY')
        self.get_logger().info('mock_nav_b ready (드라이런용)')

    def _emit(self, event: str) -> None:
        self.pub_event.publish(String(data=event))
        self.get_logger().info(f'[event] {event}')

    def _on_action(self, msg: String) -> None:
        action = msg.data.strip().upper()
        if action == 'STOP':
            self._pending = None
            self.state = 'STOPPED'
            self._emit('STOPPED')
            return
        flow = FLOW.get(action)
        if flow is None:
            self._emit(f'REJECTED:{action}:unknown_action')
            return
        accept_ev, arrive_ev, allowed = flow
        if self.state not in allowed and not (action == 'A_TO_B' and self.state == 'STOPPED'):
            self._emit(f'REJECTED:{action}:bad_state={self.state}')
            return
        self._emit(accept_ev)
        self._pending = (arrive_ev, self.get_clock().now().nanoseconds * 1e-9)

    def _tick(self) -> None:
        if self._pending is None:
            return
        arrive_ev, t0 = self._pending
        if (self.get_clock().now().nanoseconds * 1e-9 - t0) >= self.leg_sec:
            self.state = NEXT_STATE[arrive_ev]
            self._pending = None
            self._emit(arrive_ev)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockNavB()
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
