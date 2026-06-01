#!/usr/bin/env python3
"""Mission A --sim 드라이버 — 풀스택 없이 FSM 전이를 검증하기 위한 fake 토픽 주입기.

mission_a 노드가 **현재 어느 state 인지** 보고 그 state 를 다음으로 넘기는 가짜 메시지를 발행.
실제 perception / manipulation 스택 없이 INIT→A1→A2→A3→VERIFY→DONE 루프를 돌려본다.

신규 `task_management` 파이프라인을 모사한다:
- A1 에서 `/perception/task_list`(잔여) 발행 → mission_a 가 perception-owned 경로 사용
- place 마다 내부 잔여를 1 차감 후 `/perception/task_list` 재발행 (트레이 비전 차감 모사)
"""
from __future__ import annotations

import json

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


# 시나리오 초기 목표 (canonical 표기, management_node 와 동일). 총 3개.
SIM_INITIAL = {'flange nut': 1, 'hex nut': 2}
CANONICAL_PARTS = ['flange nut', 'gear ring', 'spacer ring', 'hex nut', 'dom nut']
CANON_TO_CLASS = {
    'flange nut': 'flange_nut', 'gear ring': 'gear_ring',
    'spacer ring': 'spacer_ring', 'hex nut': 'hex_nut', 'dom nut': 'dome_nut',
}


class SimDriver:
    """state 기반으로 입력 토픽 fake 발행. node.state(State enum) 를 읽는다."""

    def __init__(self, node, State, period_sec: float = 0.8) -> None:
        self._node = node
        self._State = State

        self.pub_manip = node.create_publisher(String, '/manipulator_state', 10)
        self.pub_target = node.create_publisher(
            PoseStamped, '/perception/wrist/target_one_pose', 10)
        self.pub_attached = node.create_publisher(String, '/attached_object', 10)
        self.pub_task_list = node.create_publisher(String, '/perception/task_list', 10)

        self._remaining = dict(SIM_INITIAL)   # 내부 잔여 (canonical)
        self._picked: str | None = None       # 이번 사이클에 집은 canonical 부품
        self._last_action: str | None = None
        self._timer = node.create_timer(period_sec, self._drive)
        node.get_logger().warn(
            '[SIM] sim_driver 활성 — /perception/task_list 기반 FSM 구동')

    def _once(self, key: str) -> bool:
        if self._last_action == key:
            return False
        self._last_action = key
        return True

    def _publish_task_list(self) -> None:
        parts = [{'name': n, 'count': int(self._remaining.get(n, 0))}
                 for n in CANONICAL_PARTS]
        payload = {'parts': parts, 'ocr_latest_screen_detected': True,
                   'tray_stable_frames': 3}
        self.pub_task_list.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _drive(self) -> None:
        S = self._State
        n = self._node
        st = n.state

        if st == S.INIT:
            if n.last_manipulator_state != 'IDLE' and self._once('init'):
                self.pub_manip.publish(String(data='IDLE'))
                n.get_logger().info('[SIM] /manipulator_state=IDLE 주입')

        elif st == S.A1_MONITOR:
            if not n._perception_owns_tasklist and self._once('a1'):
                self._publish_task_list()
                total = sum(self._remaining.values())
                n.get_logger().info(f'[SIM] /perception/task_list 주입 (총 {total})')

        elif st == S.A2_SCAN:
            if n.last_target_pose is None and self._once(f'a2:{n.cycle}'):
                msg = PoseStamped()
                msg.header.frame_id = 'base_link'
                msg.header.stamp = n.get_clock().now().to_msg()
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = 0.5, 0.0, 0.3
                msg.pose.orientation.w = 1.0
                self.pub_target.publish(msg)
                n.get_logger().info('[SIM] /perception/wrist/target_one_pose 주입')

        elif st == S.A3_PICK:
            if not n.last_attached_object and self._once(f'pick:{n.cycle}'):
                # 잔여가 있는 canonical 부품 1개 선택
                self._picked = next(
                    (c for c in CANONICAL_PARTS if self._remaining.get(c, 0) > 0), 'hex nut')
                cls = CANON_TO_CLASS[self._picked]
                self.pub_attached.publish(String(data=cls))
                n.get_logger().info(f'[SIM] /attached_object={cls} 주입 (파지)')

        elif st == S.A3_PLACE:
            if n.last_attached_object and self._once(f'place:{n.cycle}'):
                self.pub_attached.publish(String(data=''))
                # 트레이 비전 차감 모사: 잔여 1 감소 후 task_list 재발행
                if self._picked and self._remaining.get(self._picked, 0) > 0:
                    self._remaining[self._picked] -= 1
                self._publish_task_list()
                total = sum(self._remaining.values())
                n.get_logger().info(
                    f'[SIM] /attached_object="" + task_list 재발행 (잔여 {total})')
