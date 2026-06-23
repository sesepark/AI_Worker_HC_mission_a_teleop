#!/usr/bin/env python3
"""Mission A --sim 드라이버 — 풀스택 없이 FSM 전이를 검증하기 위한 fake 토픽 주입기.

mission_a 노드가 **현재 어느 state 인지** 보고 그 state 를 다음으로 넘기는 가짜 메시지를 발행.
실제 perception / manipulation 스택 없이 INIT→A1→A2→A3→VERIFY→DONE 루프를 돌려본다.

`/mission_a/task_list` 서비스를 모사해 MissionA의 서비스 기반 경로를 검증한다.
"""
from __future__ import annotations

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mission_interfaces.msg import TaskItem
from mission_interfaces.srv import GetTaskList


# 시나리오 초기 목표 (canonical 표기). 총 3개.
SIM_INITIAL = {'flange nut': 1, 'hex nut': 2}
CANONICAL_PARTS = ['flange nut', 'gear ring', 'spacer ring', 'hex nut', 'dom nut']
CANON_TO_CLASS = {
    'flange nut': 'flange_nut', 'gear ring': 'gear_ring',
    'spacer ring': 'spacer_ring', 'hex nut': 'hex_nut', 'dom nut': 'dome_nut',
}
CANON_TO_KOR = {
    'flange nut': '플랜지 너트', 'gear ring': '기어 링',
    'spacer ring': '스페이서 링', 'hex nut': '육각 너트', 'dom nut': '돔 너트',
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
        self.srv_task_list = node.create_service(
            GetTaskList, node.task_list_service_name, self._handle_task_list)
        # SDR v2.3: 해제는 FSM 의 A3_PLACE 게이트 통과(/detach_cmd) 후에만 주입한다.
        #   (state==A3_PLACE 즉시 주입 시 게이트 발행 전 ''→false-RECOVERY 경쟁 발생)
        self.sub_detach = node.create_subscription(
            String, '/detach_cmd', self._on_detach_cmd, 10)

        self._remaining = dict(SIM_INITIAL)   # 내부 잔여 (canonical)
        self._picked: str | None = None       # 이번 사이클에 집은 canonical 부품
        self._release_done: bool = False       # 이번 파지의 해제 주입 완료 여부
        self._last_action: str | None = None
        self._timer = node.create_timer(period_sec, self._drive)
        node.get_logger().warn(
            f'[SIM] sim_driver 활성 — {node.task_list_service_name} 서비스 기반 FSM 구동')

    def _once(self, key: str) -> bool:
        if self._last_action == key:
            return False
        self._last_action = key
        return True

    def _handle_task_list(self, request, response):
        response.success = True
        response.message = 'sim task list ready'
        response.screen_detected = True
        response.all_counts_recognized = True
        response.frames_used = int(request.frame_count) if request.frame_count else 1
        response.parts = [
            TaskItem(name=CANON_TO_KOR[n], count=int(self._remaining.get(n, 0)))
            for n in CANONICAL_PARTS
        ]
        return response

    def _drive(self) -> None:
        S = self._State
        n = self._node
        st = n.state

        if st == S.INIT:
            if n.last_manipulator_state != 'IDLE' and self._once('init'):
                self.pub_manip.publish(String(data='IDLE'))
                n.get_logger().info('[SIM] /manipulator_state=IDLE 주입')

        elif st == S.A1_MONITOR:
            if self._once('a1'):
                total = sum(self._remaining.values())
                n.get_logger().info(f'[SIM] task_list service 대기 중 (총 {total})')

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
                self._release_done = False
                cls = CANON_TO_CLASS[self._picked]
                self.pub_attached.publish(String(data=cls))
                n.get_logger().info(f'[SIM] /attached_object={cls} 주입 (파지)')

        # 해제(/attached_object="")는 A3_PLACE state 가 아니라 /detach_cmd 수신 시 주입.
        #   → _on_detach_cmd 참조.

    def _on_detach_cmd(self, msg) -> None:
        """FSM A3_PLACE 게이트 통과(/detach_cmd) 후에만 해제 주입."""
        if self._picked and not self._release_done:
            self.pub_attached.publish(String(data=''))
            if self._remaining.get(self._picked, 0) > 0:
                self._remaining[self._picked] -= 1
            self._release_done = True
            total = sum(self._remaining.values())
            self._node.get_logger().info(
                f'[SIM] /detach_cmd 수신 → /attached_object="" 주입 (잔여 {total})')
